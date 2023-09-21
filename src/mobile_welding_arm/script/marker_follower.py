#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
  Chinese National Engineering Research Centre for Steel Structure
                        (Hong Kong Branch)

  Hong Kong Polytechnic University

  Author: Victor W H Wu
  Date: 10 August 2023.

  File name: marker_follower.py

  Description:
    This python script subscribes to the  pose of the Charuco board centre
    as published by marker_pose. This pose will be used to control the UGV.

'''

import rospy
import tf2_ros
import tf2_geometry_msgs
import math
import tf.transformations
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped
from aruco_msgs.msg import MarkerArray
import time

import numpy as np

from geometry_msgs.msg import Pose

from my_utilities import invert_transform

from scipy.spatial.transform import Rotation as R

import welding_msgs.srv 
LINEAR_THRESHOLD = 0.025 # Acceptable position error in meters (1cm)
ANGULAR_THRESHOLD = 0.025 # Acceptable orientation error in radians (2.8 degree)

'''
# Initialize the node and publishers/subscribers
rospy.init_node('aruco_follower', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

desired_distance = 1.3 # the desired distance between UR10 base and marker
'''

class PID:
    # A Simple PID class 
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class MarkerFollower:

  def __init__(self, distance):

    # PID Controller for controlling the UGV
    self.linear_pid = PID(kp=0.2, ki=0.0, kd=0.00)
    self.angular_pid = PID(kp=0.5, ki=0.0, kd=0.00)

    # Create a Buffer and a TransformListener for tf2 lookup
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    # Subscribe the Marker's pose, in the camera frame
    self.marker_sub = rospy.Subscriber('/marker_pose_stamped', PoseStamped, self.marker_callback)

    # Publisher for the pose of the target and current Robot base
    self.target_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=1)
    self.current_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)

    # Publisher for the velocity command to move the UGV
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Distance from marker to target
    self.distance = distance

    # Names of the source and target frames
    self.camera_optical_frame = 'd435_color_optical_frame'
    self.camera_frame = 'd435_color_frame' # changed by V Wu on 19 Sep 2023.
    self.odom_frame = 'odom'
    self.robot_base = 'base'
    self.ugv_frame = 'bunker_pro_base_link'

    # Setup an empty marker pose in 'odom' frame
    self.marker_pose = None

    # Setup the current pose in 'odom' frame
    self.current_pose = None

    # Setup the target pose in 'odom' frame
    self.target_pose = None

    # Initialize Emergency Stop status
    self.estop_triggered = False

    # attributes for control_ugv
    self.target_position_reached = False
    self.heading_yaw_reached = False
    self.done = False

    self.target_angular = 0 # orientation of the target pose
    self.target_yaw = 0     # orientation to face target position

    # Subscribe to Emergency Stop
    rospy.Subscriber('/estop', Bool, self.estop_callback)

  def emergency_stop_publisher(self):

    # Create a Twist message with zero velocities
    stop_msg = Twist()
    stop_msg.linear.x = 0.0
    stop_msg.linear.y = 0.0
    stop_msg.linear.z = 0.0
    stop_msg.angular.x = 0.0
    stop_msg.angular.y = 0.0
    stop_msg.angular.z = 0.0

    # Update status
    self.estop_triggered = True

    # Publish the stop message
    rospy.loginfo("Emergency Stop Triggered! Publishing zero velocities.")
    while self.estop_triggered:
      self.cmd_vel_pub.publish(stop_msg)
  
  def estop_callback(self, data):
    if data.data:
      # E-stop triggered. Take necessary actions;
      rospy.loginfo("Emergency Stop Triggered!")
      # Add code to stop UGV
      self.emergency_stop_publisher()
    else:
      # Normal operation or E-stop reset.
      self.estop_triggered = False
      rospy.loginfo("Normal Operation")

  def get_current_pose(self):
    # Lookup the current pose of the camera color frame
    try:
      transform = self.tf_buffer.lookup_transform(self.odom_frame, self.camera_frame, rospy.Time(0))
      current_pose = PoseStamped()
      current_pose.pose.position.x = transform.transform.translation.x
      current_pose.pose.position.y = transform.transform.translation.y
      current_pose.pose.position.z = transform.transform.translation.z
      current_pose.pose.orientation.x = transform.transform.rotation.x
      current_pose.pose.orientation.y = transform.transform.rotation.y
      current_pose.pose.orientation.z = transform.transform.rotation.z
      current_pose.pose.orientation.w = transform.transform.rotation.w
      current_pose.header = transform.header
      return current_pose
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      rospy.logerror("Failed to get current camera pose from tf")
      return None

  def get_heading_to_target(self):
    dx = self.target_pose.pose.position.x - self.current_pose.pose.position.x
    dy = self.target_pose.pose.position.y - self.current_pose.pose.position.y
    return math.atan2(dy, dx)

  def angular_error(self):

    def angle_difference(angle1, angle2):
      return math.atan2(math.sin(angle1 - angle2), math.cos(angle1 - angle2))

    # Convert target quaternion to yaw using tf.transformations
    target_quaternion = (
      self.target_pose.pose.orientation.x,
      self.target_pose.pose.orientation.y,
      self.target_pose.pose.orientation.z,
      self.target_pose.pose.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(target_quaternion)
    target_yaw = euler[2]

    # Convert current quaternion to yaw using tf.transformations
    current_quaternion = (
      self.current_pose.pose.orientation.x,
      self.current_pose.pose.orientation.y,
      self.current_pose.pose.orientation.z,
      self.current_pose.pose.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(current_quaternion)
    current_yaw = euler[2]

    # Compute the Heading Error
    angular_error = angle_difference(target_yaw, current_yaw)

    return angular_error

  def heading_error(self):

    def angle_difference(angle1, angle2):
      return math.atan2(math.sin(angle1 - angle2), math.cos(angle1 - angle2))

    # Determine the Target Yaw (or Heading)
    target_yaw = self.get_heading_to_target()

    # Convert quaternion to yaw using tf.transformations
    quaternion = (
      self.current_pose.pose.orientation.x,
      self.current_pose.pose.orientation.y,
      self.current_pose.pose.orientation.z,
      self.current_pose.pose.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    current_yaw = euler[2]

    # Compute the Heading Error
    heading_error = angle_difference(target_yaw, current_yaw)

    return heading_error

  def linear_error(self):
    dx = self.target_pose.pose.position.x - self.current_pose.pose.position.x
    dy = self.target_pose.pose.position.y - self.current_pose.pose.position.y
    return math.sqrt(dx**2 + dy**2)

  def control_ugv(self):

    # print('In control ugv.')
    self.current_pose = self.get_current_pose()
    # print('Current pose: ', self.current_pose)
    if not self.current_pose:
      return
    
    if self.done:
      return

    linear_correction = 0
    angular_correction = 0

    linear_error = self.linear_error()
    heading_error = self.heading_error()
    # angular_error = self.angular_error()

    # Target position reached
    print('Target position reached: ', self.target_position_reached)
    if self.target_position_reached:
      angular_error = self.angular_error()
      # Orientation is correct
      print('Angular error: ', angular_error)
      if abs(angular_error) < ANGULAR_THRESHOLD:
        # Target Pose reached
        self.done = True
      else:
        # Keep turning
        angular_correction = self.angular_pid.compute(angular_error)
    else:
      # Target position not reached yet
      print('Heading yaw reached: ', self.heading_yaw_reached)
      if self.heading_yaw_reached:
        # Keep going forward
        linear_error = self.linear_error()
        print('Linear error: ', linear_error)
        if abs(linear_error) < LINEAR_THRESHOLD:
          self.target_position_reached = True
        else:
          linear_correction = self.linear_pid.compute(linear_error)
      else:
        # Not in the right direction towards target position yet
        # Keep turning
        heading_error = self.heading_error()
        print('Heading error: ', heading_error)
        if abs(heading_error) >= ANGULAR_THRESHOLD:
          angular_correction = self.angular_pid.compute(heading_error)
        else:
          print('Heading error: ', heading_error)
          self.heading_yaw_reached = True
          # Already in the right direction, keep going forward
          linear_error = self.linear_error()
          print('Linear error: ', linear_error)
          linear_correction = self.linear_pid.compute(linear_error)

    print('Linear correction: ', linear_correction)
    print('Angular correction: ', angular_correction)
    print('=================')

    if not self.estop_triggered and not self.done:
      twist = Twist()
      twist.angular.z = angular_correction
      twist.linear.x = linear_correction
      self.cmd_vel_pub.publish(twist)
    return

  def calculate_target_pose(self, marker_pose):
    # Calculate the Dynamic Target Pose, in ODOM frame
    # It is the difference between the Static Marker Pose - the Dynamic Marker Pose
    # The self.marker_pose is the Static Marker Pose, the marker_pose is the Dynamic Marker Pose
    # Note that the Static Marker Pose is in ODOM frame, whereas the Dynamic Marker Pose is in Camera frame.
    
    # Project the 3D position of the marker-pose onto the X-Y plane
    target_pose = PoseStamped()

    # Marker position
    marker_position = [marker_pose.pose.position.x, marker_pose.pose.position.y, marker_pose.pose.position.z]

    # Convert marker's orientation to rotation matrix
    marker_quaternion = (marker_pose.pose.orientation.x, marker_pose.pose.orientation.y, 
                         marker_pose.pose.orientation.z, marker_pose.pose.orientation.w)
    r = R.from_quat(marker_quaternion)
    marker_matrix = r.as_matrix()
    
    # Rotate the marker orientation so that 
    # its Z-axis becomes the negative X-axis, and its Y-axis becomes the Z-axis, and
    # its X-axis becomes the negative Y-axis
    marker_matrix = np.matmul(marker_matrix, ([0, -1, 0], [0, 0, 1], [-1, 0, 0]))
    
    # Convert the marker matrix back to quaternion for orientation
    r = R.from_matrix(marker_matrix)
    marker_quaternion = r.as_quat()

    # Subtract the Distance from its new X-axis as the new position
    # this is where the target should be
    # where marker_matrix[:, 0] is its X-axis
    target_position = marker_position - marker_matrix[:, 0] * self.distance
    target_pose.pose.position.x = target_position[0]
    target_pose.pose.position.y = target_position[1]
    target_pose.pose.position.z = target_position[2]
    # Set the target orientation
    target_pose.pose.orientation.x = marker_quaternion[0]
    target_pose.pose.orientation.y = marker_quaternion[1]
    target_pose.pose.orientation.z = marker_quaternion[2]
    target_pose.pose.orientation.w = marker_quaternion[3]

    # Complete the PoseStamped format (header)
    target_pose.header.frame_id = self.odom_frame
    target_pose.header.stamp = rospy.Time.now()

    return target_pose

  def marker_callback(self, marker_pose):
    # This pose is always the Marker pose with reference to the Odom frame 
    # changed from camera frame to Odom frame by Victor Wu on 20 Sep 2023.
    # This function will be called every time a new Pose message is received

    # print('Inside callback.')
    # Calculate the marker pose with respect to odom if not done yet
    # print('marker pose: ', self.marker_pose)
    if self.marker_pose is None:
      # The first Marker Pose received before the UGV is moved.
      self.marker_pose = marker_pose
      # From now on, the Static Marker Pose is with reference to ODOM and that is FIXED
      # print('marker pose: ', self.marker_pose)
      print('*********************** marker pose has been setup. *************************************')

    self.target_pose = self.calculate_target_pose(marker_pose)

    # Publish the Target Pose in RViz
    self.target_pub.publish(self.target_pose)

    # Calculate the CURRENT_pose for "base" of ROBOT
    transform = self.tf_buffer.lookup_transform(self.odom_frame, self.camera_frame, rospy.Time(0))
    self.current_pose = PoseStamped()
    self.current_pose.pose.position.x = transform.transform.translation.x
    self.current_pose.pose.position.y = transform.transform.translation.y
    self.current_pose.pose.position.z = transform.transform.translation.z
    self.current_pose.pose.orientation.x = transform.transform.rotation.x
    self.current_pose.pose.orientation.y = transform.transform.rotation.y
    self.current_pose.pose.orientation.z = transform.transform.rotation.z
    self.current_pose.pose.orientation.w = transform.transform.rotation.w
    self.current_pose.header = transform.header

    # Unregister the marker_pose subscriber
    self.marker_sub.unregister()
    # Hopefully, this fix the marker and the target pose.

  def run(self):
    rate = rospy.Rate(10) # 10 Hz

    # Loop until Camera is up and running
    while not rospy.is_shutdown():
      if not self.marker_pose is None: # camera is up
        # Wait for the user reaction service
        # print('****************** waiting for service *****************************')
        rospy.wait_for_service('user_reaction')
        # Create a proxy for the service
        self.user_reaction_service = rospy.ServiceProxy('user_reaction', welding_msgs.srv.InteractService1)

        # Define the message to display on the panel
        display_message = "Do you want to move the UGV?"

        # Make the service call, and it will block here until a response is received.
        print('********************** Going to request a service ***********************')
        response = self.user_reaction_service(display_message)
        print('******************* A service has been requested. **********************')

        # Check the response
        if response.approved:
          print('User wants to Move the UGV.')
          # self.move_to_target()
          print('Done: ', self.done)

          while not rospy.is_shutdown() and not self.done:
            self.control_ugv()
            rate.sleep()
        else:
          print('User does not want to Move the UGV.')
          print('Marker pose: ', self.marker_pose)
          print('Target pose: ', self.target_pose)
          print('Current pose: ', self.current_pose)

        # The service cannot be shutdown from the Client side!

        # exit the loop
        break

      rate.sleep()


