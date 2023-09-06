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
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from aruco_msgs.msg import MarkerArray
from simple_pid import PID
import time

import numpy as np

from geometry_msgs.msg import Pose

from my_utilities import invert_transform

from scipy.spatial.transform import Rotation as R

import welding_msgs.srv 
from welding_msgs.srv import InteractService1

'''
# Initialize the node and publishers/subscribers
rospy.init_node('aruco_follower', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

desired_distance = 1.3 # the desired distance between UR10 base and marker
'''

class MarkerFollower:

  def __init__(self, distance):

    # PID Controller for controlling the UGV
    self.pid_linear = PID(Kp=1.0, Ki=0.1, Kd=0.01)
    self.pid_angular = PID(Kp=1.0, Ki=0.1, Kd=0.01)

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
    self.camera_frame = 'd435_color_optical_frame'
    self.odom_frame = 'odom'
    self.robot_base = 'base'
    self.ugv_frame = 'bunker_pro_base_link'

    # Setup an empty target pose
    self.marker_pose = None

    # Initialize Emergency Stop status
    self.estop_triggered = False

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
    self.cmd_vel_pub.publish(stop_msg)
    rospy.loginfo("Emergency Stop Triggered! Publishing zero velocities.")
  
  def estop_callback(self, data):
    if data.data:
      # E-stop triggered. Take necessary actions;
      rospy.loginfo("Emergency Stop Triggered!")
      # Add code to stop UGV
      self.emergency_stop_publisher();
    else:
      # Normal operation or E-stop reset.
      self.estop_triggered = False
      rospy.loginfo("Normal Operation")

  def move_to_target(self, current_pose, target_pose):
    # Calculate distance error (linear error)
    dx = target_pose.position.x - current_pose.position.x
    dy = target_pose.position.y - current_pose.position.y
    distance_error = math.sqrt(dx**2 + dy**2)

    # Calculate angular error
    current_yaw = self.get_yaw_from_quaternion(current_pose.orientation)
    target_yaw = self.get_yam_from_quaternion(target_pose.orientaton)
    angular_error = target_yaw - current_yaw

    # Get control commands from PID controllers
    linear_velocity = self.pid_linear(distance_error)
    angular_velocity = self.pid_angular(angular_error)

    # Populate the Twist message
    cmd = Twist()
    cmd.linear.x = linear_velocity
    cmd.angular.z = angular_velocity

    # Publish the command
    self.cmd_vel_pub.publish(cmd)

  @staticmethod
  def get_yaw_from_quaternion(orientation):
    # Convert quaternion to euler using scipy
    r = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
    euler = r.as_euler('zyx', degrees=False) # 'zyx' means the intrinsic rotation sequence
    return euler[0] # Return yaw (which is the 'z' rotation in the 'zyx' sequence)

  def calculate_target_pose(self, marker_pose):
    # Calculate the Dynamic Target Pose, in ODOM frame
    # It is the difference between the Static Marker Pose - the Dynamic Marker Pose
    # The self.marker_pose is the Static Marker Pose, the marker_pose is the Dynamic Marker Pose
    # Note that the Static Marker Pose is in ODOM frame, whereas the Dynamic Marker Pose is in Camera frame.
    
    # Project the 3D position of the marker-pose onto the X-Y plane
    target_pose = PoseStamped()
    # target_pose.pose.position.x = self.marker_pose.pose.position.x - marker_pose.pose.position.z
    target_pose.pose.position.x = self.marker_pose.pose.position.x - self.distance
    # target_pose.pose.position.y = self.marker_pose.pose.position.y + marker_pose.pose.position.x
    target_pose.pose.position.y = self.marker_pose.pose.position.y
    target_pose.pose.position.z = 0 # it is on the X-Y plane therefore z = 0

    # Project the 3D orientation to 2D orientation
    # Convert marker's orientation to rotation matrix
    marker_quaternion = (marker_pose.pose.orientation.x, marker_pose.pose.orientation.y, 
                         marker_pose.pose.orientation.z, marker_pose.pose.orientation.w)
    r = R.from_quat(marker_quaternion)
    marker_matrix = r.as_matrix()
    
    # Extract the X and Y components of the marker's Z-axis
    # The Z-axis of the marker lies on the X-Y plane
    target_x_axis = [-marker_matrix[2, 0], 0, 0]
    target_y_axis = [0, marker_matrix[2, 1], 0] 
    target_z_axis = [0, 0, 0] # Nothing for the Z-axis for the X-Y plane

    target_r = R.from_matrix([target_x_axis, target_y_axis, target_z_axis])
    target_quaternion = target_r.as_quat()

    # Set the target orientation
    target_pose.pose.orientation.x = target_quaternion[0]
    target_pose.pose.orientation.y = target_quaternion[1]
    target_pose.pose.orientation.z = target_quaternion[2]
    target_pose.pose.orientation.w = target_quaternion[3]

    # Complete the PoseStamped format (header)
    target_pose.header.frame_id = self.odom_frame
    target_pose.header.stamp = rospy.Time.now()
    
    return target_pose

  def marker_callback(self, marker_pose):
    # This pose is always the Marker pose with reference to the Camera
    # This function will be called every time a new Pose message is received

    # print('Inside callback.')
    # Calculate the marker pose with respect to odom if not done yet
    # print('marker pose: ', self.marker_pose)
    if self.marker_pose is None:
      # look up the transformation from the camera frame to the robot frame
      # buffer.lookup_transform(target frame, source frame, ...)
      try:
        # look up the Camera to Odom transformation
        # self.Tc2o = self.tf_buffer.lookup_transform(self.camera_frame, self.odom_frame, rospy.Time(0))
        self.Tc2o = self.tf_buffer.lookup_transform(self.odom_frame, self.camera_frame, rospy.Time(0))

        # look up the Camera to UGV transform
        self.Tc2u = self.tf_buffer.lookup_transform(self.ugv_frame, self.camera_frame, rospy.Time(0))
        # self.Tc2r = self.tf_buffer.lookup_transform(self.camera_frame, self.robot_base, rospy.Time(0))

      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
          rospy.logerror("Transformation not available")
      
      # The first Marker Pose received before the UGV is moved.
      self.marker_pose = tf2_geometry_msgs.do_transform_pose(marker_pose, self.Tc2o)
      # From now on, the Static Marker Pose is with reference to ODOM and that is FIXED
      # print('marker pose: ', self.marker_pose)
      print('*********************** marker pose has been setup. *************************************')

    target_pose = self.calculate_target_pose(marker_pose)

    # Publish the Dynamic Target Pose in RViz
    self.target_pub.publish(target_pose)

    # Calculate the CURRENT_pose for "base" of ROBOT
    current_pose = self.tf_buffer.lookup_transform(self.camera_frame, self.robot_base, rospy.Time(0))

  def run(self):
    rate = rospy.Rate(10) # 10 Hz

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
          self.move_to_target()
        else:
          print('User does not want to Move the UGV.')

        # The service cannot be shutdown from the Client side!

        # exit the loop
        break

      rate.sleep()


