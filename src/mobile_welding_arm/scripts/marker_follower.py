#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
  Chinese National Engineering Research Centre for Steel Structure
                        (Hong Kong Branch)

  Hong Kong Polytechnic University

  Author: Victor W H Wu
  Date: 10 August 2023.

  Basically, or preliminary working.
  Date: 27 September 2023.

  File name: marker_follower.py

  Description:
    This python script subscribes to the pose of the Charuco board centre
    as published by marker_pose. This pose will be used to control the UGV.

'''

import rospy
import tf2_ros
import tf2_geometry_msgs
import math
from tf.transformations import quaternion_matrix, quaternion_from_matrix, inverse_matrix
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
import welding_msgs.srv 

LINEAR_THRESHOLD = 0.025 # Acceptable position error in meters (2.5cm)
ANGULAR_THRESHOLD = 0.025 # Acceptable orientation error in radians (1.43 degree)
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

class State:
  ALIGN_HEADING = "align_heading"
  MOVE_TO_TARGET = "move_to_target"
  ADJUST_ORIENTATION = "adjust_orientation"

class MarkerFollower:

  def __init__(self, distance):

    # PID Controller for controlling the UGV
    self.linear_pid = PID(kp=0.75, ki=0.0, kd=0.00)
    self.angular_pid = PID(kp=0.75, ki=0.0, kd=0.00)

    # Create a Buffer and a TransformListener for tf2 lookup
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    # Subscribe the Marker's pose, in the camera frame
    self.marker_sub = rospy.Subscriber('/marker_pose_stamped', PoseStamped, self.marker_callback)

    # Subscribe to Emergency Stop
    rospy.Subscriber('/estop', Bool, self.estop_callback)

    # Publisher for the pose of the target and current Robot base
    self.camera_target_pub = rospy.Publisher('/camera_target_pose', PoseStamped, queue_size=1)
    self.ugv_target_pub = rospy.Publisher('/ugv_target_pose', PoseStamped, queue_size=1)
    self.current_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)

    # Publisher for the velocity command to move the UGV
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Distance from marker to camera
    self.distance = distance

    # Names of the source and target frames
    self.camera_frame = 'd435_color_frame' # changed by V Wu on 19 Sep 2023.
    self.optical_frame = 'd435_color_optical_frame'
    self.odom_frame = 'odom'
    self.ugv_frame = 'bunker_pro_base_link'
    self.marker_frame = 'marker'

    # Setup an empty poses in 'odom' frame
    self.marker_pose = None
    self.current_pose = None
    self.camera_target_pose = None
    self.ugv_target_pose = None

    # Initialize Emergency Stop status
    self.estop_triggered = False

    # Initialize the Finite State Machine
    self.state = State.ALIGN_HEADING

    # attributes for control_ugv
    self.target_position_reached = False
    self.heading_yaw_reached = False
    self.done = False       # target pose reached
    self.target_angular = 0 # orientation of the target pose
    self.target_yaw = 0     # orientation to face target position

    # Look up the transformation from optical frame to camera frame
    try:
      self.optical2camera_transform = self.tf_buffer.lookup_transform(self.optical_frame, self.camera_frame,
                                                                      rospy.Time(0), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
      rospy.logerr(e)

    # Look up the transformation from camera to UGV base
    try:
      self.camera2ugv_transform = self.tf_buffer.lookup_transform(self.camera_frame, self.ugv_frame, rospy.Time(0), 
                                                                  rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
      rospy.logerr(e)

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
    while self.estop_triggered: # keep sending zero velocities to the UGV until the E-Stop is reset
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
    # Lookup the current pose of the UGV in odom frame
    try:
      transform = self.tf_buffer.lookup_transform(self.odom_frame, self.ugv_frame, rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      rospy.logerr("Failed to get current camera pose from tf")
      return None
    # Only construct the current pose if the transform is looked up completed
    current_pose = PoseStamped()
    current_pose.pose.position.x = transform.transform.translation.x
    current_pose.pose.position.y = transform.transform.translation.y
    current_pose.pose.position.z = transform.transform.translation.z
    current_pose.pose.orientation.x = transform.transform.rotation.x
    current_pose.pose.orientation.y = transform.transform.rotation.y
    current_pose.pose.orientation.z = transform.transform.rotation.z
    current_pose.pose.orientation.w = transform.transform.rotation.w
    current_pose.header = transform.header
    self.current_pub.publish(current_pose)
    return current_pose

  def get_heading_to_target(self):
    dx = self.ugv_target_pose.pose.position.x - self.current_pose.pose.position.x
    dy = self.ugv_target_pose.pose.position.y - self.current_pose.pose.position.y
    return math.atan2(dy, dx)

  def angle_difference(self, angle1, angle2):
    return math.atan2(math.sin(angle1 - angle2), math.cos(angle1 - angle2))

  def angular_error(self):

    # The orientation of a pose can be expressed in many different representations.
    # One of which is by quaternion, that is the preferred representation used by ROS.
    # While we are dealing with the movement of a UGV, that is on the X-Y plane
    # therefore a rotation about the Z-axis would be intuitive and convenient
    # Hence, the RPY (Row, Pitch, Yaw) representation is preferred
    # Where Row is rotation about X-axis, Pitch is rotation about Y-axis, and
    # Yaw is rotation about Z-axis, therefore we 
    # Convert target quaternion to Yaw using tf.transformations
    target_quaternion = (self.ugv_target_pose.pose.orientation.x, self.ugv_target_pose.pose.orientation.y,
                         self.ugv_target_pose.pose.orientation.z, self.ugv_target_pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(target_quaternion)
    target_yaw = euler[2]
    rospy.logdebug('target_yaw: %f', target_yaw)

    # Convert current quaternion to Yaw using tf.transformations
    current_quaternion = (self.current_pose.pose.orientation.x, self.current_pose.pose.orientation.y,
                          self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(current_quaternion)
    current_yaw = euler[2]
    rospy.logdebug("current_yaw: %f", current_yaw)

    # Compute the Angular Error
    angular_error = self.angle_difference(target_yaw, current_yaw)
    return angular_error

  def heading_error(self):

    # Determine the Yaw to face the target position (or Heading)
    # target_yaw = self.get_heading_to_target()
    heading_yaw = math.atan2(self.ugv_target_pose.pose.position.y, self.ugv_target_pose.pose.position.x)
    rospy.logdebug('heading yaw: %f', heading_yaw)
    # Convert current quaternion to yaw using tf.transformations
    current_quaternion = (self.current_pose.pose.orientation.x, self.current_pose.pose.orientation.y,
                          self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(current_quaternion)
    current_yaw = euler[2]
    rospy.logdebug("current_yaw: %f", current_yaw)
    # Compute the Heading Error
    heading_error = self.angle_difference(heading_yaw, current_yaw)
    return heading_error

  def linear_error(self):
    dx = self.ugv_target_pose.pose.position.x - self.current_pose.pose.position.x
    dy = self.ugv_target_pose.pose.position.y - self.current_pose.pose.position.y
    return math.sqrt(dx**2 + dy**2)

  def align_heading(self):
    angular_correction = 0
    heading_error = self.heading_error()
    rospy.logdebug('Heading error: %f', heading_error)
    if abs(heading_error) < ANGULAR_THRESHOLD:
      self.state = State.MOVE_TO_TARGET
    else:
      angular_correction = self.angular_pid.compute(heading_error)
    return angular_correction

  def move_to_target(self):
    linear_correction = 0
    linear_error = self.linear_error()
    rospy.logdebug('Linear error: %f', linear_error)
    if abs(linear_error) < LINEAR_THRESHOLD:
      self.state = State.ADJUST_ORIENTATION
    else:
      linear_correction = self.linear_pid.compute(linear_error)
    return linear_correction

  def adjust_orientation(self):
    angular_correction = 0
    angular_error = self.angular_error()
    rospy.logdebug('Angular error: %f', angular_error)
    if abs(angular_error) < ANGULAR_THRESHOLD:
      self.done = True
    else:
      angular_correction = self.angular_pid.compute(angular_error)
    return angular_correction

  def control_ugv(self):

    self.current_pose = self.get_current_pose()
    if not self.current_pose:
      # No current_pose, nothing can be done!
      return
    if self.done:
      # Target pose reached, nothing should be done.
      return

    linear_correction = 0
    angular_correction = 0

    if self.state == State.ALIGN_HEADING:
      angular_correction = self.align_heading()

    elif self.state == State.MOVE_TO_TARGET:
      linear_correction = self.move_to_target()

    elif self.state == State.ADJUST_ORIENTATION:
      angular_correction = self.adjust_orientation()

    '''
    linear_error = self.linear_error()
    heading_error = self.heading_error()
    angular_error = self.angular_error()

    # Target position reached
    rospy.logdebug('Target position reached: %s', self.target_position_reached)
    if self.target_position_reached:
      angular_error = self.angular_error()
      # Orientation is correct
      rospy.logdebug('Angular error: %f', angular_error)
      if abs(angular_error) < ANGULAR_THRESHOLD:
        # Target Pose reached
        self.done = True
      else:
        # Keep turning
        angular_correction = self.angular_pid.compute(angular_error)
    else:
      # Target position not reached yet
      rospy.logdebug('Heading yaw reached: %s', self.heading_yaw_reached)
      if self.heading_yaw_reached:
        # Keep going forward
        linear_error = self.linear_error()
        rospy.logdebug('Linear error: %f', linear_error)
        if abs(linear_error) < LINEAR_THRESHOLD:
          self.target_position_reached = True
        else:
          linear_correction = self.linear_pid.compute(linear_error)
      else:
        # Not in the right direction towards target position yet
        # Keep turning
        heading_error = self.heading_error()
        rospy.logdebug('Heading error: %f', heading_error)
        if abs(heading_error) >= ANGULAR_THRESHOLD:
          angular_correction = self.angular_pid.compute(heading_error)
        else:
          rospy.logdebug('Heading error: %f', heading_error)
          self.heading_yaw_reached = True
          # Already in the right direction, keep going forward
          linear_error = self.linear_error()
          rospy.logdebug('Linear error: %f', linear_error)
          linear_correction = self.linear_pid.compute(linear_error)
    '''
    rospy.logdebug('Linear correction: %f', linear_correction)
    rospy.logdebug('Angular correction: %f', angular_correction)
    if not self.estop_triggered and not self.done:
      twist = Twist()
      twist.angular.z = angular_correction
      twist.linear.x = linear_correction
      self.cmd_vel_pub.publish(twist)
    return

  def calculate_ugv_target_pose(self, marker_pose):
    # marker_pose is in camera frame already

    # Project the 3D position of the marker-pose onto the X-Y plane
    def project2xy_plane(pose_3d):
      
      ''''''
      # Set z-coordinate to zero for position
      pose_3d.pose.position.z = 0

      
      # Get the pitch of the optical frame from the orientation of the pose
      roll, pitch, yaw = tf.transformations.euler_from_quaternion([pose_3d.pose.orientation.x, pose_3d.pose.orientation.y,
                                                            pose_3d.pose.orientation.z, pose_3d.pose.orientation.w])
      # Convert yaw to a quaternion (ignoring roll and pitch); Don't know why but needs to rotate by half pi.
      q = tf.transformations.quaternion_from_euler(0, 0, (yaw + math.pi/2))

      # Update the pose's orientation
      pose_3d.pose.orientation.x = q[0]
      pose_3d.pose.orientation.y = q[1]
      pose_3d.pose.orientation.z = q[2]
      pose_3d.pose.orientation.w = q[3]

      return pose_3d

    camera_target_pose = PoseStamped()
    ugv_target_pose = PoseStamped()
    optical_pose = PoseStamped()

    # The Optical pose in the optical frame is just all zero's
    # Position at (0, 0, 0)
    optical_pose.pose.position.x = 0
    optical_pose.pose.position.y = 0
    optical_pose.pose.position.z = 0
    # Orientation - no rotation (0, 0, 0, 1)
    optical_pose.pose.orientation.x = 0
    optical_pose.pose.orientation.y = 0
    optical_pose.pose.orientation.z = 0
    optical_pose.pose.orientation.w = 1
    # Header
    optical_pose.header.frame_id = self.optical_frame
    optical_pose.header.stamp = rospy.Time.now()

    # Transform the Optical pose (in the optical_frame) to the marker frame
    try:                                          #  Target frame      Source frame
      transform = self.tf_buffer.lookup_transform(self.marker_frame, self.optical_frame, rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      rospy.logerr("Failed to get current camera pose from tf")

    # The camera target pose in the marker's frame is transformed from the optical pose
    camera_target_pose = tf2_geometry_msgs.do_transform_pose(optical_pose, transform)
    # The position of the camera target pose should be -ve self.distance of the Z-axis of the marker frame
    camera_target_pose.pose.position.z = -self.distance
    # The orientation of the camera target pose should be the same as the marker frame, no rotation
    camera_target_pose.pose.orientation.x = 0
    camera_target_pose.pose.orientation.y = 0
    camera_target_pose.pose.orientation.z = 0
    camera_target_pose.pose.orientation.w = 1

    # Transform this camera_target_pose from marker frame to ugv frame
    try:
      transform = self.tf_buffer.lookup_transform(self.ugv_frame, self.marker_frame, rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      rospy.logerr("Failed to get current camera pose from tf")
    camera_target_pose = tf2_geometry_msgs.do_transform_pose(camera_target_pose, transform)
    self.camera_target_pub.publish(camera_target_pose)

    # 1. Retrieve Current Optical Pose
    trans_current = self.tf_buffer.lookup_transform(self.ugv_frame, self.optical_frame, rospy.Time(0))

    # 2. Define Camera Target Pose 
    trans_desired = TransformStamped()
    trans_desired.transform.translation.x = camera_target_pose.pose.position.x
    trans_desired.transform.translation.y = camera_target_pose.pose.position.y
    trans_desired.transform.translation.z = camera_target_pose.pose.position.z
    trans_desired.transform.rotation.x = camera_target_pose.pose.orientation.x
    trans_desired.transform.rotation.y = camera_target_pose.pose.orientation.y
    trans_desired.transform.rotation.z = camera_target_pose.pose.orientation.z
    trans_desired.transform.rotation.w = camera_target_pose.pose.orientation.w

    # 3. Compute the Transformation Difference
    # First, get the inverse of the current transformation in matrix form
    T_current_matrix = quaternion_matrix([
      trans_current.transform.rotation.x,
      trans_current.transform.rotation.y,
      trans_current.transform.rotation.z,
      trans_current.transform.rotation.w
    ])
    T_current_matrix[0:3, 3] = [
      trans_current.transform.translation.x,
      trans_current.transform.translation.y,
      trans_current.transform.translation.z
    ]
    # Convert the desired transformation to matrix form
    T_desired_matrix = quaternion_matrix([
      trans_desired.transform.rotation.x,
      trans_desired.transform.rotation.y,
      trans_desired.transform.rotation.z,
      trans_desired.transform.rotation.w
    ])
    T_desired_matrix[0:3, 3] = [
      trans_desired.transform.translation.x,
      trans_desired.transform.translation.y,
      trans_desired.transform.translation.z
    ]
    # Invert the matrix
    T_current_inv_matrix = inverse_matrix(T_current_matrix)
    # Multiply the matrices
    T_diff_matrix = T_desired_matrix.dot(T_current_inv_matrix)

    # 4. Extract Pose Information
    ugv_target_pose.pose.position.x = T_diff_matrix[0, 3]
    ugv_target_pose.pose.position.y = T_diff_matrix[1, 3]
    # ugv_target_pose.pose.position.z = T_diff_matrix[2, 3]
    ugv_target_pose.pose.position.z = 0
    ugv_target_pose.pose.orientation.x = quaternion_from_matrix(T_diff_matrix)[0]
    ugv_target_pose.pose.orientation.y = quaternion_from_matrix(T_diff_matrix)[1]
    ugv_target_pose.pose.orientation.z = quaternion_from_matrix(T_diff_matrix)[2]
    ugv_target_pose.pose.orientation.w = quaternion_from_matrix(T_diff_matrix)[3]

    # Complete the PoseStamped format (header)
    ugv_target_pose.header.frame_id = self.ugv_frame
    ugv_target_pose.header.stamp = rospy.Time.now()
    self.ugv_target_pose = ugv_target_pose
    self.ugv_target_pub.publish(self.ugv_target_pose)

    return

  def marker_callback(self, marker_pose):
    # This pose is always the Marker pose with reference to the Odom frame 
    # This function will be called every time a new Pose message is received
    # Calculate the marker pose with respect to odom if not done yet
    if self.marker_pose is None:
      # The first Marker Pose received before the UGV is moved.
      self.marker_pose = marker_pose
      rospy.loginfo('********** marker pose has been setup. **************')
    # Calculate the UGV target pose
    self.calculate_ugv_target_pose(marker_pose)
    # Publish the Target Pose in RViz
    # self.ugv_target_pub.publish(self.ugv_target_pose)
    self.current_pose = self.get_current_pose()
    # Unregister the marker_pose subscriber
    # self.marker_sub.unregister()
    # Hopefully, this fixes the marker and the target pose.

  def run(self):
    rate = rospy.Rate(10) # 10 Hz
    # Loop until Camera is up and running
    while not rospy.is_shutdown():
      if not self.marker_pose is None: # camera is up
        # Wait for the user reaction service
        rospy.wait_for_service('user_reaction')
        # Create a proxy for the service
        self.user_reaction_service = rospy.ServiceProxy('user_reaction', welding_msgs.srv.InteractService1)
        # Define the message to display on the panel
        display_message = "Do you want to move the UGV?"
        # Make the service call, and it will block here until a response is received.
        response = self.user_reaction_service(display_message)
        # Check the response
        if response.approved:
          rospy.loginfo('User wants to Move the UGV.')
          rospy.loginfo('Done: %s', self.done)
          while not rospy.is_shutdown() and not self.done:
            self.control_ugv()
            input('Next step, enter.')
        else:
          rospy.loginfo('User does not want to Move the UGV.')
        break
      rate.sleep()


