#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
  Chinese National Engineering Research Centre for Steel Structure
                        (Hong Kong Branch)

  Hong Kong Polytechnic University
'''
'''
  Author: Victor W H Wu
  Date: 8 August 2023.

  Update: 4 October 2023.

  File name: marker_pose_node.py

  Description:
    This python script subscribes to the RGB camera and publish the pose of the
    Charuco board centre.

    Updated on 11 October 2023.
    1. Lookup the transformation from the UR10 flange to the camera optical frame.
       Use this to set the TCP in URx then can move the camera optical frame directly
       with its pose.
    2. 

  It requires:
  1. OpenCV
  2. cv_bridge in ROS
  3. The Intrinsic camera matrix and the Distortion parameters of the RealSense
     D435 camera 

'''

import rospy
import tf2_ros
import tf2_geometry_msgs
import cv2
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation as R
from collections import deque
import math

# from chsweld_core.msg import URPose
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped

# specify parameters for Charuco board
squaresX = 3  # number of chessboard squares in X direction
squaresY = 3  # number of chessboard squares in Y direction
squareLength = 0.0265  # side length of a square, in meters
markerLength = 0.02  # side length of a marker, in meters

ARUCO_DICT = {
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
}

aruco_type = "DICT_6X6_100"

class MarkerDetector:

  def camera_info_callback(self, info_msg):
    self.intrinsic_camera = np.array(info_msg.K).reshape(3, 3)
    self.distortion = np.array(info_msg.D)

  def image_callback(self, image):

    # print('I am in image callback.')
    # Check if the intrinsic parameters have been initialized
    # otherwise give up and return
    if self.intrinsic_camera is None or self.distortion is None:
      # print('I am Image Callback 1.')
      return

    # print('I am Image Callback 2.')
    # print(image)
    cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, _ = cv2.aruco.detectMarkers(
        gray,
        aruco_dict #, parameters=parameters
      )

    # Make sure at least two marker has been detected
    if len(corners) > 1:
      # rvec and tvec contain the rotation and translation vectors for each detected marker
      rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        self.markerLength, 
        self.intrinsic_camera,
        self.distortion
      )

      # print('I am Image Callback 3.')
      # Draw the detected ArUco markers (if any) on the image
      # cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

      # Display the image with the drawn markers
      # cv2.imshow('Detected Markers', cv_image)
      # cv2.waitKey(1) # Waits for 1 millisecond before moving to the next frame

      # Averaging translation 
      avg_position = np.mean(tvecs, axis=0)

      # Convert rvecs to rotation matrices
      R1 = cv2.Rodrigues(rvecs[0])[0]
      R2 = cv2.Rodrigues(rvecs[1])[0]

      # Compute the average rotation matrix
      avg_orientation = (R1 + R2) / 2

      # Normalize the averaged rotation matrix
      U, _, Vt = np.linalg.svd(avg_orientation)
      avg_orientation = U @ Vt
      # Rotate the pose about the X-axis to turn the pose away from the camera
      avg_orientation = np.matmul(avg_orientation, [[1, 0, 0], [0, -1, 0], [0, 0, -1]])

      # Convert it to quaternion
      r = R.from_matrix(avg_orientation)
      _, pitch, _ = r.as_euler('zyx')
      # Set roll, and yaw to zero
      r = R.from_euler('zyx', [0, pitch, 0])
      avg_orientation = r.as_quat()

      avg_position = avg_position.reshape((3,))
      avg_orientation = avg_orientation.reshape((4,))

      # establish the pose in ROS format
      pose_msg = PoseStamped()
      pose_msg.pose.position.x = avg_position[0]
      pose_msg.pose.position.y = avg_position[1]
      pose_msg.pose.position.z = avg_position[2]
      pose_msg.pose.orientation.x = avg_orientation[0]
      pose_msg.pose.orientation.y = avg_orientation[1]
      pose_msg.pose.orientation.z = avg_orientation[2]
      pose_msg.pose.orientation.w = avg_orientation[3]
      pose_msg.header.frame_id = 'd435_color_optical_frame'
      pose_msg.header.stamp = rospy.Time.now()

      # It is not necessary to time stamp the pose before aligning it to the camera centre
      # First, transform this pose from 'optical_frame' to robot base frame

      # Lookup the Transformation from the 'optical_frame' to the robot base frame.
      try:
        pose_transform = self.tf_buffer.lookup_transform(self.robot_base_frame, self.camera_optical_frame, rospy.Time(0))
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Failed to get current camera pose from tf")
      # Perform the transform
      pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_msg, pose_transform)

      # Convert the quaternion to rotvec
      r = R.from_quat([pose_transformed.pose.orientation.x, pose_transformed.pose.orientation.y,
                       pose_transformed.pose.orientation.z, pose_transformed.pose.orientation.w])
      rotvec = r.as_rotvec()

      optical_pose = [pose_transformed.pose.position.x, pose_transformed.pose.position.y, 
                      pose_transformed.pose.position.z, rotvec[0], rotvec[1], rotvec[2]]
      # print(optical_pose)
      # input('Hit enter to continue.')

      self.poseStamped_pub.publish(pose_msg)

      # Create the TransformStamped message
      transform_stamped = TransformStamped()
      transform_stamped.header.stamp = rospy.Time.now()
      transform_stamped.header.frame_id = 'd435_color_optical_frame'
      transform_stamped.child_frame_id = 'marker'
      transform_stamped.transform.translation.x = pose_msg.pose.position.x
      transform_stamped.transform.translation.y = pose_msg.pose.position.y
      transform_stamped.transform.translation.z = pose_msg.pose.position.z
      transform_stamped.transform.rotation = pose_msg.pose.orientation

      # Broadcast the transformation of the marker from optical frame
      self.tf_broadcaster.sendTransform(transform_stamped)

  def run(self):
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
      rate.sleep()

  def __init__(self):
    # print('I am here in marker detector.')

    # Initialize the ROS node
    rospy.init_node('marker_detector', anonymous=True)

    # Acquire the parameters from launch file or command line
    # The physical side of the marker is 60cm
    self.markerLength = rospy.get_param('~markerLength', 0.06)
    self.aruco_type = rospy.get_param('~aruco_type', "DICT_6X6_100")

    # Initialize the intrinsic camera parameters
    self.intrinsic_camera = None
    self.distortion = None

    # Create a CV bridge
    self.bridge = CvBridge()

    # Create a Buffer and a TransformListener for tf2 lookup
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
    # Names of the source and target frames
    self.camera_optical_frame = 'd435_color_optical_frame'
    self.robot_base_frame = 'base'
    self.flange_frame = 'flange'

    # Subscribe to the camera info and image topics
    self.info_subscriber = rospy.Subscriber('/d435/color/camera_info', CameraInfo, self.camera_info_callback, queue_size=10)
    # Spin until the camera info is populated
    while self.intrinsic_camera is None or self.distortion is None:
      rospy.sleep(0.1)
    
    self.subscriber = rospy.Subscriber('/d435/color/image_raw', Image, self.image_callback, queue_size=10)

    self.tf_broadcaster = tf2_ros.TransformBroadcaster()
    self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Lookup the Transformation from the "flange" to the camera optical frame.
    # This is used to set the TCP for moving the UR10.
    try:
      self.tcp_transform = self.tf_buffer.lookup_transform(self.camera_optical_frame, self.flange_frame, rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      rospy.logerr("Failed to get current camera pose from tf")

    # Publisher for the marker pose message
    self.poseStamped_pub = rospy.Publisher('/marker_pose_stamped', PoseStamped, queue_size=10)

if __name__ == '__main__':
  marker_detector = MarkerDetector()
  marker_detector.run()
