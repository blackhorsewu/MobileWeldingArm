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
      yaw, pitch, roll = r.as_euler('zyx')
      # Set roll to zero
      roll = 0
      yaw = 0
      r = R.from_euler('zyx', [yaw, pitch, roll])
      avg_orientation = r.as_quat()

      avg_position = avg_position.reshape((3,))
      avg_orientation = avg_orientation.reshape((4,))

      # establish the pose in ROS format
      pose_msg = Pose()
      pose_msg.position.x = avg_position[0]
      pose_msg.position.y = avg_position[1]
      pose_msg.position.z = avg_position[2]
      pose_msg.orientation.x = avg_orientation[0]
      pose_msg.orientation.y = avg_orientation[1]
      pose_msg.orientation.z = avg_orientation[2]
      pose_msg.orientation.w = avg_orientation[3]

      self.pose_pub.publish(pose_msg)
      # It has to be PoseStamped in order to publish to the RViz
      pose_stamped = PoseStamped()
      pose_stamped.pose = pose_msg
      pose_stamped.header.frame_id = 'd435_color_optical_frame'
      pose_stamped.header.stamp = rospy.Time.now()

      self.poseStamped_pub.publish(pose_stamped)

      # Create the TransformStamped message
      transform_stamped = TransformStamped()
      transform_stamped.header.stamp = rospy.Time.now()
      transform_stamped.header.frame_id = 'd435_color_optical_frame'
      transform_stamped.child_frame_id = 'marker'
      transform_stamped.transform.translation.x = pose_stamped.pose.position.x
      transform_stamped.transform.translation.y = pose_stamped.pose.position.y
      transform_stamped.transform.translation.z = pose_stamped.pose.position.z
      transform_stamped.transform.rotation = pose_stamped.pose.orientation

      # Broadcast the transformation of the marker from optical frame
      self.tf_broadcaster.sendTransform(transform_stamped)

      try: 
        self.Tc2u = self.tf_buffer.lookup_transform('bunker_pro_base_link', 'd435_color_optical_frame', rospy.Time(0))
        # Transform from camera to UGV

        # Construct my own 'World'
        # The 'pose_stamped' is the marker itself
        world_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, self.Tc2u)
        # It should has the same orientation as the 'Marker' except that its 'roll' and 'pitch' 
        # should be zero, with reference to the 'bunker_pro_base_link', that is horizontal.
        # same X and Y coordinates, but the Z will be different (should be 0)
        # This is in the 'bunker_pro_base_frame'.
        # Then transform it again back to the 'marker' frame
        world_pose_r = R.from_quat((world_pose.pose.orientation.x, world_pose.pose.orientation.y,
                                    world_pose.pose.orientation.z, world_pose.pose.orientation.w))
        _, _, yaw = world_pose_r.as_euler('xyz')
        world_pose_r = R.from_euler('xyz', [0, 0, (yaw + math.pi/2)])
        world_pose_q = world_pose_r.as_quat()
        world_pose.pose.orientation.x = world_pose_q[0]
        world_pose.pose.orientation.y = world_pose_q[1]
        world_pose.pose.orientation.z = world_pose_q[2]
        world_pose.pose.orientation.w = world_pose_q[3]
        world_pose.pose.position.z = 0
        # self.poseStamped_pub.publish(world_pose)
    
        self.Tu2m = self.tf_buffer.lookup_transform('marker', 'bunker_pro_base_link', rospy.Time(0))
        self.world_pose = tf2_geometry_msgs.do_transform_pose(world_pose, self.Tu2m)
        self.world_pose_Stamped_pub.publish(self.world_pose)
        # Transform from UGV to marker

        # Create the TransformStamped message for my 'world' transform
        world_transform_stamped = TransformStamped()
        world_transform_stamped.header.stamp = rospy.Time.now()
        world_transform_stamped.header.frame_id = 'marker'
        world_transform_stamped.child_frame_id = 'world'
        world_transform_stamped.transform.translation.x = self.world_pose.pose.position.x
        world_transform_stamped.transform.translation.y = self.world_pose.pose.position.y
        world_transform_stamped.transform.translation.z = self.world_pose.pose.position.z
        world_transform_stamped.transform.rotation = self.world_pose.pose.orientation

        # Broadcast the transformation of the marker from optical frame
        self.tf_broadcaster.sendTransform(world_transform_stamped)

      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Transformation not available!")


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
    self.world_frame = 'world'
    self.camera_optical_frame = 'd435_color_optical_frame'
    self.camera_frame = 'd435_color_frame'

    # Subscribe to the camera info and image topics
    self.info_subscriber = rospy.Subscriber('/d435/color/camera_info', CameraInfo, self.camera_info_callback, queue_size=10)
    # Spin until the camera info is populated
    while self.intrinsic_camera is None or self.distortion is None:
      rospy.sleep(0.1)
    
    self.subscriber = rospy.Subscriber('/d435/color/image_raw', Image, self.image_callback, queue_size=10)

    self.tf_broadcaster = tf2_ros.TransformBroadcaster()
    self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    self.world_transform = None

    # Publisher for the marker pose message
    self.pose_pub = rospy.Publisher('/marker_pose', Pose, queue_size=10)
    self.poseStamped_pub = rospy.Publisher('/marker_pose_stamped', PoseStamped, queue_size=10)
    self.world_pose_Stamped_pub = rospy.Publisher('/world_pose_stamped', PoseStamped, queue_size=10)

if __name__ == '__main__':
  marker_detector = MarkerDetector()
  marker_detector.run()
