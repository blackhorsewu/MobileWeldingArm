#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
  Chinese National Engineering Research Centre for Steel Structure
                        (Hong Kong Branch)

  Hong Kong Polytechnic University

  Author: Victor W H Wu
  Date: 11 August 2023.

  File name: my_utilities.py

  Description:
    This python script put some useful utilities methods used throughout the project.

  They are:
  1. ur_pose_to_ros_pose
  2. ros_pose_to_ur_pose
  3. invert_transform

'''

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, TransformStamped
from scipy.spatial.transform import Rotation as R
import rospy
import tf.transformations as tf_trans

def ur_pose_to_ros_pose(ur_pose):
  pose = Pose()
  pose.position.x = ur_pose[0]
  pose.position.y = ur_pose[1]
  pose.position.z = ur_pose[2]
  r = R.from_rotvec(ur_pose[3:])
  quaternion = r.as_quat()
  pose.orientation.x = quaternion[0]
  pose.orientation.y = quaternion[1]
  pose.orientation.z = quaternion[2]
  pose.orientation.w = quaternion[3]
  return pose

def ur_pose_to_ros_pose_stamped(ur_pose, frame):
  pose_stamped = PoseStamped()
  pose_stamped.header.frame_id = frame
  pose_stamped.pose = ur_pose_to_ros_pose(ur_pose)
  pose_stamped.header.stamp = rospy.Time.now()
  return pose_stamped

def invert_transform(transform_stamped):
    # Convert TransformStamped to a transformation matrix
    translation = [
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z,
    ]
    rotation = [
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w,
    ]
    transform_matrix = tf_trans.concatenate_matrices(
        tf_trans.translation_matrix(translation),
        tf_trans.quaternion_matrix(rotation),
    )

    # Invert the transformation matrix
    inverse_transform_matrix = tf_trans.inverse_matrix(transform_matrix)

    # Convert the inverted transformation matrix back to TransformStamped
    inverse_transform_stamped = TransformStamped()
    inverse_transform_stamped.header = transform_stamped.header  # Adjust as needed
    inverse_transform_stamped.child_frame_id = transform_stamped.header.frame_id
    inverse_transform_stamped.transform.translation.x, inverse_transform_stamped.transform.translation.y, inverse_transform_stamped.transform.translation.z = tf_trans.translation_from_matrix(
        inverse_transform_matrix
    )
    inverse_transform_stamped.transform.rotation.x, inverse_transform_stamped.transform.rotation.y, inverse_transform_stamped.transform.rotation.z, inverse_transform_stamped.transform.rotation.w = tf_trans.quaternion_from_matrix(
        inverse_transform_matrix
    )

    return inverse_transform_stamped
