#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
  Chinese National Engineering Research Centre for Steel Structure
                        (Hong Kong Branch)

  Hong Kong Polytechnic University

  Author: Victor W H Wu
  Date: 22 August 2023.

  File name: joint_state_publisher.py

  Description:
    This python script publishes the joint states of the UR10.

'''

import rospy
from sensor_msgs.msg import JointState
import urx  # Import the URx package

def publish_joint_states():
    rospy.init_node('my_joint_state')
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Read joint names from parameter server
    joint_names = rospy.get_param('joint_names', [])
    print('joint names: ', joint_names)

    # Read robot IP address from parameter server
    robot_ip = rospy.get_param('robot_ip', '127.0.0.1')

    robot = urx.Robot(robot_ip)  # Replace with your robot's IP address

    while not rospy.is_shutdown():
        joint_angles = robot.getj()  # Get joint angles from URx

        joint_state_msg = JointState()
        joint_state_msg.name = joint_names
        joint_state_msg.position = joint_angles
        joint_state_msg.header.stamp = rospy.Time.now()

        pub.publish(joint_state_msg)
        rate.sleep()

    robot.close()

if __name__ == '__main__':
    try:
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass
