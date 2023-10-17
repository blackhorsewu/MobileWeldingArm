#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
  Chinese National Engineering Research Centre for Steel Structure
                        (Hong Kong Branch)

  Hong Kong Polytechnic University

  Author: Victor W H Wu
  Date: 17 October 2023.

  File name: robot_arm.py

  Description:
    This python script controls a UR10 Robotic Arm.
    1. The top level "welding_system" instantiate a 'robot_arm' with the 'robot_ip'.
    2. A 'robot' is instantiated using the URx library with the 'robot_ip' address.
    3. A 'joint_publisher' is instantiated. This 'joint_publisher' will setup a publisher
       to publish the topic '/joint_states'
    4. The 'joint_publisher' has a method 'run'. This method is executed in a separate thread.
    5. The 'run' method get the joint positions from the robot via the URx library in a loop
       continuously at a rate of 10Hz.
    6. This module is also responsible to handle the Emergency Stop for the robot arm, therefore
       it subscribe to the '/estop' topic and provide a call back method, 'estop_callback'

  Assumptions:
    1. At the beginning of the launch file of the system, there is a command to load a YAML
       file "joint_names.yaml" in the directory "/config" under "mobile_welding_arm", into the
       ROS parameter server.
    2. Once the 'robot_arm' in the top level "welding_system", it will invoke the 'start' method
       of this module to start the whole thing especially to publish the joint states.

'''

import rospy
import threading
import urx # Import the URx library
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

class JointStatePublisher:
  def __init__(self, robot):
    self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    self.robot = robot
    # Read joint names from parameter server
    self.joint_names = rospy.get_param('joint_names', [])
    pass

  def run(self):
    while not rospy.is_shutdown():
      joint_state = JointState()
      joint_state.name = self.joint_names
      joint_state.position = self.robot.getj()
      joint_state.header.stamp = rospy.Time.now()
      self.joint_state_pub.publish(joint_state)
      rospy.sleep(0.1)
    self.robot.stop()
    self.robot.close()
    pass

class RobotArm(object):
  def __init__(self, robot_ip):
    self.robot = urx.Robot(robot_ip)
    self.estop_sub = rospy.Subscriber('/estop', Bool, self.estop_callback)
    self.joint_publisher = JointStatePublisher(self.robot)
    pass

  def move_j(self, angles, acc, vel):
    self.robot.movej(angles, acc, vel)
    pass

  def set_tcp(self, pose):
    self.robot.set_tcp(pose)
    pass

  def start(self):
    # Start the joint state publisher in a separate thread
    threading.Thread(target=self.joint_publisher.run).start()
    # Here, other robot commands or logic can be invoked
    pass

  def estop_callback(self, msg):
    if msg.data:
      rospy.loginfo("Emergency stop activated!")
      self.robot.stop()
      self.robot.close()
    else:
      rospy.loginfo("Emergency stop deactivated!")

