#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
  Chinese National Engineering Research Centre for Steel Structure
                        (Hong Kong Branch)

  Hong Kong Polytechnic University

  Author: Victor W H Wu
  Date: 17 October 2023.

  File name: ugv.py

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
from marker_follower import MarkerFollower

class UGV:
  def __init__(self, model):
    self.model = model
    self.robot = None

  def set_robot(self, robot):
    self.robot = robot

  # move the UGV to the target
  # the target is the distance from the marker on the Z-axis to the camera
  def move_to_target(self, distance):
    # print('********************* In move_to_target ****************************')
    marker_follower = MarkerFollower(distance)
    marker_follower.run()
    pass

  def __init__(self):
    
    