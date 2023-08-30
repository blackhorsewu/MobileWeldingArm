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
    This python script manages the connection to the UR10, and
    1. publishes the joint states of the UR10.

'''

from ur_connection_manager.srv import MoveL, MoveLResponse
from ur_connection_manager.srv import MoveJ, MoveJResponse
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import urx  # Import the URx package


class URConnectionManager:
  def __init__(self, robot_ip):
    self.robot = urx.Robot(robot_ip)
    self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    self.estop_sub = rospy.Subscriber('/estop', Bool, self.estop_callback)
    # Read joint names from parameter server
    self.joint_names = rospy.get_param('joint_names', [])
    # print('joint names: ', joint_names)
    # register the services
    self.move_l_service = rospy.Service('move_l', MoveL, self.handle_move_l)
    self.move_j_service = rospy.Service('move_j', MoveJ, self.handle_move_j)
    pass

  def publish_joint_states(self):
    joint_state = JointState()
    joint_state.name = self.joint_names
    joint_state.position = self.robot.getj()
    joint_state.header.stamp = rospy.Time.now()
    self.joint_state_pub.publish(joint_state)

  def handle_move_l(self, req):
    destination_pose = req.destination_pose
    acceleration = req.acceleration
    velocity = req.velocity
    wait = req.wait
    success = True
    return MoveLResponse(success)

  def handle_move_j(self, req):
    destination_pose = req.destination_pose
    acceleration = req.acceleration
    velocity = req.velocity
    wait = req.wait
    self.robot.movej(destination_pose, acceleration, velocity, wait)
    success = True
    return MoveJResponse(success)

  def estop_callback(self, msg):
    if msg.data:
      rospy.loginfo("Emergency stop activated!")
      self.stop_robot()
      self.close_connection()
    else:
      rospy.loginfo("Emergency stop deactivated!")

  def stop_robot(self):
    self.robot.stop()

  def close_connection(self):
    self.robot.close()

def main():

  rospy.init_node('ur_connection_manager')
  # Read robot IP address from parameter server
  robot_ip = rospy.get_param('~robot_ip')
  manager = URConnectionManager(robot_ip)

  rate = rospy.Rate(10) # 10 Hz
  try:
    while not rospy.is_shutdown():
      manager.publish_joint_states()
      rate.sleep()
    # Make sure the connection is closed before shutdown.
    manager.close_connection()
  finally:
    manager.close_connection()

if __name__ == '__main__':
  main()