#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
  Chinese National Engineering Research Centre for Steel Structure
                        (Hong Kong Branch)

  Hong Kong Polytechnic University

  Author: Victor W H Wu
  Date: 24 August 2023.

  File name: mobile_welding_arm.py

  Description:
    This is the top level script of the whole "mobile welding arm" system.

'''

import rospy
from std_msgs.msg import String

import urx
from urx import Robot

from transitions import Machine

from ur_connection_manager.srv import MoveJ, MoveJRequest

from marker_follower import MarkerFollower

# Joint angles of the view pose for UR10
VIEW_JOINTS = [-0.0524, -1.1756, -2.6466, 0.8147, 1.5948, 0.0049]

class ColorCamera:
  def __init__(self, model):
    self.model = model

class DepthCamera:
  def __init__(self, model):
    self.model = model
    pass

  def capture_point_cloud(self):
    pass

class Robot:
  def __init__(self, model='UR10'):
    self.model = model
    self.current_pose = None
    self.is_welding = False
    self.color_camera = None
    self.depth_camera = None

    # Wait for MoveJ service
    rospy.wait_for_service('move_j')
    self.move_j = rospy.ServiceProxy('move_j', MoveJ)

    self.move_to_viewing_pose(VIEW_JOINTS)
    pass

  def add_color_camera(self, camera):
    self.color_camera = camera

  def add_depth_camera(self, camera):
    self.depth_camera = camera

  def move_to_viewing_pose(self, pose):
    response = self.move_j(pose, 0.4, 0.4, False)

  # The pose here specify the pose of the TCP
  def move_to_pose(self, pose):
    # Code to move the arm to a specific pose
    self.current_pose = pose

  def start_welding(self):
    # Code to initiate welding process
    self.is_welding = True

  def stop_welding(self):
    # Code to stop welding process
    self.is_welding = False

  def follow_path(self, path):
    # Code to make the arm follow a specific path,
    # possibly calling move_to_pose in a loop
    pass

class UGV:
  def __init__(self, model):
    self.model = model
    self.robot = None

  def set_robot(self, robot):
    self.robot = robot

  # move the UGV to the target
  # the target is the distance from the marker on the Z-axis to the camera
  def move_to_target(self, distance):
    print('********************* In move_to_target ****************************')
    marker_follower = MarkerFollower(distance)
    marker_follower.run()
    pass

class WeldingPath:
  def generate_path(self):
    pass

class WeldingSystem:
  def __init__(self):
    print('Initiating the system.')
    self.ugvs = []
    self.view_joints = []
    
    # Initialize ROS node
    rospy.init_node('welding_system', anonymous=True)
    print('ROS initialized.')

    # Setup publishers
    self.ask_user_confirm_ugv_move_pub = rospy.Publisher('ask_user_confirm_ugv_move', String, queue_size=10)
    
    # Define the states
    # To start with just 3 states:
    states = ['Initialization', 'UGVMovement', 'Error']
    
    # Initialize the state machine
    self.machine = Machine(model=self, states=states, initial='Initialization')

    # Acquire parameters from launch file or command line
    # 1. UGV model
    self.ugv1_model = rospy.get_param('~ugv1_model', "Bunker_Pro")
    # 2. Robot model
    self.robot1_model = rospy.get_param('~robot1_model', "UR10")
    # 3. Robot ip address
    # self.robot1_ip = rospy.get_param('~robot1_ip', '192.168.1.203')
    # 4. Color Camera model
    self.color_camera1_model = rospy.get_param('~color_camera1_model', 'd435')
    # 5. Depth Camera model
    self.depth_camera1_model = rospy.get_param('~depth_camera1_model', 'd435')
    # 6. Joint angles of the robot arm view pose
    self.view_joints = rospy.get_param('~view_joints', VIEW_JOINTS)
    # 7. Distance from the CHS
    self.distance = rospy.get_param('~distance', '1.30')
    print('Parameters acquired.')

    # Instantiate 1 UGV first
    self.ugv1 = UGV(model=self.ugv1_model)

    # Equip the UGV with necessary parts
    # 1. A robot arm
    self.robot1 = Robot(model=self.robot1_model)
    # 2. A colour camera
    self.color_camera1 = ColorCamera(model=self.color_camera1_model)
    # 3. A depth camera
    self.depth_camera1 = DepthCamera(model=self.depth_camera1_model)
    # Then put the colour camera on the robot arm
    self.robot1.add_color_camera(self.color_camera1)
    # Then put the colour camera on the robot arm
    self.robot1.add_depth_camera(self.depth_camera1)
    # Finally put the robot arm on the UGV
    self.ugv1.set_robot(self.robot1)

    # Add the first UGV to the welding system
    self.add_ugv(self.ugv1)
    print('Parts assembled.')

    # Add transitions between states
    self.machine.add_transition(trigger='start_moving' , source='Initialization' , dest='UGVMovement')
    '''
    self.machine.add_transition()
    self.machine.add_transition()
    self.machine.add_transition()
    self.machine.add_transition()
    self.machine.add_transition()
    self.machine.add_transition()
    self.machine.add_transition()
    '''

  def start_moving(self, distance):
    self.ask_user_confirm_ugv_move_pub.publish('ask_user_confirm_ugv_move')
    print('********************** In start_moving *********************')
    self.ugv1.move_to_target(distance)

  def add_ugv(self, ugv):
    self.ugvs.append(ugv)

  def main_loop(self):
    # Main application logic here
    print('*********************** Just entered main loop *******************')

    # Before doing anything, make sure the initialization is completed.
    # while not rospy.is_shutdown():
    if self.state == 'Initialization':
      self.start_moving(distance=0.28)
      # Needs confirmation from user to carry on
      # 
      # rospy.sleep(.01)

if __name__ == '__main__':
    print('Entered into the System.')
    welding_system = WeldingSystem()
    
    welding_system.main_loop()
