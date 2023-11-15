#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
  Chinese National Engineering Research Centre for Steel Structure
                        (Hong Kong Branch)

  Hong Kong Polytechnic University

  Author: Victor W H Wu
  Date: 24 August 2023.

  Updated on: 15 November 2023.

  File name: mobile_welding_arm.py

  Description:
    This is the top level script of the whole "mobile welding arm" system.

    Do not use the "ur_connection_manager" anymore.

'''

import rospy
from std_msgs.msg import String

import urx
from urx import Robot

from transitions import Machine

from ugv import UGV
from robot_arm import RobotArm
from colour_camera import ColourCamera

# Joint angles of the view pose for UR10
VIEW_JOINTS = [-0.0524, -1.1756, -2.6466, 0.8147, 1.5948, 0.0049]

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

    self.move_to_viewing_pose(VIEW_JOINTS)
    # pass

  def add_color_camera(self, camera):
    self.color_camera = camera

  def add_depth_camera(self, camera):
    self.depth_camera = camera

  def move_to_viewing_pose(self, pose):
    # Wait for MoveJ service
    rospy.wait_for_service('move_j')
    self.move_j = rospy.ServiceProxy('move_j', MoveJ)
    response = self.move_j(pose, 0.4, 0.4, False)

  # The pose here specifies the pose of the TCP in robot arm base
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
# End Class Robot

class WeldingPath:
  def generate_path(self):
    pass

class WeldingSystem:
  
  # Initialise the welding_system

  def __init__(self):
    print('Initiating the system.')
    self.ugvs = []
    self.view_joints = []
    
    # Initialize ROS node
    rospy.init_node('welding_system', anonymous=True)
    print('ROS initialized.')

    '''
    # Setup publishers
    self.ask_user_confirm_ugv_move_pub = rospy.Publisher('ask_user_confirm_ugv_move', String, queue_size=10)
    '''
    # Define the states
    # To start with just 3 states:
    states = ['Initialization', 'Startup', 'Error']
    
    # Initialize the state machine
    self.machine = Machine(model=self, states=states, initial='Initialization')
    
    # Acquire parameters from launch file or command line
    # 1. UGV model
    self.ugv1_model = rospy.get_param('~ugv1_model', "Bunker_Pro")
    # 2. Robot model
    self.robot1_model = rospy.get_param('~robot1_model', "UR10")
    # 3. Robot ip address
    self.robot1_ip = rospy.get_param('~robot1_ip', '192.168.1.203')
    # 4. Color Camera model
    self.colour_camera1_model = rospy.get_param('~color_camera1_model', 'd435')
    # 5. Depth Camera model
    self.depth_camera1_model = rospy.get_param('~depth_camera1_model', 'd435')
    # 6. Joint angles of the robot arm view pose
    self.view_joints = rospy.get_param('~view_joints', VIEW_JOINTS)
    # 7. Distance from the CHS
    self.distance = rospy.get_param('~distance', '1.30')
    # Acquire the parameters from launch file or command line
    # The physical side of the marker is 60cm
    self.markerLength = rospy.get_param('~markerLength', 0.06)
    self.aruco_type = rospy.get_param('~aruco_type', "DICT_6X6_100")

    print('Parameters acquired.')

    # Instantiate the components of the welding_system

    # 1. Instantiate 1 UGV first
    self.ugv1 = UGV(model=self.ugv1_model)

    # Equip the UGV with necessary parts
    # 2. Instantiate a robot arm
    self.robot1 = RobotArm(self.robot1_model, self.robot1_ip)
    
    # 3. A colour camera
    self.colour_camera1 = ColourCamera(self.colour_camera1_model, 
                                       self.markerLength, self.aruco_type)
    
    # 4. A depth camera
    self.depth_camera1 = DepthCamera(model=self.depth_camera1_model)
    # Then put the colour camera on the robot arm
    
    # self.robot1.add_color_camera(self.color_camera1)
    # Then put the colour camera on the robot arm
    # self.robot1.add_depth_camera(self.depth_camera1)
    # Finally put the robot arm on the UGV
    self.ugv1.set_robot(self.robot1)

    # There could be more than ONE UGV in the system
    # Add the first UGV to the welding system
    self.add_ugv(self.ugv1)

    # Add transitions between states
    # self.machine.add_transition(trigger='start_moving' , source='Initialization' , dest='UGVMovement')
    pass
  # __init__

  def start_moving(self, distance):
    self.ask_user_confirm_ugv_move_pub.publish('ask_user_confirm_ugv_move')
    # print('********************** In start_moving *********************')
    self.ugv1.move_to_target(distance)
    pass

  def add_ugv(self, ugv):
    self.ugvs.append(ugv)
    pass

  # Start up the system components in sequence
  def startup(self):
    # Start the robot arm to publish its joint states
    self.robot1.start()
    # Move the robot arm, holding the colour camera to a viewing position
    self.robot1.move2view(VIEW_JOINTS)
    # Start the colour camera to detect marker
    self.colour_camera1.start_marker_detection()
    pass

  def main_loop(self):
    # Main application logic here
    # print('*********************** Just entered main loop *******************')

    # Before doing anything, make sure the initialization is completed.
    # while not rospy.is_shutdown():
    if self.state == 'Initialization':
      self.startup()
      # self.start_moving(distance=0.28)
      # Needs confirmation from user to carry on
      # 
      # rospy.sleep(.01)
    pass
  # End main_loop
# End Class WeldingSystem

if __name__ == '__main__':
    print('Entered into the System.')
    welding_system = WeldingSystem()
    
    welding_system.main_loop()
