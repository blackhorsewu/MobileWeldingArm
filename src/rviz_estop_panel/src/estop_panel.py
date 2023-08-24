#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
  Chinese National Engineering Research Centre for Steel Structure
                        (Hong Kong Branch)

  Hong Kong Polytechnic University
'''
'''
  Author: Victor W H Wu
  Date: 17 August 2023.

  File name: estop_panel.py

  Description:
    This python script implements the EStopPanel Class.

  It requires:
  1. OpenCV
  2. cv_bridge in ROS
  3. The Intrinsic camera matrix and the Distortion parameters of the RealSense
     D435 camera 

'''
import os
import rospy
from rviz import bindings as rviz
from std_msgs.msg import Bool
from python_qt_binding import loadUi
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

from python_qt_binding.QtWidgets import QPushButton, QVBoxLayout

class EStopPanel(rviz.PanelDockWidget):
    def __init__(self, context):
        super(EStopPanel, self).__init__(context)

        # Create a publisher for the E-Stop message
        self.estop_pub = rospy.Publisher('estop', Bool, queue_size=1)

        # Create a button for the E-Stop
        self.estop_button = QPushButton('Emergency Stop')
        self.estop_button.clicked.connect(self.onEStopButtonClicked)

        # Create a layout and add the button
        layout = QVBoxLayout()
        layout.addWidget(self.estop_button)
        self.setLayout(layout)

    def onEStopButtonClicked(self):
        msg = Bool()
        msg.data = True
        self.estop_pub.publish(msg)
