#!/usr/bin/env python

import rospy

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.prev_error = error
        return output

if __name__ == '__main__':
    rospy.init_node('pid_controller_node')
    pid = PIDController(1.0, 0.0, 0.0)
    # TODO: Set up ROS subscribers, publishers, and main loop
