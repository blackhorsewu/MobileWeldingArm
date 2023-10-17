import math
from geometry_msgs.msg import Twist

class PID:
    def __init__(self, Kp, Ki, Kd, max_output=None, min_output=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_output = max_output
        self.min_output = min_output
        self.prev_error = 0.0
        self.integral = 0.0

    def step(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        if self.max_output is not None:
            output = min(output, self.max_output)
        if self.min_output is not None:
            output = max(output, self.min_output)
        self.prev_error = error
        return output

def go_to_pose(target_pose, current_pose, linear_pid, angular_pid, dt):
    delta_x = target_pose.position.x - current_pose.position.x
    delta_y = target_pose.position.y - current_pose.position.y
    angle_to_target = math.atan2(delta_y, delta_x)
    current_orientation = current_pose.orientation
    current_yaw = math.atan2(
        2.0 * (current_orientation.w * current_orientation.z + current_orientation.x * current_orientation.y),
        1.0 - 2.0 * (current_orientation.y**2 + current_orientation.z**2)
    )
    angle_to_turn = angle_to_target - current_yaw
    angle_to_turn = (angle_to_turn + math.pi) % (2 * math.pi) - math.pi
    distance_to_target = math.sqrt(delta_x**2 + delta_y**2)

    twist_msg = Twist()
    twist_msg.linear.x = linear_pid.step(distance_to_target, dt)
    twist_msg.angular.z = angular_pid.step(angle_to_turn, dt)
    
    return twist_msg

# Create PID controllers for linear and angular control
linear_pid = PID(Kp=0.1, Ki=0.01, Kd=0.01)
angular_pid = PID(Kp=0.3, Ki=0.01, Kd=0.01)

# Example usage:
# target_pose and current_pose are assumed to be of type geometry_msgs.msg.Pose
# dt is the time step in seconds since the last call to go_to_pose
twist_msg = go_to_pose(target_pose, current_pose, linear_pid, angular_pid, dt)
cmd_vel_pub.publish(twist_msg)

'''
**************************************************************************************
'''

