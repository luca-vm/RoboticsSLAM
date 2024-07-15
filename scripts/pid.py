#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
import math

class PID(object):
    def __init__(self):
        self.kp = 0.5  # Proportional gain
        self.ki = 0.0  # Integral gain
        self.kd = 0.0  # Derivative gain
        
        self.error = 0  # Initialize error
        self.integral_error = 0  # Initialize integral
        self.error_last = 0  # Initialize previous error
        self.derivative_error = 0  # Initialize derivative
        self.output = 0
        self.ang = np.zeros(3)

        self.ku = 0.0  # Ultimate gain
        self.tu = 0.0  # Oscillation period

    def calculate_target_yaw(self, current_x, current_y, goal_x, goal_y):
        dx = goal_x - current_x
        dy = goal_y - current_y
        return math.atan2(dy, dx)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_state(self, pose_msg):
        """
        Get the robot's current state from the AMCL pose message.
        """
        pose = pose_msg.pose.pose
        orientation_q = pose.orientation  # Quaternion coordinates
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]  # Store quaternion in a list
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)  # Euler coordinates
        
        return pose.position.x, pose.position.y, yaw

    def compute_pid(self, error):
        self.error = error
        self.integral_error += self.error  # Integral is the accumulated sum
        self.derivative_error = self.error - self.error_last  # Derivative is error - previous error
        self.error_last = self.error  # Update previous error
        self.output = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error  # Calculate the PID value
        return self.output

    def compute_pid_angular(self, current_yaw, target_yaw):
        self.error = self.normalize_angle(target_yaw - current_yaw)  # Error is goal - current
        
        # Normalize the error to be between -pi and pi
        self.error = self.normalize_angle(self.error)
        
        self.integral_error += self.error  # Integral is the cumulative sum
        self.derivative_error = self.error - self.error_last  # Derivative is error - previous error
        self.error_last = self.error  # Update the previous error
        self.output = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error  # Calculate PID
        return self.output

    def set_ku_tu(self, ku, tu):
        self.ku = ku
        self.tu = tu

    def ziegler_nichols_tuning(self):
        self.kp = 0.6 * self.ku
        self.ki = 1.2 * self.ku / self.tu
        self.kd = 3 * self.ku * self.tu / 40