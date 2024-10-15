#! /usr/bin/env python3

import numpy
import rclpy
import math
from rclpy.node import Node


from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

class HeadingController(BaseHeadingController):
    def __init__(self):
        super().__init__()
        self.kp = 2.0
        
    def compute_control_with_goal(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        err = wrap_angle(goal.theta - state.theta)

        angle_velo = self.kp * err
        
        control_message = TurtleBotControl()
        control_message.omega = angle_velo

        return control_message
    

if __name__ == "__main__":
    # Initialize the ROS2 Python library
    rclpy.init()

    # Create an instance of the HeadingController
    heading_controller = HeadingController()
    rclpy.spin(heading_controller)
    rclpy.shutdown()

 

