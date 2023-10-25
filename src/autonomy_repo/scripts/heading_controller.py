#!/usr/bin/env python3
import numpy as np
import rclpy

from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle

from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

# HeadingController inherits from BaseHeadingController    
class HeadingController(BaseHeadingController):
    def __init__(self) -> None:
        # Initialize base class (must happen before everything else)
        super().__init__("headingcontrol")  # Initialize base class

        # Proportional control gain
        self.kp = 2.0

    # 3.4 Proportional control
    def compute_control_with_goal(self,cur_state:TurtleBotState, desired_state:TurtleBotState) -> TurtleBotControl:
        # Calculate heading error (∈ [−π, π]) as the wrapped difference
        err = wrap_angle(desired_state.theta - cur_state.theta)
        # compute the angular velocity required for the TurtleBot to correct its heading error.
        w = self.kp * err

        # Create a new TurtleBotControl message, set its omega attribute to the computed angular velocity,
        control_msg = TurtleBotControl()
        control_msg.omega = w
        print(control_msg)
        
        return control_msg

# 3.5 Node Execution
if __name__ == "__main__":
    print("debugging1")
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = HeadingController()  # instantiate the heartbeat node
    rclpy.spin(node)  # keep it running and listening for messages.
    rclpy.shutdown()    # cleanly shutdown ROS2 context
    
