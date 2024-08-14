#!/usr/bin/env python3
"""
offset_mission.py
E. Aaltonen 2024

This node performs a service call equivalent to command
    ros2 service call /rover_agent/mission_server rover_agent_msgs/srv/MissionManip "{task: SCALE_ROTATE_MISSION, scale_factor: x.x}",
    x.x = scale size factor (1.0 = original size) (float32)

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Int8
from rover_agent_msgs.srv import MissionManip
from src.service_clients import MissionManipClient
import sys

# MissionManip.srv literals for task field (int8)
SET_HOME = 0
ADD_WP = 1
REMOVE_WP = 2
CLEAR_MISSION = 3
BACKWARDS_MISSION = 4
OFFSET_MISSION = 5
SCALE_ROTATE_MISSION = 6
MIRROR_MISSION = 7

class WPmanip(Node):
    def __init__(self):
        super().__init__("scale_mission")
        self.get_logger().info("> calling /rover_agent/mission_server, task=6 [SCALE_ROTATE_MISSION]")
        
        self.get_logger().info("This executable takes 1 command-line argument: x.x (float). Usage:")
        self.get_logger().info("ros2 run rover_agent scale_mission x.x")
        self.get_logger().info("where x.x = scale factor (1.0 = original size)")
        
        scale_f=1.0

        if len(sys.argv)>1:
            rotation = float(sys.argv[1])
            try:
                rotation = float(sys.argv[1])
                self.get_logger().info("Scaling mission to {0} * original size".format(scale_f))

            except Exception as e:
                    self.get_logger().error("Argument must be of type float. %s"%e)

            mission_client = MissionManipClient()

            try:
                result = mission_client.send_request(task = SCALE_ROTATE_MISSION, scale_factor=scale_f)
                self.get_logger().info("Mission was rotated by {0} dgr, scaled to size {1}.".format(scale_f))
            except Exception as e:
                self.get_logger().error("Service call SCALE_ROTATE_MISSION failed: %s"%e)
            
        mission_client.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WPmanip()

if __name__ == '__main__':
    main()