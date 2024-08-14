#!/usr/bin/env python3
"""
backwards_mission.py
E. Aaltonen 2024

This node performs a service call equivalent to command
    ros2 service call /rover_agent/mission_server rover_agent_msgs/srv/MissionManip "{task: BACKWARDS_MISSION}"

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Int8
from rover_agent_msgs.srv import MissionManip
from src.service_clients import MissionManipClient

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
        super().__init__("backwards_mission")
        self.get_logger().info("> calling /rover_agent/mission_server, task=4 [BACKWARDS_MISSION]")
        
        mission_client = MissionManipClient()

        try:
            result = mission_client.send_request(task = BACKWARDS_MISSION)
            self.get_logger().info("The mission was rearranged from end to start")
        except Exception as e:
            self.get_logger().error("Service call BACKWARDS_MISSION failed: %s"%e)
        
        mission_client.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WPmanip()

if __name__ == '__main__':
    main()