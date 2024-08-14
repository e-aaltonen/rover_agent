#!/usr/bin/env python3
"""
offset_mission.py
E. Aaltonen 2024

This node performs a service call equivalent to command
    ros2 service call /rover_agent/mission_server rover_agent_msgs/srv/MissionManip "{task: OFFSET_MISSION, all_wps: True, distance: x.x, direction_angle: y.y}",
    x.x = distance in metres (float32), y.y = direction as compass heading (float32)

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
        super().__init__("offset_mission")
        self.get_logger().info("> calling /rover_agent/mission_server, task=5 [OFFSET_MISSION]")
        
        self.get_logger().info("This executable takes 2 command-line arguments: x.x (float) and y.y (float). Usage:")
        self.get_logger().info("ros2 run rover_agent offset_mission x.x y.y")
        self.get_logger().info("where x.x = distance (m), y.y = direction (compass heading in degrees, 0.0-360.0)")
        
        if len(sys.argv)>2:
            dist=0.0
            direction=0.0

            try:
                dist = float(sys.argv[1])
                direction = float(sys.argv[2])
                self.get_logger().error("Offsetting all mission waypoints by {0} metres in direction {1}".format(dist, direction))

            except Exception as e:
                self.get_logger().error("Arguments must be float values. %s"%e)

            mission_client = MissionManipClient()

            try:
                result = mission_client.send_request(task = OFFSET_MISSION, all_wps=True, distance=dist, direction_angle=direction)
                self.get_logger().info("All waypoints were relocated.")
            except Exception as e:
                self.get_logger().error("Service call OFFSET_MISSION failed: %s"%e)
            
        mission_client.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WPmanip()

if __name__ == '__main__':
    main()