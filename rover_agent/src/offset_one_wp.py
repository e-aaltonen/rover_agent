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
SCALE_MISSION = 6
ROTATE_MISSION = 7

class WPmanip(Node):
    def __init__(self):
        super().__init__("offset_one_wp")
        self.get_logger().info("> calling /rover_agent/mission_server, task=5 [OFFSET_MISSION]")
        
        self.get_logger().info("This executable takes 3 command-line arguments: x (integer), y.y (float) and z.z (float). Usage:")
        self.get_logger().info("ros2 run rover_agent offset_mission x y.y z.z")
        self.get_logger().info("where x = waypoint no., y.y = distance (m), z.z = direction (compass heading in degrees, 0.0-360.0)")
        
        if len(sys.argv)>3:
            wp=0
            dist=0.0
            direction=0.0

            try:
                wp = int(sys.argv[1])
                dist = float(sys.argv[2])
                direction = float(sys.argv[3])
                self.get_logger().error("Offsetting waypoint #{0} by {1} metres in direction {2}".format(wp, dist, direction))

            except Exception as e:
                self.get_logger().error("Arguments must be of type int, float, float. %s"%e)

            mission_client = MissionManipClient()

            try:
                result = mission_client.send_request(task = OFFSET_MISSION, all_wps=False, seq=wp, distance=dist, direction_angle=direction)
                self.get_logger().info("Waypoint {0} was relocated.".format(wp))
            except Exception as e:
                self.get_logger().error("Service call OFFSET_MISSION failed: %s"%e)
            
            mission_client.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WPmanip()

if __name__ == '__main__':
    main()