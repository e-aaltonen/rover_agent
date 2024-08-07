#!/usr/bin/env python3
"""
offset_mission.py
E. Aaltonen 2024

This node performs a service call equivalent to command
    ros2 service call /rover_agent/mission_server rover_agent_msgs/srv/MissionManip "{task: MIRROR_MISSION, direction_angle: x.x}",
    x.x = angle for the axis of symmetry in degrees (0.0 = N-S, 90.0 = E-W) (float32)

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
        super().__init__("mirror_mission")
        self.get_logger().info("> calling /rover_agent/mission_server, task=7 [MIRROR_MISSION]")
        
        self.get_logger().info("This executable takes 1 command-line argument: x.x (float). Usage:")
        self.get_logger().info("ros2 run rover_agent mirror_mission x.x")
        self.get_logger().info("where x.x = angle for the axis of symmetry in degrees (0.0 = N-S, 90.0 = E-W) (float32)")
        
        if len(sys.argv)>1:
            direction=0.0

            try:
                direction = float(sys.argv[1])
                self.get_logger().error("Mirroring the mission layout with respect to an axis of symmetry at angle of {0} degrees, passing through the centre of the mission footprint".format(direction))

            except Exception as e:
                self.get_logger().error("The argument must be of type float. %s"%e)

            mission_client = MissionManipClient()

            try:
                result = mission_client.send_request(task = MIRROR_MISSION, direction_angle=direction)
                self.get_logger().info("The mission was mirrored with a reflection angle of {0} degrees.".format(direction))
            except Exception as e:
                self.get_logger().error("Service call MIRROR_MISSION failed: %s"%e)
            
            mission_client.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WPmanip()

if __name__ == '__main__':
    main()