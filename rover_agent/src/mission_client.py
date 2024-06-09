#!/usr/bin/env python3
"""
mission_client.py
E. Aaltonen 5 Mar. 2024

Read remote control switch A and left knob VRA to insert/remove waypoints

This node subscribes to autopilot/swa and autopilot/var_a published by rc_state_sub-pub.py
(uint8 for autopilot/swa, int8 for autopilot/var_a) and calls service /bunker_autopilot/mission_manip
to manipulate the waypoint list.

***
Functions triggered by switch A down (at state change):
- if VRA middle: append new waypoint at the end of the list, using current position
- if VRA down (< -25): clear waypoint list
- if VRA up (> 25): remove waypoint at current_seq (waypoint currently navigated to if in AUTO mode, or last waypoint if mission finished)
***

Requirements:
- ugv_sdk and bunker_base (Agilex Robotics) packages 
    (modified versions; see https://github.com/e-aaltonen/bunker_ros_RC)
- CAN up (sudo ip link set can0 up type can bitrate 500000)
- bunker_bringup running (bunker_minimal.launch)
- rc_state_messenger node running
- MAVROS node (mavros_node) running

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Int8
from rover_agent_msgs.srv import MissionManip
from src.service_clients import MissionManipClient

# int literals - switch positions
SW_UP = 0
SW_MIDDLE = 1
SW_DOWN = 2

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
        super().__init__("mission_client")
        self.sub_swa = self.create_subscription(UInt8, "/rover_agent/swa", self.callback_update_swa, 10)
        self.sub_swb = self.create_subscription(UInt8, "/rover_agent/swb", self.callback_update_swb, 10)
        self.sub_var_a = self.create_subscription(Int8, "/rover_agent/var_a", self.callback_update_var_a, 10)

        self.get_logger().info("> mission_client node initiated")

        self.swa = 0
        self.swb = 0
        self.var_a = 0

            
    # Check whether Switch A position has changed
    def callback_update_swa(self, msg):
        if msg.data != self.swa:
            self.swa = msg.data

            if msg.data == SW_DOWN and self.swb != SW_DOWN: # 3 = switch down to activate function, unless SWB is down
                # left knob turned left (down): clear mission
                mission_client = MissionManipClient()

                if self.var_a < -25:
                    try:
                        result = mission_client.send_request(task = CLEAR_MISSION)
                    except Exception as e:
                        self.get_logger().error("Service call CLEAR_MISSION failed: %s"%e)
                
                # left knob turned right (up): remove current WP
                if self.var_a > 25: 
                    try:
                        result = mission_client.send_request(task = REMOVE_WP, use_last = False, use_current = True, seq = 0)
                    except Exception as e:
                        self.get_logger().error("Service call REMOVE_WP failed: %s"%e)
                
                # left knob in the middle: push WP
                if (self.var_a > -26) & (self.var_a < 26): 
                    # check whether GPS position was received
                    try:
                        result = mission_client.send_request(task = ADD_WP, use_last = True, use_current = False, seq = 0)
                    except Exception as e:
                        self.get_logger().error("Service call ADD_WP failed: %s"%e)
                
                mission_client.destroy_node()

    def callback_update_swb(self, msg):
        if msg.data != self.swb:
            self.swb = msg.data
                                        
    # Read left knob value
    def callback_update_var_a(self, msg): 
        self.var_a = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = WPmanip()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()