#!/usr/bin/env python3
"""
rc_state_messenger.py
E. Aaltonen 2024

Read remote control data feed and publish messages at switch state change
Also publish stick values [-100,100] and switch values [0,100] in topic /autopilot/channels (for GUI)

This node subscribes to /bunker_rc_status (cf. BunkerRCState.msg) published by bunker_messenger.cpp
(uint8 sws, int8 var_a), cf. Bunker Pro user manual, p. 10:

uint8 sws, switches: CAN frame 0x241, byte [0]:
bit[0-1]: SWA 2 = up, 3 = down
bit[2-3]: SWB 2 = up, 1 = middle, 3 = down
bit[4-5]: SWC 2 = up, 1 = middle, 3 = down
bit[6-7]: SWD 2 = up, 3 = down

int8 var_a "left knob": CAN frame 0x241, byte[5]:
range [-100,100]: -100 = left/down limit, 0 = middle, 100 = right/up limit

When a value changes, the node publishes the corresponding new state once in topic /autopilot/swa, swb, swc or swd
in UInt8 format, 0 = up, 1 = middle, 2 = down, and in topic /autopilot/var_a, Int8 -100 ... 100

Requirements:
- CAN up (sudo ip link set can0 up type can bitrate 500000)
- bunker_bringup running (bunker_robot_base.launch) (modified bunker_ros pkg)

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from ros2topic.api import get_msg_class
from std_msgs.msg import UInt8, Int8
from rover_agent_msgs.msg import RCchannels
from rcl_interfaces.srv import GetParameters, SetParameters
from bunker_msgs.msg import BunkerRCState
from scout_msgs.msg import ScoutRCState
from rover_agent_msgs.msg import RCchannels


# int literals for switch positions
SW_UP = 0
SW_MIDDLE = 1
SW_DOWN = 2

class ParamGetClient(Node):
    def __init__(self):
        super().__init__("paramget_client_async")
        self.cli = self.create_client(GetParameters, "/mavros/param/get_parameters")
        while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for service...")
        self.req = GetParameters.Request()
    def send_request(self, pnames):
        self.req.names = pnames
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class SWMessenger(Node):
    def __init__(self):
        super().__init__("rc_state_messenger")
        self.get_logger().info("> RC switches messenger 3")
        
        self.pub_swa = self.create_publisher(UInt8, "/rover_agent/swa", 1)
        self.pub_swb = self.create_publisher(UInt8, "/rover_agent/swb", 1)
        self.pub_swc = self.create_publisher(UInt8, "/rover_agent/swc", 1)
        self.pub_swd = self.create_publisher(UInt8, "/rover_agent/swd", 1)
        self.pub_var_a = self.create_publisher(Int8, "/rover_agent/var_a", 1)
        self.pub_channels = self.create_publisher(RCchannels, "/rover_agent/channels", 1)
        self.channel_msg = RCchannels()
        self.channel_msg.button = 0
        self.pub_timer_ = self.create_timer(0.04, self.send_channels)

        self.last_swa = 0
        self.last_swb = 0
        self.last_swc = 0
        self.last_swd = 0
        self.last_var_a = 0

        """self.get_logger().info("nyt QoS")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1            
        )"""
        
        self.declare_parameter("/rover_agent/rc_status_topic", value="/bunker_rc_status")
        self._rc_topic = self.get_parameter("/rover_agent/rc_status_topic").get_parameter_value().string_value       
        
        self.sub_rc = self.create_subscription(BunkerRCState, "/bunker_rc_status", self.callback_rc_status, 10)
        self.sub_rc2 = self.create_subscription(ScoutRCState, "/scout_rc_status", self.callback_rc_status, 10)
        
    def callback_rc_status(self, msg):
        self.get_logger().info("{0}".format(msg.swa));
        self.channel_msg.right_x = msg.stick_right_h
        self.channel_msg.right_y = msg.stick_right_v
        self.channel_msg.left_x = msg.stick_left_h
        self.channel_msg.left_y = msg.stick_left_v
        self.channel_msg.var_a = msg.var_a

        # Check if any of the switches have changed and publish in topics autopilot/switch/a, b, c or d
        # Only publish if switch value is 0, 1 or 2 (= eliminate rubbish values during 2-pos. switch transition)
        if msg.swa != self.last_swa and msg.swa in range(3):
            self.pub_swa.publish(msg.swa)
            self.last_swa = msg.swa

        if msg.swb != self.last_swb and msg.swb in range(3):
            self.pub_swb.publish(msg.swb)
            self.last_swb = msg.swb

        if msg.swc != self.last_swc and msg.swc in range(3):
            self.pub_swc.publish(msg.swc)
            self.last_swc = msg.swc

        if msg.swd != self.last_swd and msg.swd in range(3):
            self.pub_swd.publish(msg.swd)
            self.last_swd = msg.swd

        # Check if Left knob has changed and publish in topic autopilot/switch/var_a
        if msg.var_a != self.last_var_a:
            self.pub_var_a.publish(self.last_var_a)  
            self.last_var_a = msg.var_a
        
        self.channel_msg.swa = self.sw_to_chan(msg.swa)
        self.channel_msg.swb = self.sw_to_chan(msg.swb)
        self.channel_msg.swc = self.sw_to_chan(msg.swc)
        self.channel_msg.swd = self.sw_to_chan(msg.swd)
                        
            
    def sw_to_chan(self, pos):
        chan = 0
        if pos == SW_MIDDLE: chan = 50
        if pos == SW_DOWN: chan = 100
        return chan

    def send_channels(self):
        self.pub_channels.publish(self.channel_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = SWMessenger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()