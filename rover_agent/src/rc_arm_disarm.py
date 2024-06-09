#!/usr/bin/env python3

"""
rc_arm_disarm.py
E. Aaltonen 2024

Read remote control switches C & D to control FCU flight mode (C) and arming/disarming (D)

This node reads RC switch states from autopilot/swc and autopilot/swd and calls flight mode service
/mavros/set_mode to set MANUAL mode (switch C up) or AUTO mode (switch D middle) 
and arming command service /mavros/cmd/arming to arm (switch D down) or disarm (switch D up) the FCU.

Requirements:
- CAN up (sudo ip link set can0 up type can bitrate 500000)
- bunker_bringup running (bunker_minimal.launch) (modified code)
- MAVROS node running (/mavros/mavros_node)
- rc_state_messenger node running (publisher for autopilot/swc and autopilot/swd)

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Int8
from mavros_msgs.srv import CommandBool, SetMode
from src.service_clients import SetModeClient, ArmingClient

# int literals - switch positions
SW_UP = 0
SW_MIDDLE = 1
SW_DOWN = 2


class RCArming(Node):
    def __init__(self):
        super().__init__("rc_arm_disarm")
        self.sub_swc = self.create_subscription(UInt8, "/rover_agent/swc", self.callback_update_swc, 10)
        self.sub_swd = self.create_subscription(UInt8, "/rover_agent/swd", self.callback_update_swd, 10)

        self.get_logger().info("> Subscriber created: arm/disarm")

        self.swc = 0
        self.swd = 0
        self.declare_parameter("opt_mode", value="GUIDED")
        self.opt_mode = self.get_parameter("opt_mode").get_parameter_value().string_value

    # Read Switch C value
    def callback_update_swc(self, msg):

        mode_client = SetModeClient()

        #Check if switch C state has changed
        if msg.data != self.swc:
            self.swc = msg.data
            if msg.data == SW_UP: # 2 = switch up - call manual
                
                try:
                    #self.get_logger().info("Try manual")
                    result = mode_client.send_request(custom_mode='MANUAL') #return true or false
                except Exception as e:
                        self.get_logger().error("Set mode service call failed: %s"%e)
                
            if msg.data == SW_MIDDLE: # 1 = switch middle - call auto
                
                try:
                    #self.get_logger().info("Try auto")
                    result = mode_client.send_request(custom_mode='AUTO') #return true or false
                except Exception as e:
                        self.get_logger().error("Set mode service call failed: %s"%e)
                    
            if msg.data == SW_DOWN and self.opt_mode != "":
                
                try:
                    #self.get_logger().info("Try opt")
                    result = mode_client.send_request(custom_mode=self.opt_mode) #return true or false
                    if not result: # if an invalid mode name is given in the param
                        self.get_logger().warn("Invalid aux mode name: {0}".format(self.apt_mode))
                except Exception as e:
                        self.get_logger().error("Set mode service call failed: %s"%e)

            mode_client.destroy_node()

    # Read Switch D value
    def callback_update_swd(self, msg):
        arming_client = ArmingClient()
        #Check if switch D state has changed
        if msg != self.swd:
            self.swd = msg.data
            if msg.data == SW_DOWN: # 3 = switch down call arm
                
                try:
                    result = arming_client.send_request(True)
                    self.get_logger().info("Arming service call successful")
                except Exception as e:
                        self.get_logger().error("Arming service call failed: %s"%e)
                
            if msg.data == SW_UP: # call disarm
               
                try:
                    result = arming_client.send_request(False)
                    self.get_logger().info("Disarming service call successful")
                except Exception as e:
                        self.get_logger().error("Disarming service call failed: %s"%e)
        arming_client.destroy_node()
    
def main(args=None):
    rclpy.init(args=args)
    node = RCArming()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


