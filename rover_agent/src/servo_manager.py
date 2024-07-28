#!/usr/bin/env python3
"""
servo_manager.py
E. Aaltonen 2024

Subscribe to /rover_agent/var_a and /rover_agent/button and manage the state of 3 servos accordingly.
Servo PWM signals are set in parameters SERVOn_TRIM (to be used by MAVROS & ArduRover as output for Pixhawk RCOUT ports)

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Int8

from rcl_interfaces.msg import Parameter
from src.service_clients import ParamSetClient
# int literals - switch positions
SW_UP = 0
SW_MIDDLE = 1
SW_DOWN = 2

F_PARAM = "SERVO6_TRIM"
R_PARAM = "SERVO7_TRIM"
SPEED_PARAM = "SERVO8_TRIM"

   
class ServoMngr(Node):
    def __init__(self):
        super().__init__("servo_manager")
        
        self.get_logger().info("> Initiating servo_manager")
        
        self.sub_var_a = self.create_subscription(Int8, "/rover_agent/var_a", self.cb_var_a, 10)
        self.sub_button = self.create_subscription(UInt8, "/rover_agent/button", self.cb_button, 10)

        
        self.var_a = 0
        self.button = 0
        self.front_lock = False
        self.f_pwm = [ 1900, 1100 ]
        self.rear_lock = False
        self.r_pwm = [ 1100, 1900 ]
        self.speed_high = False
        self.sp_pwm = [ 1100, 1900 ]

    # Read button value
    def cb_button(self, msg):

        #Check if button state has changed
        if msg.data != self.button:
            self.button = msg.data
            if msg.data == SW_DOWN:
                # left knob turned left (down): toggle rear diff
                if self.var_a < -25:
                    self.rear_lock = not self.rear_lock
                    self.set_servos(R_PARAM, self.r_pwm[self.rear_lock])
                
                # left knob turned right (up): toggle front diff
                if self.var_a > 25: 
                    self.front_lock = not self.front_lock
                    self.set_servos(F_PARAM, self.f_pwm[self.front_lock])
                
                # left knob in the middle: toggle speed
                if (self.var_a > -26) & (self.var_a < 26): 
                    self.speed_high = not self.speed_high
                    self.set_servos(SPEED_PARAM, self.sp_pwm[self.speed_high])
               
    def set_servos(self, param_id, param_value):
        paramList = [Parameter()]
        paramList[0].name = param_id
        paramList[0].value.type = 2
        paramList[0].value.integer_value = param_value
        param_client = ParamSetClient("/mavros/param/set_parameters")

        try:
            result = param_client.send_request(paramList)
            if result.results[0].successful:
                self.get_logger().info("Sent value {0} to param {1}".format(param_value, param_id))
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

        param_client.destroy_node()
                
    # Read left knob value
    def cb_var_a(self, msg): 
        self.var_a = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = ServoMngr()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()