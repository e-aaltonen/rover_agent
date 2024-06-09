#!/usr/bin/env python3

"""
rcout_to_cmd_vel.py
E. Aaltonen 2024

Convert skid steering servo signal to Twist messages

This node reads FCU skid steering throttle output from /mavros/rc/out and publishes corresponding Twist values
to /cmd_vel.
Channel[0]: throttle left  (SERVO1_FUNCTION = 73)
Channel[2]: throttle right (SERVO3_FUNCTION = 74)

Requirements:
- CAN up (sudo ip link set can0 up type can bitrate 500000)
- bunker_base
- MAVROS node running (/mavros/mavros_node)

Default parameter values:
- /rover_agent/speed_factor_lin_x: "1.0"   Forward movement rate under FCU control
- /rover_agent/speed_factor_ang_z: "1.0"   Turning rate under FCU control
- /rover_agent/vel_topic: "/cmd_vel"       Defalt velocity topic
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from mavros_msgs.msg import RCOut

class RCOutInput(Node):
    def __init__(self):
        super().__init__("rcout_to_cmd_vel")
        #self._loop_rate = self.create_rate(50, self.get_clock())

        self.sub_rc = self.create_subscription(RCOut, "/mavros/rc/out", self.update_throttle, 10)
        self.get_logger().info("> Subscriber for /mavros/rc/out created")
       
        self._pwm_min = 900
        self._pwm_max = 2100
       
        self.declare_parameter("/rover_agent/speed_factor_lin_x", value=1.0)
        self._speed_factor_lin_x = self.get_parameter("/rover_agent/speed_factor_lin_x").get_parameter_value().double_value
        #self.get_logger().info("{0}".format(self._speed_factor_lin_x))
        self.declare_parameter("/rover_agent/speed_factor_ang_z", value=1.0)
        self._speed_factor_ang_z = self.get_parameter("/rover_agent/speed_factor_ang_z").get_parameter_value().double_value

        # set default topic for velocity commands
        self.declare_parameter("/rover_agent/vel_topic", value="/cmd_vel")
        self._vel_topic = self.get_parameter("/rover_agent/vel_topic").get_parameter_value().string_value

        self.pub_twist = self.create_publisher(Twist, self._vel_topic, 1)
        self.pub_timer_ = self.create_timer(0.02, self.send_vel_msg)
 
        self.twist_msg = Twist()

    def update_throttle(self, msg):
        self.twist_msg.linear.x = self.pwm_to_adimensional((msg.channels[0] + msg.channels[2]) * 0.5) * self._speed_factor_lin_x
        self.twist_msg.angular.z = self.pwm_to_adimensional(msg.channels[2]) - self.pwm_to_adimensional(msg.channels[0]) * self._speed_factor_ang_z

    def pwm_to_adimensional(self, pwm):
        pwm = max(pwm, self._pwm_min)
        pwm = min(pwm, self._pwm_max)
        pwm = pwm - ((self._pwm_max + self._pwm_min) * 0.5)
        pwm = pwm / ((self._pwm_max - self._pwm_min) * 0.5)
        return pwm

    def send_vel_msg(self):
        self.pub_twist.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RCOutInput()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

