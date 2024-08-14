#!/usr/bin/env python3

# E. Aaltonen 2024

import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import UInt8, Int8
from mavros_msgs.msg import RCIn
from rover_agent_msgs.msg import RCchannels
from rcl_interfaces.msg import Parameter
from src.service_clients import ParamGetClient, ParamSetClient
import math

# int literals - switch positions
SW_UP = 0
SW_MIDDLE = 1
SW_DOWN = 2

class RCChannel:
    def __init__(self, chnl):
        self.minpwm = 1000
        self.maxpwm = 2000
        self.trim = 1500
        self.dz = 0
        self.channel = chnl
        rclpy.logging.get_logger("/rover_agent/rcin_messenger").info("Init RCChannel class")
        
        self.attrlist = [self.minpwm, self.maxpwm, self.trim, self.dz]
        self.paramnames = ["_MIN", "_MAX", "_TRIM", "_DZ"]
        
        param_client = ParamGetClient()
        for i in range(3):
            self.paramnames[i] = self.channel + self.paramnames[i]

        try:
            result = param_client.send_request(self.paramnames)
        except Exception as e:
            rclpy.logging.get_logger("/rover_agent/rcin_messenger").error("Service call failed: %r" % (e,))
        for i in range(3):
            self.attrlist[i] = result.values[i].integer_value
            rclpy.logging.get_logger("/rover_agent/rcin_messenger").info("Received param {0} - value {1}".format(self.paramnames[i], result.values[i].integer_value))

        param_client.destroy_node()
    
class RCInMessenger(Node):
    def __init__(self):
        super().__init__("rcin_messenger")
        self.channel_msg = RCchannels()
        self.get_logger().info("rcin_messenger started 4")
       
        # label RC channels
        self.right_x = RCChannel("RC1")
        self.right_y = RCChannel("RC2")
        self.left_x = RCChannel("RC3")
        self.left_y = RCChannel("RC4")
        self.swa = RCChannel("RC5")
        self.swb = RCChannel("RC6")
        self.var_a = RCChannel("RC9")
        self.button = RCChannel("RC10")
                        
        self.pub_channels = self.create_publisher(RCchannels, "/rover_agent/channels", 1)
        self.pub_channels_timer_ = self.create_timer(0.04, self.send_channels)

        self.pub_swa = self.create_publisher(UInt8, "/rover_agent/swa", 1)
        self.pub_swb = self.create_publisher(UInt8, "/rover_agent/swb", 1)
        self.pub_swc = self.create_publisher(UInt8, "/rover_agent/swc", 1)
        self.pub_swd = self.create_publisher(UInt8, "/rover_agent/swd", 1)
        self.pub_var_a = self.create_publisher(Int8, "/rover_agent/var_a", 1)
        self.pub_button = self.create_publisher(UInt8, "/rover_agent/button", 1)

        self.channelsNew = [0, 0, 0, 0, 0, 0]  # PWM values 1000-2000
        self.channelsPrev = [0, 0, 0, 0, 0, 0] # PWM values 1000-2000

        self.sub_rc = self.create_subscription(RCIn, "/mavros/rc/in", self.cb_rcin, 10)
        
    def mapSwitch(self, pwm):
        swPos = SW_MIDDLE
        if pwm < 1200:
            swPos = SW_UP
        if pwm > 1800:
            swPos = SW_DOWN
        return swPos
    
    def mapVar(self, pwm):
        return int((pwm - 1500) / 5)

    def cb_rcin(self, msg):    
        if len(msg.channels) > 8:
            self.channel_msg.right_x = self.scale_pwm(msg.channels[0], self.right_x)
            self.channel_msg.right_y = self.scale_pwm(msg.channels[1], self.right_y)
            self.channel_msg.left_x = self.scale_pwm(msg.channels[3], self.left_x)
            self.channel_msg.left_y = self.scale_pwm(msg.channels[2], self.left_y)
            self.channel_msg.swa = self.scale_pwm(msg.channels[4], self.swa)
            self.channel_msg.swb = self.scale_pwm(msg.channels[5], self.swb)
            self.channel_msg.var_a = self.scale_pwm(msg.channels[8], self.var_a)
            self.channel_msg.button = self.scale_pwm(msg.channels[9], self.button)

            self.channelsNew[0] = self.mapSwitch(msg.channels[4])   # SWA
            self.channelsNew[1] = self.mapSwitch(msg.channels[5])   # SWB
            self.channelsNew[2] = self.mapSwitch(msg.channels[7])   # SWC
            self.channelsNew[3] = self.mapSwitch(msg.channels[6])   # SWD
            self.channelsNew[4] = self.mapVar(msg.channels[8])  # var_a
            self.channelsNew[5] = self.mapSwitch(msg.channels[9])  # button
        
    
        #Check if any of the switches have changed and publish in topics /switch/a, b, c, d, var_a, button
        if self.channelsNew != self.channelsPrev:
            swmsg = UInt8()
            if self.channelsNew[0] != self.channelsPrev[0]:
                swmsg.data = self.channelsNew[0]
                self.pub_swa.publish(swmsg)
            if self.channelsNew[1] != self.channelsPrev[1]:
                swmsg.data = self.channelsNew[1]
                self.pub_swb.publish(swmsg)
                self.set_servos(self.channelsNew[1])
            if self.channelsNew[2] != self.channelsPrev[2]:
                swmsg.data = self.channelsNew[2]
                self.pub_swc.publish(swmsg)
            if self.channelsNew[3] != self.channelsPrev[3]:
                swmsg.data = self.channelsNew[3]
                self.pub_swd.publish(swmsg)
            if self.channelsNew[4] != self.channelsPrev[4]:
                varmsg = Int8()
                varmsg.data = self.channelsNew[4]
                self.pub_var_a.publish(varmsg)
            if self.channelsNew[5] != self.channelsPrev[5]:
                swmsg.data = self.channelsNew[5]
                self.pub_button.publish(swmsg)
            for x in range(6):
                self.channelsPrev[x] = self.channelsNew[x]

    # If switch (B) is down, disable throttle & steering servo output to enable GUI; else enable servos
    def set_servos(self, sw):
        #paramService = rospy.ServiceProxy('mavros/param/set', ParamSet)
        disable = 0
        enable_steering = 26
        enable_throttle = 70
        param_id_1 = "SERVO1_FUNCTION"
        param_id_3 = "SERVO3_FUNCTION"
                
        # disable
        paramList = [Parameter(), Parameter()]
        paramList[0].name = param_id_1
        paramList[0].value.type = 2
        paramList[1].name = param_id_3
        paramList[1].value.type = 2
        param_client = ParamSetClient("/mavros/param/set_parameters")
        if sw == SW_DOWN:
            paramList[0].value.integer_value = disable
            paramList[1].value.integer_value = disable
            
            try:
                result = param_client.send_request(paramList)
                if result.results[0].successful:
                    self.get_logger().info("Disabled Servo 1")
                if result.results[1].successful:
                    self.get_logger().info("Disabled Servo 3")
            except Exception as e:
                self.get_logger().error("Service call failed: %r" % (e,))
            
        else:
            paramList[0].value.integer_value = enable_throttle
            paramList[1].value.integer_value = enable_steering
            try:
                result = param_client.send_request(paramList)
                if result.results[0].successful:
                    self.get_logger().info("Enabled Servo 1")
                if result.results[1].successful:
                    self.get_logger().info("Enabled Servo 3")
            except Exception as e:
                self.get_logger().error("Service call failed: %r" % (e,))
            
        param_client.destroy_node()

    def scale_pwm(self, inpwm, rcChan):
        #rospy.loginfo("inpwm: {0}, rcChan.maxpwm: {1}, rcChan.minpwm: {2}, rcChan.trim: {3}".format(inpwm, rcChan.maxpwm, rcChan.minpwm, rcChan.trim))
        inpwm = min(inpwm, rcChan.maxpwm)
        inpwm = max(inpwm, rcChan.minpwm)
        if inpwm > rcChan.trim:
            rate = (inpwm - rcChan.trim) / (rcChan.maxpwm - rcChan.trim) * 100
        else:
            rate = (inpwm - rcChan.trim) / (rcChan.trim - rcChan.minpwm) * 100
        if math.fabs(rate - rcChan.trim) < rcChan.dz:
            rate = 0
        #rospy.loginfo("RC {0}: {1} -> {2}".format(rcChan.channel, inpwm, rate))
        return int(rate)

    def send_channels(self):
        self.pub_channels.publish(self.channel_msg)
            
def main(args=None):
    rclpy.init(args=args)
    node = RCInMessenger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()