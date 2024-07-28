#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import GetParameters, SetParameters
from rover_agent_msgs.srv import MissionManip
from mavros_msgs.srv import CommandBool, SetMode

class ParamGetClient(Node):
    def __init__(self, service_name):
        super().__init__("paramget_client_async")
        self.cli = self.create_client(GetParameters, service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for service...")
        self.req = GetParameters.Request()
    def send_request(self, pnames):
        self.req.names = pnames
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class ParamSetClient(Node):
    def __init__(self, service_name):
        super().__init__("paramset_client_async")
        self.cli = self.create_client(SetParameters, service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for service...")
        self.req = SetParameters.Request()
    def send_request(self, pnames):
        self.req.parameters = pnames
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class SetModeClient(Node):
    def __init__(self):
        super().__init__("set_mode_client_async")
        self.cli = self.create_client(SetMode, "/mavros/set_mode")
        while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for service...")
        self.req = SetMode.Request()
    def send_request(self, custom_mode):
        self.req.custom_mode = custom_mode
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class ArmingClient(Node):
    def __init__(self):
        super().__init__("arming_client_async")
        self.cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for service...")
        self.req = CommandBool.Request()
    def send_request(self, bvalue):
        self.req.value = bvalue
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
        
class MissionManipClient(Node):
    def __init__(self):
        super().__init__("mission_manip_client_async")
        self.cli = self.create_client(MissionManip, "/rover_agent/mission_server")
        while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for service...")
        self.req = MissionManip.Request()
    def send_request(self, task, use_last = True, use_current = False, seq = 0, all_wps = False, distance = 0.0, direction_angle = 0.0, scale_factor = 0.0):
        self.req.task = task
        self.req.use_last = use_last
        self.req.use_current = use_current
        self.req.seq = seq
        self.req.all_wps = all_wps
        self.req.distance = distance
        self.req.direction_angle = direction_angle
        self.req.scale_factor = scale_factor
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
