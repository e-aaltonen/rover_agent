#!/usr/bin/env python3


"""
mission_server.py
E. Aaltonen 5 Mar. 2024

This service subscribes to /mavros/mission/waypoints to receive waypoint list from the FCU and to
/mavros/global_position/global to receive current GPS position.

After amending local waypoint list, the list is updated to the FCU by calling service
/mavros/mission/push. The waypoint list is cleared (waypoint list emptied except home position) by
calling /mavros/mission/clear. Calling service /mavros/mission/pull is used to verify the
current number of waypoints on the FCU. This is also necessary to update the list and to verify that
the list published in topic /mavros/mission/waypoints is actually correct.

***
Function set by the task field:

SET_HOME = 0            ()
ADD_WP = 1              (bool use_last, bool use_current, uint16 seq)
REMOVE_WP = 2           (bool use_last, bool use_current, uint16 seq)
CLEAR_MISSION = 3       ()
BACKWARDS_MISSION = 4   ()
OFFSET_MISSION = 5      (bool all_wps, uint16 seq, float32 distance, float32 direction_angle)
SCALE_MISSION = 6       (float32 scale_factor)
ROTATE_MISSION = 7      (float32 direction_angle)
***

The node also publishes relevant navigation information in topic /wp_info:
uint16 total_wps             - total number of waypoints on the list
uint16 next_wp               - current navigation index
float32 distance_to_next_wp - distance in metres
float32 target_bearing      - in degrees, 0 = north
Distances are calculated using a 2D approximation for short distances

"""


import rclpy
from rclpy.node import Node
import rclpy.node
import rclpy.parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Int8, UInt16, Float32
from rover_agent_msgs.srv import MissionManip
from mavros_msgs.msg import Waypoint, WaypointList
from mavros_msgs.srv import WaypointPush, WaypointPull, WaypointClear
from sensor_msgs.msg import NavSatFix
from rover_agent_msgs.msg import WPInfo
import math

class PushWaypoints(Node):
    def __init__(self):
        super().__init__("waypoints_push_async")
        self.cli = self.create_client(WaypointPush, "/mavros/mission/push")
        while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for service...")
        self.req = WaypointPush.Request()
    def send_request(self, start_index, waypoints):
        self.req.start_index = start_index
        self.req.waypoints = waypoints
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
class PullWaypoints(Node):
    def __init__(self):
        super().__init__("waypoints_pull_async")
        self.cli = self.create_client(WaypointPull, "/mavros/mission/pull")
        while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for service...")
        self.req = WaypointPull.Request()
    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class distVector():
    def __init__(self, dist=0.0, dxion=0.0):
        self.dist = dist
        self.dxion = dxion

class WPmanip(Node):
    def __init__(self):
        super().__init__("mission_server")
        self.serv = self.create_service(MissionManip, "/rover_agent/mission_server", self.handle_task)
        self.get_logger().info("Service mission_server initiated")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1            
        )

        self.sub_mission_wps = self.create_subscription(WaypointList, "/mavros/mission/waypoints", self.callback_mission_wps, 10)
        self.sub_global_pos = self.create_subscription(NavSatFix, "/mavros/global_position/global", self.callback_global_position, qos_profile=qos_profile)        
        self.pub_wpinfo = self.create_publisher(WPInfo, "/rover_agent/wp_info", 1)
        self.pub_wpinfo_timer = self.create_timer(0.2, self.send_wpinfo)
        self.wpinfo_msg = WPInfo()

        self.global_position = NavSatFix()        
        self.got_gp = False
        self.wp = Waypoint()
        self.wps = WaypointList()
        
        #Distance coefficients for distance approximations
        self.k_lat = 111194
        self.k_long = 50519
        self.k_set = self.has_parameter("k_lat")

         # set default topic for velocity commands
        self.declare_parameter("k_lat", value=111194)
        self.declare_parameter("k_long", value=50519)
        
        self.k_lat = self.get_parameter("k_lat").get_parameter_value().integer_value
        self.k_long = self.get_parameter("k_long").get_parameter_value().integer_value
        

        
    # Send new WP list to the FCU
    def wpPush(self): 
        wppush_client = PushWaypoints()
        try:
            result = wppush_client.send_request(start_index = 0, waypoints = self.wps.waypoints)
            if result.success:
                self.get_logger().info("Sent {0} waypoints. Number of waypoints: {1}".format(result.wp_transfered, self.wpPull()))
                return True
        except Exception as e:
            self.get_logger().error("Service call push waypoints failed: %s"%e)
            return False
        wppush_client.destroy_node()

    # Request current number of WPs
    def wpPull(self):
        wppull_client = PullWaypoints()
        try:
            wp_num = 0
            result = wppull_client.send_request()
            if result.success:
                wp_num = result.wp_received
            return wp_num
        except Exception as e:
            self.get_logger().error("Service call pull waypoints failed: %s"%e)
            return False
        wppull_client.destroy_node()

    # *** SET_HOME ***
    # Set Home at current location
    def setCurrentHome(self): 
        succ = False

        if len(self.wps.waypoints) > 0:
            self.wps.waypoints[0].x_lat = self.global_position.latitude
            self.wps.waypoints[0].y_long = self.global_position.longitude
            self.wps.waypoints[0].z_alt = self.global_position.altitude
           
            succ = self.wpPush()
        # Prompt fail message
        else:
            self.get_logger().warn("SET_HOME failed - missing global position")        
        return succ
        
    # *** ADD_WP ***
    # Add new WP as the last item on the list, using current GPS location
    def amendLocalWP(self, use_last, use_current, addSeq): 
        succ = False

        #Update local list
        if (self.global_position.latitude > 0.0): # Make sure global position data has been received
            self.wp.frame = 3 #  FRAME_GLOBAL_REL_ALT = 3
            self.wp.command = 16 # NAV_WAYPOINT = 16
            self.wp.is_current= False
            self.wp.autocontinue = True
            self.wp.param1 = 0.0 
            self.wp.param2 = 0.0
            self.wp.param3 = 0.0
            self.wp.param4 = 0.0
            self.wp.x_lat = self.global_position.latitude
            self.wp.y_long = self.global_position.longitude
            self.wp.z_alt = self.global_position.altitude

            if use_last:
                # add current position as last WP
                self.wps.waypoints.append(self.wp)
                succ = self.wpPush()
                return succ
            if use_current:
                # add current position at current seq
                self.wps.waypoints.insert(self.wps.current_seq, self.wp)
                succ = self.wpPush()
                return succ
            if addSeq < len(self.wps.waypoints):
                # add current position at indicated position (addSeq)
                self.wps.waypoints.insert(addSeq, self.wp)
                succ = self.wpPush()
        # Prompt fail message
        else:
            self.get_logger().warn("ADD_WP failed - missing global position")

        return succ        

    # *** REMOVE_WP ***
    # Remove item at current_seq from the WP list
    def removeCurrentWP(self, use_last, use_current, remSeq): 
        succ = False

        #Update local list
        if len(self.wps.waypoints) > 1:
            if use_last:
                # remove last WP
                self.wps.waypoints.pop()
                succ = self.wpPush()
                return succ
            if use_current:
                # remove WP at current_seq  
                self.wps.waypoints.pop(self.wps.current_seq)    
                succ = self.wpPush()
                return succ
            if remSeq < len(self.wps.waypoints):
                # remove WP at remSeq
                self.wps.waypoints.pop(remSeq)
                succ = self.wpPush()
        # Prompt fail message
        else:
            self.get_logger().warn("REMOVE_WP failed - waypoints list length: {0}".format(len(self.wps.waypoints)))
        return succ
                          
    # *** CLEAR_MISSION ***
    # Remove all waypoints on FCU WP list
    def clearMission(self): 
        succ = False

        # Instead of calling mavros/mission/clear service, which also clears home wp,
        # just clear local list and add current coordinates as new home
        del self.wps.waypoints[:]

        if (self.global_position.latitude > 0.0): 
            self.wp.frame = 0 #  FRAME_GLOBAL_REL_ALT = 3
            self.wp.command = 16 # NAV_WAYPOINT = 16
            self.wp.is_current = True
            self.wp.autocontinue = True
            self.wp.param1 = 0.0 
            self.wp.param2 = 0.0
            self.wp.param3 = 0.0
            self.wp.param4 = 0.0
            self.wp.x_lat = self.global_position.latitude
            self.wp.y_long = self.global_position.longitude
            self.wp.z_alt = self.global_position.altitude
            self.wps.waypoints.append(self.wp)
            self.get_logger().info("Waypoint list cleared. Current position set as new home location")
           
            succ = self.wpPush()
        # Prompt fail message
        else:
            self.get_logger().warn("CLEAR_MISSION failed - missing global position")      
        return succ

    # *** BACKWARDS_MISSION ***
    # Invert WP list after home position (excluding [0])
    def invertWPlist(self): 
        succ = False

        if(len(self.wps.waypoints) > 2):
            self.wps.waypoints[1:] = self.wps.waypoints[len(self.wps.waypoints):0:-1]
            # Process info
            self.get_logger().info("BACKWARDS_MISSION: waypoints list inverted")
        
        #Send list to the FCU
        succ = self.wpPush()
        
        return succ

    # *** OFFSET_MISSION ***
    # Add new WP as the last item on the list, using current GPS location
    def offsetWPlist(self, all_wps, offSeq, distance, direction): 
        succ = False

        if not all_wps:
            if (len(self.wps.waypoints) > offSeq):
                self.wps.waypoints[offSeq] = self.offsetWP(self.wps.waypoints[offSeq], distance, direction)
                succ = self.wpPush()
            # Prompt fail message
            else:
                self.get_logger().warn("OFFSET_MISSION failed - waypoints list length: {0} <= offSeq {1}".format(len(self.wps.waypoints), offSeq))
        
        elif (len(self.wps.waypoints) > 1):
            wp_list = WaypointList()
            wp_list.waypoints.append(self.wps.waypoints[0])
            for wpoff in self.wps.waypoints[1:]:
                wpoff = self.offsetWP(wpoff, distance, direction)
                wp_list.waypoints.append(wpoff)
            self.wps.waypoints = wp_list.waypoints
            succ = self.wpPush()

        # Prompt fail message
        else:
            self.get_logger().warn("OFFSET_MISSION failed - waypoints list length: {0}".format(len(self.wps.waypoints)))

        return succ
        
    def offsetWP(self, wpset, dist, dxion):
        if (not self.k_set) and self.got_gp:
            self.calculateCoefficients()

        #self.get_logger().info("offsetWP: lat {0} long{1}".format(wpset.x_lat, wpset.y_long))
        k_x = math.sin(math.radians(dxion))
        k_y = math.cos(math.radians(dxion))
        #self.get_logger().info("OFFSET dist: {0} dxion: {1} k_y: {2} k_x: {3}".format(dist, dxion, k_y, k_x))
        distX = dist * k_x / self.k_long
        distY = dist * k_y / self.k_lat
        #self.get_logger().info("OFFSET distY: {0} distX: {1}".format(distY, distX))
        wpset.y_long += distX
        wpset.x_lat += distY
        #self.get_logger().info("now lat {0} long{1}".format(wpset.x_lat, wpset.y_long))
        return wpset
        
    # *** SCALE_ROTATE_MISSION ***
    # Move each WP away from Home, measured by distance * sFactor, rotating it in relation to Home
    def scaleRotateMission(self, sFactor, offsetAngle):
        succ = False
        vector = distVector()
        self.get_logger().info("sFactor: {0}, offsetAngle: {1}".format(sFactor, offsetAngle))
        midpoint = self.calculateMidpoint()
        self.get_logger().info("Midpoint x_lat {0} - y_long{1}".format(midpoint.x_lat, midpoint.y_long))

        if (len(self.wps.waypoints) > 1):
            for wpoff in self.wps.waypoints[1:]:
                self.get_logger().info("WPalku - x_lat: {0}, y_long: {1}".format(wpoff.x_lat, wpoff.y_long))
            wp_list = WaypointList()
            wp_list.waypoints.append(self.wps.waypoints[0])
            
            for wpoff in self.wps.waypoints[1:]:
                #self.get_logger().info("Home - y_long: {0}, x_lat: {1}".format(self.wps.waypoints[0].y_long, self.wps.waypoints[0].x_lat))
                self.get_logger().info("WPoff - x_lat: {0}, y_long: {1}".format(wpoff.x_lat, wpoff.y_long))
                vector = self.calculateVector(midpoint, wpoff)
                self.get_logger().info("vector.dist: {0}, vector.dxion: {1}".format(vector.dist, vector.dxion))
                wpoff.x_lat = midpoint.x_lat
                wpoff.y_long = midpoint.y_long
                #self.get_logger().info("E - y_long: {0}, x_lat: {1}".format(wpoff.y_long, wpoff.x_lat))
                wpoff = self.offsetWP(wpoff, (vector.dist * sFactor), (vector.dxion + offsetAngle))
                #self.get_logger().info("J - y_long: {0}, x_lat: {1}".format(wpoff.y_long, wpoff.x_lat))
                wp_list.waypoints.append(wpoff)
                self.get_logger().info(len(wp_list.waypoints))
            self.wps.waypoints = wp_list.waypoints
            succ = self.wpPush()
        # Prompt fail message
        else:
            self.get_logger().warn("SCALE_ROTATE_MISSION failed - waypoints list length: {0}".format(len(self.wps.waypoints)))
        return succ

    # *** MIRROR_MISSION ***
    # Move each WP rotating it in relation to Home, maintaining distance
    def mirrorMission(self, mirrorAngle):
        succ = False
        oVector = distVector()
        midpoint = self.calculateMidpoint()
        
        if (len(self.wps.waypoints) > 2):
        # Determine mission mindpoint

            # mirror axle heading must be in range [-90, 90]
            while mirrorAngle > 90:
                mirrorAngle -= 90
            while mirrorAngle < -90:
                mirrorAngle += 90
            
            # direction of moving each mirrored WP
            offsetAngle = mirrorAngle + 90    
                                                            
            wp_list = WaypointList()
            wp_list.waypoints.append(self.wps.waypoints[0])

            # process each waypoint
            for wpoff in self.wps.waypoints[1:]:
                # hypotenuse of triangle ABC wpoff - midpoint - (the point where the mirror offset vector crosses the mirror axle)
                oVector = self.calculateVector(wpoff, midpoint)
                # angle BAC
                alpha = math.radians(offsetAngle - oVector.dxion)
                # side AC
                halfDist = math.cos(alpha) * oVector.dist
                # move WP to mirrored location
                wpoff = self.offsetWP(wpoff, (2 * halfDist), offsetAngle)
                wp_list.waypoints.append(wpoff)
            self.wps.waypoints = wp_list.waypoints
            succ = self.wpPush()
        # Prompt fail message
        else:
            self.get_logger().warn("MIRROR_MISSION failed - waypoints list length: {0}".format(len(self.wps.waypoints)))
        return succ

        
#    def nullF(self)
#        return False

    # *******************************************************************************************************
    # Check requested task
    def handle_task(self, req, response):
        
        response.success = False

        self.wpPull()

        # set home in current location
        if req.task == 0:
            response.success = self.setCurrentHome()

        # add WP
        if req.task == 1:
            response.success = self.amendLocalWP(req.use_last, req.use_current, req.seq)

        # remove WP
        if req.task == 2:
            response.success = self.removeCurrentWP(req.use_last, req.use_current, req.seq)

        # clear mission
        if req.task == 3:
            response.success = self.clearMission()

        # backwards mission
        if req.task == 4:
            response.success = self.invertWPlist()

        # offset mission
        if req.task == 5:
            response.success = self.offsetWPlist(req.all_wps, req.seq, req.distance, req.direction_angle)

        # scale & rotate mission
        if req.task == 6:
            response.success = self.scaleRotateMission(req.scale_factor, req.direction_angle)

        # mirror mission
        if req.task == 7:
            response.success = self.mirrorMission(req.direction_angle)

        # Request actual number of WPs from the FCU
        response.number_wps = 0
        response.number_wps = self.wpPull()
            
        # Return service response message
        return response 
    # *******************************************************************************************************

    # *** Calculation functions ***
    # Calculate distance and bearing for a distVector object
    def calculateVector(self, wpSource, wpDestination):
        resultVector = distVector()

        if (not self.k_set) and self.got_gp:
            self.calculateCoefficients()

        # here x = longitude, y = latitude
        diffy = (wpDestination.x_lat - wpSource.x_lat)
        diffx = (wpDestination.y_long - wpSource.y_long)
        
        disty = diffy * self.k_lat
        distx = diffx * self.k_long
        resultVector.dist = math.sqrt(disty**2 + distx**2)
               
        t_bearing = 0
        try:
            if resultVector.dist > 0:
                theta = math.acos(disty/resultVector.dist)
                t_bearing = math.degrees(theta)

                if distx < 0:
                    t_bearing = 360 - t_bearing
            resultVector.dist = round(resultVector.dist,2)

            resultVector.dxion = t_bearing
        except Exception as e:
            self.get_logger().error("Direction calculation failed: %s"%e)
                
        return resultVector
        
    # Calculate good-enough approximates
    def calculateCoefficients(self):
        # adjusted equatorial diameter to compensate differences between the geoid and a sphere, for  lat ~63
        diagEq = 12793172 
        self.k_lat = math.pi * diagEq / 360
        try:
            self.k_long = math.pi * math.cos(math.radians(self.global_position.latitude)) * diagEq / 360
        except Exception as e:
            self.get_logger().error("Calculating k_long: %s"%e)

        self.k_set = True

        self.k_lat = self.get_parameter("k_lat").get_parameter_value().integer_value
        self.k_long = self.get_parameter("k_long").get_parameter_value().integer_value

        param_k_lat = rclpy.parameter.Parameter(
            'k_lat',
            rclpy.Parameter.Type.INTEGER,
            self.k_lat
        )
        param_k_long = rclpy.parameter.Parameter(
            'k_long',
            rclpy.Parameter.Type.INTEGER,
            self.k_long
        )
        #param_list = [param_k_lat, param_k_long]
        self.set_parameters([param_k_lat, param_k_long])
                    
    # Calculate distance & bearing to next WP
    def distanceNextWP(self):
        if (not self.k_set) and self.got_gp:
            self.calculateCoefficients()

        nextwp = 1
        if (self.wps.current_seq < len(self.wps.waypoints)) and (self.wps.current_seq > 0):
            nextwp = self.wps.current_seq
        # here x = longitude, y = latitude
        diffy = (self.wps.waypoints[nextwp].x_lat - self.global_position.latitude)
        diffx = (self.wps.waypoints[nextwp].y_long - self.global_position.longitude)
        
        disty = (diffy * self.k_lat)
        distx = (diffx * self.k_long)
        dist = math.sqrt(disty**2 + distx**2)
        
        self.wpinfo_msg.distance_to_next_wp = dist
        
        t_bearing = 0.0
        try:
            if dist > 0:
                #self.get_logger().info("Disty: {0}, dist: {1}".format(disty,dist))
                theta = math.acos(disty/dist)
                t_bearing = math.degrees(theta)

                if distx < 0:
                    t_bearing = 360 - t_bearing
            dist = round(dist,2)
            #self.get_logger().info("dist: {0}, bearing: {1}".format(dist,t_bearing))
            self.wpinfo_msg.target_bearing = t_bearing
        except Exception as e:
            self.get_logger().error("Bearing calculation failed: %s"%e)

    # **********************************************************************************************

    def calculateMidpoint(self):
        midpoint = Waypoint()
        midpoint.x_lat = self.wps.waypoints[1].x_lat
        midpoint.y_long = self.wps.waypoints[1].y_long
        
        if (len(self.wps.waypoints) > 2):
        # Determine mission mindpoint
            maxX = midpoint.y_long
            minX = midpoint.y_long
            maxY = midpoint.x_lat
            minY = midpoint.x_lat
            
            for wp in self.wps.waypoints[2:]:
                maxX = max(maxX, wp.y_long)
                minX = min(minX, wp.y_long)
                maxY = max(maxY, wp.x_lat)
                minY = min(minY, wp.x_lat)
            midpoint.y_long = (minX + maxX) / 2
            midpoint.x_lat = (minY + maxY) / 2
        return midpoint
        
    # *** Run function ***
    def send_wpinfo(self):
        if len(self.wps.waypoints) > 1:
            self.distanceNextWP()
            self.pub_wpinfo.publish(self.wpinfo_msg)

    # *** Callback functions ***
    # Read WP list from the FCU
    def callback_mission_wps(self, data): 
        if self.wps.current_seq != data.current_seq:
            self.get_logger().info("Current mission waypoint sequence updated: {0}".format(data.current_seq))
        self.wps = data
        self.wpinfo_msg.total_wps = len(self.wps.waypoints)

        self.wpinfo_msg.next_wp = self.wps.current_seq

    # Read GPS position data
    def callback_global_position(self, data): 
        self.global_position = data
        self.got_gp = True

"""def shutdown_msg():
    rclpy.node.get_logger("mission_server").info("Exiting server node")    """

def main(args=None):
    rclpy.init(args=args)
    node = WPmanip()
    rclpy.spin(node)
    rclpy.node.get_logger("mission_server").info("Exiting server node")    
    rclpy.shutdown()

if __name__ == '__main__':
    main()