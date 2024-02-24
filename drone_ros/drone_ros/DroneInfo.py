#!/usr/bin/env python3
from pymavlink import mavutil
from drone_interfaces.msg import Telem
from  drone_ros import quaternion_tools
from rclpy.node import Node

import math

class DroneInfo():
    def __init__(self, master, 
                 telem_publisher,
                 drone_info_frequency) -> None:
        self.master = master
        self.telem_publisher = telem_publisher
        self.drone_info_frequency = drone_info_frequency

        
    def __startListening(self) -> None:
        self.__requestMessageInterval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 
                                    self.drone_info_frequency)
        self.__requestMessageInterval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                                    self.drone_info_frequency)
        self.__requestMessageInterval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                                    self.drone_info_frequency)
        self.__requestMessageInterval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
                                    self.drone_info_frequency)
        # self.__requestMessageInterval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_TARGET,
        #                             self.drone_info_frequency)
    
        self.__requestMessageInterval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
                                    self.drone_info_frequency)

    def __getData(self) -> Telem:
        
        output = Telem()
        msg = self.master.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE', 'GLOBAL_POSITION_INT', 'HEARTBEAT'], blocking=True)

        try:
            output.mode = str(self.master.messages['HEARTBEAT'].custom_mode)
            output.lat = self.master.messages['GLOBAL_POSITION_INT'].lat
            output.lon = self.master.messages['GLOBAL_POSITION_INT'].lon
            output.alt = self.master.messages['GLOBAL_POSITION_INT'].alt
            output.heading = self.master.messages['GLOBAL_POSITION_INT'].hdg
            
            output.roll = self.master.messages['ATTITUDE'].roll  
            output.pitch = self.master.messages['ATTITUDE'].pitch
            output.pitch = self.master.messages['ATTITUDE'].pitch   
            output.yaw = self.master.messages['ATTITUDE'].yaw + math.pi/2
            if output.yaw > math.pi:
                output.yaw -= 2*math.pi
            elif output.yaw < -math.pi:
                output.yaw += 2*math.pi
                
                
                
            # qx = self.master.messages['ATTITUDE_QUATERNION'].q1
            # qy = self.master.messages['ATTITUDE_QUATERNION'].q2
            # qz = self.master.messages['ATTITUDE_QUATERNION'].q3
            # qw = self.master.messages['ATTITUDE_QUATERNION'].q4

            # output.roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
            # output.pitch = math.asin(2*(qw*qy - qz*qx))
            # output.yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
            

            output.roll_rate = self.master.messages['ATTITUDE'].rollspeed
            output.pitch_rate = self.master.messages['ATTITUDE'].pitchspeed
            output.yaw_rate = self.master.messages['ATTITUDE'].yawspeed
            output.pitch_rate = -self.master.messages['ATTITUDE'].pitchspeed

            yaw_rate = -self.master.messages['ATTITUDE'].yawspeed + math.pi/2
            #output.yaw_rate = yaw_rate - math.pi/2#(yaw_rate + math.pi) % (2 * math.pi) - math.pi
            output.yaw_rate = (yaw_rate + math.pi) % (2 * math.pi) - math.pi

            output.x = self.master.messages['LOCAL_POSITION_NED'].x
            output.y = self.master.messages['LOCAL_POSITION_NED'].y
            output.z = self.master.messages['LOCAL_POSITION_NED'].z
            output.vx = self.master.messages['LOCAL_POSITION_NED'].vx
            output.vy = self.master.messages['LOCAL_POSITION_NED'].vy   
            output.vz = self.master.messages['LOCAL_POSITION_NED'].vz
        
        #catch key error 
        except KeyError:
            return output

        return output 

    def __requestMessageInterval(self, message_id:int, frequency_hz:int) -> None:
        """
        Request MAVLink message in a desired frequency,
        documentation for SET_MESSAGE_INTERVAL:
            https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

        Args:
            message_id (int): MAVLink message ID
            frequency_hz (float): Desired frequency in Hz
        """
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, # The MAVLink message ID
            1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
            0, 0, 0, 0, # Unused parameters
            0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
        )

    def publishTelemInfo(self) -> Telem:
        self.__startListening()

        output = self.__getData()
        self.telem_publisher.publish(output)
