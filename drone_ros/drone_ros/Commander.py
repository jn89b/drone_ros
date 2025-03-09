#!/usr/bin/env python3
import numpy as np
from rclpy.node import Node
from pymavlink import mavutil
from typing import Dict, List, Any
from drone_ros.quaternion_tools import to_quaternion_from_euler_dgs
class Commander():
    """
    Might callback from parent node to get information
    about the drone
    """

    def __init__(self, master:mavutil) -> None:
        self.master:mavutil = master

    def validateModeChange(self, mode:bool) -> bool:
        master = self.master
        if mode not in master.mode_mapping():
            print('mode not in args')
            return False
        return True

    def changeFlightMode(self, args:Dict[str, Any]):
        master = self.master
        mode = args['mode']
        print('mode: ' + mode)
        if self.validateModeChange(mode) == False:
            return
        
        mode_id = master.mode_mapping()[mode]
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        print('Flight mode changed to: ' + mode)

    def validateArmDisarm(self, args) -> bool:
        """Validate the arm_disarm argument"""
        try:
            arm_disarm = args['arm_disarm']
        except KeyError:
            print('arm_disarm not in args')
            return False

        if arm_disarm != 1 and arm_disarm != 0:
            print('arm_disarm is not 1 or 0')
            return False

        return True

    def armDisarm(self, args) -> None:
        """
        Arm the drone with 1 and disarm with 0
        https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
        """
        master = self.master
        
        #catch if arm_disarm is not in args
        if self.validateArmDisarm(args) == False:
            return
        
        arm_disarm = args['arm_disarm']
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            arm_disarm, 0, 0, 0, 0, 0, 0)

        if arm_disarm == 1:
            master.motors_armed_wait()
            #log the arm
            print('Armed')
            return 
        
        else:
            master.motors_disarmed_wait()
            print('Disarmed')
            return   

    def validateTakeoff(self, args) -> bool:
        """Validate the takeoff altitude argument"""
        try:
            takeoff_alt = args['altitude']
        except KeyError:
            print('altitude not in args')
            return False

        if takeoff_alt < 0:
            print('altitude is less than 0')
            return False

        return True
        
    def takeoff(self, args) -> None:
        """
        Takeoff the drone
        https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF
        """
        
        if self.validateTakeoff(args) == False:
            return

        takeoff_alt = args['altitude']
    
        #check if takeoff_lat is in args
        if 'latitude' not in args:
            #print('latitude not in args')
            takeoff_lat = 0
        else:
            takeoff_lat = args['latitude']    

        if 'longitude' not in args:
            #print('longitude not in args')
            takeoff_lon = 0
        else:
            takeoff_lon = args['longitude']

        master = self.master
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, #empty
            0, #minimum pitch degrees
            0, #empty
            0, #empty
            0, #yaw angle degrees
            takeoff_lat, #latitude
            takeoff_lon, #longitude
            takeoff_alt) #altitude meters
        

    def validateNEDVelocity(self,args)-> bool:
        """validate the velocity arguments"""
        try:
            vx = args['vx']
            vy = args['vy']
            vz = args['vz']
            set_vz = args['set_vz']
        except KeyError:
            print('vx, vy, vz, yaw, set_vz, not in args')
            return False

        #check if values are numbers
        if not isinstance(vx, (int, float)):
            print('vx is not a number')
            return False
        if not isinstance(vy, (int, float)):
            print('vy is not a number')
            return False
        if not isinstance(vz, (int, float)):
            print('vz is not a number')
            return False
        
        return True

    def sendNEDVelocity(self, args) -> None:
        """
        Set the drone velocity in NED worldframe
        
        args are the following:
        
        set_vz: boolean to set vz or not
        vx: velocity in x direction
        vy: velocity in y direction
        vz: velocity in z direction
        yaw: yaw angle in degrees
        
        """
        vx = args['vx']
        vy = args['vy']

        set_vz = args['set_vz']
        if set_vz == True:
            vz = args['vz']
        else:
            vz = 0

        master = self.master
        master.mav.set_position_target_local_ned_send(
            0, #time_boot_ms
            master.target_system, #target_system
            master.target_component, #target_component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, #coordinate frame
            0b0000111111000111, #type_mask #enable vx, vy, vz, afx, afy, afz
            # 0b110111111000, #type_mask #enable position
            0, #x
            0, #y
            0, #z
            vx, #vx
            vy, #vy
            0, #vz
            0, #afx
            0, #afy
            0, #afz
            0, #yaw
            0) #yaw_rate  

    def sendLandCMD(self, args) -> None:
        """
        Land the drone
        https://ardupilot.org/plane/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-nav-land
        """
        
        master = self.master
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, #empty
            0, #empty
            0, #empty
            0, #empty
            0, #empty
            0, #latitude
            0, #longitude
            0) #Altitude to target for the landing. Unless you are landing at a location different than home, this should be zero
        
        
    
    def sendAttitudeTarget(
        self,
        roll_angle_dg:float=0.0,
        pitch_angle_dg:float=0.0,
        yaw_angle_dg:float=None,
        yaw_rate_dps:float=0.0,
        use_yaw_rate:bool=False,
        thrust:float=0.5,
        body_roll_rate:float=0.0,
        body_pitch_rate:float=0.0) -> None:
        print("roll_angle_dg: ", roll_angle_dg)
        print("pitch_angle_dg: ", pitch_angle_dg)
        print("yaw_angle_dg: ", yaw_angle_dg)
        """
        Args:
            roll_angle_dg (float): Desired roll angle in degrees
            pitch_angle_dg (float): Desired pitch angle in degrees
            yaw_angle_dg (float): Desired yaw angle in degrees
            yaw_rate_dps (float): Desired yaw rate in degrees per second
            use_yaw_rate (bool): Use yaw rate or not
            thrust (float): Desired thrust
            body_roll_rate (float): Desired body roll rate in radian
            body_pitch_rate (float): Desired body pitch rate
        
        Notes:
            Positive yaw is CW and the value is between -180 to 180
            in addition the command is relative that is if you set 
            the yaw angle dg to 0 the drone will face the same direction
            
        where positive 
        """
        if yaw_angle_dg is None:
            yaw_angle_dg = np.rad2deg(self.master.messages['ATTITUDE'].yaw)

        self.master.mav.set_attitude_target_send(
            0,  # time_boot_ms (not used)
            self.master.target_system,  # target system
            self.master.target_component,  # target component
            0b00000000 if use_yaw_rate else 0b00000100,
            to_quaternion_from_euler_dgs(roll_angle_dg,
                                         pitch_angle_dg, yaw_angle_dg),  # Quaternion
            body_roll_rate,  # Body roll rate in radian
            body_pitch_rate,  # Body pitch rate in radian
            np.radians(yaw_rate_dps),  # Body yaw rate in radian/second
            thrust
        )
        
        
        
    
    
    
