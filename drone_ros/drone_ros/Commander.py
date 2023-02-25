#!/usr/bin/env python3
from rclpy.node import Node
from pymavlink import mavutil

class Commander():
    """
    Might callback from parent node to get information
    about the drone
    """

    def __init__(self, master):
        self.master = master

    def validateModeChange(self, mode) -> bool:
        master = self.master
        if mode not in master.mode_mapping():
            print('mode not in args')
            return False
        return True

    def changeFlightMode(self, args):
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
        vy = args['vy']
        vx = args['vx']
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
            mavutil.mavlink.MAV_FRAME_BODY_NED, #frame
            0b0000111111000111, #type_mask
            0, #x
            0, #y
            0, #z
            vx, #vx
            vy, #vy
            vz, #vz
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
        

    
