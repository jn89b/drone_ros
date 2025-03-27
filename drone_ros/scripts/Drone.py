#!/usr/bin/env python3

import rclpy
import numpy as np
import math
from pymavlink import mavutil
from typing import List, Dict, Any
from rclpy.node import Node
# from ros_mpc.quaternion_tools import (
#     get_euler_from_quaternion,
#     get_quaternion_from_euler,
#     get_relative_ned_yaw_cmd
#     )

from drone_ros.quaternion_tools import (
    get_euler_from_quaternion,
    get_relative_ned_yaw_cmd
)
from drone_ros.Commander import Commander
from drone_ros.DroneInfo import DroneInfo
from drone_interfaces.msg import Telem, CtlTraj
# from drone_ros.srv import getGSInfo

from drone_ros.DroneInterfaceModel import DroneInterfaceModel
import rclpy.publisher
import rclpy.subscription


def compute_pursuit_angle(enu_state: np.array,
                          target_x,
                          target_y) -> float:
    """
    Yaw command is in global frame in ENU convention
    """
    dx: float = target_x - enu_state[0]
    dy: float = target_y - enu_state[1]
    print("dx is: ", dx)
    print("dy is: ", dy)
    enu_yaw_cmd: float = np.arctan2(dy, dx)
    distance = np.sqrt(dx**2 + dy**2)
    print("distance is: ", distance)
    return enu_yaw_cmd


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        self.get_logger().info('Drone node has been started')

        # frequency interval
        self.declare_parameter('drone_node_frequency', 30)
        self.drone_node_frequency = self.get_parameter(
            'drone_node_frequency').get_parameter_value().integer_value

        self.__initMasterConnection()
        self.__initPublishers()

        self.DroneType: str = 'VTOL'

        self.commander: Commander = Commander(self.master)
        # self.gs_listener = GSListenerClient()
        self.drone_info: DroneInfo = DroneInfo(self.master,
                                               self.telem_publisher,
                                               self.drone_node_frequency)

        self.__initSubscribers()

        self.vel_args: Dict[str, float] = {}

    def __initMasterConnection(self) -> None:
        # drone commander initialization
        self.declare_parameter('mav_connection_string', 'udp:127.0.0.1:14551')
        self.mav_connection_string = self.get_parameter('mav_connection_string')\
            .get_parameter_value().string_value
        self.get_logger().info('mav_connection_string: ' + self.mav_connection_string)

        self.master: mavutil = mavutil.mavlink_connection(
            self.mav_connection_string)
        self.master.wait_heartbeat()

    def __initPublishers(self) -> None:
        self.telem_publisher: rclpy.publisher.Publisher = self.create_publisher(
            Telem, 'telem', self.drone_node_frequency)

    def __initSubscribers(self) -> None:
        self.telem_sub: rclpy.subscription.Subscription = self.create_subscription(
            Telem,
            'telem',
            self.__telemCallback,
            self.drone_node_frequency)

        self.traj_sub: rclpy.subscription.Subscription = self.create_subscription(
            CtlTraj,
            'trajectory',
            self.__trajCallback,
            self.drone_node_frequency)

    def __telemCallback(self, msg: Telem) -> None:
        self.lat: float = msg.lat
        self.lon: float = msg.lon
        self.alt: float = msg.alt

        self.ground_vel: List[float] = [msg.vx,
                                        msg.vy,
                                        msg.vz]

        self.heading: float = msg.heading

        self.attitudes: List[float] = [msg.roll,
                                       msg.pitch,
                                       msg.yaw]

        self.attitude_rates: List[float] = [msg.roll_rate,
                                            msg.pitch_rate,
                                            msg.yaw_rate]

        self.ned_position: List[float] = [msg.x,
                                          msg.y,
                                          msg.z]

    def __trajCallback(self, msg: CtlTraj):
        """
        Note the yaw trajectory we get
        is in global frame, we need to convert this 
        to relative yaw command 
        """
        x_traj: float = msg.x
        y_traj: float = msg.y
        z_traj: float = msg.z

        roll_traj: float = msg.roll
        pitch_traj: float = msg.pitch
        yaw_traj: float = msg.yaw

        roll_rate_traj = msg.roll_rate
        pitch_rate_traj = msg.pitch_rate
        yaw_rate_traj = msg.yaw_rate

        vx_traj = msg.vx
        vy_traj = msg.vy
        vz_traj = msg.vz
        idx_command = msg.idx

        if self.DroneType == 'VTOL':
            x_cmd = x_traj[idx_command]
            y_cmd = y_traj[idx_command]
            z_cmd = z_traj[-1]
            #TODO: fix the math model for the z orientation something is messsed up
            z_error = -z_cmd + self.ned_position[2]
            max_pitch:float = 20.0
            kp_pitch:float = 5.0
            pitch_cmd = kp_pitch * z_error
            pitch_cmd = np.clip(pitch_cmd, -max_pitch, max_pitch)
            pitch_cmd = pitch_cmd #+ np.rad2deg(current_pitch)
            roll_cmd = np.rad2deg(roll_traj[idx_command])
            
            # for this application we are going to map the roll command to yaw command
            # it just works better for some reason (probably because of ArduPilot controller)
            yaw_cmd = 0.5*roll_cmd
            # yaw_cmd = get_relative_ned_yaw_cmd(self.attitudes[2], ned_command_yaw)
            print("current yaw: ", np.rad2deg(self.attitudes[2]))
            print("roll_cmd: ", roll_cmd)
            print("pitch_cmd: ", pitch_cmd)
            print("yaw_cmd: ", yaw_cmd)
            self.sendAttitudeTarget(roll_angle=roll_cmd,
                                    pitch_angle=pitch_cmd,
                                    yaw_angle=yaw_cmd,
                                    thrust=0.5)
        else:
            vel_args = {'vx': vx_traj[idx_command],
                        'vy': vy_traj[idx_command],
                        'vz': vz_traj[idx_command],
                        'set_vz': False}

            # print(vel_args)
            self.commander.sendNEDVelocity(vel_args)

    def yaw_rate_from_roll(self, roll_command:float, 
                           airspeed:float, g=9.81) -> float:
        return (g / airspeed) * np.tan(roll_command)

    def update_yaw_command(self, current_yaw:float, 
                           roll_command:float, 
                           airspeed:float, 
                           dt:float, 
                           g=9.81) -> float:
        # Compute the yaw rate induced by the roll command
        yaw_rate = self.yaw_rate_from_roll(roll_command, airspeed, g)
        # Integrate yaw rate over the time step to update the yaw command
        new_yaw = current_yaw + yaw_rate * dt
        # Wrap the yaw angle to [-pi, pi]
        new_yaw = (new_yaw + np.pi) % (2 * np.pi) - np.pi
        return yaw_rate

    def sendAttitudeTarget(self, roll_angle=0.0, pitch_angle=0.0,
                           yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                           thrust=0.5, body_roll_rate=0.0, body_pitch_rate=0.0):
        """
        All commands are in NED frame
        Args:
            roll_angle: Attitude roll angle in degrees
            pitch_angle: Attitude pitch angle in degrees
            yaw_angle: Attitude yaw angle in degrees
            yaw_rate: Yaw rate in degrees per second
            use_yaw_rate: True if yaw rate is used, False if yaw angle is used
            thrust: Thrust
            body_roll_rate: Body roll rate in degrees per second
            body_pitch_rate: Body pitch rate in degrees per second  

        Note:
            Yaw angle command is relative to the aircraft
            that is if you send a yaw angle of 0, the aircraft 
            will go "straight" in the direction it is pointing
            90 degrees will make the aircraft go right more tight than
            a 45 degree command 

            Positive pitch is up

        Therefore our yaw angle needs to be converted from ENU to NED
        """
        master = self.master

        if yaw_angle is None:
            yaw_angle = master.messages['ATTITUDE'].yaw

        # print("yaw angle is: ", yaw_angle)
        master.mav.set_attitude_target_send(
            0,  # time_boot_ms (not used)
            master.target_system,  # target system
            master.target_component,  # target component
            0b00000000 if use_yaw_rate else 0b00000100,
            to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion
            body_roll_rate,  # Body roll rate in radian
            body_pitch_rate,  # Body pitch rate in radian
            np.radians(yaw_rate),  # Body yaw rate in radian/second
            thrust
        )

    def beginTakeoffLand(self, altitude: float) -> None:
        self.commander.takeoff(altitude)


def main(args=None):
    rclpy.init(args=args)

    drone_node: DroneNode = DroneNode()
    drone_commander: Commander = drone_node.commander
    drone_info: DroneInfo = drone_node.drone_info
    print("connected to drone")

    while rclpy.ok():
        drone_info.publishTelemInfo()
        rclpy.spin_once(drone_node, timeout_sec=0.05)


if __name__ == '__main__':
    main()
