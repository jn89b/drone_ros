#!/usr/bin/env python3

import rclpy
import numpy as np
import math
from pymavlink import mavutil
from rclpy.node import Node
from drone_ros.quaternion_tools import get_euler_from_quaternion, get_quaternion_from_euler
from drone_ros.Commander import Commander
from drone_ros.DroneInfo import DroneInfo
from drone_interfaces.msg import Telem, CtlTraj
# from drone_ros.srv import getGSInfo

from drone_ros.DroneInterfaceModel import DroneInterfaceModel


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

# class GSListenerClient(Node):
#     """
#     Make this as a service
#     """
#     def __init__(self) -> None:
#         super().__init__('gs_listener_client')

#         self.model_info = DroneInterfaceModel()
#         self.info_client = self.create_client(getGSInfo, 'gs_listener')

#         while not self.info_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('service not available, waiting again...')

#     def sendInfoRequest(self):
#         request = getGSInfo.Request()
#         future = self.info_client.call_async(request)
#         # future.add_done_callback(self.__infoResponseCallback)
#         rclpy.spin_until_future_complete(self, future)

#         return future.result()

#     def mapInfo(self, response):
#         self.model_info.setArmDisarm(response.arm_disarm)
#         self.model_info.setGoal(response.goal)
#         self.model_info.setTakeoff(response.takeoff, response.height)
#         self.model_info.setUASType(response.uas_type)


class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        self.get_logger().info('Drone node has been started')

        # frequency interval
        self.declare_parameter('drone_node_frequency', 50)
        self.drone_node_frequency = self.get_parameter(
            'drone_node_frequency').get_parameter_value().integer_value

        self.__initMasterConnection()
        self.__initPublishers()

        self.DroneType = 'VTOL'

        self.commander = Commander(self.master)
        # self.gs_listener = GSListenerClient()
        self.drone_info = DroneInfo(self.master,
                                    self.telem_publisher,
                                    self.drone_node_frequency)

        self.__initSubscribers()

        self.vel_args = {}

    def __initMasterConnection(self) -> None:
        # drone commander initialization
        self.declare_parameter('mav_connection_string', 'udp:127.0.0.1:14551')
        self.mav_connection_string = self.get_parameter('mav_connection_string')\
            .get_parameter_value().string_value
        self.get_logger().info('mav_connection_string: ' + self.mav_connection_string)

        self.master = mavutil.mavlink_connection(self.mav_connection_string)
        self.master.wait_heartbeat()

    def __initPublishers(self) -> None:
        self.telem_publisher = self.create_publisher(
            Telem, 'telem', self.drone_node_frequency)

    def __initSubscribers(self) -> None:
        self.telem_sub = self.create_subscription(
            Telem,
            'telem',
            self.__telemCallback,
            self.drone_node_frequency)

        self.traj_sub = self.create_subscription(
            CtlTraj,
            'trajectory',
            self.__trajCallback,
            self.drone_node_frequency)

    def __telemCallback(self, msg: Telem) -> None:
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt

        self.ground_vel = [msg.vx,
                           msg.vy,
                           msg.vz]

        self.heading = msg.heading

        self.attitudes = [msg.roll,
                          msg.pitch,
                          msg.yaw]

        self.attitude_rates = [msg.roll_rate,
                               msg.pitch_rate,
                               msg.yaw_rate]

        self.ned_position = [msg.x,
                             msg.y,
                             msg.z]

    def __trajCallback(self, msg: CtlTraj):
        x_traj = msg.x
        y_traj = msg.y
        z_traj = msg.z

        roll_traj = msg.roll
        pitch_traj = msg.pitch
        yaw_traj = msg.yaw

        roll_rate_traj = msg.roll_rate
        pitch_rate_traj = msg.pitch_rate
        yaw_rate_traj = msg.yaw_rate

        vx_traj = msg.vx
        vy_traj = msg.vy
        vz_traj = msg.vz
        idx_command = msg.idx

        if self.DroneType == 'VTOL':
            print("idx_command: ", idx_command)
            x_cmd = x_traj[idx_command]
            y_cmd = y_traj[idx_command]

            roll_cmd = np.rad2deg(roll_traj[idx_command])
            pitch_cmd = np.rad2deg(pitch_traj[idx_command])
            # yaw_cmd = np.rad2deg(yaw_traj[idx_command])
            yaw_cmd = np.rad2deg(np.arctan2(x_cmd, y_cmd))

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

    def sendAttitudeTarget(self, roll_angle=0.0, pitch_angle=0.0,
                           yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                           thrust=0.5, body_roll_rate=0.0, body_pitch_rate=0.0):
        """
        From MPC the yaw angle we get is in oriented in
        ENU frame where x is right, y is forward and z is up
        The flight controller is oriented in NED frame where x is forward (north)

        Therefore our yaw angle needs to be converted from ENU to NED
        """
        master = self.master

        if yaw_angle is None:
            yaw_angle = master.messages['ATTITUDE'].yaw

        # add the yaw
        current_yaw_angle = np.rad2deg(master.messages['ATTITUDE'].yaw)

        # # convert the yaw angle from ENU to NED
        yaw_angle = np.rad2deg(np.pi/2) - yaw_angle
        # wrap the angle between -180 and 180
        if yaw_angle > 180:
            yaw_angle -= 360
        elif yaw_angle < -180:
            yaw_angle += 360

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

    drone_node = DroneNode()
    drone_commander = drone_node.commander
    drone_info = drone_node.drone_info
    print("connected to drone")

    # arm_args = {'arm_disarm': 1}
    # drone_commander.armDisarm(arm_args)

    # mode_args = {'mode': 'GUIDED'}
    # drone_commander.changeFlightMode(mode_args)
    # takeoff_args = {'altitude': 15}

    # for i in range(100):
    #     drone_commander.takeoff(takeoff_args)
    #     rclpy.spin_once(drone_node, timeout_sec=0.1)

    # vel_args = {'vx': 5, 'vy': 0, 'vz': 0, 'set_vz': False}
    # drone_commander.sendNEDVelocity(vel_args)

    # rclpy.spin(drone_node)
    while rclpy.ok():
        drone_info.publishTelemInfo()
        rclpy.spin_once(drone_node, timeout_sec=0.1)


if __name__ == '__main__':
    main()
