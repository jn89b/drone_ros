#!/usr/bin/env python3
"""
Drone Control Node

This module implements a ROS2 node that interfaces with a drone via MAVLink.
It subscribes to telemetry and trajectory messages, computes control commands,
and sends attitude targets and velocity commands to the drone. The node also
manages the MAVLink connection and related utilities (e.g., quaternion conversion).

Dependencies:
    - rclpy
    - numpy
    - math
    - pymavlink
    - drone_ros (including quaternion_tools, Commander, DroneInfo, DroneInterfaceModel)
    - drone_interfaces (Telem, CtlTraj messages)
"""

import rclpy
import numpy as np
import math
import mavros
from mavros.base import STATE_QOS
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

from drone_ros.DroneInterfaceModel import DroneInterfaceModel
import rclpy.publisher
import rclpy.subscription


def compute_pursuit_angle(enu_state: np.array,
                          target_x: float,
                          target_y: float) -> float:
    """
    Compute the yaw command (pursuit angle) required to move towards a target.

    This function calculates the angle between the current position (provided
    in ENU coordinates) and a target position. It prints out the distance and the
    x/y differences for debugging purposes.

    Args:
        enu_state (np.array): The current position in ENU coordinates, where index 0
            is x and index 1 is y.
        target_x (float): The target x-coordinate.
        target_y (float): The target y-coordinate.

    Returns:
        float: The computed yaw command in radians, following the ENU convention.
    """
    dx: float = target_x - enu_state[0]
    dy: float = target_y - enu_state[1]
    print("dx is: ", dx)
    print("dy is: ", dy)
    enu_yaw_cmd: float = np.arctan2(dy, dx)
    distance = np.sqrt(dx**2 + dy**2)
    print("distance is: ", distance)
    return enu_yaw_cmd


def to_quaternion(roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> List[float]:
    """
    Convert Euler angles (in degrees) to a quaternion.

    This function converts roll, pitch, and yaw (in degrees) to the corresponding
    quaternion representation. The conversion uses standard formulas for transforming
    Euler angles to a quaternion.

    Args:
        roll (float, optional): Roll angle in degrees. Defaults to 0.0.
        pitch (float, optional): Pitch angle in degrees. Defaults to 0.0.
        yaw (float, optional): Yaw angle in degrees. Defaults to 0.0.

    Returns:
        List[float]: A list of four floats representing the quaternion [w, x, y, z].
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
    """
    ROS2 Node for controlling a drone via MAVLink.

    This node sets up the necessary publishers, subscribers, and MAVLink connection.
    It processes telemetry and trajectory data to compute control commands, such as
    attitude targets or velocity commands, and sends them to the drone.
    """

    def __init__(self):
        """
        Initialize the DroneNode.
        
        Sets up parameters, initializes the MAVLink connection, publishers,
        subscribers, and creates instances of helper classes (Commander, DroneInfo).
        """
        super().__init__('drone_node')
        self.get_logger().info('Drone node has been started')

        # Frequency interval for publishers and subscribers
        self.declare_parameter('drone_node_frequency', 30)
        self.drone_node_frequency = self.get_parameter(
            'drone_node_frequency').get_parameter_value().integer_value

        self.__initMasterConnection()
        self.__initPublishers()

        # Define control types and set the default control method
        # PID attitude for MPC
        # attitude-only for attitude control - Reinforcement learning
        self.control_type: List[str] = ["pid_attitude", "attitude_only", "velocity_only"]
        self.control_method: str = self.control_type[0]

        self.commander: Commander = Commander(self.master)
        # self.gs_listener = GSListenerClient()  # Uncomment if ground station listener is used
        self.drone_info: DroneInfo = DroneInfo(self.master,
                                               self.telem_publisher,
                                               self.drone_node_frequency)

        self.__initSubscribers()

        self.vel_args: Dict[str, float] = {}
        self.lat: float = None
        self.lon: float = None
        self.alt: float = None
        self.ground_vel: List[float] = [0.0, 0.0, 0.0]
        self.heading: float = None
        self.attitudes: List[float] = [0.0, 0.0, 0.0]
        self.attitude_rates: List[float] = [0.0, 0.0, 0.0]
        self.ned_position: List[float] = [0.0, 0.0, 0.0]
        self.current_mode: str = None

    def __initMasterConnection(self) -> None:
        """
        Initialize the MAVLink master connection.

        Reads the MAVLink connection string from parameters, establishes a connection,
        and waits for the heartbeat signal to ensure the connection is active.
        """
        self.declare_parameter('mav_connection_string', 'udp:127.0.0.1:14551')
        self.mav_connection_string = self.get_parameter('mav_connection_string')\
            .get_parameter_value().string_value
        self.get_logger().info('mav_connection_string: ' + self.mav_connection_string)

        self.master: mavutil = mavutil.mavlink_connection(self.mav_connection_string,
                                                          source_system=1)
        self.master.wait_heartbeat()

    def __initPublishers(self) -> None:
        """
        Initialize ROS2 publishers.

        Sets up the telemetry publisher for sending out drone telemetry messages.
        """
        self.telem_publisher: rclpy.publisher.Publisher = self.create_publisher(
            Telem, 'telem', self.drone_node_frequency)

    def __initSubscribers(self) -> None:
        """
        Initialize ROS2 subscribers.

        Creates subscriptions for telemetry and trajectory messages and assigns
        their respective callback functions.
        """
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

        self.mode_sub: rclpy.subscription.Subscription = self.create_subscription(
            mavros.system.State,
            'mavros/state',
            self.__modeCallback,
            STATE_QOS)


    def __telemCallback(self, msg: Telem) -> None:
        """
        Callback function for telemetry data.

        Updates the drone's state information including position, velocity, and attitudes.

        Args:
            msg (Telem): The telemetry message containing the drone's current state.
        """
        self.lat: float = msg.lat
        self.lon: float = msg.lon
        self.alt: float = msg.alt

        self.ground_vel: List[float] = [msg.vx, msg.vy, msg.vz]

        self.heading: float = msg.heading

        self.attitudes: List[float] = [msg.roll, msg.pitch, msg.yaw]

        self.attitude_rates: List[float] = [msg.roll_rate, msg.pitch_rate, msg.yaw_rate]

        self.ned_position: List[float] = [msg.x, msg.y, msg.z]

    def __trajCallback(self, msg: CtlTraj) -> None:
        """
        Callback function for trajectory data.

        Processes the trajectory message to extract target positions, angles,
        and velocities. Depending on the control method, it computes the required
        attitude or velocity commands and sends them to the drone.

        Args:
            msg (CtlTraj): The trajectory message containing the planned path and commands.
        """
        x_traj: float = msg.x
        y_traj: float = msg.y
        z_traj: float = msg.z

        roll_traj: float = msg.roll
        pitch_traj: float = msg.pitch
        yaw_traj: float = msg.yaw
        thrust_cmd = msg.thrust

        vx_traj = msg.vx
        vy_traj = msg.vy
        vz_traj = msg.vz
        idx_command = msg.idx

        if self.current_mode != "GUIDED":
            print("Not in GUIDED mode, cannot send commands")
            return

        if self.control_method == self.control_type[0]:
            yaw_cmd = np.rad2deg(yaw_traj[idx_command])
            pitch_cmd = np.rad2deg(pitch_traj[idx_command])
            roll_cmd = np.rad2deg(roll_traj[idx_command])
            thrust_cmd = thrust_cmd[idx_command]
            thrust_cmd = np.clip(thrust_cmd, 0.3, 0.7)
            print("current yaw: ", np.rad2deg(self.attitudes[2]))
            print("roll_cmd: ", roll_cmd)
            print("pitch_cmd: ", pitch_cmd)
            print("yaw_cmd: ", yaw_cmd)
            
            self.sendAttitudeTarget(roll_angle=roll_cmd,
                                    pitch_angle=pitch_cmd,
                                    yaw_angle=yaw_cmd,
                                    thrust=thrust_cmd)
            
        elif self.control_method == self.control_type[1]:
            # Attitude-only control mode
            print("index command: ", idx_command)
            # check if index comand out of range
            # if len(roll_traj) <= idx_command or \
            #    len(pitch_traj) <= idx_command or \
            #    len(yaw_traj) <= idx_command or \
            #    len(thrust_cmd) <= idx_command:
            #     print("Index command out of range, skipping command")
            #     return
            
            roll_cmd = np.rad2deg(roll_traj[idx_command])
            pitch_cmd = np.rad2deg(pitch_traj[idx_command])
            yaw_cmd = np.rad2deg(yaw_traj[idx_command])
            thrust_cmd = thrust_cmd[idx_command]
            self.sendAttitudeTarget(roll_angle=roll_cmd,
                                    pitch_angle=pitch_cmd,
                                    yaw_angle=yaw_cmd,
                                    thrust=thrust_cmd)
            # send airspeed command
        else:
            # Velocity-only control mode
            vel_args = {
                'vx': vx_traj[idx_command],
                'vy': vy_traj[idx_command],
                'vz': vz_traj[idx_command],
                'set_vz': False
            }
            self.commander.sendNEDVelocity(vel_args)

    def __modeCallback(self, msg: mavros.system.State) -> None:
        """
        Callback function for mode changes.

        Updates the current mode of the drone based on the received MAVLink message.

        Args:
            msg (mavros.system.State): The MAVLink message containing the current mode.
        """
        self.current_mode = msg.mode
    def yaw_rate_from_roll(self, roll_command: float, 
                           airspeed: float, g: float = 9.81) -> float:
        """
        Calculate the yaw rate induced by a roll command.

        This function computes the yaw rate based on the given roll command and airspeed,
        using the formula: yaw_rate = (g / airspeed) * tan(roll_command).

        Args:
            roll_command (float): The roll command in radians.
            airspeed (float): The current airspeed of the drone.
            g (float, optional): Gravitational acceleration. Defaults to 9.81 m/s².

        Returns:
            float: The computed yaw rate.
        """
        return (g / airspeed) * np.tan(roll_command)

    def update_yaw_command(self, current_yaw: float, 
                           roll_command: float, 
                           airspeed: float, 
                           dt: float, 
                           g: float = 9.81) -> float:
        """
        Update the yaw command based on the current yaw, roll command, and elapsed time.

        Integrates the yaw rate over the time step and wraps the resulting yaw angle within
        the range [-pi, pi].

        Args:
            current_yaw (float): The current yaw angle in radians.
            roll_command (float): The roll command in radians.
            airspeed (float): The current airspeed of the drone.
            dt (float): The time step (delta time) over which to integrate.
            g (float, optional): Gravitational acceleration. Defaults to 9.81 m/s².

        Returns:
            float: The computed yaw rate after applying the update.
        """
        yaw_rate = self.yaw_rate_from_roll(roll_command, airspeed, g)
        new_yaw = current_yaw + yaw_rate * dt
        # Wrap the yaw angle to [-pi, pi]
        new_yaw = (new_yaw + np.pi) % (2 * np.pi) - np.pi
        return yaw_rate

    def sendAttitudeTarget(self, roll_angle: float = 0.0, pitch_angle: float = 0.0,
                           yaw_angle: float = None, yaw_rate: float = 0.0, use_yaw_rate: bool = False,
                           thrust: float = 0.5, body_roll_rate: float = 0.0, body_pitch_rate: float = 0.0) -> None:
        """
        Send an attitude target command to the drone using MAVLink.

        All commands are assumed to be in the NED frame. Depending on whether a yaw
        angle or yaw rate is provided, the command uses the corresponding flag.

        Args:
            roll_angle (float, optional): Desired roll angle in degrees. Defaults to 0.0.
            pitch_angle (float, optional): Desired pitch angle in degrees. Defaults to 0.0.
            yaw_angle (float, optional): Desired yaw angle in degrees. If None, the current
                yaw from the drone's attitude message is used.
            yaw_rate (float, optional): Yaw rate in degrees per second. Defaults to 0.0.
            use_yaw_rate (bool, optional): True if using yaw rate for control, False for yaw angle. Defaults to False.
            thrust (float, optional): Thrust command value. Defaults to 0.5.
            body_roll_rate (float, optional): Body roll rate in degrees per second. Defaults to 0.0.
            body_pitch_rate (float, optional): Body pitch rate in degrees per second. Defaults to 0.0.

        Note:
            - The yaw angle command is relative to the aircraft’s current heading.
            - A yaw command of 0 implies flying straight ahead relative to its current heading.
            - Positive pitch commands result in upward movement.
        """
        master = self.master

        if yaw_angle is None:
            yaw_angle = master.messages['ATTITUDE'].yaw

        master.mav.set_attitude_target_send(
            0,  # time_boot_ms (not used)
            master.target_system,  # target system
            master.target_component,  # target component
            0b00000000 if use_yaw_rate else 0b00000100,  # Type mask flag
            to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion conversion
            body_roll_rate,  # Body roll rate in radians
            body_pitch_rate,  # Body pitch rate in radians
            np.radians(yaw_rate),  # Body yaw rate in radians/second
            thrust
        )

    def beginTakeoffLand(self, altitude: float) -> None:
        """
        Initiate a takeoff or landing sequence.

        This method calls the takeoff function of the Commander instance with the desired altitude.

        Args:
            altitude (float): The target altitude for takeoff.
        """
        self.commander.takeoff(altitude)


def main(args=None):
    """
    Main entry point for the drone control node.

    Initializes the ROS2 Python client library, creates the DroneNode, and enters
    the ROS2 spin loop, periodically publishing telemetry information.
    """
    rclpy.init(args=args)

    drone_node: DroneNode = DroneNode()
    # drone_commander: Commander = drone_node.commander
    drone_info: DroneInfo = drone_node.drone_info
    print("connected to drone")

    while rclpy.ok():
        # drone_info.publishTelemInfo()
        rclpy.spin_once(drone_node, timeout_sec=0.0)


if __name__ == '__main__':
    main()
