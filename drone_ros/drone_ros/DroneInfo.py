#!/usr/bin/env python3
"""
Drone Info Module

This module defines the DroneInfo class, which is responsible for configuring
the drone to send telemetry data at a desired frequency, retrieving that data
via MAVLink, and publishing it to a ROS2 topic using a provided publisher.

Dependencies:
    - pymavlink: For interfacing with MAVLink messages.
    - drone_interfaces: Provides the Telem message definition.
    - rclpy.node: For creating ROS2 nodes.
    - math: For mathematical functions.
"""

from pymavlink import mavutil
from drone_interfaces.msg import Telem
from rclpy.node import Node
import math


class DroneInfo:
    """
    Class for handling drone telemetry information via MAVLink.

    This class configures the drone to send telemetry messages at a specified
    frequency, retrieves these messages, and publishes the data using a ROS2 publisher.

    Attributes:
        master (mavutil.mavlink_connection): The MAVLink connection instance.
        telem_publisher: ROS2 publisher to publish telemetry (Telem) messages.
        drone_info_frequency (int): Frequency (in Hz) for requesting telemetry messages.
    """

    def __init__(self, master, telem_publisher, drone_info_frequency) -> None:
        """
        Initialize the DroneInfo instance.

        Args:
            master: The MAVLink connection instance.
            telem_publisher: A ROS2 publisher for sending telemetry messages.
            drone_info_frequency: The frequency (in Hz) at which to request telemetry messages.
        """
        self.master = master
        self.telem_publisher = telem_publisher
        self.drone_info_frequency = drone_info_frequency

    def __startListening(self) -> None:
        """
        Configure the drone to send telemetry messages.

        This private method requests the desired message interval for several
        MAVLink message types (local position, attitude, global position, and
        attitude quaternion) so that the telemetry data is updated at the specified
        frequency.
        """
        self.__requestMessageInterval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                                      self.drone_info_frequency)
        self.__requestMessageInterval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                                      self.drone_info_frequency)
        self.__requestMessageInterval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                                      self.drone_info_frequency)
        self.__requestMessageInterval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
                                      self.drone_info_frequency)

    def __getData(self) -> Telem:
        """
        Retrieve the latest telemetry data from the drone.

        This method blocks until a MAVLink message matching one of the specified types
        is received. It extracts data from the GLOBAL_POSITION_INT, ATTITUDE, and
        LOCAL_POSITION_NED messages to populate a Telem message with the drone's state.
        If any expected data is missing, it catches the KeyError and returns the partially
        populated Telem message.

        Returns:
            Telem: A telemetry message populated with the latest drone state data.
        """
        output = Telem()
        # Blocking call waiting for one of the desired message types.
        msg = self.master.recv_match(
            type=['LOCAL_POSITION_NED', 'ATTITUDE', 'GLOBAL_POSITION_INT'],
            blocking=True
        )

        try:
            output.lat = self.master.messages['GLOBAL_POSITION_INT'].lat
            output.lon = self.master.messages['GLOBAL_POSITION_INT'].lon
            output.alt = self.master.messages['GLOBAL_POSITION_INT'].alt
            output.heading = self.master.messages['GLOBAL_POSITION_INT'].hdg

            # Uncomment the following block to use quaternion-based attitude calculations.
            qx = self.master.messages['ATTITUDE_QUATERNION'].q1
            qy = self.master.messages['ATTITUDE_QUATERNION'].q2
            qz = self.master.messages['ATTITUDE_QUATERNION'].q3
            qw = self.master.messages['ATTITUDE_QUATERNION'].q4
            output.qx = qx
            output.qy = qy
            output.qz = qz
            output.qw = qw
            #
            # output.roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
            # output.pitch = math.asin(2*(qw*qy - qz*qx))
            # output.yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

            output.roll = self.master.messages['ATTITUDE'].roll
            output.pitch = self.master.messages['ATTITUDE'].pitch
            output.yaw = self.master.messages['ATTITUDE'].yaw

            output.roll_rate = self.master.messages['ATTITUDE'].rollspeed
            output.pitch_rate = self.master.messages['ATTITUDE'].pitchspeed
            output.yaw_rate = self.master.messages['ATTITUDE'].yawspeed

            output.x = self.master.messages['LOCAL_POSITION_NED'].x
            output.y = self.master.messages['LOCAL_POSITION_NED'].y
            output.z = self.master.messages['LOCAL_POSITION_NED'].z
            output.vx = self.master.messages['LOCAL_POSITION_NED'].vx
            output.vy = self.master.messages['LOCAL_POSITION_NED'].vy
            output.vz = self.master.messages['LOCAL_POSITION_NED'].vz

        except KeyError:
            # If some data is missing, return the output as is.
            return output

        return output

    def __requestMessageInterval(self, message_id: int, frequency_hz: int) -> None:
        """
        Request a specific MAVLink message at a desired frequency.

        This method sends a command to the drone to set the interval between messages
        of a given type. The interval is calculated in microseconds based on the desired
        frequency.

        Documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

        Args:
            message_id (int): The MAVLink message ID.
            frequency_hz (int): The desired frequency in Hertz.
        """
        self.master.mav.command_long_send(
            self.master.target_system,  # target system
            self.master.target_component,  # target component
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,  # command, confirmation
            message_id,              # The MAVLink message ID
            1e6 / frequency_hz,      # Interval between messages in microseconds
            0, 0, 0, 0, 0           # Unused parameters
        )

    def publishTelemInfo(self) -> Telem:
        """
        Retrieve and publish the latest telemetry information.

        This method configures the drone to start sending telemetry messages,
        retrieves the latest data using the __getData method, and publishes it
        using the provided ROS2 publisher.

        Returns:
            Telem: The telemetry message that was published.
        """
        self.__startListening()
        output = self.__getData()
        self.telem_publisher.publish(output)
        return output
