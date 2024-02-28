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
import ros_mpc.rotation_utils as rot_utils
from ros_mpc.aircraft_config import GOAL_STATE

import mavros
from mavros.base import SENSOR_QOS

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

        #frequency interval
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
        self.old_thrust = 0.5
        self.old_velocity = 20.0
        
        self.state_info =[
            None, #x
            None, #y
            None, #z
            None, #phi
            None, #theta
            None, #psi
            None, #airspeed
        ]

        self.control_info = [
            None, #u_phi
            None, #u_theta
            None, #u_psi
            None  #v_cmd
        ]

    def __initMasterConnection(self) -> None:
        #drone commander initialization 
        self.declare_parameter('mav_connection_string', 'udp:127.0.0.1:14550')
        self.mav_connection_string = self.get_parameter('mav_connection_string')\
            .get_parameter_value().string_value
        self.get_logger().info('mav_connection_string: ' + self.mav_connection_string)

        self.master = mavutil.mavlink_connection(self.mav_connection_string)
        self.master.wait_heartbeat()

    def __initPublishers(self)-> None:
        self.telem_publisher = self.create_publisher(
            Telem, 'telem', self.drone_node_frequency)        

    def __initSubscribers(self) -> None:
        # self.telem_sub = self.create_subscription(
        #     Telem,
        #     'telem',
        #     self.__telemCallback,
        #     self.drone_node_frequency)
        
        self.state_sub = self.create_subscription(mavros.local_position.Odometry,
                                                'mavros/local_position/odom', 
                                                self.__telemCallback, 
                                                qos_profile=SENSOR_QOS)
        
        self.traj_sub = self.create_subscription(
            CtlTraj,
            'directional_trajectory',
            self.__trajCallback,
            self.drone_node_frequency)
        
        # self.traj_sub = self.create_subscription(
        #     CtlTraj,
        #     'directional_trajectory',
        #     self.__trajCallback,
        #     self.drone_node_frequency)

    def __telemCallback(self, msg:Telem) -> None:
        
        # self.mode = msg.mode
        # self.lat = msg.lat
        # self.lon = msg.lon
        # self.alt = msg.alt
        
        # self.ground_vel = [msg.vx, 
        #                    msg.vy, 
        #                    msg.vz]
        
        # self.heading = msg.heading
        
        # self.attitudes = [msg.roll, 
        #                   msg.pitch, 
        #                   msg.yaw]
        
        # self.attitude_rates = [msg.roll_rate, 
        #                        msg.pitch_rate, 
        #                        msg.yaw_rate]
        
        # self.ned_position = [msg.x,
        #                      msg.y,
        #                      msg.z]
        
        #TODO: refactor this method here

        # positions
        self.state_info[0] = msg.pose.pose.position.x
        self.state_info[1] = msg.pose.pose.position.y
        self.state_info[2] = msg.pose.pose.position.z

        # quaternion attitudes
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        roll, pitch, yaw = rot_utils.euler_from_quaternion(
            qx, qy, qz, qw)

        self.state_info[3] = roll
        self.state_info[4] = pitch
        self.state_info[5] = yaw  # (yaw+ (2*np.pi) ) % (2*np.pi);

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        self.state_info[6] = np.sqrt(vx**2 + vy**2 + vz**2)

        #rotate roll and pitch rates to ENU frame   
        self.control_info[0] = msg.twist.twist.angular.x
        self.control_info[1] = msg.twist.twist.angular.y
        self.control_info[2] = msg.twist.twist.angular.z
        self.control_info[3] = msg.twist.twist.linear.x

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
        print(idx_command)
        print("vx traj: ", vx_traj[idx_command])
        # roll_rate_traj = roll_rate_traj[idx_command]
        # pitch_rate_traj = pitch_rate_traj[idx_command]
        # yaw_rate_traj = yaw_rate_traj[idx_command]
        
        
        # guided_mode = '4'
        # guided_mode_list = ['4', '15']
        

        # if self.mode not in guided_mode_list:
        # # if self.mode != guided_mode:
        #     print("not in guided mode")
        #     return 
        
        if self.state_info[0] is None:
            return

        if self.DroneType == 'VTOL':
            roll_cmd = np.rad2deg(roll_traj[idx_command])
            pitch_cmd = np.rad2deg(pitch_traj[idx_command])

            yaw_current = np.rad2deg(self.state_info[5])
            yaw_cmd = np.rad2deg(yaw_traj[idx_command])
        
            yaw_desired = yaw_cmd - yaw_current
            if yaw_desired > 180:
                yaw_desired = yaw_desired - 360
            elif yaw_desired < -180:
                yaw_desired = yaw_desired + 360
                
            #keep in mind z is negative positive 
            error_z = z_traj[idx_command] + GOAL_STATE[2]

            lateral_error = np.sqrt((x_traj[idx_command] - self.state_info[0])**2 +
                                    (y_traj[idx_command] - self.state_info[1])**2)
            
            pitch_desired = np.rad2deg(np.arctan2(error_z, lateral_error))
            pitch_set = (pitch_desired - pitch_cmd)
            
            print("desired roll: ", roll_cmd)
            print("desired pitch: ", pitch_set)
            print("desired yaw: ", yaw_desired)
            # print("current yaw: ", np.rad2deg(self.attitudes[2]))
            airspeed_control = vx_traj[idx_command]#self.control_info[3]
            print("airspeed control: ", airspeed_control)
            
            # self.send_airspeed_command(airspeed_control)
                        
            # z_tolerance = 10.0
            # print("error z: ", error_z)
            
            thrust = self.map_thrust(airspeed_control)


            
            self.sendAttitudeTarget(roll_angle=roll_cmd,
                                    pitch_angle=pitch_cmd,
                                    yaw_angle=yaw_desired,
                                    thrust=thrust)
            self.old_thrust = thrust        
            self.old_velocity = airspeed_control
            

        else:
            vel_args = {'vx': vx_traj[idx_command],
                        'vy': vy_traj[idx_command],
                        'vz': vz_traj[idx_command],
                        'set_vz': False}
            
            # print(vel_args)        
            self.commander.sendNEDVelocity(vel_args)
            
        return 

    def sendAttitudeTarget(self, roll_angle=0.0, pitch_angle=0.0,
                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                         thrust=0.5, body_roll_rate=0.0, body_pitch_rate=0.0):
        """
        IMPORTANT YAW COMMAND NOTES:
        If yaw is positive the drone will rotate clock wise since it is in NED position,
        if negative it will rotate counter clock wise.
        Keep in mind the yaw command does not change the heading of the drone, but 
        makes it rotate constantly at the specificed angle. The higher the angle,
        the tighter the rotation. If you do not regulate roll the drone will 
        either climb up in a spiral or descend in a spiral. Depending on the sign
        
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


    def map_thrust(self, desired_vel:float) -> float:
        vel_max = 25
        vel_min = 15
        
        thrust_min = 0.25
        thrust_max = 0.75
        
        thrust = thrust_min + ((thrust_max - thrust_min) * (desired_vel - vel_min) / (vel_max - vel_min)) 
        
        if thrust > thrust_max:
            thrust = thrust_max
        elif thrust < thrust_min:
            thrust = thrust_min
        
        return thrust

    def send_airspeed_command(self, airspeed:float) -> None:
        master = self.master
        
        master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, #command
        0, #confirmation
        0, #Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
        airspeed, #Speed #m/s
        -1, #Throttle (-1 indicates no change) % 
        0, 0, 0, 0 #ignore other parameters
        )
        print("change airspeed", airspeed)

    def beginTakeoffLand(self, altitude: float) -> None:
        self.commander.takeoff(altitude)
      
def main(args=None):
    rclpy.init(args=args)
    
    drone_node = DroneNode()
    
    # drone_commander = drone_node.commander
    # drone_info = drone_node.drone_info
    
    print("connected to drone")

    rclpy.spin(drone_node)
    while rclpy.ok():
        try:
            # if drone_node.state_info[0] is None:
            #     print("waiting for telem info")
            #     rclpy.spin(drone_node, timeout_sec=1.0)
            #     continue
            # drone_info.publishTelemInfo()
            #print roll and pitch angles
            print("sending commands")
            #drone_node.send_airspeed_command(airspeed=25.0)
            # drone_node.sendAttitudeTarget(roll_angle=0.0,
            #                                 pitch_angle=0.0,
            #                                 yaw_angle=-45,
            #                                 thrust=0.5)
            
            rclpy.spin(drone_node)
            #rclpy.spin_until_future_complete(drone_node, timeout_sec=0.05)

        #check ctrl+c
        except KeyboardInterrupt:
            print("Shutting down")
            return 
        

if __name__ == '__main__':
    main()        




        