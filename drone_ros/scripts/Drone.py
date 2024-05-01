#!/usr/bin/env python3

import rclpy
import numpy as np
import math
from pymavlink import mavutil
from rclpy.node import Node
import pandas as pd
from drone_ros.quaternion_tools import get_euler_from_quaternion, get_quaternion_from_euler
from drone_ros.Commander import Commander
from drone_ros.DroneInfo import DroneInfo
from drone_interfaces.msg import Telem, CtlTraj
# from drone_ros.srv import getGSInfo
import ros_mpc.rotation_utils as rot_utils
from ros_mpc.aircraft_config import state_constraints
import os
from ros_mpc.aircraft_config import GOAL_STATE
from ros_mpc.aircraft_config import obs_avoid_params

from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64

import mavros
from mavros.base import SENSOR_QOS
from mavros.system import STATE_QOS

from drone_ros.DroneInterfaceModel import DroneInterfaceModel

def get_time_in_secs(some_node:Node) -> float:
    return some_node.get_clock().now().nanoseconds /1E9 

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

        self.__init_variables()
        self.vel_args = {}
        self.old_thrust = 0.5
        self.old_velocity = 20.0
        self.start_time = get_time_in_secs(self)
        
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
        self.declare_parameter('mav_connection_string', 'udp:192.168.135.72:14550')
        print("connecting to", self.get_parameter('mav_connection_string').get_parameter_value().string_value)
        #self.declare_parameter('mav_connection_string', 'udp:192.168.1.101:14551')
        self.mav_connection_string = self.get_parameter('mav_connection_string')\
            .get_parameter_value().string_value
        self.get_logger().info('mav_connection_string: ' + self.mav_connection_string)

        self.master = mavutil.mavlink_connection(self.mav_connection_string)
        self.master.wait_heartbeat()

    def __init_variables(self) -> None:	
        self.effector_dmg_curr = -1.0
        self.effector_x = []
        self.effector_y = []
        self.effector_z = []
        self.current_mode = None
        
        self.cost_val = 1E10
        self.time_solution = -100
        
        self.time = []
        self.x_pos_history = []
        self.y_pos_history = []
        self.z_pos_history = []
        self.roll_history = []
        self.pitch_history = []
        self.yaw_history = []
        self.vel_history = []

        self.x_traj_history = []
        self.y_traj_history = []
        self.z_traj_history = []
        self.roll_traj_history = []
        self.pitch_traj_history = []
        self.yaw_traj_history = []

        self.x_cmd_history = []
        self.y_cmd_history = []
        self.z_cmd_history = []
        self.roll_cmd_history = []
        self.pitch_cmd_history = []
        self.yaw_cmd_history = []
        self.vel_cmd_history = []

        self.goal_x_history = []
        self.goal_y_history = []
        self.goal_z_history = []

        self.effector_dmg_history = []
        self.effector_x_history = []
        self.effector_y_history = []
        self.effector_z_history = []

        self.cost_val_history = []
        self.time_solution_history = []

    def __initPublishers(self)-> None:
        self.telem_publisher = self.create_publisher(
            Telem, 'telem', self.drone_node_frequency)        

    def __initSubscribers(self) -> None:
        
        self.state_sub = self.create_subscription(mavros.local_position.Odometry,
                                                'mavros/local_position/odom', 
                                                self.__telemCallback, 
                                                qos_profile=SENSOR_QOS)
        
        self.mode_sub = self.create_subscription(mavros.system.State,
                                                 'mavros/state',
                                                 self.__modeCallback,
                                                 qos_profile=STATE_QOS)
        
        self.effector_dmg_sub = self.create_subscription(
                        Float64,
                        '/damage_info',
                        self.effector_dmg_callback,
                        self.drone_node_frequency)
  
        self.effector_location_sub = self.create_subscription(
            PoseArray, 
            '/effector_location',
            self.effector_pos_callback,
            self.drone_node_frequency)

        self.time_solution_sub = self.create_subscription(
            Float64,
            '/waypoint_time_sol',
            self.time_solution_callback,
            self.drone_node_frequency)
        
        # self.traj_sub = self.create_subscription(
        #     CtlTraj,
        #     'directional_trajectory',
        #     self.__trajCallback,
        #     self.drone_node_frequency)
        
        self.traj_sub = self.create_subscription(
            CtlTraj,
            'omni_trajectory',
            self.__trajCallback,
            self.drone_node_frequency)
        
        # self.traj_sub = self.create_subscription(
        #     CtlTraj,
        #     'waypoint_trajectory',
        #     self.__trajCallback,
        #     self.drone_node_frequency)
        
        # self.traj_sub = self.create_subscription(
        #     CtlTraj,
        #     'directional_trajectory',
        #     self.__trajCallback,
        #     self.drone_node_frequency)

    def update_history(self) -> None:
        """update the history of the state and control variables"""
        self.x_pos_history.append(self.state_info[0])
        self.y_pos_history.append(self.state_info[1])
        self.z_pos_history.append(self.state_info[2])
        self.roll_history.append(self.state_info[3])
        self.pitch_history.append(self.state_info[4])
        self.yaw_history.append(self.state_info[5])
        self.vel_history.append(self.state_info[6])
        self.x_traj_history.append(self.x_traj_curr)
        self.y_traj_history.append(self.y_traj_curr)
        self.z_traj_history.append(self.z_traj_curr)
        self.roll_traj_history.append(self.roll_traj_curr)
        self.pitch_traj_history.append(self.pitch_traj_curr)
        self.yaw_traj_history.append(self.yaw_traj_curr)

        self.x_cmd_history.append(self.x_cmd_curr)
        self.y_cmd_history.append(self.y_cmd_curr)
        self.z_cmd_history.append(self.z_cmd_curr)
        self.roll_cmd_history.append(self.roll_cmd_curr)
        self.pitch_cmd_history.append(self.pitch_cmd_curr)
        self.yaw_cmd_history.append(self.yaw_cmd_curr)
        self.vel_cmd_history.append(self.vel_cmd_curr)

        self.effector_dmg_history.append(self.effector_dmg_curr)
        self.effector_x_history.append(self.effector_x)
        self.effector_y_history.append(self.effector_y)
        self.effector_z_history.append(self.effector_z)
        
        self.cost_val_history.append(self.cost_val)
        self.time_solution_history.append(self.time_solution)
  
    def effector_dmg_callback(self,msg: Float64):
        self.effector_dmg_curr = msg.data

    def effector_pos_callback(self,msg: PoseArray):
        poses = msg.poses
        x_position = []
        y_position = []
        z_position = []
        for pose in poses:
            x_position.append(pose.position.x)
            y_position.append(pose.position.y)
            z_position.append(pose.position.z)
            # self.effector_pos_curr.append([pose.position.x, pose.position.y, pose.position.z])
        self.effector_x = x_position
        self.effector_y = y_position
        self.effector_z = z_position

    def __modeCallback(self, msg: mavros.system.State) -> None:
        self.current_mode = msg.mode

    def __telemCallback(self, msg:Telem) -> None:
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
    
        vx_traj = msg.vx
        vy_traj = msg.vy
        vz_traj = msg.vz

        idx_command = msg.idx
    
        if self.state_info[0] is None:
            return
        
        if self.current_mode != "GUIDED":
            return 

        if self.DroneType == 'VTOL':
            roll_cmd = np.rad2deg(roll_traj[idx_command])
            pitch_cmd = np.rad2deg(pitch_traj[idx_command])

            yaw_current = np.rad2deg(self.state_info[5])
            yaw_cmd = np.rad2deg(yaw_traj[idx_command])
        
            yaw_desired = yaw_cmd - yaw_current

            # print("yaw desired", yaw_desired)
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

            airspeed_control = vx_traj[idx_command]#self.control_info[3]            
            print("desired roll: ", roll_cmd)
            print("desired pitch: ", pitch_set)
            print("desired yaw: ", yaw_desired)
            print("airspeed control: ", airspeed_control)
            thrust = self.map_thrust(airspeed_control)

            self.sendAttitudeTarget(roll_angle=roll_cmd,
                                    pitch_angle=pitch_cmd,
                                    yaw_angle=yaw_desired,
                                    thrust=thrust)
            self.old_thrust = thrust        
            self.old_velocity = airspeed_control
            
            #set airspeed
            # self.send_airspeed_command(airspeed=airspeed_control)

        else:
            vel_args = {'vx': vx_traj[idx_command],
                        'vy': vy_traj[idx_command],
                        'vz': vz_traj[idx_command],
                        'set_vz': False}
            
            # print(vel_args)        
            self.commander.sendNEDVelocity(vel_args)
            
        self.x_traj_curr = x_traj
        self.y_traj_curr = y_traj
        self.z_traj_curr = z_traj
        self.roll_traj_curr = roll_traj
        self.pitch_traj_curr = pitch_traj
        self.yaw_traj_curr = yaw_traj
        
        self.vel_cmd_curr = vx_traj[idx_command]
        self.x_cmd_curr = x_traj[idx_command]
        self.y_cmd_curr = y_traj[idx_command]
        self.z_cmd_curr = z_traj[idx_command]
        self.roll_cmd_curr = roll_traj[idx_command]
        self.pitch_cmd_curr = pitch_traj[idx_command]
        self.yaw_cmd_curr = yaw_traj[idx_command]
        self.time.append(get_time_in_secs(self) - self.start_time)
        self.update_history()    
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
        vel_max = state_constraints['airspeed_max']
        vel_min = state_constraints['airspeed_min']
        
        thrust_min = 0.25
        thrust_max = 0.75
        
        thrust = thrust_min + ((thrust_max - thrust_min) * \
            (desired_vel - vel_min) / (vel_max - vel_min)) 
        
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
        0, #Throttle (-1 indicates no change) % 
        0, 0, 0, 0 #ignore other parameters
        )
        # print("change airspeed", airspeed)

    def beginTakeoffLand(self, altitude: float) -> None:
        self.commander.takeoff(altitude)
      
    def cost_val_callback(self,msg: Float64):
        self.cost_val = msg.data

    def time_solution_callback(self,msg: Float64):
        self.time_solution = msg.data
        

def main(args=None):
    rclpy.init(args=args)
    
    drone_node = DroneNode()
    
    # drone_commander = drone_node.commander
    # drone_info = drone_node.drone_info
    
    print("connected to drone")
    time_start = get_time_in_secs(drone_node)
    
    rclpy.spin_once(drone_node, timeout_sec=0.1)
    while rclpy.ok():
        try:
            # if drone_node.state_info[0] is None:
            #     print("waiting for telem info")
            #     rclpy.spin(drone_node, timeout_sec=1.0)
            #     continue
            # drone_info.publishTelemInfo()
            #print roll and pitch angles   
            #drone_node.send_airspeed_command(airspeed=25.0)
            # drone_node.sendAttitudeTarget(roll_angle=0.0,
            #                                 pitch_angle=0.0,
            #                                 yaw_angle=-45,
            #                                 thrust=0.5)
            
            rclpy.spin_once(drone_node)
            #rclpy.spin_until_future_complete(drone_node, timeout_sec=0.05)

        #check ctrl+c
        except KeyboardInterrupt:
            #if we get a keyboard interrupt, close the node
            
            #save history to df and csv
            print("saving history to csv")
            print(drone_node.x_traj_history)
            df = pd.DataFrame({
                'current_time': drone_node.time,
                'x_pos': drone_node.x_pos_history,
                'y_pos': drone_node.y_pos_history,
                'z_pos': drone_node.z_pos_history,
                'roll': drone_node.roll_history,
                'pitch': drone_node.pitch_history,
                'yaw': drone_node.yaw_history,
                'vel': drone_node.vel_history,
                'x_traj': drone_node.x_traj_history,
                'y_traj': drone_node.y_traj_history,
                'z_traj': drone_node.z_traj_history,
                'roll_traj': drone_node.roll_traj_history,
                'pitch_traj': drone_node.pitch_traj_history,
                'yaw_traj': drone_node.yaw_traj_history,
                'x_cmd': drone_node.x_cmd_history,
                'y_cmd': drone_node.y_cmd_history,
                'z_cmd': drone_node.z_cmd_history,
                'roll_cmd': drone_node.roll_cmd_history,
                'pitch_cmd': drone_node.pitch_cmd_history,
                'yaw_cmd': drone_node.yaw_cmd_history,
                'vel_cmd': drone_node.vel_cmd_history,
                'effector_dmg': drone_node.effector_dmg_history,
                'effector_x': drone_node.effector_x_history,
                'effector_y': drone_node.effector_y_history,
                'effector_z': drone_node.effector_z_history,
                'cost': drone_node.cost_val_history,
                'time_solution': drone_node.time_solution_history
            })
            #save obstacle avoid params as a df
            df_obs = pd.DataFrame({
                'weight': obs_avoid_params['weight'],
                'safe_distance': obs_avoid_params['safe_distance'],
                'x': obs_avoid_params['x'],
                'y': obs_avoid_params['y'],
                'z': obs_avoid_params['z'],
                'radii': obs_avoid_params['radii']
            })
            
            #check if drone_history.csv exists
            if os.path.exists('drone_history.csv'):
                #append to the csv
                #save file as current
                #get string of current date and time
                date_time = pd.to_datetime('today').strftime('%Y-%m-%d-%H-%M-%S')
                file_name = 'drone_history_' + date_time + '.csv'
                obs_filename = 'obs_avoid_params_' + date_time + '.csv'
            else:
                file_name = 'drone_history'
                obs_filename = 'obs_avoid_params'
                
            df.to_csv(file_name+'.csv')
            df_obs.to_csv(obs_filename+'.csv')
            
            print("Shutting down")
            return 
        

if __name__ == '__main__':
    main()