#!/usr/bin/env python3

"""
We want to see the difference in commands between
the math model and the real drone

I need to make sure the mapping between the math model
and the real drone is correct

So from math mode to drone commander:
    - Map from ENU to NED
    - Yaw command from global to local frame (ENU to NED)  

Send controls from math model to drone commander 
and make sure the following "looks" the same:

How do I map the 

"""
import numpy as np
import rclpy
import matplotlib.pyplot as plt
import time 
from pymavlink import mavutil
from rclpy.node import Node
from typing import List, Dict, Any
from drone_ros.Commander import Commander
from drone_ros.MathModel import PlaneKinematicModel
from drone_interfaces.msg import Telem
from drone_ros.quaternion_tools import (
    ned_to_enu_states,
    enu_to_ned_states,
    enu_to_ned_controls,
    get_relative_ned_yaw_cmd,
    yaw_ned_to_enu,
    yaw_enu_to_ned
)
from rclpy.subscription import Subscription


import pandas as pd

class DataCollector:
    def __init__(self):
        self.aircraft_roll: List[float] = []
        self.aircraft_pitch: List[float] = []
        self.aircraft_yaw: List[float] = []
        self.aircraft_velocity: List[float] = []
        self.aircraft_x: List[float] = []
        self.aircraft_y: List[float] = []
        self.aircraft_z: List[float] = []
        
        self.model_roll: List[float] = []
        self.model_pitch: List[float] = []
        self.model_yaw: List[float] = []
        self.model_velocity: List[float] = []
        self.model_x: List[float] = []
        self.model_y: List[float] = []
        self.model_z: List[float] = []
         
        self.u_phi: List[float] = []
        self.u_theta: List[float] = []
        self.u_psi: List[float] = []
        self.u_velocity: List[float] = []
        
        self.mpc_roll: List[float] = []
        self.mpc_pitch: List[float] = []
        self.mpc_yaw: List[float] = []
        self.mpc_velocity: List[float] = []
        self.mpc_x: List[float] = []
        self.mpc_y: List[float] = []
        self.mpc_z: List[float] = []
        
        self.mpc_u_phi: List[float] = []
        self.mpc_u_theta: List[float] = []
        self.mpc_u_psi: List[float] = []
        self.mpc_u_velocity: List[float] = []
        
        
    def collect_aircraft_data(self, roll: float, 
                              pitch: float, 
                              yaw: float, 
                              velocity: float,
                              x:float,
                              y:float,
                              z:float) -> None:
        self.aircraft_roll.append(roll)
        self.aircraft_pitch.append(pitch)
        self.aircraft_yaw.append(yaw)
        self.aircraft_velocity.append(velocity)
        self.aircraft_x.append(x)
        self.aircraft_y.append(y)
        self.aircraft_z.append(z)

    def collect_model_data(self, 
                           roll: float, 
                           pitch: float, 
                           yaw: float, 
                           velocity: float,
                           x:float,
                           y:float,
                           z:float) -> None:
        self.model_roll.append(roll)
        self.model_pitch.append(pitch)
        self.model_yaw.append(yaw)
        self.model_velocity.append(velocity)
        self.model_x.append(x)
        self.model_y.append(y)
        self.model_z.append(z)
        
    def collect_controls(self, controls: np.array) -> None:
        self.u_phi.append(controls[0])
        self.u_theta.append(controls[1])
        self.u_psi.append(controls[2])
        self.u_velocity.append(controls[3])
        
    def collect_mpc_data(self,
                            roll: float,
                            pitch: float,
                            yaw: float,
                            velocity: float,
                            x:float,
                            y:float,
                            z:float) -> None:
        self.mpc_roll.append(roll)
        self.mpc_pitch.append(pitch)
        self.mpc_yaw.append(yaw)
        self.mpc_velocity.append(velocity)
        self.mpc_x.append(x)
        self.mpc_y.append(y)
        self.mpc_z.append(z)
        
    def collect_mpc_controls(self, controls: np.array) -> None:
        self.mpc_u_phi.append(controls[0])
        self.mpc_u_theta.append(controls[1])
        self.mpc_u_psi.append(controls[2])
        self.mpc_u_velocity.append(controls[3])
        
    def save_csv(self):
        # save the data into a csv
        info:Dict[str, List[np.array]] = {
            'aircraft_roll': self.aircraft_roll,
            'aircraft_pitch': self.aircraft_pitch,
            'aircraft_yaw': self.aircraft_yaw,
            'aircraft_velocity': self.aircraft_velocity,
            'aircraft_x': self.aircraft_x,
            'aircraft_y': self.aircraft_y,
            'aircraft_z': self.aircraft_z,
            'model_roll': self.model_roll,
            'model_pitch': self.model_pitch,
            'model_yaw': self.model_yaw,
            'model_velocity': self.model_velocity,
            'model_x': self.model_x,
            'model_y': self.model_y,
            'model_z': self.model_z,
            'u_phi': self.u_phi,
            'u_theta': self.u_theta,
            'u_psi': self.u_psi,
            'u_velocity': self.u_velocity,
            'mpc_roll': self.mpc_roll,
            'mpc_pitch': self.mpc_pitch,
            'mpc_yaw': self.mpc_yaw,
            'mpc_velocity': self.mpc_velocity,
            'mpc_x': self.mpc_x,
            'mpc_y': self.mpc_y,
            'mpc_z': self.mpc_z,
            'mpc_u_phi': self.mpc_u_phi,
            'mpc_u_theta': self.mpc_u_theta,
            'mpc_u_psi': self.mpc_u_psi,
            'mpc_u_velocity': self.mpc_u_velocity
        }
        
        df = pd.DataFrame(info)
        df.to_csv('drone_data.csv')
        print("Data has been saved")

class InfoNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        self.get_logger().info('Drone node has been started')
        self.data_collector: DataCollector = DataCollector()
        self.drone_info_subscriber:Subscription = self.create_subscription(
            Telem,
            'telem',
            self.telem_callback,
            10,
        )
        self.current_state_ned: np.array = np.zeros(7)
        
    def telem_callback(self, msg: Telem) -> None:
        roll:float = msg.roll
        pitch:float = msg.pitch
        yaw:float = msg.yaw
        vx:float = msg.vx
        vy:float = msg.vy
        vz:float = msg.vz
        velocity:float = np.sqrt(vx**2 + vy**2 + vz**2)
        x:float = msg.x
        y:float = msg.y
        z:float = msg.z
        print("x , y, z", x, y, z)
        self.current_state_ned[0] = x
        self.current_state_ned[1] = y
        self.current_state_ned[2] = z
        self.current_state_ned[3] = roll
        self.current_state_ned[4] = pitch
        self.current_state_ned[5] = yaw
        self.current_state_ned[6] = velocity
        
def compute_pursuit_angle(enu_state:np.array,
                          target_x, 
                          target_y) -> float:
    """
    Yaw command is in global frame in ENU convention
    """
    dx:float = target_x - enu_state[0]
    dy:float = target_y - enu_state[1]
    enu_yaw_cmd:float = np.arctan2(dy, dx)
    distance = np.sqrt(dx**2 + dy**2)
    print("distance is: ", distance)
    return enu_yaw_cmd

def main(args=None) -> None:
    rclpy.init(args=args)
    
    node_info: InfoNode = InfoNode()
    connection_str: str = "udp:127.0.0.1:14552"
    master:mavutil = mavutil.mavlink_connection(connection_str)
    master.wait_heartbeat()

    ENU_X:float = 250.0
    ENU_Y:float = 0.0

    print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
    plane_model: PlaneKinematicModel = PlaneKinematicModel()
    commander: Commander = Commander(
        master=master,
    )
    commander.sendAttitudeTarget()
    
    n_steps: int = 360
    history: List = []
    
    roll_angle: float = 0
    pitch_angle: float = -5
    yaw_angle: float = 0
    velocity: float = 25
    
    # make a frequency of roll angle comands

    # Ned yaw_angle is relative commands to the aircraft 
    ned_commands: np.array = [roll_angle, pitch_angle, yaw_angle, velocity]
    duration_time: float = 20.0
    start_time: float = time.time()
    # for i in range(n_steps):
    rclpy.spin_once(node_info, timeout_sec=0.1)
    roll_angles = np.linspace(-np.pi/4, np.pi/4, n_steps)
    
    while time.time() - start_time < duration_time:
   
        rclpy.spin_once(node_info, timeout_sec=0.05)
        node_info.data_collector.collect_aircraft_data(
            roll=node_info.current_state_ned[3],
            pitch=node_info.current_state_ned[4],
            yaw=node_info.current_state_ned[5],
            velocity=node_info.current_state_ned[6],
            x=node_info.current_state_ned[0],
            y=node_info.current_state_ned[1],
            z=node_info.current_state_ned[2]
        )
        print("roll is: ", np.rad2deg(node_info.current_state_ned[3]))
        enu_states = ned_to_enu_states(node_info.current_state_ned)
        node_info.data_collector.collect_model_data(
            roll=enu_states[3],
            pitch=enu_states[4],
            yaw=enu_states[5],
            velocity=enu_states[6],
            x=enu_states[0],
            y=enu_states[1],
            z=enu_states[2]
        )
        
        ## SENDING THE COMMAND TO THE DRONE 
        node_info.data_collector.collect_controls(ned_commands)
        commander.sendAttitudeTarget(
            roll_angle_dg=roll_angle,
            pitch_angle_dg=pitch_angle,
            yaw_angle_dg=yaw_angle,
            thrust=0.5,
        )        
        
        #### MPC MODEL INPUTS  
        ## This thing takes it in ENU coordinates 
        # So transform NED To ENU coordinates
        # Also the controls for the plane_model 
        # are global yaw commands in ENU convention 
        enu_model_yaw_cmd = compute_pursuit_angle(
            enu_state=enu_states,
            target_x=ENU_X,
            target_y=ENU_Y
        )
        # model_yaw_cmd = get_model_yaw_command(ned_commands[2], 
        #                                       node_info.current_state_ned[5])
        # now we need to convert this to ENU frame
        # enu_model_yaw_cmd = yaw_ned_to_enu(model_yaw_cmd)
        enu_controls = enu_to_ned_controls(
            enu_controls=np.array([ned_commands[0], 
                                   ned_commands[1], 
                                   enu_model_yaw_cmd, 
                                   ned_commands[3]]),
            change_yaw=False
        )
        mpc_model_state:np.array = plane_model.rk45(
            x= enu_states,
            u= enu_controls,
            dt=0.1
        )
        node_info.data_collector.collect_mpc_data(
            x=mpc_model_state[0],
            y=mpc_model_state[1],
            z=mpc_model_state[2],
            roll=mpc_model_state[3],
            pitch=mpc_model_state[4],
            yaw=mpc_model_state[5],
            velocity=mpc_model_state[6]
        )
        node_info.data_collector.collect_mpc_controls(enu_controls)
        
        # Now I need to output the enu controls to ned controls
        # as well as ned states from the math model
        ned_yaw_inert_cmd:float = yaw_enu_to_ned(enu_model_yaw_cmd)
        ned_yaw_cmd = get_relative_ned_yaw_cmd(node_info.current_state_ned[5], 
                                               ned_yaw_inert_cmd)
        yaw_angle = np.rad2deg(ned_yaw_cmd)
        kp = 0.2
        roll_angle = kp*yaw_angle
        
        ned_states = enu_to_ned_states(mpc_model_state)

    # save the data into a csv
    # convert the data into a pandas dataframe
    # save the data into a csv
    node_info.data_collector.save_csv()

if __name__ == "__main__":
    main()