#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Feb 25 20:35:59 2024

@author: justin
Utility Function class that can take in a dataframe and parse out stuff 
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import ast


def string_to_array(input_str):
    # Extract the type code from the input string
    type_code = input_str.split("('")[1][0]
    # Extract the list of elements from the input string
    elements_str = input_str.split(", [")[1].rstrip("])")
    elements_list = ast.literal_eval(f"[{elements_str}]")
    
    # Create and return the array
    return elements_list


class DataParser():
    def __init__(self, folder_dir:str='figures/', 
                 save_figure:bool=False) -> None:
        
        self.folder_dir = folder_dir
        self.save_figure = save_figure
    
    @staticmethod 
    def get_time_vector(df:pd.DataFrame) -> pd.DataFrame:
        """
        Get the time vector from the dataframe
        """
        time_vector = []
        curr_time = 0
        for t in df['time_solution']:
            curr_time += t
            time_vector.append(curr_time)
        
        df['time'] = time_vector
        return df 
    
    @staticmethod
    def convert_traj_to_enu(df:pd.DataFrame) -> pd.DataFrame:
        #loop through trajectory and convert from string to array
        x_traj = []
        y_traj = []
        z_traj = []
        
        #use list comprehension
        x_traj = [string_to_array(x) for x in df['x_traj']]
        y_traj = [string_to_array(y) for y in df['y_traj']]
        z_traj = [string_to_array(z) for z in df['z_traj']]
            
        df['x_traj'] = y_traj
        df['y_traj'] = x_traj
        df['z_traj'] = z_traj
            
        return df


    def plot_3d_trajectory(self, df:pd.DataFrame, 
                           use_time:bool=False, title:str='3D Trajectory',
                           include_effector=False, save:bool=False) -> None:
        """
        Plot the 3D trajectory of the drone
        """
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        if use_time:
            #color the dots as a function of time
            c = df['time']
            ax.scatter(df['x_pos'], df['y_pos'], df['z_pos'], 
                       c=c, cmap='viridis', label='Trajectory')
        else:
            ax.plot(df['x_traj'], df['y_traj'], df['z_traj'], label='Trajectory')
        
        if include_effector:
            effector_x = df['effector_x']
            effector_y = df['effector_y']
            effector_z = df['effector_z']
            for x_points,y_points,z_points in zip(effector_x, effector_y, effector_z):

                x_points = ast.literal_eval(x_points)
                y_points = ast.literal_eval(y_points)
                z_points = ast.literal_eval(z_points)
                print("x_points: ", x_points)
                print("y_points: ", y_points)
                print("z_points: ", z_points)
                
                for x, y, z in zip(x_points, y_points, z_points):                    
                    ax.plot(x, y, z, c='r', label='Effecto_points')
                
            #stack into a 3xN array
    
        if use_time:
            cbar = plt.colorbar(ax.scatter(df['x_pos'], df['y_pos'], 
                                           df['z_pos'], c=c, cmap='viridis'))
            cbar.set_label('Time (s)')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(title)

        if self.save_figure:
            # plt.savefig(title+'.png')
            self.save_figure_to_dir(fig, title)
        
        return fig, ax
    
    def plot_damage_effector(self, 
                             df:pd.DataFrame, title='Damage Effector vs Time', save:bool=False) -> None:
        """
        Plot the damage effector
        """
        fig, ax = plt.subplots()
        ax.plot(df['time'], df['effector_dmg'])
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Damage Effector')
        ax.set_title(title)
        ax.grid()
        
        #compute the integral of the effector damage
        integral = np.trapz(df['effector_dmg'], df['time'])
        #compute the total time of the engagement
        total_time = df['time'].iloc[-1] - df['time'].iloc[0]
        
        ax.set_title(title + f'\nIntegral: {integral:.2f}, Total Time: {total_time:.2f}')
        
        if self.save_figure:
            # plt.savefig(title+'.png')
            self.save_figure_to_dir(fig, title)
        
        return fig, ax
    
    @staticmethod
    def find_engagement( df:pd.DataFrame) -> pd.DataFrame:
    
        df_engagement = df[(df['effector_dmg'] > 0) ]
        return df_engagement
    
    @staticmethod
    def get_engagement_df(df:pd.DataFrame, time_start:float, 
                            time_end:float) -> float:
        """
        Return the dataframe between the start and end time
        """
        df_engagement = df[(df['time'] >= time_start) & (df['time'] <= time_end)]
        return df_engagement
    
    def plot_position_tracking(self, 
                               df:pd.DataFrame, 
                               title:str='Tracking Performance') -> None:
        """
        Plot the tracking performance
        """
        
        fig, axes = plt.subplots(nrows=3, figsize=(10, 10))
        
        axes[0].plot(df['time'], df['x_pos'], label='X Position', linewidth=2)
        axes[0].plot(df['time'], df['x_cmd'], label='X Reference', linewidth=1,
                   linestyle='--')

        axes[1].plot(df['time'], df['y_pos'], label='Y Position', linewidth=2)
        axes[1].plot(df['time'], df['y_cmd'], label='Y Reference', linewidth=1,
                   linestyle='--')


        axes[2].plot(df['time'], df['z_pos'], label='Z Position', linewidth=2)
        axes[2].plot(df['time'], -df['z_cmd'], label='Z Reference', linewidth=1,
                   linestyle='--')
        
        for a in axes:
            a.legend()
            #set grid to be gray dashed lines
            a.grid(color='gray', linestyle='--', linewidth=0.5)
        
        #compute mse for each axis
        mse_x = np.mean((df['x_pos'] - df['x_cmd'])**2)
        mse_y = np.mean((df['y_pos'] - df['y_cmd'])**2)
        mse_z = np.mean((df['z_pos'] + df['z_cmd'])**2)
            
        axes[0].set_title(title + f'\nMSE X: {mse_x:.2f}, MSE Y: {mse_y:.2f}, MSE Z: {mse_z:.2f}')
        axes[0].set_ylabel('X Position')
        axes[1].set_ylabel('Y Position')
        
        axes[2].set_ylabel('Z Position')
        axes[2].set_xlabel('Time (s)')
        
        #share the x axis
        fig.tight_layout()
        
        if self.save_figure:
            self.save_figure_to_dir(fig, title)
        
        return fig, axes   
    
    def plot_attitude_tracking(self, 
                               df:pd.DataFrame, 
                               title:str='Attitude Tracking Performance') -> None:
        """
        Plot the attitude tracking performance
        """
        
        fig, axes = plt.subplots(nrows=4, figsize=(10, 10))
        
        roll_dg = np.rad2deg(df['roll'])
        roll_cmd_dg = np.rad2deg(df['roll_cmd'])
        
        pitch_dg = np.rad2deg(df['pitch'])
        pitch_cmd_dg = np.rad2deg(df['pitch_cmd'])
        
        yaw_dg = np.rad2deg(df['yaw'])
        yaw_cmd_dg = np.rad2deg(df['yaw_cmd'])
        
        
        
        axes[0].plot(df['time'], roll_dg, label='Roll', linewidth=2)
        axes[0].plot(df['time'], roll_cmd_dg, label='Roll Reference', linewidth=1,
                   linestyle='--')

        axes[1].plot(df['time'], pitch_dg, label='Pitch', linewidth=2)
        axes[1].plot(df['time'], -pitch_cmd_dg, label='Pitch Reference', linewidth=1,
                   linestyle='--')


        axes[2].plot(df['time'], yaw_dg, label='Yaw', linewidth=2)
        axes[2].plot(df['time'], yaw_cmd_dg, label='Yaw Reference', linewidth=1,
                   linestyle='--')
        
        #plot velocity
        axes[3].plot(df['time'], df['vel'], label='Velocity', linewidth=2)
        axes[3].plot(df['time'], df['vel_cmd'], label='Velocity Reference', linewidth=1,    
                   linestyle='--')  
        
        for a in axes:
            a.legend()
            #set grid to be gray dashed lines
            a.grid(color='gray', linestyle='--', linewidth=0.5)
        
        #compute mse for each axis
        mse_roll = np.mean((roll_dg - roll_cmd_dg)**2)
        mse_pitch = np.mean((pitch_dg + pitch_cmd_dg)**2)
        mse_yaw = np.mean((yaw_dg - yaw_cmd_dg)**2)
        mse_vel = np.mean((df['vel'] - df['vel_cmd'])**2)
        
        
        axes[0].set_title(title + f'\nMSE Roll: {mse_roll:.2f}, MSE Pitch: {mse_pitch:.2f}, MSE Yaw: {mse_yaw:.2f}, MSE Vel: {mse_vel:.2f}')

        axes[0].set_ylabel('Roll')
        axes[1].set_ylabel('Pitch')
        
        axes[2].set_ylabel('Yaw')
        axes[2].set_xlabel('Time (s)')
        
        axes[3].set_ylabel('Velocity (m/s)')
        
        #share the x axis
        fig.tight_layout()
        
        if self.save_figure:
            # plt.savefig(title+'.png')
            self.save_figure_to_dir(fig, title)

        return fig, axes
    
    def plot_solution_time_mpc(self, 
                               df:pd.DataFrame, 
                               title:str='MPC Solution Time') -> None:
        """
        Plot the solution time for the MPC
        """
        fig, ax = plt.subplots()
        ax.plot(df['time_solution'])
        ax.set_xlabel('Iterations (s)')
        ax.set_ylabel('Solution Time (s)')
        ax.set_title(title)
        ax.grid()
        
        if self.save_figure:
            self.save_figure_to_dir(fig, title)
            
        return fig, ax
    

    def save_figure_to_dir(self, fig:plt.Figure, title:str) -> None:
        """
        Save the figure
        """
        folder = self.folder_dir
        full_path = folder + title + '.png'
        fig.savefig(full_path)
        return None