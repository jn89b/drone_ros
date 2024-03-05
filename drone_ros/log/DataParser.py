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
        Get the time vector from the dataframe and zero it out
        """
        time_vector = []
        start_time = df['current_time'].iloc[0]
        for t in df['current_time']:
            time_vector.append(t - start_time)
            
        # curr_time = 0
        # for t in df['time_solution']:
        #     curr_time += t
        #     time_vector.append(curr_time)
        
        df['time'] = time_vector
        return df 
    
    def plot_robot_radius_3d(self, df:pd.DataFrame, ax:plt.axes,
                             radius:float=2,
                             color:str='orange', alpha_val:float=0.2, 
                             num_points:int=20) -> plt.axes:
        x_pos = df['x_pos']
        y_pos = df['y_pos']
        z_pos = df['z_pos']
        
        #plot a 2d circle at each point
        for x,y,z in zip(x_pos,y_pos,z_pos):
            theta = np.linspace(0, 2*np.pi, num_points)
            x_circle = radius * np.cos(theta) + x
            y_circle = radius * np.sin(theta) + y
            ax.plot(x_circle, y_circle, z, color=color, alpha=alpha_val)

        return ax
    
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

    def cleanup_effector_data(self, df:pd.DataFrame) -> pd.DataFrame:
        """
        effector_x is [x1,y1,z1] effector_y is [x2,y2,z2] and so on
        loop through dataframe and extract the effector locations
        """
        effector_x = []
        effector_y = []
        effector_z = []
        for index, row in df.iterrows():
            effector_x_points = []
            effector_y_points = []
            effector_z_points = []
            first_point = ast.literal_eval(row['effector_x'])
            second_point = ast.literal_eval(row['effector_y'])
            third_point = ast.literal_eval(row['effector_z'])
                        
            effector_x_points.append((first_point[0], second_point[0], third_point[0]))
            effector_y_points.append((first_point[1], second_point[1], third_point[1]))
            effector_z_points.append((first_point[2], second_point[2], third_point[2]))
            
            effector_x.append(effector_x_points)
            effector_y.append(effector_y_points)
            effector_z.append(effector_z_points)
            
            
        #replace eachw row with the new effector data
        df['effector_x'] = effector_x 
        df['effector_y'] = effector_y
        df['effector_z'] = effector_z
        
        return df
        
    def plot_3d_trajectory(self, df:pd.DataFrame, 
                           use_time:bool=False, title:str='3D Trajectory',
                           include_effector=False, save:bool=False,
                           plot_effector:bool=False,
                           plot_robot_radius:bool=False,
                           robot_radius_m:float=2.0) -> None:
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

                for x, y, z in zip(x_points, y_points, z_points):                    
                    ax.plot(x, y, z, c='r', label='Effecto_points')
                
            #stack into a 3xN array
    
        if use_time:
            cbar = plt.colorbar(ax.scatter(df['x_pos'], df['y_pos'], 
                                           df['z_pos'], c=c, cmap='viridis'))
            cbar.set_label('Time (s)')
        
        if plot_effector:
            fig, ax = self.plot_effector_locations(df, fig, ax)
        
        
        if plot_robot_radius:
            ax = self.plot_robot_radius_3d(df, ax, radius=robot_radius_m)
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(title)
        ax.legend()

        if self.save_figure:
            # plt.savefig(title+'.png')
            self.save_figure_to_dir(fig, title)
        
        return fig, ax
    
    def plot_damage_effector(self, df:pd.DataFrame, 
                             title='Damage Effector vs Time', 
                             save:bool=False) -> None:
        """
        Plot the damage effector
        """
        fig, ax = plt.subplots()
        ax.plot(df['time'], df['effector_dmg'])
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Damage Effector')
        ax.set_title(title)
        ax.grid()
        
        engagement_df = df[(df['effector_dmg'] >= -0.01)]
        
        #compute the integral of the effector damage
        integral = np.trapz(engagement_df['effector_dmg'], engagement_df['time'])
        #compute the total time of the engagement
        total_time = engagement_df['time'].iloc[-1] - engagement_df['time'].iloc[0]
        
        ax.set_title(title + f'\nIntegral: {integral:.2f}, Total Time: {total_time:.2f}')
        
        if self.save_figure:
            # plt.savefig(title+'.png')
            self.save_figure_to_dir(fig, title)
        
        return fig, ax, engagement_df
    
    @staticmethod
    def find_engagement( df:pd.DataFrame, start_buffer:int=-1, 
                        end_buffer:int=50) -> pd.DataFrame:
    
        #get the index right before the engagement
        idx_engagement = df[(df['effector_dmg'] >= -0.01)].index[0]
        
        #get the index right after the engagement
        idx_end_engagement = df[(df['effector_dmg'] >= -0.01)].index[-1]
        
        #get datapoints after the engagement
        
        df_engagement = df.iloc[idx_engagement-start_buffer:idx_end_engagement+end_buffer]

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
        #set y limit to be from 0 to 0.5
        ax.set_ylim(0, 0.5)
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
    

    def plot_obstacles_3D(self, obs_df:pd.DataFrame, current_ax,
                        z_low:float=0, z_high:float=100,
                        num_points:int=20,
                        color_obs:str='r',
                        alpha_val:float=0.2):
        """
        Plot as a cylinder
        """
        
        x_list = obs_df['x']
        y_list = obs_df['y']
        radii_list = obs_df['radii']
        
        for x,y,r in zip(x_list, y_list, radii_list):
            # Cylinder parameters
            radius = r
            #height = z_high - z_low
            center = [x, y]
            
            theta = np.linspace(0, 2*np.pi, num_points)
            z = np.linspace(z_low, z_high, num_points)
            
            theta, z = np.meshgrid(theta, z)
            x_vector = radius * np.cos(theta) + center[0]
            y_vector = radius * np.sin(theta) + center[1]
            
            current_ax.plot_surface(x_vector, y_vector, z, 
                            color=color_obs, alpha=alpha_val)
            
        return current_ax
    
    def plot_goal_as_cylinder(self, 
                              goal_x:float, 
                              goal_y:float,
                              current_ax:plt.axes, 
                              z_low:float=0, 
                              z_high:float=100,
                              radius:float=5,
                              num_points:int=20,
                              color_obs:str='r',
                              alpha_val:float=0.2) -> None:
        
        radius = radius
        center = [goal_x, goal_y]
        theta = np.linspace(0, 2*np.pi, num_points)
        z = np.linspace(z_low, z_high, num_points)
        
        theta, z = np.meshgrid(theta, z)
        x_vector = radius * np.cos(theta) + center[0]
        y_vector = radius * np.sin(theta) + center[1]
        
        current_ax.plot_surface(x_vector, y_vector, z,
                                color=color_obs, alpha=alpha_val)
        
        return current_ax
        
    
    def plot_effector_locations(self, df:pd.DataFrame, fig:plt.Figure, 
                                ax:plt.axes) -> None:
        """
        Plot the effector locations
        """
        for i, (x,y,z) in enumerate(zip(
            df['effector_x'], df['effector_y'], df['effector_z'])):
            x1 = x[0][0]
            x2 = x[0][1]
            x3 = x[0][2]
            
            y1 = y[0][0]
            y2 = y[0][1]
            y3 = y[0][2]
            
            z1 = z[0][0]
            z2 = z[0][1]
            z3 = z[0][2]
            
            x = [x1, x2, x3, x1]
            y = [y1, y2, y3, y1]
            z = [z1, z2, z3, z1]
            
            ax.plot(x, y, z, alpha=0.5, color="lightblue",  label='Effector Locations' if i == 0 else "")
        return fig, ax