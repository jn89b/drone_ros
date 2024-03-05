#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Feb 25 20:35:59 2024

@author: justin

Show the following 

"""

import pandas as pd
import re
import numpy as np
import matplotlib.pyplot as plt
import ast

from DataParser import DataParser

GOAL_X = 200
GOAL_Y = 200
GOAL_Z = 50

def string_to_array(input_str):
    # Extract the type code from the input string
    type_code = input_str.split("('")[1][0]
    # Extract the list of elements from the input string
    elements_str = input_str.split(", [")[1].rstrip("])")
    elements_list = ast.literal_eval(f"[{elements_str}]")
    
    # Create and return the array
    return elements_list

def turn_to_float(num_str) -> list:
    # Step 1: Remove the square brackets
    num_str_no_brackets = num_str.strip("[]")

    # Step 2: Replace newlines with spaces and remove extra spaces (optional step, mainly for clarity)
    num_str_cleaned = re.sub(r'\s+', ' ', num_str_no_brackets)

    # Step 3: Split the string into a list of substrings
    num_list_str = num_str_cleaned.split()

    # Step 4: Convert each substring to a float
    num_list_float = [float(num) for num in num_list_str]
    
    return num_list_float

file_names = ['vanilla_mpc', 'direct_traj_17', 'slow_slow', 'bang_bang']
folder_dir = 'figures/'

cool_results = [
    'obs_avoid_2',
    'traj_obs_avoid',
    'obs_avoid_3',
]

obstacle_files = [
    'obs_2',
    'obs_4',
    'obstacle_config_1',
    'obstacle_config_2',
    'simple_obs',
    'walled_obstacles'
]

PLOT_OBSTACLE = True

file_name = 'slow_slow'
file_desired = 'walled_direct_traj_3'
df = pd.read_csv(file_desired+'.csv',  na_filter=True, na_values='[]')

if PLOT_OBSTACLE:
    obs_df = pd.read_csv(obstacle_files[-1]+'.csv')

df.dropna()
df.dropna(subset=['x_traj'], inplace=True)
df.dropna(subset=['effector_x'], inplace=True)


data_parser = DataParser(save_figure=False, folder_dir=folder_dir)
df = data_parser.get_time_vector(df)    
df = data_parser.cleanup_effector_data(df)
df = data_parser.convert_traj_to_enu(df)
plt.close('all')


#%% 
#plot the 3D trajectory
fig, ax = data_parser.plot_3d_trajectory(df, use_time=True, title='Overall Aircraft Trajectory')

if PLOT_OBSTACLE:
    ax = data_parser.plot_obstacles_3D(obs_df, ax)
    # data_parser.plot_cylinder(ax, obs_x, obs_y, obs_z, obs_radius, obs_height)

ax.scatter(GOAL_X, GOAL_Y, GOAL_Z, label='Goal', color='r')
ax = data_parser.plot_goal_as_cylinder(goal_x=GOAL_X, goal_y=GOAL_Y, 
                                       current_ax=ax, radius=1.0,
                                       z_low=GOAL_Z-10, z_high=GOAL_Z+10)

ax.set_xlim(-200, 250)
ax.set_ylim(-200, 250)
ax.set_zlim(0, 100)
ax.legend()

fig , ax = data_parser.plot_solution_time_mpc(df, title='Solution Time for MPC')
fig, ax = data_parser.plot_position_tracking(df, title='Position Tracking Performance')
fig , ax = data_parser.plot_attitude_tracking(df, title='Attitude Tracking Performance')

dx = GOAL_X - df['x_pos']
dy = GOAL_Y - df['y_pos']
distance = np.sqrt(dx**2 + dy**2)
fig,ax = plt.subplots()
ax.plot(df['time'], distance, label='Distance to Goal')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Distance (m)')

#%% Look at the engagement
# engagement_df = data_parser.get_engagement_df(df, 23.5, 25)
engagement_df = data_parser.find_engagement(df,end_buffer=100)
fig, ax, actual_engagement = data_parser.plot_damage_effector(engagement_df)

#get time vector
delta_t = actual_engagement['time'].values[-1] - actual_engagement['time'].values[0]
print(f'Time of engagement: {delta_t} seconds')
prf = 1000
#find how many shots I can take for the engagement 
num_shots = prf*delta_t
print(f'Number of shots: {num_shots}')
wingspan = 3.0

fig, ax = data_parser.plot_3d_trajectory(engagement_df, use_time=True, 
                                         title='Engagement Trajectory', 
                                         plot_effector=True,
                                         plot_robot_radius=True,
                                         robot_radius_m=wingspan/2,
                                        )

ax.scatter(GOAL_X, GOAL_Y, GOAL_Z, label='Goal', color='r')
ax = data_parser.plot_goal_as_cylinder(goal_x=GOAL_X, goal_y=GOAL_Y, 
                                       current_ax=ax, radius=1.0,
                                       z_low=GOAL_Z-10, z_high=GOAL_Z+10)
if PLOT_OBSTACLE:
    ax = data_parser.plot_obstacles_3D(obs_df, ax)

fig, ax = data_parser.plot_position_tracking(engagement_df, 
                                            title='Engagement Position Tracking Performance')

fig,ax = data_parser.plot_attitude_tracking(engagement_df, 
                                            title='Engagement Attitude Tracking Performance')

#print total time 
print(f'Total time: {df["time"].values[-1]} seconds')

#%% 
plt.show()