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


file_names = ['vanilla_mpc', 'directional_mpc', 'drone_history']
folder_dir = 'figures/'

file_desired = file_names[2]
df = pd.read_csv(file_desired+'.csv')
df.dropna(subset=['x_traj'], inplace=True)



data_parser = DataParser(save_figure=True, folder_dir=folder_dir)
df = data_parser.get_time_vector(df)
df = data_parser.convert_traj_to_enu(df)
plt.close('all')

#%% 
#plot the 3D trajectory
fig, ax = data_parser.plot_3d_trajectory(df, use_time=True, title='Overall Aircraft Trajectory')
ax.scatter(GOAL_X, GOAL_Y, GOAL_Z, label='Goal', color='r')
ax.set_xlim(-200, 200)
ax.set_ylim(-200, 200)
ax.set_zlim(0, 100)
ax.legend()

fig , ax = data_parser.plot_solution_time_mpc(df, title='Solution Time for MPC')

fig, ax = data_parser.plot_position_tracking(df, title='Position Tracking Performance')
fig , ax = data_parser.plot_attitude_tracking(df, title='Attitude Tracking Performance')


#%% Look at the engagement
# engagement_df = data_parser.get_engagement_df(df, 23.5, 25)
engagement_df = data_parser.find_engagement(df)
fig, ax = data_parser.plot_damage_effector(engagement_df)
ax.set_ylim(0, 0.5)

fig, ax = data_parser.plot_3d_trajectory(engagement_df, use_time=True, 
                                         title='Engagement Trajectory')
ax.scatter(GOAL_X, GOAL_Y, GOAL_Z, label='Goal', color='r')


fig, ax = data_parser.plot_position_tracking(engagement_df, 
                                                title='Engagement Position Tracking Performance')

fig,ax = data_parser.plot_attitude_tracking(engagement_df, title='Engagement Attitude Tracking Performance')



#%% 
plt.show()