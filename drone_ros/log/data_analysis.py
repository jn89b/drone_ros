#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Feb 25 20:35:59 2024

@author: justin
"""

import pandas as pd
import re

import matplotlib.pyplot as plt

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

df = pd.read_csv('avoid_traj_states.csv')
mpc_traj = pd.read_csv('avoid_traj_history.csv')

#get first row
idx = 30
first_row = df.iloc[idx] 
x = first_row['x']
y = first_row['y']
z = first_row['z']

first_traj_row = mpc_traj.iloc[idx]
#convert string to numpy array
x_traj = turn_to_float(first_traj_row['x'])
y_traj = turn_to_float(first_traj_row['y'])
z_traj = turn_to_float(first_traj_row['z'])


#plot the trajectory
fig, ax = plt.subplots()
ax.scatter(x, y, label='Drone')
ax.plot(x_traj, y_traj, label='MPC')

ax.set(xlabel='x', ylabel='y',
         title='Drone Trajectory')
ax.grid()
ax.legend()


x_history = df['x']
y_history = df['y']

fig,ax = plt.subplots()
ax.scatter(x_history, y_history, label='Drone')


plt.show()