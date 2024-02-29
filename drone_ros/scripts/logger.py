#!/usr/bin/env python
# -*- coding: utf-8 -*- 


from ipaddress import ip_address
import pandas as pd
import numpy as np
import rclpy
from rclpy.node import Node
import threading

import time
import csv
import os
import datetime

from nav_msgs.msg import Odometry

from drone_ros import quaternion_tools
from drone_interfaces.msg import Telem, CtlTraj

from mpc_ros import Config

import mavros 
from mavros.base import SENSOR_QOS
from geometry_msgs.msg import PoseArray


from std_msgs.msg import Float64


def get_time_in_secs(some_node:Node) -> float:
	return some_node.get_clock().now().nanoseconds /1E9
	

class OdomLocNode(Node):
	def __init__(self):
		super().__init__('logger_node')
		self.current_position = [None,None]
		self.orientation_euler = [None,None,None]
		self.state_sub = self.create_subscription(
						mavros.local_position.Odometry, 
						'/mavros/local_position/odom', 
						self.odom_callback, 
						qos_profile=SENSOR_QOS)
		
		
		self.telem_sub = self.create_subscription(
			Telem,
			'telem',
			self.tele_cb,
			30)
		
		self.telem_position = [None,None,None]
		self.telem_attitudes = [None,None,None]

	def odom_callback(self,msg):
		"""subscribe to odometry"""
		self.current_position[0] = msg.pose.pose.position.x
		self.current_position[1] = msg.pose.pose.position.y
		
		qx = msg.pose.pose.orientation.x
		qy = msg.pose.pose.orientation.y
		qz = msg.pose.pose.orientation.z
		qw = msg.pose.pose.orientation.w
		
		roll,pitch,yaw = quaternion_tools.get_euler_from_quaternion(qx, qy, qz, qw)
		
		self.orientation_euler[0] = roll
		self.orientation_euler[1] = pitch 
		self.orientation_euler[2] = yaw		
		# print("yaw is", np.degrees(self.orientation_euler[2]))

	def tele_cb(self,msg):
		self.telem_position[0] = msg.x
		self.telem_position[1] = msg.y
		self.telem_position[2] = msg.yaw

		self.telem_attitudes[0] = msg.roll
		self.telem_attitudes[1] = msg.pitch
		self.telem_attitudes[2] = msg.yaw

class LoggerNode(Node):

	def __init__(self):
		super().__init__('logger_node')
		rate_val = 50
		self.state_sub = self.create_subscription(
						mavros.local_position.Odometry, 
						'/mavros/local_position/odom', 
						self.odom_callback, 
						qos_profile=SENSOR_QOS)
		
		self.traj_sub = self.create_subscription(
						CtlTraj,
						'waypoint_trajectory',
						self.ctl_traj_callback,
      					rate_val)
  
		self.effector_dmg_sub = self.create_subscription(
						Float64,
						'/damage_info',
						self.effector_dmg_callback,
						rate_val)
  
		self.effector_location_sub = self.create_subscription(
			PoseArray, 
			'/effector_location',
			self.effector_pos_callback,
			rate_val)
  
		# self.cost_val_sub = self.create_subscription(
      	# 		Float64,
		# 	'/waypoint_cost_val',
		# 	self.cost_val_callback,
		# 	rate_val)
  
		self.time_solution_sub = self.create_subscription(
      		Float64,
			'/waypoint_time_sol',
			self.time_solution_callback,
			rate_val)
  
        
		self.__init_variables()
  
		self.x_curr = 0
		self.y_curr = 0
		self.z_curr = 0
  
		self.roll_curr = 0
		self.pitch_curr = 0
		self.yaw_curr = 0
		self.vel_curr = 0
  
		self.x_traj_curr = None
		self.y_traj_curr = None
		self.z_traj_curr = None
		self.roll_traj_curr = None
		self.pitch_traj_curr = None
		self.yaw_traj_curr = None

		self.x_cmd_curr = 0
		self.y_cmd_curr = 0
		self.z_cmd_curr = 0
		self.roll_cmd_curr = 0
		self.pitch_cmd_curr = 0
		self.yaw_cmd_curr = 0
		self.vel_cmd_curr = 0
  
		self.effector_dmg_curr = 0
		self.effector_x = []
		self.effector_y = []
		self.effector_z = []
  
		self.cost_val = 0
		self.time_solution = 0
  
		self.current_time = 0
  

	def __init_variables(self) -> None:	
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

	def odom_callback(self,msg):
		"""subscribe to odometry"""
		# self.time.append(get_time_in_secs(self))
		# self.x_pos_history.append(msg.pose.pose.position.x)
		# self.y_pos_history.append(msg.pose.pose.position.y)
		# self.z_pos_history.append(msg.pose.pose.position.z)
		self.x_curr = msg.pose.pose.position.x
		self.y_curr = msg.pose.pose.position.y
		self.z_curr = msg.pose.pose.position.z
  
		qx = msg.pose.pose.orientation.x
		qy = msg.pose.pose.orientation.y
		qz = msg.pose.pose.orientation.z
		qw = msg.pose.pose.orientation.w
		
		self.roll_curr,self.pitch_curr,self.yaw_curr = quaternion_tools.get_euler_from_quaternion(
      		qx, qy, qz, qw)

		self.vel_curr = msg.twist.twist.linear.x

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

	def ctl_traj_callback(self,msg: CtlTraj):
		self.x_traj_curr = msg.x
		self.y_traj_curr = msg.y
		self.z_traj_curr = msg.z
		self.roll_traj_curr = msg.roll
		self.pitch_traj_curr = msg.pitch	
		self.yaw_traj_curr = msg.yaw

		self.x_cmd_curr = msg.x[msg.idx]
		self.y_cmd_curr = msg.y[msg.idx]
		self.z_cmd_curr = msg.z[msg.idx]
		self.roll_cmd_curr = msg.roll[msg.idx]
		self.pitch_cmd_curr = msg.pitch[msg.idx]
		self.yaw_cmd_curr = msg.yaw[msg.idx]
		self.vel_cmd_curr = msg.vx[msg.idx]

		# self.x_traj_history.append(msg.x)
		# self.y_traj_history.append(msg.y)
		# self.z_traj_history.append(msg.z)
		
		# self.roll_traj_history.append(msg.roll)
		# self.pitch_traj_history.append(msg.pitch)
		# self.yaw_traj_history.append(msg.yaw)
  
		# self.x_cmd_history.append(msg.x[msg.idx])
		# self.y_cmd_history.append(msg.y[msg.idx])
		# self.z_cmd_history.append(msg.z[msg.idx])

		# self.roll_cmd_history.append(msg.roll[msg.idx])
		# self.pitch_cmd_history.append(msg.pitch[msg.idx])
		# self.yaw_cmd_history.append(msg.yaw[msg.idx])

	def update_history(self) -> None:
		"""update the history of the state and control variables"""
		self.x_pos_history.append(self.x_curr)
		self.y_pos_history.append(self.y_curr)
		self.z_pos_history.append(self.z_curr)
		self.roll_history.append(self.roll_curr)
		self.pitch_history.append(self.pitch_curr)
		self.yaw_history.append(self.yaw_curr)
		self.vel_history.append(self.vel_curr)
  
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
  
  # self.effector_pos_history.append(self.effector_pos_curr)
  
		self.cost_val_history.append(self.cost_val)
		# self.time_solution_history.append(self.time_solution)  

		# self.time.append(get_time_in_secs(self))
  
	def convert_to_dataframe(self):
		# convert to dataframe
		info_dict = {
			'time': self.time,
			'x': self.x_pos_history,
			'y': self.y_pos_history,
			'z': self.z_pos_history,
			'roll': self.roll_history,
			'pitch': self.pitch_history,
			'yaw': self.yaw_history,
			'vel': self.vel_history,
			'x_traj': self.x_traj_history,
			'y_traj': self.y_traj_history,
			'z_traj': self.z_traj_history,
			'roll_traj': self.roll_traj_history,
			'pitch_traj': self.pitch_traj_history,
			'yaw_traj': self.yaw_traj_history,
			'x_cmd': self.x_cmd_history,
			'y_cmd': self.y_cmd_history,
			'z_cmd': self.z_cmd_history,
			'roll_cmd': self.roll_cmd_history,
			'pitch_cmd': self.pitch_cmd_history,
			'yaw_cmd': self.yaw_cmd_history,
			'vel_cmd': self.vel_cmd_history,
			'effector_dmg': self.effector_dmg_history,
			'effector_x': self.effector_x_history,
			'effector_y': self.effector_y_history,
			'effector_z': self.effector_z_history,
			'cost_val': self.cost_val_history,
			'time_solution': self.time_solution_history
			# 'obstacles': self.obstacles
			}
				
		df = pd.DataFrame.from_dict(info_dict, orient='index')
		df = df.transpose()
		
		return df

	def save_to_csv(self,df, file_name):
		# save to csv
		df.to_csv(file_name)

	def cost_val_callback(self,msg: Float64):
		self.cost_val = msg.data
		self.cost_val_history.append(self.cost_val)
  
	def time_solution_callback(self,msg: Float64):
		self.time_solution = msg.data
		self.time_solution_history.append(self.time_solution)

def main():
	FILEPATH = "/home/justin/ros2_ws/src/drone_ros/drone_ros/log/"
	FILENAME = "dumpster_log.csv"

	print(os.getcwd())
	rclpy.init(args=None)
	logger_node = LoggerNode()
	thread = threading.Thread(target=rclpy.spin, args=(logger_node, ), daemon=True)
	thread.start()

	rate = logger_node.create_rate(30)

	# #---------Logfile Setup-------------#
	# # populate the data header, these are just strings, you can name them anything
	# myData = ["time", "x", "y", "roll", "pitch", "yaw", "telem_x", "telem_y", 
	#    "telem_yaw", "telem_roll", "telem_pitch", "telem_yaw"]

	fileNameBase = FILEPATH + \
	datetime.datetime.now().strftime("%b_%d_%H_%M")
	fileNameSuffix = ".csv"
	# num is used for incrementing the file path if we already have a file in the directory with the same name
	num = 1
	fileName = fileNameBase + fileNameSuffix
	# check if the file already exists and increment num until the name is unique
	while os.path.isfile(fileName):
		fileName = fileNameBase + "_" + str(num)+"_" + fileNameSuffix
		num = num + 1

	# myFile = open(fileName, 'a')
	# with myFile:
	# 	writer = csv.writer(myFile)
	# 	writer.writerow(myData)

	# time_now = get_time_in_secs(odom_node)
	# while rclpy.ok():
	# 		# get the current time and subtract off the zero_time offset
	# 	now = (get_time_in_secs(odom_node)- time_now)
	# 	# create the data vector which we will write to the file, remember if you change
	# 	# something here, but don't change the header string, your column headers won't
	# 	# match the data
	# 	myData = [now, odom_node.current_position[0], odom_node.current_position[1],
	# 		odom_node.orientation_euler[0], odom_node.orientation_euler[1], odom_node.orientation_euler[2],
	# 		odom_node.telem_position[0], odom_node.telem_position[1], odom_node.telem_position[2],
	# 		odom_node.telem_attitudes[0], odom_node.telem_attitudes[1], odom_node.telem_attitudes[2]]

	# 	# stick everything in the file
	# 	myFile = open(fileName, 'a')
	# 	with myFile:
	# 		writer = csv.writer(myFile)
	# 		writer.writerow(myData)

	# 	rclpy.spin_once(odom_node)
	start_time = get_time_in_secs(logger_node)
	while rclpy.ok():
		
		#if ctrl+c is pressed, save the data to a csv file
		try:
			rate.sleep()
			#spin node
			current_time = get_time_in_secs(logger_node)
			# rclpy.spin_once(logger_node)
			if logger_node.x_traj_curr == None:
				continue
			else:
				print("Updating history")
				delta_time = current_time - start_time
				logger_node.time.append(delta_time)
				logger_node.update_history()
    
		except KeyboardInterrupt:
			print("Saving to csv...")
			df = logger_node.convert_to_dataframe()
			logger_node.save_to_csv(df, FILEPATH+FILENAME)
			print("Saved to csv")
			# logger_node.destroy_node()
			# rclpy.shutdown()
			return 


if __name__ == '__main__':
	main()


