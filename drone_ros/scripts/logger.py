#!/usr/bin/env python
# -*- coding: utf-8 -*- 


from ipaddress import ip_address
import pandas as pd
import numpy as np
import rclpy
from rclpy.node import Node

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
		
		self.state_sub = self.create_subscription(
						mavros.local_position.Odometry, 
						'/mavros/local_position/odom', 
						self.odom_callback, 
						qos_profile=SENSOR_QOS)
		
		self.traj_sub = self.create_subscription(
						CtlTraj,
						'avoid_trajectory',
						self.ctl_traj_callback,
						qos_profile=SENSOR_QOS)

		self.__init_variables()

	def __init_variables(self) -> None:	
		self.time = []
		self.x_position = []
		self.y_position = []
		self.z_position = []
		self.roll = []
		self.pitch = []
		self.yaw = []

		self.x_traj = []
		self.y_traj = []
		self.z_traj = []
		self.roll_traj = []
		self.pitch_traj = []
		self.yaw_traj = []

		self.x_cmd = []
		self.y_cmd = []
		self.z_cmd = []
		self.roll_cmd = []
		self.pitch_cmd = []
		self.yaw_cmd = []

		self.goal_x = []
		self.goal_y = []
		self.goal_z = []

		# self.obstacles = Config.OBSTACLES

	def odom_callback(self,msg):
		"""subscribe to odometry"""
		self.time.append(get_time_in_secs(self))
		self.x_position.append(msg.pose.pose.position.x)
		self.y_position.append(msg.pose.pose.position.y)
		self.z_position.append(msg.pose.pose.position.z)

		qx = msg.pose.pose.orientation.x
		qy = msg.pose.pose.orientation.y
		qz = msg.pose.pose.orientation.z
		qw = msg.pose.pose.orientation.w
		
		roll,pitch,yaw = quaternion_tools.get_euler_from_quaternion(qx, qy, qz, qw)
		
		self.roll.append(roll)
		self.pitch.append(pitch)
		self.yaw.append(yaw)

	def ctl_traj_callback(self,msg):
		self.x_traj.append(msg.x)
		self.y_traj.append(msg.y)
		self.z_traj.append(msg.z)
		
		self.roll_traj.append(msg.roll)
		self.pitch_traj.append(msg.pitch)
		self.yaw_traj.append(msg.yaw)

		self.x_cmd.append(msg.x[msg.idx])
		self.y_cmd.append(msg.y[msg.idx])
		self.z_cmd.append(msg.z[msg.idx])

		self.roll_cmd.append(msg.roll[msg.idx])
		self.pitch_cmd.append(msg.pitch[msg.idx])
		self.yaw_cmd.append(msg.yaw[msg.idx])

	def convert_to_dataframe(self):
		# convert to dataframe
		info_dict = {
			'time': self.time,
			'x': self.x_position,
			'y': self.y_position,
			'z': self.z_position,
			'roll': self.roll,
			'pitch': self.pitch,
			'yaw': self.yaw,
			'x_traj': self.x_traj,
			'y_traj': self.y_traj,
			'z_traj': self.z_traj,
			'roll_traj': self.roll_traj,
			'pitch_traj': self.pitch_traj,
			'yaw_traj': self.yaw_traj,
			'x_cmd': self.x_cmd,
			'y_cmd': self.y_cmd,
			'z_cmd': self.z_cmd,
			'roll_cmd': self.roll_cmd,
			'pitch_cmd': self.pitch_cmd,
			'yaw_cmd': self.yaw_cmd,
			# 'obstacles': self.obstacles
			}
				
		df = pd.DataFrame.from_dict(info_dict, orient='index')
		df = df.transpose()
		
		return df

	def save_to_csv(self,df, file_name):
		# save to csv
		df.to_csv(file_name)

def main():
	FILEPATH = "/home/justin/ros2_ws/src/drone_ros/drone_ros/log/"
	FILENAME = "dumpster_log.csv"

	print(os.getcwd())
	rclpy.init(args=None)
	logger_node = LoggerNode()

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

	while rclpy.ok():
		
		#if ctrl+c is pressed, save the data to a csv file
		try:
			rclpy.spin(logger_node)

		except KeyboardInterrupt:
			print("Saving to csv...")
			df = logger_node.convert_to_dataframe()
			logger_node.save_to_csv(df, FILEPATH+FILENAME)
			print("Saved to csv")
			break


if __name__ == '__main__':
	main()


