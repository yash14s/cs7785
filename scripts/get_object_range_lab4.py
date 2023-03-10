#!/usr/bin/env python3

#Pranay Mathur & Yash Srivastava

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import sys
import math
import numpy as np

class get_object_range(Node):

	def __init__(self):
		# Creates the node.
		super().__init__('get_object_range')
		qos_profile = QoSProfile(
			reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
			history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
			durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
			depth=1
		)
		self.laserscan_subscriber = self.create_subscription(
			LaserScan, '/scan', self.scan_callback, qos_profile)

		self.object_pose_publisher = self.create_publisher(
			Pose2D , '/object_pose', 10)

		#Initialize
		self.cone_angle = self.deg2rad(60)	#Define cone angle, robot will only look for obstacles within this region


	def scan_callback(self, msg):
		#Get some useful LIDAR properties
		self.range_data = LaserScan()
		self.range_data.ranges = msg.ranges
		self.range_data.angle_min = msg.angle_min
		self.range_data.angle_max = msg.angle_max
		self.range_data.angle_increment = msg.angle_increment
		self.range_data.range_max = msg.range_max
		
		self.compute_window()

		self.compute_obstacle_distance_angle()
		
		self.compute_object_pose()


	def compute_window(self):
		theta = self.cone_angle
		self.right_index = round((theta/2 - self.range_data.angle_min)/self.range_data.angle_increment)
		self.left_index = round((self.range_data.angle_max - theta/2)/self.range_data.angle_increment) - 1

	
	def compute_obstacle_distance_angle(self):
		#Init
		min_distance = float("inf")
		index = 0
		distance = self.range_data.range_max

		#Check from 0 to left_index
		for i in range(self.left_index):
			raw_distance = self.range_data.ranges[i]
			if raw_distance != float("inf") and (not np.isnan(raw_distance)):
				if raw_distance < min_distance:
					index = i
					min_distance = raw_distance
		#Check from right_index to 0
		for i in range(self.right_index, len(self.range_data.ranges)):
			raw_distance = self.range_data.ranges[i]
			if raw_distance != float("inf") and (not np.isnan(raw_distance)):
				if raw_distance < min_distance:
					index = i
					min_distance = raw_distance

		self.distance = min_distance
		#print("Index:", index)
		#print("Distance:", self.distance)
		self.theta = 2 * math.pi - (self.range_data.angle_min + index * self.range_data.angle_increment)


	def compute_object_pose(self):
		object_pose = Pose2D()
		object_pose.x = self.distance * math.cos(self.theta)
		object_pose.y = self.distance * math.sin(self.theta)
		object_pose.theta = self.theta
		self.object_pose_publisher.publish(object_pose)


	def deg2rad(self, degrees):
		return degrees * math.pi/180


def main():
	rclpy.init()
	get_object_range_node = get_object_range()
	rclpy.spin(get_object_range_node)
	get_object_range_node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()