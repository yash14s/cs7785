#!/usr/bin/env python3

#Pranay Mathur & Yash Srivastava

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool, Int8
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

		#Get the phase of the mission from navigation
		self.phase_subscriber = self.create_subscription(
			LaserScan, '/phase_topic', self.phase_callback, qos_profile)

		self.object_detected_publisher = self.create_publisher(
			Bool, '/object_detected', 10)

		
		self.is_object_detected = False


	def phase_callback(self, msg)
		self.phase = Int8()
		self.phase.data = msg.data

	#TODO modify
	def scan_callback(self, msg):

		range_data = LaserScan()
		range_data.ranges = msg.ranges
		range_data.angle_min = msg.angle_min
		range_data.angle_max = msg.angle_max
		range_data.angle_increment = msg.angle_increment

		if self.phase == 1:
			index = round(deg2rad(270)/range_data.angle_increment)
			window_size = 4
		elif self.phase == 2:
			index = round(deg2rad(0)/range_data.angle_increment)
			window_size = 4
		elif self.phase == 3:
			index = round(deg2rad(0)/range_data.angle_increment)
			window_size = 30

		neighbour_distances = []
		for i in range(2 * window_size):
			if range_data.ranges[index + window_size - i] != float("inf") and (not np.isnan(range_data.ranges[index + window_size - i])):
				self.neighbour_distances.append(range_data.ranges[index + window_size - i])

		#print(self.neighbour_distances)
		if not self.neighbour_distances:	#If all distances were inf, default to REFERENCE_DISTANCE
			self.distance = self.REFERENCE_DISTANCE

		else:
			self.distance = float(np.min(self.neighbour_distances))

def deg2rad(degrees)
	return degrees * math.pi/180

def main():
	rclpy.init()
	get_object_range_node = get_object_range()
	rclpy.spin(get_object_range_node)
	get_object_range_node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()