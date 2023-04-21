#!/usr/bin/env python3

#Pranay Mathur & Yash Srivastava

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import math
import numpy as np

class wall_following(Node):

	def __init__(self):
		# Creates the node.
		super().__init__('wall_following')
		qos_profile = QoSProfile(
			reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
			history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
			durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
			depth=1
		)
		self.laserscan_subscriber = self.create_subscription(
			LaserScan, '/scan', self.scan_callback, qos_profile)

		self.left_wall_vel_publisher = self.create_publisher(Twist, 'left_wall_vel', 10)
		self.right_wall_vel_publisher = self.create_publisher(Twist, 'right_wall_vel', 10)

		#Initialize
		self.cone_angle = self.deg2rad(60)	#Define cone angle, robot will only look for obstacles within this region
		self.reference_dist = 0.4 	#Define distance to wall
		

	def control_rotation(self, label, distance):
		velocity_msg_left = Twist()
		velocity_msg_right = Twist()
				
		Kp = 1	# Define Proportional Controller Gain
		e = distance - self.reference_dist

		if label == 1: #Follow right wall
			velocity_msg_right.angular.z = Kp * e
			self.right_wall_vel_publisher.publish(velocity_msg_right)
		elif label == 2: #Follow left wall
			velocity_msg_left.angular.z = -Kp * e
			self.left_wall_vel_publisher.publish(velocity_msg_left)

		
	def scan_callback(self, msg):
		#Get some useful LIDAR properties
		self.range_data = LaserScan()
		self.range_data.ranges = msg.ranges
		self.range_data.angle_min = msg.angle_min
		self.range_data.angle_max = msg.angle_max
		self.range_data.angle_increment = msg.angle_increment
		self.range_data.range_max = msg.range_max
		
		right_wall_distance = self.compute_wall_distance(1)
		left_wall_distance = self.compute_wall_distance(2)
		self.control_rotation(1, right_wall_distance)
		self.control_rotation(2, left_wall_distance)
		

	def compute_window(self, label):
		theta = self.cone_angle
		if label == 1: #follow right wall
			base_index = self.angle2index(270)
		elif label == 2: #follow left wall
			base_index = self.angle2index(90)

		right_index = base_index + round((theta/2 - self.range_data.angle_min)/self.range_data.angle_increment)
		left_index =  base_index - round((theta/2- self.range_data.angle_min)/self.range_data.angle_increment)
		'''
		print("****************************")
		print("label:", label)
		print("base_index:", base_index)
		print("Left_index:", left_index)
		print("Right_index:", right_index)
		print("Array size:", len(self.range_data.ranges))
		'''
		return(left_index, right_index)

	
	def compute_wall_distance(self, label):
		left_index, right_index = self.compute_window(label)
		#Init
		min_distance = float("inf")

		#Check through the window
		for i in range(left_index, right_index):
			raw_distance = self.range_data.ranges[i]
			'''
			print("label:",label)
			print("Index:",i, "Distance:", raw_distance)
			'''
			if raw_distance != float("inf") and (not np.isnan(raw_distance)):
				if raw_distance < min_distance:
					min_distance = raw_distance
		
		distance = min_distance
		#print("Index:", index)
		#print("Distance:", self.distance)
		return distance


	def deg2rad(self, degrees):
		return degrees * math.pi/180

	def angle2index(self, degrees):
		#Convert angle in degrees (clk) to index 
		theta = self.deg2rad(degrees)
		index = round((theta - self.range_data.angle_min)/self.range_data.angle_increment)
		return index

def main():
	rclpy.init() #init routine needed for ROS2.
	wall_following_node = wall_following() #Create class object to be used.
	rclpy.spin(wall_following_node) # Trigger callback processing.		
	wall_following_node.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()