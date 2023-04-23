#!/usr/bin/env python3

#Pranay Mathur & Yash Srivastava

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import math
import numpy as np

class wall_following_test(Node):

	def __init__(self):
		# Creates the node.
		super().__init__('wall_following_test')
		qos_profile = QoSProfile(
			reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
			history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
			durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
			depth=1
		)
		

		self.left_wall_vel_subscriber = self.create_subscription(
			Twist, '/left_wall_vel', self.left_wall_callback, qos_profile)
		self.right_wall_vel_subscriber = self.create_subscription(
			Twist, '/right_wall_vel', self.right_wall_callback, qos_profile)
		
		self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
		
		self.wall_to_follow = 'left'


	def left_wall_callback(self, msg):
		vel = Twist()
		vel.linear.x = 0.3
		vel.angular.z = msg.angular.z
		if self.wall_to_follow == 'left':
			self.vel_publisher.publish(vel)
	
	def right_wall_callback(self, msg):
		vel = Twist()
		vel.linear.x = 0.3
		vel.angular.z = msg.angular.z
		if self.wall_to_follow == 'right':
			self.vel_publisher.publish(vel)

	
def main():
	rclpy.init() #init routine needed for ROS2.
	wall_following_test_node = wall_following_test() #Create class object to be used.
	rclpy.spin(wall_following_test_node) # Trigger callback processing.		
	wall_following_test_node.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()