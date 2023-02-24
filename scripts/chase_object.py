#!/usr/bin/env python3

#Pranay Mathur & Yash Srivastava

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D,Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import sys


class chase_object(Node):

	def __init__(self):
		# Creates the node.
		super().__init__('chase_object')
		qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1
        )

		self.point_subscriber = self.create_subscription(
				Pose2D, '/object_coordinates', self.object_pose_callback, qos_profile)

		self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)


	def controller(self):
		velocity_msg = Twist()
		
		# Define P Controller Gains
		Kp_angular = 1.7
		Kp_linear = 0.07

		# Define reference points
		REFERENCE_ANGLE = 0.5427975
		REFERENCE_DISTANCE = 0.4

		# Compute error
		e_angular = self.angle - REFERENCE_ANGLE
		e_linear = self.distance - REFERENCE_DISTANCE

		# Compute velocity and publish
		linear_velocity = Kp_linear * e_linear
		angular_velocity = -Kp_angular * e_angular

		if abs(linear_velocity) < 0.01:
			linear_velocity = 0.0
		
		if abs(angular_velocity) < 0.002:
			angular_velocity = 0.0

		velocity_msg.linear.x = linear_velocity
		velocity_msg.angular.z = angular_velocity

		self.velocity_publisher.publish(velocity_msg)
		
	def object_pose_callback(self, msg):	
		self.distance = msg.x
		self.angle = msg.theta
		self.controller()


def main():
	rclpy.init()
	chase_object_node = chase_object()
	rclpy.spin(chase_object_node)
	chase_object_node.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()