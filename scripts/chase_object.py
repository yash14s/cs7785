#!/usr/bin/env python3

#Pranay Mathur & Yash Srivastava

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
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
		# Declare that the rotate_robot node is subcribing to the /camera/image/compressed topic.
		self.angle_subscriber = self.create_subscription(
				Pose2D, '/angle', self.object_pose_callback, qos_profile)

		self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
		
	def control_rotation(self):
		velocity_msg = Twist()
				
		Kp = 1.25	# Define Proportional Controller Gain
		reference_point = 0.5427975	#Define Center as the reference point

		e = self.theta - reference_point
		velocity_msg.angular.z = -Kp * e

		self.velocity_publisher.publish(velocity_msg)
		

	def object_pose_callback(self, msg):	
		self.theta = msg.theta
		self.control_rotation()


def main():
	rclpy.init() #init routine needed for ROS2.
	object_pose_subscriber = chase_object() #Create class object to be used.
	rclpy.spin(object_pose_subscriber) # Trigger callback processing.		

	#Clean up and shutdown.
	object_pose_subscriber.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()