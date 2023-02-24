#!/usr/bin/env python3

#Pranay Mathur & Yash Srivastava

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point,Twist
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
		# Declare that the chase_object node is subcribing to the /camera/image/compressed topic.
		self.point_subscriber = self.create_subscription(
				Point, '/LidarCameraCoordinates', self.point_callback, qos_profile)

		self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
		
	def control_rotation(self):
		velocity_msg = Twist()
				
		Kp_angular = 0.005	# Define Proportional Controller Gain
		reference_point = 160	#Define Center as the reference point
		Kp_forward = 0.1

		e = int(self.cx) - reference_point
		e_forward = self.distance_y-0.4

		if abs(e)>10:
			velocity_msg.angular.z = -Kp_angular * e
			print("Angular Velocity:",velocity_msg.angular.z)
		if abs(e_forward)>0.1:
			velocity_msg.linear.x = Kp_forward * e_forward
			print("Linear Velocity:",velocity_msg.linear.x)
		elif abs(e_forward)>2:
			velocity_msg.linear.x = 0.4
			#print(self.distance_y)
		else:
			velocity_msg.linear.x=0.0
			velocity_msg.angular.z=0.0
		self.velocity_publisher.publish(velocity_msg)
		
	def point_callback(self, msg):	
		self.cx = msg.x
		self.distance_y = msg.y
		self.control_rotation()


def main():
	rclpy.init() #init routine needed for ROS2.
	point_subscriber = chase_object() #Create class object to be used.
	rclpy.spin(point_subscriber) # Trigger callback processing.		

	#Clean up and shutdown.
	point_subscriber.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()
