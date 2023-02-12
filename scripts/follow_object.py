#!/usr/bin/env python3

#Pranay Mathur & Yash Srivastava

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point,Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import sys

class rotate_robot(Node):

	def __init__(self):
		# Creates the node.
		super().__init__('rotate_robot')
		qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1
        )
		# Declare that the rotate_robot node is subcribing to the /camera/image/compressed topic.
		self.point_subscriber = self.create_subscription(
				Point, '/object_coordinates', self.point_callback, qos_profile)

		self.center_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
		
	def control_rotation(self):
		velocity_msg = Twist()
		#velocity_msg.linear.x = 0.0
		velocity_msg.angular.z = 0.0
		#if(self.area>1000):
			#print("moving backward")
			#velocity_msg.linear.x = -1.000
		#elif(self.area<2000):
			#print("moving forward")
		# Center was 160	
		if(int(self.cx)<=140):
			print("moving left")
			velocity_msg.angular.z = 0.2
		elif(int(self.cx)>180):
			print("moving right")
			velocity_msg.angular.z = -0.2
		else:
			print("Object centered or No object detected")
			velocity_msg.linear.x = 0.0
			velocity_msg.angular.z = 0.0
		self.center_publisher.publish(velocity_msg)
		

	def point_callback(self, msg):	
		self.cx=msg.x
		self.cy=msg.y
		self.convert_pixel_to_direction()


def main():
	rclpy.init() #init routine needed for ROS2.
	point_subscriber = rotate_robot() #Create class object to be used.
	rclpy.spin(point_subscriber) # Trigger callback processing.		

	#Clean up and shutdown.
	point_subscriber.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()