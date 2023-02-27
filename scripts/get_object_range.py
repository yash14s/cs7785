#!/usr/bin/env python3

#Pranay Mathur & Yash Srivastava

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool
import sys
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
		# Declare that the get_object_range node is subcribing to the /camera/image/compressed topic.
		self.angle_subscriber = self.create_subscription(
			Pose2D, '/angle', self.angle_callback, qos_profile)
		self.laserscan_subscriber = self.create_subscription(
			LaserScan, '/scan', self.scan_callback, qos_profile)
		self.object_detected_subscriber = self.create_subscription(
			Bool, '/object_detected', self.object_detected_callback, qos_profile)

		self.object_coordinates_publisher = self.create_publisher(Pose2D, '/object_coordinates', 10)

		#Declare constants
		self.REFERENCE_ANGLE = 0.5427975	#Angle when object is straight ahead
		self.REFERENCE_DISTANCE = 0.4	#Reference distance to track
		self.ANGULAR_INCREMENT = 0.01749303564429283 	#Angular increment of the lidar
		self.INDEX_MARGIN = 4	#Tolerance for measuring distances
		self.LIDAR_CAMERA_OFFSET = 1	# Offset between the LIDAR and Camera orientation
		

		#Init angle and distance
		self.angle = self.REFERENCE_ANGLE
		self.distance = self.REFERENCE_DISTANCE

		self.is_object_detected = False


	def object_detected_callback(self, msg):
		self.is_object_detected = msg.data

	#In one of the callbacks, send the overall Pose2D
	def scan_callback(self, msg):
		if not self.is_object_detected:
			self.distance = self.REFERENCE_DISTANCE
		
		else:
			range_data = LaserScan()
			range_data.ranges = msg.ranges

			#Take distance measurement corresponding to the angular position of the object
			self.index = round((self.angle - self.REFERENCE_ANGLE)/self.ANGULAR_INCREMENT) + self.LIDAR_CAMERA_OFFSET

			self.neighbour_distances = []
			for i in range(2 * self.INDEX_MARGIN):
				if range_data.ranges[self.index + self.INDEX_MARGIN - i] != float("inf") and (not np.isnan(range_data.ranges[self.index + self.INDEX_MARGIN - i])):
					self.neighbour_distances.append(range_data.ranges[self.index + self.INDEX_MARGIN - i])

			#print(self.neighbour_distances)
			if not self.neighbour_distances:	#If all distances were inf, default to REFERENCE_DISTANCE
				self.distance = self.REFERENCE_DISTANCE

			else:
				self.distance = float(np.min(self.neighbour_distances))


	def angle_callback(self, msg):
		self.angle = msg.theta
		#Send distance and angle
		object_pose = Pose2D()
		object_pose.x = self.distance
		object_pose.theta = self.angle
		self.object_coordinates_publisher.publish(object_pose)



def main():
	rclpy.init()
	get_object_range_node = get_object_range()
	rclpy.spin(get_object_range_node)
	get_object_range_node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
