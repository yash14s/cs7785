import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point,Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

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
		self.point_subscriber = self.create_subscription(
				Point,'/object_coordinates',self.point_callback,qos_profile)
		self.laserscan_subscriber = self.create_subscription(
			LaserScan,'/scan',self.scan_callback,qos_profile)

		self.center_publisher = self.create_publisher(Point, '/LidarCameraCoordinates', 10)
		self.cx=160.0
		self.average_distance=0.4
				
	def scan_callback(self, msg):	
		range_data=LaserScan()
		range_data.ranges=msg.ranges
		#self.average_distance=np.mean([range_data.ranges[-1],range_data.ranges[-2],range_data.ranges[-3],range_data.ranges[0],range_data.ranges[1],range_data.ranges[2],range_data.ranges[3]])		
		self.average_distance=range_data.ranges[0]
		print(self.average_distance)

	def point_callback(self, msg):	
		self.cx=msg.x
		self.cy=msg.y
		a=Point()
		a.x=self.cx
		a.y=self.average_distance
		self.center_publisher.publish(a)
		#self.convert_pixel_to_direction()


def main():
	rclpy.init() #init routine needed for ROS2.
	point_subscriber = get_object_range() #Create class object to be used.
	rclpy.spin(point_subscriber) # Trigger callback processing.		

	#Clean up and shutdown.
	point_subscriber.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()
