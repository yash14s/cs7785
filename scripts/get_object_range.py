#!/usr/bin/env python3

#Pranay Mathur & Yash Srivastava
#Converts pixel value to radians

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import sys
from find_object import find_object
from sensor_msgs.msg import LaserScan, Pose2D


class get_object_range(Node):

    def __init__(self):
        super().__init__('get_object_range')

        qos_profile = QoSProfile(
			reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
			history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
			durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
			depth=1
		)
        
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )
        self.lidar_subscriber

        self.lidar_publisher = self.create_publisher(Range, '/object_distance', 10)
    
    def lidar_callback(self, LaserScan):
        #Logic here should be that if range is between detected pixel ran
        

def main():
	rclpy.init() #init routine needed for ROS2.
	video_subscriber = find_object() #Create class object to be used.
	rclpy.spin(video_subscriber) # Trigger callback processing.		

	#Clean up and shutdown.
	video_subscriber.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()