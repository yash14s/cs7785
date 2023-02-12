#!/usr/bin/env python3

#Pranay Mathur & Yash Srivastava

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist,Point
import sys
import numpy as np
import cv2
from cv_bridge import CvBridge


class find_object(Node):

	def __init__(self):
		# Creates the node.
		super().__init__('find_object')

		# Set Parameters
		self.declare_parameter('show_image_bool', True)
		self.declare_parameter('window_name', "Raw Image")

		# center coordinates
		self.cx = 0.0
		self.cy = 0.0
		# Determine Window Showing Based on Input
		self._display_image = bool(self.get_parameter('show_image_bool').value)

		# Declare some variables
		self._titleOriginal = self.get_parameter(
		    'window_name').value  # Image Window Title
		if (self._display_image):
		# Set Up Image Viewing
			cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE)  # Viewing Window
			# Viewing Window Original Location
			cv2.moveWindow(self._titleOriginal, 50, 50)

		# Set up QoS Profiles for passing images over WiFi
		image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1
        )

		# Declare that the find_object node is subcribing to the /camera/image/compressed topic.
		self._video_subscriber = self.create_subscription(
				Image,
				'/camera/image_raw',
				self._image_callback,
				image_qos_profile)
		self._video_subscriber  # Prevents unused variable warning.

		self.object_publisher = self.create_publisher(Point,'/object_coordinates',10)


	def _image_callback(self, CompressedImage):	
		# The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
		#self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
		self._imgBGR = CvBridge().imgmsg_to_cv2(CompressedImage, "bgr8")
		if(self._display_image):
			# Display the image in a window
			self.show_image(self._imgBGR)
			hsvLower = (120, 100, 100)
			hsvUpper = (150, 255, 255)
			rgb_image = self._imgBGR
			hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
			cv2.imshow("hsv_image",hsv_image)
			cv2.waitKey(1)
			binary_image_mask = cv2.inRange(hsv_image, hsvLower, hsvUpper)  
			#cv2.imshow("hsv_image",binary_image_mask)
			#cv2.waitKey(1)
			contours, hierarchy = cv2.findContours(binary_image_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			self.cx = 160
			self.cy = 120
			for c in contours:
				area = cv2.contourArea(c)
				((x, y), radius) = cv2.minEnclosingCircle(c)
				if (area > 800):
					cv2.drawContours(rgb_image, [c], -1, (150, 250, 150), 1)
					# Get center coordinates
					M = cv2.moments(c)
					self.cx = -1
					self.cy = -1
					if (M['m00'] != 0):
						self.cx = int(M['m10']/M['m00'])
						self.cy = int(M['m01']/M['m00'])
						self.area=area
						print(self.cx, self.cy)
					cv2.circle(rgb_image, (self.cx, self.cy), (int)(radius), (0, 0, 255), 1)
			cv2.imshow("Output", rgb_image)
			cv2.waitKey(1)
			msg=Point()
			msg.x=float(self.cx)
			msg.y=float(self.cy)
			self.object_publisher.publish(msg)

	def get_image(self):
		return self._imgBGR

	def show_image(self, img):
		cv2.imshow(self._titleOriginal, img)
		# Cause a slight delay so image is displayed
		self._user_input=cv2.waitKey(10) #Use OpenCV keystroke grabber for delay.


def main():
	rclpy.init() #init routine needed for ROS2.
	video_subscriber = find_object() #Create class object to be used.
	rclpy.spin(video_subscriber) # Trigger callback processing.		

	#Clean up and shutdown.
	video_subscriber.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()