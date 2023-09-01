import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

image_topic = '/image_raw/compressed'


class camera_test(Node):

	def __init__(self):
		# Creates the node.
		super().__init__('camera_test_node')

		# Set Parameters
		self.declare_parameter('show_image_bool', True)
		self.declare_parameter('window_name', "Raw Image")

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

		# Declare that the detect_object node is subcribing to the /camera/image/compressed topic.
		self._video_subscriber = self.create_subscription(
				CompressedImage,
				image_topic,
				self._image_callback,
				image_qos_profile)
		self._video_subscriber  # Prevents unused variable warning.

	def _image_callback(self, CompressedImage):
		is_object_detected = False
		# The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
		self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
		#self._imgBGR = CvBridge().imgmsg_to_cv2(CompressedImage, "bgr8")
		if(self._display_image):
			# Display the image in a window
			self.show_image(self._imgBGR)
			cv2.waitKey(1)

	def get_image(self):
		return self._imgBGR

	def show_image(self, img):
		cv2.imshow(self._titleOriginal, img)
		# Cause a slight delay so image is displayed
		self._user_input=cv2.waitKey(10) #Use OpenCV keystroke grabber for delay.


def main():
	rclpy.init()
	camera_test_node = camera_test()
	rclpy.spin(camera_test_node)
	camera_test_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()