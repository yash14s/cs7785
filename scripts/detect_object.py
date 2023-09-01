import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist, Point

import sys

import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Bool

#Select image topic depending upon sim/real
#image_topic = '/camera/image_raw'
image_topic = '/image_raw/compressed'


class detect_object(Node):

	def __init__(self):
		# Creates the node.
		super().__init__('detect_object')

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

		# Declare that the detect_object node is subcribing to the /camera/image/compressed topic.
		self._video_subscriber = self.create_subscription(
				CompressedImage,
				image_topic,
				self._image_callback,
				image_qos_profile)
		self._video_subscriber  # Prevents unused variable warning.

		self.angle_publisher = self.create_publisher(Pose2D, '/angle', 10)
		self.object_detected_publisher = self.create_publisher(Bool, '/object_detected', 10)


	def pixel_to_radians(self, pixel):
		#camera horizontal FOV = 1.085595
		self.angle = pixel * (1.085595/320)

	def _image_callback(self, CompressedImage):
		is_object_detected = False
		# The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
		self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
		#self._imgBGR = CvBridge().imgmsg_to_cv2(CompressedImage, "bgr8")
		if(self._display_image):
			# Display the image in a window
			self.show_image(self._imgBGR)
			# For Gazebo
			#hsvLower = (120, 100, 100)
			#hsvUpper = (150, 255, 255)

			# For tb3
			hsvLower = (80, 100, 60)
			hsvUpper = (120, 200, 200)
			rgb_image = self._imgBGR
			hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
			cv2.imshow("hsv_image",hsv_image)
			cv2.waitKey(1)
			binary_image_mask = cv2.inRange(hsv_image, hsvLower, hsvUpper)
			#cv2.imshow("hsv_image",binary_image_mask)
			#cv2.waitKey(1)
			contours, hierarchy = cv2.findContours(binary_image_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			self.cx=160
			self.cy=120
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
						self.area = area
						is_object_detected = True
					cv2.circle(rgb_image, (self.cx, self.cy), (int)(radius), (0, 0, 255), 1)
			cv2.imshow("Output", rgb_image)
			cv2.waitKey(1)

			self.pixel_to_radians(self.cx)
			msg = Pose2D()
			msg.theta = self.angle
			self.angle_publisher.publish(msg)

			is_object_detected_msg = Bool()
			is_object_detected_msg.data = is_object_detected
			self.object_detected_publisher.publish(is_object_detected_msg)

	def get_image(self):
		return self._imgBGR

	def show_image(self, img):
		cv2.imshow(self._titleOriginal, img)
		# Cause a slight delay so image is displayed
		self._user_input=cv2.waitKey(10) #Use OpenCV keystroke grabber for delay.


def main():
	rclpy.init()
	detect_object_node = detect_object()
	rclpy.spin(detect_object_node)
	detect_object_node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
