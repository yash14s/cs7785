#!/usr/bin/env python3
### Authors: Pranay Mathur & Yash Srivastava

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Int8
import numpy as np
import cv2
from cv_bridge import CvBridge
import os
from PIL import Image
from sklearn import svm
import pickle
import warnings


class classifier_real_time(Node):

	def __init__(self):
		# Creates the node.
		super().__init__('classifier_real_time')

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

		self.SVM_CLASSIFIER = pickle.load(open('./svm_model.pickle', 'rb'))

		# Declare that the find_object node is subcribing to the /camera/image/compressed topic.
		self._video_subscriber = self.create_subscription(
				CompressedImage,
				'/camera/image/compressed',
				self.image_callback,
				image_qos_profile)
		self._video_subscriber  # Prevents unused variable warning.

		
		self.prediction_publisher = self.create_publisher(Int8, '/predicted_label', 10)
		

	def crop(self, img):
		img = np.array(img)
		gray = cv2.medianBlur(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY), 5)
		(_, image_th) = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
		edges = cv2.Canny(image_th, 50, 200)
		
		try:
			contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
			c = max(contours, key=cv2.contourArea)
			x, y, w, h = cv2.boundingRect(c)
			cropped_contour = img[y-10:y + h+10, x-10:x + w+10]
			resized_img = cv2.resize(cropped_contour,(150, 150),interpolation = cv2.INTER_LINEAR)
		except Exception as e:
			resized_img = None

		return resized_img


	def transform_image(self, image):
		image = np.array(image)
		gray = cv2.medianBlur(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY), 7)
		(_, image_th) = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
		edges = cv2.Canny(image_th, 50, 200)

		try:
			contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
			c = max(contours, key=cv2.contourArea)
			x, y, w, h = cv2.boundingRect(c)        
			cropped_contour = image[y-10:y + h+10, x-10:x + w+10]
			if cropped_contour.shape[0]/image.shape[0] > 0.2 and cropped_contour.shape[1]/image.shape[1] > 0.2:
				train_image=cv2.resize(cropped_contour,(150, 150),interpolation=cv2.INTER_LINEAR)
			else:
				train_image=cv2.resize(cropped_contour,(150, 150),interpolation=cv2.INTER_LINEAR)
				return True , train_image
				
		except Exception as e:
			train_image = image[image.shape[0]//2-75:image.shape[0]//2+75, image.shape[1]//2-75:image.shape[1]//2+75]
		return False, train_image
	

	def match_template_in_case_of_wall(self, img_rgb):
		img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
		
		template = cv2.imread('templates/72.png', 0)
		res4 = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)

		template = cv2.imread('templates/78.jpg', 0)
		res1 = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)

		template = cv2.imread('templates/3.png', 0)
		res_1 = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)
		if(res4>0.6):
			return 4
		elif(res1>0.5):
			return 1
		elif(res_1>0.5):
			return 1
		else:
			return -1
		

	def get_preds(self, img, model):
				
		isWall, cropped = self.transform_image(img)
		
		if isWall:
			img = np.array(img)
			#class_num = self.match_template_in_case_of_wall(img)
			# if(class_num != -1):
			# 	pred = int(class_num)
			# else:
			# 	pred = 0 
			pred=0
		else:
			img_bw = cv2.cvtColor(cropped, cv2.COLOR_RGB2GRAY)
			hog = cv2.HOGDescriptor((64,64),(16,16),(8,8),(8,8),9,1,4.0,0,2.0e-1,0,64)
			hist = hog.compute(img_bw)
			dec = model.decision_function(hist[None,:])
			pred = np.argmax(dec, axis = 1)[0]
				
		return pred


	def image_callback(self, CompressedImage):
		self.imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
		cv2.imshow("Window", self.imgBGR)
		cv2.waitKey(1)
		self.pred = self.get_preds(self.imgBGR, self.SVM_CLASSIFIER)
		
		msg = Int8()
		msg.data = int(self.pred)
		self.prediction_publisher.publish(msg)



def main():
	warnings.filterwarnings("ignore")
	rclpy.init()
	classifier_real_time_node = classifier_real_time()
	rclpy.spin(classifier_real_time_node)
	classifier_real_time_node.destroy_node()
	rclpy.shutdown()
	

if __name__=='__main__':
	main()