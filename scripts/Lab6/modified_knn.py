#!/usr/bin/env python3

import cv2
import sys
import csv
import time
import numpy as np

def process(img):
	img_copy = img.copy()
	## Blurring and Threshing
	blurred       = cv2.medianBlur(img_copy, 9)
	filter       = cv2.bilateralFilter(blurred, 5, 75, 75)
	_, thresh = cv2.threshold(filter,90,255,cv2.THRESH_BINARY)

	## Morphological Operations
	element       = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
	dilated       = cv2.dilate(thresh, element, iterations=1)

	## Bitwise And Operation
	mask          = 255-dilated
	sample        = mask*img_copy

	## Contour Detection using Canny Edge Detection Algorithm
	edged = cv2.Canny(sample,30,200)
	contours, _ = cv2.findContours(edged, 
						  cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
	## Find the contour with the maximum area
	try:
		areas = [cv2.contourArea(c) for c in contours]
		if max(areas) > 0:
			max_index = np.argmax(areas)
			cnt = contours[max_index]
		else:
			return img
	except:
		return img

	x,y,w,h = cv2.boundingRect(cnt)
	return img[y-20:y+h+10,x-20:x+w+10]


### Load training images and labels

imageDirectory = './2022Fimgs/'

with open(imageDirectory + 'train.txt', 'r') as f:
	reader = csv.reader(f)
	lines = list(reader)

# this line reads in all images listed in the file in GRAYSCALE, and resizes them to 33x25 pixels
train = np.array([np.array(cv2.resize(cv2.imread(imageDirectory +lines[i][0]+".png",0),(33,25))) for i in range(len(lines))])

# here we reshape each image into a long vector and ensure the data type is a float (which is what KNN wants)
train_data = train.flatten().reshape(len(lines), 33*25)
train_data = train_data.astype(np.float32)

# read in training labels
train_labels = np.array([np.int32(lines[i][1]) for i in range(len(lines))])


### Train classifier
knn = cv2.ml.KNearest_create()
knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)

if(__debug__):
	Title_images = 'Original Image'
	Title_resized = 'Image Resized'
	cv2.namedWindow( Title_images, cv2.WINDOW_AUTOSIZE )


imageDirectory = './2022Fimgs/'

### Run test images
with open(imageDirectory + 'test.txt', 'r') as f:
	reader = csv.reader(f)
	lines = list(reader)

correct = 0.0
confusion_matrix = np.zeros((6,6))

k = 21

for i in range(len(lines)):
	original_img = cv2.imread(imageDirectory+lines[i][0]+".png",0)
	test_img = np.array(cv2.resize(cv2.imread(imageDirectory+lines[i][0]+".png",0),(33,25)))
	if(__debug__):
		cv2.imshow(Title_images, original_img)
		cv2.imshow(Title_resized, test_img)
		key = cv2.waitKey()
		if key==27:    # Esc key to stop
			break

	#TODO process test img
	test_img = process(test_img)
	test_img = test_img.flatten().reshape(1, 33*25)
	test_img = test_img.astype(np.float32)

	test_label = np.int32(lines[i][1])

	ret, results, neighbours, dist = knn.findNearest(test_img, k)

	if test_label == ret:
		print(str(lines[i][0]) + " Correct, " + str(ret))
		correct += 1
		confusion_matrix[np.int32(ret)][np.int32(ret)] += 1
	else:
		confusion_matrix[test_label][np.int32(ret)] += 1

		print(str(lines[i][0]) + " Wrong, " + str(test_label) + " classified as " + str(ret))
		print("\tneighbours: " + str(neighbours))
		print("\tdistances: " + str(dist))



print("\n\nTotal accuracy: " + str(correct/len(lines)))
print(confusion_matrix)