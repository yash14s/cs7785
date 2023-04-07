import cv2
import numpy as np;
import sklearn as sk
from sklearn.neighbors import KNeighborsClassifier
from sklearn import metrics
import pickle
from sklearn import preprocessing
from sklearn.metrics import confusion_matrix
import os
import numpy as np
import csv
# Read image

IMG_SIZE1=205
IMG_SIZE2=154


def load_data(data):
	images=[]
	labels=[]
	filename=[]
	path='./2022Fimgs/'
	for each in data:
		im=cv2.imread(os.path.join(path,each[0]+".png"))
		images.append(im)
		labels.append(int(each[1]))
		filename.append(each[0])
	return np.array(images),labels


def load_data_from_txt(path):
	DIR='./2022Fimgs/'
	f=open(DIR+path,'r')
	reader = csv.reader(f)
	data = np.array(list(reader))
	return data


def process(image):

	cv2.imshow("image", image)
	
	img_copy      = image.copy()
	img_copy      = cv2.cvtColor(img_copy,cv2.COLOR_BGR2GRAY)

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
		if max(areas)>0:
			max_index = np.argmax(areas)
			cnt=contours[max_index]
		else:
			return image
	except:
		return image

	x,y,w,h = cv2.boundingRect(cnt)
	return image[y-20:y+h+10,x-20:x+w+10]


train_data = load_data_from_txt('train.txt')
test_data = load_data_from_txt('test.txt')

#print(train_data)

train_images, train_labels = load_data(train_data)
test_images, test_labels   = load_data(test_data)

processed_test_images=[]

#print(test_images)

for image in test_images:
	processed_image=process(image)
	if processed_image.shape[1] ==0 or processed_image.shape[0] == 0:
		processed_image = image
	processed_image=cv2.resize(processed_image,(IMG_SIZE1,IMG_SIZE2))
	processed_test_images.append(processed_image)

processed_test_images=np.array(processed_test_images)
train_images,processed_test_images = (train_images.reshape(train_images.shape[0],-1)),(processed_test_images.reshape(processed_test_images.shape[0],-1))
train_images=train_images/255
processed_test_images=processed_test_images/255

knn = KNeighborsClassifier(n_neighbors=1)
knn.fit(train_images, np.squeeze(train_labels))


filename = 'finalized_model.pkl'
pickle.dump(knn, open(filename, 'wb'))
y_pred = knn.predict(processed_test_images)
print("Accuracy:",metrics.accuracy_score(test_labels, y_pred))
print(confusion_matrix(test_labels, y_pred))