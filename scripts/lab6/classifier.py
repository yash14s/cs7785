from sklearn.metrics import confusion_matrix
import os
import cv2
import numpy as np
from PIL import Image
from glob import glob
import pickle
import warnings

def process_image(img):
	img = np.array(img)
	gray = cv2.medianBlur(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY), 7)
	(_, blackAndWhiteImage) = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
	edges = cv2.Canny(blackAndWhiteImage, 50, 200)
	
	try:
		contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		c = max(contours, key=cv2.contourArea)
		x, y, w, h = cv2.boundingRect(c)        
		cropped_contour = img[y-10:y + h+10, x-10:x + w+10]
		if cropped_contour.shape[0]/img.shape[0] > 0.2 and cropped_contour.shape[1]/img.shape[1] > 0.2:
			train_image=cv2.resize(cropped_contour,(150, 150),interpolation=cv2.INTER_LINEAR)
		else:
			return True , None
			
	except Exception as e:
		train_image = img[img.shape[0]//2-75:img.shape[0]//2+75, img.shape[1]//2-75:img.shape[1]//2+75]
	return False, train_image


def computeHOG(image):
	winSize = (64,64)
	blockSize = (16,16)
	blockStride = (8,8)
	cellSize = (8,8)
	nbins = 9
	derivAperture = 1
	winSigma = 4.
	histogramNormType = 0
	L2HysThreshold = 2.0000000000000001e-01
	gammaCorrection = 0
	nlevels = 64
	hog = cv2.HOGDescriptor(winSize,blockSize,blockStride,cellSize,nbins,derivAperture,winSigma,
					  histogramNormType,L2HysThreshold,gammaCorrection,nlevels)
	hist = hog.compute(image)
	return hist


def get_preds(test_folder , model):
	y = []
	imgs = glob(test_folder+"*.jpg") + glob(test_folder+"*.png")

	for i in range(len(imgs)):
		img = Image.open(imgs[i]).convert("RGB")
		name = imgs[i].split("/")[-1].split(".")[0]
		
		isWall, cropped = process_image(img)
		
		if isWall:
			y.append((int(name) , 0))
		else:
			img_bw = cv2.cvtColor(cropped , cv2.COLOR_RGB2GRAY)
			hist = computeHOG(img_bw)
			dec = model.decision_function(hist[None,:])
			y_pred = np.argmax(dec, axis = 1)
			y.append((int(name), y_pred[0]))
		
	return (y)


def metrics(y_labels, y_pred):
	dict_preds = dict(y_pred)
	accuracy = len([(key,val,dict_preds[key]) for key,val in y_labels if dict_preds.get(key,val) == val])/len(y_labels)
	print("Accuracy: ", accuracy)
	y_labels = [i[1] for i in sorted(y_labels , key = lambda x:x[0])]
	y_pred = [i[1] for i in sorted(y_pred , key = lambda x:x[0])]
	conf = confusion_matrix(y_labels, y_pred)
	print("Confusion Matrix: \n", conf)
	return accuracy, conf
	

warnings.filterwarnings("ignore")

#This is the path to the directory that contains test images and labels.txt
DIR = "./testF2022imgs/"

test_list_txt = os.path.join(DIR , "labels.txt")
f = open(test_list_txt , 'r')
y_labels = list(f)
y_labels = [(int(i.split('\n')[0].split(',')[0]) , int(i.split('\n')[0].split(',')[1])) for i in y_labels]

SVM_CLASSIFIER = pickle.load(open('./svm_model.pickle', 'rb'))

y_pred = get_preds(DIR, SVM_CLASSIFIER)

metrics(y_labels, y_pred)