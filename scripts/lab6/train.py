import os
import cv2
import numpy as np
from sklearn import svm
from PIL import Image
import pickle
import warnings

warnings.filterwarnings("ignore")

train_folder = "./train/"
train_list_txt = os.path.join(train_folder , "labels.txt")
train_list = []

f = open(train_list_txt , 'r')
train_list = list(f)
train_list = [(int(i.split('\n')[0].split(',')[0]) , int(i.split('\n')[0].split(',')[1])) for i in train_list] 


def crop(img, label):
    img = np.array(img)
    gray = cv2.medianBlur(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY), 7)
    (_, blackAndWhiteImage) = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
    edges = cv2.Canny(blackAndWhiteImage, 50, 200)
    
    try:
        contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        cropped_contour = img[y-10:y + h+10, x-10:x + w+10]
        train_image=cv2.resize(cropped_contour,(150, 150),interpolation=cv2.INTER_LINEAR)
    except Exception as e:
        
        if label is not None and label == 0:
            print(e)
            train_image = img[img.shape[0]//2-75:img.shape[0]//2+75 , img.shape[1]//2-75:img.shape[1]//2+75]
        
        elif label is None:
            train_image = img[img.shape[0]//2-75:img.shape[0]//2+75 , img.shape[1]//2-75:img.shape[1]//2+75]

        else:
            train_image  = None
    return train_image


def createDataset(img_folder, data_list):
    X = []
    y = []
    n = len(data_list)
    for i in range(n):
        try:
            img = Image.open(os.path.join(img_folder , f"{data_list[i][0]}.jpg")).convert("RGB")
        except:
            img = Image.open(os.path.join(img_folder , f"{data_list[i][0]}.png")).convert("RGB")
  
        label = data_list[i][1]
        cropped = crop(img, label)

        if cropped is not None:
            X.append(cropped)
            y.append(label)

            if label == 1:
                X.append(np.flip(cropped , axis = 1))
                y.append(2)

            if label == 2:
                X.append(np.flip(cropped , axis = 1))
                y.append(1)          
    return np.array(X),np.array(y)


X_train, y_train = createDataset(train_folder, train_list)


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


def create_dataset_hist(data, labels):
    X = []
    y = []
    for i in range(data.shape[0]):
        img = data[i]
        label = labels[i]
        img_bw = cv2.cvtColor(img , cv2.COLOR_RGB2GRAY)
        hist = computeHOG(img_bw)
        X.append(hist)
        y.append(label) 
    return np.array(X),np.array(y)


X_train_hist,y_train_hist = create_dataset_hist(X_train , y_train)

SVM = svm.LinearSVC()
SVM.fit(X_train_hist, y_train_hist)
filename = 'model.pkl'
pickle.dump(SVM, open(filename, 'wb'))