#!/usr/bin/env python3
### Authors: Yash and Pranay
### Credits: Sohan Anisetty, Internet Codes, Scikit documentation
import os
import cv2
import sys
import numpy as np
from PIL import Image
from glob import glob
from sklearn import svm
import pickle
import warnings
from sklearn.metrics import confusion_matrix

def crop(img, label):
    img = np.array(img)
    gray = cv2.medianBlur(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY), 5)
    (_, image_th) = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
    edges = cv2.Canny(image_th, 50, 200)
    
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


def create_dataset_hist(data, labels):
    X = []
    y = []
    for i in range(data.shape[0]):
        img = data[i]
        label = labels[i]
        img_bw = cv2.cvtColor(img , cv2.COLOR_RGB2GRAY)
        hog = cv2.HOGDescriptor((64,64),(16,16),(8,8),(8,8),9,1,4.0,0,2.0e-1,0,64)
        hist = hog.compute(img_bw)
        X.append(hist)
        y.append(label) 
    return np.array(X),np.array(y)

def transform_image(image):
    image = np.array(image)
    gray = cv2.medianBlur(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY), 7)
    (_, image_th) = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
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

def match_template_in_case_of_wall(img_rgb):
    img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
    
    template = cv2.imread('72.png', 0)
    res4 = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)

    template = cv2.imread('78.jpg', 0)
    res1 = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)

    template = cv2.imread('3.png', 0)
    res_1 = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)
    if(res4>0.6):
        return 4
    elif(res1>0.5):
        return 1
    elif(res_1>0.5):
        return 1
    else:
        return -1
    

def get_preds(test_folder , model):
    y = []
    imgs = glob(test_folder+"*.jpg") + glob(test_folder+"*.png")

    for i in range(len(imgs)):
        img = Image.open(imgs[i]).convert("RGB")
        name = imgs[i].split("/")[-1].split(".")[0]
        isWall, cropped = transform_image(img)
        
        if isWall:
            img=np.array(img)
            class_num=match_template_in_case_of_wall(img)
            if(class_num!=-1):
                y.append((int(name), int(class_num)))
            else:
                y.append((int(name) , 0))  
        else:
                img_bw = cv2.cvtColor(cropped , cv2.COLOR_RGB2GRAY)
                hog = cv2.HOGDescriptor((64,64),(16,16),(8,8),(8,8),9,1,4.0,0,2.0e-1,0,64)
                hist = hog.compute(img_bw)
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
    
def main(argv):
    Train=argv[1]

    if(int(Train)):
        print("Training Model")
        DIR = "./2022Fheldout/"
        warnings.filterwarnings("ignore")
        train_folder = DIR
        train_list_txt = os.path.join(train_folder , "train.txt")
        train_list = []

        f = open(train_list_txt , 'r')
        train_list = list(f)
        train_list = [(int(i.split('\n')[0].split(',')[0]) , int(i.split('\n')[0].split(',')[1])) for i in train_list] 

        X_train, y_train = createDataset(train_folder, train_list)
        X_train_hist,y_train_hist = create_dataset_hist(X_train , y_train)
        SVM = svm.LinearSVC()
        SVM.fit(X_train_hist, y_train_hist)
        filename = 'model.pkl'
        pickle.dump(SVM, open(filename, 'wb'))

    warnings.filterwarnings("ignore")

    if len(argv)!=3:
        DIR = "./2022Fimgs/"
    else:
        DIR = str(argv[2])

    test_list_txt = os.path.join(DIR , "labels.txt")
    f = open(test_list_txt , 'r')
    y_labels = list(f)
    y_labels = [(int(i.split('\n')[0].split(',')[0]) , int(i.split('\n')[0].split(',')[1])) for i in y_labels]
    
    SVM_CLASSIFIER = pickle.load(open('./7785_model.pkl', 'rb'))
    y_pred = get_preds(DIR, SVM_CLASSIFIER)
    metrics(y_labels, y_pred)

if __name__=='__main__':
    main(sys.argv)
