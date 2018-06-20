"""Eval on images"""
import tensorflow as tf
config = tf.ConfigProto()
config.gpu_options.allow_growth = True
session = tf.Session(config=config)
from keras import backend as K
K.set_session(session)
# import logging
from keras.layers.core import Dense, Activation, Flatten, Dropout
from keras.layers.convolutional import Convolution2D
# from keras.layers.pooling import MaxPooling2D
from keras.callbacks import ModelCheckpoint, CSVLogger, EarlyStopping, TensorBoard
# from keras.applications.vgg16 import VGG16
# from keras.preprocessing import image
# from keras.applications.vgg16 import preprocess_input
from keras.layers import Input, Dense

from keras.layers import Input, Dense, GlobalAveragePooling2D
from keras.layers import Flatten, Lambda, ELU, Conv2D, MaxPooling2D
from keras.models import Model, Sequential
from keras.regularizers import l2
# from keras import backend as K
import argparse
import os
import cv2
import numpy as np
from tqdm import tqdm
import pickle
import matplotlib.pyplot as plt

# from loader import generate_thunderhill_batches, getDataFromFolder, genSim001, genSim002, genSession5,genPolysync0,genPolysync2,genAll
import time
from keras.models import load_model

BATCH_SIZE = 1024
EPOCHS = 300
DROPOUT = 0.5

def LoadData(list, dataset):
    with open(list, 'r') as f:
        lines = f.readlines()
        print(len(lines), lines[0])

    images = []
    angles = []

    for f in tqdm(lines):
        fname = dataset + '/' + f.strip()
        img = cv2.imread(fname)
        img = ProcessData(img)
        # hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        # cv2.imshow("processed_h", hsv[:,:,0])
        # cv2.imshow("processed_v", hsv[:,:,2])

        # cv2.imshow("processed", img)
        # cv2.imshow("processed_s", hsv[:,:,0])
        # cv2.waitKey(0)
        # images.append(img)
        angles.append(ProcessAngle(int(f.split('_')[1])))
        images.append(img/255-0.5)
        angles.append(- ProcessAngle(int(f.split('_')[1])))
        images.append(img[:,::-1,:]/255-0.5)
        # images.append(hsv[:, :, 0]/255-0.5)
    return np.array(images), np.array(angles)


def ProcessData(img):
    return img[int(img.shape[0]/2)::8, ::16, :]


def ProcessAngle(angle):
    return (angle - 800) / (1800-800) - 0.5


def OriginalAngle(angle):
    return (angle + 0.5) * (1800-800) + 800

def FilterData(images, angles):
    new_images = []
    new_angles = []

    for img, angle in zip(images, angles):
        if angle>0.1 or angle<-0.1:
            new_angles.append(angle)
            new_images.append(img)
    return np.array(new_images), np.array(new_angles)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Steering angle model')
    parser.add_argument('--batch', type=int, default=BATCH_SIZE,
                        help='Batch size.')
    parser.add_argument('--epoch', type=int, default=EPOCHS,
                        help='Number of epochs.')
    parser.add_argument('--dropout', type=float, default=DROPOUT,
                        help='Dropout rate')
    parser.add_argument('--model', type=str, help='Load model')
    parser.add_argument('--output', type=str, required=True,
                        help='Save model here')
    parser.add_argument('--list', type=str, required=True,
                        help='List')
    parser.add_argument('--dataset', type=str, required=True,
                        help='dataset')
    args = parser.parse_args()

    data = LoadData(args.list, args.dataset)
    print(data[0].shape)
    print(data[1].shape)

    print('-------------')
    print('BATCH: {}'.format(args.batch))
    print('EPOCH: {}'.format(args.epoch))
    print('DROPOUT: {}'.format(args.dropout))
    print('Load Weights?: {}'.format(args.model))
    print('Dataset: {}'.format(args.dataset))
    print('Model: {}'.format(args.output))
    print('-------------')

    args.output = 'logs/'+args.output+'_%d' % int(time.time())

    if not os.path.exists(args.output):
        os.makedirs(args.output)


    print('Loading model from file:', args.model)
    model = load_model(args.model)
    counter=0
    y_pred=[]
    y_gt=[]
    for img, angle in list(zip(data[0], data[1]))[::2]:
        img_exp = np.expand_dims(img, axis=0)
        pred = model.predict(img_exp)
        y_gt.append(angle)
        y_pred.append(float(pred))
        image = 255*(img+0.5)
        show = cv2.resize(image.astype(np.uint8),(0,0),fx=10,fy=10)
        cv2.putText(show,"Data: %.0f (%.2f)"%(OriginalAngle(angle),angle),(30,170), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),1,cv2.LINE_AA)
        cv2.putText(show,"Pred: %.0f (%.2f)"%(OriginalAngle(pred),
         pred),(30,200), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),1,cv2.LINE_AA)

        cv2.line(show, (int(show.shape[1]/2), int(show.shape[0])-50),
                        (int(show.shape[1]/2 + angle*show.shape[1]/2), int(show.shape[0])-50),(0,255,0),20)
        cv2.line(show, (int(show.shape[1]/2), int(show.shape[0])-30),(int(show.shape[1]/2 + pred*show.shape[1]/2), int(show.shape[0])-30),(0,0,255),20)

        # cv2.imshow('img', show)
        cv2.imwrite('/tmp/image%08d.jpg' % counter, show)
        # cv2.waitKey(1)
        counter = counter + 1
    for img, angle in list(zip(data[0], data[1]))[1::2]:
        img_exp = np.expand_dims(img, axis=0)
        pred = model.predict(img_exp)
        y_gt.append(angle)
        y_pred.append(float(pred))
        image = 255*(img+0.5)
        show = cv2.resize(image.astype(np.uint8),(0,0),fx=10,fy=10)
        cv2.putText(show,"Data: %.0f (%.2f)"%(OriginalAngle(angle),angle),(30,170), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),1,cv2.LINE_AA)
        cv2.putText(show,"Pred: %.0f (%.2f)"%(OriginalAngle(pred),
         pred),(30,200), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),1,cv2.LINE_AA)

        cv2.line(show, (int(show.shape[1]/2), int(show.shape[0])-50),
                        (int(show.shape[1]/2 + angle*show.shape[1]/2), int(show.shape[0])-50),(0,255,0),20)
        cv2.line(show, (int(show.shape[1]/2), int(show.shape[0])-30),(int(show.shape[1]/2 + pred*show.shape[1]/2), int(show.shape[0])-30),(0,0,255),20)

        # cv2.imshow('img', show)
        cv2.imwrite('/tmp/image%08d.jpg' % counter, show)
        # cv2.waitKey(1)
        counter = counter + 1
    plt.plot(y_gt, 'g')
    plt.plot(y_pred, 'r')
    plt.show()
