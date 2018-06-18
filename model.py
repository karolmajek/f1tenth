"""Model by Karol Majek."""
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
# from keras.models import load_model

BATCH_SIZE = 1024
EPOCHS = 300
DROPOUT = 0.5

def SimpleCNN(input_shape, dropout):
    model = Sequential()
    model.add(Conv2D(8, kernel_size=(5, 5),
                 activation='elu',
                 input_shape=input_shape))
    model.add(Conv2D(16, (5, 5), activation='elu'))
    model.add(Conv2D(24, (5, 5), activation='elu'))
    model.add(Conv2D(32, (5, 5), activation='elu'))
    model.add(Conv2D(48, (3, 3), activation='elu'))
    model.add(Conv2D(64, (3, 3), activation='elu'))
    model.add(Conv2D(128, (3, 3), activation='elu'))
    model.add(Conv2D(256, (3, 3), activation='relu'))
    model.add(Dropout(0.7))
    model.add(Flatten())
    model.add(Dense(256, activation='elu'))
    model.add(Dropout(0.5))
    model.add(Dense(128, activation='elu'))
    model.add(Dropout(0.5))
    model.add(Dense(64, activation='elu'))
    model.add(Dropout(0.5))
    model.add(Dense(1))

    model.compile(loss='mse', optimizer="adam", metrics=['mse', 'mape'])
    print(model.summary())
    return model


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

    try:
        with open("dataset001.p", 'rb') as f:
            data = pickle.load(f)
    except:
        data = LoadData(args.list, args.dataset)
        with open("dataset001.p", 'wb') as f:
            pickle.dump(data, f)
    print(data[0].shape)
    print(data[1].shape)

    # cv2.imshow("img", data[0])
    # cv2.waitKey(0)

    print(np.min(data[1]), np.max(data[1]))

    plt.plot(data[1])
    plt.xlabel('Image')
    plt.ylabel('Angle')
    plt.title('Dataset 001 - Stfeering angle')
    plt.grid(True)
    # plt.show()


    # the histogram of the data
    n, bins, patches = plt.hist(100*(data[1]+0.5), 100, facecolor='green', alpha=0.75)

    # add a 'best fit' line
    l = plt.plot(bins)
    plt.xlabel('Stering')
    plt.ylabel('Images count')
    plt.title('Dataset 001 - histogram')
    plt.grid(True)
    # plt.show()

    # filtered = FilterData(data[0], data[1])
    filtered = [data[0], data[1]]

    print(filtered[0].shape)
    print(filtered[1].shape)
    print(np.min(filtered[0]), np.max(filtered[0]))
    print(np.min(filtered[1]), np.max(filtered[1]))

    plt.plot(filtered[1])
    plt.xlabel('Image')
    plt.ylabel('Angle')
    plt.title('Dataset 001 - Steering angle filtered')
    plt.grid(True)
    # plt.show()


    # the histogram of the data
    n, bins, patches = plt.hist(100*(filtered[1]+0.5), 100, facecolor='green', alpha=0.75)

    # add a 'best fit' line
    l = plt.plot(bins)
    plt.xlabel('Stering')
    plt.ylabel('Images count')
    plt.title('Dataset 001 - Histogram filtered')
    plt.grid(True)
    # plt.show()

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


    print('='*80)
    print(filtered[0].shape)
    filtered = [filtered[0],
                np.expand_dims(filtered[1], axis=1)]
    print(filtered[0].shape)
    print(filtered[1].shape)

    model = SimpleCNN((filtered[0].shape[1:]), args.dropout)

    # returns a compiled model
    # identical to the previous one
    try:
        if args.model:
            print('Loading model from file:', args.model)
            model = load_model(args.model)
    except IOError:
        print("No model found")

    checkpointer = ModelCheckpoint(
        os.path.join(args.output,
                     'weights.{epoch:04d}-{val_loss:.3f}.hdf5'),
        monitor='val_loss', verbose=0, save_best_only=True,
        save_weights_only=False, mode='auto', period=10)
    # early_stop = EarlyStopping(monitor='val_loss', patience=50,
    # verbose=0, mode='auto')

    board = TensorBoard(log_dir=args.output,
                        histogram_freq=0, write_graph=True, write_images=True)

    model.fit(x=filtered[0], y=filtered[1], batch_size=args.batch,
              epochs=args.epoch, callbacks=[checkpointer, board], validation_split=0.2)
    if args.model:
        model.save(args.model)
    else:
        model.save('my_model.h5')
