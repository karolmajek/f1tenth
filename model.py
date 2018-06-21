"""Model by Karol Majek."""
from keras.callbacks import ModelCheckpoint, EarlyStopping, TensorBoard
from keras.layers import Flatten, Conv2D  # ,Input,  Lambda, ELU, MaxPooling2D
from keras.layers.core import Dense, Dropout
from keras.utils.vis_utils import plot_model
from keras.models import Sequential  # , Model
from keras.models import load_model
from keras import backend as K
import matplotlib.pyplot as plt
import tensorflow as tf
from tqdm import tqdm
import numpy as np
import argparse
import cv2
import pickle
import time
import os
from models import model001

config = tf.ConfigProto()
config.gpu_options.allow_growth = True
session = tf.Session(config=config)
K.set_session(session)

BATCH_SIZE = 1024
EPOCHS = 1000
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
    model.add(Dropout(0.3))
    model.add(Flatten())
    model.add(Dense(512, activation='elu'))
    model.add(Dropout(0.3))
    model.add(Dense(128, activation='elu'))
    model.add(Dropout(0.3))
    model.add(Dense(64, activation='elu'))
    model.add(Dropout(0.3))
    model.add(Dense(1))

    model.compile(loss='mse', optimizer="adam", metrics=['mse', 'mape'])
    print(model.summary())
    return model


def LoadData(list, dataset, procImg_func, procAng_func):
    with open(list, 'r') as f:
        lines = f.readlines()
        print(len(lines), lines[0])

    images = []
    angles = []

    for f in tqdm(lines):
        fname = dataset + '/' + f.strip()
        img = cv2.imread(fname)
        img = procImg_func(img)
        # hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        # cv2.imshow("processed_h", hsv[:,:,0])
        # cv2.imshow("processed_v", hsv[:,:,2])

        # cv2.imshow("processed", img)
        # cv2.imshow("processed_s", hsv[:,:,0])
        # cv2.waitKey(0)
        # images.append(img)
        angles.append(procAng_func(int(f.split('_')[1])))
        images.append(img/255-0.5)
        angles.append(- procAng_func(int(f.split('_')[1])))
        images.append(img[:, ::-1, :]/255-0.5)
        # images.append(hsv[:, :, 0]/255-0.5)
    return np.array(images), np.array(angles)


def ProcessImage(img):
    return img[int(img.shape[0]/2)::8, ::16, :]


def ProcessAngle(angle):
    return (angle - 800) / (1800-800) - 0.5


def FilterData(images, angles):
    new_images = []
    new_angles = []

    for img, angle in zip(images, angles):
        if angle > 0.15 or angle < -0.15:
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
    except (AttributeError,  EOFError, ImportError, IndexError,
            pickle.UnpicklingError, Exception) as e:
        print(e)
        print("Loading data from images; Pickling")
        data = LoadData(args.list, args.dataset, model001.ProcessImage,
                        model001.ProcessAngle)
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
    n, bins, patches = plt.hist(100*(data[1]+0.5), 100, facecolor='green',
                                alpha=0.75)

    # add a 'best fit' line
    plt.plot(bins)
    plt.xlabel('Stering')
    plt.ylabel('Images count')
    plt.title('Dataset 001 - histogram')
    plt.grid(True)
    # plt.show()

    filtered = FilterData(data[0], data[1])

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
    n, bins, patches = plt.hist(100*(filtered[1]+0.5), 100, facecolor='green',
                                alpha=0.75)

    # add a 'best fit' line
    plt.plot(bins)
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

    model_function = model001.Model001

    model = model_function((filtered[0].shape[1:]))
    plot_model(model, to_file='images/%s.png' % (model_function.__name__),
               show_shapes=True, show_layer_names=True)
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
    early_stop = EarlyStopping(monitor='val_loss', patience=100, verbose=0,
                               mode='auto')

    board = TensorBoard(log_dir=args.output,
                        histogram_freq=0, write_graph=True, write_images=True)

    model.fit(x=filtered[0], y=filtered[1], batch_size=args.batch,
              epochs=args.epoch, callbacks=[checkpointer, board, early_stop],
              validation_split=0.2)
    if args.model:
        model.save(args.model)
    else:
        model.save('my_model.h5')
