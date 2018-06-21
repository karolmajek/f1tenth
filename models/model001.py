"""Model by Karol Majek."""
from keras.layers import Flatten, Conv2D
from keras.layers.core import Dense, Dropout
from keras.models import Sequential


def Model001(input_shape=(30, 40, 3)):
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
    model.add(Dropout(0.2))
    model.add(Flatten())
    model.add(Dense(256, activation='elu'))
    model.add(Dropout(0.2))
    model.add(Dense(128, activation='elu'))
    model.add(Dropout(0.2))
    model.add(Dense(64, activation='elu'))
    model.add(Dropout(0.2))
    model.add(Dense(1))

    model.compile(loss='mse', optimizer="adam", metrics=['mse', 'mape'])
    return model


def ProcessImage(img):
    return img[int(img.shape[0]/2)::8, ::16, :]


def ProcessAngle(angle):
    return (angle - 800) / (1800-800) - 0.5


def OriginalAngle(angle):
    return (angle + 0.5) * (1800-800) + 800
