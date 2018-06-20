#!/usr/bin/env python3.4

import json
import numpy as np
import time
import cv2
import tensorflow as tf
from keras.models import model_from_json
from keras.preprocessing.image import ImageDataGenerator, array_to_img, img_to_array
from keras.models import Sequential
from keras.optimizers import Adam
from keras.layers.normalization import BatchNormalization
from sklearn.metrics import mean_squared_error
from keras.layers import ZeroPadding2D,Dense, Input, Activation, Dropout, Conv2D, Flatten, MaxPooling2D, Activation, Lambda
import rospy, time, sys, select, termios, tty
##from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
 

class move_auto:
    def __init__(self):
        self.model = Sequential()
        self.model.add(Lambda(lambda x: x/127.5-1.0,input_shape=(60,160,3),output_shape=(60,160,3)))
        self.model.add(BatchNormalization())
        self.model.add(Conv2D(24, (5,5),padding='valid',activation='relu',strides=(2,2)))
        self.model.add(Conv2D(36, (5,5),padding='valid',activation='relu',strides=(2,2)))
        self.model.add(Conv2D(48, (5,5),padding='valid',activation='relu',strides=(2,2)))
        self.model.add(Conv2D(64, (3,3),padding='valid',activation='relu'))
        self.model.add(Flatten())
        self.model.add(Dropout(0.3))
        self.model.add(Dense(100,activation='relu'))
        self.model.add(Dense(50,activation='relu'))
        self.model.add(Dense(10,activation='relu'))
        self.model.add(Dense(1))

        self.model.compile(loss='mse', optimizer='adam') 
        self.graph = tf.get_default_graph()

        weights_file = 'model_1_46_25.h5'
        self.model.load_weights(weights_file)
        
        self.pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=5)
        self.sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback, queue_size = 1)
        
        self.throttle=1500
        self.yaw=1300
        
        self.throttle_channel=2
        self.steer_channel=0
        
        self.msg = OverrideRCIn()
           
    def callback(self, ros_data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.Header)

        np_arr = np.fromstring(ros_data.data, np.uint8)
        image = cv2.imdecode(np_arr,1) #, cv2.CV_LOAD_IMAGE_COLOR)

        image=image[240:480]

        image=cv2.resize(image,None,fx=0.25, fy=0.25, interpolation = cv2.INTER_AREA)

        x=image[None,:,:,:]
        
        with self.graph.as_default():
            self.yaw=int(self.model.predict(x))
        
        #self.throttle=1546
        rospy.loginfo(self.yaw)

        


        #r = rospy.Rate(10) #10hz
        
        
        self.msg.channels[self.throttle_channel]=self.throttle
        self.msg.channels[self.steer_channel]=self.yaw

        self.pub.publish(self.msg)
def main(args):
    ma=move_auto()
    rospy.init_node('tryrover_node', anonymous=True)
    rospy.wait_for_service('/mavros/set_mode')
    change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    answer = change_mode(custom_mode='manual')
    print (answer)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
if __name__ == '__main__':
    main(sys.argv)
    
    
    
       
