#!/usr/bin/env python3.4
#import json
import numpy as np
import time
# import cv2
#import tensorflow as tf
#from keras.models import model_from_json
#from keras.preprocessing.image import ImageDataGenerator, array_to_img, img_to_array
#from keras.models import Sequential
#from keras.optimizers import Adam
#from keras.layers.normalization import BatchNormalization
#from sklearn.metrics import mean_squared_error
#from keras.layers import ZeroPadding2D,Dense, Input, Activation, Dropout, Conv2D, Flatten, MaxPooling2D, Activation, Lambda
import rospy, time, sys, select, termios, tty
##from tf.transformations import quaternion_from_euler
#from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header, Float64
#from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode
#from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
#from cv_bridge import CvBridge, CvBridgeError


class move_auto:
    def __init__(self):

        self.pub = rospy.Publisher('mavros/rc/override', OverrideRCIn,
                                   queue_size=5)
        # self.sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback, queue_size = 1)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.laserCallback,
                                    queue_size=1)

        self.throttle = 0
        self.yaw = 1570

        self.throttle_channel = 2
        self.steer_channel = 0

        self.msg = OverrideRCIn()

    #def originalAngle(self, angle):
    #    return (angle + 0.5) * (1800.0-800.0) + 800.0 + 1570-1300

    def laserCallback(self, msg):
        min_id = 180
        max_id = 900
        groups_count = 10
        scans_in_group = int((max_id-min_id)/float(groups_count))

        scan = np.array(msg.ranges)[min_id:max_id]

        scan[scan>=msg.range_max] = 0
        scan[scan<=msg.range_min] = 0

        groups = [0]*groups_count
        for i in range(groups_count):
            groups[i]=np.sum(scan[i*scans_in_group:(i+1)*scans_in_group])

        max_group_id = np.argmax(np.array(groups))

        angle = -0.5 + max_group_id / float(groups_count)

        if angle >= -0.1 and angle <= 0.1:
            sum1 = np.sum(scan[:180])
            sum2 = np.sum(scan[-180:])

            if sum2 > 1.25*sum1:
                angle += 0.03
            if sum1 > 1.25*sum2:
                angle -= 0.03

        angle = int(1570 + angle * 800 * 1.5)

        self.yaw = angle

        rospy.loginfo(self.yaw)

        self.msg.channels[self.throttle_channel] = self.throttle
        self.msg.channels[self.steer_channel] = self.yaw
        self.pub.publish(self.msg)


def main(args):
    rospy.init_node('tryrover_node', anonymous=True)
    ma = move_auto()
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
