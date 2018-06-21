#!/usr/bin/env python3.4
import numpy as np
import cv2
import tensorflow as tf
import rospy
import sys
from sensor_msgs.msg import CompressedImage
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode
from models import model001


class move_auto:
    def __init__(self):
        self.model = model001.Model001()
        self.graph = tf.get_default_graph()

        weights_file = '001-weights.0040-0.036.hdf5'
        self.model.load_weights(weights_file)

        self.pub = rospy.Publisher('mavros/rc/override', OverrideRCIn,
                                   queue_size=5)
        self.sub = rospy.Subscriber("/camera/image/compressed",
                                    CompressedImage, self.callback,
                                    queue_size=1)

        self.throttle = 1500
        self.yaw = 1300

        self.throttle_channel = 2
        self.steer_channel = 0

        self.msg = OverrideRCIn()

    def callback(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image = cv2.imdecode(np_arr, 1)

        image = model001.ProcessImage(image)
        image = np.expand_dims(image, axis=0)

        with self.graph.as_default():
            self.yaw = model001.OriginalAngle(self.model.predict(image))

        rospy.loginfo(self.yaw)

        self.msg.channels[self.throttle_channel] = self.throttle
        self.msg.channels[self.steer_channel] = self.yaw

        self.pub.publish(self.msg)


def main(args):
    ma = move_auto()
    rospy.init_node('tryrover_node', anonymous=True)
    rospy.wait_for_service('/mavros/set_mode')
    change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    answer = change_mode(custom_mode='manual')
    print(answer)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")


if __name__ == '__main__':
    main(sys.argv)
