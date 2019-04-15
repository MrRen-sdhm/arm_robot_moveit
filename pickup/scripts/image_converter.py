#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv_bridge
from cv_bridge.core import CvBridge, CvBridgeError
import numpy as np
import tensorflow as tf

class image_converter():
    def __init__(self):
        image_topic = "/kinect2/qhd/image_color_rect"

        self._cv_bridge = cv_bridge.core.CvBridge()
        # self.image_pub = rospy.Publisher("image_topic_2",Image)

        self._sub = rospy.Subscriber(image_topic, Image, self.callback, queue_size=1)
        # self._pub = rospy.Publisher('result', String, queue_size=1)

    def callback(self, image_msg):
        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            cv2.imshow("img", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print (e)

        # rospy.loginfo('%s (score = %.5f)' % (human_string, score))
        # self._pub.publish(human_string)


def main():
    rospy.init_node('ros_tensorflow_image_recognition', anonymous=True)
    image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
