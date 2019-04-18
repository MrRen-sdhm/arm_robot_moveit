#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import sys
import rospy
import cv2
import os
import time
import threading
import cv_bridge
import numpy as np
import tensorflow as tf
from sensor_msgs.msg import Image
from pickup.msg import PickupObj
from cv_bridge.core import CvBridge, CvBridgeError

from core import utils

path = os.path.dirname(__file__) 


class objDetect():
    def __init__(self, gpu_nms=True):  
        rospy.init_node('ros_tensorflow_image_recognition', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        self._cv_bridge = cv_bridge.core.CvBridge()

        image_topic = "/kinect2/qhd/image_color_rect"        
        self._sub = rospy.Subscriber(image_topic, Image, self.callback, queue_size=1)
        self._pub = rospy.Publisher('detected_pickup_object', Image, queue_size=1)
        self._msg_pub = rospy.Publisher('PickupObjMsg', PickupObj, queue_size=1)

        # 待抓取物体信息
        self.obj_msg = PickupObj()

        # 加载模型
        self.IMAGE_H, self.IMAGE_W = 416, 416
        names_path = path + '/data/coco.names'
        self.classes = utils.read_coco_names(names_path)
        self.num_classes = len(self.classes)

        self.detection_graph = tf.get_default_graph()
        if gpu_nms: 
            rospy.loginfo("Use GPU NMS...")
            pb_path = path + "/checkpoint/yolov3_gpu_nms.pb"
            input_tensor, output_tensors = utils.read_pb_return_tensors(self.detection_graph, 
                                                                    pb_path, ["Placeholder:0", "concat_10:0", "concat_11:0", "concat_12:0"])
        else: 
            rospy.loginfo("Use CPU NMS...")
            pb_path = path + "/checkpoint/yolov3_cpu_nms.pb"        
            input_tensor, output_tensors = utils.read_pb_return_tensors(self.detection_graph,
                                                                    pb_path, ["Placeholder:0", "concat_9:0", "mul_6:0"])

        self.gpu_nms = gpu_nms
        self.cv_image = None
        self.input_tensor, self.output_tensors = input_tensor, output_tensors

        # 启动目标检测线程
        thread =threading.Thread(target=self.objDetect,args=(1,))
        thread.setDaemon(True)
        thread.start()

    def objDetect(self, arg):
        with tf.Session(graph=self.detection_graph) as sess:
            rospy.loginfo("Object dectecter is running...")
            while(True):
                prev_time = time.time()
                # 添加阻塞
                time.sleep(0.040) # 0.015->17fps  0.020->13fps  0.040->10fps

                # 物体识别及非极大值抑制
                if self.gpu_nms:
                    boxes, scores, labels = sess.run(self.output_tensors, feed_dict={self.input_tensor: np.expand_dims(self.proc_image(self.cv_image), axis=0)})
                else:
                    boxes, scores = sess.run(self.output_tensors, feed_dict={self.input_tensor: np.expand_dims(self.proc_image(self.cv_image), axis=0)})
                    boxes, scores, labels = utils.cpu_nms(boxes, scores, self.num_classes, score_thresh=0.4, iou_thresh=0.5)

                image, boxes = utils.draw_boxes_cv(self.cv_image, boxes, scores, labels, self.classes, (self.IMAGE_H, self.IMAGE_W), show=False)

                # fps计算
                curr_time = time.time()
                exec_time = curr_time - prev_time
                info = "fps: %.2f, time: %.2f ms" % (1/exec_time, 1000*exec_time)                
                # rospy.loginfo("fps: %.2f, time: %.2f ms" % (1/exec_time, 1000*exec_time))

                # 发布物体坐标及标签到ROS
                self.pub_obj_msg(boxes, labels)

                # 发布处理后的图像到ROS
                cv2.putText(image, info, (0, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
                image_ros = self._cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
                self._pub.publish(image_ros)

                # opencv显示处理后的图像                
                # cv2.imshow("img", image)
                # if cv2.waitKey(1) & 0xFF == 27: 
                #     cv2.destroyAllWindows()

    def pub_obj_msg(self, boxes, labels):
        self.obj_msg.boxes = boxes.reshape(-1)
        self.obj_msg.labels = labels
        self._msg_pub.publish(self.obj_msg)
        # print( self.obj_msg.boxes, "shape:", np.array(self.obj_msg.boxes).shape, "\n")

    def shutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo("Object dectecter is stopped...")

    def callback(self, image_msg):
        self.cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
    
    def proc_image(self, image):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_resized = np.array(cv2.resize(image_rgb, (self.IMAGE_H, self.IMAGE_W)), dtype=np.float32)
        image_resized = image_resized / 255.
        return image_resized


def main(): 
    try:
        objDetect(gpu_nms=True)
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
