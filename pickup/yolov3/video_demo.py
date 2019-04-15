#! /usr/bin/env python2
# coding=utf-8
#================================================================
#   Copyright (C) 2018 * Ltd. All rights reserved.
#
#   Editor      : VIM
#   File name   : video_demo.py
#   Author      : YunYang1994
#   Created date: 2018-11-30 15:56:37
#   Description :
#
#================================================================


import os
import sys
# sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
print("\n",sys.path,"\n")
import cv2 
import rospy
import time
import numpy as np
import tensorflow as tf
from PIL import Image
from core import utils

path = os.path.dirname(__file__) 


def proc_image(image):
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_resized = np.array(cv2.resize(image_rgb, (IMAGE_H, IMAGE_W)), dtype=np.float32)
    image_resized = image_resized / 255.
    return image_resized


IMAGE_H, IMAGE_W = 416, 416
video_path = path + "/data/demo_data/road.mp4"
video_path = 0 # use camera
names_path = path + '/data/coco.names'
classes = utils.read_coco_names(names_path)
num_classes = len(classes)
pb_path = path + "/checkpoint/yolov3_cpu_nms.pb"
input_tensor, output_tensors = utils.read_pb_return_tensors(tf.get_default_graph(),
                                                            pb_path, ["Placeholder:0", "concat_9:0", "mul_6:0"])
with tf.Session() as sess:
    vid = cv2.VideoCapture(video_path)
    while True:
        prev_time = time.time()

        return_value, frame = vid.read()
        if not return_value:
            raise ValueError("No image!")            

        boxes, scores = sess.run(output_tensors, feed_dict={input_tensor: np.expand_dims(proc_image(frame), axis=0)})
        boxes, scores, labels = utils.cpu_nms(boxes, scores, num_classes, score_thresh=0.4, iou_thresh=0.5)
        image = utils.draw_boxes_cv(frame, boxes, scores, labels, classes, (IMAGE_H, IMAGE_W), show=False)

        curr_time = time.time()
        exec_time = curr_time - prev_time
        info = "fps: %.2f time: %.2f ms" % (1/exec_time, 1000*exec_time)
        cv2.putText(image, info, (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        
        cv2.namedWindow("result", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("result", image)
        if cv2.waitKey(1) & 0xFF == 27: break


