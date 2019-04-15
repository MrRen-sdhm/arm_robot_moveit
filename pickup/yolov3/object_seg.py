#! /usr/bin/env python3
# coding=utf-8

# 目标物体分割

from __future__ import print_function

import os
import sys
# sys.path.append("~/.virtualenvs/cv/lib/python3.5/site-packages")
# sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
# print("\n",sys.path,"\n")
import cv2
import time
import numpy as np
import tensorflow as tf
from core import utils

np.set_printoptions(threshold=np.inf)

def proc_image(image):
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_resized = np.array(cv2.resize(image_rgb, (IMAGE_H, IMAGE_W)), dtype=np.float32)
    image_resized = image_resized / 255.
    return image_resized

IMAGE_H, IMAGE_W = 416, 416
EPOCHS = 5
names_path = './data/coco.names'
classes = utils.read_coco_names(names_path)
num_classes = len(classes)
image_path = "./data/demo_data/0000_color.jpg"
image_depth_path = "./data/demo_data/0000_depth_colored.png"

image_cv = cv2.imread(image_path)
depth = cv2.imread(image_depth_path)
# print (np.asarray(depth))

cpu_nms_graph, gpu_nms_graph = tf.Graph(), tf.Graph()

# nms on CPU
cpu_nms_pb_path = "./checkpoint/yolov3_cpu_nms.pb"
input_tensor, output_tensors = utils.read_pb_return_tensors(cpu_nms_graph, cpu_nms_pb_path,
                                           ["Placeholder:0", "concat_9:0", "mul_6:0"])
with tf.Session(graph=cpu_nms_graph) as sess:
    for i in range(EPOCHS):
        start = time.time()
        boxes, scores = sess.run(output_tensors, feed_dict={input_tensor: np.expand_dims(proc_image(image_cv), axis=0)})
        boxes, scores, labels = utils.cpu_nms(boxes, scores, num_classes, score_thresh=0.5, iou_thresh=0.5)
        print("=> nms on cpu the number of boxes= %d  time=%.2f ms" %(len(boxes), 1000*(time.time()-start)))
    image, boxes = utils.draw_boxes_cv(image_cv, boxes, scores, labels, classes, [IMAGE_H, IMAGE_W], show=True)
    
    cv2.imshow("img", image)

    gray = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2GRAY)
    ret, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    cv2.imshow("threshold", threshold)
    
    cv2.imshow("depth", depth)
    cv2.waitKey(0)



