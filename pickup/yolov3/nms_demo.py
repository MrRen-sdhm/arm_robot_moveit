#! /usr/bin/env python2
# coding=utf-8
#================================================================
#   Copyright (C) 2018 * Ltd. All rights reserved.
#
#   Editor      : VIM
#   File name   : nms_demo.py
#   Author      : YunYang1994
#   Created date: 2018-11-27 13:02:17
#   Description :
#
#================================================================
from __future__ import print_function

import os
import sys
# sys.path.append("~/.virtualenvs/cv/lib/python3.5/site-packages")
# sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
print("\n",sys.path,"\n")
import cv2
import time
import numpy as np
import tensorflow as tf
from PIL import Image
from core import utils

path = os.path.dirname(__file__)
print ("path:", path)

def proc_image(image):
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_resized = np.array(cv2.resize(image_rgb, (IMAGE_H, IMAGE_W)), dtype=np.float32)
    image_resized = image_resized / 255.
    return image_resized


def proc_image_PIL(image_path):
    img = Image.open(image_path)
    img_resized = np.array(img.resize(size=(IMAGE_H, IMAGE_W)), dtype=np.float32)
    img_resized = img_resized / 255.
    return img_resized


IMAGE_H, IMAGE_W = 416, 416
EPOCHS = 5
# SIZE = [608, 608]
names_path = path + './data/coco.names'
classes = utils.read_coco_names(names_path)
num_classes = len(classes)
image_path = path + "./data/demo_data/dog.jpg"
image_path = path + "./data/demo_data/test1.png"

image_cv = cv2.imread(image_path)
image_PIL = Image.open(image_path)

cpu_nms_graph, gpu_nms_graph = tf.Graph(), tf.Graph()

# nms on GPU
# gpu_nms_pb_path = path + "/checkpoint/yolov3_gpu_nms.pb"
# input_tensor, output_tensors = utils.read_pb_return_tensors(gpu_nms_graph, gpu_nms_pb_path,
#                                            ["Placeholder:0", "concat_10:0", "concat_11:0", "concat_12:0"])
# with tf.Session(graph=gpu_nms_graph) as sess:
#     for i in range(EPOCHS):
#         start = time.time()
#         boxes, scores, labels = sess.run(output_tensors, feed_dict={input_tensor: np.expand_dims(proc_image(image_cv), axis=0)})
#         print("=> nms on gpu the number of boxes= %d  time=%.2f ms" %(len(boxes), 1000*(time.time()-start)))
#     image = utils.draw_boxes_cv(image_cv, boxes, scores, labels, classes, [IMAGE_H, IMAGE_W], show=True)

# nms on CPU
cpu_nms_pb_path = path + "./checkpoint/yolov3_cpu_nms.pb"
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
    cv2.waitKey(0)



