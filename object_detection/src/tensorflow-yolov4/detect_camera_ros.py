#!/usr/bin/env python3
import time
import os

os.environ['CUDA_VISIBLE_DEVICES'] = '-1'


import tensorflow as tf
physical_devices = tf.config.experimental.list_physical_devices('GPU')
if len(physical_devices) > 0:
    tf.config.experimental.set_memory_growth(physical_devices[0], True)

from absl import app, flags, logging
from absl.flags import FLAGS
import core.utils as utils
from core.yolov4 import filter_boxes
from tensorflow.python.saved_model import tag_constants
from PIL import Image
import cv2
import numpy as np
from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession

import sys

import rospy
import sensor_msgs.msg
from object_detection.msg import StringArray
from cv_bridge import CvBridge

input_size = 416
iou = 0.45
score = 0.25
weights_path = os.environ['YOLOSIM']+"/checkpoints/yolov4-tiny-416"

config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)
saved_model_loaded = tf.saved_model.load(weights_path, tags=[tag_constants.SERVING])
infer = saved_model_loaded.signatures['serving_default']
 
def main_callback(raw_image):
  
    frame = bridge.imgmsg_to_cv2(raw_image, desired_encoding='passthrough')
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image = Image.fromarray(frame)
        
    frame_size = frame.shape[:2]
    image_data = cv2.resize(frame, (input_size, input_size))
    image_data = image_data / 255.
    image_data = image_data[np.newaxis, ...].astype(np.float32)
    prev_time = time.time()

    batch_data = tf.constant(image_data)
    pred_bbox = infer(batch_data)
    for key, value in pred_bbox.items():
        boxes = value[:, :, 0:4]
        pred_conf = value[:, :, 4:]

    boxes, scores, classes, valid_detections = tf.image.combined_non_max_suppression(
        boxes=tf.reshape(boxes, (tf.shape(boxes)[0], -1, 1, 4)),
        scores=tf.reshape(
            pred_conf, (tf.shape(pred_conf)[0], -1, tf.shape(pred_conf)[-1])),
        max_output_size_per_class=50,
        max_total_size=50,
        iou_threshold=iou,
        score_threshold=score
    )
    pred_bbox = [boxes.numpy(), scores.numpy(), classes.numpy(), valid_detections.numpy()]
    image, object_names = utils.draw_bbox(frame, pred_bbox)
    
    curr_time = time.time()
    exec_time = curr_time - prev_time
    result = np.asarray(image)
    info = "time: %.2f ms" %(1000*exec_time)
    cv2.putText(result, info, (10, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 1, cv2.LINE_AA)

    ros_image = bridge.cv2_to_imgmsg(result, encoding="passthrough")
    yolo_image_pub.publish(ros_image)
    ### publish objects
    names = StringArray()
    names.strings = object_names
    yolo_object_pub.publish(names)
    
    # ??? 
    # result = cv2.cvtColor(result, cv2.COLOR_RGB2BGR)
    # cv2.namedWindow("result", cv2.WINDOW_AUTOSIZE)
    # cv2.imshow("result", result)
    # if cv2.waitKey(1) & 0xFF == ord('q'): 
    #      os._exit(0)

if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node("yolo")
    rospy.Subscriber("/camera/rgb/image_raw", sensor_msgs.msg.Image, main_callback)
    yolo_image_pub = rospy.Publisher("/image_yolo", sensor_msgs.msg.Image, queue_size=1)
    yolo_object_pub = rospy.Publisher("/object_yolo", StringArray , queue_size=1)
    rospy.spin()
