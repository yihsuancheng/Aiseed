#!/usr/bin/env python2
import time
import os

# os.environ['CUDA_VISIBLE_DEVICES'] = '-1'

import tensorflow as tf
physical_devices = tf.config.experimental.list_physical_devices('GPU')
if len(physical_devices) > 0:
	tf.config.experimental.set_memory_growth(physical_devices[0], True)
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

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

input_size = 416
iou = 0.45
score = 0.25

video = "udpsrc port=5500 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink"
# video = "rtspsrc location=rtsp://192.168.144.26:8554/main.264 ! decodebin clock-rate=(int)900 ! videoconvert ! appsink max-buffers=1 drop=true sync=false"
# video = "rtsp://192.168.144.25:8554/main.264"
# video = 0
weights_path = "./checkpoints/yolov4-tiny-416"


### gstreamer ###
IP = "127.0.0.1"
port = "5600"
src = "appsrc"
fps = 25

w = 1280
h = 720

gst_out = f"{src} ! videoconvert ! videoscale ! video/x-raw,width={w},height={h} ! x264enc tune=zerolatency byte-stream=true bitrate=3000 threads=2 ! h264parse config-interval=1 ! rtph264pay ! udpsink host={IP} port={port} "

out= cv2.VideoWriter(gst_out, cv2.CAP_GSTREAMER, 0 , float(fps), (int(w), int(h)))

def main():
    config = ConfigProto()
    config.gpu_options.allow_growth = True
    session = InteractiveSession(config=config)
    vid = cv2.VideoCapture(video)
    saved_model_loaded = tf.saved_model.load(weights_path, tags=[tag_constants.SERVING])
    infer = saved_model_loaded.signatures['serving_default']
    
    while True:
        return_value, frame = vid.read()
        if return_value != 1:
            continue
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # image = Image.fromarray(frame)
       
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
            # print(value)

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
        image, _= utils.draw_bbox(frame, pred_bbox)

        # print(type(image))
        curr_time = time.time()
        exec_time = curr_time - prev_time
        result = np.asarray(image)
        # print(result)
        info = "time: %.2f ms" %(1000*exec_time)
        print(info)

        result = cv2.cvtColor(result, cv2.COLOR_RGB2BGR)
        cv2.namedWindow("result", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("result", result)
        out.write(result)
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break

if __name__ == '__main__':
	main()
