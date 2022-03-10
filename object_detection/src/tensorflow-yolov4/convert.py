import tensorflow as tf
from absl import app, flags, logging
from absl.flags import FLAGS
import numpy as np
import cv2
from core.yolov4 import YOLOv4, YOLOv3, YOLOv3_tiny, decode
import core.utils as utils
import os
from core.config import cfg

flags.DEFINE_string('weights', './data/yolov4.weights', 'path to weights file')
flags.DEFINE_string('output', './data/yolov4-pb', 'path to output')
flags.DEFINE_boolean('tiny', False, 'path to output')
flags.DEFINE_integer('input_size', 416, 'path to output')
flags.DEFINE_string('model', 'yolov4', 'yolov3 or yolov4')
flags.DEFINE_string('dataset', "/media/user/Source/Data/coco_dataset/coco/5k.txt", 'path to dataset')

def representative_data_gen():
  fimage = open(FLAGS.dataset).read().split()
  for input_value in range(100):
    if os.path.exists(fimage[input_value]):
      original_image=cv2.imread(fimage[input_value])
      original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
      image_data = utils.image_preporcess(np.copy(original_image), [FLAGS.input_size, FLAGS.input_size])
      img_in = image_data[np.newaxis, ...].astype(np.float32)
      print(input_value)
      yield [img_in]
    else:
      continue

def save_tf():
  NUM_CLASS = len(utils.read_class_names(cfg.YOLO.CLASSES))
  input_layer = tf.keras.layers.Input([FLAGS.input_size, FLAGS.input_size, 3])
  if FLAGS.tiny:
    feature_maps = YOLOv3_tiny(input_layer, NUM_CLASS)
    bbox_tensors = []
    for i, fm in enumerate(feature_maps):
      bbox_tensor = decode(fm, NUM_CLASS, i)
      bbox_tensors.append(bbox_tensor)
    model = tf.keras.Model(input_layer, bbox_tensors)
    utils.load_weights_tiny(model, FLAGS.weights)
  else:
    if FLAGS.model == 'yolov3':
      feature_maps = YOLOv3(input_layer, NUM_CLASS)
      bbox_tensors = []
      for i, fm in enumerate(feature_maps):
        bbox_tensor = decode(fm, NUM_CLASS, i)
        bbox_tensors.append(bbox_tensor)
      model = tf.keras.Model(input_layer, bbox_tensors)
      utils.load_weights_v3(model, FLAGS.weights)
    elif FLAGS.model == 'yolov4':
      feature_maps = YOLOv4(input_layer, NUM_CLASS)
      bbox_tensors = []
      for i, fm in enumerate(feature_maps):
        bbox_tensor = decode(fm, NUM_CLASS, i)
        bbox_tensors.append(bbox_tensor)
      model = tf.keras.Model(input_layer, bbox_tensors)
      utils.load_weights(model, FLAGS.weights)

    model = tf.keras.Model(input_layer, bbox_tensors)
    model.summary()
    utils.load_weights(model, FLAGS.weights)

  model.save(FLAGS.output)

  logging.info("model saved to: {}".format(FLAGS.output))

def demo():
  model = tf.keras.models.load_model(FLAGS.output, compile=False)
  logging.info('tf model loaded')

  input_shape = model.inputs[0].shape[1:]
  input_data = np.array([np.random.random_sample(input_shape)], dtype=np.float32)

  output_data = model.predict(input_data)
  
  print(output_data)

def main(_argv):
  save_tf()
  demo()

if __name__ == '__main__':
    try:
        app.run(main)
    except SystemExit:
        pass


