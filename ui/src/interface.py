#!/usr/bin/env python3
from __future__ import print_function
import sys
import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, UInt16, UInt8, Float64MultiArray
from sensor_msgs.msg import Image
from gazebo_msgs.srv import GetJointProperties

import os

x_pixel = 0 
y_pixel = 0
z = 0

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        # rospy.Subscriber("/camera/rgb/image_raw",Image,self.color_image_callback)
        # rospy.Subscriber("/camera/depth/image_raw",Image,self.depth_image_callback)
        rospy.Subscriber("/yolo/image_yolo",Image,self.yolo_image_callback)
    
    def color_image_callback(self,data):
        try:
            cv_image_color = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)

        cv2.namedWindow("color_image")

        cv2.imshow("color_image",cv_image_color)
        cv2.waitKey(1)

    def depth_image_callback(self,data):
        global x_pixel,y_pixel,z
        try:
            cv_image_depth = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
        # print(cv_image_depth[y_pixel][x_pixel])
        z = cv_image_depth[y_pixel][x_pixel] # m

        # cv2.imshow("depth_image", cv_image_depth)
        # key = cv2.waitKey(1)
    
    def yolo_image_callback(self,data):
        try:
            cv_image_yolo = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)


        cv2.namedWindow("yolo_image")

        cv2.imshow("yolo_image",cv_image_yolo)
        if cv2.waitKey(1)  == ord('q'):
            os._exit(0)



def main(args):
    print("UI starts!!!")
    ic = image_converter()
    rospy.init_node('ui')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
