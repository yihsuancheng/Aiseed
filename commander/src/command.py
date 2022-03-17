#!/usr/bin/env python3

import rospy
import sensor_msgs.msg
from object_detection.msg import StringArray
from mavros_msgs.srv import SetMode

import time

def object_callback(data):
    global set_mode_call
    object_names = data.strings
    
    if "person" in object_names:
        
        set_mode_call(custom_mode = 'AUTO.LOITER')
        print("mode hold")
        time.sleep(5)
        set_mode_call(custom_mode = 'AUTO.LAND')
        print("mode land")
    
if __name__ == "__main__":
    rospy.init_node("command")
    rospy.wait_for_service('/mavros/set_mode')
    set_mode_call = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    rospy.Subscriber("/object_yolo", StringArray, object_callback)
    rospy.spin()
