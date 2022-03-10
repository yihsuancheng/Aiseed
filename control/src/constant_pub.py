#!/usr/bin/env python3
from pynput import keyboard
import rospy
from mavros_msgs.msg import ManualControl


manual_cmd = ManualControl()

manual_cmd.x = 0 
manual_cmd.y = 0 
manual_cmd.z = 0 
manual_cmd.r = 0 

def main_pub():
    global manual_cmd
    while not rospy.is_shutdown():
        key_pub.publish(manual_cmd)


if __name__=="__main__":
    rospy.init_node("constant_pub")
    key_pub = rospy.Publisher('/constant_value', ManualControl, queue_size=10)
    main_pub()
