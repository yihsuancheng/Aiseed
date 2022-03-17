#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from object_detection.msg import StringArray

from mavros_msgs.srv import SetMode

import time
local_position_current = PoseStamped() 
def object_callback(data):
    global set_mode_call
    object_names = data.strings
    
    if "person" in object_names:
        pass


def state_callback(state):
    global local_position_current
    mode = state.mode    
    pose = PoseStamped() 

    if mode == "OFFBOARD":
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = local_position_current.pose.position.z
        local_pos_pub.publish(pose)
 
def local_position_callback(position):
    global local_position_current
    local_position_current = position


if __name__ == "__main__":
    rospy.init_node("commander")
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size =1)
    rospy.Subscriber('/mavros/state', State, state_callback)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_position_callback)
    rospy.Subscriber("/object_yolo", StringArray, object_callback)
    rospy.spin()
