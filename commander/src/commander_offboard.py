#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge

from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, TwistStamped
from object_detection.msg import StringArray, ObjectsArray
from sensor_msgs.msg import Image

from mavros_msgs.srv import SetMode

import time
local_position_current = PoseStamped() 

pError = 0
pid = [0.001, 0, 0]
def object_callback(data):
    global objects
    objects = data.objects

def depth_image_callback(data):
    global cv_image_depth
    cv_image_depth = bridge.imgmsg_to_cv2(data)

def state_callback(state):
    global local_position_current
    global cv_image_depth
    global objects
    global pid
    global pError

    mode = state.mode    
    pose = PoseStamped()
    velocity = TwistStamped() 
    if mode == "OFFBOARD":
        velocity.twist.linear.x = 0
        velocity.twist.linear.y = 0
        velocity.twist.linear.z = 0
        local_vel_pub.publish(velocity)
        for obj in objects:
            if obj.name == "person":
                pError = tracking(cv_image_depth, obj, pError, pid)


def tracking(image, obj, pErr, pid):
    vel = TwistStamped() # local frame ??
    object_x = int((obj.UL_x+obj.BR_x)/2)
    object_y = int((obj.UL_y+obj.BR_y)/2)
    object_z = cv_image_depth[object_y][object_x]
    image_x = cv_image_depth.shape[1]
    image_y = cv_image_depth.shape[0]
    print("z : ", object_z)
    
    error = -(object_x - image_x/2)
    angular_z = error*pid[0]+(error-pErr)*pid[2]
    print(angular_z)
    vel.twist.angular.x = 0
    vel.twist.angular.y = 0
    vel.twist.angular.z = angular_z

    if object_z<5:
        vel.twist.linear.x = 1
    if object_z>7:
        vel.twist.linear.x = -1
    else:
        vel.twist.linear.x = 0
    
    vel.twist.linear.y = 0 
    vel.twist.linear.z = 0 
    pError = error
    local_vel_pub.publish(vel)
    return pError

    
if __name__ == "__main__":
    rospy.init_node("commander")
    bridge = CvBridge()
    local_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size =1)
    rospy.Subscriber('/mavros/state', State, state_callback)
    rospy.Subscriber("/yolo/object_yolo", ObjectsArray, object_callback)
    
    rospy.Subscriber("/camera/depth/image_raw",Image,depth_image_callback)

    rospy.spin()
