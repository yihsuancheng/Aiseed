#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge

from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, TwistStamped
from object_detection.msg import StringArray, ObjectsArray
from sensor_msgs.msg import Image

from mavros_msgs.srv import SetMode

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import math

pError = [0, 0 , 0]
# pid = [[0.003, 0, 0.01],[0.01, 0, 0], [0.1, 0, 0]] 
pid = [[0.003, 0, 0.01],[0.01, 0, 0], [0.0001, 0, 0.001]] 
# pid = [[0.0, 0, 0.0],[0.0, 0, 0], [0.1, 0, 0]] 
# 0.002 , 0.01
mode = "default" 
objects = []
cv_image_depth = []
attitude = [0, 0, 0]
HZ = 30

def object_callback(data):
    global objects
    objects = data.objects

def depth_image_callback(data):
    global cv_image_depth
    cv_image_depth = bridge.imgmsg_to_cv2(data)

def state_callback(state):
    global mode
    mode = state.mode

def pose_callback(pose):
    global attitude
    quar = pose.pose.orientation
    quar_list = [quar.x, quar.y, quar.z, quar.w]
    (attitude[0], attitude[1], attitude[2]) = euler_from_quaternion(quar_list)
    # print(attitude)

def offboard_main():
    global cv_image_depth
    global objects
    global mode
    global pid
    global pError
    global attitude
    
    while True:
        if mode == "OFFBOARD":
            pError = tracking(cv_image_depth, objects, attitude,  pError, pid, target = "bus", flag = "vision")
            rate.sleep()
       

def tracking(image, objects, attitude, pErr, pid, target, flag = "default"):
    velocity = TwistStamped() # local frame NED/ENU 
    if objects != []:
        for obj in objects:
            if obj.name == target:
                object_x = int((obj.UL_x+obj.BR_x)/2)
                object_y = int((obj.UL_y+obj.BR_y)/2)
                object_z = image[object_y][object_x]
                area = (obj.BR_x-obj.UL_x)*(obj.BR_y-obj.UL_y)
                
                image_x = image.shape[1]
                image_y = image.shape[0]
                
                error_x = -(object_x - image_x/2)
                error_y = -(object_y - image_y/2)
                if flag == "vision":
                    error_z = -(area - 5000)
                else:
                    error_z = (object_z - 5)
                yawAngle = attitude[2]
                angular_z = error_x*pid[0][0]+(error_x-pErr[0])*pid[0][2]
                velocity_z = error_y*pid[1][0]+(error_y-pErr[1])*pid[1][2]
                velocity_x = error_z*pid[2][0]+(error_z-pErr[2])*pid[2][2]
                velocity_body_x = velocity_x* math.cos(yawAngle) 
                velocity_body_y = velocity_x* math.sin(yawAngle) 

                print("angular_z : ", angular_z)
                print("velocity_x : ", velocity_x)
                print("velocity_z : ", velocity_z)
                print("object_z : ", object_z)
                # angular
                velocity.twist.angular.x = 0
                velocity.twist.angular.y = 0
                velocity.twist.angular.z = angular_z
                # linear
                velocity.twist.linear.x = velocity_body_x
                velocity.twist.linear.y = velocity_body_y 
                velocity.twist.linear.z = velocity_z 
                
                pError = [error_x, error_y, error_z]
                local_vel_pub.publish(velocity)
            else:
                velocity.twist.linear.x = 0
                velocity.twist.linear.y = 0
                velocity.twist.linear.z = 0
                velocity.twist.angular.x = 0
                velocity.twist.angular.y = 0
                velocity.twist.angular.z = 0
                local_vel_pub.publish(velocity)
                pError = pErr
    else:
        velocity.twist.linear.x = 0
        velocity.twist.linear.y = 0
        velocity.twist.linear.z = 0
        velocity.twist.angular.x = 0
        velocity.twist.angular.y = 0
        velocity.twist.angular.z = 0
        local_vel_pub.publish(velocity)
        pError = pErr
    return pError
    
if __name__ == "__main__":
    rospy.init_node("commander")
    bridge = CvBridge()
    local_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size =1)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    rospy.Subscriber('/mavros/state', State, state_callback)
    rospy.Subscriber("/yolo/object_yolo", ObjectsArray, object_callback)
    
    rospy.Subscriber("/camera/depth/image_raw",Image,depth_image_callback)
    rate = rospy.Rate(HZ)
    offboard_main()
