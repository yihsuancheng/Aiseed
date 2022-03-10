#!/usr/bin/env python3

import rospy 
from mavros_msgs.msg import AttitudeTarget, State, ManualControl, OverrideRCIn
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
import threading
import time

vehicleState = State()
def vehicleStateCallBack(msg):
    global vehicleState
    vehicleState = msg
    print("armed:" + str(vehicleState.armed) + "\tconnect:" + str(vehicleState.connected) + "\tmode:" + str(vehicleState.mode))

manualControl_ = ManualControl()
def ManualControlCallBack(msg):
    global manualControl_
    manualControl_ = msg
    print("x=" + str(manualControl_.x) + "\ty=" + str(manualControl_.y) + "\tz=" + str(manualControl_.z))


rospy.init_node('rospy_node', anonymous=True)
rospy.Subscriber('/mavros/state', State, vehicleStateCallBack)
manual_contoller_pub = rospy.Publisher('/mavros/manual_control/send', ManualControl, queue_size=10)
rospy.Subscriber('/mavros/manual_control/control', ManualControl, ManualControlCallBack)
arm_ser = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_ser = rospy.ServiceProxy('/mavros/set_mode', SetMode)
rc_in_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
manual_controller = ManualControl()
manual_controller.x = -300
manual_controller.y = 0
manual_controller.z = 1000
manual_controller.r = 500
rate = rospy.Rate(50)

rc_in = OverrideRCIn()
rc_in.channels = [1500, 1500, 1800, 1500, 0, 0, 0, 0]

def main_threading():
    while not rospy.is_shutdown():
        global vehicleState

        if not vehicleState.armed:
            arm_ser(True) 
    # set_mode_ser(custom_mode = 'OFFBOARD')
        manual_contoller_pub.publish(manual_controller)
    # rc_in_pub.publish(rc_in)
        rate.sleep()
    

if __name__ == "__main__":
    main_threading()

