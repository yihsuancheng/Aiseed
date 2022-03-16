#! /usr/bin/env python3
import rospy
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Joy
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

HZ=20

manual_cmd = ManualControl()
manual_cmd.x = 0
manual_cmd.y = 0
manual_cmd.z = 0
manual_cmd.r = 0

armed = 0
disarmed = 0
land = 0
takeoff = 0
offboard = 0
rtl = 0

def joy_remapping(msg):
    global manual_cmd, armed, disarmed
    global land, takeoff, offboard, rtl
    buttons = msg.buttons
    axes = msg.axes
    
    axes = map(lambda x:int(x*1000),axes)  
    axes = list(axes)
    
    # print(axes)
    LRleft,UDleft,LT,LRright,UDright,RT,ckLR,ckUD = axes 
    A,B,X,Y,LB,RB,back,start,power,BSL,BSR=buttons
    
    # stay in the air
    UDleft = (UDleft+1000)/2
    
    manual_cmd.x = UDright
    manual_cmd.y = -LRright
    manual_cmd.z = UDleft
    manual_cmd.r = -LRleft
    
    armed = LB
    disarmed = RB
    land = A
    takeoff = Y
    offboard = B
    rtl = X

    # rate.sleep()

def main():
    global manual_cmd, armed, disarmed
    global land, takeoff, offboard, rtl
    rospy.wait_for_service('/mavros/cmd/arming')
    rospy.wait_for_service('/mavros/set_mode')
    arm_call = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_call = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    while not rospy.is_shutdown():
        if armed == 1:
            arm_call(True)
        if disarmed == 1:
            arm_call(False)
        if land == 1:
            set_mode_call(custom_mode = 'AUTO.LAND')
        if takeoff == 1:
            set_mode_call(custom_mode = 'AUTO.TAKEOFF')
        if offboard == 1:
            set_mode_call(custom_mode = 'OFFBOARD')
        if rtl == 1:
            set_mode_call(custom_mode = 'AUTO.RTL')

        joy_pub.publish(manual_cmd)
        rate.sleep()
 
if __name__ == '__main__':
    rospy.init_node('xbox')
    rate = rospy.Rate(HZ)
    joy_pub = rospy.Publisher('/mavros/manual_control/send', ManualControl, queue_size=10)
    rospy.Subscriber("/joy", Joy, joy_remapping, queue_size = 1, buff_size = 52428800)
    main()
    # rospy.spin()
