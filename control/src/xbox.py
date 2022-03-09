#! /usr/bin/env python3
import rospy
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Joy

HZ=20

manual_cmd = ManualControl()
manual_cmd.x = 0
manual_cmd.y = 0
manual_cmd.z = 0
manual_cmd.r = 0

def joy_remapping(msg):
    global manual_cmd
    buttons = msg.buttons
    axes = msg.axes
    
    axes = map(lambda x:int(x*1000),axes)  
    axes = list(axes)
    
    print(axes)
    LRleft,UDleft,LT,LRright,UDright,RT,ckLR,ckUD = axes 
    A,B,X,Y,LB,RB,back,start,power,BSL,BSR=buttons
    
    UDleft = (UDleft+1000)/2
    print(LRleft)
    manual_cmd.x = UDright
    manual_cmd.y = -LRright
    manual_cmd.z = UDleft
    manual_cmd.r = -LRleft

    # rate.sleep()

def main():
    global manual_cmd
    while not rospy.is_shutdown():
        joy_pub.publish(manual_cmd)
        rate.sleep()
 
if __name__ == '__main__':
    rospy.init_node('xbox')
    rate = rospy.Rate(HZ)
    joy_pub = rospy.Publisher('/mavros/manual_control/send', ManualControl, queue_size=10)
    rospy.Subscriber("/joy", Joy, joy_remapping, queue_size = 1, buff_size = 52428800)
    main()
    # rospy.spin()
