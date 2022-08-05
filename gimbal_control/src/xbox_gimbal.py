#! /usr/bin/env python3
import rospy
from mavros_msgs.msg import ManualControl
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from mavros_msgs.srv import CommandBool, CommandVtolTransition
from mavros_msgs.srv import SetMode

from pymavlink import mavutil
import os
HZ=5

os.environ["MAVLINK20"] = '1'
the_connection = mavutil.mavlink_connection('udpin:localhost:14030')
LRleft_pre = 0
UDright_pre = 0

def connecting():
    global the_connection
    print("Connecting")
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

def send_command(connection, command_num, param1, param2, param3, param4, param5, param6, param7):    
    connection.mav.command_long_send(connection.target_system, connection.target_component, command_num, 0, param1, param2, param3, param4, param5, param6, param7)

def joy_remapping(msg):
    global LRleft_pre, UDright_pre
    buttons = msg.buttons
    axes = msg.axes
    
    axes = map(lambda x:int(x*180),axes)  
    axes = list(axes)
    
    LRleft,UDleft,LT,LRright,UDright,RT,ckLR,ckUD = axes 
    A,B,X,Y,LB,RB,back,start,power,BSL,BSR=buttons
    # print("buttons : ", buttons)
    # print("axes : ",axes)
    # right is positive
    LRleft = -LRleft
    # map -90~45
    if UDright > 45:
        UDright = 45
    elif UDright < 0:
        UDright = int(UDright / 2) 
    
    pitch_rate = 0
    yaw_rate = 0
    
    vel = 1
    # if (LRleft-LRleft_pre) < 0:
    #     yaw_rate = -vel
    # elif (LRleft-LRleft_pre) > 0:
    #     yaw_rate = vel
    # 
    # if (UDright-UDright_pre) < 0:
    #     pitch_rate = -vel
    # elif (UDright-UDright_pre) > 0:
    #     pitch_rate = vel
    if LRleft < 0:
        yaw_rate = -vel
    elif LRleft > 0:
        yaw_rate = vel
    
    if UDright < 0:
        pitch_rate = -vel
    elif UDright > 0:
        pitch_rate = vel
  
    print("pitch_rate : ", pitch_rate) 
    print("yaw_rate : ", yaw_rate) 
    send_command(the_connection, 1000, UDright, LRleft, pitch_rate, yaw_rate, 4, 0, 0)
    LRleft_pre = LRleft
    UDright_pre = UDright
    rate.sleep() 

if __name__ == '__main__':
    connecting()
    send_command(the_connection, 1001, 255, 0, 0, 0, 0, 0, 0)
    rospy.init_node('xbox_gimbal')
    rate = rospy.Rate(HZ)
    rospy.Subscriber("/gimbal/joy", Joy, joy_remapping, queue_size = 1, buff_size = 52428800)
    rospy.spin()
