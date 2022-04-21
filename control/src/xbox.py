#! /usr/bin/env python3
import rospy
from mavros_msgs.msg import ManualControl
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from mavros_msgs.srv import CommandBool, CommandVtolTransition
from mavros_msgs.srv import SetMode

HZ=20

manual_cmd = ManualControl()
manual_cmd.x = 0
manual_cmd.y = 0
manual_cmd.z = 0
manual_cmd.r = 0

armed = 0
disarmed = 0

manual = 0
position = 0
land = 0
takeoff = 0
hold = 0
mission = 0
offboard = 0
rtl = 0

transition = 0
def joy_remapping(msg):
    global manual_cmd, armed, disarmed
    global manual, position, land, takeoff, hold, mission, offboard, rtl
    global transition
    buttons = msg.buttons
    axes = msg.axes
    
    axes = map(lambda x:int(x*1000),axes)  
    axes = list(axes)
    
    LRleft,UDleft,LT,LRright,UDright,RT,ckLR,ckUD = axes 
    A,B,X,Y,LB,RB,back,start,power,BSL,BSR=buttons
    # print("buttons : ", buttons)
    # print("axes : ",axes)
    # stay in the air
    UDleft = (UDleft+1000)/2
    
    manual_cmd.x = UDright
    manual_cmd.y = -LRright
    manual_cmd.z = UDleft
    manual_cmd.r = -LRleft
    
    armed = LT
    disarmed = RT

    manual = LB
    position = RB
    land = ckUD
    takeoff = ckUD
    hold = Y
    mission = A
    offboard = B
    rtl = X

    transition += power
    # rate.sleep()

def main():
    global manual_cmd, armed, disarmed
    global manual, position, land, takeoff, hold, mission, offboard, rtl
    global transition
    rospy.wait_for_service('/mavros/cmd/arming')
    rospy.wait_for_service('/mavros/cmd/vtol_transition')
    arm_call = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    transition_call = rospy.ServiceProxy('/mavros/cmd/vtol_transition', CommandVtolTransition)
    ### still cannot figure out how to use service message ### 
    while not rospy.is_shutdown():
        if armed == -1000:
            arm_call(True)
        if disarmed == -1000:
            arm_call(False)
        if manual == 1:
            mode_pub.publish('MANUAL')
        if position == 1:
            mode_pub.publish('POSCTL')
        if land == -1000:
            mode_pub.publish('AUTO.LAND')
        if takeoff == 1000:
            mode_pub.publish('AUTO.TAKEOFF')
        if hold == 1:
            mode_pub.publish('AUTO.LOITER')
        if mission == 1:
            mode_pub.publish('AUTO.MISSION')
        if offboard == 1:
            mode_pub.publish('OFFBOARD')
        if rtl == 1:
            mode_pub.publish('AUTO.RTL')
        if transition%2 == 1:
            transition_call(state = 4)
        else:
            transition_call(state = 3)
        
        joy_pub.publish(manual_cmd)
        rate.sleep()
 
if __name__ == '__main__':
    rospy.init_node('xbox')
    rate = rospy.Rate(HZ)
    joy_pub = rospy.Publisher('/mavros/manual_control/send', ManualControl, queue_size=10)
    mode_pub = rospy.Publisher('/mode', String , queue_size=1)
    rospy.Subscriber("/joy", Joy, joy_remapping, queue_size = 1, buff_size = 52428800)
    main()
