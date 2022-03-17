#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped

HZ = 20

local_position_current = PoseStamped() 

def mode_callback(mode):
    global local_position_current
    if mode.data == 'AUTO.LAND':
        set_mode_call(custom_mode = 'AUTO.LAND')
    elif mode.data == 'AUTO.TAKEOFF':
        set_mode_call(custom_mode = 'AUTO.TAKEOFF')
    elif mode.data == 'OFFBOARD':
        for _ in range(1):
            pose = PoseStamped() 
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = local_position_current.pose.position.z
            local_pos_pub.publish(pose)
        
        set_mode_call(custom_mode = 'OFFBOARD')
    elif mode.data == 'AUTO.RTL':
        set_mode_call(custom_mode = 'AUTO.RTL')

def local_position_callback(position):
    global local_position_current
    local_position_current = position


if __name__ == "__main__":
    rospy.init_node("mode_center")
    rate = rospy.Rate(HZ)
    rospy.wait_for_service('/mavros/set_mode')
    set_mode_call = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size =1)
    rospy.Subscriber("/mode", String, mode_callback)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_position_callback)
    rospy.spin()
