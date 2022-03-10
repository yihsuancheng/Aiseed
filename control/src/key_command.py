#!/usr/bin/env python3
from pynput import keyboard
import rospy
from mavros_msgs.msg import ManualControl


manual_cmd = ManualControl()
manual = [0,0,0,0]

manual_cmd.x = 0 
manual_cmd.y = 0 
manual_cmd.z = 0 
manual_cmd.r = 0 

# rate = rospy.Rate(10)

def on_press(key):
    try:
        if key.char == "w":
            manual[0] = 0.0
            manual[1] = 0.0
            manual[2] = 1000.0
            manual[3] = 0.0
        elif key.char == "s":
            manual[0] = 0.0
            manual[1] = 0.0
            manual[2] = 0.0
            manual[3] = 0.0
        elif key.char == "a":
            manual[0] = 0.0
            manual[1] = 0.0
            manual[2] = 500.0
            manual[3] = -1000.0
        elif key.char == "d":
            manual[0] = 0.0
            manual[1] = 0.0
            manual[2] = 500.0
            manual[3] = 1000.0
    except AttributeError:
        if key == keyboard.Key.up:
            manual[0] = 1000.0
            manual[1] = 0.0
            manual[2] = 500.0
            manual[3] = 0.0
        elif key == keyboard.Key.down:
            manual[0] = -1000.0
            manual[1] = 0.0
            manual[2] = 500.0
            manual[3] = 0.0
        elif key == keyboard.Key.left:
            manual[0] = 0.0
            manual[1] = 1000.0
            manual[2] = 500.0
            manual[3] = 0.0
        elif key == keyboard.Key.right:
            manual[0] = 0.0
            manual[1] = 1000.0
            manual[2] = 500.0
            manual[3] = 0.0

    manual_cmd.x = manual[0] 
    manual_cmd.y = manual[1] 
    manual_cmd.z = manual[2] 
    manual_cmd.r = manual[3] 
    key_pub.publish(manual_cmd)

def pub_callback(cmd):
    global manual_cmd
    cmd.x = manual_cmd.x
    cmd.y = manual_cmd.y
    cmd.z = 500
    cmd.r = manual_cmd.r
    key_pub.publish(cmd)
    # rate.sleep()
if __name__=="__main__":
    rospy.init_node("keyboard")
    key_pub = rospy.Publisher('/mavros/manual_control/send', ManualControl, queue_size=10)
    rospy.Subscriber("/constant_value", ManualControl, pub_callback, queue_size = 10, buff_size = 52428800) 
    # Collect events until released
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()
    rospy.spin()
