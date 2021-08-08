#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String
from motion.motionControl import WalkTheDistance, SetBodyhubTo_walking, ResetBodyhub, SetBodyhubTo_setStatus
from lejufunc import client_action

def forward():
    start(True)
    WalkTheDistance(0.20, 0, 0)
    stop()

def back():
    start(True)
    WalkTheDistance(-0.10, 0, 0)
    stop()

def put_hand():
    start()
    put_hand_points = [
            ([0,0,0,0,0,0,0,0,0,0,0,0,0,-61,-18,0,61,18,0,0,0,0],1000,0),
            ([-4,0,-3,-2,-2,0,-4,0,3,2,2,0,-150,-53,-35,0,43,12,0,0,-3,-9],1000,1000),
            ([0,0,0,0,0,0,0,0,0,0,0,0,0,-61,-18,0,61,18,0,0,0,0],1500,0)
    ]
    client_action.action(put_hand_points)
    stop()

def keep_down():
    start()
    keep_down_points = [
        ([0,0,0,0,0,0,0,0,0,0,0,0,0,-61,-18,0,61,18,0,0,0,0],1000,100),
        ([0,0,58,-98,-40,0,0,0,-58,98,40,0,0,-49,-24,0,49,24,0,0,0,0],800,600),
        ([0,0,0,0,0,0,0,0,0,0,0,0,0,-61,-18,0,61,18,0,0,0,0],1000,100)
    ]
    client_action.action(keep_down_points)
    stop()

def shake_head():
    start()
    shake_head_points = [
        ([0,0,0,0,0,0,0,0,0,0,0,0,0,-61,-18,0,61,18,0,0,0,0],1000,0),
        ([0,0,0,0,0,0,0,0,0,0,0,0,7,-49,-24,7,49,24,0,0,-24,0],400,200),
        ([0,0,0,0,0,0,0,0,0,0,0,0,-7,-49,-24,-7,49,24,0,0,24,0],600,300),
        ([0,0,0,0,0,0,0,0,0,0,0,0,7,-49,-24,7,49,24,0,0,-24,0],600,300),
        ([0,0,0,0,0,0,0,0,0,0,0,0,0,-61,-18,0,61,18,0,0,0,0],500,0)
    ]
    client_action.action(shake_head_points)
    stop()

def start(is_walking=False):
    id = 2
    if is_walking:
        SetBodyhubTo_walking(id)
    else:
        SetBodyhubTo_setStatus(id)

def stop():
    ResetBodyhub()

def voice_control(text):
    data = text.data
    if "前进" in data:
        forward()
    elif "后退" in data:
        back()
    elif "下蹲" in data:
        keep_down()
    elif "摇头" in data:
        shake_head()
    elif "举手" in data:
        put_hand()

if __name__ == "__main__":
    rospy.init_node("voice_control", anonymous=True)
    rospy.Subscriber("/voice_control_text", String, voice_control)
    rospy.spin()
