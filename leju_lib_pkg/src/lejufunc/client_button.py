#!/usr/bin/env python
# coding=utf-8
import rospy
from std_msgs.msg import String

def button():
    msg = rospy.wait_for_message("/VirtualPanel",String)
    return msg.data

if __name__ == '__main__':
    rospy.init_node("button_test", anonymous=True)

    try:
        result = button()
        print("result:",result)
    except Exception as err:
        print(err)