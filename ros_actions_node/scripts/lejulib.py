#!/usr/bin/env python
# coding=utf-8

import os
import signal
import rospy
import rosservice

from std_msgs.msg import String
from ros_actions_node.msg import DeviceList
from ros_msg_node.srv import *

from lejufunc import client_action
from lejufunc import client_face
from lejufunc import client_color
from lejufunc import client_audio
from lejufunc import client_video
from lejufunc import client_sensor
from lejufunc import client_controller
from lejufunc import client_wakeup
from lejufunc import client_speech
from lejufunc import client_gripper
from lejufunc import client_button
from lejufunc import client_walk
from lejufunc import client_label
from lejufunc.client_logger import *


finish_pub = rospy.Publisher("/Finish",String,queue_size = 2)
device_pub = rospy.Publisher("/ActRunner/DeviceList", DeviceList, queue_size=2)


# terminate current process
def terminate(data):
	rospy.loginfo(data.data)
	rospy.signal_shutdown("kill")


# check device list in demo
def report_device_status(camera_status, controller_status):
	device_pub.publish(camera_status, controller_status)


def node_initial():
	# initial listener node
	rospy.init_node("act_runner", anonymous=True)
	rospy.sleep(0.2)
	rospy.on_shutdown(finishsend)
	rospy.Subscriber('terminate_current_process', String, terminate)
	rospy.loginfo("action node running")
