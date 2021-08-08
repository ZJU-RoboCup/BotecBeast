#!/usr/bin/env python
# coding=utf-8

import rospy
import cv2
import time
import os
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

bridge = CvBridge()
video_path = os.environ['HOME'] + '/Videos/'
photo_path = os.environ['HOME'] + '/Pictures/'

def produce_video(duration, filename):
	# fourcc = cv2.VideoWriter_fourcc(*"H264")
	# Can't find a valid fourcc code for mp4, solution link: 
	# https://devtalk.nvidia.com/default/topic/1029451/jetson-tx2/-python-what-is-the-four-characters-fourcc-code-for-mp4-encoding-on-tx2/
	file_path = '%s%s%s' % (video_path, filename, '.mp4')
	out = cv2.VideoWriter(file_path, 0x00000021, 15.0, (640,480))

	image_topic = "/camera/color/image_raw"

	start_time = time.time()
	try:
		while True:
			msg = rospy.wait_for_message(image_topic, Image, timeout=duration)
			cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
			out.write(cv_image)

			current_time = time.time()
			if current_time - start_time > duration:
				break
	except Exception as err:
		raise
	finally:
		out.release()


def take_photo(filename):
	"""
	Take photo

	Args:
	  filename: file name of photo
	"""

	file_path = '%s%s%s' % (photo_path, filename, '.jpg')

	image_topic = "/camera/color/image_raw"
	msg = rospy.wait_for_message(image_topic, Image, 2)
	cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
	cv2.imwrite(file_path, cv_image)


if __name__ == '__main__':
	rospy.init_node('ros_video_node', anonymous=True)

	try:
		# produce_video(10, "test")
		take_photo("test")
	except Exception as err:
		rospy.logerr(err)
