#! /usr/bin/python

import rospy
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from ros_color_node.srv import ColorRecognition

import color_filter

bridge = CvBridge()

default_center = [0.0, 0.0]
default_area = 0.0
default_region = "None"

def handle_color_request(req):
	try:
		image_msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=2)
		cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
		center, area, region = color_filter.color_filter(cv_image, req.color_param, req.color_type)
	except Exception as err:
		# print(err)
		return default_center, default_area, default_region
	else:
		return center, area, region

def chin_color_request(req):
	try:
		image_msg = rospy.wait_for_message("/chin_camera/image", Image, timeout=2)
		cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
		center, area, region = color_filter.color_filter(cv_image, req.color_param, req.color_type)
	except Exception as err:
		# print(err)
		return default_center, default_area, default_region
	else:
		return center, area, region

def main():
	rospy.init_node('ros_color_node')

	# provide color service
	s = rospy.Service('color_recognition', ColorRecognition, handle_color_request)
	c = rospy.Service('chinCamera_color_recognition',ColorRecognition, chin_color_request)
	print('ready to process color request')

	rospy.spin()

if __name__ == '__main__':
	main()
