#!/usr/bin/env python

import rospy

from ros_color_node.srv import ColorRecognition


def color_service(camera, color_type, color_param):
	"""

	:param camera: 'head' or 'chin' ; select the relevant camera
	:param color_type: default RGB
	:param color_param: value of color
	:return:
	"""
	ser_name = ''
	if camera == 'head':
		ser_name = 'color_recognition'
	elif camera == 'chin':
		ser_name = 'chinCamera_color_recognition'
	else:
		rospy.logerr("Unknown camera")
	rospy.wait_for_service(ser_name, timeout=2)
	color_recognition = rospy.ServiceProxy(ser_name, ColorRecognition)
	response = color_recognition(color_type, color_param)

	return response


def color_recognition(camera, color_param, color_type="RGB"):
	"""
	Search color in picture

	Args:
	  camera: 'head' or 'chin'
	  color_param: value of color
	  color_type: default RGB

	Returns:
	  True / False
	"""

	response = color_service(camera, color_type, color_param)
	if response.region == "None":
		return False
	else:
		return True


def color_size(camera, color_param, range_low=None, range_high=None, color_type="RGB"):
	"""
	Get color percentage in picture

	Args:
	  camera: 'head' or 'chin'
	  color_param: value of color
	  color_type: default RGB
	  range_low: low range for color
	  range_high: high range for color

	Returns:
	  if no range passed, return percentage of color
	  if range passed, return True / False for range check
	"""

	response = color_service(camera, color_type, color_param)
	if range_low is None or range_high is None:
		if response.region == "None":
			return 0.0
		else:
			return response.area
	else:
		size = response.area
		if response.region != "None" and size > range_low and size <= range_high:
			return True
		else:
			return False


def color_position(camera, color_param, axis, color_type="RGB"):
	"""
	Get color position in picture

	Args:
	  camera: 'head' or 'chin'
	  color_param: value of color
	  axis: x or y

	Returns:
	  x / y of color, 0.0 if no color found
	"""

	response = color_service(camera, color_type, color_param)
	if response.region == "None":
		return 0.0
	else:
		pos = response.center
		if axis == "x":
			return pos[0]
		else:
			return pos[1]


def color_region(camera, color_param, region, color_type="RGB"):
	"""
	Color region check

	Args:
	  camera: 'head' or 'chin'
	  color_param: value of color
	  region: left, center or right

	Returns:
	  True / False for color region check
	"""

	response = color_service(camera, color_type, color_param)
	# print(response.region)
	if response.region == "None" or response.region != region:
		return False
	else:
		return True


def color_identification(camera, mark_color_param, target_color_param):
	"""
	Mark target color with mark color in picture

	Args:
	  camera: 'head' or 'chin'
	  mark_color_param: color used to mark
	  target_color_param: target color

	No return, only used for PC software check
	"""
	pass


if __name__ == '__main__':
	rospy.init_node('color_test', anonymous=True)
	color_param = [140, 30, 50]

	try:
		if color_recognition('head',color_param):
			print("Find red area")

			size = color_size('head',color_param)
			print("Red is %f percent of picture" % size)

			range_check_1 = color_size('head',color_param, 0.1, 0.25)
			print(range_check_1)
			range_check_2 = color_size('head',color_param, 0.4, 0.6)
			print(range_check_2)

			x = color_position('head',color_param, "x")
			y = color_position('head',color_param, "y")
			print("Red postion is (%f, %f)" % (x, y))

			if color_region('head',color_param, "center"):
				print("Red in center")
			else:
				print("Red not in center")
		else:
			print("No red area")
	except Exception as err:
		rospy.logerr(err)

