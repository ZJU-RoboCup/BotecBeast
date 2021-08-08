#!/usr/bin/env python

import rospy
import json
import time

from std_msgs.msg import String

def get_sound_orientation(timeout, direction):
	'''
	Wait for wakeup command

	Args:
	  timeout: time to wait in second
	  direction: left / right

	Return:
	  true or false
	'''

	try:
		msg = rospy.wait_for_message('/micarrays/wakeup', String, timeout)
		data = json.loads(msg.data.replace("'", '"'))
		angle = data['angle']
		if angle < 180:
			cur_direction = "left"
		else:
			cur_direction = "right"

		return cur_direction == direction
	except Exception as err:
		# rospy.logerr(err)
		return False


if __name__ == '__main__':
	rospy.init_node("wakeup_test", anonymous=True)

	print("Current time : %s" % time.ctime())
	result = get_sound_orientation(10, "left")
	print("Current time : %s" % time.ctime())
	print(result)