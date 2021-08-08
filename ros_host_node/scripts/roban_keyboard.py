#!/usr/bin/python
import rospy
import time
from lejufunc.client_walk import *
from std_msgs.msg import String
from motion.motionControl import *
controlId = 2
GAIT_RANGE = 0.05
Walker = WalkTransfer()


moveBindings = {
    'w': [1, 0, 0],
    'a': [0, 0.8, 0],
    's': [-1, 0, 0],
    'd': [0, -0.8, 0],
    'W': [1, 0, 0],
    'A': [0, 0.8, 0],
    'S': [-1, 0, 0],
    'D': [0, -0.8, 0],
}



def begin_walking():
    if SetBodyhubTo_setStatus(controlId):
        Walker.walkingClient("walking")
        return True
    else:
        rospy.loginfo("bodyhub busy,please check the masterID.")
        return False

def stop_walking():
    Walker.waitForWalkingDone()
    Walker.walkingClient("stop")


if __name__ == "__main__":
    rospy.init_node('teleop_keyboard')
    time.sleep(0.2)
    walk_state = False
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        data = rospy.wait_for_message('teleop_keyboard/keyboard_control', String).data
        if data == "r" or data == "R":
            walk_state = begin_walking()
        if data in moveBindings.keys():
            if walk_state:
                array = [i * GAIT_RANGE for i in moveBindings[data]]
                Walker.walkingPublish(array, 2)
        if data == "q" or data == "Q":
            stop_walking()
            walk_state = False
        rate.sleep()


