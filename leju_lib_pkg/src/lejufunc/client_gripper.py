#!/usr/bin/env python
#coding=utf-8
import rospy
from client_action import *
from client_logger import serror
from utils import judge_range_number
import sys


LEFT_KEY = (19, 1)
RIGHT_KEY = (18, -1)
KEYS = {"left": (LEFT_KEY,), "right": (RIGHT_KEY,),
        "both": (LEFT_KEY, RIGHT_KEY)}


def appropriate_angle(direction, angle):
    result = []
    for index, a in enumerate(angle):
        key, ratio = KEYS[direction][index]
        a = a * ratio
        a = judge_range_number(a, 0, 35)
        result.append((key, a * ratio))
    return result


def gripper_control(angles):
    '''
    Set the angel of gripper opening

    :param angles: [arg1,arg2] , if only one gripper;
                     arg1: 'left' or 'right'
                     arg2: gripper angles

           angles: [arg1,arg2,arg3] , if two grippers.
                     arg1: 'both'
                     arg2: left_gripper angles
                     arg3: right_gripper angles
    :return:
    '''

    if angles[0] not in KEYS.keys():
        rospy.logerr("Unknown gripper order")
        return

    all_angles = get_servo_value()
    new_angles = [servo for servo in all_angles]
    try:
        for key, angle in appropriate_angle(angles[0], angles[1:]):
            new_angles[key] = angle
    except Exception as err:
        lineno = sys._getframe().f_back.f_lineno
        co_name = sys._getframe().f_code.co_name
        serror("爪子的{}, line {}, in {}".format(err.message, lineno, co_name))

    set_all_servo(new_angles, DEFAULT_TIME)


if __name__ == '__main__':
    rospy.init_node('gripper_test', anonymous=True)
    rospy.sleep(2)
    try:

        angles_l = ['left', 15]
        gripper_control(angles_l)
        rospy.sleep(2)

        angles_r = ['right', -20]
        gripper_control(angles_r)
        rospy.sleep(2)

        angles_b = ['both', 30, -35]
        gripper_control(angles_b)
    except Exception as err:
        print(err)
