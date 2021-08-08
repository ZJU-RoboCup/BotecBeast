#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import math
import time
import tty
import termios
import numpy as np
import scipy.linalg as linalg
import ConfigParser

import rospy
import rospkg

import cv2
import numpy as np
from cv_bridge import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.bodyhub_client as bodycli
import vision.imageProcessing as imgPrcs
import algorithm.pidAlgorithm as pidAlg

CFG_FILE_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "./stair_convert_artag.conf"))
NODE_NAME = 'stair_convert'
REAL_POS_TOPIC = "/chin/visualization_marker_chin"
CONTROL_ID = 2
GOAL_POS = [0.130, 0.220, 0.0]


class Action(object):
    '''
    robot action
    '''

    def __init__(self, name, ctl_id):
        rospy.init_node(name, anonymous=True)
        time.sleep(0.2)
        rospy.on_shutdown(self.__ros_shutdown_hook)

        self.bodyhub = bodycli.BodyhubClient(ctl_id)

    def __ros_shutdown_hook(self):
        if self.bodyhub.reset() == True:
            rospy.loginfo('bodyhub reset, exit')
        else:
            rospy.loginfo('exit')

    def bodyhub_ready(self):
        if self.bodyhub.ready() == False:
            rospy.logerr('bodyhub to ready failed!')
            rospy.signal_shutdown('error')
            time.sleep(1)
            exit(1)

    def bodyhub_walk(self):
        if self.bodyhub.walk() == False:
            rospy.logerr('bodyhub to walk failed!')
            rospy.signal_shutdown('error')
            time.sleep(1)
            exit(1)

    def start(self):
        print('action start')


class StairConvert(Action):
    def __init__(self):
        super(StairConvert, self).__init__(NODE_NAME, CONTROL_ID)

        rospy.Subscriber(REAL_POS_TOPIC, Marker, self.marker_callback)

        self.goal_pos = GOAL_POS
        self.pid_gain = [0.7, 0.8, 0.9]
        self.err_threshold = [0.01, 0.02, 2.0]
        self.cf = ConfigParser.ConfigParser()
        self.load_cfg()

        self.real_pos = [0.0, 0.0, 0.0]
        self.pos_valid = False
        self.pid_x = pidAlg.PositionPID(p=self.pid_gain[0])
        self.pid_y = pidAlg.PositionPID(p=self.pid_gain[1])
        self.pid_a = pidAlg.PositionPID(p=self.pid_gain[2])

    def rotate_mat(self, axis, radian):
        rot_matrix = linalg.expm(np.cross(np.eye(3), axis / linalg.norm(axis) * radian))
        return rot_matrix

    def quart_to_rpy(self, w, x, y, z):
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
        return [r, p, y]

    def marker_callback(self, msg):
        rpy = self.quart_to_rpy(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        cam_rot = self.rotate_mat([0, 1, 0], 10 * math.pi / 180.0) # 绕y轴旋转  
        pos_in_torso = np.dot(cam_rot, np.array([[-msg.pose.position.y], [-msg.pose.position.x], [msg.pose.position.z]])).tolist()
        self.real_pos[0] = pos_in_torso[0][0]
        self.real_pos[1] = pos_in_torso[1][0]
        self.real_pos[2] = -rpy[2] * 180.0 / math.pi
        self.pos_valid = True
        # print("real pose:",  self.real_pos)

    def goto_rot(self, goal_ang):
        while not rospy.is_shutdown():
            self.pos_valid = False
            time.sleep(0.5)
            if self.pos_valid:
                aError = self.real_pos[2] - goal_ang
                if (abs(aError) < self.err_threshold[2]):
                    break
                aLength = self.pid_a.run(aError)
                self.bodyhub.walking_the_distance(0.0, 0.0, aLength)
                self.bodyhub.wait_walking_done()
            else:
                rospy.logwarn('ar tag no found!')
                self.bodyhub.walking_n_steps([0.0, 0.0, 5.0], 2)
                self.bodyhub.wait_walking_done()

    def goto_pose(self, goal_pos):
        while not rospy.is_shutdown():
            self.pos_valid = False
            time.sleep(0.5)
            if self.pos_valid:
                xError = self.real_pos[0] - goal_pos[0]
                yError = self.real_pos[1] - goal_pos[1]
                aError = self.real_pos[2] - goal_pos[2]
                if (xError < self.err_threshold[0]) and (abs(yError) < self.err_threshold[1]) and (abs(aError) < self.err_threshold[2]):
                    print('到达目标位置，当前位置:', self.real_pos)
                    break
                xLength = self.pid_x.run(xError)
                yLength = self.pid_y.run(yError)
                aLength = self.pid_a.run(aError)
                self.bodyhub.walking_the_distance(xLength, yLength, aLength)
                self.bodyhub.wait_walking_done()
            else:
                rospy.logwarn('ar tag no found!')
                self.bodyhub.walking_n_steps([0.02, 0.0, 0.0], 2)
                self.bodyhub.wait_walking_done()

    def load_cfg(self):
        self.cf.read(CFG_FILE_PATH)
        setion = self.cf.items('target_pos')
        self.goal_pos = [float(setion[0][1]), float(setion[1][1]), float(setion[2][1])]
        setion = self.cf.items('pid_gain')
        self.pid_gain = [float(setion[0][1]), float(setion[1][1]), float(setion[2][1])]
        setion = self.cf.items('err_threshold')
        self.err_threshold = [float(setion[0][1]), float(setion[1][1]), float(setion[2][1])]

    def set_target_pos(self):
        self.bodyhub_walk()
        print('将机器人放到目标位置, 按下回车键校准')
        while not rospy.is_shutdown():
            if raw_input('input:') == 'q':
                break
            if self.pos_valid:
                self.cf.set('target_pos', 'x', str(self.real_pos[0]))
                self.cf.set('target_pos', 'y', str(self.real_pos[1]))
                self.cf.set('target_pos', 'yaw', str(self.real_pos[1]))
                self.cf.write(open(CFG_FILE_PATH, 'w'))
                self.load_cfg()
                
                setion = self.cf.items('reference_pos')
                ref_pos = [float(setion[0][1]), float(setion[1][1]), float(setion[2][1])]
                print('参考位置: ', ref_pos)
                print('校准后位置: ', self.goal_pos)
                print('校准完成，程序退出')
                break
            else:
                print('没有识别到ar_tag码，请移动机器人或检查摄像头！')
                print('按下回车键继续，按q键退出')
        rospy.signal_shutdown('exit')

    def start(self):
        self.bodyhub_walk()
        while not rospy.is_shutdown():
            self.bodyhub.walking_n_steps([0.04, 0.0, 10.0], 9)
            self.bodyhub.walking_n_steps([0.08, 0.0, 0.0], 2)
            self.bodyhub.walking_n_steps([0.0, 0.0, 10.0], 9)
            self.bodyhub.wait_walking_done()
            self.goto_rot(0.0)
            self.goto_pose(self.goal_pos)
            rospy.signal_shutdown('exit')
            pass


if __name__ == '__main__':
    if len(sys.argv) >= 2 and (sys.argv[1] == 'debug'):
        StairConvert().set_target_pos()
    else:
        StairConvert().start()
