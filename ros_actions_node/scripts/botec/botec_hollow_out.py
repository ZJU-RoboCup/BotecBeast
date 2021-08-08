#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import math
import time
import numpy as np
import rospy
import rospkg
import tf
import numpy.matlib
import scipy.linalg as linalg
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
import bodyhub_action as bodact
from algorithm import pidAlgorithm as pidAlg

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *

NODE_NAME = 'botech_hollow_out_node'
CHIN_MARKER_TOPIC = "/chin/visualization_marker_chin"
HEAD_MARKER_TOPIC = "/head/visualization_marker_head"
CONTROL_ID = 2


class HollowOut(bodact.Action):
    def __init__(self):
        super(HollowOut, self).__init__(NODE_NAME, CONTROL_ID)

        rospy.Subscriber(CHIN_MARKER_TOPIC, Marker, self.chin_marker_callback)
        rospy.Subscriber(HEAD_MARKER_TOPIC, Marker, self.head_marker_callback)

        self.__debug_print(= False)
        self.pid_gain = [0.8, 0.8, 0.9]
        self.err_threshold = [0.02, 0.03, 3.0]

        self.chin_marker_info = [{'valid':False, 'pos':[0, 0, 0], 'rot':[0, 0, 0]} for i in range(18)]
        self.head_marker_info = [{'valid':False, 'pos':[0, 0, 0], 'rot':[0, 0, 0]} for i in range(18)]
        self.pid_x = pidAlg.PositionPID(p=self.pid_gain[0])
        self.pid_y = pidAlg.PositionPID(p=self.pid_gain[1])
        self.pid_theta = pidAlg.PositionPID(p=self.pid_gain[2])

        self.head_pitch = 0

    def rotate_mat(self, axis, radian):
        rot_matrix = linalg.expm(np.cross(np.eye(3), axis / linalg.norm(axis) * radian))
        return rot_matrix

    def quart_to_rpy(self, w, x, y, z):
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
        return [r, p, y]

    def chin_marker_callback(self, msg):
        if msg.id > 18:
            print(msg.id, 'chin marker id > 18 ！！！')
            return
        rpy = self.quart_to_rpy(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        self.chin_marker_info[msg.id]['pos'] = [-msg.pose.position.y, -msg.pose.position.x, msg.pose.position.z]
        self.chin_marker_info[msg.id]['rot'] = [-rpy[1] * 180.0 / math.pi, rpy[0] * 180.0 / math.pi, -rpy[2] * 180.0 / math.pi]
        self.chin_marker_info[msg.id]['valid'] = True

        cam_rot = self.rotate_mat([0, 1, 0], -(self.head_pitch - 10) * math.pi / 180.0) # 绕y轴旋转  
        pos_in_torso = np.dot(cam_rot, np.array([[-msg.pose.position.y], [-msg.pose.position.x], [msg.pose.position.z]])).tolist()
        self.chin_marker_info[msg.id]['pos'] = [pos_in_torso[0][0], pos_in_torso[1][0], pos_in_torso[2][0]]

        if self.__debug_print(== True:)
            # print('pos in toros: ', pos_in_torso)
            # print("marker:",  msg.id, [self.chin_marker_info[msg.id]['pos'][0], self.chin_marker_info[msg.id]['pos'][1], self.chin_marker_info[msg.id]['rot'][2]])
            print("chin marker:",  msg.id, self.chin_marker_info[msg.id])
        
    def head_marker_callback(self, msg):
        if msg.id > 18:
            print(msg.id, 'head marker id > 18 ！！！')
            return
        rpy = self.quart_to_rpy(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        self.head_marker_info[msg.id]['pos'] = [msg.pose.position.z, -msg.pose.position.x, -msg.pose.position.y]
        self.head_marker_info[msg.id]['rot'] = [rpy[2] * 180.0 / math.pi, rpy[0] * 180.0 / math.pi, -rpy[1] * 180.0 / math.pi]
        self.head_marker_info[msg.id]['valid'] = True

        cam_rot = self.rotate_mat([0, 1, 0], -(self.head_pitch) * math.pi / 180.0) # 绕y轴旋转  
        pos_in_torso = np.dot(cam_rot, np.array([[msg.pose.position.z], [-msg.pose.position.x], [-msg.pose.position.y]])).tolist()
        self.head_marker_info[msg.id]['pos'] = [pos_in_torso[0][0], pos_in_torso[1][0], pos_in_torso[2][0]]

        if self.__debug_print(== True:)
            # print('pos in toros: ', pos_in_torso)
            # print("marker:",  msg.id, [self.head_marker_info[msg.id]['pos'][0], self.head_marker_info[msg.id]['pos'][1], self.head_marker_info[msg.id]['rot'][2]])
            print("head marker:",  msg.id, self.head_marker_info[msg.id])

    def goto_rot(self, mk_id,  goal_rot):
        while not rospy.is_shutdown():
            self.chin_marker_info[mk_id]['valid'] = False
            time.sleep(1)
            if self.chin_marker_info[mk_id]['valid']:
                err = self.chin_marker_info[mk_id]['rot'][2] - goal_rot
                if (abs(err) < self.err_threshold[2]):
                    break
                theta = self.pid_theta.run(err)
                self.bodyhub.walking_the_distance(0.0, 0.0, theta)
                self.bodyhub.wait_walking_done()
            else:
                rospy.logwarn('%d marker no found!', mk_id)
                self.bodyhub.walking_n_steps([0.0, 0.0, 10.0], 2)
                self.bodyhub.wait_walking_done()

    def goto_pose(self, mk_id, goal_pose):
        # self.goto_rot(mk_id, goal_pose[2])
        while not rospy.is_shutdown():
            self.chin_marker_info[mk_id]['valid'] = False
            time.sleep(1)
            if self.chin_marker_info[mk_id]['valid']:
                xError = self.chin_marker_info[mk_id]['pos'][0] - goal_pose[0]
                yError = self.chin_marker_info[mk_id]['pos'][1] - goal_pose[1]
                aError = self.chin_marker_info[mk_id]['rot'][2] - goal_pose[2]
                if (abs(xError) < self.err_threshold[0]) and (abs(yError) < self.err_threshold[1]) and (abs(aError) < self.err_threshold[2]):
                    break
                xLength = self.pid_x.run(xError)
                yLength = self.pid_y.run(yError)
                theta = self.pid_theta.run(aError)
                self.bodyhub.walking_the_distance(xLength, yLength, theta)
                self.bodyhub.wait_walking_done()
            else:
                rospy.logwarn('%d marker no found!', mk_id)
                self.bodyhub.walking_n_steps([-0.04, 0.0, 0.0], 2)
                self.bodyhub.wait_walking_done()

    def set_head_rot(self, head_rot):
        keyframes = [
            ([0, -1, 16, -34, -17, -1, 0, 1, -16, 34, 17, 1, 0, -70, -15, 0, 70, 15, 0, 0, head_rot[0], head_rot[1]], 500, 0)
        ]
        self.movement.linearMove(keyframes)
        self.head_pitch = head_rot[1]

    def hollow_out_act(self):
        try:
            hollow_out_frames = {"1":[[[1000,0],[0,0],[0,0]]],"2":[[[1000,-1.5],[0,0],[0,0]]],"3":[[[1000,0],[0,0],[0,0]]],"4":[[[1000,0],[0,0],[0,0]]],"5":[[[1000,0],[0,0],[0,0]]],"6":[[[1000,-1.5],[0,0],[0,0]]],"7":[[[1000,0],[0,0],[0,0]]],"8":[[[1000,1.5],[0,0],[0,0]]],"9":[[[1000,0],[0,0],[0,0]]],"10":[[[1000,0],[0,0],[0,0]]],"11":[[[1000,0],[0,0],[0,0]]],"12":[[[1000,1.5],[0,0],[0,0]]],"13":[[[1000,0],[0,0],[0,0]]],"14":[[[1000,-35],[0,0],[0,0]]],"15":[[[1000,0],[0,0],[0,0]]],"16":[[[1000,0],[0,0],[0,0]]],"17":[[[1000,-5],[0,0],[0,0]]],"18":[[[1000,85],[0,0],[0,0]]],"19":[[[1000,0],[0,0],[0,0]]],"20":[[[1000,0],[0,0],[0,0]]],"21":[[[1000,0],[0,0],[0,0]]],"22":[[[1000,0],[0,0],[0,0]]]}
            hollow_out_musics = []
            client_action.custom_action(hollow_out_musics,hollow_out_frames)

        except Exception as err:
            serror(err)
        finally:
            pass

    def debug(self):
        self.bodyhub_ready()
        self.__debug_print(= True)
        while not rospy.is_shutdown():
            time.sleep(0.01)

    def start(self):
        self.bodyhub_walk()
        self.bodyhub.walking_n_steps([0.08, 0.0, 0.0], 4)
        self.bodyhub.wait_walking_done()
        self.goto_pose(7, [0.15, -0.23, 0.0])

        self.bodyhub_ready()
        self.hollow_out_act()
        time.sleep(10)
        self.bodyhub.reset()
        
        self.bodyhub_walk()
        self.bodyhub.walking_n_steps([0.1, 0.0, 0.0], 8)
        self.bodyhub.wait_walking_done()
        self.goto_rot(8, 0.0)
        self.goto_pose(8, [0.13, -0.23, 0.0])
    
if __name__ == '__main__':
    if len(sys.argv) >= 2 and (sys.argv[1] == 'debug'):
        HollowOut().debug()
    else:
        HollowOut().start()