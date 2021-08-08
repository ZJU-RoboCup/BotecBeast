#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import array
import math
import numpy

from bodyhub.msg import JointControlPoint
from bodyhub.srv import *  # for SrvState.srv
from std_msgs.msg import UInt16
import time
from bezier import get_bezier_frames
from motion.motionControl import *

DEFAULT_TIME = 100
FRAME_TIME = 10

import yaml



class ServoJointTrajectoryC():
    """ 发送舵机运行序列数据 """

    def __init__(self):

        # 节点运行状态
        self.StateStyle = {
            'init': 20,
            'preReady': 21,
            'running': 23,
            'pause': 24,
            'stoping': 25,
            'error': 26,
        }

        self.StateEnumEnding = 97

        self.motoWaitFlag = 0  # 舵机运动结束 置0

        self.EndingOrder = 0  # 终止指令

        self.StatusData = 0  # receive the returned status data

        # Publisher Subscriber
        self.MotoJointPub = rospy.Publisher('MediumSize/BodyHub/MotoPosition', JointControlPoint,
                                            queue_size=1000)  # 定义一个话题发布者，发布数据格式为JointControlPoint
        self.HeadOnlyJointPub = rospy.Publisher('MediumSize/BodyHub/HeadPosition', JointControlPoint,
                                                queue_size=1000)  # 定义一个话题发布者，发布数据格式为JointControlPoint
        self.BodyHubStatusSub = rospy.Subscriber('MediumSize/BodyHub/Status', UInt16, self.BodyHubStatusCallback)
        self.EdningExecSrv = rospy.Service('MediumSize/ActPackageExec/EndingExec', SrvTLSuint, self.EndingExecCallback)

    def GetMotoValue(self):
        id_array = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
        id_count = 22

        rospy.wait_for_service('/MediumSize/BodyHub/GetJointAngle', 2)
        moto_client = rospy.ServiceProxy('/MediumSize/BodyHub/GetJointAngle', SrvServoAllRead)
        result = moto_client(id_array, id_count)
        moto_value = result.getData

        return moto_value

    def GetFrame(self, p0, p1, p2, act_time):
        return get_bezier_frames(p0, p1, p2, act_time, FRAME_TIME)[1:]

    def MotoJointTransfer(self, p1, time=DEFAULT_TIME, p0=None, p2=None):
        self.motoWaitFlag = 1
        if not isinstance(time, int) or time < 10 or time > 180000:
            time = DEFAULT_TIME
        if p0 == None:
            p0 = self.GetMotoValue()
        move_frames = self.GetFrame(p0, p1, p2, time)
        for frame in move_frames:
            process_frame = array.array("d", frame)
            self.MotoJointPub.publish(positions=process_frame, mainControlID=2)

        rospy.logdebug("MotoJointTransfer publish")

    def HeadJointTransfer(self, arraySend, time=DEFAULT_TIME):
        self.motoWaitFlag = 1
        if not isinstance(time, int) or time < 10 or time > 180000:
            time = DEFAULT_TIME

        cur_head_value = self.GetMotoValue()[20:]
        move_frames = self.GetFrame(cur_head_value, arraySend, None, time)
        for frame in move_frames:
            process_frame = array.array("d", frame)
            self.HeadOnlyJointPub.publish(positions=process_frame, mainControlID=2)

        rospy.logdebug("HeadJointTransfer publish")

    def MotoJointFrames(self, frames):
        for frame in frames:
            SendJointPsition(2, frame)

    def MotoWait(self, completely=True):
        WaitForActionExecute(completely)

    def BodyHubStatusCallback(self, receiveData):
        # MotoWait Ending Signal

        self.StatusData = receiveData.data
        rospy.logdebug("MotoJointTransfer receive the state data:%d", self.StatusData)

    def EndingExecCallback(self, req):
        rospy.logdebug("Ending ActMoveExample node")

        if req.uintReq == self.StateEnumEnding:
            self.EndingOrder = self.StateEnumEnding

        return SrvStateResponse(79)


# // EXAMPLE
if __name__ == '__main__':

    # example send array
    arraySendA = array.array("d",
                             [-45, -45, -45, -45, -45, -45, -45, -45, -45, -45, -45, -45, -45, -45, -45, -45, -45, -45,
                              -45, -45])  # double angle
    arraySendB = array.array("d", [45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45,
                                   45])  # double angle
    arraySendC = array.array("d",
                             [-45, -45, -45, -45, -45, -45, -45, -45, -45, -45, -45, -45, -45, -45, -45, -45, -45, -45,
                              -45])  # double angle
    arraySendD = array.array("d", [45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45,
                                   45])  # double angle

    headarraySendA = array.array("d", [90.0, 90.0])
    headarraySendB = array.array("d", [0.0, 0.0])

    servoDataSend = ServoJointTrajectoryC()

    try:
        # 初始化ros节点
        rospy.init_node('ServoJointTrajectory', anonymous=True, log_level=rospy.INFO)  # DEBUG INFO ERROR WARN
        rospy.loginfo("Starting ServoJointTrajectory node")

        rospy.sleep(2.0)  # 必要延迟，给予初始化时间

        while not rospy.is_shutdown():
            servoDataSend.MotoJointTransfer(arraySendC)
            servoDataSend.MotoWait()
            rospy.sleep(3)
            servoDataSend.MotoJointTransfer(arraySendD)
            servoDataSend.MotoWait()
            rospy.sleep(3)

            servoDataSend.HeadJointTransfer(headarraySendA)
            servoDataSend.MotoWait()
            rospy.sleep(3)
            servoDataSend.HeadJointTransfer(headarraySendB)
            servoDataSend.MotoWait()
            rospy.sleep(3)

    except KeyboardInterrupt:
        rospy.logwarn("Shutting down ServoJointTrajectory node.")
