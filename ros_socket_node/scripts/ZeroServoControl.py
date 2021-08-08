#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy
from bodyhub.srv import *

DEFAULT_TIME = 1500
FRAME_TIME = 10
def GetCurrentValue():
    id_array = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
    id_count = 22
    rospy.wait_for_service('/MediumSize/BodyHub/DirectMethod/GetServoPositionValAll', 2)
    moto_client = rospy.ServiceProxy('/MediumSize/BodyHub/DirectMethod/GetServoPositionValAll', SrvServoAllRead)
    result = moto_client(id_array, id_count)
    current_value = result.getData
    return current_value

def GetFrameWithSpeed(start, end, time):
    frame_number = time // FRAME_TIME + 1
    move_sequence = []
    for i in range(len(start)):
        move_sequence.append(numpy.linspace(start[i], end[i], num=frame_number, endpoint=True))

    target_frames = []
    for i in range(frame_number):
        cur_frame = []
        for j in range(len(start)):
            cur_frame.append(move_sequence[j][i])
        target_frames.append(cur_frame)

    return target_frames

def ZeroServoTransfer(arraySend, time=DEFAULT_TIME):
    rospy.loginfo("ZeroServoTransfer begin...")

    if not isinstance(time, int) or time < 20 or time > 180000:
        time = DEFAULT_TIME
    ids = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22]
    cur_moto_value = GetCurrentValue()
    move_frames = GetFrameWithSpeed(cur_moto_value, arraySend, 10*time)
    rospy.wait_for_service("/MediumSize/BodyHub/DirectMethod/SetServoTarPositionValAll",2)
    servomoto = rospy.ServiceProxy("/MediumSize/BodyHub/DirectMethod/SetServoTarPositionValAll",SrvServoAllWrite)
    for frame in move_frames:
        servomoto(ids,len(frame),frame)




