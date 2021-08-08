#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time

from std_msgs.msg import *
from sensor_msgs.msg import *

from bodyhub.msg import *
from bodyhub.srv import *

__STEP_LEN_MAX = [0.1, 0.05, 10.0]  # x, y ,theta

__jointPositionPub = rospy.Publisher('MediumSize/BodyHub/MotoPosition', JointControlPoint, queue_size=1000)
__jointControlPub = rospy.Publisher('MediumSize/BodyHub/jointControl', JointState, queue_size=1000)
__gaitCommandPub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=2)
__currentId = 0
__isWalking = 0
__timeout = 15 # s

__state_init = 20
__state_preReady = 21
__state_ready = 22
__state_running = 23
__state_pause = 24
__state_stoping = 25
__state_error = 26
__state_directOperate = 27
__state_walking = 28


def __requestGaitCallback(msg):
    pass


def __walkingStateCallback(data):
    global __isWalking
    __isWalking = data.data


rospy.Subscriber('/requestGaitCommand', Bool, __requestGaitCallback)
rospy.Subscriber("/MediumSize/BodyHub/WalkingStatus", Float64, __walkingStateCallback, queue_size=2)


def __GetbodyhubControlId():
    try:
        rospy.wait_for_service('MediumSize/BodyHub/GetMasterID', __timeout)
    except:
        print('error: wait_for_service GetMasterID!')
        return None
    client = rospy.ServiceProxy('MediumSize/BodyHub/GetMasterID', SrvTLSstring)
    response = client('get')
    return response.data


def GetBodyhubStatus():
    try:
        rospy.wait_for_service('MediumSize/BodyHub/GetStatus', __timeout)
    except:
        print('error: wait_for_service GetStatus!')
        return None
    client = rospy.ServiceProxy('MediumSize/BodyHub/GetStatus', SrvString)
    response = client('get')
    return response


def __SetBodyhubStatus(id, status):
    try:
        rospy.wait_for_service('MediumSize/BodyHub/StateJump', __timeout)
    except:
        print('error: wait_for_service StateJump!')
        return None
    client = rospy.ServiceProxy('MediumSize/BodyHub/StateJump', SrvState)
    response = client(id, status)
    return response.stateRes


def __CheckBodyhubId(ctlId):
    global __currentId
    __currentId = __GetbodyhubControlId()
    if __currentId == None:
        return -1
    if __currentId == 0 or __currentId == ctlId:
        return True
    else:
        rospy.logwarn('bodyhub busy, id is %d' % (__currentId))
        return False


def ResetBodyhub():
    global __currentId
    status = GetBodyhubStatus()
    if status == None:
        return -1
    if status.data == 'preReady':
        return True
    __currentId = __GetbodyhubControlId()
    result = __SetBodyhubStatus(__currentId, 'stop')
    if result == None:
        return -2
    result = __SetBodyhubStatus(__currentId, 'reset')
    if result == None:
        return -3
    if result == __state_preReady:
        return True
    return False


def SetBodyhubTo_setStatus(controlId):
    if __CheckBodyhubId(controlId) != True:
        return False
    status = GetBodyhubStatus()
    if status == None:
        return -1
    if status.data == 'ready':
        return True
    if ResetBodyhub() == True:
        result = __SetBodyhubStatus(controlId, 'setStatus')
        if result == None:
            return -2
        if result == __state_ready:
            return True
    return False


def SetBodyhubTo_walking(controlId):
    if __CheckBodyhubId(controlId) != True:
        return False
    status = GetBodyhubStatus()
    if status == None:
        return -1
    if status.data == 'walking':
        return True
    if SetBodyhubTo_setStatus(controlId) == True:
        result = __SetBodyhubStatus(controlId, 'walking')
        if result == None:
            return -2
        if result == __state_walking:
            return True
    return False


def GetJointAngle(ids):
    try:
        rospy.wait_for_service('MediumSize/BodyHub/GetJointAngle', __timeout)
    except:
        print('error: wait_for_service GetJointAngle!')
        return None
    client = rospy.ServiceProxy('MediumSize/BodyHub/GetJointAngle', SrvServoAllRead)
    response = client(ids, 22)
    return response.getData


def SendJointPsition(controlId, posList):
    __jointPositionPub.publish(positions=posList, mainControlID=controlId)


def WaitForActionExecute(completely=False):
    while not rospy.is_shutdown():
        if completely:
            if GetBodyhubStatus().data == 'pause':
                break
        else:
            if GetBodyhubStatus().poseQueueSize <= 50:
                break


def SendJointCommand(controlId, idList, posList):
    strList = [str(i) for i in idList]
    strList.append(str(controlId))
    __jointControlPub.publish(name=strList, position=posList)


def WaitTrajectoryExecute(completely=False):
    while not rospy.is_shutdown():
        if completely:
            if GetBodyhubStatus().data == 'pause':
                break
        else:
            if GetBodyhubStatus().jointQueueSize <= 50:
                break


def SendGaitCommand(deltax, deltay, theta):
    global __isWalking
    rospy.wait_for_message('/requestGaitCommand', Bool, __timeout)
    __isWalking = 1
    __gaitCommandPub.publish(data=[deltax, deltay, theta])


def WaitForWalkingDone():
    global __isWalking
    while not rospy.is_shutdown():
        if __isWalking == 0:
            break


def WalkTheDistance(xLength, yLength, aLength):
    global __STEP_LEN_MAX
    direction = [0, 0, 0]
    direction[0] = 1 if xLength >= 0 else -1
    direction[1] = 1 if yLength >= 0 else -1
    direction[2] = 1 if aLength >= 0 else -1

    xNumberOfStep = abs(xLength)//__STEP_LEN_MAX[0]
    yNumberOfStep = abs(yLength)//__STEP_LEN_MAX[1]
    aNumberOfStep = abs(aLength)//__STEP_LEN_MAX[2]
    xLength = xLength - ((__STEP_LEN_MAX[0] * xNumberOfStep)*direction[0])
    yLength = yLength - ((__STEP_LEN_MAX[1] * yNumberOfStep)*direction[1])
    aLength = aLength - ((__STEP_LEN_MAX[2] * aNumberOfStep)*direction[2])
    numberOfStep = max(xNumberOfStep, yNumberOfStep, aNumberOfStep)

    for i in range(0, int(numberOfStep+1)):
        if xNumberOfStep >= 1:
            x = __STEP_LEN_MAX[0]*direction[0]
            xNumberOfStep = xNumberOfStep - 1
        elif xNumberOfStep == 0:
            x = xLength
            xNumberOfStep = xNumberOfStep - 1
        else:
            x = 0.0

        if yNumberOfStep >= 1:
            y = __STEP_LEN_MAX[1]*direction[1]
            yNumberOfStep = yNumberOfStep - 1
        elif yNumberOfStep == 0:
            y = yLength
            yNumberOfStep = yNumberOfStep - 1
        else:
            y = 0.0

        if aNumberOfStep >= 1:
            a = __STEP_LEN_MAX[2]*direction[2]
            aNumberOfStep = aNumberOfStep - 1
        elif aNumberOfStep == 0:
            a = aLength
            aNumberOfStep = aNumberOfStep - 1
        else:
            a = 0.0
        SendGaitCommand(x, y, a)
    WaitForWalkingDone()


def WalkingNSteps(cmd, n):
    for i in range(0, n):
        SendGaitCommand(cmd[0], cmd[1], cmd[2])
