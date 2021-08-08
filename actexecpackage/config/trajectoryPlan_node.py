#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time

import rospy
import rospkg

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.motionControl as mCtrl
import motion.TrajectoryPlan as tPlan

DIRECTLY_RUN = True
NodeControlId = 2

def SendPoseTrajectory(controlId, trajectoryPoint):
    for m in range(len(trajectoryPoint[0])):
        jointPosition = []
        for n in range(len(trajectoryPoint)):
            jointPosition.append(trajectoryPoint[n][m].y)

        mCtrl.SendJointPsition(controlId, jointPosition)

def rosShutdownHook():
    mCtrl.ResetBodyhub()
    rospy.signal_shutdown('node_close')

if __name__ == '__main__':
    poseList = [
        [0,0,0,0,0,0, 0,0,0,0,0,0, 0,-75,-10, 0,75,10, 0,0, 0,0],

        [0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0, -90,75,10, 0,0, 0,0],
        [0,0,0,0,0,0, 0,0,0,0,0,0, 0,-75,-10, 0,75,10, 0,0, 0,0],

        [0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0, 90,75,10, 0,0, 0,0],
        [0,0,0,0,0,0, 0,0,0,0,0,0, 0,-75,-10, 0,75,10, 0,0, 0,0],

        [0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0, -90,75,10, 0,0, 0,0],
        [0,0,0,0,0,0, 0,0,0,0,0,0, 0,-75,-10, 0,75,10, 0,0, 0,0],
    ]

    rospy.init_node('trajectoryPlan_node', anonymous=True)
    time.sleep(0.2)
    rospy.on_shutdown(rosShutdownHook)
    rospy.loginfo('node runing...')

    tpObject = tPlan.TrajectoryPlanning(22,10.0)

    if DIRECTLY_RUN:
        if mCtrl.SetBodyhubTo_setStatus(NodeControlId) == False:
            rospy.logerr('bodyhub to setStatus fail!')
            rospy.signal_shutdown('error')
            exit(1)

    idList = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22]
    angleList = [0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0, 0,0,0, 0,0, 0,0]
    angleList = mCtrl.GetJointAngle(idList)
    poseList.insert(0, angleList)

    while not rospy.is_shutdown():
        tpObject.setInterval(1000.0)
        tpObject.planningBegin(poseList[0], poseList[1])

        for poseIndex in range(2, len(poseList)):
            tpObject.setInterval(1200.0)
            trajectoryPoint = tpObject.planning(poseList[poseIndex])
            SendPoseTrajectory(NodeControlId, trajectoryPoint)
            mCtrl.WaitForActionExecute()

        trajectoryPoint = tpObject.planningEnd()
        SendPoseTrajectory(NodeControlId, trajectoryPoint)
        mCtrl.WaitForActionExecute(True)

        if DIRECTLY_RUN:
            mCtrl.ResetBodyhub()
        rospy.signal_shutdown('over')

        