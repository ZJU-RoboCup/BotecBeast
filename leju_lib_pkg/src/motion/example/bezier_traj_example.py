#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import threading

from sensor_msgs.msg import *

import rospy
import rospkg

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.motionControl as mCtrl
import motion.bezierPlan as bPlan

BODY_CTL_ID = 2


def SendTrajectory(controlId, jointId, trajectoryPoint):
    for i in range(len(max(trajectoryPoint, key=len))):
        jointPosition = []
        for traj in trajectoryPoint:
            if i >= len(traj):
                jointPosition.append(traj[-1].y)
            else:
                jointPosition.append(traj[i].y)
        mCtrl.SendJointCommand(controlId, jointId, jointPosition)


def rosShutdownHook():
    mCtrl.ResetBodyhub()


if __name__ == '__main__':
    jointId = [13, 17]
    frames = [
        [
            [[1000, -70], [-200, 0], [200, 0]],
            [[2000, 70], [-200, 0], [200, 0]],
            [[3000, -70], [-200, 0], [200, 0]],
            [[4000, 70], [-200, 0], [200, 0]],
        ],
        [
            [[500, 0], [-200, 0], [200, 0]],
            [[1000, 60], [-200, 0], [200, 0]],
            [[1500, 0], [-200, 0], [200, 0]],
            [[2000, 60], [-200, 0], [200, 0]],
            [[2500, 0], [-200, 0], [200, 0]],
            [[3000, 60], [-200, 0], [200, 0]],
        ],
    ]

    rospy.init_node('bezier_plan', anonymous=True)
    time.sleep(0.2)
    rospy.on_shutdown(rosShutdownHook)
    rospy.loginfo('node runing...')

    if mCtrl.SetBodyhubTo_setStatus(BODY_CTL_ID) == False:
        rospy.logerr('bodyhub to setStatus fail!')
        rospy.signal_shutdown('error')
        exit(1)

    angleList = [0, 0]

    angleLimitSet = [
        [-70, 70],
        [-70, 70]
    ]
    speedLimitSet = 0.2
    angleList = mCtrl.GetJointAngle(jointId)
    for i in range(len(jointId)):
        frames[i].insert(0, [[0, angleList[jointId[i]-1]], [0, 0], [200, 0]])

    mbPlanObj = bPlan.MultiBeizerPlan(2, 10, frames, angleLimitSet, speedLimitSet)

    while not rospy.is_shutdown():
        mbPlanObj.planing(100)

        trajectoryPoint = mbPlanObj.getTrajectory(50)
        SendTrajectory(BODY_CTL_ID, jointId, trajectoryPoint)
        mCtrl.WaitTrajectoryExecute()

        if mbPlanObj.sendComplete() == True:
            mCtrl.WaitTrajectoryExecute(True)
            break

    mCtrl.ResetBodyhub()
    rospy.signal_shutdown('over')
