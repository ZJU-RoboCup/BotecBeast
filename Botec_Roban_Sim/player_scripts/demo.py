#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time

import rospy
import rospkg

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.motionControl as mCtrl
import motion.TrajectoryPlan as tPlan
from ik_lib.ikmodulesim import IkModuleSim
from ik_lib.ikmodulesim.CtrlType import CtrlType as C
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

NodeControlId = 2
tpObject = tPlan.TrajectoryPlanning(22,10.0)

idList = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22]
angleList = [[0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0, 0,0,0, 0,0, 0,0],1000,0]

def SendTrajectory(controlId, trajectoryPoint):
    for m in range(len(trajectoryPoint[0])):
        jointPosition = []
        for n in range(len(trajectoryPoint)):
            jointPosition.append(trajectoryPoint[n][m].y)

        mCtrl.__jointPositionPub.publish(positions=jointPosition, mainControlID=controlId)


def actionExecute(poseList):
    angleList[0] = mCtrl.GetJointAngle(idList)
    poseList.insert(0, angleList)

    tpObject.setInterval(poseList[0][1])
    tpObject.planningBegin(poseList[0][0], poseList[1][0])

    for poseIndex in range(2, len(poseList)):
        tpObject.setInterval(poseList[poseIndex-1][1])
        trajectoryPoint = tpObject.planning(poseList[poseIndex][0])
        SendTrajectory(NodeControlId, trajectoryPoint)
        if poseList[poseIndex-1][2] > 0:
            mCtrl.WaitForActionExecute(True)
            time.sleep(poseList[poseIndex-1][2]/1000.0)
        else:
            mCtrl.WaitForActionExecute()
            
    trajectoryPoint = tpObject.planningEnd()
    SendTrajectory(NodeControlId, trajectoryPoint)
    mCtrl.WaitForActionExecute(True)

def action():
    mCtrl.SetBodyhubTo_setStatus(NodeControlId)

    print('蹲着...')
    u1_u8e72u7740 = [
        ([0,0,0,0,0,0,0,0,0,0,0,0,0,-65,-30,0,65,30,0,0,0,0],1000,0),
        ([0,0,65,-100,-43,0,0,0,-65,100,43,0,-10,-65,-40,10,65,40,0,0,0,23],1000,0)
    ]
    actionExecute(u1_u8e72u7740)

    mCtrl.ResetBodyhub()

def walk(step):
    if mCtrl.SetBodyhubTo_walking(NodeControlId) == False:
        rospy.logerr('bodyhub to setStatus fail!')
        rospy.signal_shutdown('error')
        exit(1)

    for i in range(0, step):
        mCtrl.SendGaitCommand(0.1, 0.0, 0.0)

    mCtrl.WaitForWalkingDone()
    mCtrl.ResetBodyhub()

def rosShutdownHook():
    mCtrl.ResetBodyhub()



def torso_callback(data):
    global roban_torsoPR
    roban_torsoPR = data.data
    # print(roban_torsoPR)
    

def listener():

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
    
    rospy.Subscriber("/sim/torso/PR", Float64MultiArray, torso_callback)
    

        # spin() simply keeps python from exiting until this node is stopped
    
def talker():
    pub = rospy.Publisher('/playerStart', Float64, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = 0.0
        #rospy.loginfo(hello_str)
        pub.publish(hello_str)
        # print('\r Ctrl+c to break; total time use .... %f  当前坐标为：X:%f ,Y:%f ,Z:%f' % (time_total, roban_torsoPR[0],roban_torsoPR[1],roban_torsoPR[2]),end="")
        print('\r Ctrl+c to break;  当前坐标为：X:%f 当前坐标为：Y:%f 当前坐标为：Z:%f ' % (roban_torsoPR[0],roban_torsoPR[1],roban_torsoPR[2]))
        rate.sleep()

if __name__ == '__main__':
    #rospy.init_node('listener', anonymous=True)
    print('node runing...')
    roban_torsoPR = []
    listener()
    print("显示Roban坐标：")
    mCtrl.ResetBodyhub()
    time.sleep(1)
    rospy.init_node('walking_SPath_node', anonymous=True)
    time.sleep(0.5)
    # talker()

    while not rospy.is_shutdown():
        action()

