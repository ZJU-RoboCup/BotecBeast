#!/usr/bin/env python
# coding=utf-8

import rospy
# import math
# import eigen as e
from paramofposture import ParameterOfPosture


from geometry_msgs.msg import Pose
from ik_module.msg import PoseArray
from ik_module.srv import SrvPoses
from bodyhub.srv import SrvState
# from bodyhub.msg import JointControlPoint
# from std_msgs.msg import Float64MultiArray

class IkModuleSim(object):
    def __init__(self):
        # left leg pose
        self.leftLegX = 0.0
        self.leftLegY = 0.0
        self.leftLegZ = 0.0
        self.leftLegPosW = 0.0
        self.leftLegPosX = 0.0
        self.leftLegPosY = 0.0
        self.leftLegPosZ = 0.0
        # right leg pose
        self.rightLegX = 0.0
        self.rightLegY = 0.0
        self.rightLegZ = 0.0
        self.rightLegPosW = 0.0
        self.rightLegPosX = 0.0
        self.rightLegPosY = 0.0
        self.rightLegPosZ = 0.0
        # left arm pose
        self.leftArmX = 0.0
        self.leftArmY = 0.0
        self.leftArmZ = 0.0
        # right arm pose
        self.rightArmX = 0.0
        self.rightArmY = 0.0
        self.rightArmZ = 0.0
        # pose msg
        self.leftLegPosMsg  = Pose() #TODO:取消属性,修改到方法中去
        self.rightLegPosMsg = Pose()
        self.leftArmPosMsg  = Pose()
        self.rightArmPosMsg = Pose()
        self.PosPara_wF = ParameterOfPosture()

        # client and publisher
        self.setStatusClient = rospy.ServiceProxy('MediumSize/IKmodule/SetStatus', SrvState)
        self.getPosesClient = rospy.ServiceProxy('MediumSize/IKmodule/GetPoses', SrvPoses)
        self.targetPosesPub = rospy.Publisher('MediumSize/IKmodule/TargetPoses', PoseArray, queue_size=1000)
    
    def setstatus(self):
        rospy.wait_for_service("MediumSize/IKmodule/SetStatus", 30)
        try:
            resp = self.setStatusClient.call(6,"setStatus")
            rospy.loginfo("Set status with response: %d" % resp.stateRes)
            return True
        except rospy.ServiceException as e:
            rospy.logwarn("setstatus error: %s" % e)

    def reset(self):
        rospy.wait_for_service("MediumSize/IKmodule/SetStatus", 30)
        try:
            resp = self.setStatusClient.call(6,"reset")
            rospy.loginfo("Set status with response: %d" % resp.stateRes)
            return True
        except rospy.ServiceException as e:
            rospy.logwarn("Reset status service call failed: %s" % e)

    def getposes(self):
        '''
        更新当前姿态参数
        '''
        rospy.wait_for_service("MediumSize/IKmodule/GetPoses", 30)
        try:
            resp = self.getPosesClient.call("IkModuleSim")
            self.leftLegX = resp.poses[0].position.x
            self.leftLegY = resp.poses[0].position.y
            self.leftLegZ = resp.poses[0].position.z
            self.leftLegPosW = resp.poses[0].orientation.w
            self.leftLegPosX = resp.poses[0].orientation.x
            self.leftLegPosY = resp.poses[0].orientation.y
            self.leftLegPosZ = resp.poses[0].orientation.z

            self.rightLegX = resp.poses[1].position.x
            self.rightLegY = resp.poses[1].position.y
            self.rightLegZ = resp.poses[1].position.z
            self.rightLegPosW = resp.poses[1].orientation.w
            self.rightLegPosX = resp.poses[1].orientation.x
            self.rightLegPosY = resp.poses[1].orientation.y
            self.rightLegPosZ = resp.poses[1].orientation.z

            self.leftArmX = resp.poses[2].position.x
            self.leftArmY = resp.poses[2].position.y
            self.leftArmZ = resp.poses[2].position.z

            self.rightArmX = resp.poses[3].position.x
            self.rightArmY = resp.poses[3].position.y
            self.rightArmZ = resp.poses[3].position.z
            return True
        except rospy.ServiceException as e:
            rospy.logwarn("getposes error: %s" % e)

    def getleftarmPosMsg(self):
        '''
        直接通过给定相对坐标得到的左臂位姿消息
        '''
        self.leftArmPosMsg.position.x = self.leftArmX
        self.leftArmPosMsg.position.y = self.leftArmY
        self.leftArmPosMsg.position.z = self.leftArmZ
        self.leftArmPosMsg.orientation.x = 0
        self.leftArmPosMsg.orientation.y = 0
        self.leftArmPosMsg.orientation.z = 0
        self.leftArmPosMsg.orientation.w = 1
        return self.leftArmPosMsg

    def getrightarmPosMsg(self):
        '''
        直接通过给定相对坐标得到的右臂位姿消息
        '''
        self.rightArmPosMsg.position.x = self.rightArmX
        self.rightArmPosMsg.position.y = self.rightArmY
        self.rightArmPosMsg.position.z = self.rightArmZ
        self.rightArmPosMsg.orientation.x = 0
        self.rightArmPosMsg.orientation.y = 0
        self.rightArmPosMsg.orientation.z = 0
        self.rightArmPosMsg.orientation.w = 1
        return self.rightArmPosMsg
    
    def getleftlegPosMsg(self):
        '''
        通过给定躯干坐标转换得到的左腿位姿消息
        '''
        leftlegpos  = self.PosPara_wF.getleftlegpos()
        self.leftLegPosMsg.position.x = leftlegpos.X
        self.leftLegPosMsg.position.y = leftlegpos.Y
        self.leftLegPosMsg.position.z = leftlegpos.Z
        self.leftLegPosMsg.orientation.x = leftlegpos.x
        self.leftLegPosMsg.orientation.y = leftlegpos.y
        self.leftLegPosMsg.orientation.z = leftlegpos.z
        self.leftLegPosMsg.orientation.w = leftlegpos.w
        return self.leftLegPosMsg

    def getrightlegPosMsg(self):
        '''
        通过给定躯干坐标转换得到的右腿位姿消息
        '''
        rightlegpos = self.PosPara_wF.getrightlegpos()
        self.rightLegPosMsg.position.x = rightlegpos.X
        self.rightLegPosMsg.position.y = rightlegpos.Y
        self.rightLegPosMsg.position.z = rightlegpos.Z
        self.rightLegPosMsg.orientation.x = rightlegpos.x
        self.rightLegPosMsg.orientation.y = rightlegpos.y
        self.rightLegPosMsg.orientation.z = rightlegpos.z
        self.rightLegPosMsg.orientation.w = rightlegpos.w
        return self.rightLegPosMsg


if __name__ == '__main__':
    rospy.loginfo('Inherit IkModuleSim class to simulate with ikmodule')