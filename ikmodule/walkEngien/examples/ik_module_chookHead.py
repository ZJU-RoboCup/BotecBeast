#!/usr/bin/env python
# coding=utf-8
import time
import math
import rospy
from sensor_msgs.msg import MultiDOFJointState
from geometry_msgs.msg import Transform
from ik_module.msg import PoseArray
from paramofposture import ParameterOfPosture
from ikmodulesim import IkModuleSim
from bodyhub.srv import SrvString

Shoulder2_X = 0.0
Shoulder2_Y = 0.08138
Shoulder2_Z = 0.18392

Shoulder1_X = 0.00
Shoulder1_Y = 0.00
Shoulder1_Z = 0.00

Elbow1_X = 0.0
Elbow1_Y = 0.07581
Elbow1_Z = 0.0

Wrist1_X = 0.00
Wrist1_Y = 0.08179
Wrist1_Z = 0.0

toPosesPub = rospy.Publisher('MediumSize/IKmodule/toPoses', MultiDOFJointState, queue_size=10)

class SwingAround(IkModuleSim):
    def __init__(self):
        super(SwingAround,self).__init__()

    def waitJointTrajDone(self):
        rospy.wait_for_service('MediumSize/BodyHub/GetStatus', 30)
        client = rospy.ServiceProxy('MediumSize/BodyHub/GetStatus', SrvString)
        while not rospy.is_shutdown():
            response = client.call('get')
            if response.jointQueueSize > 0 or response.poseQueueSize > 0:
                break
        while not rospy.is_shutdown():
            response = client.call('get')
            if response.jointQueueSize == 0 and response.poseQueueSize == 0:
                break
            time.sleep(0.1)

    def toInitPoses(self):
        if self.setstatus() and self.getposes():
            posesMsg = MultiDOFJointState()
            pose = Transform()
            pose.translation.x = self.leftLegX
            pose.translation.y = self.leftLegY
            pose.translation.z = self.leftLegZ+0.03
            pose.rotation.w = self.leftLegPosW
            pose.rotation.x = self.leftLegPosX
            pose.rotation.y = self.leftLegPosY
            pose.rotation.z = self.leftLegPosZ
            posesMsg.transforms.append(pose)

            pose = Transform()
            pose.translation.x = self.rightLegX
            pose.translation.y = self.rightLegY
            pose.translation.z = self.rightLegZ+0.03
            pose.rotation.w = self.rightLegPosW
            pose.rotation.x = self.rightLegPosX
            pose.rotation.y = self.rightLegPosY
            pose.rotation.z = self.rightLegPosZ
            posesMsg.transforms.append(pose)

            linkIdList = [1,2]
            posesMsg.joint_names = [str(i) for i in linkIdList]
            posesMsg.joint_names.append(str(1.5))
            posesMsg.joint_names.append(str(6))
            toPosesPub.publish(posesMsg)
            self.waitJointTrajDone()
            time.sleep(1)
            return True
        return False

    def waitPostureDone(self):
        rospy.wait_for_service('MediumSize/BodyHub/GetStatus', 30)
        jointTraj = rospy.ServiceProxy('MediumSize/BodyHub/GetStatus', SrvString)
        rospy.wait_for_service('MediumSize/IKmodule/GetStatus', 30)
        posture = rospy.ServiceProxy('MediumSize/IKmodule/GetStatus', SrvString)
        while not rospy.is_shutdown():
            response1 = jointTraj.call('get')
            response2 = posture.call('get')
            if (response1.jointQueueSize == 0 and response1.poseQueueSize == 0) and response2.poseQueueSize == 0:
                break
            time.sleep(0.1)

    def main(self):
        if self.toInitPoses():
            self.getposes()
            self.PosPara_wF.Torso_z = -self.leftLegZ
            self.PosPara_wF.Lfoot_y = self.leftLegY
            self.PosPara_wF.Rfoot_y = self.rightLegY
            # 手臂摆动的中心坐标,相对半径,弧度
            centerX, centerY, centerZ = Shoulder2_X, Shoulder2_Y, Shoulder2_Z
            radius = centerZ - self.leftArmZ
            radian = 80*math.pi/180.0 # in radians
            # loop
            i, count = 1, 100
            while not rospy.is_shutdown():
                self.leftArmX = centerX + radius*math.sin(radian/count*i)
                self.leftArmZ = centerZ - radius*math.cos(radian/count*i)
                self.rightArmX = self.leftArmX
                self.rightArmZ = self.leftArmZ

                poseArrayMsg = PoseArray()
                poseArrayMsg.poses.append(self.getleftlegPosMsg())
                poseArrayMsg.poses.append(self.getrightlegPosMsg())
                poseArrayMsg.poses.append(self.getleftarmPosMsg())
                poseArrayMsg.poses.append(self.getrightarmPosMsg())
                poseArrayMsg.controlId = 6
                self.targetPosesPub.publish(poseArrayMsg)

                i += 1
                if i == count+1:
                    rospy.loginfo("waitPostureDone...")
                    self.waitPostureDone()
                    break 

            self.getposes()
            startHandY = self.leftArmY
            # 手臂摆动的中心坐标,相对半径,弧度
            centerX, centerY, centerZ = Shoulder2_X, Shoulder2_Y, Shoulder2_Z
            radius = self.leftArmX - centerX
            radian = 10*math.pi/180.0 # in radians
            i, count = 1, 50
            while not rospy.is_shutdown():
                self.leftArmX = centerX + radius*math.cos(radian/count*i)
                self.leftArmY = startHandY - radius*math.sin(radian/count*i)
                self.rightArmX = self.leftArmX
                self.rightArmY = -self.leftArmY

                poseArrayMsg = PoseArray()
                poseArrayMsg.poses.append(self.getleftlegPosMsg())
                poseArrayMsg.poses.append(self.getrightlegPosMsg())
                poseArrayMsg.poses.append(self.getleftarmPosMsg())
                poseArrayMsg.poses.append(self.getrightarmPosMsg())
                poseArrayMsg.controlId = 6
                self.targetPosesPub.publish(poseArrayMsg)

                i += 1
                if i == count+1:
                    rospy.loginfo("waitPostureDone...")
                    self.waitPostureDone()
                    break 

            self.getposes()
            startHandX = self.leftArmX
            distance = 0.015
            i, count = 1, 50
            while not rospy.is_shutdown():
                self.leftArmX = startHandX - (distance/count*i)
                self.rightArmX = self.leftArmX
                
                poseArrayMsg = PoseArray()
                poseArrayMsg.poses.append(self.getleftlegPosMsg())
                poseArrayMsg.poses.append(self.getrightlegPosMsg())
                poseArrayMsg.poses.append(self.getleftarmPosMsg())
                poseArrayMsg.poses.append(self.getrightarmPosMsg())
                poseArrayMsg.controlId = 6
                self.targetPosesPub.publish(poseArrayMsg)

                i += 1
                if i == count+1:
                    rospy.loginfo("waitPostureDone...")
                    self.waitPostureDone()
                    self.getposes()
                    break 

            self.getposes()
            leftHandX, rightHandX = self.leftArmX, self.rightArmX
            A = 0.025
            numberOfCycle = 3
            i, count = 1, numberOfCycle*200
            while not rospy.is_shutdown():
                self.PosPara_wF.Torso_x = A*math.sin(2*math.pi*numberOfCycle/count*i)

                self.leftArmX = leftHandX-self.PosPara_wF.Torso_x
                self.rightArmX = rightHandX-self.PosPara_wF.Torso_x

                poseArrayMsg = PoseArray()
                poseArrayMsg.poses.append(self.getleftlegPosMsg())
                poseArrayMsg.poses.append(self.getrightlegPosMsg())
                poseArrayMsg.poses.append(self.getleftarmPosMsg())
                poseArrayMsg.poses.append(self.getrightarmPosMsg())
                poseArrayMsg.controlId = 6
                self.targetPosesPub.publish(poseArrayMsg)

                i += 1
                if i == count+1:
                    rospy.loginfo("waitPostureDone...")
                    self.waitPostureDone()
                    break 

            self.getposes()
            leftHandY, rightHandY = self.leftArmY, self.rightArmY
            A = 0.05
            numberOfCycle = 3
            i, count = 1, numberOfCycle*200
            while not rospy.is_shutdown():
                self.PosPara_wF.Torso_y = A*math.sin(2*math.pi*numberOfCycle/count*i)

                self.leftArmY = leftHandY-self.PosPara_wF.Torso_y
                self.rightArmY = rightHandY-self.PosPara_wF.Torso_y

                poseArrayMsg = PoseArray()
                poseArrayMsg.poses.append(self.getleftlegPosMsg())
                poseArrayMsg.poses.append(self.getrightlegPosMsg())
                poseArrayMsg.poses.append(self.getleftarmPosMsg())
                poseArrayMsg.poses.append(self.getrightarmPosMsg())
                poseArrayMsg.controlId = 6
                self.targetPosesPub.publish(poseArrayMsg)

                i += 1
                if i == count+1:
                    rospy.loginfo("waitPostureDone...")
                    self.waitPostureDone()
                    break 

            self.getposes()
            leftHandZ, rightHandZ = self.leftArmZ, self.rightArmZ
            A = 0.03
            numberOfCycle = 3
            i, count = 1, numberOfCycle*200
            while not rospy.is_shutdown():
                distance = A*math.sin(2*math.pi*numberOfCycle/count*i)
                self.PosPara_wF.Torso_z = -self.leftLegZ + distance

                self.leftArmZ = leftHandZ-distance
                self.rightArmZ = rightHandZ-distance

                poseArrayMsg = PoseArray()
                poseArrayMsg.poses.append(self.getleftlegPosMsg())
                poseArrayMsg.poses.append(self.getrightlegPosMsg())
                poseArrayMsg.poses.append(self.getleftarmPosMsg())
                poseArrayMsg.poses.append(self.getrightarmPosMsg())
                poseArrayMsg.controlId = 6
                self.targetPosesPub.publish(poseArrayMsg)

                i += 1
                if i == count+1:
                    rospy.loginfo("waitPostureDone...")
                    self.waitPostureDone()
                    break

        rospy.sleep(1)

if __name__ == '__main__':

    swingAround = SwingAround()

    def rosShutdownHook():
        swingAround.reset()
        rospy.signal_shutdown('node_close')

    rospy.init_node('swing_around_node', anonymous=True)
    time.sleep(0.2)
    rospy.on_shutdown(rosShutdownHook)

    # start the simulation once
    swingAround.main()
