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
            pose.translation.z = self.leftLegZ+0.02
            pose.rotation.w = self.leftLegPosW
            pose.rotation.x = self.leftLegPosX
            pose.rotation.y = self.leftLegPosY
            pose.rotation.z = self.leftLegPosZ
            posesMsg.transforms.append(pose)

            pose = Transform()
            pose.translation.x = self.rightLegX
            pose.translation.y = self.rightLegY
            pose.translation.z = self.rightLegZ+0.02
            pose.rotation.w = self.rightLegPosW
            pose.rotation.x = self.rightLegPosX
            pose.rotation.y = self.rightLegPosY
            pose.rotation.z = self.rightLegPosZ
            posesMsg.transforms.append(pose)

            pose = Transform()
            pose.translation.x = 0.0
            pose.translation.y = Shoulder2_Y+Shoulder1_Y+Elbow1_Y+Wrist1_Y-1e-6
            pose.translation.z = Shoulder2_Z
            posesMsg.transforms.append(pose)
            
            pose = Transform()
            pose.translation.x = 0.0
            pose.translation.y = -(Shoulder2_Y+Shoulder1_Y+Elbow1_Y+Wrist1_Y-1e-6)
            pose.translation.z = Shoulder2_Z
            posesMsg.transforms.append(pose)

            linkIdList = [1,2,3,4]
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
            poseArrayMsg = PoseArray()
            # torso parameters
            torsoHeight = -self.leftLegZ
            torsoHeightWalk = torsoHeight
            t = math.sqrt(torsoHeightWalk*torsoHeightWalk + 0*0)
            r = 0.06
            a = r/(2*math.pi)
            i, rCount = 1, 3
            # 手臂摆动的角度,中心坐标,相对半径
            count = 300
            downAngle = 70*math.pi/180.0 # in radians
            centerX, centerY, centerZ = Shoulder2_X, Shoulder2_Y, Shoulder2_Z
            downRadius = Shoulder1_Y+Elbow1_Y+Wrist1_Y-1e-6
            # loop
            while not rospy.is_shutdown():
                if i <= count:
                    i += 1
                    sita = 2*math.pi*i/count
                    x = a*sita*math.cos(sita)
                    y = a*sita*math.sin(sita)
                    # put hands down
                    self.leftArmY  =  centerY + downRadius*math.cos( i*downAngle/count )
                    self.rightArmY = -centerY - downRadius*math.cos( i*downAngle/count )
                    self.leftArmZ  =  centerZ - downRadius*math.sin( i*downAngle/count )
                    self.rightArmZ =  centerZ - downRadius*math.sin( i*downAngle/count )

                elif i <= count*(rCount+1):
                    i += 1
                    sita = 2*math.pi*i/count
                    x = r*math.cos(sita)
                    y = r*math.sin(sita)
                
                elif i <= count*(rCount+2):
                    i += 1
                    sita = -2*math.pi*(count*(rCount+2)-i)/count
                    x = a*sita*math.cos(sita+math.pi)
                    y = a*sita*math.sin(sita+math.pi)

                elif i >  count*(rCount+2):
                    i = 1;  #reset i
                    rospy.sleep(1)
                    rospy.loginfo("waitPostureDone...")
                    self.waitPostureDone()
                    rospy.sleep(1)
                    rospy.loginfo("Swing around done.")
                    self.reset()
                    return True

                t_V = math.sqrt(t*t-x*x-y*y)
                # update torso parameters
                self.PosPara_wF.Torso_x = 0
                self.PosPara_wF.Torso_y = 0
                self.PosPara_wF.Torso_z = torsoHeightWalk
                self.PosPara_wF.Torso_R = math.asin(y/t_V)
                self.PosPara_wF.Torso_P = math.asin(x/t_V)
                self.PosPara_wF.Torso_Y = 0.0
                self.PosPara_wF.Lfoot_y = self.leftLegY
                self.PosPara_wF.Rfoot_y = self.rightLegY
                # print( "Torso Pose: ", i, self.PosPara_wF.Torso_x, self.PosPara_wF.Torso_y, self.PosPara_wF.Torso_z, self.PosPara_wF.Torso_R, self.PosPara_wF.Torso_P,self.PosPara_wF.Torso_Y)
                ###############################################
                # Left Leg
                leftLegPosMsg = self.getleftlegPosMsg()
                poseArrayMsg.poses.append(leftLegPosMsg)
                # Right Leg
                rightLegPosMsg = self.getrightlegPosMsg()
                poseArrayMsg.poses.append(rightLegPosMsg)
                # Left Arm
                leftArmPosMsg = self.getleftarmPosMsg()
                poseArrayMsg.poses.append(leftArmPosMsg)
                # Right Arm
                rightArmPosMsg = self.getrightarmPosMsg()
                poseArrayMsg.poses.append(rightArmPosMsg)
                # Publish target Poses
                poseArrayMsg.controlId = 6
                self.targetPosesPub.publish(poseArrayMsg)
                poseArrayMsg.poses=[]

if __name__ == '__main__':
    rospy.init_node('swing_around_node', anonymous=True)
    time.sleep(0.2)
    swingAround = SwingAround()
    # start the simulation once
    swingAround.main()
