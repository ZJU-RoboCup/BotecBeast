#!/usr/bin/env python
# coding=utf-8
import time
import math
import rospy
from sensor_msgs.msg import MultiDOFJointState
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Pose
from ik_module.msg import PoseArray
from ik_module.srv import SrvPoses
from bodyhub.srv import SrvState
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

class PutDownHands(IkModuleSim):
    def __init__(self):
        super(PutDownHands,self).__init__()

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
            pose.translation.z = self.leftLegZ
            pose.rotation.w = self.leftLegPosW
            pose.rotation.x = self.leftLegPosX
            pose.rotation.y = self.leftLegPosY
            pose.rotation.z = self.leftLegPosZ
            posesMsg.transforms.append(pose)

            pose = Transform()
            pose.translation.x = self.rightLegX
            pose.translation.y = self.rightLegY
            pose.translation.z = self.rightLegZ
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
            xcom  = self.leftLegX
            width = self.leftLegY
            torsoHeight = -self.leftLegZ
            torsoHeightWalk = torsoHeight - 0.02
            footSeprate = 0.055*2+0.03
            # 手臂摆动的中心坐标和运动半径
            centerX, centerY, centerZ = Shoulder2_X, Shoulder2_Y, Shoulder2_Z
            radius = Shoulder1_Y+Elbow1_Y+Wrist1_Y-1e-6
            # 手臂下摆角度
            downAngle = 70*math.pi/180.0 # in radians
            count = 100
            if not rospy.is_shutdown():
                for i in range(count):
                    #############################################################################
                    self.leftLegX  = xcom - (xcom)*i/count
                    self.rightLegX = xcom - (xcom)*i/count
                    self.leftLegY  =  width + ((footSeprate/2 - width))/count*i
                    self.rightLegY = -(width + ((footSeprate/2 - width))/count*i)
                    self.leftLegZ  = -(torsoHeight - (torsoHeight - torsoHeightWalk)*i/count)
                    self.rightLegZ = -(torsoHeight - (torsoHeight - torsoHeightWalk)*i/count)

                    self.leftArmY  =  centerY + radius*math.cos(i*downAngle/count)
                    self.rightArmY = -centerY - radius*math.cos(i*downAngle/count)
                    self.leftArmZ  =  centerZ - radius*math.sin(i*downAngle/count)
                    self.rightArmZ =  centerZ - radius*math.sin(i*downAngle/count)
                    #############################################################################
                    # Left Leg
                    self.leftLegPosMsg.position.x = self.leftLegX
                    self.leftLegPosMsg.position.y = self.leftLegY
                    self.leftLegPosMsg.position.z = self.leftLegZ
                    self.leftLegPosMsg.orientation.x = self.leftLegPosX
                    self.leftLegPosMsg.orientation.y = self.leftLegPosY
                    self.leftLegPosMsg.orientation.z = self.leftLegPosZ
                    self.leftLegPosMsg.orientation.w = self.leftLegPosW
                    poseArrayMsg.poses.append(self.leftLegPosMsg)
                    # Right Leg
                    self.rightLegPosMsg.position.x = self.rightLegX
                    self.rightLegPosMsg.position.y = self.rightLegY
                    self.rightLegPosMsg.position.z = self.rightLegZ
                    self.rightLegPosMsg.orientation.x = self.rightLegPosX
                    self.rightLegPosMsg.orientation.y = self.rightLegPosY
                    self.rightLegPosMsg.orientation.z = self.rightLegPosZ
                    self.rightLegPosMsg.orientation.w = self.rightLegPosW
                    poseArrayMsg.poses.append(self.rightLegPosMsg)
                    # Left Arm
                    self.leftArmPosMsg.position.x = self.leftArmX
                    self.leftArmPosMsg.position.y = self.leftArmY
                    self.leftArmPosMsg.position.z = self.leftArmZ
                    self.leftArmPosMsg.orientation.x = 0
                    self.leftArmPosMsg.orientation.y = 0
                    self.leftArmPosMsg.orientation.z = 0
                    self.leftArmPosMsg.orientation.w = 1
                    poseArrayMsg.poses.append(self.leftArmPosMsg)
                    # Right Arm
                    self.rightArmPosMsg.position.x = self.rightArmX
                    self.rightArmPosMsg.position.y = self.rightArmY
                    self.rightArmPosMsg.position.z = self.rightArmZ
                    self.rightArmPosMsg.orientation.x = 0
                    self.rightArmPosMsg.orientation.y = 0
                    self.rightArmPosMsg.orientation.z = 0
                    self.rightArmPosMsg.orientation.w = 1
                    poseArrayMsg.poses.append(self.rightArmPosMsg)
                    # Publish target Poses
                    poseArrayMsg.controlId = 6
                    self.targetPosesPub.publish(poseArrayMsg)
                    poseArrayMsg.poses=[]
                
                rospy.sleep(1)
                rospy.loginfo("waitPostureDone...")
                self.waitPostureDone()
                rospy.sleep(1)
                rospy.loginfo("waitPostureDone...")
                self.reset()
                return True

if __name__ == '__main__':
    rospy.init_node('put_down_hands_node', anonymous=True)
    time.sleep(0.2)
    putDownHands = PutDownHands()
    putDownHands.main()
