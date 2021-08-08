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
            pose.translation.z = self.leftLegZ+0.01
            pose.rotation.w = self.leftLegPosW
            pose.rotation.x = self.leftLegPosX
            pose.rotation.y = self.leftLegPosY
            pose.rotation.z = self.leftLegPosZ
            posesMsg.transforms.append(pose)

            pose = Transform()
            pose.translation.x = self.rightLegX
            pose.translation.y = self.rightLegY
            pose.translation.z = self.rightLegZ+0.01
            pose.rotation.w = self.rightLegPosW
            pose.rotation.x = self.rightLegPosX
            pose.rotation.y = self.rightLegPosY
            pose.rotation.z = self.rightLegPosZ
            posesMsg.transforms.append(pose)

            linkIdList = [1,2]
            posesMsg.joint_names = [str(i) for i in linkIdList]
            posesMsg.joint_names.append(str(1.0))
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
            
            i, count = 1, 100
            yOffset = 0.05
            while not rospy.is_shutdown():
                self.PosPara_wF.Torso_y = yOffset * i / count
                poseArrayMsg = PoseArray()
                poseArrayMsg.poses.append(self.getleftlegPosMsg())
                poseArrayMsg.poses.append(self.getrightlegPosMsg())
                poseArrayMsg.poses.append(self.getleftarmPosMsg())
                poseArrayMsg.poses.append(self.getrightarmPosMsg())
                poseArrayMsg.controlId = 6
                self.targetPosesPub.publish(poseArrayMsg)

                i += 1
                if i >= count+1:
                    rospy.loginfo("waitPostureDone...")
                    self.waitPostureDone()
                    break 

            self.getposes()
            i, count = 1, 60
            zOffset = 0.03
            while not rospy.is_shutdown():
                self.PosPara_wF.Rfoot_z = zOffset * i / count
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
            rospy.sleep(2)

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
