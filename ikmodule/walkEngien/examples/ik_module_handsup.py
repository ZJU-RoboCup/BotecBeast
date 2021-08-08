#!/usr/bin/env python
# coding=utf-8
import time
import math
import rospy
from sensor_msgs.msg import MultiDOFJointState
from geometry_msgs.msg import Transform
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

class HandsUp(IkModuleSim):
    def __init__(self):
        super(HandsUp,self).__init__()

    def waitJointTrajDone(self):
        rospy.wait_for_service('MediumSize/BodyHub/GetStatus')
        client = rospy.ServiceProxy('MediumSize/BodyHub/GetStatus', SrvString, 30)
        while not rospy.is_shutdown():
            response = client.call('get')
            if response.jointQueueSize > 0 or response.poseQueueSize > 0:
                break
        while not rospy.is_shutdown():
            response = client.call('get')
            if response.jointQueueSize == 0 and response.poseQueueSize == 0:
                break
            time.sleep(0.1)

    def main(self):
        if self.setstatus() and self.getposes():
            posesMsg = MultiDOFJointState()
            pose = Transform()
            pose.translation.x = self.leftLegX
            pose.translation.y = self.leftLegY
            pose.translation.z = self.leftLegZ+0.04
            pose.rotation.w = self.leftLegPosW
            pose.rotation.x = self.leftLegPosX
            pose.rotation.y = self.leftLegPosY
            pose.rotation.z = self.leftLegPosZ
            posesMsg.transforms.append(pose)

            pose = Transform()
            pose.translation.x = self.rightLegX
            pose.translation.y = self.rightLegY
            pose.translation.z = self.rightLegZ+0.04
            pose.rotation.w = self.rightLegPosW
            pose.rotation.x = self.rightLegPosX
            pose.rotation.y = self.rightLegPosY
            pose.rotation.z = self.rightLegPosZ
            posesMsg.transforms.append(pose)

            pose = Transform()
            pose.translation.x = 0.0
            pose.translation.y = Shoulder2_Y+Shoulder1_Y+Elbow1_Y+Wrist1_Y-1e-4
            pose.translation.z = Shoulder2_Z
            posesMsg.transforms.append(pose)
            
            pose = Transform()
            pose.translation.x = 0.0
            pose.translation.y = -(Shoulder2_Y+Shoulder1_Y+Elbow1_Y+Wrist1_Y-1e-4)
            pose.translation.z = Shoulder2_Z
            posesMsg.transforms.append(pose)

            linkIdList = [1,2,3,4]
            posesMsg.joint_names = [str(i) for i in linkIdList]
            posesMsg.joint_names.append(str(1.5))
            posesMsg.joint_names.append(str(6))
            toPosesPub.publish(posesMsg)
            self.waitJointTrajDone()
            time.sleep(2)

            self.reset()
            
            return True

if __name__ == '__main__':
    rospy.init_node('ikModelControl_node', anonymous=True)
    time.sleep(0.2)
    handsUp = HandsUp()
    handsUp.main()
