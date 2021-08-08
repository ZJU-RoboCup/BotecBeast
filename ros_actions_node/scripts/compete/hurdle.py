#!/usr/bin/env python
# coding=utf-8
import time
import math
import rospy
from sensor_msgs.msg import MultiDOFJointState

from geometry_msgs.msg import Transform

from ik_lib.ikmodulesim import IkModuleSim
from bodyhub.srv import SrvString

from ik_lib.ikmodulesim.CtrlType import CtrlType as C

from motion.motionControl import ResetBodyhub, GetBodyhubStatus


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


toPosesPub = rospy.Publisher(
    'MediumSize/IKmodule/toPoses', MultiDOFJointState, queue_size=10)

class Hurdle(IkModuleSim):
    def __init__(self):
        ResetBodyhub()
        while GetBodyhubStatus().data != "preReady":
            time.sleep(0.1)
            continue

        super(Hurdle, self).__init__()

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
            self.waitPostureDone()
            time.sleep(2)
            self.getposes()

            self.PosPara_wF.Torso_z = -self.leftLegZ
            self.PosPara_wF.Lfoot_y = self.leftLegY
            self.PosPara_wF.Rfoot_y = self.rightLegY
            self.PosPara_wF.RArm_x = self.rightArmX
            self.PosPara_wF.RArm_y = self.rightArmY
            self.PosPara_wF.RArm_z = self.rightArmZ
            self.PosPara_wF.LArm_x = self.leftArmX
            self.PosPara_wF.LArm_y = self.leftArmY
            self.PosPara_wF.LArm_z = self.leftArmZ
            return True
        return False

    def main(self):
        if self.toInitPoses():
            self.body_motion(C.Torso, [math.pi / 12, -math.pi / 12, 0 ,0, -0.07, -0.05], 200)
            self.body_motion([C.Lfoot, C.Rfoot_R], [[-math.pi / 180 * 2, 0, 0, 0, 0.05, 0.09], -math.pi / 180 * 3])
            self.body_motion(C.Lfoot, [0, 0, 0, 0.19, -0.03, 0])
            self.body_motion([C.Lfoot, C.Torso], [[0, -math.pi / 6, 0, 0, 0, -0.045], [-math.pi / 12, 0, math.pi / 12, 0.04, 0.02, 0.03]])
            self.body_motion([C.Lfoot, C.Torso], [[0, math.pi / 12, 0, 0, 0, -0.025], [0, 0, 0, 0.02, 0.02, -0.02]])
            self.body_motion([C.Lfoot, C.Torso], [[0, math.pi / 12, math.pi / 180 * 9, 0, 0, -0.02], [0, 0, 0, 0.04, 0.04, -0.01]])
            self.body_motion(C.Torso, [0, math.pi / 6, 0, 0.03, 0.03, 0])
            self.body_motion([C.Lfoot_R, C.Lfoot_Y, C.Rfoot_R], [math.pi / 180 * 2, 0, math.pi / 180 * 3])
            self.body_motion([C.Rfoot, C.Torso], [[0, math.pi / 12, 0, 0, 0, 0.03], [0, 0, 0, 0.03, 0.03, 0.0]])
            self.body_motion([C.Rfoot, C.Torso], [[0, math.pi / 12, 0, 0, 0, 0.03], [0, 0, 0, 0.03, 0.03, 0.0]])
            self.body_motion([C.Rfoot, C.Torso], [[0, -math.pi / 12, 0, 0, 0, 0.05], [0, 0, math.pi / 12, 0, 0, 0.03]])
            self.body_motion(C.Rfoot, [0, 0, math.pi / 180 * 9, 0.22, -0.04, 0.0], 300)
            self.body_motion(C.Rfoot, [0, -math.pi / 12, 0, 0, 0.04, -0.09])
            self.reset()



if __name__ == '__main__':

    hurdle = Hurdle()
    def rosShutdownHook():
        hurdle.reset()
        rospy.signal_shutdown('node_close')

    rospy.init_node('hurdles', anonymous=True)

    rospy.on_shutdown(rosShutdownHook)

    hurdle.main()
