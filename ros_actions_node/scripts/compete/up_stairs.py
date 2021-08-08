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

class UpStairs(IkModuleSim):
    def __init__(self):
        ResetBodyhub()
        while GetBodyhubStatus().data != "preReady":
            time.sleep(0.1)
            continue
        super(UpStairs, self).__init__()

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

    def main(self):
        if self.toInitPoses():
            
            stepX = 0.18
            stairH = 0.020
            timestamp = 100

            self.body_motion(C.Torso, [0, 0, 0, 0, 0, -0.02], timestamp)

            step_count = 3
            for index in range(step_count):
                y_offset = 0.075
                z_offset = 0.025
                left_diff_x = 0.02
                right_diff_x = 0.02
                if index == 0:
                    right_diff_x = 0
                if index == step_count - 1:
                    left_diff_x = 0

                self.body_motion(C.Torso, [0, 0, 0, 0.2 * stepX, y_offset, 0], timestamp)
                self.body_motion([C.Rfoot, C.Lfoot_R], [[0, 0, 0, -0.1 * stepX, 0, stairH + z_offset], math.pi / 180 * 4], timestamp)
                self.body_motion(C.Rfoot, [0, 0, 0, 1.1 * stepX + right_diff_x, 0, 0], timestamp)
                self.body_motion([C.Torso, C.Rfoot_z], [[0, math.pi / 6 , 0, 0.3 * stepX, -y_offset * 2, 0], -z_offset], timestamp * 2)
                self.body_motion([C.Lfoot, C.Torso_x, C.Rfoot_R],  [[-math.pi / 180 * 4, 0, 0, 0, 0, z_offset + stairH], 0.1 * stepX, -math.pi / 180 * 4], timestamp)
                self.body_motion([C.Lfoot, C.Torso],  [[0, 0, 0, stepX + left_diff_x, 0, -z_offset/2], [0, -math.pi / 6, 0, 0.4 * stepX + left_diff_x, 0, 0]], timestamp)
                self.body_motion([C.Lfoot, C.Rfoot_R],  [[0, 0, 0, 0, 0, -z_offset/2], math.pi / 180 * 4], timestamp)
                self.body_motion(C.Torso, [0, 0, 0, 0, y_offset, stairH], timestamp)

            self.reset()


if __name__ == '__main__':

    stairs = UpStairs()
    def rosShutdownHook():
        stairs.reset()
        rospy.signal_shutdown('node_close')

    rospy.init_node('stairs', anonymous=True)

    rospy.on_shutdown(rosShutdownHook)

    stairs.main()