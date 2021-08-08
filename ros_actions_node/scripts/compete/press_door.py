#!/usr/bin/env python
# coding=utf-8
import time
import rospy
from ik_lib.ikmodulesim import IkModuleSim
from ik_lib.ikmodulesim.CtrlType import CtrlType as C
from motion.motionControl import ResetBodyhub, GetBodyhubStatus

class PressDoor(IkModuleSim):
    def __init__(self):
        ResetBodyhub()
        while GetBodyhubStatus().data != "preReady":
            time.sleep(0.1)
            ResetBodyhub()
            continue

        super(PressDoor, self).__init__()

    def main(self):
        if self.toInitPoses():
            self.body_motion([C.RArm_z, C.RArm_y], [0.05, -0.05], 50)
            self.body_motion([C.RArm_z, C.RArm_x], [0.11, 0.04], 50)
            self.body_motion([C.RArm_x, C.RArm_y], [0.09, 0.03], 10)
            self.reset()


if __name__ == '__main__':

    door = PressDoor()
    def rosShutdownHook():
        door.reset()
        rospy.signal_shutdown('node_close')

    rospy.init_node('door', anonymous=True)

    rospy.on_shutdown(rosShutdownHook)

    door.main()
