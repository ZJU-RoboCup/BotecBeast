#!/usr/bin/python
# coding=utf-8

import rospy
from motion.motionControl import WalkTheDistance, SetBodyhubTo_walking, ResetBodyhub
from image_manager import ImageManager

class Up2Down:
    def __init__(self):
        ResetBodyhub()
        self.start()

    def start(self):
        SetBodyhubTo_walking(2)
        WalkTheDistance(0.03, 0, -10)
        ResetBodyhub()

       
            

if __name__ == "__main__":
    rospy.init_node("ball2stairs")

    Up2Down()



