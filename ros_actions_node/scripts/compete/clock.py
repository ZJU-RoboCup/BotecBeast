#!/usr/bin/python
# coding=utf-8

import rospy
from motion.motionControl import WalkTheDistance, SetBodyhubTo_walking, SendGaitCommand
import time
import math


class Clock:
    def __init__(self):
        self.start()

    def start(self):
        SetBodyhubTo_walking(2)
        
        WalkTheDistance(0.40, 0, 0)
        WalkTheDistance(0, 0, 45)

        r = 0.4
        t = 10
        x = r * math.sin(t * math.pi / 180)
        y = r - r * math.cos(t * math.pi / 180)
        for _ in range(100):
            SendGaitCommand(math.sqrt(x ** 2 + y ** 2), 0, -t)


            
if __name__ == "__main__":
    rospy.init_node("clock")
    Clock()