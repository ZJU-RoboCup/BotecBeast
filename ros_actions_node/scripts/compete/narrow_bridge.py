#!/usr/bin/python
# coding=utf-8

import rospy
from motion.motionControl import WalkTheDistance, SetBodyhubTo_walking
import time
from image_manager import ImageManager

import yaml


with open("robot_params.yaml", "r") as f:
    PAR_YAML = f.read()
    PAR_YAML = yaml.load(PAR_YAML)


MAX_X = PAR_YAML["GREEN_MAX_X"]
MIN_X = PAR_YAML["GREEN_MIN_X"]


class NarrowBridge:
    def __init__(self, image_manager):
        self.image_manager = image_manager
        self.start()

    def start(self):

        SetBodyhubTo_walking(2)

        while not rospy.is_shutdown():
            par = self.image_manager.get_green_scope_x()
            if par is None:
                break
            scope, mid_x, _ = par
            angle = scope
            symbol = 1 if angle > 0 else -1
            angle = (90 - angle * symbol) * symbol
            if abs(angle) > 45:
                angle = (90 - angle * symbol)
            if angle < -3 or angle > 3:
                WalkTheDistance(0, 0, angle)
                continue

            if MIN_X < mid_x < MAX_X:
                WalkTheDistance(0.30, 0, 0)
            else:
                WalkTheDistance(0, ((MIN_X + MAX_X) / 2 - mid_x) / 1000.0, 0)
            


if __name__ == "__main__":

    rospy.init_node("narrow_bridge")
    NarrowBridge(ImageManager())