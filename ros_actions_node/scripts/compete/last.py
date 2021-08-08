#!/usr/bin/python
# coding=utf-8

import rospy
from motion.motionControl import WalkTheDistance, SetBodyhubTo_walking, ResetBodyhub
import time
from image_manager import ImageManager

import yaml

with open("robot_params.yaml", "r") as f:
    PAR_YAML = f.read()
    PAR_YAML = yaml.load(PAR_YAML)


# 机器人中心点的 X 值
ROBOT_CENTER_X = PAR_YAML["ROBOT_CENTER_X"]

WALL_MAX_X = PAR_YAML["WALL_MAX_X"]
WALL_MIN_X = PAR_YAML["WALL_MIN_X"]


class Last:
    def __init__(self, image_manager):
        self.image_manager = image_manager
        self.start()

    def start(self):
        SetBodyhubTo_walking(2)

        count = 0
        while not rospy.is_shutdown():
            time.sleep(1)

            wall_par = self.image_manager.get_wall_scope_x()
            if wall_par is None:
                count += 1
                WalkTheDistance(0.20, 0, 0)
            else:
                scope, mid_x = wall_par
                if mid_x > ROBOT_CENTER_X: # 墙在右边
                    if mid_x < WALL_MAX_X:
                        WalkTheDistance(0, 0.20, 0)
                else:
                    if mid_x > WALL_MIN_X:
                        WalkTheDistance(0, -0.20, 0)
                angle = scope
                symbol = 1 if angle > 0 else -1
                angle = (90 - angle * symbol) * symbol
                if angle < -3 or angle > 3:
                    WalkTheDistance(0, 0, angle)
                else:
                    count += 1
                    WalkTheDistance(0.20, 0, 0)
            if count > 5:
                break




if __name__ == "__main__":
    rospy.init_node("last")

    Last(ImageManager())



