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


class Ball2Stairs:
    def __init__(self, image_manager):
        self.image_manager = image_manager
        self.start()

    def go_wall(self):
        wall_par = self.image_manager.get_wall_scope_x()
        if wall_par is None:
            WalkTheDistance(0.20, 0, 0)
        else:
            scope, mid_x = wall_par
            if mid_x > ROBOT_CENTER_X: # 墙在右边
                if mid_x < WALL_MAX_X:
                    WalkTheDistance(0, 0.10, 0)
            else:
                if mid_x > WALL_MIN_X:
                    WalkTheDistance(0, -0.10, 0)

            angle = scope
            symbol = 1 if angle > 0 else -1
            angle = (90 - angle * symbol) * symbol
            if angle < -3 or angle > 3:
                WalkTheDistance(0, 0, angle)
            else:
                WalkTheDistance(0.20, 0, 0)


    def start(self):
        SetBodyhubTo_walking(2)
        WalkTheDistance(0, 0, 150)
        count = 1
        while not rospy.is_shutdown():
            time.sleep(1)
            par = self.image_manager.get_step_scope_y()

            if par is None:
                count += 1
                if count < 2:
                    continue
                else:
                    count = 1
                    self.go_wall()
                    print("往前走")
                    continue
            count = 1
            print(par)
    
            scope, max_y = par
            angle = scope
            symbol = 1 if angle > 0 else -1
            angle = (90 - angle * symbol) * symbol
            if abs(angle) > 45:
                angle = (90 - angle * symbol)
            if angle < -3 or angle > 3:
                WalkTheDistance(-0.01, 0, angle)
                continue

            if max_y < 220:
                distance = (220 - max_y) / 1000.0
                WalkTheDistance(distance if distance < 0.10 else 0.10, 0, 0)
                continue

            ResetBodyhub()
            break
            


if __name__ == "__main__":
    rospy.init_node("ball2stairs")

    Ball2Stairs(ImageManager())



