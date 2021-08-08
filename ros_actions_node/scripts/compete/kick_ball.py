#!/usr/bin/python
# coding=utf-8

from lejufunc import client_action
import rospy
from motion.motionControl import SendGaitCommand, WaitForWalkingDone, WalkTheDistance, SetBodyhubTo_walking, SetBodyhubTo_setStatus, ResetBodyhub
import math
import time
from image_manager import ImageManager

import yaml

with open("robot_params.yaml", "r") as f:
    PAR_YAML = f.read()
    PAR_YAML = yaml.load(PAR_YAML)

LEFT_LEG_CENTER = PAR_YAML["LEFT_LEG_CENTER"]


class KickBall:
    def __init__(self, image_manager=ImageManager()):
        self.image_manager = image_manager
        self.start()
    
    def kick_ball(self):
        code = [
            ([0,0,0,0,0,0,0,0,0,0,0,0,0,-61,-18,0,61,18,0,0,0,0],1000,0),
            ([7,-13,0,-9,-6,-13,7,-7,0,0,0,-13,0,-37,-18,0,37,18,0,0,0,0],800,0),
            ([-4,-6,40,-65,-25,-11,-4,-3,-23,40,18,-11,0,-37,-18,0,37,18,0,0,0,0],800,0),
            ([0,-10,45,-3,55,-13,0,-3,-4,6,4,-10,-27,-61,-6,-27,61,18,0,0,0,0],300,500),
            ([7,-14,44,-76,-36,-13,7,-3,-4,7,4,-13,-9,-61,-6,-9,61,18,0,0,0,0],1000,0),
            ([7,-13,0,-9,-6,-13,7,-7,0,0,0,-13,0,-61,-18,0,61,18,0,0,0,0],800,0),
            ([0,0,0,0,0,0,0,0,0,0,0,0,0,-61,-18,0,61,18,0,0,0,0],800,0)
        ]
        client_action.action(code)

    def start(self):
        have_ball = False
        SetBodyhubTo_walking(2)
        while not rospy.is_shutdown():
            time.sleep(1)
            left_leg_point = (LEFT_LEG_CENTER, self.image_manager.chin_image.shape[0])
            
            red_center = self.image_manager.get_center_circle("ball")
            is_red = False
            if red_center:
                is_red = True
                have_ball = True

            black_center = self.image_manager.get_center_circle("hole", 1000)
            is_black = False
            if black_center:
                is_black = True

            if is_red and is_black:
                if black_center[0] - red_center[0] != 0:
                    k = (black_center[1] - red_center[1]) * 1.0 / (black_center[0] - red_center[0])
                    angle = math.atan(k) / math.pi * 180
                    symbol = 1 if angle > 0 else -1
                    angle = (90 - angle * symbol) * symbol
                    if angle < -5 or angle > 5:
                        WalkTheDistance(0, 0, angle)
                        continue

            if is_red:
                if left_leg_point[1] - 20 > red_center[1]:
                    print("球离得远，往前走")
                    WalkTheDistance((left_leg_point[1] - 20 - red_center[1]) / 1000.0, 0, 0)
                    continue
                if not (left_leg_point[0] - 20) < red_center[0] < (left_leg_point[0] + 20):
                    dis = (left_leg_point[0] - red_center[0]) / 1000.0
                    WalkTheDistance(0, dis, 0)
                    continue
            elif have_ball:
                WalkTheDistance(-0.05, 0, 0)
                continue

            if not is_red:
                WalkTheDistance(0.10, 0, 0)

            if is_red and is_black:
                WalkTheDistance(0.02, 0, 0)
                SetBodyhubTo_setStatus(2)
                self.kick_ball()
                ResetBodyhub()
                break

if __name__ == "__main__":
    rospy.init_node("kick_ball")
    SetBodyhubTo_walking(2)
    KickBall()