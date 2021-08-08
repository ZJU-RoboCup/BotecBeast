#!/usr/bin/python
# coding=utf-8

import rospy
from motion.motionControl import SendGaitCommand, WaitForWalkingDone, WalkTheDistance, SetBodyhubTo_walking
import time
from image_manager import ImageManager
import yaml



with open("robot_params.yaml", "r") as f:
    PAR_YAML = f.read()
    PAR_YAML = yaml.load(PAR_YAML)


ROBOT_CENTER_X = PAR_YAML["ROBOT_CENTER_X"]



class Door():
    def __init__(self, image_manager):
        self.image_manager = image_manager
        self.start()

    def start(self):
        SetBodyhubTo_walking(2)
        
        is_up = False
        WalkTheDistance(0.15, 0.00, 0)
        WalkTheDistance(0.00, 0.00, 130)
        WalkTheDistance(0.15, 0.00, 0)

        while not rospy.is_shutdown():
            # 获取边线 如果没有，则往边上挪挪

            wall_par = self.image_manager.get_wall_scope_x()
            if wall_par is None:
                print("no wall..")
                continue
                # raise Exception("no wall")
            
            wall_scope, mid_x = wall_par
            center_x = ROBOT_CENTER_X - mid_x

            angle = wall_scope
            symbol = 1 if angle > 0 else -1
            angle = (90 - angle * symbol) * symbol
            if abs(angle) > 45:
                angle = (90 - angle * symbol)
            if angle < -3 or angle > 3:
                print("与墙对齐, 旋转 {}".format(angle))
                WalkTheDistance(0, 0, angle)


            if is_up == False:
                distance = self.image_manager.get_blue_distance()
                print(distance, center_x)
                if distance == 0:
                    WalkTheDistance(0.10, 0, 0)
                elif distance > 300:
                    is_up = True
                    WalkTheDistance((distance - 300) / 1000.0, 0, 0)
                else:
                    is_up = True
            else:
                print(center_x)

                WalkTheDistance(0, -(480 - center_x) / 1000.0, -10)
                WalkTheDistance(0.50, 0, 0)
                break



if __name__ == "__main__":
    rospy.init_node("landmine_node")
    Door(ImageManager())
    # rospy.spin()
    