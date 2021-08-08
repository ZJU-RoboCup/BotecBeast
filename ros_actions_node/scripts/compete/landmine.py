#!/usr/bin/python
# coding=utf-8

import rospy
from image_manager import ImageManager
from motion.motionControl import WalkTheDistance, SetBodyhubTo_walking, ResetBodyhub
import time
import yaml


with open("robot_params.yaml", "r") as f:
    PAR_YAML = f.read()
    PAR_YAML = yaml.load(PAR_YAML)

# 可跨越的最低像素宽度
GO_MIN_WIDTH = PAR_YAML["GO_MIN_WIDTH"]

# 机器人中心点的 X 值
ROBOT_CENTER_X = PAR_YAML["ROBOT_CENTER_X"]

# 差值
ROBOT_CENTER_DIFF = PAR_YAML["ROBOT_CENTER_DIFF"]

LANDMINE_MAX_Y = PAR_YAML["LANDMINE_MAX_Y"]

WALL_MAX_X = PAR_YAML["WALL_MAX_X"]
WALL_MIN_X = PAR_YAML["WALL_MIN_X"]


class Landmine:
    def __init__(self, image_manager=ImageManager()):
        self.image_manager = image_manager
        self.start()
        ResetBodyhub()

    def start(self):
        SetBodyhubTo_walking(2)

        have_landmine = False
        pre_landmine_y = 0
        have_orange = False
        while not rospy.is_shutdown():
            time.sleep(1)
            params = self.image_manager.get_orange_params()
            
            if params and not have_landmine:
                have_orange = True
                center, _, scope = params
                print("角度: {}".format(scope))
                if abs(scope) > 30:
                    symbol = -1 if scope > 0 else 1
                    scope = (90 - abs(scope)) * symbol

                if abs(scope) > 3:
                    print(scope)
                    print("机器人与栏杆不平行, 调整姿态")
                    WalkTheDistance(0, 0, -1 * scope)
                else:
                    # 判断机器人是否在区域中心
                    if center[1] < 200:
                        WalkTheDistance((200 - center[1]) / 1000.0, 0, 0)
                    elif abs(center[0] - ROBOT_CENTER_X) > ROBOT_CENTER_DIFF:
                        print("机器人不在中心，调整到中心位置")
                        WalkTheDistance(0, (ROBOT_CENTER_X - center[0]) / 1000.0, 0)
                    else:
                        print(center[1])
                        print("机器人已经在中心，走到栏杆前方")
                        WalkTheDistance((430 - center[1]) / 1000.0, 0, 0)
                        break
                continue
            if have_orange:
                print("栏杆丢失, 后退")
                WalkTheDistance(-0.02, 0, 0)
                continue

            wall, landmine = self.image_manager.get_landmine_params()

            all_x = []
            if wall:
                wall_scope, wall_x = wall
                if wall_x > ROBOT_CENTER_X: # 墙在右边
                    if wall_x < WALL_MAX_X:
                        WalkTheDistance(0, 0.05, 0)
                else:
                    if wall_x > WALL_MIN_X:
                        WalkTheDistance(0, -0.05, 0)

                angle = wall_scope
                symbol = 1 if angle > 0 else -1
                angle = (90 - angle * symbol) * symbol
                if abs(angle) > 45:
                    angle = (90 - angle * symbol)
                if angle < -5 or angle > 5:
                    print("与墙对齐, 旋转 {}".format(angle))
                    WalkTheDistance(0, 0, angle)
                    continue
                all_x.append(wall_x)

            # 获取当前视野中的 "地雷" 或者 "墙壁"
            if landmine:
                have_landmine = True
                max_y = max(landmine, key=lambda n: n[1])[1]
                if max_y - pre_landmine_y < -100:
                    print(max_y, pre_landmine_y)
                    print("障碍物消失,往后退一点")
                    WalkTheDistance(-0.05, 0, 0)
                    continue
                pre_landmine_y = max_y
                # print(max_y)
                if max_y < LANDMINE_MAX_Y - 10:
                    print("地雷离得比较远....")
                    WalkTheDistance((LANDMINE_MAX_Y - max_y) / 1000.0, 0, 0)
                    continue

                for l_center in landmine:
                    # 剔除远处的障碍物
                    if l_center[1] > LANDMINE_MAX_Y - 20:
                        all_x.append(l_center[0])
                all_x.sort()

                
                # 首先查询 all_x 中是否有可以通过的间距, 如果存在 则移动到该点即可.

                is_continue = False
                for i in range(len(all_x) - 1):
                    print(all_x[i+1] - all_x[i])
                    if all_x[i+1] - all_x[i] > GO_MIN_WIDTH:
                        print("有可跨越的位置")
                        center_x = (all_x[i+1] + all_x[i]) / 2
                        if abs(ROBOT_CENTER_X - center_x) < 20:
                            print("可以跨越了 go")
                            have_landmine = False
                            pre_landmine_y = 0
                            WalkTheDistance(0.40, 0, 0)
                        else:
                            WalkTheDistance(
                                0, (ROBOT_CENTER_X - center_x) / 1000.0, 0)
                        is_continue = True
                        break

                if is_continue:
                    continue

                if all_x[0] - ROBOT_CENTER_X > GO_MIN_WIDTH / 2 or \
                    ROBOT_CENTER_X - all_x[-1] > GO_MIN_WIDTH / 2:
                    print("可以跨越了 go")
                    have_landmine = False
                    pre_landmine_y = 0
                    WalkTheDistance(0.40, 0, 0)
                    continue
                
                if wall:
                    if all_x[0] == wall[1]:
                        # 墙在左边
                        WalkTheDistance(0, -0.20, 0)
                    elif all_x[-1] == wall[1]:
                        #  墙在右边
                        WalkTheDistance(0, 0.20, 0)
                    else:
                        raise Exception("墙体位置异常")
                    continue

                if len(all_x) == 1:
                    distance = abs(ROBOT_CENTER_X -
                                   all_x[0]) - GO_MIN_WIDTH / 2
                    if distance < 0:
                        symbol = -1 if ROBOT_CENTER_X - all_x[0] > 0 else 1
                        WalkTheDistance(
                            0, symbol * (distance + 20) / 1000.0, 0)
                        continue
                else:
                    raise Exception("此时有两个障碍物,情况未处理")

                raise Exception("条件异常,请处理")

            else:
                if have_landmine:
                    print("障碍物消失,往后退一点")
                    WalkTheDistance(-0.05, 0, 0)
                else:
                    print("没看到障碍物, 往前走")
                    WalkTheDistance(0.10, 0, 0)


if __name__ == "__main__":
    rospy.init_node("landmind")
    Landmine()
