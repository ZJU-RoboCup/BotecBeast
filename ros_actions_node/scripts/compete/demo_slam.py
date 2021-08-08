
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped 
import tf
import math
from std_msgs.msg import Bool, Float64MultiArray
from motion.motionControl import SetBodyhubTo_walking, WaitForWalkingDone, SetBodyhubTo_setStatus
import time
import sys
import press_door
from red_board import RedBoard
from pendulum import Pendulum
from lejufunc import client_action
import os

current_pose_topic = "/initialpose"

# (x, y, Y)
seesaw_middle_pose = (0.6308, 0.0328, -0.08)
seesaw_last_pose = (1.19, 0.0156, -0.075)
disk_front_pose = (1.625, 0.05, -0.11)
disk_middle_pose = (2.2027, 0.1186, 0.22)
disk_end_pose = (2.662, -0.8930, -1.52)
door_start_pose = (2.533, -1.378, -1.555)
pendulum_first_center = (210, 229)
pendulum_last_center = pendulum_first_center

uphill_first_pose = (2.761, -3.30, -1.62)

landmine_first_pose = (2.5648, -4.24, -2.27)
landmine_second_pose = (2.32, -4.469, -2.49)
landmine_middle_pose = (2.06, -4.63, -2.6686)
landmine_end_pose = (2.03, -4.60, 3.07)

bearing_first_pose = (1.339, -4.63, 3.08)
bearing_middle_pose = (1.329, -4.45, 1.549)
bearing_s_pose = (1.2923, -3.93, 1.575)
bearing_t_pose = (1.2771, -3.1297, 1.575)
brearing_end_pose = (1.246, -2.799, 1.57 )

gaitCommandPub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=10)


pendulum = Pendulum()




def publish(deltax, deltay, theta):
    gaitCommandPub.publish(data=[deltax / 2.0, deltay / 2.0, theta / 2.0])
    rospy.wait_for_message('/requestGaitCommand', Bool, 10)
    gaitCommandPub.publish(data=[deltax / 2.0, deltay / 2.0, theta / 2.0])


def toRPY(pose):
    return tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])


def is_arrive(current_pose, target_pose):
    if abs(target_pose[0] - current_pose[0]) < 0.02 and abs(target_pose[1] - current_pose[1]) < 0.02 and abs(target_pose[2] - current_pose[2]) < 0.1:
        return True
    return False

def pose_callback(data=PoseWithCovarianceStamped):
    p = data.pose.pose.position
    x, y = p.x, p.y
    _, _, Y = toRPY(data.pose.pose)
    return x, y, Y

def get_current_pose():
    pose = rospy.wait_for_message(current_pose_topic, PoseWithCovarianceStamped, 10)
    return pose_callback(pose)

def get_target_pose(target_pose):
    current_pose = get_current_pose()
    x = (target_pose[0] - current_pose[0]) * math.cos(-current_pose[2]) - (target_pose[1] - current_pose[1]) * math.sin(-current_pose[2])
    y = (target_pose[0] - current_pose[0]) * math.sin(-current_pose[2]) + (target_pose[1] - current_pose[1]) * math.cos(-current_pose[2])
    Y = (target_pose[2] - current_pose[2]) * 180 / math.pi % 360    
    Y = Y if Y <= 180 else (Y - 360)
    return x, y, Y


def line_go(target_pose, limit_x, limit_y, limit_Y, is_end=True, rotation_first=False, is_stop=False):
    if is_stop:
        WaitForWalkingDone()
        time.sleep(0.5)

    rospy.wait_for_message('/requestGaitCommand', Bool, 10)

    is_wait = False
    while not rospy.is_shutdown():
        x, y, Y = get_target_pose(target_pose)
        # print("x ^2 + y ^ 2 = " , x ** 2 + y **2)
        if x ** 2 + y ** 2 <= 0.015 and not is_wait:
            is_wait = True
            WaitForWalkingDone()
            continue
        break


    # 判断角度旋转
    current_angle = min(Y, limit_Y[0]) if Y > 0 else max(Y, limit_Y[1])

    # 用于判断机器人是否走到前面去了， 不需要精准定位的时候用
    _is_ending = False
    if is_end:
        if x < 0:
            _is_ending = True

    print(x, y, current_angle, is_end)

    if rotation_first:
        if abs(current_angle) >= 20:
            print("angle: {}".format(current_angle))
            publish(0, 0, current_angle)
            return True


    if abs(y) > 0.02 and not _is_ending:
        distance = min(y, limit_y[0]) if y > 0 else max(y, limit_y[1])
        print("y: {}, {}".format(distance, current_angle))
        publish(0, distance, current_angle)
        return True
    
    if abs(x) > 0.02 and not _is_ending:
        distance = min(x, limit_x[0]) if x > 0 else max(x, limit_y[1])
        print("x: {}, {}".format(distance, current_angle))
        publish(distance, 0, current_angle)
        return True

    if abs(current_angle) >= 5:
        print("angle: {}".format(current_angle))
        publish(0, 0, current_angle)
        return True

    return False

def publish_raduis():
    for _ in range(9):
        rospy.wait_for_message('/requestGaitCommand', Bool, 10)
        publish(0.3 * 0.173, 0, -10)


def run_raduis_first():
    while not rospy.is_shutdown():
        x, y, Y = get_target_pose(disk_middle_pose)
        if abs(Y) < 3:
            break
        current_angle = min(Y, 10) if Y > 0 else max(Y, -10)

        publish(0, 0, current_angle)
        WaitForWalkingDone()

def run_raduis_end():
    while not rospy.is_shutdown():
        x, y, Y = get_target_pose(disk_middle_pose)
        if x < 0.10:
            break
        distance = min(x, 0.14) if x > 0 else max(x, -0.14)
        publish(distance, 0, 0)
        WaitForWalkingDone()

    go_pose(disk_middle_pose, is_end=False, is_stop=True)

    if not rospy.is_shutdown():
        publish_raduis()

def run_raduis_push():
    while not rospy.is_shutdown():
        x, y, Y = get_target_pose(disk_end_pose)
        if abs(Y) < 5:
            break
        current_angle = min(Y, 10) if Y > 0 else max(Y, -10)

        publish(0, 0, current_angle)
        WaitForWalkingDone()

    while not rospy.is_shutdown():
        x, y, Y = get_target_pose(disk_end_pose)
        if x < 0.02:
            break
        distance = min(x, 0.14) if x > 0 else max(x, -0.14)
        publish(distance, 0, 0)
        WaitForWalkingDone()
    
    go_pose(disk_end_pose)


def pendulum_go(center):
    y = None
    while not rospy.is_shutdown():
        WaitForWalkingDone()
        rospy.wait_for_message('/requestGaitCommand', Bool, 10)
        time.sleep(0.5)
        c, angle = pendulum.get_gray()
        if not c:
            publish(0.05, 0, 0)
            continue
        if c[1] == None and y != None:
            publish(-0.02, 0, 0)
            continue

        x, y = c
        # angle 
        symbol = 1 if angle > 0 else -1
        angle = (90 - angle * symbol) * symbol
        if abs(angle) > 45:
            angle = (90 - angle * symbol)
        if angle < -3 or angle > 3:
            publish(-0.01, 0.01, angle)
            continue

        x1 = (center[0] - x + 30) / 1000.0
        if y != None:
            y1 = (center[1] - y - 20) / 1000.0
        else:
            y1 = 0
        print("c:", c)
        print(x1, y1)
        if abs(x1) < 0.05:
            x1 = 0
        if abs(y1) < 0.03:
            y1 = 0
        if x1 ==0 and y1 == 0:
            break
        x1 = min(x1, 0.10) if x1 > 0 else max(x1, -0.10)
        y1 = min(y1, 0.10) if y1 > 0 else max(y1, -0.10)
        print(y1, x1, 0)
        publish(y1, x1, 0)

    left_mean = None
    is_down = False
    count = 3
    print("get_mean")
    while not rospy.is_shutdown():
        mean = pendulum.get_mean()
        print(left_mean, mean)
        if left_mean != None and mean != None:
            if left_mean > mean:
                count -= 1
                if count == 0:
                    print("is down")
                    is_down = True
            else:
                count = 3

        left_mean = mean
        if not mean and is_down:
            break
        time.sleep(0.1)

    time.sleep(0.5)

    for _ in range(4):
        rospy.wait_for_message('/requestGaitCommand', Bool, 10)
        publish(0.10, 0, 0)
        


def go_pose(target_pose, limit_x=(0.14, -0.14), limit_y=(0.06, -0.06), limit_Y=(20, -20), is_end=True, rotation_first=False, is_stop=False):
    while not rospy.is_shutdown():
        if line_go(target_pose, limit_x, limit_y, limit_Y, is_end, rotation_first, is_stop):
            continue
        break


def open_hand():
    frames = [
            ([0,0,0,0,0,0,0,0,0,0,0,0,0,-30,0,0,-10,90,0,0,0,0],1000,0)
    ]
    client_action.action(frames)

if __name__ == "__main__":
    rospy.init_node("demo_slam")
    # SetBodyhubTo_setStatus(2)
    SetBodyhubTo_walking(2)

    while not rospy.is_shutdown() and len(sys.argv) > 1:
        pose = rospy.wait_for_message(current_pose_topic, PoseWithCovarianceStamped, 2)
        current_pose = pose_callback(pose)
        print("x: %.4f y: %.4f theta: %.4f"%current_pose)
        time.sleep(1)
    
    # 从跷跷板走到门口, 并按下开门按钮
    if 1:
        go_pose(seesaw_middle_pose, rotation_first=True,limit_x=(0.10, -0.10), is_stop=True)

        go_pose(seesaw_last_pose, rotation_first=True, limit_x=(0.10, -0.10), is_stop=True)
                
        go_pose(disk_front_pose, is_end=False, is_stop=True)

        run_raduis_first()

        RedBoard().check_red_board()

        run_raduis_end()

        for _ in range(4):
            rospy.wait_for_message('/requestGaitCommand', Bool, 10)
            publish(0.10, 0, 0)

        go_pose(disk_end_pose, is_end=False, is_stop=True)
        
        # 测试走圆弧
        # publish_raduis()
        
        go_pose(door_start_pose, limit_x=(0.10, -0.10), is_end=False, is_stop=True)

        press_door.PressDoor().main()
    

    # 中间转换 TODO 

    # 大摆锤
    pendulum_go(pendulum_first_center)
    # 当前机器人补一个旋转
    # rospy.wait_for_message('/requestGaitCommand', Bool, 10)
    # publish(0, 0, 10)
    # WaitForWalkingDone()
    # pendulum_go(pendulum_last_center)

    # 标定大摆锤前端点
    # while not rospy.is_shutdown():
    #     print(pendulum.get_gray())
    
    # # 走到坡道面前
    # go_pose(uphill_first_pose, is_end=False, is_stop=True)
    # rospy.wait_for_message('/requestGaitCommand', Bool, 10)
    # publish(0.04, 0, 5)
    # WaitForWalkingDone()

    # os.system("echo \"softdev\" | sudo -S killall roslaunch")

    # os.system(". /home/lemon/ls_pro/roban_ws/devel/setup.sh; roslaunch roban_walk walk.launch &")
    # time.sleep(5)
    # os.system(". /home/lemon/ls_pro/roban_ws/devel/setup.sh; rosrun walk_telecontrol walkTelecontrol_node.py")
    # time.sleep(5)
    # os.system("echo \"softdev\" | sudo -S killall roslaunch")


    # TODO 上坡
    # go_pose(landmine_first_pose, is_stop=True)
    # go_pose(landmine_second_pose, rotation_first=True, is_stop=True)
    # rospy.wait_for_message('/requestGaitCommand', Bool, 10)
    # publish(0.20, 0, 0)
    # WaitForWalkingDone()
    # go_pose(landmine_middle_pose, rotation_first=True, is_stop=True)
    # go_pose(landmine_end_pose, rotation_first=True, is_stop=True)
    # go_pose(bearing_first_pose, is_end=False, rotation_first=True)
    # go_pose(bearing_middle_pose, rotation_first=True)
    # go_pose(bearing_s_pose)
    # go_pose(bearing_t_pose)
    # go_pose(brearing_end_pose, is_end=False, is_stop=True)
    # # # 动作
    # SetBodyhubTo_setStatus(2)
    # open_hand()