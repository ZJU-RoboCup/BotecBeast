#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy 
from motion.motionControl import SetBodyhubTo_walking
import threading
import press_door
import time
from std_msgs.msg import Bool, Float64MultiArray
from mediumsize_msgs.srv import SetAction


walk_status = "stop"
gaitCommandPub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=10)

def publish(deltax, deltay, theta):
    gaitCommandPub.publish(data=[deltax, deltay, theta])

speed = 0
deltax = 5
rotate = 5
xz = -1


def walk():
    while not rospy.is_shutdown():
        print(walk_status)
        if walk_status == "stop":
            time.sleep(1)
        else:
            rospy.wait_for_message('/requestGaitCommand', Bool, 30)
            if walk_status == "stop":
                time.sleep(1)
                continue
            if walk_status == "w":
                publish(deltax / 100.0, 0, 0)
            elif walk_status == "a":
                publish(0, 0.02, 0)
            elif walk_status == "s":
                publish(-deltax / 100.0, 0, 0)
            elif walk_status == "d":
                publish(0, -0.02, 0)
            elif walk_status == "l":
                publish(0, 0, rotate)
            elif walk_status == "r":
                publish(0, 0, -rotate)
            elif walk_status == "x":
                publish(0.2 * 0.173, 0, 10 * xz)



def callback(data=Joy()):

    global walk_status
    if data.axes[-1] == 1:
        # up
        print("up")
        walk_status = "w"
    elif data.axes[-1] == -1:
        # down
        print("down")
        walk_status = "s"
    elif data.axes[-2] == 1:
        # left
        print("left")
        walk_status = "a"
    elif data.axes[-2] == -1:
        # right
        print("right")
        walk_status = "d"
    elif data.buttons[4] == 1:
        # lb
        print("lb")
        walk_status = "l"
    elif data.buttons[5] == 1:
        # rb
        print("rb")
        walk_status = "r"
    elif data.buttons[0] == 1:
        # A
        print("A")
        walk_status = "stop"
    elif data.buttons[1] == 1:
        # B
        print("B")
        SetBodyhubTo_walking(2)
    elif data.buttons[2] == 1:
        # X 
        print("X")
        press_door.PressDoor().main()
    elif data.buttons[3] == 1:
        # Y
        if walk_status != "stop":
            print("robot is running")
            return
        print("Y")
        client = rospy.ServiceProxy("/MediumSize/BodyHub/paceType", SetAction)
        global speed
        speed += 1
        speed %= 3
        print("speed: ", speed)
        client(speed, "")
    elif data.buttons[6] == 1:
        global deltax
        deltax += 2 
        deltax %= 10
        print("前进距离: {}".format(deltax))
    elif data.buttons[7] == 1:
        # global rotate
        # rotate += 2 
        # rotate %= 10
        # print("旋转距离: {}".format(rotate))
        global xz
        xz = -xz
        print(xz)
    elif data.buttons[8] == 1:
        print("xbox")
        walk_status = "x"

    

if __name__ == "__main__":
    rospy.init_node("joy_move")
    walk_status = "stop"
    threading.Thread(target=walk).start()
    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()