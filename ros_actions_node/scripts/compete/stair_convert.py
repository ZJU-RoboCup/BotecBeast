#!/usr/bin/python
# coding=utf-8

import rospy
from motion.motionControl import SetBodyhubTo_walking, ResetBodyhub, SendGaitCommand, WaitForWalkingDone, WalkTheDistance
import yaml
import time
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys



MAX_Y = 230

SHOW_IMAGE = False

class StairConvert:
    def __init__(self):
        rospy.init_node("stair_convert")
        self.bridge = CvBridge()
        self.__chin_image = None
        self.is_calibrate = len(sys.argv) > 1
        
        rospy.Subscriber("/chin_camera/image", Image, self._chin_callback)



    def walk(self, i, x, y, r):
        if self.is_calibrate:
            return
        if i < 0:
            WalkTheDistance(x, y, r)
        else:
            for _ in range(i):
                SendGaitCommand(x, y, r)


    @property
    def chin_image(self):
        while not rospy.is_shutdown() and self.__chin_image is None:
            continue
        if SHOW_IMAGE:
            cv.imshow("chin_image", self.__chin_image)
            cv.waitKey(1)
        return self.__chin_image
        
    def _get_contours(self):
        hsv = (0, 43, 0), (20, 200, 255)
        binary = cv.cvtColor(self.chin_image, cv.COLOR_BGR2HSV)
        binary = cv.inRange(binary, hsv[0], hsv[1])
        if SHOW_IMAGE:
            cv.imshow("green_binary", binary)
            cv.waitKey(1)
        _, contours, _ = cv.findContours(binary, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        contours = [ cv.approxPolyDP(contour, 10, True) for contour in contours ]

        p = []
        for contour in contours:
            if cv.contourArea(contour) < 1000:
                continue
            center, rect, scope = cv.minAreaRect(contour)
            p.append((center[0], center[1] + min(rect) / 2, scope))

        return p

    def _chin_callback(self, image): 
        image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.__chin_image = image[:350]

    def start(self):
        SetBodyhubTo_walking(2)

        self.walk(6, 0.045, 0, 0)
        self.walk(9, 0, 0, 10)

        WaitForWalkingDone()
        have_p = False
        while not rospy.is_shutdown():
            # 左右找位�?            
            p = self._get_contours()

            if not p:
                if not have_p:
                    self.walk(-1, 0.05, 0, 0)
                continue
            have_p = True

            angle = p[-1][2] 
            symbol = 1 if angle > 0 else -1
            angle = (90 - angle * symbol) * symbol
            print(angle, "angle")
            if abs(angle) > 45:
                angle = (90 - angle * symbol)
            if angle < -5 or angle > 5:
                print("angle", angle)
                self.walk(-1, -0.01, 0, angle)
                continue

            break

        self.walk(12, 0.06, 0, 1)
        self.walk(9, 0, 0, 10)

        WaitForWalkingDone()
        left_max_y = None
        have_p = False


        while not rospy.is_shutdown():
            # 左右找位�?            
            p = self._get_contours()

            if not p:
                if not have_p:
                    self.walk(-1, 0.05, 0, 0)
                continue
            have_p = True

            angle = p[-1][2] 
            symbol = 1 if angle > 0 else -1
            angle = (90 - angle * symbol) * symbol
            
            if abs(angle) > 45:
                angle = (90 - angle * symbol)
            if angle < -5 or angle > 5:
                print("angle", angle)
                self.walk(-1, -0.01, 0, angle)
                continue

            max_y = max(p, key=lambda p: p[1])[1]
            print("max_y", max_y)
            if self.is_calibrate:
                continue

            if max_y < MAX_Y:
                if left_max_y is None:
                    left_max_y = max_y
                else:
                    if left_max_y - 10 > max_y:
                        continue

                distance = (MAX_Y + 30 - max_y) / 1000.0
                self.walk(-1, distance if distance < 0.10 else 0.10, 0, 0)
                continue
            return
            


if __name__ == "__main__":
    StairConvert().start()



