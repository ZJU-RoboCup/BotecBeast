#!/usr/bin/env python
# coding=utf-8

import numpy as np
import cv2
import tensorflow as tf
from keras.models import model_from_json
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from lejufunc import client_action
import rospkg
from motion.motionControl import *

def get_model():
    pkg_path = rospkg.RosPack().get_path("ros_host_node")
    with open(pkg_path + "/scripts/model.json", 'r') as json_file:
        loaded_model_json = json_file.read()
        loaded_model = model_from_json(loaded_model_json)
        loaded_model.load_weights(pkg_path + "/scripts/model.h5")
        return loaded_model


def get_contour_thresh(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray,
                              80,
                              255,
                              cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(thresh,
                                   cv2.RETR_TREE,
                                   cv2.CHAIN_APPROX_SIMPLE)[-2:]
    return contours, thresh


def camera_call_back(model, img):
    cv_image = CvBridge().imgmsg_to_cv2(img, "bgr8")
    number_recognition(model, cv_image)


def right_hand_up():
    frames = [([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -61, -18, 0, 61, 18, 0, 0, 0, 0], 1000, 0),
              ([-4, 0, 0, -2, -4, 0, -4, 0, 0, 2, 4, 0, -4, 28, 0, 0, 43, 12, 0, 0, -3, -9], 1000, 1000),
              ([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -61, -18, 0, 61, 18, 0, 0, 0, 0], 1500, 0)]
    client_action.action(frames)


def left_hand_up():
    frames = [([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -61, -18, 0, 61, 18, 0, 0, 0, 0], 1000, 0),
              ([4, 0, 0, -2, -4, 0, 4, 0, 0, 2, 4, 0, 0, -43, -12, 4, -28, 0, 0, 0, -3, -9], 1000, 1000),
              ([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -61, -18, 0, 61, 18, 0, 0, 0, 0], 1500, 0)]
    client_action.action(frames)


num2_count = 0
num3_count = 0


def number_recognition(model, img):
    actual_height, actual_width, _ = img.shape
    identify_height, identify_width = (300, 300)
    pt1 = ((actual_width - identify_width) / 2,
           (actual_height - identify_height) / 2)
    pt2 = ((actual_width + identify_width) / 2,
           (actual_height + identify_height) / 2)
    cv2.rectangle(img, pt1, pt2, (0, 255, 0), 2)

    contours, thresh = get_contour_thresh(img[pt1[1]:pt1[1] + identify_height,
                                              pt1[0]:pt1[0] + identify_width])
    if len(contours) > 0:
        contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(contour)
        number_img = thresh[y:y + h, x:x + w]
        number_img = cv2.copyMakeBorder(number_img,
                                        50, 50, 50, 50,
                                        cv2.BORDER_CONSTANT,
                                        None, 0)
        number_img = cv2.resize(number_img, (28, 28))
        cv2.imshow("new_img", number_img)

        number_img = number_img.flatten()
        number_img = number_img.reshape(1, 28, 28, 1).astype('float32') / 255
        ans = model.predict(number_img)
        ans = ans.tolist()
        num = ans[0].index(max(ans[0]))
        x += pt1[0]
        y += pt1[1]
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        if ans[0][num] > 0.8:
            global num2_count, num3_count
            if num == 2:
                num2_count += 1
                if num2_count > 10:
                    num2_count = 0
                    print("识别到 2 了, 举右手")
                    right_hand_up()
            elif num == 3:
                num3_count += 1
                if num3_count > 10:
                    num3_count = 0
                    print("识别到 3 了, 举左手")
                    left_hand_up()
            else:
                num2_count = 0
                num3_count = 0

    cv2.imshow("img", img)
    cv2.imshow("thresh", thresh)
    cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("number_recognition")
    camera_topic = "/camera/color/image_raw"
    SetBodyhubTo_setStatus(2)
    model = get_model()
    while not rospy.is_shutdown():
        image = rospy.wait_for_message(camera_topic, Image)
        camera_call_back(model, image)
