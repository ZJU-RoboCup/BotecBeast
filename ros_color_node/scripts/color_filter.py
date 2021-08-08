#! /usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import cv2


def read_img_file(fname):
    img = cv2.imread(fname)
    return img


def show_img(img):
    cv2.imshow(img)
    k = cv2.waitKey(0)
    if k == 27:
        cv2.destroyWindow()


def search_rgb(img, color=None):
    return None


def search_hsv(img, color=None):
    return None


def rgb_to_hsv(rgb):
    """convert rgb to hsv value
    """
    rgb_pixel = np.uint8([[rgb]])
    hsv = cv2.cvtColor(rgb_pixel, cv2.COLOR_RGB2HSV)
    return hsv[0][0]


def color_range(hsv, delta):
    """color hsv upper and lower boundary
    """
    hsv_l = [hsv[0] - delta, hsv[1] - delta, hsv[2] - delta]
    hsv_h = [hsv[0] + delta, hsv[1] + delta, hsv[2] + delta]
    for i, v in enumerate(hsv_l):
        hsv_l[i] = hsv_l[i] if hsv_l[i] > 0 else 0
    for i, v in enumerate(hsv_h):
        hsv_h[i] = hsv_h[i] if hsv_h[i] < 255 else 255
    hsv_h[0] = hsv_h[0] if hsv_h[0] < 180 else hsv_h[0]
    return np.array(hsv_l), np.array(hsv_h)


def mask_center(mask):
    """find mask center
    """
    M = cv2.moments(mask)
    if M["m00"] != 0.0:
        c_x = int(M["m10"] / M["m00"])
        c_y = int(M["m01"] / M["m00"])
    else:
        c_x = 0.0
        c_y = 0.0
    return c_x, c_y


def mask_area(mask):
    """calculate mask area size
    """
    mask_size = mask.shape[0] * mask.shape[1]
    target_size = len(np.where(mask > 0)[0])
    return float(target_size) / float(mask_size)


def mask_region(mask, center):
    """left center and right region
    """
    if center[0] == 0.0:
        return 'None'
    location = 'center'
    width, height = mask.shape
    if center[0] < width / 3:
        location = 'left'
    elif center[0] > 2 * width / 3:
        location = 'right'
    else:
        location = 'center'
    return location


def color_filter(img, color, color_type='rgb'):
    """find color
    Arguments:
        img {[str or cv.img]} -- frame of image
        color {[list]} -- rgb or hsv value
    Keyword Arguments:
        color_type {str} -- [description] (default: {'rgb'})
    """
    if type(img) == str:
        img = cv2.imread(img)
    else:
        img = img
    rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    hsv_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)
    hsv = color if color_type == 'hsv' else rgb_to_hsv(color)
    color_lower, color_upper = color_range(hsv, 30)
    mask = cv2.inRange(hsv_img, color_lower, color_upper)
    im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda cnt: cv2.contourArea(cnt), reverse=True)

    center = (0.0, 0.0)
    area = 0.0
    region = 'None'

    if len(contours) > 0:
        box_x, box_y, box_w, box_h = cv2.boundingRect(contours[0])
        tmp_area = float(box_w * box_h) / float(mask.shape[0] * mask.shape[1])
        if tmp_area >= 0.01:
            area = tmp_area
            center = box_x+box_w//2, box_y+box_h//2
            region = mask_region(mask, center)

    return center, area, region

if __name__ == '__main__':
    center, area, region = color_filter('../dataset/cube.png', [2, 54, 123])
    print(center, area, region)
