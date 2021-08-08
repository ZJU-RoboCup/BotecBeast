#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import cv2
import numpy as np 

class ColorObject:
    def __init__(self,lower,upper,cName='none'):
        self.coLowerColor = lower
        self.coUpperColor = upper
        self.coResult = {'find':False, 'name':cName}

    def detection(self,image):
        blurred = cv2.GaussianBlur(image, (5, 5), 0)  # 高斯模糊
        hsvImg = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # 转换颜色空间到HSV
        mask = cv2.inRange(hsvImg, self.coLowerColor, self.coUpperColor) # 对图片进行二值化处理
        mask = cv2.dilate(mask, None, iterations=2) # 膨胀操作
        mask = cv2.erode(mask, None, iterations=2) # 腐蚀操作
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  # 寻找图中轮廓

        self.coResult['find'] = False
        if len(contours) > 0: # 如果存在至少一个轮廓则进行如下操作
            c = max(contours, key=cv2.contourArea) # 找到面积最大的轮廓
            self.coResult['boundingR'] = cv2.boundingRect(c) #x,y,w,h
            if self.coResult['boundingR'][2] > 5 and self.coResult['boundingR'][3] > 5:
                M = cv2.moments(c)
                self.coResult['Cx'] = int(M['m10'] / M['m00'])
                self.coResult['Cy'] = int(M['m01'] / M['m00'])
                self.coResult['contour'] = c
                self.coResult['find'] = True

        return self.coResult

def putVisualization(image, result):
    if result['find'] == True:
        x1, y1 = result['boundingR'][0]+result['boundingR'][2], result['boundingR'][1]+result['boundingR'][3]
        cv2.rectangle(image, (result['boundingR'][0],result['boundingR'][1]), (x1, y1), (0, 0, 255), 1)
        cv2.circle(image, center=(result['Cx'], result['Cy']), radius=1, color=(0, 0, 255), thickness=-1)
        if result['name'] != 'none':
            cv2.putText(image, result['name'], (result['boundingR'][0], result['boundingR'][1]-2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

def putTextInfo(img,fps,time):
    x, y = 10, 20
    text = 'fps: ' + '%5.2f'%fps
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    y = y + 30
    text = 'time: '+'%5.2f'%(time)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

if __name__ == '__manin__':
    # HSV阈值
    lowerOrange = np.array([15, 100, 100])
    upperOrange = np.array([25, 255, 255])
    lowerCyan = np.array([80, 100, 100])
    upperCyan = np.array([95, 255, 255])
    lowerGreen = np.array([35, 100, 100])
    upperGreen = np.array([75, 255, 255])
    lowerRed = np.array([0, 224, 96])
    upperRed = np.array([16, 255, 240])
    # creat object
    ball = ColorObject(lowerOrange, upperOrange)
