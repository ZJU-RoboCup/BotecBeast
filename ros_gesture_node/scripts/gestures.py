#! /usr/bin/python
# coding=utf-8
import urllib2
import urllib
import time
import json
import os
import cv2 as cv

"""
图片要求
图片格式：JPG(JPEG)，PNG
图片文件大小：2MB

"""

config_path = '/home/lemon/.lejuconfig/config.json'
with open (config_path,'r') as file:
    Facepp_config = json.load(file)
    key = str(Facepp_config['gesture_key'])
    secret =str(Facepp_config['gesture_secret'])
assert key
assert secret


http_url = 'https://api-cn.faceplusplus.com/humanbodypp/v1/gesture'
boundary = '----------%s' % hex(int(time.time() * 1000))

gesturevalue = {
    "heart_a":"比心A",
    "heart_b":"比心B",
    "heart_c":"比心C",
    "heart_d":"比心D",
    "ok":"OK",
    "hand_open":"手张开",
    "thumb_up":u"点赞",
    "thumb_down":"差评",
    "rock":"ROCK",
    "namaste":"合十",
    "palm_up":"手心向上",
    "fist":"握拳",
    "index_finger_up":"食指朝上",
    "double_finger_up":"双指场上",
    "victory":"胜利",
    "big_v":"大V",
    "phonecall":"打电话",
    "beg":"作揖",
    "thanks":"感谢"
}


def get_img(img_path):
    '''

    :param img_path: images path
    :return: resize images path
    '''
    FILESIZE = 2 * 1024 * 1024
    fileSize = os.path.getsize(img_path)
    img = cv.imread(img_path)
    if fileSize < FILESIZE:
        img_type = os.path.splitext(img_path)[1]
        if img_type != '.png' and img_type != '.jpg' and img_type != '.jpeg':
            img_path = img_path.replace(img_type, ".jpg")
            cv.imwrite(img_path, img)
        return img_path
    else:
        print("the picture is bigger than 2M ...")
        rate = FILESIZE / fileSize
        img = cv.resize(img, None,fx=rate,fy=rate, interpolation=cv.INTER_CUBIC)
        img_type = os.path.splitext(img_path)[1]
        if img_type != '.png' and img_type != '.jpg' and img_type != '.jpeg':
            img_path = img_path.replace(img_type, ".jpg")
        cv.imwrite(img_path, img)
        return img_path



def get_the_data(filepath):
    '''

    :param filepath: appropriate images path
    :return: http request body data
    '''
    data = []
    data.append('--%s' % boundary)
    data.append('Content-Disposition: form-data; name="%s"\r\n' % 'api_key')
    data.append(key)
    data.append('--%s' % boundary)
    data.append('Content-Disposition: form-data; name="%s"\r\n' % 'api_secret')
    data.append(secret)
    data.append('--%s' % boundary)
    fr=open(filepath,'rb')
    data.append('Content-Disposition: form-data; name="%s"; filename=" "' % 'image_file')
    data.append('Content-Type: %s\r\n' % 'application/octet-stream')
    data.append(fr.read())
    fr.close()
    data.append('--%s' % boundary)
    data.append('Content-Disposition: form-data; name="%s"\r\n' % 'return_landmark')
    data.append('1')
    data.append('--%s--\r\n' % boundary)

    http_body='\r\n'.join(data)
    return http_body

def http_request(img_path):
    '''

    :param img_path: appropriate images path
    :return: http request result
    '''
    filepath = r''+get_img(img_path)
    http_body = get_the_data(filepath)

    req=urllib2.Request(http_url)
    req.add_header('Content-Type', 'multipart/form-data; boundary=%s' % boundary)
    req.add_data(http_body)

    try:
        resp = urllib2.urlopen(req, timeout=5)
	qrcont=resp.read()
	result = json.loads(qrcont)
	return result

    except urllib2.HTTPError as e:
        print(e.read())


def get_gesture(img_path):
    '''

    :param img_path: appropriate images path
    :return: gesture of the images
    '''
    result = http_request(img_path)
    hand = result['hands'][0]
    gestures = hand['gesture']
    gesture = str(max(gestures, key=gestures.get))
    return gesture


if __name__ == '__main__':
    try:
        img_path = '../images/timg.jpeg'
        gesture = get_gesture(img_path)
        print(gesture)
    except Exception as e:
        print(e)
