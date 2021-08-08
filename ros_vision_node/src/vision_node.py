#! /usr/bin/python

import rospy
import cv2
import Queue
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from ros_vision_node.srv import FaceRecognition, FaceVerify

import face

bridge = CvBridge()

def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as err:
        print(err)
    else:
        return cv_image

def handle_face_detect(req):
    try:
        if req.path == "":
            image_topic = "/camera/color/image_raw"
            msg = rospy.wait_for_message(image_topic, Image, 1)
            current_frame = image_callback(msg)
            image_path = '/tmp/temp_image.jpg'
            cv2.imwrite(image_path, current_frame)
        else:
            image_path = req.path
        result = face.face_cognition(image_path)
    except Exception as err:
        rospy.logerr(err)
        return "Error"
    else:
        return result

def handle_face_verify(req):
    result, confidence = face.face_verify(req.faceId1, req.faceId2)

    return result, confidence

def main():
    rospy.init_node('ros_vision_node')

    # provide service for face detect
    face_detect_srv = rospy.Service('ros_vision_node/face_detect', FaceRecognition, handle_face_detect)
    face_verify_srv = rospy.Service('ros_vision_node/face_verify', FaceVerify, handle_face_verify)
    print('ready to process vision request')

    rospy.spin()

if __name__ == '__main__':
    main()