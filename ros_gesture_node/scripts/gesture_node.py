#! /usr/bin/python

import rospy
import cv2
import Queue
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ros_gesture_node.srv import GestureRecognition
import gestures
bridge = CvBridge()

def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as err:
        print(err)
    else:
        return cv_image

def handle_gesture_detect(req):
    try:
        if req.path == "":
            image_topic = "/camera/color/image_raw"
            msg = rospy.wait_for_message(image_topic, Image, 1)
            current_frame = image_callback(msg)
            image_path = '/tmp/temp_gesture.jpg'
            cv2.imwrite(image_path, current_frame)
        else:
            image_path = req.path
        result = gestures.get_gesture(image_path)
    except Exception as err:
        rospy.logerr(err)
        return "Error"
    else:
        return result


def main():
    rospy.init_node('ros_gesture_node')

    # provide service for gesture detect
    gesture_detect_srv = rospy.Service('ros_gesture_node/gesture_detect', GestureRecognition, handle_gesture_detect)
    print('ready to process vision request')

    rospy.spin()

if __name__ == '__main__':
    main()