#!/usr/bin/python
import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image


def callback(image):
    cv_image = bridge.imgmsg_to_cv2(image, "bgr8")
    cv2.imshow("image", cv_image)
    cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('pc_image_view')
    camera_topic = "/camera/color/image_raw"
    bridge = CvBridge()
    rospy.Subscriber(camera_topic, Image, callback)
    rospy.spin()
