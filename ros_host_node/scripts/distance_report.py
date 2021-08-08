#!/usr/bin/python
# coding=utf-8
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Float32
ROI = (100, 100)
distancePub = rospy.Publisher('/current_distance', Float32, queue_size=2)

def callback(image):
    cv_image = bridge.imgmsg_to_cv2(image, "16UC1")

    cv_image = np.array(cv_image)
    height, width = cv_image.shape
    roi_image = cv_image[height / 2 - ROI[1] / 2: height / 2 + ROI[1] / 2,
                width / 2 - ROI[0] / 2: width / 2 + ROI[0] / 2]
    mean_distance = roi_image.mean()
    return mean_distance


if __name__ == "__main__":
    rospy.init_node('distance_report')
    bridge = CvBridge()
    topic = "/camera/depth/image_rect_raw"
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        means_distance = []
        for _ in range(5):
            means_distance.append(
                callback(rospy.wait_for_message(topic, Image)))
        distance = sum(means_distance) / len(means_distance)
        rospy.loginfo("current distance:%f mm",distance)
        distancePub.publish(distance)
        rate.sleep()

