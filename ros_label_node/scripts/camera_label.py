#!/usr/bin/python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ros_label_node.msg import Labels, Label
import cv2
from std_srvs.srv import SetBool

class CameraLabel:
    def __init__(self):
        rospy.init_node('ros_label_node')

        self.camera_rects = []
        self.chin_camera_rects = []

        self.bridge = CvBridge()
        camera_label_topic = "/camera/label/image_raw"
        chin_camera_label_topic = "/chin_camera/label/image_raw"

        self.camera_label_pub = rospy.Publisher(camera_label_topic,
                                                Image, queue_size=1000)
        self.chin_camera_label_pub = rospy.Publisher(chin_camera_label_topic,
                                                     Image, queue_size=1000)

        self.subscribers = []
        
        rospy.Service("switch_label", SetBool, self.switch_service)

        rospy.spin()

    def switch_service(self, req):
        if req.data:
            self.create_subscriber()
        else:
            self.delete_subsciber()
        return [True, ""]

    def create_subscriber(self):
        if len(self.subscribers) != 0:
            return

        camera_topic = "/camera/color/image_raw"
        chin_camera_topic = "/chin_camera/image"
        camera_make_topic = "/camera/make/label"
        sub = rospy.Subscriber(camera_topic,
                               Image,
                               lambda image: self.camera_call_back(image,
                                                                   self.camera_label_pub,
                                                                   self.camera_rects))
        self.subscribers.append(sub)

        sub = rospy.Subscriber(chin_camera_topic,
                         Image,
                         lambda image: self.camera_call_back(image,
                                                             self.chin_camera_label_pub,
                                                             self.chin_camera_rects))
        self.subscribers.append(sub)

        sub = rospy.Subscriber(camera_make_topic,
                         Labels,
                         lambda labels: self.label_call_back(labels, [self.camera_rects, self.chin_camera_rects]))
        self.subscribers.append(sub)
        

    def delete_subsciber(self):
        if len(self.subscribers) == 0:
            return
        for sub in self.subscribers:
            sub.unregister()
        self.subscribers[:] = []

    def camera_call_back(self, image, publisher, rects):
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        for pt1, pt2, color, thickness in rects:
            cv2.rectangle(cv_image, pt1, pt2, color, thickness, cv2.LINE_AA)
        rects[:] = []
        publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    def label_call_back(self, labels, rects):
        rects[labels.camera_type - 1][:] = [
            ((int(label.center_point[0] - label.width / 2),
              int(label.center_point[1] - label.height / 2)),
             (int(label.center_point[0] + label.width / 2),
              int(label.center_point[1] + label.height / 2)),
             (label.border_color[2], label.border_color[1],
              label.border_color[0]),
             label.thickness)
            for label in labels.labels
        ]


if __name__ == "__main__":
    CameraLabel()
