#! /usr/bin/python
import cv2
import Queue
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ros_face_node.srv import FaceDetect
import rospkg

Face_center_x = 160
Face_center_y = 120
Face_area = 0
QUEUE_IMG = Queue.Queue(maxsize=1)
bridge = CvBridge()
faceadd = rospkg.RosPack().get_path("ros_actions_node") + "/scripts/tracking/haarcascade_frontalface_alt2.xml"
face_detector = cv2.CascadeClassifier(faceadd)


def face_size(face):
    x, y, w, h = face
    return w * h


def face_filter(face_list):
    face_size_list = map(face_size, face_list)
    target_index = face_size_list.index(max(face_size_list))
    return face_list[target_index]


def detectFace(frame):
    """Detect face

    :param frame:
    :return:
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    face_locations = face_detector.detectMultiScale(
        gray, scaleFactor=1.1, minNeighbors=3, minSize=(frame.shape[1] / 12, frame.shape[0] / 12),
        maxSize=(2 * frame.shape[1] / 3, 2 * frame.shape[1] / 3))
    if len(face_locations) <= 0:
        result = Face_center_x, Face_center_y, Face_area
    else:
        face = face_filter(face_locations)
        result = face[0] + face[2] / 2, face[1] + face[3] / 2, face[2] * face[3]
    return result


def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as err:
        print(err)
    else:
        cv_image = cv2.resize(cv_image, (0, 0), fx=0.5, fy=0.5)
        return cv_image


def handle_face_detect(req):
    try:
        image_topic = "/camera/color/image_raw"
        msg = rospy.wait_for_message(image_topic, Image, 1)
        current_frame = image_callback(msg)
        result = detectFace(current_frame)
    except Exception as err:
        rospy.logerr(err)
        return [Face_center_x, Face_center_y, Face_area]
    else:
        return [result]


def main():
    rospy.init_node('ros_face_node')

    face_detect_srv = rospy.Service('ros_face_node/face_detect', FaceDetect, handle_face_detect)
    rospy.spin()


if __name__ == '__main__':
    main()
