#! /usr/bin/python
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

import recognition

import time
import Queue
import threading

QUEUE_IMG = Queue.Queue(maxsize=1)

STOP_THREAD = False

# Instantiate CvBridge
bridge = CvBridge()


def face_tracking_loop():
    process_this_frame = 0
    time.sleep(3)
    face = recognition.Face()
    face.read_encoding()
    while not STOP_THREAD:
        time.sleep(0.01)

        if not QUEUE_IMG.empty():
            frame = QUEUE_IMG.get()
            QUEUE_IMG.queue.clear()
        else:
            continue

        if process_this_frame > 8:
            start = time.time()
            faces = face.locate(frame)
            print(faces, time.time() - start)
            process_this_frame = 0
        process_this_frame += 1
    print("Thread, signing off")


def image_callback(msg):
    # print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as err:
        print(err)
    else:
        # cv2.imwrite('camera_image.jpeg', cv2_img)
        # cv2.imshow('Video', cv2_img)
        QUEUE_IMG.put(cv2_img, block=True)


def main():
    try:
        rospy.init_node('image_listener')
        image_topic = "/camera/color/image_raw"
        rospy.Subscriber(image_topic, Image, image_callback)

        detect_job = threading.Thread(target=face_tracking_loop)
        detect_job.daemon = True
        detect_job.start()

        rospy.spin()
    except Exception as _:
        print('Terminate all threads!')
        global STOP_THREAD
        STOP_THREAD = True
        detect_job.join()


if __name__ == '__main__':
    main()
