#!/usr/bin/env python

import pyrealsense2 as rs
import numpy as np
import cv2
import rospy 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

if __name__ == '__main__':
    rospy.init_node("realsense_image")
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # Start streaming
    pipeline.start(config)
    rate = rospy.Rate(30)
    imagePub = rospy.Publisher('/realsense_image', Image, queue_size=2)
    bridge = CvBridge()
    while not rospy.is_shutdown():
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.2), cv2.COLORMAP_JET)
        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))
        msg = bridge.cv2_to_imgmsg(images)
        msg.encoding = "bgr8"
        imagePub.publish(msg)
        rate.sleep()
    # Stop streaming
    pipeline.stop()