import numpy as np
import cv2

# Load an color image in grayscale
img = cv2.imread('~/robot_ros_application/faces/obama.jpg', 0)
cv2.imshow('image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
