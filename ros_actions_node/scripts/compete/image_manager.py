import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import math



SHOW_IMAGE = True



class ImageManager:
    def __init__(self):
        self.bridge = CvBridge()
        self.__chin_image = None
        self.__eye_image = None
        self.__eye_depth = None
        
        rospy.Subscriber("/chin_camera/image", Image, self._chin_callback)
        rospy.Subscriber("/camera/color/image_raw", Image, self._eye_callback)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self._eye_depth_callback)
        
    @property
    def chin_image(self):
        while not rospy.is_shutdown() and self.__chin_image is None:
            continue
        if SHOW_IMAGE:
            cv.imshow("chin_image", self.__chin_image)
            cv.waitKey(1)
        return self.__chin_image

    @property
    def eye_image(self):
        while not rospy.is_shutdown() and self.__eye_image is None:
            continue
        return self.__eye_image
    
    @property
    def eye_depth(self):
        while not rospy.is_shutdown() and self.__eye_depth is None:
            continue
        return self.__eye_depth

    def _chin_callback(self, image): 
        image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        src_point = np.float32(((190, 0), (160, 300), (522, 0), (575, 300)))
        dst_point = np.float32(((160, 0), (160, 300), (575, 0), (575, 300)))
        matrix = cv.getPerspectiveTransform(src_point, dst_point)
        image = cv.warpPerspective(image, matrix, image.shape[1::-1])
        self.__chin_image = image[:300]

    def _eye_callback(self, image):
        image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.__eye_image = image    

    def _eye_depth_callback(self, image):
        image = self.bridge.imgmsg_to_cv2(image, "16UC1")
        self.__eye_depth = image

    def get_wall_scope_x(self):
        wall_hsv = (0, 0, 0), (180, 255, 46)
        binary = cv.cvtColor(self.chin_image, cv.COLOR_BGR2HSV)
        wall_binary = cv.inRange(binary, wall_hsv[0], wall_hsv[1])
        if SHOW_IMAGE:
            cv.imshow("wall_binary", wall_binary)
            cv.waitKey(1)
        _, contours, _ = cv.findContours(wall_binary, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        contours = [ cv.approxPolyDP(contour, 10, True) for contour in contours if len(cv.approxPolyDP(contour, 10, True)) == 4 ]
        if len(contours) == 0:
            return
        contour = contours[0]
        points = [ point[0] for point in contour ]
        p1 = points[0] - points[1]
        width = math.hypot(p1[0], p1[1])
        p2 = points[0] - points[3]
        height = math.hypot(p2[0], p2[1])
        if width > height:
            scope = math.atan(p1[1] * 1.0 / p1[0] ) / math.pi * 180 if p1[0] != 0 else 90
        else:
            scope = math.atan(p2[1] * 1.0 / p2[0] ) / math.pi * 180 if p2[0] != 0 else 90
        max_x = max(points, key=lambda n: n[0])[0]
        min_x = min(points, key=lambda n: n[0])[0]
        return scope, (max_x + min_x) / 2

    def _get_green_contours(self):
        green_hsv = (40, 85, 46), (70, 255, 255)
        # green_hsv = (35, 43, 46), (77, 255, 255)
        binary = cv.cvtColor(self.chin_image, cv.COLOR_BGR2HSV)
        green_binary = cv.inRange(binary, green_hsv[0], green_hsv[1])
        if SHOW_IMAGE:
            cv.imshow("green_binary", green_binary)
            cv.waitKey(1)
        _, contours, _ = cv.findContours(green_binary, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        contours = [ cv.approxPolyDP(contour, 10, True) for contour in contours ]
        return contours

    def get_green_scope_x(self):
        contours = self._get_green_contours()
        if not contours:
            return
        contour = max(contours, key=cv.contourArea)
        image = self.chin_image.copy()
        cv.drawContours(image, [contour], -1, (255, 0, 0), 3)
        if SHOW_IMAGE:
            cv.imshow("img_copy", image)
            cv.waitKey(1)

        points = [ point[0].tolist() for point in contour ]
        if len(points) < 2:
            return
        p1 = min(points, key=lambda p: p[0])
        points.remove(p1)
        p2 = min(points, key=lambda p: p[0])
        scope = (math.atan((p1[1] - p2[1]) * 1.0 / (p1[0] - p2[0])) / math.pi * 180) if  (p1[0] - p2[0]) != 0 else 90
        return scope, (p1[0] + p2[0]) / 2, min(p1[1], p2[1])

    def get_step_scope_y(self):
        contours = self._get_green_contours()
        if len(contours) == 0:
            return
        contour = max(contours, key=cv.contourArea)
        if cv.contourArea(contour) < 1000:
            return

        center, rect, scope = cv.minAreaRect(contour)

        return scope, center[1] + min(rect) / 2

    def get_center_circle(self, form, area=None):
        if form == "ball":
            hsvs = (((0, 43, 46), (10, 255, 255)), ((156, 43, 46), (180, 255, 255)))
        elif form == "hole":
            hsvs = (((0, 0, 0), (180, 255, 46)), )

        hsv_image = cv.cvtColor(self.chin_image, cv.COLOR_BGR2HSV)
        hsv_binaries = np.zeros((hsv_image.shape[0], hsv_image.shape[1]), dtype=hsv_image.dtype)
        for hsv in hsvs:
            hsv_binaries += cv.inRange(hsv_image, hsv[0], hsv[1])
        if SHOW_IMAGE:
            cv.imshow(form, hsv_binaries)
            cv.waitKey(1)
            
        _, contours, _ = cv.findContours(hsv_binaries, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        if not contours:
            return
        contour = max(contours, key=cv.contourArea)
        if area and cv.contourArea(contour) <= area:
            return
        m = cv.moments(contour)
        if m['m00'] == 0:
            return
        return (int(m['m10'] / m['m00']), int(m['m01'] / m['m00']))


    def get_landmine_params(self):
        wall = None    # wall = wall_scope, wall_x

        landmine = []  # landmine center points

        black_hsv = (0, 0, 0), (180, 255, 46)
        binary = cv.cvtColor(self.chin_image, cv.COLOR_BGR2HSV)
        black_binary = cv.inRange(binary, black_hsv[0], black_hsv[1])
        if SHOW_IMAGE:
            cv.imshow("black_binary", black_binary)
            cv.waitKey(1)

        _, contours, _ = cv.findContours(black_binary, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        contours = [ cv.approxPolyDP(contour, 10, True) for contour in contours if len(cv.approxPolyDP(contour, 10, True)) >= 4 ]

        for contour in contours:
            print(cv.contourArea(contour))
            if cv.contourArea(contour) < 1000:
                continue
            center, rect, scope = cv.minAreaRect(contour)

            if abs(rect[0] - rect[1]) > 50: # wall
                print("append wall")
                wall = scope, center[0]
            else:
                print("append land")
                landmine.append(center)

        return wall, landmine
    
    def get_orange_params(self):
        orange_hsv = (0, 43, 46), (20, 200, 200)
        binary = cv.cvtColor(self.chin_image, cv.COLOR_BGR2HSV)
        orange_binary = cv.inRange(binary, orange_hsv[0], orange_hsv[1])
        if SHOW_IMAGE:
            cv.imshow("orange_binary", orange_binary)
            cv.waitKey(1)
        _, contours, _ = cv.findContours(orange_binary, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return

        contour = max(contours, key=cv.contourArea)

        if cv.contourArea(contour) < 100:
            return

        center, rect, scope = cv.minAreaRect(contour)
        return center, rect, scope


    def get_blue_distance(self):
        blue_hsv = (95, 65, 120), (124, 100, 180)
        blue = cv.cvtColor(self.eye_image, cv.COLOR_BGR2HSV)
        blue_binary = cv.inRange(blue, blue_hsv[0], blue_hsv[1])
        image = self.eye_depth[blue_binary == 255]
        image = image[image != 0]
        image = image[image < 800]
        return np.sum(image) / image.size


if __name__ == "__main__":
    rospy.init_node("image_manager")
    manager = ImageManager()
    while not rospy.is_shutdown():
        # print(manager.get_wall_scpoe_x(), 2)
        # print(manager.get_green_scope_x(), 1)
        cv.imshow("chin", manager.chin_image)
        # cv.imshow("eye", manager.eye_image)
        cv.waitKey(1)
        rospy.spin()