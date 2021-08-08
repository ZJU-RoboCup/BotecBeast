import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Pendulum:
    def __init__(self):
        self.bridge = CvBridge()
        self._image = None
        self._chin_image = None
        rospy.Subscriber("/camera/rgb/image_raw", Image, self._callback)
        rospy.Subscriber("/chin_camera/image", Image, self._chin_callback)


    @property
    def image(self):
        while not rospy.is_shutdown() and self._image is None:
            continue
        return self._image

    @property
    def chin_image(self):
        while not rospy.is_shutdown() and self._chin_image is None:
            continue
        return self._chin_image

    def get_mean(self):
        hsv = (0, 43, 46), (20, 200, 200)
        binary = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)
        binary = cv.inRange(binary, hsv[0], hsv[1])

        _, contours, _ = cv.findContours(binary, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        contours = [ cv.approxPolyDP(contour, 10, True) for contour in contours ]

        mean = None
        for contour in contours:
            if cv.contourArea(contour) < 1000:
                continue
            numbers_up = [ n[0][0] for n in contour if n[0][1] < 10  ]
            numbers_down = [ n[0][0] for n in contour if n[0][1] > 470 ]

            if numbers_up and numbers_down:
                mean = sum(numbers_up) / len(numbers_up)
                break

        return mean

    def get_gray(self):
        hsv = (0, 43, 0), (20, 200, 255)
        binary = cv.cvtColor(self.chin_image, cv.COLOR_BGR2HSV)
        binary = cv.inRange(binary, hsv[0], hsv[1])

        _, contours, _ = cv.findContours(binary, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        contours = [ cv.approxPolyDP(contour, 10, True) for contour in contours ]

        center, rect, scope = None, None, None
        not_none = True
        for contour in contours:
            if cv.contourArea(contour) < 3000:
                continue
            # print(contour)
            max_x, max_y = max(contour, key=lambda a: a[0][0])[0]
            numbers_up = [ n[0][0] for n in contour if n[0][1] < 10  ]
            numbers_down = [ n[0][0] for n in contour if n[0][1] > 220 ]
            if len(contour) < 6:
                max_y = None
            if len(numbers_up) == 2 and abs(numbers_up[0] - numbers_up[1]) < 80:
                # is up
                binary = binary[:50]
            elif len(numbers_down) == 2 and abs(numbers_down[0] - numbers_down[1]) < 80:
                # is down
                binary = binary[180:]
            else:
                not_none = False
            
            _, _contours, _ = cv.findContours(binary, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            _contours = [ cv.approxPolyDP(_contour, 10, True) for _contour in _contours ]
            for _contour in _contours:
                if cv.contourArea(_contour) < 1000:
                    continue
                contour = _contour
                break
            
            center, rect, scope = cv.minAreaRect(contour)
            break

        cv.imshow("binary", binary)
        cv.imshow("img", self.chin_image)
        cv.waitKey(1)
        if not center or not not_none:
            return None, None 
        return (center[0], max_y), scope

    def _callback(self, image): 
        image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self._image = image
    
    def _chin_callback(self, image):
        image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self._chin_image = image[:230]


        
if __name__ == "__main__":
    rospy.init_node("pendulum")

    pen =  Pendulum()

    while not rospy.is_shutdown():
        cv.waitKey(1)
    
        center, x = pen.get_gray()
        if center:
            print(center, x)
