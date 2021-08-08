import rospy
from ros_label_node.msg import Label, Labels

CAMERA = "camera"
CHIN_CAMERA = "chin_camera"


class LabelCon:
    def __init__(self):
        self.camera_pub = rospy.Publisher(
            "/camera/make/label", Labels, queue_size=1000)
        self.camera_labels = Labels()
        self.camera_labels.camera_type = Labels.eye
        self.chin_camera_labels = Labels()
        self.chin_camera_labels.camera_type = Labels.chin

    def set_labels(self, name, labels):
        for label in labels:
            _label = Label()
            _label.border_color, _label.center_point, _label.width, _label.height, _label.thickness = label
            if name == CAMERA:
                self.camera_labels.labels.append(_label)
            elif name == CHIN_CAMERA:
                self.chin_camera_labels.labels.append(_label)
        self.publish(name)

    def publish(self, name):
        if name == CAMERA:
            self.camera_pub.publish(self.camera_labels)
            self.camera_labels.labels[:] = []
        elif name == CHIN_CAMERA:
            self.camera_pub.publish(self.chin_camera_labels)
            self.chin_camera_labels.labels[:] = []

label_con = LabelCon()

def set_camera_label(color, center_point, width, height, thickness=2):
    label_con.set_labels(CAMERA, [(color, center_point, width, height, thickness)])

def set_camera_labels(labels):
    label_con.set_labels(CAMERA, labels)

def set_chin_camera_label(color, center_point, width, height, thickness=2):
    label_con.set_labels(CHIN_CAMERA, [(color, center_point, width, height, thickness)])

def set_chin_camera_labels(labels):
    label_con.set_labels(CHIN_CAMERA, labels)


