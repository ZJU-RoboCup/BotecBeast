#!/usr/bin/env python
#coding=utf-8
import rospy
from bodyhub.srv import SrvState
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from client_logger import serror
from utils import judge_range_number
import sys

GAIT_RANGE = 0.10


class WalkTransfer:
    def __init__(self):
        """
        """
        self.walking = 1.0
        self.walking_done = 0.0
        self.status_data = self.walking_done
        self.walk_running = False
        self.walkingPub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=2)
        self.walkingStatusSub = rospy.Subscriber('/MediumSize/BodyHub/WalkingStatus', Float64,
                                                 self.walkingStatusCallback)

    def walkingClient(self, walkstate):
        """
        :param walkstate: "walking" or "stop"
        :return:
        """
        rospy.wait_for_service("/MediumSize/BodyHub/StateJump")
        client = rospy.ServiceProxy("/MediumSize/BodyHub/StateJump", SrvState)
        client(2, walkstate)

    def walkingPublish(self, array, stepnum):
        """
        :param array: walk array
        :param stepnum: int num
        :return:
        """
        self.walk_running = True
        self.status_data = self.walking
        for i in range(stepnum):
            if rospy.wait_for_message("/requestGaitCommand", Bool):
                self.walkingPub.publish(data=array)

    def walkingStatusCallback(self, status_data):
        """
        :param status_data: float64
        :return:
        """
        self.status_data = status_data.data
        rospy.loginfo("receive walkingstatus: %f", status_data.data)

    def waitForWalkingDone(self):
        """
        :return:
        """
        ros_rate = rospy.Rate(100)
        while self.walk_running:
            if self.status_data == self.walking_done:
                self.walk_running = False
            ros_rate.sleep()


def slow_walk(direction, stepnum):
    """
    :param direction: "forward" or "backward"
    :param stepnum: int num
    :return:
    """
    try:
        stepnum = int(judge_range_number(stepnum, 0, 255))
    except Exception as err:
        lineno = sys._getframe().f_back.f_lineno
        co_name = sys._getframe().f_code.co_name
        serror("步数{}, line {}, in {}".format(err.message, lineno, co_name))
        return

    array = [0.0, 0.0, 0.0]
    if direction == "forward":
        array[0] = GAIT_RANGE
    elif direction == "backward":
        array[0] = -0.5 * GAIT_RANGE
    else:
        rospy.logerr("error walk direction")

    Walker = WalkTransfer()
    Walker.walkingClient("walking")
    if stepnum == 0:
        Walker.walkingClient("stop")
        return
    Walker.walkingPublish(array, stepnum)
    Walker.waitForWalkingDone()
    Walker.walkingClient("stop")


if __name__ == '__main__':
    rospy.init_node("walk_test", anonymous=True)

    try:
        slow_walk("forward", 10)
    except Exception as err:
        print(err)
