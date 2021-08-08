#! /usr/bin/python
import sys
import rospy
import time
from ros_dance_node.srv import DanceAction
from bodyhub.srv import SrvState
from lejufunc.ServoJointTrajectory import ServoJointTrajectoryC
SERVO = ServoJointTrajectoryC()

act_frames = [
    ([0,0,0,0,0,0,0,0,0,0,0,0,0,-61,-18,0,61,18,0,0,0,0],1000,100),
    ([0,0,0,0,0,0,0,0,0,0,0,0,0,-49,-24,0,49,24,0,0,0,0],2000,0),
    ([0,0,0,0,0,0,0,0,0,0,0,0,76,-49,-24,-76,49,24,0,0,0,18],2000,0),
    ([0,0,0,0,0,0,0,0,0,0,0,0,40,-49,-24,-40,49,24,0,0,0,30],2000,0),
    ([0,0,0,0,0,0,0,0,0,0,0,0,60,-12,-24,-60,12,24,0,0,0,0],2000,0),
    ([0,0,0,0,0,0,0,0,0,0,0,0,60,-67,-24,-60,67,24,0,0,0,0],2000,0),
    ([0,0,0,0,0,0,0,0,0,0,0,0,60,-67,-24,-60,67,24,0,0,0,18],2000,0),
    ([0,0,0,0,0,0,0,0,0,0,0,0,0,-49,-24,0,49,24,0,0,0,0],2000,0),
    ([0,0,0,0,0,0,0,0,0,0,0,0,0,-61,-18,0,61,18,0,0,0,0],3000,100)
]

def action_client():
    """
    :param:
    """
    rospy.wait_for_service("/MediumSize/BodyHub/StateJump")
    client = rospy.ServiceProxy("/MediumSize/BodyHub/StateJump", SrvState)
    client(2, "setStatus")

def set_all_servo(angles, time):
    SERVO.MotoJointTransfer(angles, time)
    SERVO.MotoWait()

def action(act_frames):
    """
    act_frames: a frame list
    """
    for frame in act_frames:
        if not rospy.is_shutdown():
            servo_value = frame[0]
            act_time = frame[1]
            interval = frame[2] / 1000.0
            set_all_servo(servo_value, act_time)
            time.sleep(interval)
        else:
            break

def handle_dance_action(req):
    try:
        action_client()
        action(act_frames)
    except Exception as err:
        rospy.logerr(err)
        return False
    else:
        return True


def main():
    rospy.init_node('ros_dance_node')
    dance_srv = rospy.Service('ros_dance_node/dance_action', DanceAction, handle_dance_action)
    rospy.spin()

if __name__ == '__main__':
    main()