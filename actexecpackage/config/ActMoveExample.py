#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import array

from ServoJointTrajectory import ServoJointTrajectoryC

def end_it():
  print('END IT')

if __name__ == '__main__':
  # example send array
  arraySendA = [-45, -45, -45, -45, -45, -45, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]#double angle
  arraySendB = [45, 45, 45, 45, 45, 45, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]#double angle
  arraySendC = [-45, -45, -45, -45, -45, -45, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]#double angle
  arraySendD = [45, 45, 45, 45, 45, 45, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]#double angle

  headarraySendA = [90.0,90.0]
  headarraySendB = [0.0,0.0]

  try:
      # 初始化ros节点
      rospy.init_node('ActMoveExampleNode', anonymous=True)
      rospy.loginfo("Starting ActMoveExampleNode node")

      servoDataSend = ServoJointTrajectoryC(end_it)

      ros_rate = rospy.Rate(10)
      while( servoDataSend.MotoJointPub.get_num_connections() <=0):
        rospy.loginfo("wait topic connect")
        ros_rate.sleep()
      while( servoDataSend.HeadOnlyJointPub.get_num_connections() <=0):
        rospy.loginfo("wait topic connect")
        ros_rate.sleep()

      while not rospy.is_shutdown():
        durationSet = 100  # (n*20ms)

        servoDataSend.MotoJointTransfer(arraySendC, 500)
        servoDataSend.MotoWait()
        servoDataSend.MotoJointTransfer(arraySendD, 500)
        servoDataSend.MotoWait()

        servoDataSend.HeadJointTransfer(headarraySendA, durationSet)
        servoDataSend.MotoWait()
        servoDataSend.HeadJointTransfer(headarraySendB)
        servoDataSend.MotoWait()


        ros_rate.sleep()
        
  except KeyboardInterrupt:
    print(("Shutting down ServoJointTrajectory node."))
