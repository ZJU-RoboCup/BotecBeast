#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os 
import threading
import signal

import rospy
import array
import numpy

from bodyhub.msg import JointControlPoint, ServoPositionAngle
from bodyhub.srv import *   # for SrvState.srv  SrvServoAllRead.srv
from std_msgs.msg import UInt16

class ServoJointTrajectoryC():
  """ 发送舵机运行序列数据 """

  def __init__(self, release_func):

    # 节点运行状态
    self.StateStyle = {
      'init' : 20,
      'preReady' : 21,
      'running' : 23,
      'pause' : 24,
      'stoping' : 25,
      'error' : 26,
    }

    self.StateEnumEnding = 97

    self.motoWaitFlag = 0 # 舵机运动结束 置0
    self.controlID = 6
    self.presentPosition = []

    self.EndingOrder = 0  # 终止指令
    self.killpid = os.getpid()

    # run multithreading
    detect_terminal = threading.Thread(target=self.terminalScript, args=(release_func,))
    detect_terminal.daemon = True
    detect_terminal.start()

    # Publisher Subscriber
    self.MotoJointPub =rospy.Publisher('MediumSize/BodyHub/MotoPosition', JointControlPoint, queue_size=100)           #定义一个话题发布者，发布数据格式为JointControlPoint
    self.HeadOnlyJointPub =rospy.Publisher('MediumSize/BodyHub/HeadPosition', JointControlPoint, queue_size=100)       #定义一个话题发布者，发布数据格式为JointControlPoint
    self.BodyHubStatusSub =rospy.Subscriber('MediumSize/BodyHub/Status', UInt16, self.BodyHubStatusCallback)
    self.ServoPositionSub =rospy.Subscriber('MediumSize/BodyHub/ServoPositions', ServoPositionAngle, self.ServoPositionCallback)
    self.EdningExecSrv = rospy.Service('MediumSize/ActPackageExec/EndingExec', SrvTLSuint, self.EndingExecCallback)

    rospy.wait_for_service('MediumSize/BodyHub/GetJointAngle')
    try:
      GetJointAngleClient = rospy.ServiceProxy('MediumSize/BodyHub/GetJointAngle', SrvServoAllRead)
      resData = GetJointAngleClient("[1,1]",2)
      for i in range(20):
        self.presentPosition.append(resData.getData[i])
      print("presentPosition :",self.presentPosition)
      rospy.logdebug("GetJointAngleClient Receive ")
    except rospy.ServiceException, e:
      rospy.logerr('EndingExec Failed ')

  def MotoJointTransfer( self, positionSend, duration=100):
    """ positionSend:目标位置list   duration:运动持续时间ms """
    self.motoWaitFlag = 1
    intervalCount = duration//20 +1  # duration (ms)

    moveSequence = []
    for i in range(len(positionSend)):
      if positionSend[i] == 200:
        moveSequence.append( numpy.linspace(self.presentPosition[i], self.presentPosition[i], num=intervalCount, endpoint=True) )
      else:
        moveSequence.append( numpy.linspace(self.presentPosition[i], positionSend[i], num=intervalCount, endpoint=True) )
    for i in range(intervalCount):
      sequenceSend = []
      for j in range(len(positionSend)):
        sequenceSend.append(moveSequence[j][i])
      self.MotoJointPub.publish(positions=sequenceSend, mainControlID=self.controlID)
    rospy.logdebug("MotoJointTransfer publish")

  def HeadJointTransfer( self, positionSend, duration=100):
    """ positionSend:目标位置list   duration:运动持续时间ms """
    self.motoWaitFlag = 1
    intervalCount = duration//20 +1  # duration (ms)
    moveSequence = []
    moveSequence.append( numpy.linspace(self.presentPosition[19-1], positionSend[0], num=intervalCount, endpoint=True) )
    moveSequence.append( numpy.linspace(self.presentPosition[20-1], positionSend[1], num=intervalCount, endpoint=True) )
    print(moveSequence[0])
    print(moveSequence[1])
    for i in range(intervalCount):
      sequenceSend = []
      sequenceSend.append(moveSequence[0][i])
      sequenceSend.append(moveSequence[1][i])
      self.HeadOnlyJointPub.publish(positions=sequenceSend, mainControlID=self.controlID)
    rospy.logdebug("HeadJointTransfer publish")

  def MotoWait(self):
    ros_rate = rospy.Rate(10)
    while self.motoWaitFlag : #and (not rospy.is_shutdown()) 
      rospy.logdebug("MotoWait() moto is moving... motoWaitFlag:%d",self.motoWaitFlag)
      try:
        rospy.wait_for_message('MediumSize/BodyHub/Status', UInt16, 195) # MotoWait超时时间195second
      except:
        self.motoWaitFlag = 0
        rospy.logerr("MotoWaitTimeOut: Publish failed ")
      ros_rate.sleep()
    rospy.sleep(0.1)#fftest

  def BodyHubStatusCallback(self, receiveData):
    # MotoWait Ending Signal
    if receiveData.data == self.StateStyle['pause'] :
      self.motoWaitFlag = 0
      rospy.logdebug("MotoJointTransfer act completed... motoWaitFlag:%d",self.motoWaitFlag)

  def ServoPositionCallback(self, receiveData):
    rospy.logdebug("ServoPositionCallback...")
    for i in range(20):
      if receiveData.angle[i] > 200:
        self.presentPosition[i] = 0
      else:
        self.presentPosition[i] = receiveData.angle[i]

  def EndingExecCallback(self, req):
    rospy.logerr("Ending ActMoveExample node")

    if req.uintReq == self.StateEnumEnding:
      self.EndingOrder = self.StateEnumEnding

    return SrvTLSuintResponse(97)
  
  def terminalScript(self, end_func):
    detect_rate = rospy.Rate(10)
    while (1):
      detect_rate.sleep()
      if self.EndingOrder == self.StateEnumEnding :    # 强制终止命令
        end_func()
        os.kill(self.killpid,signal.SIGKILL)



#// EXAMPLE
if __name__ == '__main__':

  # example send array
  arraySendA = [-45, -45, -45, -45, -45, -45, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]#double angle
  arraySendB = [45, 45, 45, 45, 45, 45, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]#double angle
  arraySendC = [-45, -45, -45, -45, -45, -45, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]#double angle
  arraySendD = [45, 45, 45, 45, 45, 45, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]#double angle

  headarraySendA = [90.0,90.0]
  headarraySendB = [0.0,0.0]

  servoDataSend = ServoJointTrajectoryC()

  try:
      # 初始化ros节点
      rospy.init_node('ServoJointTrajectory', anonymous=True, log_level=rospy.INFO)# DEBUG INFO ERROR WARN
      rospy.loginfo("Starting ServoJointTrajectory node")

      rospy.sleep(2.0) # 必要延迟，给予初始化时间
      
      while not rospy.is_shutdown():
        durationSet = 1000  # (n*20ms)

        servoDataSend.MotoJointTransfer(arraySendC)
        servoDataSend.MotoWait()
        rospy.sleep(3)
        servoDataSend.MotoJointTransfer(arraySendD)
        servoDataSend.MotoWait()
        rospy.sleep(3)

        servoDataSend.HeadJointTransfer(headarraySendA, durationSet)
        servoDataSend.MotoWait()
        rospy.sleep(3)
        servoDataSend.HeadJointTransfer(headarraySendB, durationSet)
        servoDataSend.MotoWait()
        rospy.sleep(3)
        
  except KeyboardInterrupt:
      rospy.logwarn("Shutting down ServoJointTrajectory node.")