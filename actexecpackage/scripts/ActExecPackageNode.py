#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os

import rospy
import array

from std_msgs.msg import *
from bodyhub.msg import *
from bodyhub.srv import * 
from actexecpackage.srv import *

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../config")))

class StatusOfNode():

  def __init__(self):

    # 节点运行状态
    self.StateStyle = {
      'init' : 20,
      'preReady' : 21,
      'ready' : 22,
      'running' : 23,
      'pause' : 24,
      'stoping' : 25,
      'error' : 26,
    }

    self.StateEnumEnding = 97

    self.presentState = self.StateStyle['init']
    self.presentStateStr = 'init'
    self.actNaemString = ''
    self.controlID = 2

    self.StatusPub = rospy.Publisher('MediumSize/ActPackageExec/Status', UInt16, queue_size=1)
    self.GetStatusSrv = rospy.Service('MediumSize/ActPackageExec/GetStatus', SrvString, self.GetStatusCallback)

  def UpdateState(self, stateNew):
    self.presentState = self.StateStyle[stateNew]
    actExecPackageStateMsg = self.presentState
    self.StatusPub.publish(actExecPackageStateMsg)
    self.presentStateStr = stateNew
    rospy.loginfo("StateStyle['" + stateNew + "']")
    
  def GetStatusCallback(self, req):
    if req.str != '':
      return [self.presentStateStr,0]


  def STATEinit(self):
    self.UpdateState('init')

    rospy.logdebug("Init complete!!!")

    self.UpdateState('preReady')  #更新下一个状态

  def StateSrvCallback(self,req):
    rospy.logdebug("StateSrvCallback %d", req.stateReq)

    if (req.stateReq == "setStatus") :
      if (self.presentState == self.StateStyle['preReady']) :
        rospy.wait_for_service('/MediumSize/BodyHub/GetMasterID')
        try:
          GetMasterIDClient = rospy.ServiceProxy('/MediumSize/BodyHub/GetMasterID', SrvTLSstring)
          resState = GetMasterIDClient('ActExecPackageNode')
          rospy.loginfo("GetMasterIDClient Receive %d", resState.data)
          if resState.data == 0:
            rospy.wait_for_service('/MediumSize/BodyHub/StateJump')
            try:
              SetBodyHubStaClient = rospy.ServiceProxy('MediumSize/BodyHub/StateJump', SrvState)
              resData = SetBodyHubStaClient(self.controlID, "setStatus")
              self.UpdateState('ready') #更新下一个状态
            except rospy.ServiceException, e:
              rospy.logerr('BodyHub StateJump error')
          else:
            rospy.logwarn("BodyHub is bussy now %d",resState.data)
        except rospy.ServiceException, e:
          rospy.logerr('GetMasterID Failed')
      
    elif (req.stateReq == "resetStatus") :
      rospy.loginfo('no such operation')
      
    elif (req.stateReq == "break") :
      if (self.presentState == self.StateStyle['running']) :
        rospy.wait_for_service('MediumSize/ActPackageExec/EndingExec')
        try:
          EndingExecClient = rospy.ServiceProxy('MediumSize/ActPackageExec/EndingExec', SrvTLSuint)
          resData = EndingExecClient(self.StateEnumEnding) 
          rospy.logdebug("EndingExecClient Receive %d", resData.uintRes) 
        except rospy.ServiceException, e:
          rospy.logerr('EndingExec Failed ')
        # self.UpdateState('preReady') #更新下一个状态 执行中断,自动跳转 'pause'
      
    elif (req.stateReq == "stop") :
      if (self.presentState == self.StateStyle['pause'])  :
        self.UpdateState('stoping') #更新下一个状态

    elif (req.stateReq == "reset") :
      if ((self.presentState == self.StateStyle['pause']) or \
        (self.presentState == self.StateStyle['error']) or \
          (self.presentState == self.StateStyle['ready'])):

        #释放下级节点
        rospy.wait_for_service('/MediumSize/BodyHub/StateJump')
        try:
          SetBodyHubStaClient = rospy.ServiceProxy('MediumSize/BodyHub/StateJump', SrvState)
          resData = SetBodyHubStaClient(self.controlID, "reset")
        except rospy.ServiceException, e:
          rospy.logerr('BodyHub StateJump error')

        self.UpdateState('preReady') #更新下一个状态

    return SrvStateResponse(self.presentState)

  def ActSrvCallback(self, req):
    
    rospy.logdebug("ActSrvCallback %s", req.actNameReq)

    # 请求服务更新下一个状态
    if (self.presentState == self.StateStyle['ready']) or (self.presentState == self.StateStyle['pause']) :
      self.actNaemString = req.actNameReq
      self.UpdateState('running') #更新下一个状态

    return SrvActScriptResponse('55')


def STATEpreReady(actExecPackageStatus) :
  rospy.logdebug("STATEpreReady") 

def STATEready(actExecPackageStatus) :
  rospy.logdebug("STATEready") 

def STATErunning(actExecPackageStatus) :
  rospy.logdebug("STATErunning__steady to run ...")
  try:
    os.system(actExecPackageStatus.actNaemString) # */actexecpackage/config/*Example.py
  except:
    rospy.logerr("run .py error")
    actExecPackageStatus.UpdateState('error') #更新下一个状态
    return

  actExecPackageStatus.UpdateState('pause') #更新下一个状态

def STATEpause(actExecPackageStatus) :
  test = 1

def STATEstoping(actExecPackageStatus) :
  #下级节点stop
  rospy.wait_for_service('/MediumSize/BodyHub/StateJump')
  try:
    SetBodyHubStaClient = rospy.ServiceProxy('MediumSize/BodyHub/StateJump', SrvState)
    resData = SetBodyHubStaClient(actExecPackageStatus.controlID, "stop")
    actExecPackageStatus.UpdateState('ready') #更新下一个状态
  except rospy.ServiceException, e:
    rospy.logerr('BodyHub StateJump error')

def STATEerror(actExecPackageStatus) :
  test = 1


def ActExecPackage():
  # 初始化ros节点
  rospy.init_node('ActExecPackageNode', anonymous=True, log_level=rospy.INFO)# DEBUG INFO ERROR WARN
  rospy.loginfo('Starting ActExecPackageNode node')

  actExecStatus = StatusOfNode()

  StateServer = rospy.Service('MediumSize/ActPackageExec/StateJump', SrvState, actExecStatus.StateSrvCallback)
  ActScriptServer = rospy.Service('MediumSize/ActPackageExec/actNameString', SrvActScript, actExecStatus.ActSrvCallback)

  actExecStatus.STATEinit() 

  ros_rate = rospy.Rate(100) #100Hz
  while not rospy.is_shutdown():

    if (actExecStatus.presentState == actExecStatus.StateStyle['preReady']):
      STATEpreReady(actExecStatus)
    elif (actExecStatus.presentState == actExecStatus.StateStyle['ready']):
      STATEready(actExecStatus)
    elif(actExecStatus.presentState == actExecStatus.StateStyle['running']):
      STATErunning(actExecStatus)
    elif (actExecStatus.presentState == actExecStatus.StateStyle['pause']):
      STATEpause(actExecStatus)
    elif (actExecStatus.presentState == actExecStatus.StateStyle['stoping']):
      STATEstoping(actExecStatus)
    elif (actExecStatus.presentState == actExecStatus.StateStyle['error']):
      STATEerror(actExecStatus)

    ros_rate.sleep()

if __name__ == '__main__':

  try:
     ActExecPackage()

  except KeyboardInterrupt:
      rospy.logwarn ("Shutting down ActExecPackageNode node.")