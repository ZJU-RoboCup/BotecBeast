#include "WalkingEngine/walkPlaning/CPWalking5.h"
#include "WalkingEngine/walkPlaning/LIPMWalk.h"
#include "mediumsize_msgs/SetAction.h"
#include "ros/ros.h"

typedef enum PaceType {
  paceNormal = 0,
  paceFast,
  paceSlow,
} PaceType_t;

extern GaitManager::LIPMWalk mWalk;
extern uint8_t armMode;

ros::ServiceServer standSrv;
ros::ServiceServer paceSrv;
ros::ServiceServer armModeSrv;
uint8_t standType = 0;
PaceType_t paceType;

bool standSrvCallback(mediumsize_msgs::SetAction::Request &req,
                      mediumsize_msgs::SetAction::Response &res) {
  standType = req.cmd;
  if (standType == 1)
    mWalk.RobotState = mWalk.Action_SlopeStand;
  else
    mWalk.RobotState = mWalk.Walk_stand;
  res.result = standType;
  ROS_INFO("standType: %d", standType);
  return true;
}

bool paceSrvCallback(mediumsize_msgs::SetAction::Request &req,
                     mediumsize_msgs::SetAction::Response &res) {
  switch (req.cmd) {
    case 0:
      paceType = paceNormal;
      mWalk.setStepTime(0.5);
      break;
    case 1:
      paceType = paceFast;
      mWalk.setStepTime(0.4);
      break;
    case 2:
      paceType = paceFast;
      mWalk.setStepTime(0.6);
      break;
  }
  res.result = paceType;
  ROS_INFO("paceType: %d", paceType);
  return true;
}

bool armModeSrvCallback(mediumsize_msgs::SetAction::Request &req,
                        mediumsize_msgs::SetAction::Response &res) {
  switch (req.cmd) {
    case 0:
      armMode = 0;
      break;
    case 1:
      armMode = 1;
      break;
  }
  res.result = armMode;
  ROS_INFO("armMode: %d", armMode);
  return true;
}

void actionManageInit() {
  ros::NodeHandle nh;
  standSrv =
      nh.advertiseService("/MediumSize/BodyHub/standType", standSrvCallback);
  paceSrv =
      nh.advertiseService("/MediumSize/BodyHub/paceType", paceSrvCallback);
  paceSrv =
      nh.advertiseService("/MediumSize/BodyHub/armMode", armModeSrvCallback);
}