#ifndef HEADCONTROLLER_H
#define HEADCONTROLLER_H

#include <stdio.h>
#include <stdlib.h>
#include <queue>  //For queue container

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/UInt16.h"

#include "bodyhub/JointControlPoint.h"
#include "bodyhub/ServoPositionAngle.h"
#include "bodyhub/SrvServoAllRead.h"
#include "bodyhub/SrvState.h"
#include "bodyhub/SrvString.h"
#include "bodyhub/SrvTLSstring.h"
#include "bodyhub/bodyhubStatus.h"
#include "headcontroller/SrvHeadJoint.h"

#define DEBUG

int8_t headcontrollerState = 0;
double presentPosition[20] = {0};

std_msgs::UInt16 headcontrollerStateMsg;

std::queue<bodyhub::JointControlPoint> headJointQueue;

uint8_t controlID = 5;
std::string stateNewStr;

bodyhub::SrvState reqDataSrv;

ros::Publisher StatusPub;
ros::Publisher HeadJointPub;
ros::Subscriber ServoPositionSub;
ros::ServiceServer HeadJointServer;
ros::ServiceServer StateServer;
ros::ServiceServer GetStatusServer;
ros::ServiceClient GetMasterIDClient;
ros::ServiceClient GetJointAngleClient;
ros::ServiceClient SetBodyHubStaClient;

#endif