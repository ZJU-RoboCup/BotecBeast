#include <iostream>
#include "Util.h"
#include "../include/walkPlaning/RobotDimensions.h"
#include "bodyhub/SrvState.h"
#include "ik_module/PoseArray.h"
#include "ik_module/SrvPoses.h"
#include "ros/ros.h"

using namespace std;

ros::Publisher targetPosesPub;
ros::ServiceClient setStatusClient;
ros::ServiceClient getPosesClient;

bodyhub::SrvState statusSrv;
geometry_msgs::Pose poseMsg;
ik_module::SrvPoses posesSrv;
ik_module::PoseArray poseArrayMsg;

std::vector<double> linspace(double start, double end, int num)
{
  std::vector<double> linspaced;

  if (num == 0) {
    return linspaced;
  }
  if (num == 1) {
    linspaced.push_back(start);
    return linspaced;
  }

  double delta = (end - start) / (num);

  for (int i = 0; i <= num; ++i) {
    linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end);

  return linspaced;
}

bool toTargetPose(double_t duration, std::vector<uint8_t> linkId, std::vector<geometry_msgs::Pose> targetPose)
{
  uint8_t numberOfLink = 4;
  uint32_t numberOfFrame = duration / 0.025;
  ik_module::SrvPoses posesSrv;
  posesSrv.request.str = "get";
  if (getPosesClient.call(posesSrv)) 
  {
    uint8_t linkIdx = 0, linkIdFlag[numberOfLink] = {0};
    std::vector<std::vector<double_t> > tempVector;
    geometry_msgs::Pose poseRoute[4][numberOfFrame];
    ik_module::PoseArray poseArrayMsg;
    
    for(uint8_t i=0;i<linkId.size();i++)
    {
      linkIdFlag[linkId[i]-1] = 1;
    }
    
    tempVector.resize(7);
    for(uint8_t i=0;i<numberOfLink;i++)
    {
      if(linkIdFlag[i])
      {
        tempVector[0] = linspace(posesSrv.response.poses[i].position.x, targetPose[linkIdx].position.x, numberOfFrame);
        tempVector[1] = linspace(posesSrv.response.poses[i].position.y, targetPose[linkIdx].position.y, numberOfFrame);
        tempVector[2] = linspace(posesSrv.response.poses[i].position.z, targetPose[linkIdx].position.z, numberOfFrame);
        tempVector[3] = linspace(posesSrv.response.poses[i].orientation.w, targetPose[linkIdx].orientation.w, numberOfFrame);
        tempVector[4] = linspace(posesSrv.response.poses[i].orientation.x, targetPose[linkIdx].orientation.x, numberOfFrame);
        tempVector[5] = linspace(posesSrv.response.poses[i].orientation.y, targetPose[linkIdx].orientation.y, numberOfFrame);
        tempVector[6] = linspace(posesSrv.response.poses[i].orientation.z, targetPose[linkIdx].orientation.z, numberOfFrame);
        linkIdx++;
      }
      else
      {
        tempVector[0] = linspace(posesSrv.response.poses[i].position.x, posesSrv.response.poses[i].position.x, numberOfFrame);
        tempVector[1] = linspace(posesSrv.response.poses[i].position.y, posesSrv.response.poses[i].position.y, numberOfFrame);
        tempVector[2] = linspace(posesSrv.response.poses[i].position.z, posesSrv.response.poses[i].position.z, numberOfFrame);
        tempVector[3] = linspace(posesSrv.response.poses[i].orientation.w, posesSrv.response.poses[i].orientation.w, numberOfFrame);
        tempVector[4] = linspace(posesSrv.response.poses[i].orientation.x, posesSrv.response.poses[i].orientation.x, numberOfFrame);
        tempVector[5] = linspace(posesSrv.response.poses[i].orientation.y, posesSrv.response.poses[i].orientation.y, numberOfFrame);
        tempVector[6] = linspace(posesSrv.response.poses[i].orientation.z, posesSrv.response.poses[i].orientation.z, numberOfFrame);
      }
    
      for(uint8_t n=0;n<numberOfFrame;n++)
      {
        poseRoute[i][n].position.x = tempVector[0][n];
        poseRoute[i][n].position.y = tempVector[1][n];
        poseRoute[i][n].position.z = tempVector[2][n];
        poseRoute[i][n].orientation.w = tempVector[3][n];
        poseRoute[i][n].orientation.x = tempVector[4][n];
        poseRoute[i][n].orientation.y = tempVector[5][n];
        poseRoute[i][n].orientation.z = tempVector[6][n];
      }
    }

    poseArrayMsg.controlId = 6;
    for(uint8_t n=0;n<numberOfFrame;n++)
    {
      for(uint8_t i=0;i<numberOfLink;i++)
      {
        poseArrayMsg.poses.push_back(poseRoute[i][n]);
      }
      targetPosesPub.publish(poseArrayMsg);
      poseArrayMsg.poses.clear();
      usleep(25*1000);
    }
    return true;
  }
  return false;
}

void advertise(ros::NodeHandle nh) {
  targetPosesPub =
      nh.advertise<ik_module::PoseArray>("MediumSize/IKmodule/TargetPoses", 1);
  setStatusClient =
      nh.serviceClient<bodyhub::SrvState>("MediumSize/IKmodule/SetStatus");
  getPosesClient =
      nh.serviceClient<ik_module::SrvPoses>("MediumSize/IKmodule/GetPoses");
}

void initTarget() {
  ROS_INFO("Start initializing the position!");

  std::vector<uint8_t> linkId;
  geometry_msgs::Pose pose;
  std::vector<geometry_msgs::Pose> tergetPoseList;

  linkId.push_back(3);
  pose.position.x = 0.0;
  pose.position.y = Shoulder2_Y+Shoulder1_Y+Elbow1_Y+Wrist1_Y-0e-6;
  pose.position.z = Shoulder2_Z; 
  tergetPoseList.push_back(pose);

  linkId.push_back(4);
  pose.position.x = 0.0;
  pose.position.y = -(Shoulder2_Y+Shoulder1_Y+Elbow1_Y+Wrist1_Y-0e-6);
  pose.position.z = Shoulder2_Z; 
  tergetPoseList.push_back(pose);

  toTargetPose(1.5, linkId, tergetPoseList);
  usleep(2*1000*1000);
  
  // 初始化目标位姿
  double_t leftFootX, leftFootY, leftFootZ;
  double_t leftFootR_W, leftFootR_X, leftFootR_Y, leftFootR_Z;
  
  double_t rightFootX, rightFootY, rightFootZ;
  double_t rightFootR_W, rightFootR_X, rightFootR_Y, rightFootR_Z;

  double_t leftArmX, leftArmY, leftArmZ;
  double_t rightArmX, rightArmY, rightArmZ;
  // 手臂移动的中心坐标和半径
  double_t centerX = -Shoulder2_X, centerY = Shoulder2_Y, centerZ = Shoulder2_Z;
  double_t radius = Shoulder1_Y+Elbow1_Y+Wrist1_Y-1e-6;
  posesSrv.request.str = "get";
  // 获取当前的位姿
  if (getPosesClient.call(posesSrv)) {
    leftFootX = posesSrv.response.poses[0].position.x;
    leftFootY = posesSrv.response.poses[0].position.y;
    leftFootZ = posesSrv.response.poses[0].position.z;
    leftFootR_W = posesSrv.response.poses[0].orientation.w;
    leftFootR_X = posesSrv.response.poses[0].orientation.x;
    leftFootR_Y = posesSrv.response.poses[0].orientation.y;
    leftFootR_Z = posesSrv.response.poses[0].orientation.z;

    rightFootX = posesSrv.response.poses[1].position.x;
    rightFootY = posesSrv.response.poses[1].position.y;
    rightFootZ = posesSrv.response.poses[1].position.z;
    rightFootR_W = posesSrv.response.poses[1].orientation.w;
    rightFootR_X = posesSrv.response.poses[1].orientation.x;
    rightFootR_Y = posesSrv.response.poses[1].orientation.y;
    rightFootR_Z = posesSrv.response.poses[1].orientation.z;

    leftArmX = posesSrv.response.poses[2].position.x;
    leftArmY = posesSrv.response.poses[2].position.y;
    leftArmZ = posesSrv.response.poses[2].position.z;

    rightArmX = posesSrv.response.poses[3].position.x;
    rightArmY = posesSrv.response.poses[3].position.y;
    rightArmZ = posesSrv.response.poses[3].position.z;

  } else {
    ROS_ERROR("Failed to getPosesClient!");
  }

  double xcom  = leftFootX; 
  double width = leftFootY;
  double torsoHeight = -leftFootZ; 
  double torsoHeightWalk = torsoHeight - 0.02;  // torso height at walking
  double footSeprate = 0.055*2+0.03;
  ros::Rate loop_rate(10);

  // 发布目标位姿
  for (uint8_t j = 1; j <= 50; ++j) {
    /////////////////////////////////////////////////////////////////////
    leftFootX  = xcom - (xcom)*j / 50.;
    rightFootX = xcom - (xcom)*j / 50.;
    leftFootY  =  width + ((footSeprate / 2 - width)) * j / 50.;
    rightFootY = -width - ((footSeprate / 2 - width)) * j / 50.;
    leftFootZ  = -(torsoHeight - (torsoHeight - torsoHeightWalk) * j / 50.);
    rightFootZ = -(torsoHeight - (torsoHeight - torsoHeightWalk) * j / 50.);

    leftArmY  =  centerY + radius * cos((j*70/50.)*Util::TO_RADIAN);
    rightArmY = -centerY - radius * cos((j*70/50.)*Util::TO_RADIAN);
    leftArmZ  =  centerZ - radius * sin((j*70/50.)*Util::TO_RADIAN);
    rightArmZ =  centerZ - radius * sin((j*70/50.)*Util::TO_RADIAN);
    /////////////////////////////////////////////////////////////////////
    // Left Leg
    poseMsg.position.x = leftFootX;
    poseMsg.position.y = leftFootY;
    poseMsg.position.z = leftFootZ;
    poseMsg.orientation.x = leftFootR_X;
    poseMsg.orientation.y = leftFootR_Y;
    poseMsg.orientation.z = leftFootR_Z;
    poseMsg.orientation.w = leftFootR_W;
    poseArrayMsg.poses.push_back(poseMsg);
    // Right Leg
    poseMsg.position.x = rightFootX;
    poseMsg.position.y = rightFootY;
    poseMsg.position.z = rightFootZ;
    poseMsg.orientation.x = rightFootR_X;
    poseMsg.orientation.y = rightFootR_Y;
    poseMsg.orientation.z = rightFootR_Z;
    poseMsg.orientation.w = rightFootR_W;
    poseArrayMsg.poses.push_back(poseMsg);
    // Left Arm
    poseMsg.position.x = leftArmX;
    poseMsg.position.y = leftArmY;
    poseMsg.position.z = leftArmZ;
    poseMsg.orientation.x = 0;
    poseMsg.orientation.y = 0;
    poseMsg.orientation.z = 0;
    poseMsg.orientation.w = 1;
    poseArrayMsg.poses.push_back(poseMsg);
    // Right Arm
    poseMsg.position.x = rightArmX;
    poseMsg.position.y = rightArmY;
    poseMsg.position.z = rightArmZ;
    poseMsg.orientation.x = 0;
    poseMsg.orientation.y = 0;
    poseMsg.orientation.z = 0;
    poseMsg.orientation.w = 1;
    poseArrayMsg.poses.push_back(poseMsg);
    // Publish Msg
    poseArrayMsg.controlId = 6;
    targetPosesPub.publish(poseArrayMsg);
    poseArrayMsg.poses.clear();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void setStatus() {
  statusSrv.request.masterID = 6;
  statusSrv.request.stateReq = "setStatus";
  if (setStatusClient.call(statusSrv)) {
    ROS_INFO("Set status with response: %d", statusSrv.response.stateRes);
  } else {
    ROS_ERROR("Failed to setStatus");
  }
}

void reSet() {
  statusSrv.request.masterID = 6;
  statusSrv.request.stateReq = "reset";
  if (setStatusClient.call(statusSrv)) {
    ROS_INFO("Reset status with response: %d", statusSrv.response.stateRes);
  } else {
    ROS_ERROR("Failed to reset");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "targetPubNode");
  ros::NodeHandle nh;
  advertise(nh);
  char cmd;
  while (ros::ok()) {
    usleep(100 * 1000);
    std::cout << "q--quit s--hands down r--reset" << std::endl;
    cmd = tolower(getchar());
    switch (cmd) {
      case 'q':
        reSet();
        exit(0);
        break;
      case 's':
        setStatus();
        std::cout << "Start to hands down!" << std::endl;
        initTarget();
        break;
      case 'r':
        reSet();
        break;
      default:
        break;
    }
  }
}