#include "headcontroller/headcontroller.h"

void UpdateState(uint8_t stateNew);

void ServoPositionCallback(const bodyhub::ServoPositionAngle::ConstPtr& msg) {
  presentPosition[19 - 1] = msg->angle.at(18);
  presentPosition[20 - 1] = msg->angle.at(19);
}

bool StateSrvCallback(bodyhub::SrvState::Request& req,
                      bodyhub::SrvState::Response& res) {
  bodyhub::SrvTLSstring reqStateSrv;
  res.stateRes = headcontrollerState;
  if (req.stateReq == "setStatus") {
    if (headcontrollerState == StateEnum::preReady) {
      reqStateSrv.request.str = "HeadController";
      if (GetMasterIDClient.call(reqStateSrv)) {
        if (reqStateSrv.response.data == 0) {  // call setStatus
          reqDataSrv.request.masterID = controlID;
          reqDataSrv.request.stateReq = "setStatus";
          if (SetBodyHubStaClient.call(reqDataSrv)) {
            ROS_INFO("SetBodyHubStaClient %d", reqDataSrv.response.stateRes);
            UpdateState(StateEnum::ready);  //更新下一个状态
          } else
            ROS_ERROR("BodyHub StateJump error");
        } else
          ROS_WARN("BodyHub is bussy now %d", reqStateSrv.response.data);
        res.stateRes = reqStateSrv.response.data;
      } else
        ROS_ERROR("GetMasterID Failed");
    }
  } else if (req.stateReq == "resetStatus") {
  } else if (req.stateReq == "break") {
  } else if (req.stateReq == "stop") {
    if (headcontrollerState == StateEnum::pause)
      UpdateState(StateEnum::stoping);  //更新下一个状态
  } else if (req.stateReq == "reset") {
    if (headcontrollerState != StateEnum::stoping)
      UpdateState(StateEnum::preReady);  //更新下一个状态
  }

  return true;
}

bool GetStatusCallback(bodyhub::SrvString::Request& req,
                       bodyhub::SrvString::Response& res) {
  if (req.str != "") res.data = stateNewStr;
  return true;
}

std::vector<double> linspace(double start, double end, int num) {
  std::vector<double> linspaced;

  if (num == 0) {
    return linspaced;
  }
  if (num == 1) {
    linspaced.push_back(start);
    return linspaced;
  }

  double delta = (end - start) / (num - 1);

  for (int i = 0; i < num - 1; ++i) {
    linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end);

  return linspaced;
}

bool HeadJointSrvCallback(headcontroller::SrvHeadJoint::Request& req,
                          headcontroller::SrvHeadJoint::Response& res) {
  uint8_t motoID, intervalCount, sequenceNum;
  double presentPosition19, presentPosition20, goalPosition19, goalPosition20;
  std::vector<double> moveSequence19, moveSequence20;

  presentPosition19 = presentPosition[19 - 1];
  presentPosition20 = presentPosition[20 - 1];

  intervalCount = req.duration / 20 + 1;  // duration (ms)
  for (motoID = 0; motoID < req.IDnum.size(); motoID++) {
    if (req.IDnum[motoID] == "ID19") {
      goalPosition19 = req.position[motoID];
      moveSequence19 =
          linspace(presentPosition19, goalPosition19, intervalCount);
    }
    if (req.IDnum[motoID] == "ID20") {
      goalPosition20 = req.position[motoID];
      moveSequence20 =
          linspace(presentPosition20, goalPosition20, intervalCount);
    }
  }
  //如果moveSequence为空
  if ((moveSequence19.size() == 0) && (moveSequence20.size() == 0)) return 0;
  if (moveSequence19.size() == 0)
    for (uint8_t i = 0; i < intervalCount; i++)
      moveSequence19.push_back(presentPosition19);
  if (moveSequence20.size() == 0)
    for (uint8_t i = 0; i < intervalCount; i++)
      moveSequence20.push_back(presentPosition19);

  for (sequenceNum = 0; sequenceNum < intervalCount; sequenceNum++) {
    bodyhub::JointControlPoint HeadJointMsg;
    HeadJointMsg.positions.push_back(moveSequence19[sequenceNum]);  // 19
    HeadJointMsg.positions.push_back(moveSequence20[sequenceNum]);  // 20
    HeadJointMsg.mainControlID = controlID;
    headJointQueue.push(HeadJointMsg);
  }

  //数据到达
  if ((headcontrollerState == StateEnum::ready) ||
      (headcontrollerState == StateEnum::pause))
    UpdateState(StateEnum::running);
  res.headJointRes = 88;
}

void UpdateState(uint8_t stateNew) {
  headcontrollerState = stateNew;
  headcontrollerStateMsg.data = stateNew;
  StatusPub.publish(headcontrollerStateMsg);
  switch (stateNew) {
    case StateEnum::init:
      stateNewStr = "init";
      break;
    case StateEnum::preReady:
      stateNewStr = "preReady";
      break;
    case StateEnum::ready:
      stateNewStr = "ready";
      break;
    case StateEnum::running:
      stateNewStr = "running";
      break;
    case StateEnum::pause:
      stateNewStr = "pause";
      break;
    case StateEnum::stoping:
      stateNewStr = "stoping";
      break;
    case StateEnum::error:
      stateNewStr = "error";
      break;
    case StateEnum::directOperate:
      stateNewStr = "directOperate";
      break;
    default:
      break;
  }
  ROS_INFO("The new headcontrollerState: %s--%d ", stateNewStr.c_str(),
           stateNew);

  if (stateNew == StateEnum::preReady) {
    //释放下级节点
    reqDataSrv.request.masterID = controlID;
    reqDataSrv.request.stateReq = "reset";
    if (SetBodyHubStaClient.call(reqDataSrv))
      ROS_INFO("SetBodyHubStaClient %d", reqDataSrv.response.stateRes);
    else
      ROS_ERROR("BodyHub StateJump error");
  }
}

void STATEpreReady() { const char* log = NULL; }
void STATEready() { const char* log = NULL; }
void STATErunning() {
  bodyhub::JointControlPoint HeadJointMsg;

  if (headJointQueue.empty()) {
    //更新下一个状态
    UpdateState(StateEnum::pause);
  } else {
#ifdef DEBUG
    ROS_INFO("HeadJointPub publish");
#endif
    HeadJointMsg = headJointQueue.front();
    HeadJointPub.publish(HeadJointMsg);
    headJointQueue.pop();
  }
}
void STATEpause() { const char* log = NULL; }
void STATEstoping() {
  //下级节点stop
  reqDataSrv.request.masterID = controlID;
  reqDataSrv.request.stateReq = "stop";
  if (SetBodyHubStaClient.call(reqDataSrv)) {
    ROS_INFO("SetBodyHubStaClient %d", reqDataSrv.response.stateRes);
    //更新下一个状态
    UpdateState(StateEnum::ready);
  } else
    ROS_ERROR("BodyHub StateJump error");
}
void STATEerror() { const char* log = NULL; }

void STATEinit(ros::NodeHandle nh) {
  bodyhub::SrvServoAllRead reqGetAngleSrv;
  reqGetAngleSrv.request.idArray = {0, 1, 2, 3};
  reqGetAngleSrv.request.idCnt = 2;

  UpdateState(StateEnum::init);

  ROS_INFO("Init complete!!!");

  if (GetJointAngleClient.call(reqGetAngleSrv)) {
    ROS_INFO("GetJointAngleClient ID19:%f ID20:%f",
             reqGetAngleSrv.response.getData[18],
             reqGetAngleSrv.response.getData[19]);
    presentPosition[19 - 1] = reqGetAngleSrv.response.getData[18];
    presentPosition[20 - 1] = reqGetAngleSrv.response.getData[19];
  } else
    ROS_ERROR("BodyHub StateJump error");

  //更新下一个状态
  UpdateState(StateEnum::preReady);
}

int main(int argc, char** argv) {
  //初始化节点
  ros::init(argc, argv, "HeadControllerNode");
  ros::NodeHandle nodeHandle("");
  ROS_INFO("start HeadControllerNode");

  StatusPub = nodeHandle.advertise<std_msgs::UInt16>(
      "MediumSize/HeadController/Status", 1);
  HeadJointPub = nodeHandle.advertise<bodyhub::JointControlPoint>(
      "MediumSize/BodyHub/HeadPosition", 100);
  ServoPositionSub = nodeHandle.subscribe<bodyhub::ServoPositionAngle>(
      "MediumSize/BodyHub/ServoPositions", 1, ServoPositionCallback);
  StateServer = nodeHandle.advertiseService(
      "MediumSize/HeadController/StateJump", StateSrvCallback);
  GetStatusServer = nodeHandle.advertiseService(
      "MediumSize/HeadController/GetStatus", GetStatusCallback);
  HeadJointServer = nodeHandle.advertiseService(
      "MediumSize/HeadController/HeadJointTrajectory", HeadJointSrvCallback);
  GetMasterIDClient = nodeHandle.serviceClient<bodyhub::SrvTLSstring>(
      "/MediumSize/BodyHub/GetMasterID");
  SetBodyHubStaClient = nodeHandle.serviceClient<bodyhub::SrvState>(
      "MediumSize/BodyHub/StateJump");
  GetJointAngleClient = nodeHandle.serviceClient<bodyhub::SrvServoAllRead>(
      "MediumSize/BodyHub/GetJointAngle");

  STATEinit(nodeHandle);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    if (headcontrollerState == StateEnum::preReady)
      STATEpreReady();
    else if (headcontrollerState == StateEnum::ready)
      STATEready();
    else if (headcontrollerState == StateEnum::running)
      STATErunning();
    else if (headcontrollerState == StateEnum::pause)
      STATEpause();
    else if (headcontrollerState == StateEnum::stoping)
      STATEstoping();
    else if (headcontrollerState == StateEnum::error)
      STATEerror();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
