#include "ikModule.h"

#define JOINT_NUM 22

int jointDirection[JOINT_NUM] = {
  1, 1, -1, -1, 1, -1,
  1, 1, 1, 1, -1, -1,
  -1, -1, -1, 
  1, -1, -1,
  1, 1, 1, 1
};

std::string stateStr;
uint8_t ikmoduleState = 20, bodyhubState = 20;
uint8_t currentControlId = 4, bodyhubControlId = 0;     // ikmodule当前控制ID, bodyhub当前控制ID
std::queue<std::queue<sva::PTransformd>> posturesQueue; // 目标位姿队列
std::queue<std::vector<double>> jointValuesQueue;       //　需要下发的关节角度队列

// 创建互斥锁
pthread_mutex_t mtxPQ;  // posturesQueue
pthread_mutex_t mtxJVQ; // jointValuesQueue

// ///////////////////////////////LIPM//////////////////////////////////////////
LIPMWalk mWalk;

// ///////////////////////////////ROS//////////////////////////////////////////

ros::Publisher MotoPositionPub;
ros::Publisher StatusPub;
ros::Publisher jointCmdPub;

ros::ServiceServer SetStatusService;
ros::ServiceServer GetStatusService;
ros::ServiceServer GetPosesService;
ros::ServiceServer MasterIDService;

ros::ServiceClient BodyHubSetStatusClient;
ros::ServiceClient BodyHubMasterIDClient;
ros::ServiceClient bodyhubStatusClient;
ros::ServiceClient bodyhubJointAngleClient;

void waitBodyhubReady()
{
  bodyhub::SrvString getBodyhubStatus;
  getBodyhubStatus.request.str = "get";
  while (ros::ok())
  {
    if (bodyhubStatusClient.call(getBodyhubStatus))
    {
      if (getBodyhubStatus.response.data != "init")
      {
        break;
      }
    }
    usleep(100 * 1000);
  }
}

std::vector<double> linspace(double start, double end, int num)
{
  std::vector<double> linspaced;

  if (num == 0)
  {
    return linspaced;
  }
  if (num == 1)
  {
    linspaced.push_back(start);
    return linspaced;
  }

  double delta = (end - start) / (num);

  for (int i = 0; i <= num; ++i)
  {
    linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end);

  return linspaced;
}

std::vector<double_t> boyhubToIkmodule(std::vector<double_t> bhj) // convert joint of ik model and bodyhub
{
  std::vector<double_t> imj(bhj.size()); // ik model joint, bodyhub joint

  for (uint8_t i = 0; i < 12; i++)
  {
    if (i >= bhj.size())
      return imj;
    imj[i] = bhj[i] * jointDirection[i];
  }
  for (uint8_t i = 12; i < 15; i++)
  {
    if (i >= bhj.size())
      return imj;
    imj[i] = bhj[i + 3] * jointDirection[i];
  }
  for (uint8_t i = 15; i < 18; i++)
  {
    if (i >= bhj.size())
      return imj;
    imj[i] = bhj[i - 3] * jointDirection[i];
  }
  for (uint8_t i = 18; i < bhj.size(); i++)
  {
    if (i >= bhj.size())
      return imj;
    imj[i] = bhj[i] * jointDirection[i];
  }
  return imj;
}

std::vector<double_t> ikmoduleToBodyhub(std::vector<double_t> imj) // convert joint of ik model and bodyhub
{
  std::vector<double_t> bhj(imj.size()); // bodyhub joint, ik model joint

  for (uint8_t i = 0; i < 12; i++)
  {
    if (i >= imj.size())
      return bhj;
    bhj[i] = imj[i] * jointDirection[i];
  }
  for (uint8_t i = 12; i < 15; i++)
  {
    if (i >= imj.size())
      return bhj;
    bhj[i] = imj[i + 3] * jointDirection[i+3];
  }
  for (uint8_t i = 15; i < 18; i++)
  {
    if (i >= imj.size())
      return bhj;
    bhj[i] = imj[i - 3] * jointDirection[i-3];
  }
  for (uint8_t i = 18; i < imj.size(); i++)
  {
    if (i >= imj.size())
      return bhj;
    bhj[i] = imj[i] * jointDirection[i];
  }
  return bhj;
}

bool getAngleOfJoint(std::vector<double_t> &ikModelAngleOfJoint)
{
  bodyhub::SrvServoAllRead getJointAngle;
  if (!bodyhubJointAngleClient.call(getJointAngle))
  {
    ROS_ERROR("request for joint angle failed!");
    return false;
  }
  ikModelAngleOfJoint.resize(getJointAngle.response.getData.size());
  ikModelAngleOfJoint = boyhubToIkmodule(getJointAngle.response.getData);
  return true;
}

bool syncIkModlePose()
{
  Eigen::VectorXd servoValueVector; // 舵机实时关节角度
  std::vector<double_t> ikModelJoint;
  if (!getAngleOfJoint(ikModelJoint))
  {
    ROS_ERROR("getAngleOfJoint error!");
    return false;
  }
  servoValueVector.resize(ikModelJoint.size());
  for (uint16_t i = 0; i < ikModelJoint.size(); i++)
  {
    servoValueVector[i] = ikModelJoint[i];
  }
  mWalk.talosRobot.talosmbc.q = sVectorToParam(mWalk.talosRobot.talos, servoValueVector.segment(0, 18) * Util::TO_RADIAN);
  rbd::forwardKinematics(mWalk.talosRobot.talos, mWalk.talosRobot.talosmbc);

  return true;
}

void sendJointCommand(uint32_t controlId, std::vector<uint8_t> idList, std::vector<double_t> angleList)
{
  sensor_msgs::JointState jointCmdMsg;
  jointCmdMsg.name.resize(idList.size());
  for (uint8_t i = 0; i < idList.size(); i++)
  {
    jointCmdMsg.name[i] = std::to_string(idList[i]);
  }
  jointCmdMsg.name.push_back(std::to_string(controlId));
  jointCmdMsg.position = angleList;
  jointCmdPub.publish(jointCmdMsg);
}

void toAngleOfJoint(double_t duration, std::vector<uint8_t> idList, std::vector<double_t> currentAngle, std::vector<double_t> targetAngle)
{
  uint32_t controlId = 6;
  uint32_t numberOfFrame = duration / 0.01;
  std::vector<double_t> angleList(idList.size());
  std::vector<std::vector<double_t>> angleVector(idList.size());
  for (uint8_t i = 0; i < idList.size(); i++)
  {
    angleVector[i] = linspace(currentAngle[i], targetAngle[i], numberOfFrame);
  }
  for (uint8_t n = 0; n < numberOfFrame; n++)
  {
    for (uint8_t i = 0; i < idList.size(); i++)
    {
      angleList[i] = angleVector[i][n];
    }
    sendJointCommand(controlId, idList, ikmoduleToBodyhub(angleList));
    // usleep(0.01*1000);
  }
}

//　订阅主题的回调函数
void BodyHubStatusCallback(const std_msgs::UInt16::ConstPtr &msg)
{
  // ROS_INFO("BodyHub node status update: %d",msg->data);
  bodyhubState = msg->data;
}

void TargetPosesCallback(const ik_module::PoseArray::ConstPtr &msg)
{
  sva::PTransformd targetPos;
  std::queue<sva::PTransformd> targetQueue;
  std::vector<geometry_msgs::Pose> posesArr;

  if (msg->controlId == currentControlId)
  {
    // ROS_INFO("Received new posearray with ID %d", msg->controlId);
    posesArr = msg->poses;

    for (uint8_t i = 0; i < posesArr.size(); i++)
    {
      Eigen::Quaterniond q(posesArr[i].orientation.w, posesArr[i].orientation.x,
                           posesArr[i].orientation.y, posesArr[i].orientation.z);
      targetPos.rotation() = q.toRotationMatrix();
      targetPos.translation() << posesArr[i].position.x, posesArr[i].position.y,posesArr[i].position.z;
      targetQueue.push(targetPos);
    }

    pthread_mutex_lock(&mtxPQ);
    posturesQueue.push(targetQueue);
    pthread_mutex_unlock(&mtxPQ);

    // 数据到达
    if ((ikmoduleState == StateEnum::ready) ||
        (ikmoduleState == StateEnum::pause))
      UpdateState(StateEnum::running);
  }
  else
  {
    ROS_ERROR("IkModule is busy with controlID %d", currentControlId);
  }
}

void toPosesCallback(const sensor_msgs::MultiDOFJointState::ConstPtr &msg)
{
  uint16_t msgId;
  double_t duration;
  try
  {
    msgId = std::stoi(msg->joint_names.at(msg->joint_names.size() - 1));
    duration = std::stof(msg->joint_names.at(msg->joint_names.size() - 2));
  }
  catch (const std::exception &e)
  {
    std::cerr << "toPosesCallback catch error: " << e.what() << '\n';
    return;
  }
  if (msgId != currentControlId)
  {
    ROS_ERROR("toPosesCallback controlId error!");
    return;
  }
  sensor_msgs::MultiDOFJointState posesMsg;
  posesMsg = *msg;
  posesMsg.joint_names.pop_back();
  posesMsg.joint_names.pop_back();

  Eigen::Quaterniond q;
  sva::PTransformd pose;
  std::vector<uint8_t> linkId(4);
  std::vector<sva::PTransformd> targetPoses;
  for (uint16_t i = 0; i < posesMsg.joint_names.size(); i++)
  {
    linkId[i] = std::stoi(posesMsg.joint_names[i]);
    if ((linkId[i] < 1) || (linkId[i] > 4))
    {
      ROS_ERROR("toPosesCallback linkId error!");
      return;
    }
#if 0
    std::cout << "posesMsg.transforms[" << i << "]:\n" << posesMsg.transforms[i] << "\n";
#endif

    pose.translation() << posesMsg.transforms[i].translation.x,
        posesMsg.transforms[i].translation.y,
        posesMsg.transforms[i].translation.z;
    q = Eigen::Quaterniond(posesMsg.transforms[i].rotation.w, posesMsg.transforms[i].rotation.x,
                           posesMsg.transforms[i].rotation.y, posesMsg.transforms[i].rotation.z);
    pose.rotation() = q.toRotationMatrix();
    targetPoses.push_back(pose);
  }

  // syncIkModlePose();

  std::vector<rbd::InverseKinematics> linkIk;
  linkIk.push_back(rbd::InverseKinematics(mWalk.talosRobot.talos, mWalk.talosRobot.talos.bodyIndexByName("leftLegLinkSole")));
  linkIk.push_back(rbd::InverseKinematics(mWalk.talosRobot.talos, mWalk.talosRobot.talos.bodyIndexByName("rightLegLinkSole")));
  linkIk.push_back(rbd::InverseKinematics(mWalk.talosRobot.talos, mWalk.talosRobot.talos.bodyIndexByName("leftArmLinkSole")));
  linkIk.push_back(rbd::InverseKinematics(mWalk.talosRobot.talos, mWalk.talosRobot.talos.bodyIndexByName("rightArmLinkSole")));

  for (uint8_t i = 0; i < posesMsg.joint_names.size(); i++)
  {
    if (linkId[i] < 3)
    {
      if (!linkIk[linkId[i] - 1].inverseKinematics(mWalk.talosRobot.talos, mWalk.talosRobot.talosmbc, targetPoses[i]))
      {
        ROS_ERROR("toPosesCallback ik foot error! linkIk[%d].", linkId[i]);
        return;
      }
    }
    else
    {
      if (!linkIk[linkId[i] - 1].inverseKinematics(mWalk.talosRobot.talos, mWalk.talosRobot.talosmbc, targetPoses[i], 3))
      {
        ROS_ERROR("toPosesCallback ik hand error! linkIk[%d].", linkId[i]);
        return;
      }
    }
  }
  mWalk.jointValue.segment(0, 18) = sParamToVector(mWalk.talosRobot.talos, mWalk.talosRobot.talosmbc.q).segment(0, 18) * Util::TO_DEGREE;

  std::vector<uint8_t> idList{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};
  std::vector<double_t> currentAngle(18), targetAngle(18);
  if (!getAngleOfJoint(currentAngle))
  {
    ROS_ERROR("toPosesCallback getAngleOfJoint error!");
    return;
  }
  for (uint8_t i = 0; i < 18; i++)
  {
    targetAngle[i] = mWalk.jointValue[i];
  }
  currentAngle.resize(18);
  toAngleOfJoint(duration, idList, currentAngle, targetAngle);
}

// 提供服务的回调函数
bool SetStatusCallback(bodyhub::SrvState::Request &req,
                       bodyhub::SrvState::Response &res)
{
  bodyhub::SrvState bodyhubStateSrv;
  bodyhubStateSrv.request.masterID = req.masterID;
  bodyhubStateSrv.request.stateReq = req.stateReq;

  if ((req.masterID == currentControlId) || (currentControlId == 4))
  {
    if (BodyHubSetStatusClient.call(bodyhubStateSrv))
    {
      if (req.stateReq == "setStatus")
      {
        if (bodyhubStateSrv.response.stateRes == StateEnum::ready)
        {
          currentControlId = req.masterID;
          bodyhubControlId = req.masterID;
          syncIkModlePose();
          if (ikmoduleState == StateEnum::preReady)
          {
            UpdateState(StateEnum::ready);
          }
        }
        else
        {
          bodyhubControlId = bodyhubStateSrv.response.stateRes;
          ROS_WARN("Failed to setStatus bodyhub, current MasterID is: %d", bodyhubStateSrv.response.stateRes);
        }
      }
      else if (req.stateReq == "break")
      {
        if ((ikmoduleState == StateEnum::running) ||
            (ikmoduleState == StateEnum::pause))
          UpdateState(StateEnum::preReady);
        currentControlId = 4;
      }
      else if (req.stateReq == "stop")
      {
        if (bodyhubStateSrv.response.stateRes == StateEnum::stopping)
        {
          if ((ikmoduleState == StateEnum::running) ||
              (ikmoduleState == StateEnum::pause))
            UpdateState(StateEnum::stopping);
        }
        else
        {
          bodyhubControlId = bodyhubStateSrv.response.stateRes;
          ROS_WARN("Failed to setStatus bodyhub, current MasterID is: %d", bodyhubStateSrv.response.stateRes);
        }
      }
      else if (req.stateReq == "reset")
      {
        if (bodyhubStateSrv.response.stateRes == StateEnum::preReady)
        {
          // 恢复默认ID
          currentControlId = 4;
          bodyhubControlId = 0;
          if (ikmoduleState != StateEnum::stopping)
            UpdateState(StateEnum::preReady);
        }
        else
        {
          bodyhubControlId = bodyhubStateSrv.response.stateRes;
          ROS_WARN("Failed to setStatus bodyhub, current MasterID is: %d", bodyhubStateSrv.response.stateRes);
        }
      }
      res.stateRes = ikmoduleState;
    }
    else
      ROS_ERROR("Bodyhub status service call failed!");
  }
  else
    res.stateRes = currentControlId;

  return true;
}

bool GetStatusCallback(bodyhub::SrvString::Request &req,
                       bodyhub::SrvString::Response &res)
{
  if (req.str == "")
    return false;
  res.data = stateStr;
  res.poseQueueSize = posturesQueue.size();
  return true;
}

bool GetPosesCallback(ik_module::SrvPoses::Request &req,
                      ik_module::SrvPoses::Response &res)
{
  if (syncIkModlePose())
  {
    geometry_msgs::Pose poseMsg;
    Eigen::Matrix4d leftFootPose, rightFootPose, leftHandPose, rightHandPose, pose;
    std::queue<Eigen::Matrix4d> poseHomogeneousQueue;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    Eigen::Quaterniond q;

    leftFootPose = sva::conversions::toHomogeneous(mWalk.talosRobot.talosmbc.bodyPosW[7]);
    rightFootPose = sva::conversions::toHomogeneous(mWalk.talosRobot.talosmbc.bodyPosW[14]);
    leftHandPose = sva::conversions::toHomogeneous(mWalk.talosRobot.talosmbc.bodyPosW[18]);
    rightHandPose = sva::conversions::toHomogeneous(mWalk.talosRobot.talosmbc.bodyPosW[22]);

    poseHomogeneousQueue.push(leftFootPose);
    poseHomogeneousQueue.push(rightFootPose);
    poseHomogeneousQueue.push(leftHandPose);
    poseHomogeneousQueue.push(rightHandPose);

    res.poses.clear();
    while (ros::ok() && !poseHomogeneousQueue.empty())
    {
      pose = poseHomogeneousQueue.front();
      poseHomogeneousQueue.pop();
      rotation = pose.block<3, 3>(0, 0).transpose();
      translation = pose.block<3, 1>(0, 3);
      q = Eigen::Quaterniond(rotation);

      poseMsg.position.x = translation[0];
      poseMsg.position.y = translation[1];
      poseMsg.position.z = translation[2];

      poseMsg.orientation.w = q.w();
      poseMsg.orientation.x = q.x();
      poseMsg.orientation.y = q.y();
      poseMsg.orientation.z = q.z();

      res.poses.push_back(poseMsg);
    }
    return true;
  }
  return false;
}

bool MasterIDSrvCallback(bodyhub::SrvTLSstring::Request &req,
                         bodyhub::SrvTLSstringResponse &res)
{
  ROS_INFO("%s get IkModule masterID: %d", req.str.c_str(), currentControlId);
  res.data = currentControlId;
  return true;
}

// 消息线程处理
void subcribeThread()
{
  ROS_INFO("SubcribeThread initialized!");
  ros::NodeHandle n;
  ros::CallbackQueue topicQueue;

  n.setCallbackQueue(&topicQueue);

  ros::Subscriber BodyHubStatusSub = n.subscribe<std_msgs::UInt16>("MediumSize/BodyHub/Status", 10, BodyHubStatusCallback);
  ros::Subscriber TargetPosesSub = n.subscribe<ik_module::PoseArray>("MediumSize/IKmodule/TargetPoses", 1000, TargetPosesCallback);

  ros::Subscriber toPosesSub = n.subscribe<sensor_msgs::MultiDOFJointState>("MediumSize/IKmodule/toPoses", 1000, toPosesCallback);

  while (n.ok())
  {
    topicQueue.callAvailable(ros::WallDuration(0.01));
  }
}

void ikThread()
{
  ROS_INFO("IkThread initialized!");

  std::vector<double> jointValueVector;
  std::queue<sva::PTransformd> targetPosesQueue;
  sva::PTransformd legLeftPos, legRightPos, armLeftPos, armRightPos;
  rbd::InverseKinematics leftLegIk(mWalk.talosRobot.talos, mWalk.talosRobot.talos.bodyIndexByName("leftLegLinkSole"));
  rbd::InverseKinematics rightLegIk(mWalk.talosRobot.talos, mWalk.talosRobot.talos.bodyIndexByName("rightLegLinkSole"));
  rbd::InverseKinematics leftArmIk(mWalk.talosRobot.talos, mWalk.talosRobot.talos.bodyIndexByName("leftArmLinkSole"));
  rbd::InverseKinematics rightArmIk(mWalk.talosRobot.talos, mWalk.talosRobot.talos.bodyIndexByName("rightArmLinkSole"));

  mWalk.talosRobot.talosmbc.zero(mWalk.talosRobot.talos);
  rbd::forwardKinematics(mWalk.talosRobot.talos, mWalk.talosRobot.talosmbc);
  // Eigen::Matrix<double, 18, 1> theta;
  // theta << 0, 0, -10, 30, -10, 0, 0, 0, -10, 30, -10, 0, 0, 0, 0, 0, 0, 0;
  // mWalk.talosRobot.talosmbc.q = sVectorToParam(mWalk.talosRobot.talos, theta.segment(0,18)*Util::TO_RADIAN);
  // rbd::forwardKinematics(mWalk.talosRobot.talos, mWalk.talosRobot.talosmbc);
  while (ros::ok())
  {
    // 求运动学逆解
    if (!posturesQueue.empty())
    {
      pthread_mutex_lock(&mtxPQ);
      targetPosesQueue = posturesQueue.front();
      posturesQueue.pop();
      pthread_mutex_unlock(&mtxPQ);

      legLeftPos = targetPosesQueue.front();
      targetPosesQueue.pop();
      if (leftLegIk.inverseKinematics(mWalk.talosRobot.talos, mWalk.talosRobot.talosmbc, legLeftPos))
      {
        mWalk.jointValue.segment(mWalk.LLEG_JOINT_START, mWalk.LLEG_JOINT_NUM) = sParamToVector(mWalk.talosRobot.talos, mWalk.talosRobot.talosmbc.q).segment(0, 6) * Util::TO_DEGREE;
      }
      else
        ROS_WARN("Left leg ik failed!!!!!!!!!!!!!\n");

      legRightPos = targetPosesQueue.front();
      targetPosesQueue.pop();
      if (rightLegIk.inverseKinematics(mWalk.talosRobot.talos, mWalk.talosRobot.talosmbc, legRightPos))
      {
        mWalk.jointValue.segment(mWalk.RLEG_JOINT_START, mWalk.RLEG_JOINT_NUM) = sParamToVector(mWalk.talosRobot.talos, mWalk.talosRobot.talosmbc.q).segment(6, 6) * Util::TO_DEGREE;
      }
      else
        ROS_WARN("Right leg ik failed!!!!!!!!!!!!!\n");

      armLeftPos = targetPosesQueue.front();
      targetPosesQueue.pop();
      if (leftArmIk.inverseKinematics(mWalk.talosRobot.talos, mWalk.talosRobot.talosmbc, armLeftPos, 3))
      {
        mWalk.jointValue.segment(mWalk.LARM_JOINT_START, mWalk.LARM_JOINT_NUM) = sParamToVector(mWalk.talosRobot.talos, mWalk.talosRobot.talosmbc.q).segment(12, 3) * Util::TO_DEGREE;
      }
      else
        ROS_WARN("Left arm ik failed!!!!!!!!!!!!!\n");

      armRightPos = targetPosesQueue.front();
      targetPosesQueue.pop();
      if (rightArmIk.inverseKinematics(mWalk.talosRobot.talos, mWalk.talosRobot.talosmbc, armRightPos, 3))
      {
        mWalk.jointValue.segment(mWalk.RARM_JOINT_START, mWalk.RARM_JOINT_NUM) = sParamToVector(mWalk.talosRobot.talos, mWalk.talosRobot.talosmbc.q).segment(15, 3) * Util::TO_DEGREE;
      }
      else
        ROS_WARN("Right arm ik failed!!!!!!!!!!!!!\n");

      jointValueVector.resize(mWalk.jointValue.size());
      for (uint8_t i = 0; i < 12; i++)
        jointValueVector[i] = mWalk.jointValue[i] * jointDirection[i];

      for (uint8_t i = 12; i < 15; i++)
        jointValueVector[i] = mWalk.jointValue[i + 3] * jointDirection[i + 3];

      for (uint8_t i = 15; i < 18; i++)
        jointValueVector[i] = mWalk.jointValue[i - 3] * jointDirection[i - 3];

      pthread_mutex_lock(&mtxJVQ);
      jointValuesQueue.push(jointValueVector);
      pthread_mutex_unlock(&mtxJVQ);

      mWalk.talosRobot.talosmbc.q = sVectorToParam(mWalk.talosRobot.talos, mWalk.jointValue.segment(0, 18) * Util::TO_RADIAN);
      rbd::forwardKinematics(mWalk.talosRobot.talos, mWalk.talosRobot.talosmbc);
    }
  }
}

void publishThread()
{
  ROS_INFO("PublishThread initialized!");
  bodyhub::JointControlPoint jointValueMsg;
  std::vector<double> jointValueVector;

  while (ros::ok())
  {
    if (!jointValuesQueue.empty())
    {
      // 下发数据
      if (bodyhubControlId == currentControlId)
      {
        pthread_mutex_lock(&mtxJVQ);
        jointValueVector = jointValuesQueue.front();
        jointValuesQueue.pop();
        pthread_mutex_unlock(&mtxJVQ);

        jointValueMsg.mainControlID = currentControlId;
        for (auto iter = jointValueVector.begin(); iter != jointValueVector.end(); ++iter)
          jointValueMsg.positions.push_back(*iter); // 发布关节角度(角度制)
        MotoPositionPub.publish(jointValueMsg);

        jointValueMsg.positions.clear(); // 发送消息后清空消息队列
      }
    }
  }
}

void AdvertiseInit(ros::NodeHandle n)
{
  BodyHubSetStatusClient = n.serviceClient<bodyhub::SrvState>("MediumSize/BodyHub/StateJump");
  BodyHubMasterIDClient = n.serviceClient<bodyhub::SrvTLSstring>("MediumSize/BodyHub/GetMasterID");

  bodyhubStatusClient = n.serviceClient<bodyhub::SrvString>("MediumSize/BodyHub/GetStatus");
  bodyhubJointAngleClient = n.serviceClient<bodyhub::SrvServoAllRead>("MediumSize/BodyHub/GetJointAngle");

  MotoPositionPub = n.advertise<bodyhub::JointControlPoint>("MediumSize/BodyHub/MotoPosition", 1000);
  jointCmdPub = n.advertise<sensor_msgs::JointState>("MediumSize/BodyHub/jointControl", 1000);

  SetStatusService = n.advertiseService("MediumSize/IKmodule/SetStatus", SetStatusCallback);
  GetStatusService = n.advertiseService("MediumSize/IKmodule/GetStatus", GetStatusCallback);
  MasterIDService = n.advertiseService("MediumSize/IKmodule/GetMasterID", MasterIDSrvCallback);
  GetPosesService = n.advertiseService("MediumSize/IKmodule/GetPoses", GetPosesCallback);
}

void UpdateState(uint8_t stateNew)
{
  std_msgs::UInt16 ikmoduleStateMsg;

  if (stateNew == StateEnum::preReady)
  {
    ClearQueue(posturesQueue);
    ClearQueue(jointValuesQueue);
  }

  ikmoduleState = stateNew;
  ikmoduleStateMsg.data = stateNew;
  StatusPub.publish(ikmoduleStateMsg);

  switch (stateNew)
  {
  case StateEnum::init:
    stateStr = "init";
    break;
  case StateEnum::preReady:
    stateStr = "preReady";
    break;
  case StateEnum::ready:
    stateStr = "ready";
    break;
  case StateEnum::running:
    stateStr = "running";
    break;
  case StateEnum::pause:
    stateStr = "pause";
    break;
  case StateEnum::stopping:
    stateStr = "stopping";
    break;
  case StateEnum::error:
    stateStr = "error";
    break;
  default:
    break;
  }

  ROS_INFO("The new IkModule State: %s--%d ", stateStr.c_str(), stateNew);
}

void STATEinit(ros::NodeHandle nh)
{
  StatusPub = nh.advertise<std_msgs::UInt16>("MediumSize/IKmodule/Status", 10);

  UpdateState(StateEnum::init);
  AdvertiseInit(nh);

  pthread_mutex_init(&mtxPQ, NULL);
  pthread_mutex_init(&mtxJVQ, NULL);

  ROS_INFO("request for joint angle...");
  waitBodyhubReady();
  syncIkModlePose();
  ROS_INFO("joint angle reciveed.");

  UpdateState(StateEnum::preReady);
}

void STATEpreReady() {}

void STATEready() { const char *log = NULL; }

void STATErunning()
{
  // Check empty & Moving
  if (posturesQueue.empty())
    UpdateState(StateEnum::pause); //更新下一个状态
}
void STATEpause() { const char *log = NULL; }

void STATEstopping()
{
  ClearQueue(posturesQueue);
  ClearQueue(jointValuesQueue);

  UpdateState(StateEnum::ready);
}

void STATEerror() { UpdateState(StateEnum::preReady); }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ikmodule");
  ros::NodeHandle nh;

  STATEinit(nh);
  boost::thread receiveThread(subcribeThread);
  boost::thread ikSolutionThread(ikThread);
  boost::thread sendThread(publishThread);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    if (ikmoduleState == StateEnum::preReady)
      STATEpreReady();
    else if (ikmoduleState == StateEnum::ready)
      STATEready();
    else if (ikmoduleState == StateEnum::running)
      STATErunning();
    else if (ikmoduleState == StateEnum::pause)
      STATEpause();
    else if (ikmoduleState == StateEnum::stopping)
      STATEstopping();
    else if (ikmoduleState == StateEnum::error)
      STATEerror();

    ros::spinOnce();
    loop_rate.sleep();
  }

  receiveThread.join();
}
