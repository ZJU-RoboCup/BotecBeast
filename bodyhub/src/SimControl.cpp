#include "bodyhub/bodyhub.h"
// #include "Util.h"

extern ServoStore_s ServoStore;

namespace SimControll {

ros::Publisher JointCmdPub;
ros::Publisher SimStartPub;
ros::Publisher SimStopPub;
ros::Publisher SimPausePub;
ros::Publisher SimEnSyncPub;
ros::Publisher SimTrigNextPub;
ros::Subscriber SimStepDoneSub;
ros::Subscriber SimStateSub;

ros::Subscriber SimJointPositionSub;
ros::Subscriber SimJointVelocitySub;
ros::Subscriber SimLeftFTSub;
ros::Subscriber SimRightFTSub;

pthread_mutex_t mtxJV;

bool stepDone = false;   //一步仿真完成标志
bool simEnable = false;  //仿真启动标志
bool getJointState = false;
int8_t simState = 0;
std_msgs::Bool simCtrMsg;
std::queue<std::vector<double>> jointCmdQueue;

ClassRobotData SimRobotData;

std_msgs::Float64MultiArray entityToSimOfIndex(
    std_msgs::Float64MultiArray entity) {
  uint8_t i = 0;
  std_msgs::Float64MultiArray sim;
  sim.data.resize(22);

  for (i = 0; i < 12; i++) {
    sim.data[i] = entity.data[i];
  }
  for (i = 12; i < 15; i++) {
    sim.data[i] = entity.data[i + 3];
  }
  sim.data[15] = entity.data[19];
  for (i = 16; i < 19; i++) {
    sim.data[i] = entity.data[i - 4];
  }
  sim.data[19] = entity.data[18];
  for (i = 20; i < 22; i++) {
    sim.data[i] = entity.data[i];
  }
  return sim;
}

std_msgs::Float64MultiArray simToEntityOfIndex(
    std_msgs::Float64MultiArray sim) {
  uint8_t i = 0;
  std_msgs::Float64MultiArray entity;
  entity.data.resize(22);

  for (i = 0; i < 12; i++) {
    entity.data[i] = sim.data[i];
  }
  for (i = 12; i < 15; i++) {
    entity.data[i] = sim.data[i + 4];
  }
  for (i = 15; i < 18; i++) {
    entity.data[i] = sim.data[i - 3];
  }
  entity.data[18] = sim.data[19];
  entity.data[19] = sim.data[15];
  for (i = 20; i < 22; i++) {
    entity.data[i] = sim.data[i];
  }
  return entity;
}

bool jointState() { return getJointState; }

void SimJointPositionCallback(
    const std_msgs::Float64MultiArray::ConstPtr &msg) {
  SimRobotData.setJointPosition(simToEntityOfIndex(*msg));
  if (!getJointState) getJointState = true;
}

void SimJointVelocityCallback(
    const std_msgs::Float64MultiArray::ConstPtr &msg) {
  SimRobotData.setJointVelocity(simToEntityOfIndex(*msg));
}

void SimLeftFootFTCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
  SimRobotData.setLeftFT(*msg);
}

void SimRightFootFTCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
  SimRobotData.setRightFT(*msg);
}

void SimulateRobotTopicInit(ros::NodeHandle nh) {
  SimJointPositionSub = nh.subscribe<std_msgs::Float64MultiArray>(
      "/sim/joint/angle", 1, &SimJointPositionCallback);
  SimJointVelocitySub = nh.subscribe<std_msgs::Float64MultiArray>(
      "/sim/joint/velocity", 1, &SimJointVelocityCallback);
  SimLeftFTSub = nh.subscribe<std_msgs::Float64MultiArray>(
      "/sim/force/leftFoot", 1, &SimLeftFootFTCallback);
  SimRightFTSub = nh.subscribe<std_msgs::Float64MultiArray>(
      "/sim/force/rightFoot", 1, &SimRightFootFTCallback);

  JointCmdPub =
      nh.advertise<std_msgs::Float64MultiArray>("/sim/joint/command", 1);
}

void simInit(ros::NodeHandle nh) {
  pthread_mutex_init(&mtxJV, NULL);

  SimStepDoneSub = nh.subscribe<std_msgs::Bool>("simulationStepDone", 10,
                                                &SimStepDoneCallback);
  SimStateSub =
      nh.subscribe<std_msgs::Int32>("simulationState", 1, &SimStateCallback);

  SimStartPub = nh.advertise<std_msgs::Bool>("startSimulation", 1);
  SimStopPub = nh.advertise<std_msgs::Bool>("stopSimulation", 1);
  SimPausePub = nh.advertise<std_msgs::Bool>("pauseSimulation", 1);
  SimEnSyncPub = nh.advertise<std_msgs::Bool>("enableSyncMode", 1);
  SimTrigNextPub = nh.advertise<std_msgs::Bool>("triggerNextStep", 1);

  SimulateRobotTopicInit(nh);

  //等待发布者与接收者建立连接
  ROS_INFO("Waiting for connection with Vrep......");
  while (ros::ok() && (SimStartPub.getNumSubscribers() <= 0 ||
                       SimEnSyncPub.getNumSubscribers() <= 0 ||
                       SimTrigNextPub.getNumSubscribers() <= 0))
    ;
  ROS_INFO("Connection with Vrep completed!!!");
  simCtrMsg.data = 1;               //仿真控制变量
  SimEnSyncPub.publish(simCtrMsg);  //开启vrep同步模式
  SimStartPub.publish(simCtrMsg);   //开始vrep仿真
}

void SimStepDoneCallback(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data) stepDone = true;
}

void SimStateCallback(const std_msgs::Int32::ConstPtr &msg) {
  // pause--2 start--1
  simState = msg->data;
  // ROS_INFO("Simulation state update: %d", msg->data);
}

void updateJointCmdQueue(const std::vector<double> &jointPositions) {
  pthread_mutex_lock(&mtxJV);
  jointCmdQueue.push(jointPositions);  // radian vector
  pthread_mutex_unlock(&mtxJV);
}

void simStart() {
  simCtrMsg.data = 1;              //仿真控制变量
  SimStartPub.publish(simCtrMsg);  //开始vrep仿真
  ROS_INFO("Simulation in Vrep started!");
}

void simStop() {
  simCtrMsg.data = 1;             //仿真控制变量
  SimStopPub.publish(simCtrMsg);  //停止vrep仿真
  ROS_INFO("Simulation in Vrep stopped!");
}

void simPause() {
  simCtrMsg.data = 1;              //仿真控制变量
  SimPausePub.publish(simCtrMsg);  //暂停vrep仿真
  ROS_INFO("Simulation in Vrep paused!");
}

void simJoinPositionPublish(std_msgs::Float64MultiArray jointValue) {
  std::vector<double> jointPosition;
  std_msgs::Float64MultiArray getPosition =
      SimRobotData.getJointPostion();  // radian

  if (jointValue.data.size() < 22) {
    jointValue.data.resize(22);
    jointValue.data[18] = getPosition.data[18];
    jointValue.data[19] = getPosition.data[19];
  }
  jointValue.data.resize(22);

  for (uint8_t i = 0; i < SERVO_NUM; i++)
    ServoStore.angle[i] = jointValue.data[i] * Util::TO_DEGREE;
  JointCmdPub.publish(entityToSimOfIndex(jointValue));  // 转发数据

  bodyhub::ServoPositionAngle servoPositionsMsg;
  servoPositionsMsg.angle = ServoStore.angle;
  ServoPositionPub.publish(servoPositionsMsg);
}

void jointTrajQueuePoll() {
  if (jointTrajQueue.size() > 0) {
    sensor_msgs::JointState jointStateMsg;

    mtxJointTrajQueue.lock();
    jointStateMsg = jointTrajQueue.front();
    jointTrajQueue.pop();
    mtxJointTrajQueue.unlock();

    uint8_t number = jointStateMsg.name.size();
    uint8_t idList[SERVO_NUM] = {0};
    std::vector<double> radian;

    for (uint8_t i = 0; i < SERVO_NUM; i++) {
      radian.push_back(ServoStore.angle[i] * Util::TO_RADIAN);
    }

    for (uint8_t i = 0; i < number; i++) {
      idList[i] = std::stoi(jointStateMsg.name[i]);
      radian[idList[i] - 1] = jointStateMsg.position[i] * Util::TO_RADIAN;
    }
    updateJointCmdQueue(radian);
  }
}

void JointControlMessagePoll() {
  if (jointTrajQueue.size() > 0) {
    jointTrajQueuePoll();
  } else if ((motoQueue.size() > 0) || (headCtrlQueue.size() > 0)) {
    if (motoQueue.size() > 0) {
      bodyhub::JointControlPoint jointControlMsg;
      pthread_mutex_lock(&mtxMo);
      jointControlMsg = motoQueue.front();
      motoQueue.pop();
      pthread_mutex_unlock(&mtxMo);
      for (uint8_t i = 0; i < jointControlMsg.positions.size(); i++)
        jointControlMsg.positions[i] =
            Angle2Radian(jointControlMsg.positions[i]);
      updateJointCmdQueue(jointControlMsg.positions);
    }
    if (headCtrlQueue.size() > 0) {
      bodyhub::JointControlPoint headControlMsg;
      pthread_mutex_lock(&mtxHe);
      headControlMsg = headCtrlQueue.front();
      headCtrlQueue.pop();
      pthread_mutex_unlock(&mtxHe);
      for (uint8_t i = 0; i < headControlMsg.positions.size(); i++)
        headControlMsg.positions[i] = Angle2Radian(headControlMsg.positions[i]);
      updateJointCmdQueue(headControlMsg.positions);
    }
  }
}

void simThread() {
  if (!SimControll::simEnable) return;

  ROS_INFO("Start 'simThread' thrand...");

  std_msgs::Float64MultiArray jointCmd;
  stepDone = true;
  ros::Rate loop_rate(1000);

  while (ros::ok()) {
    if (stepDone == true) {
      if (jointCmdQueue.empty()) {
        if (bodyhubState == StateEnum::walking)
          control_thread_robot();
        else
          JointControlMessagePoll();
      }
      if (!jointCmdQueue.empty()) {
        pthread_mutex_lock(&mtxJV);
        jointCmd.data.assign(jointCmdQueue.front().begin(),
                             jointCmdQueue.front().end());
        jointCmdQueue.pop();
        pthread_mutex_unlock(&mtxJV);
        simJoinPositionPublish(jointCmd);
      }

      stepDone = false;
      SimTrigNextPub.publish(
          simCtrMsg);  // Vrep仿真一步,大约要150ms才能收到simulationStepDone消息
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("'simThread' thrand exit");
}

}  // namespace SimControll