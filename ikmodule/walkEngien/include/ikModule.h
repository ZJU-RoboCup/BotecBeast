#include <Eigen/Dense>
#include <boost/thread.hpp>
#include <iostream>
#include <queue>

// LIPM
#include "../include/walkPlaning/LIPMWalk.h"

// rbd
#include "CoM.h"
#include "FK.h"
#include "FV.h"
#include "IK.h"
#include "Util.h"

// sva
#include "Conversions.h"

// ROS
#include "ros/callback_queue.h"
#include "ros/ros.h"

// msg & srv
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/MultiDOFJointState.h"
#include "geometry_msgs/Transform.h"
#include "std_srvs/SetBool.h"
#include "bodyhub/JointControlPoint.h"
#include "bodyhub/ServoPositionAngle.h"
#include "bodyhub/SrvServoAllRead.h"
#include "bodyhub/SrvState.h"
#include "bodyhub/SrvString.h"
#include "bodyhub/SrvTLSstring.h"
#include "ik_module/PoseArray.h"
#include "ik_module/SrvPoses.h"
#include "std_msgs/UInt16.h"

using namespace GaitManager;
using namespace std;

// ///////////////////////////////State Machine////////////////////////////////
namespace StateEnum
{
  enum StateStyle
  {
    init = 20,
    preReady,
    ready,
    running,
    pause,
    stopping, // stopping
    error,
    directOperate
  };
}

void STATEinit(ros::NodeHandle nh);
void STATEpreReady();
void STATEready();
void STATErunning();
void STATEpause();
void STATEstopping();
void STATEerror();

void UpdateState(uint8_t stateNew);

// ///////////////////////////////Thread//////////////////////////////////////
void subcribeThread(); // subcribeThread()
void ikThread();       // ikSolutionThread()
void publishThread();  // publishThread()

// 清空队列
template <typename T>
void ClearQueue(T &Qtempt)
{
  if (!Qtempt.empty())
  {
    T empty;
    swap(empty, Qtempt);
  }
}