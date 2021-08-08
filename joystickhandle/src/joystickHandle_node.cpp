
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

#include <pthread.h>
#include <Eigen/Dense>

#define JOY_A 0
#define JOY_B 1
#define JOY_X 2
#define JOY_Y 3
#define JOY_LB 4
#define JOY_RB 5
#define JOY_BACK 6
#define JOY_START 7

#define JOY_L_R_AXIS_LIFE 0
#define JOY_U_D_AXIS_LIFE 1
#define JOY_L_R_AXIS_RIGHT 3
#define JOY_U_D_AXIS_RIGHT 4

#define GAIT_X_RANGE (0.10)
#define GAIT_B_RANGE (GAIT_X_RANGE * 0.8) // back
#define GAIT_Y_RANGE (0.04)
#define GAIT_A_RANGE (10.0) // yaw

#define JOY_X_RANGE (1.0)
#define JOY_Y_RANGE (1.0)
#define JOY_A_RANGE (1.0) // yaw

#define JOY_X_THRESHOLD (0.2)
#define JOY_Y_THRESHOLD (0.2)
#define JOY_A_THRESHOLD (0.2) // yaw

#define X_COEFFICIENT ((GAIT_X_RANGE) / (JOY_X_RANGE - JOY_X_THRESHOLD))
#define B_COEFFICIENT ((GAIT_B_RANGE) / (JOY_X_RANGE - JOY_X_THRESHOLD))
#define Y_COEFFICIENT ((GAIT_Y_RANGE) / (JOY_Y_RANGE - JOY_Y_THRESHOLD))
#define A_COEFFICIENT ((GAIT_A_RANGE) / (JOY_A_RANGE - JOY_A_THRESHOLD))

ros::Publisher GaitCommandPub;
ros::Subscriber ReqGaitCommandSub;
ros::Subscriber JoytickSub;
std_msgs::Float64MultiArray GaitCommandMsg;
uint8_t JoyLockStatus = 1;

void reqGaitCommandCallback(const std_msgs::Bool::ConstPtr &req)
{
  if ((JoyLockStatus == 1) && (req->data == 1))
  {
    if ((GaitCommandMsg.data[0] != 0) || (GaitCommandMsg.data[1] != 0) || (GaitCommandMsg.data[2] != 0))
      GaitCommandPub.publish(GaitCommandMsg);
  }
}

void JoyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  if ((msg->buttons[JOY_LB] == 1) || (msg->buttons[JOY_RB] == 1))
  {
    JoyLockStatus = !JoyLockStatus;
    ROS_INFO("JoyLockStatus: %d\n", JoyLockStatus);
  }

  if (JoyLockStatus)
  {
    if (msg->axes[JOY_U_D_AXIS_LIFE] > JOY_X_THRESHOLD)
      GaitCommandMsg.data[0] = (msg->axes[JOY_U_D_AXIS_LIFE] - JOY_X_THRESHOLD) * X_COEFFICIENT;
    else if (msg->axes[JOY_U_D_AXIS_LIFE] < -JOY_X_THRESHOLD)
      GaitCommandMsg.data[0] = (msg->axes[JOY_U_D_AXIS_LIFE] + JOY_X_THRESHOLD) * B_COEFFICIENT;
    else
      GaitCommandMsg.data[0] = 0.0;

    if (msg->axes[JOY_L_R_AXIS_LIFE] > JOY_A_THRESHOLD)
      GaitCommandMsg.data[2] = (msg->axes[JOY_L_R_AXIS_LIFE] - JOY_A_THRESHOLD) * A_COEFFICIENT;
    else if (msg->axes[JOY_L_R_AXIS_LIFE] < -JOY_A_THRESHOLD)
      GaitCommandMsg.data[2] = (msg->axes[JOY_L_R_AXIS_LIFE] + JOY_A_THRESHOLD) * A_COEFFICIENT;
    else
      GaitCommandMsg.data[2] = 0.0;

    if (msg->axes[JOY_L_R_AXIS_RIGHT] > JOY_Y_THRESHOLD)
      GaitCommandMsg.data[1] = (msg->axes[JOY_L_R_AXIS_RIGHT] - JOY_Y_THRESHOLD) * Y_COEFFICIENT;
    else if (msg->axes[JOY_L_R_AXIS_RIGHT] < -JOY_Y_THRESHOLD)
      GaitCommandMsg.data[1] = (msg->axes[JOY_L_R_AXIS_RIGHT] + JOY_Y_THRESHOLD) * Y_COEFFICIENT;
    else
      GaitCommandMsg.data[1] = 0.0;
#if 0
    ROS_INFO("GaitCommandMsg.data: %f,%f,%f\n", GaitCommandMsg.data[0], GaitCommandMsg.data[1], GaitCommandMsg.data[2]);
#endif
  }
}

int main(int argc, char **argv)
{
  GaitCommandMsg.data.resize(3);

  ros::init(argc, argv, "JoystickHandle");
  ros::NodeHandle nh;
  JoytickSub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &JoyCallback);
  ReqGaitCommandSub = nh.subscribe<std_msgs::Bool>("requestGaitCommand", 1, &reqGaitCommandCallback);
  GaitCommandPub = nh.advertise<std_msgs::Float64MultiArray>("gaitCommand", 1);

  ROS_INFO("JoyLockStatus: %d\n", JoyLockStatus);

  ros::Rate loopRate(1000);
  while (ros::ok())
  {
    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}