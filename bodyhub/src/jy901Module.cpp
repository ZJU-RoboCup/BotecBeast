#include <std_msgs/Float64MultiArray.h>
#include "imuData.h"
#include "ros/ros.h"

extern ImuData torsoImu;

void jy901Callback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
  torsoImu.setGyro(msg->data[0], msg->data[1], msg->data[2]);
  torsoImu.setAcc(msg->data[3], msg->data[4], msg->data[5]);
  torsoImu.setAttitude(msg->data[6], msg->data[7], msg->data[8]);
}

void jy901ModuleInit() {
  ros::NodeHandle nh;
  ros::Subscriber jy901_sub =
      nh.subscribe("/jy901Module_node/jy901Data", 1, jy901Callback);
}