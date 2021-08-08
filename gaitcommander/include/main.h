#include <iostream> 
#include <pthread.h>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include <Eigen/Dense>  

using namespace std;

int countCommand = 0;
// ///////////////////////////////ROS//////////////////////////////////////////
ros::Publisher gaitCommand_pub;
ros::Subscriber reqGaitCommand_sub;

// ///////////////////////////////ROS//////////////////////////////////////////

