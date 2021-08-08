#ifndef  SIMCONTROLL_H
#define  SIMCONTROLL_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>  




namespace SimControll
{
	extern ros::Publisher simStart_pub;
	extern ros::Publisher simStop_pub;
	extern ros::Publisher simPause_pub;
	extern ros::Publisher simEnSync_pub;
	extern ros::Publisher simTrigNext_pub;
	extern ros::Publisher jointCmd_pub;
	extern ros::Subscriber jointAngle_sub;

	extern  bool stepDone;   //一步仿真完成标志
	extern std_msgs::Bool simCtr;


	extern std_msgs::Float64MultiArray jointCmd;


	extern Eigen::Matrix<double, 20, 1> jointAngle,jointVel;
	extern Eigen::Matrix<double, 3, 1> cop;
	extern Eigen::Matrix<double, 2, 1> fsr;
	extern Eigen::Matrix<double, 3, 1> com;
	extern Eigen::Matrix<double, 3, 1> comv;

	struct InertialSensorData
	{

	  Eigen::Vector3d gyro=Eigen::Vector3d::Zero();   /*< The change in orientation around the x-, y-, and z-axis (in radian/s). */
	  Eigen::Vector3d acc=Eigen::Vector3d::Zero();  /**< The acceleration along the x-, y- and z-axis (in m/s^2). */
	};
	extern struct InertialSensorData inertialSensorData;

	void simInit();

	void simControll(Eigen::Matrix<double, 20, 1> jointValue);
}


#endif