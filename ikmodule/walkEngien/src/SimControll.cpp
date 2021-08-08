#include "SimControll.h"
#include "Util.h"

// #include "CPWalking6.h"


namespace SimControll
{
	ros::Subscriber simStepDone_sub;
	ros::Subscriber simState_sub;
	ros::Publisher simStart_pub;
	ros::Publisher simStop_pub;
	ros::Publisher simPause_pub;
	ros::Publisher simEnSync_pub;
	ros::Publisher simTrigNext_pub;
	ros::Publisher jointCmd_pub;

	ros::Publisher torsoAngle_pub;
	ros::Publisher comEsti_pub;

	ros::Publisher trigForce_pub;

	ros::Subscriber jointAngle_sub;
	ros::Subscriber jointVel_sub;
	ros::Subscriber leftFT_sub;
	ros::Subscriber rightFT_sub;
	ros::Subscriber cop_sub;
	ros::Subscriber fsr_sub;
	ros::Subscriber com_sub;
	ros::Subscriber comv_sub;
	ros::Subscriber Imu_sub;

	bool stepDone;   //一步仿真完成标志
	std_msgs::Bool simCtr;
	std_msgs::Bool trigF;


	struct InertialSensorData inertialSensorData; 


	std_msgs::Float64MultiArray jointCmd;
	std_msgs::Float64MultiArray torsoAngle;
	std_msgs::Float64MultiArray comEsti;

	Eigen::Matrix<double, 20, 1> jointAngle,jointVel;
	Eigen::Matrix<double, 3, 1> cop;
	Eigen::Matrix<double, 2, 1> fsr;
	Eigen::Matrix<double, 3, 1> com;
	Eigen::Matrix<double, 3, 1> comv;

	void simStateCallback(const std_msgs::Int32::ConstPtr& state)
	{
		// std_msgs::Bool start;
		// std::cout<<"here:" << std::endl;
		// if(state->data == 0)
		// {
		// 	start.data=1;
		// 	start_pub.publish(start);
		// }
	}

	void simStepDoneCallback(const std_msgs::Bool::ConstPtr& done)
	{
		stepDone=true;
	}

	void jointAngleCallback(const std_msgs::Float64MultiArray::ConstPtr& joint)
	{
		for(int i = 0; i < 20; ++i)
			jointAngle[i]=joint->data[i];
		// std::cout<< jointAngle[2] <<std::endl;
		// std::cout<< "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" <<std::endl;
	}

	void jointVelCallback(const std_msgs::Float64MultiArray::ConstPtr& vel)
	{
		for(int i = 0; i < 20; ++i)
			jointVel[i]=vel->data[i];
		// std::cout<< jointSensor[1] <<std::endl;
		// std::cout<< "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" <<std::endl;
	}


	void copCallback(const std_msgs::Float64MultiArray::ConstPtr& fcop)
	{
		for(int i = 0; i < 3; ++i)
			cop[i]=fcop->data[i]; 
		// std::cout<< cop <<std::endl;
		// std::cout<< "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" <<std::endl;
	}
	void fsrCallback(const std_msgs::Float64MultiArray::ConstPtr& fsr1)
	{
		for(int i = 0; i < 2; ++i)
			fsr[i]=fsr1->data[i]; 
		// std::cout<< "fsr:"<<fsr.transpose() <<std::endl;
		// std::cout<< "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" <<std::endl;
	}
	void comCallback(const std_msgs::Float64MultiArray::ConstPtr& com1)
	{
		for(int i = 0; i < 3; ++i)
			com[i]=com1->data[i];
		// std::cout<< com <<std::endl;
		// std::cout<< "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" <<std::endl;
	}
	void comvCallback(const std_msgs::Float64MultiArray::ConstPtr& comv1)
	{
		for(int i = 0; i < 3; ++i)
			comv[i]=comv1->data[i];	
		// std::cout<< comv <<std::endl;
		// std::cout<< "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" <<std::endl;
	}

	void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu)
	{
		if(std::abs(imu->angular_velocity.x)<10 && std::abs(imu->angular_velocity.y)<10 && std::abs(imu->angular_velocity.z)<10)
		{
			inertialSensorData.gyro.x()=imu->angular_velocity.x;
			inertialSensorData.gyro.y()=imu->angular_velocity.y;
			inertialSensorData.gyro.z()=imu->angular_velocity.z;
		}
		if(std::abs(imu->linear_acceleration.x)<20 && std::abs(imu->linear_acceleration.y)<20 && std::abs(imu->linear_acceleration.z)<20)
		{
			inertialSensorData.acc.x()=imu->linear_acceleration.x;
			inertialSensorData.acc.y()=imu->linear_acceleration.y;
			inertialSensorData.acc.z()=imu->linear_acceleration.z;
		}
		

	
		// std::cout<< "acc: " << inertialSensorData.acc.transpose() <<std::endl;
		// std::cout<< "gyro: " << inertialSensorData.gyro.transpose() <<std::endl;
	}



	
	void simInit()
	{
	    ros::NodeHandle node;

	    simStepDone_sub=node.subscribe<std_msgs::Bool>("simulationStepDone",1,&simStepDoneCallback);
		simState_sub=node.subscribe<std_msgs::Int32>("simulationState",1,&simStateCallback);
		simStart_pub=node.advertise<std_msgs::Bool>("startSimulation",1);
		simStop_pub=node.advertise<std_msgs::Bool>("stopSimulation",1);
		simPause_pub=node.advertise<std_msgs::Bool>("pauseSimulation",1);
		simEnSync_pub=node.advertise<std_msgs::Bool>("enableSyncMode",1);
		simTrigNext_pub=node.advertise<std_msgs::Bool>("triggerNextStep",1);
	    jointCmd_pub=node.advertise<std_msgs::Float64MultiArray>("joint_command",1);
	    torsoAngle_pub=node.advertise<std_msgs::Float64MultiArray>("torso_angle",1);
	    comEsti_pub=node.advertise<std_msgs::Float64MultiArray>("com_esti",1);
	    trigForce_pub=node.advertise<std_msgs::Bool>("trig_force",1);
 

		jointAngle_sub=node.subscribe<std_msgs::Float64MultiArray>("joint_angle",10,&jointAngleCallback);
		jointVel_sub=node.subscribe<std_msgs::Float64MultiArray>("joint_velocity",10,&jointVelCallback);
		fsr_sub=node.subscribe<std_msgs::Float64MultiArray>("fsr",10,&fsrCallback);
		cop_sub=node.subscribe<std_msgs::Float64MultiArray>("cop",10,&copCallback);
		com_sub=node.subscribe<std_msgs::Float64MultiArray>("com",10,&comCallback);
		comv_sub=node.subscribe<std_msgs::Float64MultiArray>("comv",10,&comvCallback);
		Imu_sub=node.subscribe<sensor_msgs::Imu>("vrep/imu",100,&ImuCallback);
	    while(simStart_pub.getNumSubscribers() <= 0 ||simEnSync_pub.getNumSubscribers() <= 0 ||simTrigNext_pub.getNumSubscribers() <= 0);   //等待发布者与接收者建立连接
		
		jointCmd.data.resize(20);
		simCtr.data=1;                  //仿真控制变量
		simEnSync_pub.publish(simCtr);  //开启vrep同步模式
		simStart_pub.publish(simCtr);  //开始vrep仿真

		torsoAngle.data.resize(2);
		comEsti.data.resize(4);
	
	}


	void simControll(Eigen::Matrix<double, 20, 1> jointValue)//
	{
	    for(int i = 0; i < 20; ++i)
		{
			jointCmd.data[i]=jointValue[i]*Util::TO_RADIAN;
		}

		// for(int i = 0; i < 2; ++i)
		// {
		// 	torsoAngle.data[i]=GaitManager::CPWalking3::GetInstance()->inertialDataFilter.inertialData.angle[i]*Util::TO_DEGREE;

		// 	comEsti.data[i]=GaitManager::CPWalking3::GetInstance()->lipStateEstimator.getEstimate().com[i];
		// 	comEsti.data[i+2]=GaitManager::CPWalking3::GetInstance()->lipStateEstimator.getEstimate().comVel[i];
		// }
		
		// if(GaitManager::CPWalking4::GetInstance()->trigForce == true)
		// {
		// 	printf("trigforce!\n");
		// 	trigF.data=1; 
		// 	trigForce_pub.publish(trigF);
		// 	GaitManager::CPWalking4::GetInstance()->trigForce = false;
		// }

		stepDone=false;
		simTrigNext_pub.publish(simCtr);   //vrep仿真一步  //大约要150ms才能收到simulationStepDone消息
		while(jointAngle_sub.getNumPublishers () <=0);
		while(com_sub.getNumPublishers () <=0);
		while(comv_sub.getNumPublishers () <=0);
		while(cop_sub.getNumPublishers () <=0);
		// while(Imu_sub.getNumPublishers () <=0);
		while(jointCmd_pub.getNumSubscribers() <= 0);
		jointCmd_pub.publish(jointCmd);

		while(torsoAngle_pub.getNumSubscribers() <= 0);
		torsoAngle_pub.publish(torsoAngle);
		while(comEsti_pub.getNumSubscribers() <= 0);
		comEsti_pub.publish(comEsti);

		while(stepDone == false && ros::ok())  ros::spinOnce();
	}
}

