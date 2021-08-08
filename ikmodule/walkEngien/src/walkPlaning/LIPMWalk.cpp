 /*
Biped LIPMWalk based LIPM 
LIPMWalk.cpp
*/
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include "LIPMWalk.h"
#include "Kinematics.h"

//sva
#include "Conversions.h"

//rbd
#include "Util.h"
#include "FK.h"
#include "FV.h"
#include "IK.h"
#include "CoM.h"

using namespace GaitManager;
using namespace std;

const double LIPMWalk::Rad2Deg = 57.296;
LIPMWalk::LIPMWalk()
{
	// cpWalk_lipm = CPWalking5::GetInstance();
	footSeprate=0.05475*2+0.04;      	//foot seprate at standing for footprint(plan)
	jointValue.resize(18);
	measuredJointValue.resize(18);
	lastJointValue.resize(18);
	jointVelocity.resize(18);
	lastMeasuredJointValue.resize(18);
	measuredJointVelocity.resize(18);
	footPrint.resize(100);
	plannedCoM_wF.resize(100);
	armJointValue.resize(6);
	// initSquat();	//fftest

	timeStep=0.02;						//s
	double	t = 0.8; // timetime
	double	DSP = 0.2;  //0.2


	for(int i=0;i<1000;i++)
		WayPoint_X0[i] = 0.0;//0.035;
	// WayPoint_X0[5] = 0.0;//0.035;
	for(int i=0;i<1000;i++)
		WayPoint_Y0[i] = footSeprate/2.;
	for(int i=0;i<1000;i++)
		WayPoint_Yaw[i] = 0;

	#ifdef SIM_ROBOT
		Step_TC_x = 8.5; //4.5
		Step_TC_y = 1.2*sqrt(Gravity/torsoHeightWalk);  //1.2
	#else
		Step_TC_x = 4.5; //4.5
		// Step_TC_y = 0.55*sqrt(Gravity/torsoHeightWalk);//1.0s
		Step_TC_y = 0.85*sqrt(Gravity/torsoHeightWalk);//0.6s
	#endif

	//步态参数第一次初始化
	StepCountTarget = 0;
	T_circle = t*(1-DSP);
	T_dsp = t*DSP;
	Swing_H = 0.04;

	DSPRatio = DSP;
	
}

LIPMWalk::~LIPMWalk() {

}

void LIPMWalk::simStart()
{
	SimControll::simInit();
}

void LIPMWalk::initSquat(std::vector< Eigen::VectorXd > &squatSequence)
{
		
	Eigen::Matrix4d poseLeft,poseRight,armposeLeft,armposeRight;
	// Eigen::Matrix4d center_pose;
	// bool successLeft,successRight;
	double height,width,xcom;
	double centerX=0.0226, centerY=0.09543, centerZ=0.0454, radius=0.13153; // 手臂移动的重心坐标和半径

	talosRobot.talosmbc.zero(talosRobot.talos);
	rbd::forwardKinematics(talosRobot.talos, talosRobot.talosmbc);
	poseLeft=sva::conversions::toHomogeneous(talosRobot.talosmbc.bodyPosW[7]);
	poseRight=sva::conversions::toHomogeneous(talosRobot.talosmbc.bodyPosW[14]);

	// center_pose=sva::conversions::toHomogeneous(talosRobot.talosmbc.bodyPosW[16]);
    armposeLeft=sva::conversions::toHomogeneous(talosRobot.talosmbc.bodyPosW[18]);
    armposeRight=sva::conversions::toHomogeneous(talosRobot.talosmbc.bodyPosW[22]);

    // // 基于重心世界坐标系的相对位置
	// std::cout << "poseLeft:\n"<<poseLeft<<std::endl;
    // std::cout << "poseRight:\n"<<poseRight<<std::endl;
	// std::cout << "center_pose:\n"<<center_pose<<std::endl;	
	// std::cout << "armposeLeft:\n"<<armposeLeft<<std::endl;
    // std::cout << "armposeRight:\n"<<armposeRight<<std::endl;
	// getchar();

	torsoHeight=-poseLeft(2,3);    // mm to m
	width=poseLeft(1,3);
	xcom=poseLeft(0,3);

	torsoHeightWalk=torsoHeight-0.012;  	//torso height at walking 

	// std::cout<< "poseLeft:" <<std::endl<< poseLeft <<std::endl;
	// std::cout<< "poseRight:" <<std::endl<< poseRight <<std::endl;
	
	Eigen::Matrix<double,18,1> theta;
	theta<<0,0,-10,30,-10,0,       0,0,-10,30,-10,0,    0,80,0,    0,-80,0;
	talosRobot.talosmbc.q=sVectorToParam(talosRobot.talos,theta.segment(0,18)*Util::TO_RADIAN);

	for(int j=0;j<=50;j++)
	{
		poseLeft(0,3)=xcom-(xcom)*j/50.;    //let torso center at foot center in forward direction
		poseRight(0,3)=xcom-(xcom)*j/50.;

		poseLeft(2,3)=-(torsoHeight-(torsoHeight-torsoHeightWalk)*j/50.);  //squat down to torsoHeightWalk
		poseRight(2,3)=-(torsoHeight-(torsoHeight-torsoHeightWalk)*j/50.);

		poseLeft(1,3)=width+((footSeprate/2-width))*j/50.;
		poseRight(1,3)=-width-((footSeprate/2-width))*j/50.;


		// 目标坐标为相对于重心(世界坐标系)的相对位置
		// 手臂的运动轨迹为以center_pose为圆心，圆心角为80度的圆弧,其中X不变
		armposeLeft(0,3) = armposeLeft(0,3);
		armposeRight(0,3)= armposeRight(0,3);
	
		armposeLeft(1,3) =  centerY + radius*cos((j*80/50.)*Util::TO_RADIAN);
		armposeRight(1,3)= -centerY - radius*cos((j*80/50.)*Util::TO_RADIAN);

		armposeLeft(2,3) =  centerZ - radius*sin((j*80/50.)*Util::TO_RADIAN);  
		armposeRight(2,3)=  centerZ - radius*sin((j*80/50.)*Util::TO_RADIAN);

		// Left Leg Inverse Kinematics
		rbd::InverseKinematics leftLegIk(talosRobot.talos,talosRobot.talos.bodyIndexByName("leftLegLinkSole"));
		sva::PTransformd leftPos;
		leftPos.rotation()=poseLeft.block<3,3>(0,0).transpose();
		leftPos.translation()=poseLeft.block<3,1>(0,3);
		if(leftLegIk.inverseKinematics(talosRobot.talos,talosRobot.talosmbc,leftPos)) 
		{
			jointValue.segment(LLEG_JOINT_START,LLEG_JOINT_NUM)=sParamToVector(talosRobot.talos,talosRobot.talosmbc.q).segment(0,6)*Util::TO_DEGREE;
			// cout<<"leftJoint:\n"<<jointValue.segment(LLEG_JOINT_START,LLEG_JOINT_NUM)<<endl<<endl<<endl;//FIXME:
		}
		else
		{
			ROS_WARN("leftleg ik failed!!!!!!!!!!!!!\n");
		}

		// Right Leg Inverse Kinematics
		rbd::InverseKinematics rightLegIk(talosRobot.talos,talosRobot.talos.bodyIndexByName("rightLegLinkSole"));
		sva::PTransformd rightPos;
		rightPos.rotation()=poseRight.block<3,3>(0,0).transpose();
		rightPos.translation()=poseRight.block<3,1>(0,3);
		if(rightLegIk.inverseKinematics(talosRobot.talos,talosRobot.talosmbc,rightPos)) 
		{
			jointValue.segment(RLEG_JOINT_START,RLEG_JOINT_NUM)=sParamToVector(talosRobot.talos,talosRobot.talosmbc.q).segment(6,6)*Util::TO_DEGREE;
			// cout<<"rightleg:\n"<<jointValue.segment(RLEG_JOINT_START,RLEG_JOINT_NUM)<<endl<<endl<<endl;//TODO:
		}
		else
		{
			ROS_WARN("rightleg ik failed!!!!!!!!!!!!!\n");
		}

		// Left Arm Inverse Kinematics
		rbd::InverseKinematics leftArmIk(talosRobot.talos,talosRobot.talos.bodyIndexByName("leftArmLinkSole"));
		sva::PTransformd armleftPos;
		armleftPos.rotation()=armposeLeft.block<3,3>(0,0).transpose();   // 姿态--不起作用
		armleftPos.translation()=armposeLeft.block<3,1>(0,3);			 // 位置--相对于重心
		if(leftArmIk.inverseKinematics(talosRobot.talos,talosRobot.talosmbc,armleftPos,3)) 
		{
			jointValue.segment(LARM_JOINT_START,LARM_JOINT_NUM)=sParamToVector(talosRobot.talos,talosRobot.talosmbc.q).segment(12,3)*Util::TO_DEGREE;
			// cout<<"leftarm ik passed!\n"<<jointValue.segment(LARM_JOINT_START,LARM_JOINT_NUM)<<endl<<endl<<endl;
			// ROS_INFO("leftarm ik passed!\n");
		}
		else
		{
			ROS_WARN("leftarm ik failed!!!!!!!!!!!!!\n");
		}

		// Right Arm Inverse Kinematics
		rbd::InverseKinematics rightArmIk(talosRobot.talos,talosRobot.talos.bodyIndexByName("rightArmLinkSole"));
		sva::PTransformd armrightPos;
		armrightPos.rotation()=armposeRight.block<3,3>(0,0).transpose(); // 姿态--不起作用
		armrightPos.translation()=armposeRight.block<3,1>(0,3);			 // 位置--相对于重心
		if(rightArmIk.inverseKinematics(talosRobot.talos,talosRobot.talosmbc,armrightPos,3)) 
		{
			jointValue.segment(RARM_JOINT_START,RARM_JOINT_NUM)=sParamToVector(talosRobot.talos,talosRobot.talosmbc.q).segment(15,3)*Util::TO_DEGREE;
			// cout<<"rightarm ik passed!\n"<<jointValue.segment(RARM_JOINT_START,RARM_JOINT_NUM)<<endl<<endl<<endl;
			// ROS_INFO("rightarm ik passed!\n");
		}
		else
		{
			ROS_WARN("rightarm ik failed!!!!!!!!!!!!!\n");
		}
	
		squatSequence.push_back(jointValue.segment(0,18));
		// std::cout<< "poseLeft:\n" << poseLeft <<std::endl;

	}
	// std::cout<< "poseLeft:\n" << poseLeft <<std::endl;

	// jointValue.setZero();
	lastJointValue.segment(0,18)=jointValue.segment(0,18);
	lastMeasuredJointValue.segment(0,18)=jointValue.segment(0,18);
	jointVelocity=(jointValue-lastJointValue)/timeStep;
	// talosRobot.talosmbc.zero(talosRobot.talos);
	talosRobot.talosmbc.q=sVectorToParam(talosRobot.talos,jointValue.segment(0,18)*Util::TO_RADIAN);
	rbd::forwardKinematics(talosRobot.talos, talosRobot.talosmbc);
	// std::cout<< "com:\n" << rbd::computeCoM(talosRobot.talos,talosRobot.talosmbc)<<std::endl;
	// std::cout<< "bodyPosW:\n" << sva::conversions::toHomogeneous(talosRobot.talosmbc.bodyPosW[0])<<std::endl;

	talosRobot.measuredmbc.zero(talosRobot.talos);
	talosRobot.measuredmbc.q=sVectorToParam(talosRobot.talos,jointValue.segment(0,18)*Util::TO_RADIAN);

}








void LIPMWalk::ForwardKinematics(Eigen::VectorXd jointvalue)
{
	Eigen::VectorXd jv;
	jv.resize(12);
	jv=sParamToVector(talosRobot.talos,talosRobot.talosmbc.q).segment(0,12);


	talosRobot.talosmbc.q=sVectorToParam(talosRobot.talos,jointvalue.segment(0,12)*Util::TO_RADIAN);
	rbd::forwardKinematics(talosRobot.talos, talosRobot.talosmbc);
	LeftLegPosture=sva::conversions::toHomogeneous(talosRobot.talosmbc.bodyPosW[7]);
	RightLegPosture=sva::conversions::toHomogeneous(talosRobot.talosmbc.bodyPosW[14]);


	talosRobot.talosmbc.q=sVectorToParam(talosRobot.talos,jv.segment(0,12));
}

void LIPMWalk::run(){
	// estimateComAndComv();
	// estimateComAndComv1();


	// if(WayPoint_X0[StepCount+1] == 0 )  //&&  fabs(CurrentPos.Torso_x)<0.005
	// 	return;
	// GroundContactDetect();
	if(RobotState == Action_SwingAround)
	{
		SwingAround();
		computerJointValue();

	}
	else if(RobotState == Action_YawAround)
	{
		YawAround();
		computerJointValue();
	}

	

}








void LIPMWalk::CubicSplineInit(Eigen::Vector6d *In, Eigen::Vector6d *Out, double t){
	Matrix<double, 6, 6> m6X6;
	m6X6<<	1, 0, 0, 0, 0, 0,
				1, t, pow(t,2), pow(t,3), pow(t,4), pow(t,5),
				0, 1, 0, 0, 0, 0, 
				0, 1, 2*t, 3*pow(t,2), 4*pow(t,3), 5*pow(t,4),
				0, 0, 2, 0, 0, 0, 
				0, 0, 2, 6*t, 12*pow(t,2), 20*pow(t,3);

	(*Out) = m6X6.inverse()*(*In);
}

double LIPMWalk::CubicSpline(Eigen::Vector6d *In,  double t){
	double s = (*In)(0) + (*In)(1)*t + (*In)(2)*pow(t,2) + (*In)(3)*pow(t,3) +  (*In)(4)*pow(t,4) +  (*In)(5)*pow(t,5);
	return s;
}

double LIPMWalk::CubicSplineVel(Eigen::Vector6d *In,  double t){
	double s = (*In)(1) + 2*((*In)(2))*t + 3*((*In)(3))*pow(t,2) + 4*((*In)(4))*pow(t,3) +  5*((*In)(5))*pow(t,4);
	return s;
}



void LIPMWalk::YawAround(int Yaw_Amplitude, int Roll_Amplitude)
{
    static int YawAmplitude = Yaw_Amplitude;   // 手臂左右张开的角度,默认为0
	static int RollAmplitude = Roll_Amplitude; // 手臂前后摆动的角度,默认为80
	static int YawCount = 0;
	static int i;
	static int count = 200;
	i++;
	//初始化 xyz--坐标　XYZ--角度
	PosPara_wF.Torso_x = 0;
	PosPara_wF.Torso_y = 0;
	PosPara_wF.Torso_z = torsoHeightWalk;
	PosPara_wF.Lfoot_y = footSeprate/2;
	PosPara_wF.Rfoot_y = -footSeprate/2;

	PosPara_wF.Torso_Y = Pi/9*sin(2*Pi*i/count);


    // RArm_R, RArm_P, RArm_elbow... 等为关节角度
	// RArm_Rdf, RArm_Pdf, RArm_elbowdf... 等为关节角度初始值
	// elbow 关节角度设置为零
	if (i == count)
	{
		i = 1;
		YawCount++;
	}
	if(YawCount == 0)
	{
		RArm_R = RArm_Rdf + (YawAmplitude - RArm_Rdf)*i/count;
		RArm_P = RArm_Pdf + (0 - RArm_Pdf)*i/count;
		// RArm_elbow = RArm_elbowdf + (-80 - RArm_elbowdf)*i/count;
		RArm_elbow = 0;
		LArm_R = LArm_Rdf + (YawAmplitude - LArm_Rdf)*i/count;
		LArm_P = LArm_Pdf + (0 - LArm_Pdf)*i/count;
		// LArm_elbow = LArm_elbowdf + (-80 - LArm_elbowdf)*i/count;
		LArm_elbow = 0;
	}
	else if(YawCount ==1||YawCount ==2)
	{
		double t = (1+sin(4*Pi*i/count-Pi/2))/2;
		RArm_R = YawAmplitude - RollAmplitude*t;
		LArm_R = - YawAmplitude + RollAmplitude*t;
		RArm_P = RollAmplitude*sin(2*Pi*i/count);
		LArm_P = - RollAmplitude*sin(2*Pi*i/count);
	}
	else if(YawCount == 3)
	{
		RArm_R = YawAmplitude + (RArm_Rdf-YawAmplitude)*i/count;
		RArm_P = RArm_Pdf*i/count;
		// RArm_elbow = -80 + (RArm_elbowdf-(-80))*i/count;
		RArm_elbow = 0;
		LArm_R = YawAmplitude + (LArm_Rdf-YawAmplitude)*i/count;
		LArm_P = LArm_Pdf*i/count;
		// LArm_elbow = -80 + (LArm_elbowdf-(-80))*i/count;
		LArm_elbow = 0;
	}
	else if (YawCount == 4)
	{
		//ActionDone = true;
		ROS_INFO("Yaw around done.");
		RobotState = Walk_stand;
        // YawCount 复位
		YawCount = 0;
		// 手臂关节角度复位
		RArm_R = RArm_Rdf;
		RArm_P = RArm_Pdf;
		RArm_elbow = RArm_elbowdf;
		LArm_R = LArm_Rdf;
		LArm_P = LArm_Pdf;
		LArm_elbow = LArm_elbowdf;
		// 计数复位
		i = 0;
		YawCount = 0;
	}
	//　更新手臂关节角度
	jointValue.segment(12,6) << LArm_P, LArm_R, LArm_elbow,  RArm_P, RArm_R, RArm_elbow;
}


void LIPMWalk::SwingAround()
{
	ActionDone = false;

	static int i=1;
	double t = sqrt(torsoHeightWalk*torsoHeightWalk+0*0);
	double a = 0.04/(2*Pi);
	double sita,x,y;
	int RCount = 3;
	if(i<=90)//螺旋线开始
	{
		i++;
		sita = 2*Pi*i/90;
		x = a*sita*cos(sita);
		y = a*sita*sin(sita);
		PosPara_wF.Torso_x = 0;
		// PosPara_wF.Torso_y = -0.045+0.045*i/90;
		PosPara_wF.Torso_y = 0;
		PosPara_wF.Torso_z = torsoHeightWalk;
	}
	else if(i<=90*(RCount+1))//绕圆
	{
		i++;
		sita = 2*Pi*i/90;
		x = 0.04*cos(sita);
		y = 0.04*sin(sita);
	}
	else if(i<=90*(RCount+2))//螺旋线结束
	{
		i++;
		sita = -2*Pi*(90*(RCount+2)-i)/90;
		x = a*sita*cos(sita+Pi);
		y = a*sita*sin(sita+Pi);
	}
	double t_V = sqrt(t*t-x*x-y*y);
	PosPara_wF.Torso_R = asin(y/t_V);
	PosPara_wF.Torso_P = asin(x/t_V);

	std::cout<< "torso pos: "<< i <<" "<<PosPara_wF.Torso_x <<" "<<PosPara_wF.Torso_y <<" "<<PosPara_wF.Torso_z << "  " <<PosPara_wF.Torso_R << "  " <<PosPara_wF.Torso_P << std::endl;
	// std::cout<< "L pos: " <<  PosPara_wF.Lfoot_x <<" "<<PosPara_wF.Lfoot_y <<" "<<PosPara_wF.Lfoot_z << "  " <<PosPara_wF.Lfoot_P << "  " <<PosPara_wF.Lfoot_P << std::endl;
	// std::cout<< "R pos: " <<  PosPara_wF.Rfoot_x <<" "<<PosPara_wF.Rfoot_y <<" "<<PosPara_wF.Rfoot_z << "  " <<PosPara_wF.Rfoot_R << "  " <<PosPara_wF.Rfoot_P << std::endl;

	PosPara_wF.Lfoot_y = footSeprate/2;
	PosPara_wF.Rfoot_y = -footSeprate/2;


	if(i>90*(RCount+2)) {
		//ActionDone = true; //扭腰动作结束
		ROS_INFO("Swing around done.");
		i = 1;  // reset i
		RobotState = Action_YawAround;
		ROS_INFO("Start yaw around.");
		// while(1);
	}
}



void LIPMWalk::computerJointValue(){
	///TODO : change Htranformation of leg to a simple form 
	// Vector3d torsoEulor(-PosPara.Torso_P,-PosPara.Torso_Y,-PosPara.Torso_R);
	// PosPara_wF.Torso_P = M_PI/180;
	// PosPara_wF.Torso_R = M_PI/180;
	Vector3d torsoEulor(-PosPara_wF.Torso_P,-PosPara_wF.Torso_Y,-PosPara_wF.Torso_R);
	Matrix3d torsoRotation = Euler_2_rotation(torsoEulor);

	// Vector3d leftEulor(PosPara.Lfoot_P-PosPara.Torso_P, PosPara.Lfoot_Y-PosPara.Torso_Y,PosPara.Lfoot_R-PosPara.Torso_R);
	Vector3d leftEulor(PosPara_wF.Lfoot_P-PosPara_wF.Torso_P, PosPara_wF.Lfoot_Y-PosPara_wF.Torso_Y,PosPara_wF.Lfoot_R-PosPara_wF.Torso_R);
	Matrix3d leftRotation = Euler_2_rotation(leftEulor);
	Vector3d leftVector = {(PosPara.Lfoot_x-PosPara.Torso_x), (PosPara.Lfoot_y-PosPara.Torso_y), (PosPara.Lfoot_z-PosPara.Torso_z)};
	Vector3d leftVector_wF = {(PosPara_wF.Lfoot_x-PosPara_wF.Torso_x), (PosPara_wF.Lfoot_y-PosPara_wF.Torso_y), (PosPara_wF.Lfoot_z-PosPara_wF.Torso_z)};

	// cout << "leftVectorInS:  " <<leftVector[0]<<"  "<<leftVector[1]<<"  "<<leftVector[2]<<"  "<<endl;
	// cout << "leftVectorInW:  " <<leftVector_wF[0]<<"  "<<leftVector_wF[1]<<"  "<<leftVector_wF[2]<<"  "<<endl;

	// cout << "LfootZInS:  " <<PosPara.Lfoot_z<<"  "<<"TorsoZInS:  "<<"  "<<PosPara.Torso_z<<"  "<<endl;
	// cout << "LfootZInW:  " <<PosPara_wF.Lfoot_z<<"  "<<"TorsoZInW:  "<<"  "<<PosPara_wF.Torso_z<<"  "<<endl;


	// cout  << "  " <<endl;

	// leftVector = torsoRotation*leftVector;
	// Translation<double,3> leftTrans(leftVector);

	leftVector_wF = torsoRotation*leftVector_wF;
	Translation<double,3> leftTrans(leftVector_wF);

	Transform<double,3,Affine> leftTransform = leftTrans*leftRotation;


	// Vector3d rightEulor(PosPara.Rfoot_P-PosPara.Torso_P, PosPara.Rfoot_Y-PosPara.Torso_Y,PosPara.Rfoot_R-PosPara.Torso_R);
	Vector3d rightEulor(PosPara_wF.Rfoot_P-PosPara_wF.Torso_P, PosPara_wF.Rfoot_Y-PosPara_wF.Torso_Y,PosPara_wF.Rfoot_R-PosPara_wF.Torso_R);
	Matrix3d rightRotation = Euler_2_rotation(rightEulor);
	Vector3d rightVector = {(PosPara.Rfoot_x-PosPara.Torso_x), (PosPara.Rfoot_y-PosPara.Torso_y), (PosPara.Rfoot_z-PosPara.Torso_z)};
	Vector3d rightVector_wF = {(PosPara_wF.Rfoot_x-PosPara_wF.Torso_x), (PosPara_wF.Rfoot_y-PosPara_wF.Torso_y), (PosPara_wF.Rfoot_z-PosPara_wF.Torso_z)};

	// cout << "rightVectorInS:  " <<rightVector[0]<<"  "<<rightVector[1]<<"  "<<rightVector[2]<<"  "<<endl;
	// cout << "rightVectorInW:  " <<rightVector_wF[0]<<"  "<<rightVector_wF[1]<<"  "<<rightVector_wF[2]<<"  "<<endl;
	// cout  << "  " <<endl;

	// rightVector = torsoRotation*rightVector;
	// Translation<double,3> rightTrans(rightVector);

	rightVector_wF = torsoRotation*rightVector_wF;
	Translation<double,3> rightTrans(rightVector_wF);


	Transform<double,3,Affine> rightTransform = rightTrans*rightRotation;




	Eigen::MatrixXd leftDesirH(4,4);  //homogeneous transform leftfoot in com frame,use to inverse kinematics
	leftDesirH.setIdentity();
	leftDesirH=leftTransform.matrix();

	
	rbd::InverseKinematics leftLegIk(talosRobot.talos,talosRobot.talos.bodyIndexByName("leftLegLinkSole"));
	sva::PTransformd leftPos;
	leftPos.rotation()=leftDesirH.block<3,3>(0,0).transpose();
	leftPos.translation()=leftDesirH.block<3,1>(0,3);
	if(leftLegIk.inverseKinematics(talosRobot.talos,talosRobot.talosmbc,leftPos)) 
	{
		jointValue.segment(LLEG_JOINT_START,LLEG_JOINT_NUM)=sParamToVector(talosRobot.talos,talosRobot.talosmbc.q).segment(0,6)*Util::TO_DEGREE;
	
		// cout<<"leftJoint:\n"<<jointValue.segment(LLEG_JOINT_START,LLEG_JOINT_NUM)<<endl<<endl<<endl;
	}
	else
	{
		printf("leftleg ik failed!!!!!!!!!!!!!\n");
	}

	// std::cout<< "leftDesirH:"<<std::endl<< leftDesirH<<std::endl;
	// cout<< "left trans: "<< endl<< leftDesirH <<endl;  

	Eigen::MatrixXd rightDesirH(4,4);
	rightDesirH.setIdentity();
	rightDesirH=rightTransform.matrix();


	rbd::InverseKinematics rightLegIk(talosRobot.talos,talosRobot.talos.bodyIndexByName("rightLegLinkSole"));
	sva::PTransformd rightPos;
	rightPos.rotation()=rightDesirH.block<3,3>(0,0).transpose();
	rightPos.translation()=rightDesirH.block<3,1>(0,3);
	if(rightLegIk.inverseKinematics(talosRobot.talos,talosRobot.talosmbc,rightPos)) 
	{
		jointValue.segment(RLEG_JOINT_START,RLEG_JOINT_NUM)=sParamToVector(talosRobot.talos,talosRobot.talosmbc.q).segment(6,6)*Util::TO_DEGREE;
	
		// cout<<"leftJoint:\n"<<jointValue.segment(RLEG_JOINT_START,RLEG_JOINT_NUM)<<endl<<endl<<endl;
	}
	else
	{
		printf("rightleg ik failed!!!!!!!!!!!!!\n");
	}

	// std::cout<< "rightDesirH:"<<std::endl<< rightDesirH<<std::endl;


	// HipOffset();

	talosRobot.talosmbc.q=sVectorToParam(talosRobot.talos,jointValue.segment(0,18)*Util::TO_RADIAN);
	jointVelocity=(jointValue-lastJointValue)/timeStep;
	lastJointValue.segment(0,18)=jointValue.segment(0,18);
	
}

Eigen::Matrix3d LIPMWalk::Euler_2_rotation(Eigen::Matrix<double,3,1> euler)
{
	double heading,attitude,bank;
	heading=euler(0);
	attitude=euler(1);
	bank=euler(2);
  // Assuming the angles are in radians.
  double ch = cos(heading);//heading  y
  double sh = sin(heading);
  double ca = cos(attitude);//attitude z
  double sa = sin(attitude);
  double cb = cos(bank);//bank 
  double sb = sin(bank);

  Eigen::Matrix3d rot;

  rot(0,0) = ch * ca;
  rot(0,1) = sh*sb - ch*sa*cb;
  rot(0,2) = ch*sa*sb + sh*cb;
  rot(1,0) = sa;
  rot(1,1) = ca*cb;
  rot(1,2) = -ca*sb;
  rot(2,0) = -sh*ca;
  rot(2,1) = sh*sa*cb + ch*sb;
  rot(2,2) = -sh*sa*sb + ch*cb;
	
  return rot;
}

