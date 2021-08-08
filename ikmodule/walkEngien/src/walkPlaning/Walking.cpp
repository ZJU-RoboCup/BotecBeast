/*
 *   Walking.cpp
 *
 *   Author: ROBOTIS
 *
 */
#include <iostream>
#include <stdio.h>
#include <math.h>
// #include "Vector.h"
// #include "Matrix.h"
// #include "MX28.h"
// #include "MotionStatus.h"
// #include "Kinematics.h"
#include "Walking.h"

#include <Eigen/Dense>
#include "Kinematics.h"

using namespace Robot;
// using namespace GaitManager;

#define PI (3.14159265)

Walking* Walking::m_UniqueInstance = new Walking();


Walking::Walking()
{
	X_OFFSET = 0;   //站立时脚与髋关节的X方向差
	Y_OFFSET = 132;     //站立时脚与髋关节的Y方向差
	Z_OFFSET = 0;  //站立时下蹲高度
	R_OFFSET = 0;
	P_OFFSET = 0;
	A_OFFSET = 0;
	HIP_PITCH_OFFSET = 0;
	PERIOD_TIME = 500;			//ms a step  
	DSP_RATIO = 0.1;
	STEP_FB_RATIO = 0.28;
	Z_MOVE_AMPLITUDE = 30;  //抬脚高度
	Y_SWAP_AMPLITUDE = 10.0;
	Z_SWAP_AMPLITUDE = 3;
	PELVIS_OFFSET = 3.0;
	// ARM_SWING_GAIN = 1.5;
	// BALANCE_KNEE_GAIN = 0.3;
	// BALANCE_ANKLE_PITCH_GAIN = 0.9;
	// BALANCE_HIP_ROLL_GAIN = 0.5;
	// BALANCE_ANKLE_ROLL_GAIN = 1.0;

	// P_GAIN = JointData::P_GAIN_DEFAULT;
	// I_GAIN = JointData::I_GAIN_DEFAULT;
	// D_GAIN = JointData::D_GAIN_DEFAULT;

	X_MOVE_AMPLITUDE = 0;
	Y_MOVE_AMPLITUDE = 0;
	A_MOVE_AMPLITUDE = 0;	
	A_MOVE_AIM_ON = false;
	// BALANCE_ENABLE = true;

	// m_Joint.SetAngle(JointData::ID_R_SHOULDER_PITCH, -48.345);
	// m_Joint.SetAngle(JointData::ID_L_SHOULDER_PITCH, 41.313);
	// m_Joint.SetAngle(JointData::ID_R_SHOULDER_ROLL, -17.873);
	// m_Joint.SetAngle(JointData::ID_L_SHOULDER_ROLL, 17.580);
	// m_Joint.SetAngle(JointData::ID_R_ELBOW, 29.300);
	// m_Joint.SetAngle(JointData::ID_L_ELBOW, -29.593);

	// m_Joint.SetAngle(JointData::ID_HEAD_TILT, Kinematics::EYE_TILT_OFFSET_ANGLE);

	// m_Joint.SetSlope(JointData::ID_R_SHOULDER_PITCH, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
	// m_Joint.SetSlope(JointData::ID_L_SHOULDER_PITCH, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
	// m_Joint.SetSlope(JointData::ID_R_SHOULDER_ROLL, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
	// m_Joint.SetSlope(JointData::ID_L_SHOULDER_ROLL, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
	// m_Joint.SetSlope(JointData::ID_R_ELBOW, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
	// m_Joint.SetSlope(JointData::ID_L_ELBOW, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
	// m_Joint.SetSlope(JointData::ID_HEAD_PAN, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);

	// m_Joint.SetPGain(JointData::ID_R_SHOULDER_PITCH, 8);
	// m_Joint.SetPGain(JointData::ID_L_SHOULDER_PITCH, 8);
	// m_Joint.SetPGain(JointData::ID_R_SHOULDER_ROLL, 8);
	// m_Joint.SetPGain(JointData::ID_L_SHOULDER_ROLL, 8);
	// m_Joint.SetPGain(JointData::ID_R_ELBOW, 8);
	// m_Joint.SetPGain(JointData::ID_L_ELBOW, 8);
}
Walking::~Walking()
{
}

void Walking::simStart()
{
	SimControll::simInit();
}


double Walking::wsin(double time, double period, double period_shift, double mag, double mag_shift)
{
	return mag * sin(2 * 3.141592 / period * time - period_shift) + mag_shift;
}

void Walking::update_param_time()
{
	m_PeriodTime = PERIOD_TIME;
	m_DSP_Ratio = DSP_RATIO;
	m_SSP_Ratio = 1 - DSP_RATIO;

	m_X_Swap_PeriodTime = m_PeriodTime / 2;
	m_X_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;
	m_Y_Swap_PeriodTime = m_PeriodTime;
	m_Y_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;
	m_Z_Swap_PeriodTime = m_PeriodTime / 2;
	m_Z_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio / 2;
	m_A_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;

	m_SSP_Time = m_PeriodTime * m_SSP_Ratio;
	m_SSP_Time_Start_L = (1 - m_SSP_Ratio) * m_PeriodTime / 4;
	m_SSP_Time_End_L = (1 + m_SSP_Ratio) * m_PeriodTime / 4;
	m_SSP_Time_Start_R = (3 - m_SSP_Ratio) * m_PeriodTime / 4;
	m_SSP_Time_End_R = (3 + m_SSP_Ratio) * m_PeriodTime / 4;

	m_Phase_Time1 = (m_SSP_Time_End_L + m_SSP_Time_Start_L) / 2;
	m_Phase_Time2 = (m_SSP_Time_Start_R + m_SSP_Time_End_L) / 2;
	m_Phase_Time3 = (m_SSP_Time_End_R + m_SSP_Time_Start_R) / 2;

	m_Pelvis_Offset = PELVIS_OFFSET*11.378;
	m_Pelvis_Swing = m_Pelvis_Offset * 0.35;
	m_Arm_Swing_Gain = ARM_SWING_GAIN;
}

void Walking::update_param_move()
{
	// Forward/Back
	m_X_Move_Amplitude = X_MOVE_AMPLITUDE;
	m_X_Swap_Amplitude = X_MOVE_AMPLITUDE * STEP_FB_RATIO;

	// Right/Left
	m_Y_Move_Amplitude = Y_MOVE_AMPLITUDE / 2;
	if(m_Y_Move_Amplitude > 0)
		m_Y_Move_Amplitude_Shift = m_Y_Move_Amplitude;
	else
		m_Y_Move_Amplitude_Shift = -m_Y_Move_Amplitude;
	m_Y_Swap_Amplitude = Y_SWAP_AMPLITUDE + m_Y_Move_Amplitude_Shift * 0.04;

	m_Z_Move_Amplitude = Z_MOVE_AMPLITUDE / 2;
	m_Z_Move_Amplitude_Shift = m_Z_Move_Amplitude / 2;
	m_Z_Swap_Amplitude = Z_SWAP_AMPLITUDE;
	m_Z_Swap_Amplitude_Shift = m_Z_Swap_Amplitude;

	// Direction
	if(A_MOVE_AIM_ON == false)
	{
		m_A_Move_Amplitude = A_MOVE_AMPLITUDE * PI / 180.0 / 2;
		if(m_A_Move_Amplitude > 0)
			m_A_Move_Amplitude_Shift = m_A_Move_Amplitude;
		else
			m_A_Move_Amplitude_Shift = -m_A_Move_Amplitude;
	}
	else
	{
		m_A_Move_Amplitude = -A_MOVE_AMPLITUDE * PI / 180.0 / 2;
		if(m_A_Move_Amplitude > 0)
			m_A_Move_Amplitude_Shift = -m_A_Move_Amplitude;
		else
			m_A_Move_Amplitude_Shift = m_A_Move_Amplitude;
	}
}

void Walking::update_param_balance()
{
	m_X_Offset = X_OFFSET;
	m_Y_Offset = Y_OFFSET;
	m_Z_Offset = Z_OFFSET;
	m_R_Offset = R_OFFSET * PI / 180.0;
	m_P_Offset = P_OFFSET * PI / 180.0;
	m_A_Offset = A_OFFSET * PI / 180.0;
	m_Hip_Pitch_Offset = HIP_PITCH_OFFSET*11.378;
}

void Walking::Initialize()
{
	X_MOVE_AMPLITUDE   = 0;
	Y_MOVE_AMPLITUDE   = 0;
	A_MOVE_AMPLITUDE   = 0;

	// m_Body_Swing_Y = 0;
	// m_Body_Swing_Z = 0;

	m_X_Swap_Phase_Shift = PI;
	m_X_Swap_Amplitude_Shift = 0;
	m_X_Move_Phase_Shift = PI / 2;
	m_X_Move_Amplitude_Shift = 0;
	m_Y_Swap_Phase_Shift = 0;
	m_Y_Swap_Amplitude_Shift = 0;
	m_Y_Move_Phase_Shift = PI / 2;
	m_Z_Swap_Phase_Shift = PI * 3 / 2;
	m_Z_Move_Phase_Shift = PI / 2;
	m_A_Move_Phase_Shift = PI / 2;

	m_Ctrl_Running = false;
	m_Real_Running = false;
	m_Time = 0;
	update_param_time();
	update_param_move();

	Process();
}

void Walking::Start()
{
	m_Ctrl_Running = true;
	m_Real_Running = true;

	StepCount = 0;
}

void Walking::Stop()
{
			StepCount = 0;
	m_Ctrl_Running = false;
}
		
bool Walking::IsRunning()
{
	return m_Real_Running;
}

void Walking::Process()
{
	double x_swap, y_swap, z_swap, a_swap, b_swap, c_swap;
	double x_move_r, y_move_r, z_move_r, a_move_r, b_move_r, c_move_r;
	double x_move_l, y_move_l, z_move_l, a_move_l, b_move_l, c_move_l;
	double pelvis_offset_r, pelvis_offset_l;
	double TIME_UNIT = 20; //ms
	// Update walk parameters
	if(m_Time == 0)
	{
		if(m_Real_Running == true)
			StepCount++;
		update_param_time();
		m_Phase = PHASE0;
		if(m_Ctrl_Running == false)
		{
			if(m_X_Move_Amplitude == 0 && m_Y_Move_Amplitude == 0 && m_A_Move_Amplitude == 0)
			{
				m_Real_Running = false;
			}
			else
			{
				X_MOVE_AMPLITUDE = 0;
				Y_MOVE_AMPLITUDE = 0;
				A_MOVE_AMPLITUDE = 0;
			}
		}
	}
	else if(m_Time >= (m_Phase_Time1 - TIME_UNIT/2) && m_Time < (m_Phase_Time1 + TIME_UNIT/2))
	{
		update_param_move();
		m_Phase = PHASE1;
	}
	else if(m_Time >= (m_Phase_Time2 - TIME_UNIT/2) && m_Time < (m_Phase_Time2 + TIME_UNIT/2))
	{
		update_param_time();
		m_Time = m_Phase_Time2;
		m_Phase = PHASE2;
		if(m_Ctrl_Running == false)
		{
			if(m_X_Move_Amplitude == 0 && m_Y_Move_Amplitude == 0 && m_A_Move_Amplitude == 0)
			{
				m_Real_Running = false;
			}
			else
			{
				X_MOVE_AMPLITUDE = 0;
				Y_MOVE_AMPLITUDE = 0;
				A_MOVE_AMPLITUDE = 0;
			}
		}
	}
	else if(m_Time >= (m_Phase_Time3 - TIME_UNIT/2) && m_Time < (m_Phase_Time3 + TIME_UNIT/2))
	{
		update_param_move();
		m_Phase = PHASE3;
	}
	update_param_balance();

	// Compute endpoints
	x_swap = wsin(m_Time, m_X_Swap_PeriodTime, m_X_Swap_Phase_Shift, m_X_Swap_Amplitude, m_X_Swap_Amplitude_Shift);
	y_swap = wsin(m_Time, m_Y_Swap_PeriodTime, m_Y_Swap_Phase_Shift, m_Y_Swap_Amplitude, m_Y_Swap_Amplitude_Shift);
	z_swap = wsin(m_Time, m_Z_Swap_PeriodTime, m_Z_Swap_Phase_Shift, m_Z_Swap_Amplitude, m_Z_Swap_Amplitude_Shift);
	a_swap = 0;
	b_swap = 0;
	c_swap = 0;

	if(m_Time <= m_SSP_Time_Start_L)
	{
		x_move_l = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
		y_move_l = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
		z_move_l = wsin(m_SSP_Time_Start_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
		c_move_l = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
		x_move_r = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
		y_move_r = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
		z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
		c_move_r = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
		pelvis_offset_l = 0;
		pelvis_offset_r = 0;
	}
	else if(m_Time <= m_SSP_Time_End_L)
	{
		x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
		y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
		z_move_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
		c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
		x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
		y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
		z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
		c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
		pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Pelvis_Swing / 2, m_Pelvis_Swing / 2);
		pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, -m_Pelvis_Offset / 2, -m_Pelvis_Offset / 2);
	}
	else if(m_Time <= m_SSP_Time_Start_R)
	{
		x_move_l = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
		y_move_l = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
		z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
		c_move_l = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
		x_move_r = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
		y_move_r = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
		z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
		c_move_r = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
		pelvis_offset_l = 0;
		pelvis_offset_r = 0;
	}
	else if(m_Time <= m_SSP_Time_End_R)
	{
		x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
		y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
		z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
		c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
		x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
		y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
		z_move_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
		c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
		pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Pelvis_Offset / 2, m_Pelvis_Offset / 2);
		pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, -m_Pelvis_Swing / 2, -m_Pelvis_Swing / 2);
	}
	else
	{
		x_move_l = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
		y_move_l = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
		z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
		c_move_l = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
		x_move_r = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
		y_move_r = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
		z_move_r = wsin(m_SSP_Time_End_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
		c_move_r = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
		pelvis_offset_l = 0;
		pelvis_offset_r = 0;
	}

	a_move_l = 0;
	b_move_l = 0;
	a_move_r = 0;
	b_move_r = 0;

	
	double ep[12];
	ep[0] = x_swap + x_move_r + m_X_Offset;
	ep[1] = y_swap + y_move_r - m_Y_Offset / 2;
	ep[2] = z_swap + z_move_r + m_Z_Offset - TorsoHight;
	ep[3] = a_swap + a_move_r - m_R_Offset / 2;
	ep[4] = b_swap + b_move_r + m_P_Offset;
	ep[5] = c_swap + c_move_r - m_A_Offset / 2;
	ep[6] = x_swap + x_move_l + m_X_Offset;
	ep[7] = y_swap + y_move_l + m_Y_Offset / 2;
	ep[8] = z_swap + z_move_l + m_Z_Offset - TorsoHight;
	ep[9] = a_swap + a_move_l + m_R_Offset / 2;
	ep[10] = b_swap + b_move_l + m_P_Offset;
	ep[11] = c_swap + c_move_l + m_A_Offset / 2;

	computerJointValue(ep);

	// printf("\nm_Time = %f   %f   %f  %f  %f\n",m_Time, X_MOVE_AMPLITUDE, x_swap , x_move_r, m_X_Offset);
	// for(int i=0; i<12; i++)
	// 	printf("%f    ",ep[i] );

	// // Compute body swing
	// if(m_Time <= m_SSP_Time_End_L)
	// {
	// 	m_Body_Swing_Y = -ep[7];
	// 	m_Body_Swing_Z = ep[8];
	// }
	// else
	// {
	// 	m_Body_Swing_Y = -ep[1];
	// 	m_Body_Swing_Z = ep[2];
	// }
	// m_Body_Swing_Z -= 307.6;//Kinematics::LEG_LENGTH;

	// Compute arm swing
	if(m_X_Move_Amplitude == 0)
	{
		JointValue[13] = 0; // Right
		JointValue[16] = 0; // Left
	}
	else
	{
		JointValue[13] = wsin(m_Time, m_PeriodTime, PI * 1.5, -m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
		JointValue[16] = wsin(m_Time, m_PeriodTime, PI * 1.5, m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
	}

	if(m_Real_Running == true)
	{
		m_Time += TIME_UNIT;
		if(m_Time >= m_PeriodTime)
			m_Time = 0;
	}

/*	double offset;
	double outValue[14];
	// Compute motor value
	for(int i=0; i<14; i++)
	{
	offset = (double)dir[i] * angle[i] * MX28::RATIO_ANGLE2VALUE;
	    if(i == 1) // R_HIP_ROLL
	        offset += (double)dir[i] * pelvis_offset_r;
	    else if(i == 7) // L_HIP_ROLL
	        offset += (double)dir[i] * pelvis_offset_l;
	    else if(i == 2 || i == 8) // R_HIP_PITCH or L_HIP_PITCH
	        offset -= (double)dir[i] * HIP_PITCH_OFFSET * MX28::RATIO_ANGLE2VALUE;

	    outValue[i] = MX28::Angle2Value(initAngle[i]) + (int)offset;
	}*/

}

Eigen::Matrix3d Walking::Euler_2_rotation(Eigen::Matrix<double,3,1> euler)
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

void Walking::computerJointValue(double* ep){
	double angleDirection[18] = {  1, -1, -1, 1, 1, -1, 
				                   1, -1, 1, -1, -1, -1, 
				                   1, 1, 1, 
				                   1, 1, 1};
	double jointOffset[18] = {  0, 0, 0, 0, 0, 0, 
				                        0, 0, 0, 0, 0, 0, 
				                        0, 0, 0, 
				                        0, 0, 0};

	Vector3d leftEulor(ep[9], ep[10], ep[11]);
	Matrix3d leftRotation = Euler_2_rotation(leftEulor);
	Vector3d leftVector(ep[6], ep[7], ep[8]);
	Translation<double,3> leftTrans(leftVector);
	Transform<double,3,Affine> leftTransform = leftTrans*leftRotation;
	Matrix<double, 4, 4> leftMatrix =leftTransform.matrix();
	Matrix<double,6,1> lefttheta;
	for(int i =0;i<6;i++)
		lefttheta(i) = JointValue[i+1];

	Kinematics::inverse_kinematics(leftMatrix, Chains::leftLeg,lefttheta);
	lefttheta(2) -= HIP_PITCH_OFFSET;

// std::cout<<"leftMatrix:\n"<<leftMatrix<<std::endl;

	for(int i =0;i<6;i++)
		JointValue[i+1] = lefttheta(i);


	Vector3d rightEulor(ep[3], ep[4], ep[5]);
	Matrix3d rightRotation = Euler_2_rotation(rightEulor);
	Vector3d rightVector(ep[0], ep[1], ep[2]);
	Translation<double,3> rightTrans(rightVector);
	Transform<double,3,Affine> rightTransform = rightTrans*rightRotation;
	Matrix<double, 4, 4> rightMatrix =rightTransform.matrix();
	Matrix<double,6,1> righttheta;

	for(int i =0;i<6;i++)
		righttheta(i) = JointValue[i+7];

	Kinematics::inverse_kinematics(rightMatrix, Chains::rightLeg,righttheta);
	righttheta(2) -= HIP_PITCH_OFFSET;
	// std::cout<<"rightMatrix:\n"<<rightMatrix<<std::endl;

	for(int i =0;i<6;i++)
		JointValue[i+7] = righttheta(i);


}