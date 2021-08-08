#ifndef _imuData_h_
#define _imuData_h_

#include <math.h>
#include <iomanip>
#include <iostream>
#include "lpf.h"
#include "robotDataStruct.h"

class ImuData {
 public:
  ImuData();
  ~ImuData();
  void calibrateGyro(double_t gx, double_t gy, double_t gz);
  void calibrateAcc(double_t ax, double_t ay, double_t az);
  void calibrateAttitude(double_t r, double_t p, double_t y);
  void setGyro(double_t gx, double_t gy, double_t gz);
  void setAcc(double_t ax, double_t ay, double_t az);
  void setGyroLpf(double_t sampleFreq, double_t cutoffFreq);
  void setAccLpf(double_t sampleFreq, double_t cutoffFreq);
  void setAttitude(double_t roll, double_t pitch, double_t yaw);
  ImuParam_t getData(void);
  Axis getGyro(void);
  Axis getAcc(void);
  AttitudeAngle_t getAttitude(void);
  void fusionCoefficient(float kp, float ki, float halfT);
  void datafusion(void);

 private:
  void fusion(float gx, float gy, float gz, float ax, float ay, float az);

  ImuParam_t dataOffset;
  ImuParam_t imuData;

  LowPassFilter2p lpfGyroX;
  LowPassFilter2p lpfGyroY;
  LowPassFilter2p lpfGyroZ;
  LowPassFilter2p lpfAccX;
  LowPassFilter2p lpfAccY;
  LowPassFilter2p lpfAccZ;

  float _kp;             // 比例增益支配率收敛到加速度计/磁强计
  float _ki;             // 积分增益支配率的陀螺仪偏见的衔接
  float _halfT;          // 采样周期的一半
  float q0, q1, q2, q3;  // 四元数的元素，代表估计方向
  float exInt, eyInt, ezInt;  // 按比例缩小积分误差
};

#endif