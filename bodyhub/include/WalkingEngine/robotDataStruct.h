#ifndef _robotDataStruct_h_
#define _robotDataStruct_h_

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <iostream>

typedef struct Axis {
  double_t x;
  double_t y;
  double_t z;
} Axis_t;

typedef struct AttitudeAngle {
  double_t roll;
  double_t pitch;
  double_t yaw;
} AttitudeAngle_t;

typedef struct JointParam {
  double_t angle;
  double_t torque;
} JointParam_t;

typedef struct ForceParam {
  Axis_t force[4];
  Axis_t moment[4];
} ForceParam_t;

typedef struct ImuParam {
  Axis_t angularVelocity;
  Axis_t linearAcceleration;
  AttitudeAngle_t attitude;
} ImuParam_t;

typedef struct {
  JointParam_t joint;
  ForceParam_t force;
  ImuParam_t imu;
} robotData_t;

#endif