#ifndef _walkDataStruct_h_
#define _walkDataStruct_h_

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <iostream>

class Posture {
 public:
  double_t x;
  double_t y;
  double_t z;
  double_t roll;
  double_t pitch;
  double_t yaw;

 public:
  Posture();
  ~Posture();
  void zero();
};

#endif
