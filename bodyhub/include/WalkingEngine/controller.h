#ifndef _controller_h_
#define _controller_h_

#include "robotDataStruct.h"
#include "walkDataStruct.h"

void controllerInit(void);
void setRestrainWobbleOnoff(bool on);
void setRestrainWobbleParam(double_t *kList);
Posture restrainWobble(void);
void setSlopeBalanceOnoff(bool on);
void setSlopeBalanceRParam(double_t *kList);
void setSlopeBalancePParam(double_t *kList);
Posture slopeBalance(void);

#endif