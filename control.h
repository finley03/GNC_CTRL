#ifndef CONTROL_H
#define CONTROL_H

#include "util.h"
#include "pwm.h"

// currently only orientation
//void _control(float* set, float* measured);
// control algorithm
void control(float roll, float pitch, float* orientation);
// disable integral from PID loops on next frame
void control_disable_integral();
// pass through values
void control_passthrough(PWM_in* pwm_in);

#endif