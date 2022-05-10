#ifndef GUIDANCE_H
#define GUIDANCE_H

#include "util.h"
#include "pwm.h"
#include "control.h"

void guidance_set_origin(float* value);

void reset_guidance();
//void guidance(float* position, float* target_orientation, float* target_vector);
//void guidance(float* position, float orientation_z, float* roll, float* pitch);

// set_origin is true if the current point should be set to the origin
void guidance_auto_waypoint(float* position, float* orientation, bool *set_origin);

void guidance_manual(PWM_in* pwm_in, float* orientation);
void guidance_manual_heading_hold(PWM_in* pwm_in, float* position, float* orientation, bool* set_origin);

#endif