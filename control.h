#ifndef CONTROL_H
#define CONTROL_H

#include "util.h"

void control_load_values();
void control_load_value(CTRL_Param parameter);
void control_save_value(CTRL_Param parameter);
void control_set_value(CTRL_Param parameter, float* value);
void control_read_value(CTRL_Param parameter, float* value);

// currently only orientation
void control(float* set, float* measured);

#endif