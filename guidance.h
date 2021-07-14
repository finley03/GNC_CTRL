#ifndef GUIDANCE_H
#define GUIDANCE_H

#include "util.h"

void guidance_set_origin(float* value);

float guidance(float* position, float* target_orientation, float* target_vector, float* debug);

#endif