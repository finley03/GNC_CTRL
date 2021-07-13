#ifndef GUIDANCE_H
#define GUIDANCE_H

#include "util.h"

void guidance_set_origin(float* value);

void guidance(float* target_orientation, float* target_vector);

#endif