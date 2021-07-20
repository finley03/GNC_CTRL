#ifndef UTIL_H
#define UTIL_H

#include "samd21g18a.h"
#include "time.h"
#include "mat.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#define _STD_G 9.80665

typedef enum {
	_PID_X,
	_PID_Y,
	_PID_Z
} CTRL_Param;

//#define F_CPU 48000000
#define LED PORT_PB11

#define ABS(a) ((a < 0) ? -a : a)
#define MAX_2(a, b) ((a > b) ? a : b)
#define MIN_2(a, b) ((b > a) ? a : b);
#define MAX_3(a, b, c) (MAX_2(MAX_2(a, b), c))
#define MIN_3(a, b, c) (MIN_2(MIN_2(a, b), c))
#define UMAX_2(a, b) MAX_2(ABS(a), ABS(b))
#define UMIN_2(a, b) MIN_2(ABS(a), ABS(b))
#define UMAX_3(a, b, c) MAX_3(ABS(a), ABS(b), ABS(c))
#define UMIN_3(a, b, c) MIN_3(ABS(a), ABS(b), ABS(c))

#define pow(a, b) exp2(b * log2(a))
#define powf(a, b) exp2f(b * log2f(a))

#define radians(x) (x * 0.01745329251994329576923690768489)
#define degrees(x) (x * 57.295779513082320876798154814105)

void LED_print_8(uint8_t data);

void crc_init();

#define CRC32_CHECK 0x2144DF1C
uint32_t crc32(uint8_t* data, uint32_t data_size);

//float radians(float value);
//float degrees(float value);

#endif