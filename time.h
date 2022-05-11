#ifndef TIME_H
#define TIME_H

#include "util.h"

#ifndef F_CPU
#define F_CPU 48000000UL
#endif

#define TIMER_MS_MULTIPLIER (1000.0f/F_CPU)
#define TIMER_S_MULTIPLIER (1.0f/F_CPU)

#define TIMER_REG (TC4->COUNT32.COUNT.reg)

#define FRAME_RATE 50
#define FRAME_TIME_C (F_CPU / FRAME_RATE) // in cycles

void set_clock_48m();


void delay_8c(uint32_t n);

#define delay_us(n) delay_8c(n*6);
#define delay_ms(n) delay_8c(n*6000);


void init_timer();

void start_timer();

uint32_t read_timer_20ns();

float read_timer_ms();

float read_timer_s();

void frame_delay();


#endif