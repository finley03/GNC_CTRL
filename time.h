#ifndef TIME_H
#define TIME_H

#include "samd21g18a.h"

#ifndef F_CPU
#define F_CPU 48000000UL
#endif

void set_clock_48m();


void delay_8c(uint32_t n);

#define delay_us(n) delay_8c(n*6);
#define delay_ms(n) delay_8c(n*6000);

#endif