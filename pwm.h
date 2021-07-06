#ifndef PWM_H
#define PWM_H

#include "util.h"

#define PWM_FREQUENCY 50
#define PWM_TOP (F_CPU / PWM_FREQUENCY) // 960000
#define PWM_DUTY_MAX (PWM_TOP * 0.1) // 96000
#define PWM_DUTY_MID (PWM_TOP * 0.075) // 72000
#define PWM_DUTY_MIN (PWM_TOP * 0.05) // 48000
#define PWM_DUTY_RANGE (PWM_DUTY_MAX - PWM_DUTY_MIN)
#define PWM_DUTY_HALF_RANGE (PWM_DUTY_RANGE / 2)

#define PWM_OUTPUT_CHANNELS 4
#define PWM_WRITE_THRO 0
#define PWM_WRITE_ALE 1
#define PWM_WRITE_ELEV 2
#define PWM_WRITE_RUDD 3

// pwm output on PA14 to PA17

void pwm_init_out();
// writes to PWM channel and accepts value between +1 and -1
void pwm_write(uint8_t channel, float position);

#endif