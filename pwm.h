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

#define PWM_IN_THRO_MASK PORT_PA02
#define PWM_IN_ALE_MASK PORT_PA03
#define PWM_IN_ELEV_MASK PORT_PA04
#define PWM_IN_RUDD_MASK PORT_PA05
#define PWM_IN_AUX_MASK PORT_PA06
#define PWM_IN_OVR_MASK PORT_PA07
#define PWM_IN_MASK (PORT_PA02 | PORT_PA03 | PORT_PA04 | PORT_PA05 | PORT_PA06 | PORT_PA07)

#define PWM_INPUT_CHANNELS 6
#define PWM_READ_THRO

#define PWM_BOOL(a) ((a > 0) ? true : false)

typedef struct {
	float thro; // throttle channel
	float ale; // aileron channel
	float elev; // elevator channel
	float rudd; // rudder channel
	float aux; // aux channel
	float ovr; // override channel
} PWM_in;

// pwm output on PA14 to PA17

void pwm_init_out();
// writes to PWM channel and accepts value between +1 and -1
void pwm_write(uint8_t channel, float position);

void pwm_write_all(PWM_in value);

void pwm_init_in();

// doesn't really read data, just interprets already existing data
PWM_in pwm_read();

#endif