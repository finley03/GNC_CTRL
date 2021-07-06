#include "pwm.h"


void pwm_write(uint8_t channel, float position) {
	// calculate PWM value
	uint32_t pwm_val = position * PWM_DUTY_HALF_RANGE + PWM_DUTY_MID;
	
	// check value is within range
	pwm_val = (pwm_val <= PWM_DUTY_MAX) ? pwm_val : PWM_DUTY_MAX;
	pwm_val = (pwm_val >= PWM_DUTY_MIN) ? pwm_val : PWM_DUTY_MIN;
	
	// check channel is in range
	if (channel > PWM_OUTPUT_CHANNELS) return;
	
	// write value to PWM channel
	TCC0->CC[channel].bit.CC = pwm_val;
}

void pwm_init_out() {
	// enable bus clock for TCC0
	PM->APBCMASK.bit.TCC0_ = 1;
	
	// pipe main clock to TCC0 and TCC1
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC0_TCC1;
	// wait until configration done
	while (GCLK->STATUS.bit.SYNCBUSY);
	
	// set prescaler to no divide
	TCC0->CTRLA.bit.PRESCALER = 0x0;
	
	// set waveform generation mode to normal pwm
	TCC0->WAVE.bit.WAVEGEN = 0x2;
	// wait
	while (TCC0->SYNCBUSY.bit.WAVE);
	
	// set TOP value of PWM
	TCC0->PER.bit.PER = PWM_TOP;
	while (TCC0->SYNCBUSY.bit.PER);
	
	// set duty cycle to middle for init
	TCC0->CC[0].bit.CC = PWM_DUTY_MID;
	while (TCC0->SYNCBUSY.bit.CC0);
	TCC0->CC[1].bit.CC = PWM_DUTY_MID;
	while (TCC0->SYNCBUSY.bit.CC1);
	TCC0->CC[2].bit.CC = PWM_DUTY_MID;
	while (TCC0->SYNCBUSY.bit.CC2);
	TCC0->CC[3].bit.CC = PWM_DUTY_MID;
	while (TCC0->SYNCBUSY.bit.CC3);
	
	// configure PORT for pwm output
	PORT_WRCONFIG_Type wrconfig = {
		// enable write to PORT
		.bit.WRPINCFG = 1,
		
		// enable write to PMUX
		.bit.WRPMUX = 1,
		
		// select PMUX funciton F
		.bit.PMUX = 0x5,
		
		// enable periperal multiplexign
		.bit.PMUXEN = 1
	};
	
	// configure pins in lower half
	wrconfig.bit.HWSEL = 0;
	wrconfig.bit.PINMASK = (uint16_t)((PORT_PA14 | PORT_PA15));
	PORT->Group[0].WRCONFIG.reg = wrconfig.reg;
	
	// configure pins in upper half
	wrconfig.bit.HWSEL = 1;
	wrconfig.bit.PINMASK = (uint16_t)((PORT_PA16 | PORT_PA17) >> 16);
	PORT->Group[0].WRCONFIG.reg = wrconfig.reg;
	
	// enable PWM
	TCC0->CTRLA.bit.ENABLE = 1;
	while (TCC0->SYNCBUSY.bit.ENABLE);
}