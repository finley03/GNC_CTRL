#include "pwm.h"
#include "uart.h"

// data structures for pwm in interrupts
static uint32_t port_in_state[12];
static uint32_t pwm_in_time[12];
static uint_fast8_t pwm_in_index;
extern bool armed;


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

void pwm_write_thro(float position) {
	uint32_t pwm_val;
	if (armed) {
		// calculate PWM value
		pwm_val = position * PWM_DUTY_HALF_RANGE + PWM_DUTY_MID;
	
		// check value is in range
		pwm_val = (pwm_val <= PWM_DUTY_MAX) ? pwm_val : PWM_DUTY_MAX;
		pwm_val = (pwm_val >= PWM_DUTY_MIN) ? pwm_val : PWM_DUTY_MIN;
	}
	else pwm_val = PWM_DUTY_MIN;
	
	// write to pwm channel
	TCC0->CC[PWM_WRITE_THRO].bit.CC = pwm_val;
}

void pwm_write_all(PWM_in value) {
	// calculate pwm values
	uint32_t thro = value.thro * PWM_DUTY_HALF_RANGE + PWM_DUTY_MID;
	uint32_t ale = value.ale * PWM_DUTY_HALF_RANGE + PWM_DUTY_MID;
	uint32_t elev = value.elev * PWM_DUTY_HALF_RANGE + PWM_DUTY_MID;
	uint32_t rudd = value.rudd * PWM_DUTY_HALF_RANGE + PWM_DUTY_MID;
	
	// check all values are within range
	thro = (thro <= PWM_DUTY_MAX) ? thro : PWM_DUTY_MAX;
	thro = (thro >= PWM_DUTY_MIN) ? thro : PWM_DUTY_MIN;
	
	ale = (ale <= PWM_DUTY_MAX) ? ale : PWM_DUTY_MAX;
	ale = (ale >= PWM_DUTY_MIN) ? ale : PWM_DUTY_MIN;
	
	elev = (elev <= PWM_DUTY_MAX) ? elev : PWM_DUTY_MAX;
	elev = (elev >= PWM_DUTY_MIN) ? elev : PWM_DUTY_MIN;
	
	rudd = (rudd <= PWM_DUTY_MAX) ? rudd : PWM_DUTY_MAX;
	rudd = (rudd >= PWM_DUTY_MIN) ? rudd : PWM_DUTY_MIN;
	
	// write to pwm channels
	// check throttle channel is armed
	TCC0->CC[PWM_WRITE_THRO].bit.CC = (armed) ? thro : PWM_DUTY_MIN;
	TCC0->CC[PWM_WRITE_ALE].bit.CC = ale;
	TCC0->CC[PWM_WRITE_ELEV].bit.CC = elev;
	TCC0->CC[PWM_WRITE_RUDD].bit.CC = rudd;
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
	TCC0->CC[0].bit.CC = PWM_DUTY_MIN_THROTTLE; // throttle channel, min required
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
		
		// enable periperal multiplexing
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


// configuring interrupts for pwm in
// interrupt is shaped to take as little time as possible
// as to not destroy data transactions
void pwm_init_in() {
	// link EIC to bus
	PM->APBAMASK.bit.EIC_ = 1;
	
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID_EIC;
	
	// configure PORT for interrupt input
	PORT_WRCONFIG_Type wrconfig = {
		// lower half of pins
		.bit.HWSEL = 0,
		
		// enable writing to PORT
		.bit.WRPINCFG = 1,
		
		// enable writing to PMUX
		.bit.WRPMUX = 1,
		
		// enable peripheral multiplexing
		.bit.PMUXEN = 1,
		
		// select peripheral multiplexing function A
		.bit.PMUX = 0,
		
		// enable input
		.bit.INEN = 1,
		
		// select pins
		.bit.PINMASK = (uint16_t)(PORT_PA02 | PORT_PA03 | PORT_PA04 | PORT_PA05 | PORT_PA06 | PORT_PA07)
	};
	
	REG_PORT_DIRCLR0 = PORT_PA02 | PORT_PA03 | PORT_PA04 | PORT_PA05 | PORT_PA06 | PORT_PA07;
	
	// apply pin config
	PORT->Group[0].WRCONFIG.reg = wrconfig.reg;
	
	// interrupt on both edges
	EIC->CONFIG[0].reg =
		EIC_CONFIG_SENSE2_BOTH | EIC_CONFIG_SENSE3_BOTH | EIC_CONFIG_SENSE4_BOTH |
		EIC_CONFIG_SENSE5_BOTH | EIC_CONFIG_SENSE6_BOTH | EIC_CONFIG_SENSE7_BOTH;
	
	// enable EIC
	EIC->CTRL.bit.ENABLE = 1;
	while(EIC->STATUS.bit.SYNCBUSY);
	
	// clear all interrupts
	EIC->INTFLAG.reg = EIC->INTFLAG.reg;
	
	EIC->INTENSET.reg =
		EIC_INTENSET_EXTINT2 | EIC_INTENSET_EXTINT3 | EIC_INTENSET_EXTINT4 |
		EIC_INTENSET_EXTINT5 | EIC_INTENSET_EXTINT6 | EIC_INTENSET_EXTINT7;
	
	pwm_in_index = 0;
	
	NVIC_EnableIRQ(EIC_IRQn);
}

PWM_in pwm_read() {
	static PWM_in ret;
	
	const float pwm_mult = 1 / PWM_DUTY_HALF_RANGE;
	
	if (pwm_in_index == 12) {
		bool defaultb = false;
		
		uint32_t time_int[6];
		
		char binbuffer[17];
		char buffer[50];
		for (uint8_t i = 0; i < 12; i += 2) {
			switch (port_in_state[i] & PWM_IN_MASK) {
			case PWM_IN_THRO_MASK:
				time_int[0] = pwm_in_time[i + 1] - pwm_in_time[i];
				break;
				
			case PWM_IN_ALE_MASK:
				time_int[1] = pwm_in_time[i + 1] - pwm_in_time[i];
				break;
				
			case PWM_IN_ELEV_MASK:
				time_int[2] = pwm_in_time[i + 1] - pwm_in_time[i];
				break;
			
			case PWM_IN_RUDD_MASK:
				time_int[3] = pwm_in_time[i + 1] - pwm_in_time[i];
				break;
				
			case PWM_IN_AUX1_MASK:
				time_int[4] = pwm_in_time[i + 1] - pwm_in_time[i];
				break;
				
			case PWM_IN_AUX2_MASK:
				time_int[5] = pwm_in_time[i + 1] - pwm_in_time[i];
				break;
				
			default:
				sbinary16(binbuffer, port_in_state[i] & PWM_IN_MASK);
				sprintf(buffer, "Unknown pwm input: %s\n", binbuffer);
				serial_print(buffer);
				defaultb = true;
				break;
			}
		}
		pwm_in_index = 0;
		if (!defaultb) {
			//ret.thro = (float) time_int[0] * TIMER_MS_MULTIPLIER;
			//ret.ale = (float) time_int[1] * TIMER_MS_MULTIPLIER;
			//ret.elev = (float) time_int[2] * TIMER_MS_MULTIPLIER;
			//ret.rudd = (float) time_int[3] * TIMER_MS_MULTIPLIER;
			//ret.aux = (float) time_int[4] * TIMER_MS_MULTIPLIER;
			//ret.ovr = (float) time_int[5] * TIMER_MS_MULTIPLIER;
			ret.thro = (float)((int32_t) time_int[0] - PWM_DUTY_MID) * pwm_mult;
			ret.ale = (float)((int32_t) time_int[1] - PWM_DUTY_MID) * pwm_mult;
			ret.elev = (float)((int32_t) time_int[2] - PWM_DUTY_MID) * pwm_mult;
			ret.rudd = (float)((int32_t) time_int[3] - PWM_DUTY_MID) * pwm_mult;
			ret.aux1 = (float)((int32_t) time_int[4] - PWM_DUTY_MID) * pwm_mult;
			ret.aux2 = (float)((int32_t) time_int[5] - PWM_DUTY_MID) * pwm_mult;
		}
	}
	
	EIC->INTFLAG.reg = EIC->INTFLAG.reg;
	
	return ret;
}

int pwm_enum(float val, int count) {
	val = (val + 1) * 0.5;
	val *= count - 1;
	return (int)(val + 0.5);
}

void EIC_Handler() {
	//serial_print(".");
	// != should be faster than <
	if (pwm_in_index != 12) {
		if (pwm_in_index != 0 || (REG_PORT_IN0 & PWM_IN_MASK)) {
			pwm_in_time[pwm_in_index] = TIMER_REG;
			port_in_state[pwm_in_index] = REG_PORT_IN0;
			++pwm_in_index;
		}
	}
	EIC->INTFLAG.reg = EIC->INTFLAG.reg;
}