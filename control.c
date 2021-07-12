#include "control.h"
#include "pwm.h"
#include "spi_eeprom.h"


static float PID_X[3];
static float PID_Y[3];
static float PID_Z[3];


void control_load_values() {
	spi_eeprom_read_n(EEPROM_PID_X, PID_X, EEPROM_PID_SIZE);
	spi_eeprom_read_n(EEPROM_PID_Y, PID_Y, EEPROM_PID_SIZE);
	spi_eeprom_read_n(EEPROM_PID_Z, PID_Z, EEPROM_PID_SIZE);
}

void control_load_value(CTRL_Param parameter) {
	switch (parameter) {
		case _PID_X:
		spi_eeprom_read_n(EEPROM_PID_X, PID_X, EEPROM_PID_SIZE);
		break;
		case _PID_Y:
		spi_eeprom_read_n(EEPROM_PID_Y, PID_Y, EEPROM_PID_SIZE);
		break;
		case _PID_Z:
		spi_eeprom_read_n(EEPROM_PID_Z, PID_Z, EEPROM_PID_SIZE);
		break;
	}
}

void control_save_value(CTRL_Param parameter) {
	switch (parameter) {
		case _PID_X:
		spi_eeprom_write_n_s(EEPROM_PID_X, PID_X, EEPROM_PID_SIZE);
		break;
		case _PID_Y:
		spi_eeprom_write_n_s(EEPROM_PID_Y, PID_Y, EEPROM_PID_SIZE);
		break;
		case _PID_Z:
		spi_eeprom_write_n_s(EEPROM_PID_Z, PID_Z, EEPROM_PID_SIZE);
		break;
	}
}

void control_set_value(CTRL_Param parameter, float* value) {
	switch (parameter) {
		case _PID_X:
		mat_copy(value, 3, PID_X);
		break;
		case _PID_Y:
		mat_copy(value, 3, PID_Y);
		break;
		case _PID_Z:
		mat_copy(value, 3, PID_Z);
		break;
	}
}

void control_read_value(CTRL_Param parameter, float* value) {
	switch (parameter) {
		case _PID_X:
		mat_copy(PID_X, 3, value);
		break;
		case _PID_Y:
		mat_copy(PID_Y, 3, value);
		break;
		case _PID_Z:
		mat_copy(PID_Z, 3, value);
		break;
	}
}


void control(float* set, float* measured) {
	static uint32_t previous_time = 0;
	// get current time
	uint32_t current_time = read_timer_20ns();
	// calculate time difference
	uint32_t delta_time = current_time - previous_time;
	// reset previous time
	previous_time = current_time;
	// convert previous time to float
	float i_time = delta_time * TIMER_S_MULTIPLIER;
	
	
	// variable that stores the difference between set orientation and measured orientation
	float world_error[3];
	mat_subtract(set, measured, 3, world_error);
	
	// matrix that transforms orientation from world space to board space
	const float pi_180 = 0.01745329;
	
	float sinx = sin(measured[0] * pi_180);
	float siny = sin(measured[1] * pi_180);
	float cosx = cos(measured[0] * pi_180);
	float cosy = cos(measured[1] * pi_180);
	
	float euler_mat[9] = {
		1, 0, -siny,
		0, cosx, sinx * cosy,
		0, -sinx, cosx * cosy
	};
	
	// multiply error by euler_mat to get error in board space
	static float previous_error[3] = {0, 0, 0};
	static float integral[3] = {0, 0, 0};
	
	float error[3];
	mat_multiply(euler_mat, 3, 3, world_error, 3, 1, error);
	
	// run PID on each axis
	float proportional[3];
	mat_copy(error, 3, proportional);
	
	//float integral[3];
	float error_dt[3];
	mat_scalar_product(error, i_time, 3, error_dt);
	mat_add(integral, error_dt, 3, integral);
	
	float derivative[3];
	float delta_error[3];
	mat_subtract(error, previous_error, 3, delta_error);
	mat_scalar_product(delta_error, 1 / i_time, 3, derivative);
	
	float output[3];
	//mat_scalar_product(proportional, PID_values[0], 3, proportional);
	//mat_scalar_product(integral, PID_values[1], 3, integral);
	//mat_scalar_product(derivative, PID_values[2], 3, derivative);
	float PID_values[3] = {PID_X[0], PID_Y[0], PID_Z[0]};
	mat_element_multiply(proportional, PID_values, 3, proportional);
	PID_values[0] = PID_X[1];
	PID_values[1] = PID_Y[1];
	PID_values[2] = PID_Z[1];
	mat_element_multiply(integral, PID_values, 3, integral);
	PID_values[0] = PID_X[2];
	PID_values[1] = PID_Y[2];
	PID_values[2] = PID_Z[2];
	mat_element_multiply(derivative, PID_values, 3, derivative);
	
	mat_copy(proportional, 3, output);
	mat_add(output, integral, 3, output);
	mat_add(output, derivative, 3, output);
	
	// reset previous error
	mat_copy(error, 3, previous_error);
	
	
	// matrix to mix values across channels
	float mix_mat[9] = {
		1, 0, 0.3,
		0, 1, 0,
		-0.1, 0, 1
	};
	
	float mixed_output[3];
	mat_multiply(mix_mat, 3, 3, output, 3, 1, mixed_output);
	
	// output values to PWM
	pwm_write(PWM_WRITE_ALE, mixed_output[0] / 90);
	pwm_write(PWM_WRITE_ELEV, mixed_output[1] / 90);
	pwm_write(PWM_WRITE_RUDD, mixed_output[2] / 90);
	
	
}