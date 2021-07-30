#include "control.h"
#include "pwm.h"
//#include "spi_eeprom.h"


float PID_X[3];
float PID_Y[3];
float PID_Z[3];
float mix_mat[9];


//void control_init() {
	//const float init_mix_mat[9] = {
		//1, 0, 0.3,
		//0, 1, 0,
		//-0.1, 0, 1,
	//}
//}

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
	
	
	// correct values so orientation goes correct way
	if (measured[0] > set[0] + 180) set[0] += 360;
	if (measured[0] < set[0] - 180) set[0] -= 360;
	
	if (measured[2] > set[2] + 180) set[2] += 360;
	if (measured[2] < set[2] - 180) set[2] -= 360;
	
	
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
	// as integral is static it cannot be written back to
	float newIntegral[3];
	mat_element_multiply(integral, PID_values, 3, newIntegral);
	PID_values[0] = PID_X[2];
	PID_values[1] = PID_Y[2];
	PID_values[2] = PID_Z[2];
	mat_element_multiply(derivative, PID_values, 3, derivative);
	
	mat_copy(proportional, 3, output);
	mat_add(output, newIntegral, 3, output);
	mat_add(output, derivative, 3, output);
	
	// reset previous error
	mat_copy(error, 3, previous_error);
	
	
	//// matrix to mix values across channels
	//float mix_mat[9] = {
		//1, 0, 0.3,
		//0, 1, 0,
		//-0.1, 0, 1
	//};
	
	float mixed_output[3];
	mat_multiply(mix_mat, 3, 3, output, 3, 1, mixed_output);
	
	// output values to PWM
	pwm_write(PWM_WRITE_ALE, mixed_output[0] / 90);
	pwm_write(PWM_WRITE_ELEV, mixed_output[1] / 90);
	pwm_write(PWM_WRITE_RUDD, mixed_output[2] / 90);
	
	
}