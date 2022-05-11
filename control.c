#include "control.h"
#include "pwm.h"
//#include "spi_eeprom.h"

#define CHANNEL_REVERSE_AILERON_MASK (1 << 0)
#define CHANNEL_REVERSE_ELEVATOR_MASK (1 << 1)
#define CHANNEL_REVERSE_RUDDER_MASK (1 << 2)

float PID_X[3];
float PID_Y[3];
float PID_Z[3];
float mix_mat[9];
float thro_config[3];
float channel_trim[3];
int32_t channel_reverse;
float elevator_turn_p;

bool disable_integral;
extern bool arm;


void control_write(float thro, float ale, float elev, float rudd) {
	// trim channels
	//float throttle = (arm) ? thro : -1.0f;
	float throttle = thro;
	float aileron = ale + channel_trim[0];
	float elevator = elev + channel_trim[1];
	float rudder = rudd + channel_trim[2];
	
	// reverse channels
	if (channel_reverse & CHANNEL_REVERSE_AILERON_MASK) aileron = -aileron;
	if (channel_reverse & CHANNEL_REVERSE_ELEVATOR_MASK) elevator = -elevator;
	if (channel_reverse & CHANNEL_REVERSE_RUDDER_MASK) rudder = -rudder;
	
	// output values
	//pwm_write(PWM_WRITE_THRO, throttle);
	pwm_write_thro(throttle);
	pwm_write(PWM_WRITE_ALE, aileron);
	pwm_write(PWM_WRITE_ELEV, elevator);
	pwm_write(PWM_WRITE_RUDD, rudder);
}


void control(float roll, float pitch, float* orientation) {
	static uint32_t previous_time = 0;
	// get current time
	uint32_t current_time = read_timer_20ns();
	// calculate time difference
	uint32_t delta_time = current_time - previous_time;
	// reset previous time
	previous_time = current_time;
	// convert previous time to float
	float i_time = delta_time * TIMER_S_MULTIPLIER;
	
	// limit roll and pitch values
	if (roll > 30) roll = 30;
	else if (roll < -30) roll = -30;
	if (pitch > 15) pitch = 15;
	else if (pitch < -15) pitch = -15;
	
	// correct values so orientation goes correct way
	if (orientation[0] > roll + 180) roll += 360;
	if (orientation[0] < roll - 180) roll -= 360;
	
	
	// calculate difference between set orientation and actual orientation
	float world_error[3];
	world_error[0] = roll - orientation[0];
	world_error[1] = pitch - orientation[1];
	world_error[2] = 0.0f;
	
	// matrix that transforms orientation from world space to board space
	float sinx = sin(radians(orientation[0]));
	float siny = sin(radians(orientation[1]));
	float cosx = cos(radians(orientation[0]));
	float cosy = cos(radians(orientation[1]));
	
	float euler_mat[9] = {
		1, 0, -siny,
		0, cosx, sinx * cosy,
		0, -sinx, cosx * cosy
	};
	
	// multiply error by euler_mat to get error in board space
	float error[3];
	mat_multiply(euler_mat, 3, 3, world_error, 3, 1, error);
	
	//// define rudder error as horizontal acceleration divided by average speed
	//error[2] = acceleration[3] / speed;
	
	// run PID on each axis
	static float previous_error[3] = {0, 0, 0};
	static float integral[3] = {0, 0, 0};
		
	// proportional
	float proportional[3];
	mat_copy(error, 3, proportional);
	
	// integral
	float error_dt[3];
	float integral_time = (disable_integral) ? 0 : i_time;
	disable_integral = false;
	mat_scalar_product(error, integral_time, 3, error_dt);
	mat_add(integral, error_dt, 3, integral);
	
	// derivative
	float derivative[3];
	float delta_error[3];
	mat_subtract(error, previous_error, 3, delta_error);
	mat_scalar_product(delta_error, 1 / i_time, 3, derivative);
	
	// output
	float output[3];
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
	
	// calculate throttle
	float thro = thro_config[0] * 2 - 1;
	thro += thro_config[1] * pitch / 90;
	
	// calculate up elevator for turn
	float extra_elevator = 1.0f;
	if (cosx != 0) {
		extra_elevator = ((1.0f / cosx) - 1.0f) * elevator_turn_p;
		extra_elevator = MIN_2(1.0f, extra_elevator);
	}
	output[1] += extra_elevator * 90;
	
	// mix output value channels
	float mixed_output[3];
	mat_multiply(mix_mat, 3, 3, output, 3, 1, mixed_output);
	
	//// output values to PWM
	//pwm_write(PWM_WRITE_ALE, mixed_output[0] / 90);
	//pwm_write(PWM_WRITE_ELEV, mixed_output[1] / 90);
	//pwm_write(PWM_WRITE_RUDD, mixed_output[2] / 90);
	//if (arm) pwm_write(PWM_WRITE_THRO, thro);
	//else pwm_write(PWM_WRITE_THRO, -1.0f);
	control_write(thro, mixed_output[0] / 90, mixed_output[1] / 90, mixed_output[2] / 90);
}


void control_disable_integral() {
	disable_integral = true;
}


void control_passthrough(PWM_in* pwm_in) {
	control_write(pwm_in->thro * 1.3f, pwm_in->ale, pwm_in->elev, pwm_in->rudd);
	//// trim channels
	//float throttle = (arm) ? pwm_in->thro : -1.0f;
	//float aileron = pwm_in->ale + channel_trim[0];
	//float elevator = pwm_in->elev + channel_trim[1];
	//float rudder = pwm_in->rudd + channel_trim[2];
	//
	//// reverse channels
	//if (channel_reverse & CHANNEL_REVERSE_AILERON_MASK) aileron = -aileron;
	//if (channel_reverse & CHANNEL_REVERSE_ELEVATOR_MASK) elevator = -elevator;
	//if (channel_reverse & CHANNEL_REVERSE_RUDDER_MASK) rudder = -rudder;
	//
	//// output values
	//pwm_write(PWM_WRITE_THRO, throttle);
	//pwm_write(PWM_WRITE_ALE, aileron);
	//pwm_write(PWM_WRITE_ELEV, elevator);
	//pwm_write(PWM_WRITE_RUDD, rudder);
}


//void _control(float* set, float* measured) {
	//static uint32_t previous_time = 0;
	//// get current time
	//uint32_t current_time = read_timer_20ns();
	//// calculate time difference
	//uint32_t delta_time = current_time - previous_time;
	//// reset previous time
	//previous_time = current_time;
	//// convert previous time to float
	//float i_time = delta_time * TIMER_S_MULTIPLIER;
	//
	//
	//// correct values so orientation goes correct way
	//if (measured[0] > set[0] + 180) set[0] += 360;
	//if (measured[0] < set[0] - 180) set[0] -= 360;
	//
	//if (measured[2] > set[2] + 180) set[2] += 360;
	//if (measured[2] < set[2] - 180) set[2] -= 360;
	//
	//
	//// variable that stores the difference between set orientation and measured orientation
	//float world_error[3];
	//mat_subtract(set, measured, 3, world_error);
	//
	//// matrix that transforms orientation from world space to board space
	////const float pi_180 = 0.01745329;
	//
	////float sinx = sin(measured[0] * pi_180);
	////float siny = sin(measured[1] * pi_180);
	////float cosx = cos(measured[0] * pi_180);
	////float cosy = cos(measured[1] * pi_180);
	//float sinx = sin(radians(measured[0]));
	//float siny = sin(radians(measured[1]));
	//float cosx = cos(radians(measured[0]));
	//float cosy = cos(radians(measured[1]));
	//
	//float euler_mat[9] = {
		//1, 0, -siny,
		//0, cosx, sinx * cosy,
		//0, -sinx, cosx * cosy
	//};
	//
	//// multiply error by euler_mat to get error in board space
	//static float previous_error[3] = {0, 0, 0};
	//static float integral[3] = {0, 0, 0};
	//
	//float error[3];
	//mat_multiply(euler_mat, 3, 3, world_error, 3, 1, error);
	//
	//// run PID on each axis
	//float proportional[3];
	//mat_copy(error, 3, proportional);
	//
	////float integral[3];
	//float error_dt[3];
	//mat_scalar_product(error, i_time, 3, error_dt);
	//mat_add(integral, error_dt, 3, integral);
	//
	//float derivative[3];
	//float delta_error[3];
	//mat_subtract(error, previous_error, 3, delta_error);
	//mat_scalar_product(delta_error, 1 / i_time, 3, derivative);
	//
	//float output[3];
	////mat_scalar_product(proportional, PID_values[0], 3, proportional);
	////mat_scalar_product(integral, PID_values[1], 3, integral);
	////mat_scalar_product(derivative, PID_values[2], 3, derivative);
	//float PID_values[3] = {PID_X[0], PID_Y[0], PID_Z[0]};
	//mat_element_multiply(proportional, PID_values, 3, proportional);
	//PID_values[0] = PID_X[1];
	//PID_values[1] = PID_Y[1];
	//PID_values[2] = PID_Z[1];
	//// as integral is static it cannot be written back to
	//float newIntegral[3];
	//mat_element_multiply(integral, PID_values, 3, newIntegral);
	//PID_values[0] = PID_X[2];
	//PID_values[1] = PID_Y[2];
	//PID_values[2] = PID_Z[2];
	//mat_element_multiply(derivative, PID_values, 3, derivative);
	//
	//mat_copy(proportional, 3, output);
	//mat_add(output, newIntegral, 3, output);
	//mat_add(output, derivative, 3, output);
	//
	//// reset previous error
	//mat_copy(error, 3, previous_error);
	//
	//// calculate throttle
	//float thro = thro_config[0] * 2 - 1;
	//thro += thro_config[1] * set[1] / 90;
	////// matrix to mix values across channels
	////float mix_mat[9] = {
		////1, 0, 0.3,
		////0, 1, 0,
		////-0.1, 0, 1
	////};
	//
	//float mixed_output[3];
	//mat_multiply(mix_mat, 3, 3, output, 3, 1, mixed_output);
	//
	//// output values to PWM
	//pwm_write(PWM_WRITE_ALE, mixed_output[0] / 90);
	//pwm_write(PWM_WRITE_ELEV, mixed_output[1] / 90);
	//pwm_write(PWM_WRITE_RUDD, mixed_output[2] / 90);
	//if (arm) pwm_write(PWM_WRITE_THRO, thro);
	//else pwm_write(PWM_WRITE_THRO, -1.0f);
//}