#include "util.h"
#include "spi_eeprom.h"
#include "uart.h"
#include "dma.h"
#include "pwm.h"
#include "main.h"

NAV_Data_Packet nav_data_packet;

extern float PID_X[3];
extern float PID_Y[3];
extern float PID_Z[3];
extern float mix_mat[9];
extern float position_pid[3];
extern float waypoint_threshold;
extern float thro_config[3];
extern float channel_trim[3];
extern int32_t channel_reverse;
extern float heading_pid[3];
extern float altitude_pid[3];
extern float elevator_turn_p;
extern int flight_mode;
extern float disable_kalman_update_delay;
extern int32_t ctrl_flags_1;
extern float angle_of_attack;
extern float roll_limit;
extern float pitch_limit;
extern int flight_mode_0, flight_mode_1, flight_mode_2;
extern float loiter_radius;
extern float home_loiter_alt;
extern float launch_thro, launch_pitch, launch_minacc, launch_minspd, launch_throdelay, launch_time;
extern float fbwh_heading_slew, fbwh_altitude_slew;
extern float landing_descent_pitch, landing_flair_pitch, landing_descent_throttle, landing_flair_param;
int failsafe_flight_mode;

extern bool kalman_orientation_update_enabled;
extern bool armed;
extern bool set_origin;
extern bool set_home;
extern int last_flight_mode;


void LED_print_8(uint8_t data) {
	for (uint16_t i = 0; i < 8; ++i) {
		uint8_t bit = data & 0x80;
		
		if (bit != 0) {
			REG_PORT_OUTSET1 = LED;
		}
		
		delay_ms(250);
		
		REG_PORT_OUTCLR1 = LED;
		
		delay_ms(250);
		
		data = (data << 1);
	}
}


void crc_init() {
	// enable writing to DSU
	PAC1->WPCLR.reg = 0x2;
	// define CRC type to 32bit
	DMAC->CRCCTRL.bit.CRCPOLY = 0x1;
}


uint32_t crc32(uint8_t* data, uint32_t data_size) {
	*((volatile unsigned int*) 0x41007058) &= ~0x30000UL;
	
	// fill data register with starting value
	//DSU->DATA.reg = 0xffffffff;
	DSU->DATA.reg = 0xffffffff;
	
	DSU->ADDR.reg = (uint32_t) data;
	DSU->LENGTH.reg = DSU_LENGTH_LENGTH((data_size / 4));
	
	// clear done bit
	DSU->STATUSA.bit.DONE = 1;
	
	// start CRC calculation
	DSU->CTRL.bit.CRC = 1;
	
	// wait until done
	while(!DSU->STATUSA.bit.DONE);
	
	uint32_t out = DSU->DATA.reg ^ 0xffffffff;
	
	*((volatile unsigned int*) 0x41007058) |= 0x20000UL;
	
	return out;
}


void sbinary8(char* buffer, uint8_t value) {
	for (uint_fast8_t i = 0; i < 8; ++i) {
		uint_fast8_t bit = value & 0x80;
		
		if (bit != 0) buffer[i] = '1';
		else buffer[i] = '0';
		
		value <<= 1;
	}
	buffer[8] = '\0';
}

void sbinary16(char* buffer, uint16_t value) {
	for (uint_fast8_t i = 0; i < 16; ++i) {
		uint_fast8_t bit = value & 0x80;
		
		if (bit != 0) buffer[i] = '1';
		else buffer[i] = '0';
		
		value <<= 1;
	}
	buffer[16] = '\0';
}

void sbinary32(char* buffer, uint32_t value) {
	for (uint_fast8_t i = 0; i < 32; ++i) {
		uint_fast8_t bit = value & 0x80;
		
		if (bit != 0) buffer[i] = '1';
		else buffer[i] = '0';
		
		value <<= 1;
	}
	buffer[32] = '\0';
}

void nav_load_vec3(CTRL_Param parameter) {
	float value[3];
	control_read_eeprom(parameter, value);
	nav_set_vec3(parameter, value);
}

void nav_load_scalar(CTRL_Param parameter) {
	float value;
	control_read_eeprom(parameter, &value);
	nav_set_scalar(parameter, &value);
}

void nav_save_vec3(CTRL_Param parameter) {
	float value[3];
	nav_read_vec3(parameter, value);
	control_write_value(parameter, value);
}

void nav_save_scalar(CTRL_Param parameter) {
	float value;
	nav_read_scalar(parameter, &value);
	control_write_value(parameter, &value);
}

void control_load_values() {
	control_load_value(_PID_X);
	control_load_value(_PID_Y);
	control_load_value(_PID_Z);
	control_load_value(_X_MIX);
	control_load_value(_Y_MIX);
	control_load_value(_Z_MIX);
	control_load_value(_POSITION_PID);
	control_load_value(_WAYPOINT_THRESHOLD);
	control_load_value(_THRO_CONFIG);
	control_load_value(_CHANNEL_TRIM);
	control_load_value(_CHANNEL_REVERSE);
	control_load_value(_HEADING_PID);
	control_load_value(_ALTITUDE_PID);
	control_load_value(_ELEVATOR_TURN_P);
	control_load_value(_FLIGHT_MODE);
	control_load_value(_DISABLE_KALMAN_UPDATE_DELAY);
	control_load_value(_CTRL_FLAGS_1);
	control_load_value(_AOA);
	control_load_value(_ROLL_LIMIT);
	control_load_value(_PITCH_LIMIT);
	control_load_value(_FLIGHT_MODE_0);
	control_load_value(_FLIGHT_MODE_1);
	control_load_value(_FLIGHT_MODE_2);
	control_load_value(_LOITER_RADIUS);
	control_load_value(_HOME_LOITER_ALT);
	control_load_value(_LAUNCH_THRO);
	control_load_value(_LAUNCH_PITCH);
	control_load_value(_LAUNCH_MINACC);
	control_load_value(_LAUNCH_MINSPD);
	control_load_value(_LAUNCH_THRODELAY);
	control_load_value(_LAUNCH_TIME);
	control_load_value(_FBWH_HEADING_SLEW);
	control_load_value(_FBWH_ALTITUDE_SLEW);
	control_load_value(_LANDING_DESCENT_PITCH);
	control_load_value(_LANDING_FLAIR_PITCH);
	control_load_value(_LANDING_DESCENT_THROTTLE);
	control_load_value(_LANDING_FLAIR_PARAM);
	control_load_value(_FAILSAFE_FLIGHT_MODE);
	nav_load_vec3(_KALMAN_POSITION_UNCERTAINTY);
	nav_load_vec3(_KALMAN_VELOCITY_UNCERTAINTY);
	nav_load_vec3(_KALMAN_ORIENTATION_UNCERTAINTY);
	nav_load_vec3(_KALMAN_ORIENTATION_MEASUREMENT_UNCERTAINTY);
	nav_load_scalar(_KALMAN_GNSS_HORIZONTAL_UNCERTAINTY_MUL);
	nav_load_scalar(_KALMAN_GNSS_VERTICAL_UNCERTAINTY_MUL);
	nav_load_scalar(_KALMAN_BARO_VARIANCE);
	nav_load_scalar(_KALMAN_ACCEL_VARIANCE);
	nav_load_scalar(_KALMAN_ANGULARVELOCITY_VARIANCE);
	nav_load_scalar(_KALMAN_GNSS_ZEROLAT);
	nav_load_scalar(_KALMAN_GNSS_ZEROLONG);
	nav_load_scalar(_BARO_HEIGHT_CAL);
	nav_load_vec3(_MAG_A_1);
	nav_load_vec3(_MAG_A_2);
	nav_load_vec3(_MAG_A_3);
	nav_load_vec3(_MAG_B);
	nav_load_vec3(_ACCEL_B);
	nav_load_vec3(_GYRO_B);
}

void control_load_value(CTRL_Param parameter) {
	switch (parameter) {
		case _PID_X:
		spi_eeprom_read_n(EEPROM_PID_X, PID_X, VEC3_SIZE);
		break;
		case _PID_Y:
		spi_eeprom_read_n(EEPROM_PID_Y, PID_Y, VEC3_SIZE);
		break;
		case _PID_Z:
		spi_eeprom_read_n(EEPROM_PID_Z, PID_Z, VEC3_SIZE);
		break;
		case _X_MIX:
		spi_eeprom_read_n(EEPROM_X_MIX, mix_mat, VEC3_SIZE);
		break;
		case _Y_MIX:
		spi_eeprom_read_n(EEPROM_Y_MIX, mix_mat + 3, VEC3_SIZE);
		break;
		case _Z_MIX:
		spi_eeprom_read_n(EEPROM_Z_MIX, mix_mat + 6, VEC3_SIZE);
		break;
		case _POSITION_PID:
		spi_eeprom_read_n(EEPROM_POSITION_PID, position_pid, VEC3_SIZE);
		break;
		case _WAYPOINT_THRESHOLD:
		spi_eeprom_read_n(EEPROM_WAYPOINT_THRESHOLD, &waypoint_threshold, SCALAR_SIZE);
		break;
		case _THRO_CONFIG:
		spi_eeprom_read_n(EEPROM_THRO_CONFIG, thro_config, VEC3_SIZE);
		break;
		case _CHANNEL_TRIM:
		spi_eeprom_read_n(EEPROM_CHANNEL_TRIM, channel_trim, VEC3_SIZE);
		break;
		case _CHANNEL_REVERSE:
		spi_eeprom_read_n(EEPROM_CHANNEL_REVERSE, &channel_reverse, INT32_SIZE);
		break;
		case _HEADING_PID:
		spi_eeprom_read_n(EEPROM_HEADING_PID, heading_pid, VEC3_SIZE);
		break;
		case _ALTITUDE_PID:
		spi_eeprom_read_n(EEPROM_ALTITUDE_PID, altitude_pid, VEC3_SIZE);
		break;
		case _ELEVATOR_TURN_P:
		spi_eeprom_read_n(EEPROM_ELEVATOR_TURN_P, &elevator_turn_p, SCALAR_SIZE);
		break;
		case _FLIGHT_MODE:
		spi_eeprom_read_n(EEPROM_FLIGHT_MODE, &flight_mode, INT32_SIZE);
		break;
		case _DISABLE_KALMAN_UPDATE_DELAY:
		spi_eeprom_read_n(EEPROM_DISABLE_KALMAN_UPDATE_DELAY, &disable_kalman_update_delay, SCALAR_SIZE);
		break;
		case _CTRL_FLAGS_1:
		spi_eeprom_read_n(EEPROM_CTRL_FLAGS_1, &ctrl_flags_1, INT32_SIZE);
		break;
		case _AOA:
		spi_eeprom_read_n(EEPROM_AOA, &angle_of_attack, SCALAR_SIZE);
		break;
		case _ROLL_LIMIT:
		spi_eeprom_read_n(EEPROM_ROLL_LIMIT, &roll_limit, SCALAR_SIZE);
		break;
		case _PITCH_LIMIT:
		spi_eeprom_read_n(EEPROM_PITCH_LIMIT, &pitch_limit, SCALAR_SIZE);
		break;
		case _FLIGHT_MODE_0:
		spi_eeprom_read_n(EEPROM_FLIGHT_MODE_0, &flight_mode_0, SCALAR_SIZE);
		break;
		case _FLIGHT_MODE_1:
		spi_eeprom_read_n(EEPROM_FLIGHT_MODE_1, &flight_mode_1, SCALAR_SIZE);
		break;
		case _FLIGHT_MODE_2:
		spi_eeprom_read_n(EEPROM_FLIGHT_MODE_2, &flight_mode_2, SCALAR_SIZE);
		break;
		case _LOITER_RADIUS:
		spi_eeprom_read_n(EEPROM_LOITER_RADIUS, &loiter_radius, SCALAR_SIZE);
		break;
		case _HOME_LOITER_ALT:
		spi_eeprom_read_n(EEPROM_HOME_LOITER_ALT, &home_loiter_alt, SCALAR_SIZE);
		break;
		case _LAUNCH_THRO:
		spi_eeprom_read_n(EEPROM_LAUNCH_THRO, &launch_thro, SCALAR_SIZE);
		break;
		case _LAUNCH_PITCH:
		spi_eeprom_read_n(EEPROM_LAUNCH_PITCH, &launch_pitch, SCALAR_SIZE);
		break;
		case _LAUNCH_MINACC:
		spi_eeprom_read_n(EEPROM_LAUNCH_MINACC, &launch_minacc, SCALAR_SIZE);
		break;
		case _LAUNCH_MINSPD:
		spi_eeprom_read_n(EEPROM_LAUNCH_MINSPD, &launch_minspd, SCALAR_SIZE);
		break;
		case _LAUNCH_THRODELAY:
		spi_eeprom_read_n(EEPROM_LAUNCH_THRODELAY, &launch_throdelay, SCALAR_SIZE);
		break;
		case _LAUNCH_TIME:
		spi_eeprom_read_n(EEPROM_LAUNCH_TIME, &launch_time, SCALAR_SIZE);
		break;
		case _FBWH_HEADING_SLEW:
		spi_eeprom_read_n(EEPROM_FBWH_HEADING_SLEW, &fbwh_heading_slew, SCALAR_SIZE);
		break;
		case _FBWH_ALTITUDE_SLEW:
		spi_eeprom_read_n(EEPROM_FBWH_ALTITUDE_SLEW, &fbwh_altitude_slew, SCALAR_SIZE);
		break;
		case _LANDING_DESCENT_PITCH:
		spi_eeprom_read_n(EEPROM_LANDING_DESCENT_PITCH, &landing_descent_pitch, SCALAR_SIZE);
		break;
		case _LANDING_FLAIR_PITCH:
		spi_eeprom_read_n(EEPROM_LANDING_FLAIR_PITCH, &landing_flair_pitch, SCALAR_SIZE);
		break;
		case _LANDING_DESCENT_THROTTLE:
		spi_eeprom_read_n(EEPROM_LANDING_DESCENT_THROTTLE, &landing_descent_throttle, SCALAR_SIZE);
		break;
		case _LANDING_FLAIR_PARAM:
		spi_eeprom_read_n(EEPROM_LANDING_FLAIR_PARAM, &landing_flair_param, SCALAR_SIZE);
		break;
		case _FAILSAFE_FLIGHT_MODE:
		spi_eeprom_read_n(EEPROM_FAILSAFE_FLIGHT_MODE, &failsafe_flight_mode, SCALAR_SIZE);
		break;
		default:
		break;
	}
}

void control_save_value(CTRL_Param parameter) {
	switch (parameter) {
		case _PID_X:
		spi_eeprom_write_n_s(EEPROM_PID_X, PID_X, VEC3_SIZE);
		break;
		case _PID_Y:
		spi_eeprom_write_n_s(EEPROM_PID_Y, PID_Y, VEC3_SIZE);
		break;
		case _PID_Z:
		spi_eeprom_write_n_s(EEPROM_PID_Z, PID_Z, VEC3_SIZE);
		break;
		case _X_MIX:
		spi_eeprom_write_n_s(EEPROM_X_MIX, mix_mat, VEC3_SIZE);
		break;
		case _Y_MIX:
		spi_eeprom_write_n_s(EEPROM_Y_MIX, mix_mat + 3, VEC3_SIZE);
		break;
		case _Z_MIX:
		spi_eeprom_write_n_s(EEPROM_Z_MIX, mix_mat + 6, VEC3_SIZE);
		break;
		case _POSITION_PID:
		spi_eeprom_write_n_s(EEPROM_POSITION_PID, position_pid, VEC3_SIZE);
		break;
		case _WAYPOINT_THRESHOLD:
		spi_eeprom_write_n_s(EEPROM_WAYPOINT_THRESHOLD, &waypoint_threshold, SCALAR_SIZE);
		break;
		case _THRO_CONFIG:
		spi_eeprom_write_n_s(EEPROM_THRO_CONFIG, thro_config, VEC3_SIZE);
		break;
		case _CHANNEL_TRIM:
		spi_eeprom_write_n_s(EEPROM_CHANNEL_TRIM, channel_trim, VEC3_SIZE);
		break;
		case _CHANNEL_REVERSE:
		spi_eeprom_write_n_s(EEPROM_CHANNEL_REVERSE, &channel_reverse, INT32_SIZE);
		break;
		case _HEADING_PID:
		spi_eeprom_write_n_s(EEPROM_HEADING_PID, heading_pid, VEC3_SIZE);
		break;
		case _ALTITUDE_PID:
		spi_eeprom_write_n_s(EEPROM_ALTITUDE_PID, altitude_pid, VEC3_SIZE);
		break;
		case _ELEVATOR_TURN_P:
		spi_eeprom_write_n_s(EEPROM_ELEVATOR_TURN_P, &elevator_turn_p, SCALAR_SIZE);
		break;
		case _FLIGHT_MODE:
		spi_eeprom_write_n_s(EEPROM_FLIGHT_MODE, &flight_mode, INT32_SIZE);
		break;
		case _DISABLE_KALMAN_UPDATE_DELAY:
		spi_eeprom_write_n_s(EEPROM_DISABLE_KALMAN_UPDATE_DELAY, &disable_kalman_update_delay, SCALAR_SIZE);
		break;
		case _CTRL_FLAGS_1:
		spi_eeprom_write_n_s(EEPROM_CTRL_FLAGS_1, &ctrl_flags_1, INT32_SIZE);
		break;
		case _AOA:
		spi_eeprom_write_n_s(EEPROM_AOA, &angle_of_attack, SCALAR_SIZE);
		break;
		case _ROLL_LIMIT:
		spi_eeprom_write_n_s(EEPROM_ROLL_LIMIT, &roll_limit, SCALAR_SIZE);
		break;
		case _PITCH_LIMIT:
		spi_eeprom_write_n_s(EEPROM_PITCH_LIMIT, &pitch_limit, SCALAR_SIZE);
		break;
		case _FLIGHT_MODE_0:
		spi_eeprom_write_n_s(EEPROM_FLIGHT_MODE_0, &flight_mode_0, SCALAR_SIZE);
		break;
		case _FLIGHT_MODE_1:
		spi_eeprom_write_n_s(EEPROM_FLIGHT_MODE_1, &flight_mode_1, SCALAR_SIZE);
		break;
		case _FLIGHT_MODE_2:
		spi_eeprom_write_n_s(EEPROM_FLIGHT_MODE_2, &flight_mode_2, SCALAR_SIZE);
		break;
		case _LOITER_RADIUS:
		spi_eeprom_write_n_s(EEPROM_LOITER_RADIUS, &loiter_radius, SCALAR_SIZE);
		break;
		case _HOME_LOITER_ALT:
		spi_eeprom_write_n_s(EEPROM_HOME_LOITER_ALT, &home_loiter_alt, SCALAR_SIZE);
		break;
		case _LAUNCH_THRO:
		spi_eeprom_write_n_s(EEPROM_LAUNCH_THRO, &launch_thro, SCALAR_SIZE);
		break;
		case _LAUNCH_PITCH:
		spi_eeprom_write_n_s(EEPROM_LAUNCH_PITCH, &launch_pitch, SCALAR_SIZE);
		break;
		case _LAUNCH_MINACC:
		spi_eeprom_write_n_s(EEPROM_LAUNCH_MINACC, &launch_minacc, SCALAR_SIZE);
		break;
		case _LAUNCH_MINSPD:
		spi_eeprom_write_n_s(EEPROM_LAUNCH_MINSPD, &launch_minspd, SCALAR_SIZE);
		break;
		case _LAUNCH_THRODELAY:
		spi_eeprom_write_n_s(EEPROM_LAUNCH_THRODELAY, &launch_throdelay, SCALAR_SIZE);
		break;
		case _LAUNCH_TIME:
		spi_eeprom_write_n_s(EEPROM_LAUNCH_TIME, &launch_time, SCALAR_SIZE);
		break;
		case _FBWH_HEADING_SLEW:
		spi_eeprom_write_n_s(EEPROM_FBWH_HEADING_SLEW, &fbwh_heading_slew, SCALAR_SIZE);
		break;
		case _FBWH_ALTITUDE_SLEW:
		spi_eeprom_write_n_s(EEPROM_FBWH_ALTITUDE_SLEW, &fbwh_altitude_slew, SCALAR_SIZE);
		break;
		case _LANDING_DESCENT_PITCH:
		spi_eeprom_write_n_s(EEPROM_LANDING_DESCENT_PITCH, &landing_descent_pitch, SCALAR_SIZE);
		break;
		case _LANDING_FLAIR_PITCH:
		spi_eeprom_write_n_s(EEPROM_LANDING_FLAIR_PITCH, &landing_flair_pitch, SCALAR_SIZE);
		break;
		case _LANDING_DESCENT_THROTTLE:
		spi_eeprom_write_n_s(EEPROM_LANDING_DESCENT_THROTTLE, &landing_descent_throttle, SCALAR_SIZE);
		break;
		case _LANDING_FLAIR_PARAM:
		spi_eeprom_write_n_s(EEPROM_LANDING_FLAIR_PARAM, &landing_flair_param, SCALAR_SIZE);
		break;
		case _FAILSAFE_FLIGHT_MODE:
		spi_eeprom_write_n_s(EEPROM_FAILSAFE_FLIGHT_MODE, &failsafe_flight_mode, SCALAR_SIZE);
		break;
		default:
		break;
	}
}

void control_write_value(CTRL_Param parameter, void* value) {
	switch (parameter) {
		case _KALMAN_POSITION_UNCERTAINTY:
		spi_eeprom_write_n_s(EEPROM_KALMAN_POSITION_UNCERTAINTY, value, VEC3_SIZE);
		break;
		case _KALMAN_VELOCITY_UNCERTAINTY:
		spi_eeprom_write_n_s(EEPROM_KALMAN_VELOCITY_UNCERTAINTY, value, VEC3_SIZE);
		break;
		case _KALMAN_ORIENTATION_UNCERTAINTY:
		spi_eeprom_write_n_s(EEPROM_KALMAN_ORIENTATION_UNCERTAINTY, value, VEC3_SIZE);
		break;
		case _KALMAN_ORIENTATION_MEASUREMENT_UNCERTAINTY:
		spi_eeprom_write_n_s(EEPROM_KALMAN_ORIENTATION_MEASUREMENT_UNCERTAINTY, value, VEC3_SIZE);
		break;
		case _KALMAN_GNSS_HORIZONTAL_UNCERTAINTY_MUL:
		spi_eeprom_write_n_s(EEPROM_KALMAN_GNSS_HORIZONTAL_UNCERTAINTY_MUL, value, SCALAR_SIZE);
		break;
		case _KALMAN_GNSS_VERTICAL_UNCERTAINTY_MUL:
		spi_eeprom_write_n_s(EEPROM_KALMAN_GNSS_VERTICAL_UNCERTAINTY_MUL, value, SCALAR_SIZE);
		break;
		case _KALMAN_BARO_VARIANCE:
		spi_eeprom_write_n_s(EEPROM_KALMAN_BARO_VARIANCE, value, SCALAR_SIZE);
		break;
		case _KALMAN_ACCEL_VARIANCE:
		spi_eeprom_write_n_s(EEPROM_KALMAN_ACCEL_VARIANCE, value, SCALAR_SIZE);
		break;
		case _KALMAN_ANGULARVELOCITY_VARIANCE:
		spi_eeprom_write_n_s(EEPROM_KALMAN_ANGULARVELOCITY_VARIANCE, value, SCALAR_SIZE);
		break;
		case _KALMAN_GNSS_ZEROLAT:
		spi_eeprom_write_n_s(EEPROM_KALMAN_GNSS_ZEROLAT, value, SCALAR_SIZE);
		break;
		case _KALMAN_GNSS_ZEROLONG:
		spi_eeprom_write_n_s(EEPROM_KALMAN_GNSS_ZEROLONG, value, SCALAR_SIZE);
		break;
		case _MAG_A_1:
		spi_eeprom_write_n_s(EEPROM_MAG_A_1, value, VEC3_SIZE);
		break;
		case _MAG_A_2:
		spi_eeprom_write_n_s(EEPROM_MAG_A_2, value, VEC3_SIZE);
		break;
		case _MAG_A_3:
		spi_eeprom_write_n_s(EEPROM_MAG_A_3, value, VEC3_SIZE);
		break;
		case _MAG_B:
		spi_eeprom_write_n_s(EEPROM_MAG_B, value, VEC3_SIZE);
		break;
		case _ACCEL_B:
		spi_eeprom_write_n_s(EEPROM_ACCEL_B, value, VEC3_SIZE);
		break;
		case _GYRO_B:
		spi_eeprom_write_n_s(EEPROM_GYRO_B, value, VEC3_SIZE);
		break;
		case _BARO_HEIGHT_CAL:
		spi_eeprom_write_n_s(EEPROM_BARO_HEIGHT_CAL, value, SCALAR_SIZE);
		break;
		default:
		break;
	}
}

void control_read_eeprom(CTRL_Param parameter, void* value) {
	switch (parameter) {
		case _KALMAN_POSITION_UNCERTAINTY:
		spi_eeprom_read_n(EEPROM_KALMAN_POSITION_UNCERTAINTY, value, VEC3_SIZE);
		break;
		case _KALMAN_VELOCITY_UNCERTAINTY:
		spi_eeprom_read_n(EEPROM_KALMAN_VELOCITY_UNCERTAINTY, value, VEC3_SIZE);
		break;
		case _KALMAN_ORIENTATION_UNCERTAINTY:
		spi_eeprom_read_n(EEPROM_KALMAN_ORIENTATION_UNCERTAINTY, value, VEC3_SIZE);
		break;
		case _KALMAN_ORIENTATION_MEASUREMENT_UNCERTAINTY:
		spi_eeprom_read_n(EEPROM_KALMAN_ORIENTATION_MEASUREMENT_UNCERTAINTY, value, VEC3_SIZE);
		break;
		case _KALMAN_GNSS_HORIZONTAL_UNCERTAINTY_MUL:
		spi_eeprom_read_n(EEPROM_KALMAN_GNSS_HORIZONTAL_UNCERTAINTY_MUL, value, SCALAR_SIZE);
		break;
		case _KALMAN_GNSS_VERTICAL_UNCERTAINTY_MUL:
		spi_eeprom_read_n(EEPROM_KALMAN_GNSS_VERTICAL_UNCERTAINTY_MUL, value, SCALAR_SIZE);
		break;
		case _KALMAN_BARO_VARIANCE:
		spi_eeprom_read_n(EEPROM_KALMAN_BARO_VARIANCE, value, SCALAR_SIZE);
		break;
		case _KALMAN_ACCEL_VARIANCE:
		spi_eeprom_read_n(EEPROM_KALMAN_ACCEL_VARIANCE, value, SCALAR_SIZE);
		break;
		case _KALMAN_ANGULARVELOCITY_VARIANCE:
		spi_eeprom_read_n(EEPROM_KALMAN_ANGULARVELOCITY_VARIANCE, value, SCALAR_SIZE);
		break;
		case _KALMAN_GNSS_ZEROLAT:
		spi_eeprom_read_n(EEPROM_KALMAN_GNSS_ZEROLAT, value, SCALAR_SIZE);
		break;
		case _KALMAN_GNSS_ZEROLONG:
		spi_eeprom_read_n(EEPROM_KALMAN_GNSS_ZEROLONG, value, SCALAR_SIZE);
		break;
		case _MAG_A_1:
		spi_eeprom_read_n(EEPROM_MAG_A_1, value, VEC3_SIZE);
		break;
		case _MAG_A_2:
		spi_eeprom_read_n(EEPROM_MAG_A_2, value, VEC3_SIZE);
		break;
		case _MAG_A_3:
		spi_eeprom_read_n(EEPROM_MAG_A_3, value, VEC3_SIZE);
		break;
		case _MAG_B:
		spi_eeprom_read_n(EEPROM_MAG_B, value, VEC3_SIZE);
		break;
		case _ACCEL_B:
		spi_eeprom_read_n(EEPROM_ACCEL_B, value, VEC3_SIZE);
		break;
		case _GYRO_B:
		spi_eeprom_read_n(EEPROM_GYRO_B, value, VEC3_SIZE);
		break;
		case _BARO_HEIGHT_CAL:
		spi_eeprom_read_n(EEPROM_BARO_HEIGHT_CAL, value, SCALAR_SIZE);
		break;
		default:
		break;
	}
}

void control_set_value(CTRL_Param parameter, void* value) {
	switch (parameter) {
		case _PID_X:
		mat_copy((float*)value, 3, PID_X);
		break;
		case _PID_Y:
		mat_copy((float*)value, 3, PID_Y);
		break;
		case _PID_Z:
		mat_copy((float*)value, 3, PID_Z);
		break;
		case _X_MIX:
		mat_copy((float*)value, 3, mix_mat);
		break;
		case _Y_MIX:
		mat_copy((float*)value, 3, mix_mat + 3);
		break;
		case _Z_MIX:
		mat_copy((float*)value, 3, mix_mat + 6);
		break;
		case _POSITION_PID:
		mat_copy((float*)value, 3, position_pid);
		break;
		case _WAYPOINT_THRESHOLD:
		waypoint_threshold = *(float*)value;
		break;
		case _THRO_CONFIG:
		mat_copy((float*)value, 3, thro_config);
		break;
		case _CHANNEL_TRIM:
		mat_copy((float*)value, 3, channel_trim);
		break;
		case _CHANNEL_REVERSE:
		channel_reverse = *(int32_t*)value;
		break;
		case _HEADING_PID:
		mat_copy((float*)value, 3, heading_pid);
		break;
		case _ALTITUDE_PID:
		mat_copy((float*)value, 3, altitude_pid);
		break;
		case _ELEVATOR_TURN_P:
		elevator_turn_p = *(float*)value;
		break;
		case _FLIGHT_MODE:
		//flight_mode = *(int*)value;
		set_flight_mode(*(int*)value);
		break;
		case _DISABLE_KALMAN_UPDATE_DELAY:
		disable_kalman_update_delay = *(float*)value;
		break;
		case _CTRL_FLAGS_1:
		ctrl_flags_1 = *(int32_t*)value;
		break;
		case _AOA:
		angle_of_attack = *(float*)value;
		break;
		case _ROLL_LIMIT:
		roll_limit = *(float*)value;
		break;
		case _PITCH_LIMIT:
		pitch_limit = *(float*)value;
		break;
		case _FLIGHT_MODE_0:
		flight_mode_0 = *(int*)value;
		break;
		case _FLIGHT_MODE_1:
		flight_mode_1 = *(int*)value;
		break;
		case _FLIGHT_MODE_2:
		flight_mode_2 = *(int*)value;
		break;
		case _LOITER_RADIUS:
		loiter_radius = *(float*)value;
		break;
		case _HOME_LOITER_ALT:
		home_loiter_alt = *(float*)value;
		break;
		case _LAUNCH_THRO:
		launch_thro = *(float*)value;
		break;
		case _LAUNCH_PITCH:
		launch_pitch = *(float*)value;
		break;
		case _LAUNCH_MINACC:
		launch_minacc = *(float*)value;
		break;
		case _LAUNCH_MINSPD:
		launch_minspd = *(float*)value;
		break;
		case _LAUNCH_THRODELAY:
		launch_throdelay = *(float*)value;
		break;
		case _LAUNCH_TIME:
		launch_time = *(float*)value;
		break;
		case _FBWH_HEADING_SLEW:
		fbwh_heading_slew = *(float*)value;
		break;
		case _FBWH_ALTITUDE_SLEW:
		fbwh_altitude_slew = *(float*)value;
		break;
		case _LANDING_DESCENT_PITCH:
		landing_descent_pitch = *(float*)value;
		break;
		case _LANDING_FLAIR_PITCH:
		landing_flair_pitch = *(float*)value;
		break;
		case _LANDING_DESCENT_THROTTLE:
		landing_descent_throttle = *(float*)value;
		break;
		case _LANDING_FLAIR_PARAM:
		landing_flair_param = *(float*)value;
		break;
		case _FAILSAFE_FLIGHT_MODE:
		failsafe_flight_mode = *(int*)value;
		break;
		default:
		break;
	}
}

void control_read_value(CTRL_Param parameter, void* value) {
	switch (parameter) {
		case _PID_X:
		mat_copy(PID_X, 3, (float*)value);
		break;
		case _PID_Y:
		mat_copy(PID_Y, 3, (float*)value);
		break;
		case _PID_Z:
		mat_copy(PID_Z, 3, (float*)value);
		break;
		case _X_MIX:
		mat_copy(mix_mat, 3, (float*)value);
		break;
		case _Y_MIX:
		mat_copy(mix_mat + 3, 3, (float*)value);
		break;
		case _Z_MIX:
		mat_copy(mix_mat + 6, 3, (float*)value);
		break;
		case _POSITION_PID:
		mat_copy(position_pid, 3, (float*)value);
		break;
		case _WAYPOINT_THRESHOLD:
		*(float*)value = waypoint_threshold;
		break;
		case _THRO_CONFIG:
		mat_copy(thro_config, 3, (float*)value);
		break;
		case _CHANNEL_TRIM:
		mat_copy(channel_trim, 3, (float*)value);
		break;
		case _CHANNEL_REVERSE:
		*(int32_t*)value = channel_reverse;
		break;
		case _HEADING_PID:
		mat_copy(heading_pid, 3, (float*)value);
		break;
		case _ALTITUDE_PID:
		mat_copy(altitude_pid, 3, (float*)value);
		break;
		case _ELEVATOR_TURN_P:
		*(float*)value = elevator_turn_p;
		break;
		case _FLIGHT_MODE:
		*(int*)value = flight_mode;
		break;
		case _DISABLE_KALMAN_UPDATE_DELAY:
		*(float*)value = disable_kalman_update_delay;
		break;
		case _CTRL_FLAGS_1:
		*(int32_t*)value = ctrl_flags_1;
		break;
		case _AOA:
		*(float*)value = angle_of_attack;
		break;
		case _ROLL_LIMIT:
		*(float*)value = roll_limit;
		break;
		case _PITCH_LIMIT:
		*(float*)value = pitch_limit;
		break;
		case _FLIGHT_MODE_0:
		*(int*)value = flight_mode_0;
		break;
		case _FLIGHT_MODE_1:
		*(int*)value = flight_mode_1;
		break;
		case _FLIGHT_MODE_2:
		*(int*)value = flight_mode_2;
		break;
		case _LOITER_RADIUS:
		*(float*)value = loiter_radius;
		break;
		case _HOME_LOITER_ALT:
		*(float*)value = home_loiter_alt;
		break;
		case _LAUNCH_THRO:
		*(float*)value = launch_thro;
		break;
		case _LAUNCH_PITCH:
		*(float*)value = launch_pitch;
		break;
		case _LAUNCH_MINACC:
		*(float*)value = launch_minacc;
		break;
		case _LAUNCH_MINSPD:
		*(float*)value = launch_minspd;
		break;
		case _LAUNCH_THRODELAY:
		*(float*)value = launch_throdelay;
		break;
		case _LAUNCH_TIME:
		*(float*)value = launch_time;
		break;
		case _FBWH_HEADING_SLEW:
		*(float*)value = fbwh_heading_slew;
		break;
		case _FBWH_ALTITUDE_SLEW:
		*(float*)value = fbwh_altitude_slew;
		break;
		case _LANDING_DESCENT_PITCH:
		*(float*)value = landing_descent_pitch;
		break;
		case _LANDING_FLAIR_PITCH:
		*(float*)value = landing_flair_pitch;
		break;
		case _LANDING_DESCENT_THROTTLE:
		*(float*)value = landing_descent_throttle;
		break;
		case _LANDING_FLAIR_PARAM:
		*(float*)value = landing_flair_param;
		break;
		case _FAILSAFE_FLIGHT_MODE:
		*(int*)value = failsafe_flight_mode;
		break;
		default:
		break;
	}
}


void nav_set_vec3(CTRL_Param parameter, float* value) {
	NAV_ACK_Packet nav_ack_packet;
	nav_uart_send(0x82);
	nav_read(nav_ack_packet.reg, sizeof(nav_ack_packet.reg));
	if (nav_ack_packet.bit.status_code == NAV_ACK_OK && crc32(nav_ack_packet.reg, sizeof(nav_ack_packet.reg)) == CRC32_CHECK) {
		Set_Vec3_Request set_request;
		set_request.bit.header = NAV_SET_VEC3_REQUEST_HEADER;
		set_request.bit.parameter = parameter;
		mat_copy(value, 3, set_request.bit.data);
		set_request.bit.crc = crc32(set_request.reg, sizeof(set_request.reg) - 4);
		nav_stream(set_request.reg, sizeof(set_request.reg));
	}
	else {
		//LED_ON();
		//while(1);
		
		// do nothing
	}
}

void nav_read_vec3(CTRL_Param parameter, float* value) {
	NAV_ACK_Packet nav_ack_packet;
	nav_uart_send(0x83);
	nav_read(nav_ack_packet.reg, sizeof(nav_ack_packet.reg));
	if (nav_ack_packet.bit.status_code == NAV_ACK_OK && crc32(nav_ack_packet.reg, sizeof(nav_ack_packet.reg)) == CRC32_CHECK) {
		Read_Vec3_Request read_request;
		Read_Vec3_Response read_packet;
		read_request.bit.header = NAV_READ_VEC3_REQUEST_HEADER;
		read_request.bit.parameter = parameter;
		read_request.bit.crc = crc32(read_request.reg, sizeof(read_request.reg) - 4);
		nav_stream(read_request.reg, sizeof(read_request.reg));
		nav_read(read_packet.reg, sizeof(read_packet.reg));
		mat_copy(read_packet.bit.data, 3, value);
	}
	else {
		//LED_ON();
		//while(1);
		
		// do nothing
	}
}

void nav_set_scalar(CTRL_Param parameter, float* value) {
	NAV_ACK_Packet nav_ack_packet;
	nav_uart_send(0x84);
	nav_read(nav_ack_packet.reg, sizeof(nav_ack_packet.reg));
	if (nav_ack_packet.bit.status_code == NAV_ACK_OK && crc32(nav_ack_packet.reg, sizeof(nav_ack_packet.reg)) == CRC32_CHECK) {
		Set_Scalar_Request set_request;
		set_request.bit.header = NAV_SET_SCALAR_REQUEST_HEADER;
		set_request.bit.parameter = parameter;
		set_request.bit.data = *value;
		set_request.bit.crc = crc32(set_request.reg, sizeof(set_request.reg) - 4);
		nav_stream(set_request.reg, sizeof(set_request.reg));
	}
	else {
		//LED_ON();
		//while(1);
		
		// do nothing
	}
}

void nav_read_scalar(CTRL_Param parameter, float* value) {
	NAV_ACK_Packet nav_ack_packet;
	nav_uart_send(0x85);
	nav_read(nav_ack_packet.reg, sizeof(nav_ack_packet.reg));
	if (nav_ack_packet.bit.status_code == NAV_ACK_OK && crc32(nav_ack_packet.reg, sizeof(nav_ack_packet.reg)) == CRC32_CHECK) {
		Read_Scalar_Request read_request;
		Read_Scalar_Response read_packet;
		read_request.bit.header = NAV_READ_SCALAR_REQUEST_HEADER;
		read_request.bit.parameter = parameter;
		read_request.bit.crc = crc32(read_request.reg, sizeof(read_request.reg) - 4);
		nav_stream(read_request.reg, sizeof(read_request.reg));
		nav_read(read_packet.reg, sizeof(read_packet.reg));
		*value = read_packet.bit.data;
	}
	else {
		//LED_ON();
		//while(1);
		
		// do nothing
	}
}


// send okay acknowledge packet
void ack_ok() {
	CTRL_ACK_Packet ctrl_ack_packet;
	// set acknowledge response to ok
	ctrl_ack_packet.bit.status_code = CTRL_ACK_OK;
	// set acknowledge packet crc
	ctrl_ack_packet.bit.crc = crc32(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg) - 4);
	// send acknowledge packet
	wireless_stream(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg));
}

// send error acknowledge packet
void ack_error() {
	CTRL_ACK_Packet ctrl_ack_packet;
	// set acknowledge response to errir
	ctrl_ack_packet.bit.status_code = CTRL_ACK_ERROR;
	// set acknowledge packet crc
	ctrl_ack_packet.bit.crc = crc32(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg) - 4);
	// send acknowledge packet
	wireless_stream(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg));
}

// send error acknowledge packet, flush buffer, restart dma
void ack_error_flush_restart() {
	ack_error();
	wireless_flush();
	wireless_rx_dma_start();
}

// flush and restart dma, without ack packet
void wireless_flush_restart() {
	wireless_flush();
	wireless_rx_dma_start();
}

void enable_kalman_orientation_update() {
	nav_uart_send(0x89);
	kalman_orientation_update_enabled = true;
}

void disable_kalman_orientation_update() {
	nav_uart_send(0x8A);
	kalman_orientation_update_enabled = false;
}


void set_flight_mode(int fmode) {
	last_flight_mode = flight_mode;
	flight_mode = fmode;
	set_origin = true;
}


void arm() {
	if (nav_data_packet.bit.h_acc < 50) {
		armed = true;
		set_origin = true;
		set_home = true;

		// disable kalman position
		nav_uart_send(0x8D);
		delay_ms(10);
		// set launch position	
		nav_uart_send(0x8E);
		delay_ms(10);
		// reset kalman uncertainties
		nav_load_vec3(_KALMAN_POSITION_UNCERTAINTY);
		nav_load_vec3(_KALMAN_VELOCITY_UNCERTAINTY);
		delay_ms(10);
		// enable kalman position
		nav_uart_send(0x8C);
		
	}
}

void disarm() {
	armed = false;
	
	pwm_write_thro(-1.0f);
}

void FAILSAFE() {
	// disarm
	//arm = false;
	if (ctrl_flags_1 & CTRL_FLAGS_1_DISARM_ON_FAILSAFE_MASK) disarm();
	
	//pwm_write_thro(-1.0f);
	
	set_flight_mode(failsafe_flight_mode);
}