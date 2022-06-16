#include "guidance.h"
#include "spi_eeprom.h"
#include "uart.h"
#include <complex.h>
#define CMPLXF(x, y) ((float complex)((float)(x) + I * (float)(y)))

#define POINT 0x01
#define PRINT 0x02
#define WHILE 0x03
#define WHILE_VAR 0x04
#define ENDWHILE 0x05
#define FOR 0x06
#define ENDFOR 0x07
#define INTEGER 0x08
#define INCREMENT 0x09
#define DECREMENT 0x0A
#define ADD 0x0B
#define ADD_ASSIGN 0x0C
#define ASSIGN 0x0D
#define SUB 0x0E
#define SUB_ASSIGN 0x0F
#define MUL 0x10
#define MUL_ASSIGN 0x11
#define DIV 0x12
#define DIV_ASSIGN 0x13
#define FOR_VAR 0x14
#define IF_Z 0x15
#define IF_NZ 0x16
#define IF_POS 0x17
#define IF_NEG 0x18
#define ENDIF 0x19
#define BREAK_WHILE 0x20
#define END 0x21

#define ENABLE_DISABLING_KALMAN_UPDATE_MASK (1 << 0)
#define LOITER_CW_MASK (1 << 1)

typedef union {
	float bit[3];

	uint8_t reg[12];
} Point;


float position_pid[3];
float heading_pid[3];
float altitude_pid[3];

extern bool kalman_orientation_update_enabled;
int32_t ctrl_flags_1;

float loiter_radius;
float home_loiter_alt;

Point origin_point;
Point target_point;
//Point previous_origin_point;
float target_plane_normal[3];
//float target_line_point[3];
//float target_line_vector[3];

float waypoint_threshold;
float disable_kalman_update_delay;
extern float roll_limit;
extern float pitch_limit;

static bool once;

//void guidance_set_origin(float* value) {
	//mat_copy(origin_point.bit, 3, value);
//}

void skip_to(uint32_t* address, uint8_t opcode) {
	uint8_t byte;
	while ((byte = spi_eeprom_read_byte(*address)) != opcode) {
		switch (byte) {
		case WHILE:
		case ENDWHILE:
		case ENDFOR:
		case ENDIF:
		case BREAK_WHILE:
		case END:
			*address += 1;
			break;

		case FOR_VAR:
		case WHILE_VAR:
		case INCREMENT:
		case DECREMENT:
		case PRINT:
		case IF_Z:
		case IF_NZ:
		case IF_POS:
		case IF_NEG:
			*address += 2;
			break;

		case FOR:
		case ASSIGN:
			*address += 3;
			break;

		case INTEGER:
		case ADD:
		case SUB:
		case MUL:
		case DIV:
		case ADD_ASSIGN:
		case SUB_ASSIGN:
		case MUL_ASSIGN:
		case DIV_ASSIGN:
			*address += 4;
			break;

		case POINT:
			*address += 13;
			break;
			
		default:
			//REG_PORT_OUTSET1 = LED;
			//while(1);
			// trigger failsafe, program flow broken
			FAILSAFE();
			break;
		}
	}
}

// returns false if end of code
bool run_code(bool reset) {
	static int16_t integers[16];
	//uint32_t labels[16];
	static uint32_t stack[16];
	static int32_t stack_pointer = -1;

	static uint32_t address = 0;
	
	if (reset) {
		stack_pointer = -1;
		address = 0;
	}
	
	bool notend = true;

	bool run = true;
	// code loop
	while (run) {
		switch (spi_eeprom_read_byte(address)) {
		case POINT:
		{
			//mat_copy(origin_point.bit, 3, previous_origin_point.bit);
			// only copy if route is not being reset
			if (!reset) mat_copy(target_point.bit, 3, origin_point.bit);
			//for (uint8_t i = 0; i < 3; ++i) {
				//origin.bit[i] = target.bit[i];
			//}
			for (uint8_t i = 0; i < 12; ++i) {
				target_point.reg[i] = spi_eeprom_read_byte(++address);
			}
			
			mat_subtract(target_point.bit, origin_point.bit, 3, target_plane_normal);
			vec_3_normalize(target_plane_normal, target_plane_normal);
			
			++address;
			//printf("%f, %f, %f\n", target.bit[0], target.bit[1], target.bit[2]);
			run = false;
		}
		break;
		case PRINT:
		{
			//uint8_t id = code[address + 1];
			//printf("%d\n", integers[id]);
			address += 2;
		}
		break;
		case END:
			run = false;
			notend = false;
		break;
		case FOR_VAR:
		{
			uint8_t id = spi_eeprom_read_byte(address + 1);
			if (integers[id] > 0) {
				stack[++stack_pointer] = (uint32_t) integers[id];
				stack[++stack_pointer] = address + 2;
				address += 2;
			}
			else {
				skip_to(&address, ENDFOR);
				++address;
			}
		}
		break;
		case FOR:
		{
			uint16_t value = ((uint16_t) spi_eeprom_read_byte(address + 1)) | ((uint16_t) spi_eeprom_read_byte(address + 2) << 8);
			if (value > 0) {
				stack[++stack_pointer] = (uint32_t) value;
				stack[++stack_pointer] = address + 3;
				address += 3;
			}
			else {
				skip_to(&address, ENDFOR);
				++address;
			}
		}
		break;
		case ENDFOR:
		{
			--stack[stack_pointer - 1];
			if (stack[stack_pointer - 1] != 0) {
				address = stack[stack_pointer];
			}
			else {
				stack_pointer -= 2;
				++address;
			}
		}
		break;
		case WHILE:
		{
			stack[++stack_pointer] = (uint32_t) WHILE << 16;
			stack[++stack_pointer] = address + 1;
			++address;
		}
		break;
		case WHILE_VAR:
		{
			stack[++stack_pointer] = (uint32_t) spi_eeprom_read_byte(address + 1) | (uint32_t) WHILE_VAR << 16;
			stack[++stack_pointer] = address + 2;
			address += 2;
		}
		break;
		case ENDWHILE:
		{
			if (stack[stack_pointer - 1] >> 16 == WHILE_VAR) {
				if (integers[stack[stack_pointer - 1]] != 0) {
					address = stack[stack_pointer];
				}
				else {
					stack_pointer -= 2;
					++address;
				}
			}
			else {
				address = stack[stack_pointer];
			}
		}
		break;
		case INTEGER:
		{
			uint8_t id = spi_eeprom_read_byte(address + 1);
			int16_t value = ((uint16_t) spi_eeprom_read_byte(address + 2)) | ((uint16_t) spi_eeprom_read_byte(address + 3) << 8);
			integers[id] = value;
			address += 4;
		}
		break;
		case INCREMENT:
		{
			++integers[spi_eeprom_read_byte(address + 1)];
			address += 2;
		}
		break;
		case DECREMENT:
		{
			--integers[spi_eeprom_read_byte(address + 1)];
			address += 2;
		}
		break;
		case ADD:
		{
			uint8_t id1 = spi_eeprom_read_byte(address + 1);
			uint8_t id2 = spi_eeprom_read_byte(address + 2);
			uint8_t id3 = spi_eeprom_read_byte(address + 3);
			integers[id3] = integers[id1] + integers[id2];
			address += 4;
		}
		break;
		case ADD_ASSIGN:
		{
			uint8_t id = spi_eeprom_read_byte(address + 1);
			int16_t value = ((uint16_t) spi_eeprom_read_byte(address + 2)) | ((uint16_t) spi_eeprom_read_byte(address + 3) << 8);
			integers[id] += value;
			address += 4;
		}
		break;
		case ASSIGN:
		{
			uint8_t id1 = spi_eeprom_read_byte(address + 1);
			uint8_t id2 = spi_eeprom_read_byte(address + 2);
			integers[id1] = integers[id2];
			address += 3;
		}
		break;
		case SUB:
		{
			uint8_t id1 = spi_eeprom_read_byte(address + 1);
			uint8_t id2 = spi_eeprom_read_byte(address + 2);
			uint8_t id3 = spi_eeprom_read_byte(address + 3);
			integers[id3] = integers[id1] - integers[id2];
			address += 4;
		}
		break;
		case SUB_ASSIGN:
		{
			uint8_t id = spi_eeprom_read_byte(address + 1);
			int16_t value = ((uint16_t) spi_eeprom_read_byte(address + 2)) | ((uint16_t) spi_eeprom_read_byte(address + 3) << 8);
			integers[id] -= value;
			address += 4;
		}
		break;
		case MUL:
		{
			uint8_t id1 = spi_eeprom_read_byte(address + 1);
			uint8_t id2 = spi_eeprom_read_byte(address + 2);
			uint8_t id3 = spi_eeprom_read_byte(address + 3);
			integers[id3] = integers[id1] * integers[id2];
			address += 4;
		}
		break;
		case MUL_ASSIGN:
		{
			uint8_t id = spi_eeprom_read_byte(address + 1);
			int16_t value = ((uint16_t) spi_eeprom_read_byte(address + 2)) | ((uint16_t) spi_eeprom_read_byte(address + 3) << 8);
			integers[id] *= value;
			address += 4;
		}
		break;
		case DIV:
		{
			uint8_t id1 = spi_eeprom_read_byte(address + 1);
			uint8_t id2 = spi_eeprom_read_byte(address + 2);
			uint8_t id3 = spi_eeprom_read_byte(address + 3);
			integers[id3] = integers[id1] / integers[id2];
			address += 4;
		}
		break;
		case DIV_ASSIGN:
		{
			uint8_t id = spi_eeprom_read_byte(address + 1);
			int16_t value = ((uint16_t) spi_eeprom_read_byte(address + 2)) | ((uint16_t) spi_eeprom_read_byte(address + 3) << 8);
			integers[id] /= value;
			address += 4;
		}
		break;
		case IF_Z:
		{
			uint8_t id = spi_eeprom_read_byte(address + 1);
			if (integers[id] == 0) {
				address += 2;
			}
			else {
				skip_to(&address, ENDIF);
				++address;
			}
		}
		break;
		case IF_NZ:
		{
			uint8_t id = spi_eeprom_read_byte(address + 1);
			if (integers[id] != 0) {
				address += 2;
			}
			else {
				skip_to(&address, ENDIF);
				++address;
			}
				
		}
		break;
		case ENDIF:
			++address;
			break;
		case IF_POS:
		{
			uint8_t id = spi_eeprom_read_byte(address + 1);
			if (integers[id] > 0) {
				address += 2;
			}
			else {
				skip_to(&address, ENDIF);
				++address;
			}

		}
		break;
		case IF_NEG:
		{
			uint8_t id = spi_eeprom_read_byte(address + 1);
			if (integers[id] < 0) {
				address += 2;
			}
			else {
				skip_to(&address, ENDIF);
				++address;
			}

		}
		break;
		case BREAK_WHILE:
		{
			skip_to(&address, ENDWHILE);
			++address;
		}
		break;
		default:
			//printf("Unrecognized command %02X at address %08X\n", code[address], address);
			++address;
			//REG_PORT_OUTSET1 = LED;
			//while(1);
			// program flow broken, trigger failsafe
			FAILSAFE();
		break;
		}
	}
	return notend;
}

void reset_guidance() {
	once = true;
}

//void guidance(float* position, float orientation_z, float* roll, float* pitch) {
	//*roll = -orientation_z * 0.7;
	//pitch = 0;
//}

void guidance_auto_waypoint(float* position, float* orientation, bool* set_origin) {
	static uint32_t previous_time = 0;
	// get current time
	uint32_t current_time = read_timer_20ns();
	// calculate time difference
	uint32_t delta_time = current_time - previous_time;
	// reset previous time
	previous_time = current_time;
	// convert previous time to float
	float i_time = delta_time * TIMER_S_MULTIPLIER;
	
	
	float target_heading = 0.0f;
	float roll = 0.0f, pitch = 0.0f;
	
	if (once) {
		mat_copy(position, 3, origin_point.bit);
		run_code(true);
		once = false;
	}
	
	if (*set_origin) {
		mat_copy(position, 3, origin_point.bit);
		*set_origin = false;
	}
	
	{ // scoped so variables are deleted
		static uint32_t waypoint_start_time;
		uint32_t waypoint_current_time;
				
		// get vector from position to target
		float position_target_vector[3];
		mat_subtract(target_point.bit, position, 3, position_target_vector);
		// check if waypoint has been reached
		
		// if the waypoint is missed and the plane goes past it, count it as hit
		float waypoint_dotp = mat_dotp(target_point.bit, target_plane_normal, 3);
		float current_dotp = mat_dotp(position, target_plane_normal, 3);
		
		if (vec_2_length(position_target_vector) <= waypoint_threshold || current_dotp > waypoint_dotp) {
			// set next waypoint
			run_code(false);
			// disable kalman orientation update if flag is set
			if (ctrl_flags_1 & ENABLE_DISABLING_KALMAN_UPDATE_MASK) {
				disable_kalman_orientation_update();
				waypoint_start_time = read_timer_20ns();
			}
		}
		
		// reenable orientation update
		if (!kalman_orientation_update_enabled && (ctrl_flags_1 & ENABLE_DISABLING_KALMAN_UPDATE_MASK)) {
			waypoint_current_time = read_timer_20ns();
			uint32_t waypoint_delta_time = waypoint_current_time - waypoint_start_time;
			float waypoint_time = (float)waypoint_delta_time * TIMER_S_MULTIPLIER;
			if (waypoint_time >= MIN_2(80.0f, disable_kalman_update_delay)) { // 80 seconds is near the max for a 32 bit unsigned int
				enable_kalman_orientation_update();
			}
		}
	}
	
	// get vector of line
	float line_vector[3];
	mat_subtract(target_point.bit, origin_point.bit, 3, line_vector);
	
	//// get normalized vector of line
	//float line_vector_2d_norm[2];
	//vec_2_normalize(line_vector_2d, line_vector_2d_norm);
	
	// calculate distance between current position and line in 2d (negative is left of line)
	float position_offset = ((target_point.bit[0] - origin_point.bit[0]) * (origin_point.bit[1] - position[1]) -
		(origin_point.bit[0] - position[0]) * (target_point.bit[1] - origin_point.bit[1])) / vec_2_length(line_vector);
		
	
	// run PID on position offset
	float position_offset_pid;
	{
		float error = position_offset;
		static float previous_error = 0;
		static float integral = 0;
		
		// proportional
		float proportional = error;
		
		// integral
		float error_dt = error * i_time;
		integral += error_dt;
		
		// derivative
		float delta_error = error - previous_error;
		float derivative = delta_error / i_time;
		
		position_offset_pid = proportional * position_pid[0] + integral * position_pid[1] + derivative * position_pid[2];
		
		previous_error = error;
	}
	
	// get line heading
	float complex line_complex = CMPLXF(line_vector[0], line_vector[1]);
	float line_heading = degrees(cargf(line_complex));
	
	// set target heading
	target_heading = line_heading + degrees(atanf(radians(position_offset_pid)));
	
	if (target_heading > 180) target_heading -= 360;
	if (target_heading <= -180) target_heading += 360;
	
	// run PID on heading
	float measured_heading = orientation[2];
	if (measured_heading > target_heading + 180) target_heading += 360;
	if (measured_heading < target_heading - 180) target_heading -= 360;
	
	{
		float error = target_heading - measured_heading;
		static float previous_error = 0;
		static float integral = 0;
		
		// proportional
		float proportional = error;
		
		// integral
		float error_dt = error * i_time;
		integral += error_dt;
		
		// derivative
		float delta_error = error - previous_error;
		float derivative = delta_error / i_time;
		
		roll = proportional * heading_pid[0] + integral * heading_pid[1] + derivative * heading_pid[2];
		
		previous_error = error;
	}
	
	// run PID on altitude
	{
		// error points UP
		float error = position[2] - target_point.bit[2];
		static float previous_error = 0;
		static float integral = 0;
		
		// proportional
		float proportional = error;
		
		// integral
		float error_dt = error * i_time;
		integral += error_dt;
		
		// derivative
		float delta_error = error - previous_error;
		float derivative = delta_error / i_time;
		
		pitch = proportional * altitude_pid[0] + integral * altitude_pid[1] + derivative * altitude_pid[2];
		
		previous_error = error;
	}
	
	control(roll, pitch, orientation);
}

void guidance_manual(PWM_in* pwm_in, float* orientation) { // THERE IS A BUG HERE WITH KALMAN UPDATE - MUST FIX LATER OR WILL FORGET IT EXISTS
	//static bool kalman_orientation_update_enabled = false;
	if (ctrl_flags_1 & ENABLE_DISABLING_KALMAN_UPDATE_MASK) {
		if (ABS(pwm_in->ale) > 0.05) {
			if (kalman_orientation_update_enabled) {
				disable_kalman_orientation_update();
				//kalman_orientation_update_enabled = false;
			}
		}
		else {
			if (!kalman_orientation_update_enabled) {
				enable_kalman_orientation_update();
				//kalman_orientation_update_enabled = true;
			}
		}
	}
	
	float roll = pwm_in->ale * roll_limit;
	float pitch = pwm_in->elev * pitch_limit;
	control_mthrottle(pwm_in->thro, roll, pitch, orientation);
}

void guidance_manual_heading_hold(PWM_in* pwm_in, float* position, float* orientation, bool* set_origin) {
	static uint32_t previous_time = 0;
	// get current time
	uint32_t current_time = read_timer_20ns();
	// calculate time difference
	uint32_t delta_time = current_time - previous_time;
	// reset previous time
	previous_time = current_time;
	// convert previous time to float
	float i_time = delta_time * TIMER_S_MULTIPLIER;
	
	
	static float heading_lock = 0.0f;
	static float altitude_lock = 0.0f;
	float roll = 0.0f, pitch = 0.0f;
	
	if (*set_origin) {
		heading_lock = orientation[2];
		altitude_lock = position[2];
		*set_origin = false;
	}
	
	// set to true when turning
	// used to detect when to reset heading lock
	static bool end_turn = false;
	
	//static bool kalman_orientation_update_enabled = false;
	if (ABS(pwm_in->ale) > 0.05f) {
		if (kalman_orientation_update_enabled && (ctrl_flags_1 & ENABLE_DISABLING_KALMAN_UPDATE_MASK)) {
			disable_kalman_orientation_update();
			//kalman_orientation_update_enabled = false;
		}
		
		roll = pwm_in->ale * roll_limit;
		end_turn = true;
	}
	else {
		if (!kalman_orientation_update_enabled) {
			enable_kalman_orientation_update();
			//kalman_orientation_update_enabled = true;
			// set heading lock
		}
		
		if (end_turn) {
			heading_lock = orientation[2];
			end_turn = false;
		}
		
		// run PID on heading
		float measured_heading = orientation[2];
		if (heading_lock > measured_heading + 180) measured_heading += 360;
		if (heading_lock < measured_heading - 180) measured_heading -= 360;
		
		{
			float error = heading_lock - measured_heading;
			static float previous_error = 0;
			static float integral = 0;
			
			// proportional
			float proportional = error;
			
			// integral
			float error_dt = error * i_time;
			integral += error_dt;
			
			// derivative
			float delta_error = error - previous_error;
			float derivative = delta_error / i_time;
			
			roll = proportional * heading_pid[0] + integral * heading_pid[1] + derivative * heading_pid[2];
			
			previous_error = error;
		}
	}
	
	static bool altitude_once = true;
	if (ABS(pwm_in->elev) > 0.05f) {
		pitch = pwm_in->elev * pitch_limit;
		altitude_once = true;
	}
	else {
		if (altitude_once) {
			altitude_lock = position[2];
			altitude_once = false;
		}
		
		// run PID on altitude
		{
			// error points UP
			float error = position[2] - altitude_lock;
			static float previous_error = 0;
			static float integral = 0;
			
			// proportional
			float proportional = error;
			
			// integral
			float error_dt = error * i_time;
			integral += error_dt;
			
			// derivative
			float delta_error = error - previous_error;
			float derivative = delta_error / i_time;
			
			pitch = proportional * altitude_pid[0] + integral * altitude_pid[1] + derivative * altitude_pid[2];
			
			previous_error = error;
		}
	}
	
	//pitch = pwm_in->elev * 15;
	
	control(roll, pitch, orientation);
}


void guidance_loiter(float* position, float* orientation, bool* set_origin) {
	static uint32_t previous_time = 0;
	// get current time
	uint32_t current_time = read_timer_20ns();
	// calculate time difference
	uint32_t delta_time = current_time - previous_time;
	// reset previous time
	previous_time = current_time;
	// convert previous time to float
	float i_time = delta_time * TIMER_S_MULTIPLIER;
		
		
	float target_heading = 0.0f;
	float roll = 0.0f, pitch = 0.0f;
	
	static float loiter_point[3];
		
	if (*set_origin) {
		mat_copy(position, 3, loiter_point);
		*set_origin = false;
	}
	
	float point_position_vector[2];
	mat_subtract(position, loiter_point, 2, point_position_vector);
	
	float position_offset = vec_2_length(point_position_vector) - loiter_radius;
	
	position_offset = (ctrl_flags_1 & LOITER_CW_MASK) ? -position_offset : position_offset;
		
	// run PID on position offset
	float position_offset_pid;
	{
		float error = position_offset;
		static float previous_error = 0;
		static float integral = 0;
			
		// proportional
		float proportional = error;
			
		// integral
		float error_dt = error * i_time;
		integral += error_dt;
			
		// derivative
		float delta_error = error - previous_error;
		float derivative = delta_error / i_time;
			
		position_offset_pid = proportional * position_pid[0] + integral * position_pid[1] + derivative * position_pid[2];
			
		previous_error = error;
	}
		
	// get line heading
	float complex line_complex;
	if (ctrl_flags_1 & LOITER_CW_MASK) line_complex = CMPLXF(point_position_vector[1], -point_position_vector[0]);
	else line_complex = CMPLXF(-point_position_vector[1], point_position_vector[0]);
	//float complex line_complex = CMPLXF(line_vector[0], line_vector[1]);
	float line_heading = degrees(cargf(line_complex));
		
	// set target heading
	target_heading = line_heading + degrees(atanf(radians(position_offset_pid)));
		
	if (target_heading > 180) target_heading -= 360;
	if (target_heading <= -180) target_heading += 360;
		
	// run PID on heading
	float measured_heading = orientation[2];
	if (measured_heading > target_heading + 180) target_heading += 360;
	if (measured_heading < target_heading - 180) target_heading -= 360;
		
	{
		float error = target_heading - measured_heading;
		static float previous_error = 0;
		static float integral = 0;
			
		// proportional
		float proportional = error;
			
		// integral
		float error_dt = error * i_time;
		integral += error_dt;
			
		// derivative
		float delta_error = error - previous_error;
		float derivative = delta_error / i_time;
			
		roll = proportional * heading_pid[0] + integral * heading_pid[1] + derivative * heading_pid[2];
			
		previous_error = error;
	}
		
	// run PID on altitude
	{
		// error points UP
		float error = position[2] - loiter_point[2];
		static float previous_error = 0;
		static float integral = 0;
			
		// proportional
		float proportional = error;
			
		// integral
		float error_dt = error * i_time;
		integral += error_dt;
			
		// derivative
		float delta_error = error - previous_error;
		float derivative = delta_error / i_time;
			
		pitch = proportional * altitude_pid[0] + integral * altitude_pid[1] + derivative * altitude_pid[2];
			
		previous_error = error;
	}
		
	control(roll, pitch, orientation);
}

void guidance_rtl(float* position, float* orientation) {
	static uint32_t previous_time = 0;
	// get current time
	uint32_t current_time = read_timer_20ns();
	// calculate time difference
	uint32_t delta_time = current_time - previous_time;
	// reset previous time
	previous_time = current_time;
	// convert previous time to float
	float i_time = delta_time * TIMER_S_MULTIPLIER;
		
		
	float target_heading = 0.0f;
	float roll = 0.0f, pitch = 0.0f;
		
	float position_offset = vec_2_length(position) - loiter_radius;
		
	position_offset = (ctrl_flags_1 & LOITER_CW_MASK) ? -position_offset : position_offset;
		
	// run PID on position offset
	float position_offset_pid;
	{
		float error = position_offset;
		static float previous_error = 0;
		static float integral = 0;
			
		// proportional
		float proportional = error;
			
		// integral
		float error_dt = error * i_time;
		integral += error_dt;
			
		// derivative
		float delta_error = error - previous_error;
		float derivative = delta_error / i_time;
			
		position_offset_pid = proportional * position_pid[0] + integral * position_pid[1] + derivative * position_pid[2];
			
		previous_error = error;
	}
		
	// get line heading
	float complex line_complex;
	if (ctrl_flags_1 & LOITER_CW_MASK) line_complex = CMPLXF(position[1], -position[0]);
	else line_complex = CMPLXF(-position[1], position[0]);
	//float complex line_complex = CMPLXF(line_vector[0], line_vector[1]);
	float line_heading = degrees(cargf(line_complex));
		
	// set target heading
	target_heading = line_heading + degrees(atanf(radians(position_offset_pid)));
		
	if (target_heading > 180) target_heading -= 360;
	if (target_heading <= -180) target_heading += 360;
		
	// run PID on heading
	float measured_heading = orientation[2];
	if (measured_heading > target_heading + 180) target_heading += 360;
	if (measured_heading < target_heading - 180) target_heading -= 360;
		
	{
		float error = target_heading - measured_heading;
		static float previous_error = 0;
		static float integral = 0;
			
		// proportional
		float proportional = error;
			
		// integral
		float error_dt = error * i_time;
		integral += error_dt;
			
		// derivative
		float delta_error = error - previous_error;
		float derivative = delta_error / i_time;
			
		roll = proportional * heading_pid[0] + integral * heading_pid[1] + derivative * heading_pid[2];
			
		previous_error = error;
	}
		
	// run PID on altitude
	{
		// error points UP
		float error = position[2] - home_loiter_alt;
		static float previous_error = 0;
		static float integral = 0;
			
		// proportional
		float proportional = error;
			
		// integral
		float error_dt = error * i_time;
		integral += error_dt;
			
		// derivative
		float delta_error = error - previous_error;
		float derivative = delta_error / i_time;
			
		pitch = proportional * altitude_pid[0] + integral * altitude_pid[1] + derivative * altitude_pid[2];
			
		previous_error = error;
	}
		
	control(roll, pitch, orientation);
}