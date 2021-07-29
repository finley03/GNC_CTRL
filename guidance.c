#include "guidance.h"
#include "spi_eeprom.h"

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

typedef union {
	float bit[3];

	uint8_t reg[12];
} Point;


float position_pid[3];


Point origin_point;
Point target_point;
Point previous_origin_point;
float target_plane_normal[3];
float target_line_point[3];
float target_line_vector[3];

float waypoint_threshold;

void guidance_set_origin(float* value) {
	mat_copy(origin_point.bit, 3, value);
}

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
		case ADD_ASSIGN:
		case SUB_ASSIGN:
		case MUL_ASSIGN:
			*address += 4;
			break;

		case POINT:
			*address += 13;
			break;
			
		default:
			REG_PORT_OUTSET1 = LED;
			while(1);
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
			mat_copy(origin_point.bit, 3, previous_origin_point.bit);
			mat_copy(target_point.bit, 3, origin_point.bit);
			//for (uint8_t i = 0; i < 3; ++i) {
				//origin.bit[i] = target.bit[i];
			//}
			for (uint8_t i = 0; i < 12; ++i) {
				target_point.reg[i] = spi_eeprom_read_byte(++address);
			}
			
			mat_subtract(target_point.bit, origin_point.bit, 3, target_plane_normal);
			mat_3_normalize(target_plane_normal, target_plane_normal);
			
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
			if (stack[stack_pointer - 1 != 0]) {
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
			REG_PORT_OUTSET1 = LED;
			while(1);
		break;
		}
	}
	return notend;
}

float guidance(float* position, float* target_orientation, float* target_vector, float* debug) {
	static uint32_t previous_time = 0;
	// get current time
	uint32_t current_time = read_timer_20ns();
	// calculate time difference
	uint32_t delta_time = current_time - previous_time;
	// reset previous time
	previous_time = current_time;
	// convert previous time to float
	float i_time = (float) delta_time * TIMER_S_MULTIPLIER;
	
	float ret = 0;
	
	// run read function once on init
	static bool once = true;
	if (once) {
		run_code(false);
		run_code(false);
		once = false;
	}
	
	static float average_velocity = 0;
	{
		static float last_position[3] = {0, 0, 0};
		float path_vector[3];
		float path_length;
		const float measurement_time = 5;
		mat_subtract(position, last_position, 3, path_vector);
		path_length = mat_3_length(path_vector);
		float velocity = path_length / i_time;
		average_velocity = average_velocity + ((velocity - average_velocity) * i_time / measurement_time);
		mat_copy(position, 3, last_position);
	}
	
	//const float waypoint_threshold = 1;
	
	// turn parameters
	static float turn_center[3];
	static float turn_start[3];
	static float turn_end[3];
	static float turn_angle;
	static float turn_radius;
	static float turn_axis[3];
	// true if craft is turning to new way point
	static bool turn = false;
	// if not turning continuously check whether craft has reached way point
	if (!turn) {
		float target_dotp_value = mat_dotp(target_point.bit, target_plane_normal, 3);
	
		// find distance from point to target plane
		// negative when moving towards origin
		float position_target_dotp_value = mat_dotp(position, target_plane_normal, 3);
		
		mat_copy(origin_point.bit, 3, target_line_point);
		mat_copy(target_plane_normal, 3, target_line_vector);
	
		// if distance is less than 15 (> -15) get new point
		if (position_target_dotp_value >= target_dotp_value - waypoint_threshold) {
			run_code(false);
			turn = true;
			
			// assign turn parameters
			float waypoint_turn_start_vector[3];
			mat_subtract(previous_origin_point.bit, origin_point.bit, 3, waypoint_turn_start_vector);
			mat_3_normalize(waypoint_turn_start_vector, waypoint_turn_start_vector);
			mat_scalar_product(waypoint_turn_start_vector, waypoint_threshold, 3, waypoint_turn_start_vector);
			
			float waypoint_turn_end_vector[3];
			mat_subtract(target_point.bit, origin_point.bit, 3, waypoint_turn_end_vector);
			mat_3_normalize(waypoint_turn_end_vector, waypoint_turn_end_vector);
			mat_scalar_product(waypoint_turn_end_vector, waypoint_threshold, 3, waypoint_turn_end_vector);
			
			turn_angle = 180 - degrees(acos(mat_dotp(waypoint_turn_start_vector, waypoint_turn_end_vector, 3) / (waypoint_threshold * waypoint_threshold)));
			float waypoint_turn_center_distance = waypoint_threshold / sin(radians(turn_angle) / 2);
			
			float waypoint_turn_center_vector[3];
			mat_add(waypoint_turn_start_vector, waypoint_turn_end_vector, 3, waypoint_turn_center_vector);
			mat_3_normalize(waypoint_turn_center_vector, waypoint_turn_center_vector);
			mat_scalar_product(waypoint_turn_center_vector, waypoint_turn_center_distance, 3, waypoint_turn_center_vector);
			
			// assign turn center
			mat_add(origin_point.bit, waypoint_turn_center_vector, 3, turn_center);
			// assign turn start
			mat_add(origin_point.bit, waypoint_turn_start_vector, 3, turn_start);
			// assign turn end
			mat_add(origin_point.bit, waypoint_turn_end_vector, 3, turn_end);
			// assign turn radius
			turn_radius = waypoint_threshold / tan(radians(turn_angle) / 2);
			// assign turn axis unnormalized
			mat_crossp(waypoint_turn_end_vector, waypoint_turn_start_vector, turn_axis);
		}
	}
	// separate if statement to enable execution if turn is set true in previous statement
	if (turn) {
		// get vector from turn center to current position
		float turn_center_position[3];
		mat_subtract(position, turn_center, 3, turn_center_position);
		
		// get vector from turn center to turn start
		float turn_center_start[3];
		mat_subtract(turn_start, turn_center, 3, turn_center_start);
		
		// get angle round turn
		float angle_round_turn = degrees(acos(mat_dotp(turn_center_start, turn_center_position, 3) / (turn_radius * mat_3_length(turn_center_position))));
		
		float target_angle;
		if (angle_round_turn >= turn_angle - 1) {
			turn = false;
			target_angle = turn_angle;
		}
		else {
			// calculate a target angle for next iteration
			// use rolling average velocity to calculate
			target_angle = angle_round_turn + degrees(atan(average_velocity * i_time / turn_radius));
		}
		ret = 1;
		
		// calculate position vector of target line
		float turn_center_target_point[3];
		vec_rotate_axis(turn_center_start, turn_axis, radians(target_angle), turn_center_target_point);
		mat_add(turn_center, turn_center_target_point, 3, target_line_point);
		
		// calculate target vector of target line
		// target vector for start of turn
		float turn_start_target_vector[3];
		mat_subtract(origin_point.bit, turn_start, 3, turn_start_target_vector);
		vec_rotate_axis(turn_start_target_vector, turn_axis, radians(target_angle), target_line_vector);
		mat_3_normalize(target_line_vector, target_line_vector);
	}
	
	
	// vector from line point to current position
	float line_point_position_vector[3];
	mat_subtract(position, target_line_point, 3, line_point_position_vector);
	// distance from line point to position when projected on to target line
	float line_position_projected = mat_dotp(line_point_position_vector, target_line_vector, 3);
	// vector from line point to position projected onto target line
	float line_position_projected_vector[3];
	mat_scalar_product(target_line_vector, line_position_projected, 3, line_position_projected_vector);
	// absolute location of position projected to line
	float position_projected_vector[3];
	mat_add(target_line_point, line_position_projected_vector, 3, position_projected_vector);
	// calculate offset vector
	float offset_vector[3];
	mat_subtract(position_projected_vector, position, 3, offset_vector);
	
	// scale offset vector to add to target vector
	mat_scalar_product(offset_vector, position_pid[0], 3, offset_vector);
	
	// add to target plane normal to get target vector
	mat_add(target_line_vector, offset_vector, 3, target_vector);
	
	// normalize target vector
	mat_3_normalize(target_vector, target_vector);
	
	*debug = average_velocity;
	return ret;
}