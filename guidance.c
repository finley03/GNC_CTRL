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


Point origin;
Point target;

void guidance_set_origin(float* value) {
	mat_copy(origin.bit, 3, value);
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
			mat_copy(target.bit, 3, origin.bit);
			//for (uint8_t i = 0; i < 3; ++i) {
				//origin.bit[i] = target.bit[i];
			//}
			for (uint8_t i = 0; i < 12; ++i) {
				target.reg[i] = spi_eeprom_read_byte(++address);
			}
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
			//printf("Unrecognised command %02X at address %08X\n", code[address], address);
			++address;
			REG_PORT_OUTSET1 = LED;
			while(1);
		break;
		}
	}
	return notend;
}

float guidance(float* position, float* target_orientation, float* target_vector, float* debug) {
	// run read function once on init
	static bool once = true;
	if (once) {
		run_code(false);
		run_code(false);
		once = false;
	}
	
	// find vector from origin point to target point
	float origin_target_vector[3];
	mat_subtract(target.bit, origin.bit, 3, origin_target_vector);
	
	// find normal to plane located at target point
	float target_plane_normal[3];
	mat_3_normalize(origin_target_vector, target_plane_normal);
	
	float target_dotp_value = mat_dotp(target.bit, target_plane_normal, 3);
	
	// find distance from point to target plane
	// negative when moving towards origin
	float position_target_dotp_value = mat_dotp(position, target_plane_normal, 3);
	
	// if distance is less than 15 (> -15) get new point
	if (position_target_dotp_value >= target_dotp_value) {
		run_code(false);
	}
	
	// offset from line for position
	float offset_vector[3];
	// vector from origin to current position
	float origin_position_vector[3];
	mat_subtract(position, origin.bit, 3, origin_position_vector);
	// distance from origin to position when projected on to target line
	float origin_position_projected = mat_dotp(origin_position_vector, target_plane_normal, 3);
	// vector from origin to position projected onto target line
	float origin_position_projected_vector[3];
	mat_scalar_product(target_plane_normal, origin_position_projected, 3, origin_position_projected_vector);
	// absolute location of position projected to line
	float position_projected_vector[3];
	mat_add(origin.bit, origin_position_projected_vector, 3, position_projected_vector);
	// calculate offset vector
	mat_subtract(position_projected_vector, position, 3, offset_vector);
	
	// scale offset vector to add to target vector
	mat_scalar_product(offset_vector, 0.3, 3, offset_vector);
	
	// add to target plane normal to get target vector
	mat_add(target_plane_normal, offset_vector, 3, target_vector);
	
	*debug = target_dotp_value;
	return position_target_dotp_value;
}