#include "util.h"
#include "spi_eeprom.h"


extern float PID_X[3];
extern float PID_Y[3];
extern float PID_Z[3];


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


void control_load_values() {
	spi_eeprom_read_n(EEPROM_PID_X, PID_X, VEC3_SIZE);
	spi_eeprom_read_n(EEPROM_PID_Y, PID_Y, VEC3_SIZE);
	spi_eeprom_read_n(EEPROM_PID_Z, PID_Z, VEC3_SIZE);
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