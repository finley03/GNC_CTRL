#include "util.h"
#include "spi_eeprom.h"
#include "uart.h"
#include "dma.h"


extern float PID_X[3];
extern float PID_Y[3];
extern float PID_Z[3];
extern float mix_mat[9];
extern float position_pid[3];
extern float waypoint_threshold;
extern float thro_config[3];


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
		spi_eeprom_read_n(EEPROM_THRO_CONFIG, &thro_config, VEC3_SIZE);
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
		spi_eeprom_write_n_s(EEPROM_THRO_CONFIG, &thro_config, VEC3_SIZE);
		break;
		default:
		break;
	}
}

void control_write_value(CTRL_Param parameter, float* value) {
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

void control_read_eeprom(CTRL_Param parameter, float* value) {
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
		case _X_MIX:
		mat_copy(value, 3, mix_mat);
		break;
		case _Y_MIX:
		mat_copy(value, 3, mix_mat + 3);
		break;
		case _Z_MIX:
		mat_copy(value, 3, mix_mat + 6);
		break;
		case _POSITION_PID:
		mat_copy(value, 3, position_pid);
		break;
		case _WAYPOINT_THRESHOLD:
		waypoint_threshold = *value;
		break;
		case _THRO_CONFIG:
		mat_copy(value, 3, thro_config);
		break;
		default:
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
		case _X_MIX:
		mat_copy(mix_mat, 3, value);
		break;
		case _Y_MIX:
		mat_copy(mix_mat + 3, 3, value);
		break;
		case _Z_MIX:
		mat_copy(mix_mat + 6, 3, value);
		break;
		case _POSITION_PID:
		mat_copy(position_pid, 3, value);
		break;
		case _WAYPOINT_THRESHOLD:
		*value = waypoint_threshold;
		break;
		case _THRO_CONFIG:
		mat_copy(thro_config, 3, value);
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
		LED_ON();
		while(1);
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
		LED_ON();
		while(1);
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
		LED_ON();
		while(1);
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
		LED_ON();
		while(1);
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