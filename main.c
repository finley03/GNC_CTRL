#include "main.h"

#include <stdio.h>
#include <stdlib.h>


void init();


int main(void) {
	
	init();
	
	NAV_Data_Packet nav_data_packet;
	char buffer[200];
	
	while(1) {
		delay_ms(200);
		
		nav_uart_send(0x81);
			
		for (uint32_t i = 0; i < sizeof(nav_data_packet.reg); ++i) {
			// wait until data available
			while(!SERCOM0->USART.INTFLAG.bit.RXC);
				
			nav_data_packet.reg[i] = (uint8_t) (SERCOM0->USART.DATA.reg);
		}
		
		if (gen_crc32((uint32_t) &nav_data_packet.reg[0], sizeof(nav_data_packet.reg)) != 0x2144df1c) {
			serial_print("CRC Check Failed\n");
		}
		
		
		sprintf(buffer,
		"Latitude: %f\nLongitude: %f\nHeight: %f\nHorizontal Accuracy: %f\nGNSS Satellites: %d\nAccel X: %f\nAccel Y: %f\nAccel Z: %f\nRotation X: %f\nRotation Y: %f\nRotation Z: %f\nPressure: %f\nTemperature: %f\n\n",
		nav_data_packet.bit.longitude,
		nav_data_packet.bit.latitude,
		nav_data_packet.bit.gps_height,
		nav_data_packet.bit.h_acc,
		nav_data_packet.bit.gps_satellites,
		nav_data_packet.bit.accel_x,
		nav_data_packet.bit.accel_y,
		nav_data_packet.bit.accel_z,
		nav_data_packet.bit.angularvelocity_x,
		nav_data_packet.bit.angularvelocity_y,
		nav_data_packet.bit.angularvelocity_z,
		nav_data_packet.bit.pressure,
		nav_data_packet.bit.temperature
		);
		
		serial_print(buffer);
		
		
		
	}
	
	
	return 0;
}


uint8_t system_check() {
	uint8_t state = 0;
	
	serial_print("Starting System Check...\n");
	
	// check SPI flash
	serial_print("SPI Flash...");
	if (!spi_flash_checkid()) serial_print("OK\n");
	else {
		serial_print("FAIL\n");
		state = 1;
	}
	
	
	// test navigation computer
	
	serial_print("Navigation computer test:\n");
	
	NAV_Selftest_Packet nav_selftest_packet;
	
	nav_uart_send(0x80);
	
	for (uint32_t i = 0; i < sizeof(nav_selftest_packet.reg); ++i) {
		// wait until data available
		while(!SERCOM0->USART.INTFLAG.bit.RXC);
		
		nav_selftest_packet.reg[i] = (uint8_t) (SERCOM0->USART.DATA.reg);
	}
	
	if (gen_crc32((uint32_t) &nav_selftest_packet.reg[0], sizeof(nav_selftest_packet.reg)) == 0x2144df1c) {
		serial_print("CRC Check Passed\n");
	}
	else {
		serial_print("CRC Check Failed\n");
		state = 1;
	}
	
	serial_print("NAV device ID...");
	if (nav_selftest_packet.bit.device_id == NAV_DEVICE_ID) serial_print("OK\n");
	else {
		serial_print("FAIL\n");
		state = 1;
	}
	
	serial_print("NAV IMU...");
	if (nav_selftest_packet.bit.imu_code == 0) serial_print("OK\n");
	else {
		serial_print("FAIL\n");
		state = 1;
	}
	
	serial_print("NAV Barometer...");
	if (nav_selftest_packet.bit.baro_code == 0) serial_print("OK\n");
	else {
		serial_print("FAIL\n");
		state = 1;
	}
	
	
	
	
	if (!state) serial_print("System Check Passed\n\n");
	else serial_print("System Check Failed\n\n");
	
	return state;
}


void init() {
	set_clock_48m();
	crc_init();
	delay_ms(100);
	serial_init();
	nav_uart_init();
	spi_init();
	
	// set LED to output
	REG_PORT_DIRSET1 = (1 << 11);
	
	// set SS pins high
	REG_PORT_DIRSET0 = PORT_PA25 | PORT_PA27 | PORT_PA28;
	REG_PORT_OUTSET0 = PORT_PA25 | PORT_PA27 | PORT_PA28;
	
	
	serial_print("Embedded Navigation v0.0.2\nFinley Blaine 2021\n\n");
	
	delay_ms(1000);
	system_check();
	
}
