#include "main.h"


void init();


static NAV_Data_Packet nav_data_packet;
static NAV_Selftest_Packet nav_selftest_packet;
static CTRL_ACK_Packet ctrl_ack_packet;


int main(void) {
	
	init();
	
	

	
	while(1) {
		delay_ms(20);
		
		REG_PORT_OUTSET1 = LED;
		nav_uart_send(0x81);
			
		for (uint32_t i = 0; i < sizeof(nav_data_packet.reg); ++i) {
			// wait until data available
			while(!SERCOM0->USART.INTFLAG.bit.RXC);
				
			nav_data_packet.reg[i] = (uint8_t) (SERCOM0->USART.DATA.reg);
		}
		
		if (gen_crc32((uint32_t) &nav_data_packet.reg[0], sizeof(nav_data_packet.reg)) != CRC32_CHECK) {
			//serial_print("CRC Check Failed\n");
			REG_PORT_OUTSET1 = LED;
		}
		REG_PORT_OUTCLR1 = LED;
		

		
		
		// check for data request from computer
		// MSB is one for requests to NAV computer
		// zero for requests to the CTRL computer
		if (SERCOM2->USART.INTFLAG.bit.RXC) {
			uint8_t command = SERIAL_REG;
			
			while(SERCOM2->USART.INTFLAG.bit.RXC) SERCOM2->USART.DATA.reg;
			
			
			EEPROM_Read_Request eeprom_read_request;
			CTRL_EEPROM_Read_packet ctrl_eeprom_read_packet;
			EEPROM_Write_Request eeprom_write_request;
			
			switch (command) {
					// eeprom read
				case 0x40:
					// create data type
					// set acknowledge response to ok
					ctrl_ack_packet.bit.status_code = CTRL_ACK_OK;
					// set acknowledge packet crc
					ctrl_ack_packet.bit.crc = gen_crc32((uint32_t) ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg) - 4);
					// send acknowledge packet
					serial_stream(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg));
					// wait for data request
					serial_read(eeprom_read_request.reg, sizeof(eeprom_read_request.reg));
					
					// check packet is valid
					if (gen_crc32((uint32_t) eeprom_read_request.reg, sizeof(eeprom_read_request.reg)) == CRC32_CHECK &&
						eeprom_read_request.bit.header == EEPROM_READ_REQUEST_HEADER) {
						// get data from eeprom
						ctrl_eeprom_read_packet.bit.device_id = DEVICE_ID;
												
						ctrl_eeprom_read_packet.bit.data = spi_eeprom_read_byte(eeprom_read_request.bit.address);
							
						ctrl_eeprom_read_packet.bit.status = 1;
						ctrl_eeprom_read_packet.bit.crc = gen_crc32((uint32_t) ctrl_eeprom_read_packet.reg, sizeof(ctrl_eeprom_read_packet.reg));
							
						serial_stream(ctrl_eeprom_read_packet.reg, sizeof(ctrl_eeprom_read_packet.reg));
					}
					else {
						REG_PORT_OUTSET1 = LED;
						while(1);
					}

					break;
				
					// eeprom write	
				case 0x41:
					// set acknowledge response to ok
					ctrl_ack_packet.bit.status_code = CTRL_ACK_OK;
					// set acknowledge packet crc
					ctrl_ack_packet.bit.crc = gen_crc32((uint32_t) ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg) - 4);
					// send acknowledge packet
					serial_stream(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg));
					// wait for data request
					serial_read(eeprom_write_request.reg, sizeof(eeprom_write_request.reg));
					
					// check packet is valid
					if (gen_crc32((uint32_t) eeprom_write_request.reg, sizeof(eeprom_write_request.reg)) == CRC32_CHECK &&
						eeprom_write_request.bit.header == EEPROM_WRITE_REQUEST_HEADER) {
						// write data to eeprom
						spi_eeprom_write_enable();
						spi_eeprom_write_byte(eeprom_write_request.bit.address, eeprom_write_request.bit.data);
						spi_eeprom_write_disable();
					}
					else {
						REG_PORT_OUTSET1 = LED;
						while(1);
					}
					break;
					
					// flash read
				case 0x42:
					break;
					
					// flash write
				case 0x43:
					break;
				
					// case for nav self test
				case 0x80:
					// send command for self test
					nav_uart_send(0x80);
				
					for (uint32_t i = 0; i < sizeof(nav_selftest_packet.reg); ++i) {
						// wait until data available
						while(!SERCOM0->USART.INTFLAG.bit.RXC);
							
						nav_selftest_packet.reg[i] = (uint8_t) (SERCOM0->USART.DATA.reg);
					}
						
					if (gen_crc32((uint32_t) &nav_selftest_packet.reg[0], sizeof(nav_selftest_packet.reg)) != CRC32_CHECK) {
						//serial_print("CRC Check Failed\n");
						REG_PORT_OUTSET1 = LED;
					}
				
					serial_stream(nav_selftest_packet.reg, sizeof(nav_selftest_packet.reg));
					break;
			
					// command to send nav_data_packet
				case 0x81:
					serial_stream(nav_data_packet.reg, sizeof(nav_data_packet.reg));
					break;
					
				default:
					//delay_ms(1);
					serial_send(command);
					REG_PORT_OUTSET1 = (1 << 11);
					break;
			}
		}
		
		
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
	
	if (gen_crc32((uint32_t) &nav_selftest_packet.reg[0], sizeof(nav_selftest_packet.reg)) == CRC32_CHECK) {
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
	spi_eeprom_init();
	
	// set LED to output
	REG_PORT_DIRSET1 = LED;
	
	// set SS pins high
	REG_PORT_DIRSET0 = PORT_PA25 | PORT_PA27 | PORT_PA28;
	REG_PORT_OUTSET0 = PORT_PA25 | PORT_PA27 | PORT_PA28;
	
	ctrl_ack_packet.bit.device_id = DEVICE_ID;
	
	
	//serial_print("Embedded Navigation v0.0.2\nFinley Blaine 2021\n\n");
	
	delay_ms(1000);
	//system_check();
	
}
