#include "main.h"

#include "spi_flash.h"
#include "spi_eeprom.h"
#include "uart.h"
#include "spi.h"
#include "dma.h"
#include "pwm.h"


void init();


static NAV_Data_Packet nav_data_packet;
static NAV_Selftest_Packet nav_selftest_packet;
volatile Transfer_Request transfer_request;
static CTRL_ACK_Packet ctrl_ack_packet;


int main(void) {
	
	init();

	
	while(1) {
		delay_ms(20);
		
		REG_PORT_OUTSET1 = LED;
		nav_uart_send(0x81);
		
		nav_read(nav_data_packet.reg, sizeof(nav_data_packet.reg));
		//for (uint32_t i = 0; i < sizeof(nav_data_packet.reg); ++i) {
			//// wait until data available
			//while(!SERCOM0->USART.INTFLAG.bit.RXC);
				//
			//nav_data_packet.reg[i] = (uint8_t) (SERCOM0->USART.DATA.reg);
		//}
		
		if (crc32(nav_data_packet.reg, sizeof(nav_data_packet.reg)) != CRC32_CHECK) {
			//serial_print("CRC Check Failed\n");
			REG_PORT_OUTSET1 = LED;
			while(1);
		}
		REG_PORT_OUTCLR1 = LED;
		
		
		pwm_write(PWM_WRITE_ALE, nav_data_packet.bit.orientation_x / 90);
		pwm_write(PWM_WRITE_ELEV, nav_data_packet.bit.orientation_y / 90);
		pwm_write(PWM_WRITE_RUDD, nav_data_packet.bit.orientation_z / 90);

		
		
		// check for data request from computer
		// MSB is one for requests to NAV computer
		// zero for requests to the CTRL computer
		//if (SERIAL_RX_FLAG) {
			//uint8_t command = SERIAL_REG;
		if (serial_rx_dma_end()) {
			uint16_t command;
			if (crc32(transfer_request.reg, sizeof(transfer_request.reg)) == CRC32_CHECK &&
				transfer_request.bit.header == TRANSFER_REQUEST_HEADER) {
				command = transfer_request.bit.command;
			}
			else  {
				REG_PORT_OUTSET1 = LED;
				while(1);
			}
			
			serial_flush();
			
			
			static EEPROM_Read_Request eeprom_read_request;
			static CTRL_EEPROM_Read_packet ctrl_eeprom_read_packet;
			static EEPROM_Write_Request eeprom_write_request;
			
			switch (command) {
					// eeprom read
				case 0x0040:
					// create data type
					// set acknowledge response to ok
					ctrl_ack_packet.bit.status_code = CTRL_ACK_OK;
					// set acknowledge packet crc
					ctrl_ack_packet.bit.crc = crc32(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg) - 4);
					// send acknowledge packet
					serial_stream(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg));
					// wait for data request
					serial_read(eeprom_read_request.reg, sizeof(eeprom_read_request.reg));
					
					// check packet is valid
					if (crc32(eeprom_read_request.reg, sizeof(eeprom_read_request.reg)) == CRC32_CHECK &&
						eeprom_read_request.bit.header == EEPROM_READ_REQUEST_HEADER) {
						// get data from eeprom
						ctrl_eeprom_read_packet.bit.device_id = DEVICE_ID;
												
						ctrl_eeprom_read_packet.bit.data = spi_eeprom_read_byte(eeprom_read_request.bit.address);
							
						ctrl_eeprom_read_packet.bit.status = 1;
						ctrl_eeprom_read_packet.bit.crc = crc32(ctrl_eeprom_read_packet.reg, sizeof(ctrl_eeprom_read_packet.reg));
							
						serial_stream(ctrl_eeprom_read_packet.reg, sizeof(ctrl_eeprom_read_packet.reg));
					}
					else {
						REG_PORT_OUTSET1 = LED;
						while(1);
					}

					break;
				
					// eeprom write	
				case 0x0041:
					// set acknowledge response to ok
					ctrl_ack_packet.bit.status_code = CTRL_ACK_OK;
					// set acknowledge packet crc
					ctrl_ack_packet.bit.crc = crc32(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg) - 4);
					// send acknowledge packet
					serial_stream(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg));
					// wait for data request
					serial_read(eeprom_write_request.reg, sizeof(eeprom_write_request.reg));
					
					// check packet is valid
					if (crc32(eeprom_write_request.reg, sizeof(eeprom_write_request.reg)) == CRC32_CHECK &&
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
				case 0x0042:
					break;
					
					// flash write
				case 0x0043:
					break;
				
					// case for nav self test
				case 0x0080:
					// send command for self test
					nav_uart_send(0x80);
				
					nav_read(nav_selftest_packet.reg, sizeof(nav_selftest_packet.reg));
					//for (uint32_t i = 0; i < sizeof(nav_selftest_packet.reg); ++i) {
						//// wait until data available
						//while(!SERCOM0->USART.INTFLAG.bit.RXC);
							//
						//nav_selftest_packet.reg[i] = (uint8_t) (SERCOM0->USART.DATA.reg);
					//}
						
					if (crc32(nav_selftest_packet.reg, sizeof(nav_selftest_packet.reg)) != CRC32_CHECK) {
						//serial_print("CRC Check Failed\n");
						REG_PORT_OUTSET1 = LED;
						while(1);
					}
				
					serial_stream(nav_selftest_packet.reg, sizeof(nav_selftest_packet.reg));
					break;
			
					// command to send nav_data_packet
				case 0x0081:
					serial_stream(nav_data_packet.reg, sizeof(nav_data_packet.reg));
					break;
					
				default:
					//delay_ms(1);
					serial_send(command);
					REG_PORT_OUTSET1 = LED;
					while(1);
					break;
			}
			
			// restart DMA
			serial_rx_dma_start();
		}
		
		
	}
	
	
	return 0;
}


void init() {
	set_clock_48m();
	crc_init();
	//delay_ms(100);
	pwm_init_out();
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
	
	delay_ms(1000);
	
	serial_rx_init_dma();
}
