#include "main.h"

#include "spi_flash.h"
#include "spi_eeprom.h"
#include "uart.h"
#include "spi.h"
#include "dma.h"
#include "pwm.h"
#include "control.h"
#include "guidance.h"


void init();
void txc_nav_data();
//void txc_serial_data();
void txc_wireless_data();


Transfer_Request transfer_request;
//Transfer_Request wireless_transfer_request;
NAV_Data_Packet nav_data_packet;
NAV_Selftest_Packet nav_selftest_packet;
CTRL_ACK_Packet ctrl_ack_packet;

bool nav_poll;
bool guidance_run;
bool arm;

#ifdef TEST
static float position[3] = {0, -2, -2};
#endif


int main(void) {
	init();

	while(1) {
		delay_ms(20);
		static uint32_t previous_time = 0;
		// get current time
		uint32_t current_time = read_timer_20ns();
		// calculate time difference
		uint32_t delta_time = current_time - previous_time;
		// reset previous time
		previous_time = current_time;
		// convert previous time to float
		float i_time = delta_time * TIMER_S_MULTIPLIER;
		
		if (nav_poll) {
			LED_ON();
			txc_nav_data();
			LED_OFF();
		}
		
		// get pwm values
		PWM_in pwm_in = pwm_read();
		// check if override is set
		if (PWM_BOOL(pwm_in.ovr)) pwm_write_all(pwm_in);
		// run guidance routine
		else {
			static float target_orientation[3] = {0, 0, 0};
			
			if (guidance_run) {
				//#ifdef TEST
				//static float position[3] = {0, -2, -2};
				#ifndef TEST
				float position[3] = {nav_data_packet.bit.position_x, nav_data_packet.bit.position_y, nav_data_packet.bit.position_z};
				#endif
				
				float target_vector[3];
				guidance(position, target_orientation, target_vector);
				mat_scalar_product(target_vector, i_time, 3, target_vector);
				
				#ifdef TEST
				mat_add(position, target_vector, 3, position);
				#endif
			}
			
			float measured_orientation[3] = {nav_data_packet.bit.orientation_x, nav_data_packet.bit.orientation_y, nav_data_packet.bit.orientation_z};
			control(target_orientation, measured_orientation);
		
			#ifdef TEST
			nav_data_packet.bit.position_x = position[0];
			nav_data_packet.bit.position_y = position[1];
			nav_data_packet.bit.position_z = position[2];
			#endif
		}
		
		txc_wireless_data();
	}
	
	return 0;
}


void txc_nav_data() {
	nav_uart_send(0x81);
		
	nav_read_timed(nav_data_packet.reg, sizeof(nav_data_packet.reg));
	
	// assume internal communications are okay	
	
	//if (crc32(nav_data_packet.reg, sizeof(nav_data_packet.reg)) != CRC32_CHECK) {
		////serial_print("CRC Check Failed\n");
		//REG_PORT_OUTSET1 = LED;
		//while(1);
	//}
}


void txc_wireless_data() {
	// check for data request from computer
	// MSB is one for requests to NAV computer
	// zero for requests to the CTRL computer
	if (wireless_rx_dma_end()) {
		// true if ack_ok packet should be sent at the end.
		bool end_ack_packet = true;
		//REG_PORT_OUTSET1 = LED;
		uint16_t command;
		if (crc32(transfer_request.reg, sizeof(transfer_request.reg)) == CRC32_CHECK &&
		transfer_request.bit.header == TRANSFER_REQUEST_HEADER) {
			command = transfer_request.bit.command;
		}
		else  {
			//LED_ON();
			//while(1);
			// flush buffer
			wireless_flush();
			// no ack
			//end_ack_packet = false;
			// reset rx buffer
			//goto restart_dma;
			wireless_rx_dma_start();
			// report bad packet
			ack_error();
			return;
		}
		
		wireless_flush();
		
		
		switch (command) {
			// CTRL COMPUTER
			
			// enable nav computer polling
			case 0x0000:
			nav_poll = true;
			break;
			
			// disable nav computer polling
			case 0x0001:
			nav_poll = false;
			break;
			
			// reset guidance
			case 0x0002:
			reset_guidance();
			#ifdef TEST
			position[0] = 0;
			position[1] = -2;
			position[2] = -2;
			#endif
			break;
			
			// start guidance
			case 0x0003:
			guidance_run = true;
			break;
			
			// stop guidance;
			case 0x0004:
			guidance_run = false;
			break;
			
			// motor arm
			case 0x0005:
			arm = true;
			break;
			
			// motor disarm
			case 0x0006:
			arm = false;
			break;
						
			// eeprom read byte
			case 0x0040:
			{
				// create data type
				EEPROM_Read_Request eeprom_read_request;
				ack_ok();
				//// set acknowledge response to ok
				//ctrl_ack_packet.bit.status_code = CTRL_ACK_OK;
				//// set acknowledge packet crc
				//ctrl_ack_packet.bit.crc = crc32(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg) - 4);
				//// send acknowledge packet
				//wireless_stream(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg));
				// wait for data request
				if (!wireless_read(eeprom_read_request.reg, sizeof(eeprom_read_request.reg))) {
					wireless_flush_restart();
					return;
				}
				
				// check packet is valid
				if (crc32(eeprom_read_request.reg, sizeof(eeprom_read_request.reg)) == CRC32_CHECK &&
				eeprom_read_request.bit.header == EEPROM_READ_REQUEST_HEADER) {
					ack_ok();
					
					CTRL_EEPROM_Read_packet ctrl_eeprom_read_packet;
					// get data from eeprom
					ctrl_eeprom_read_packet.bit.device_id = DEVICE_ID;
					
					ctrl_eeprom_read_packet.bit.data = spi_eeprom_read_byte(eeprom_read_request.bit.address);
					
					ctrl_eeprom_read_packet.bit.status = 1;
					ctrl_eeprom_read_packet.bit.crc = crc32(ctrl_eeprom_read_packet.reg, sizeof(ctrl_eeprom_read_packet.reg) - 4);
					
					wireless_stream(ctrl_eeprom_read_packet.reg, sizeof(ctrl_eeprom_read_packet.reg));
				}
				else {
					//LED_ON();
					//while(1);
					//ack_error();
					//wireless_flush();
					//wireless_rx_dma_start();
					ack_error_flush_restart();
					return;
				}
				end_ack_packet = false;
				wireless_rx_dma_start();
			}
			break;
			
			// eeprom write byte
			case 0x0041:
			{
				// create data type
				EEPROM_Write_Request eeprom_write_request;
				ack_ok();
				//// set acknowledge response to ok
				//ctrl_ack_packet.bit.status_code = CTRL_ACK_OK;
				//// set acknowledge packet crc
				//ctrl_ack_packet.bit.crc = crc32(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg) - 4);
				//// send acknowledge packet
				//wireless_stream(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg));
				// wait for data request
				if (!wireless_read(eeprom_write_request.reg, sizeof(eeprom_write_request.reg))) {
					wireless_flush_restart();
					return;
				}
				
				// check packet is valid
				if (crc32(eeprom_write_request.reg, sizeof(eeprom_write_request.reg)) == CRC32_CHECK &&
				eeprom_write_request.bit.header == EEPROM_WRITE_REQUEST_HEADER) {
					// write data to eeprom
					spi_eeprom_write_enable();
					spi_eeprom_write_byte(eeprom_write_request.bit.address, eeprom_write_request.bit.data);
					spi_eeprom_write_disable();
					
					ack_ok();
				}
				else {
					//LED_ON();
					//while(1);
					//ack_error();
					//wireless_flush();
					//wireless_rx_dma_start();
					ack_error_flush_restart();
					return;
				}
				end_ack_packet = false;
				wireless_rx_dma_start();
			}
			break;
			
			// eeprom read n up to 64 bytes
			case 0x0042:
			{
				// create data type
				EEPROM_Read_N_Request eeprom_read_request;
				ack_ok();
				//// send response ok
				//ctrl_ack_packet.bit.status_code = CTRL_ACK_OK;
				//ctrl_ack_packet.bit.crc = crc32(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg) - 4);
				//// send acknowledge packet
				//wireless_stream(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg));
				// wait for data request
				if (!wireless_read(eeprom_read_request.reg, sizeof(eeprom_read_request.reg))) {
					wireless_flush_restart();
					return;
				}
				
				// check packet is valid
				if (crc32(eeprom_read_request.reg, sizeof(eeprom_read_request.reg)) == CRC32_CHECK &&
				eeprom_read_request.bit.header == EEPROM_READ_N_REQUEST_HEADER) {
					ack_ok();
					
					CTRL_EEPROM_Read_N_packet ctrl_eeprom_read_packet;
					// get data from eeprom
					ctrl_eeprom_read_packet.bit.device_id = DEVICE_ID;
					
					uint8_t data[64];
					spi_eeprom_read_n(eeprom_read_request.bit.address, data, eeprom_read_request.bit.size);
					//ctrl_eeprom_read_packet.bit.data = spi_eeprom_read_byte(eeprom_read_request.bit.address);
					
					ctrl_eeprom_read_packet.bit.status = 1;
					ctrl_eeprom_read_packet.bit.crc = crc32(ctrl_eeprom_read_packet.reg, sizeof(ctrl_eeprom_read_packet.reg) - 4);
					
					wireless_stream(ctrl_eeprom_read_packet.reg, sizeof(ctrl_eeprom_read_packet.reg));
				}
				else {
					//LED_ON();
					//while(1);
					//ack_error();
					//wireless_flush();
					//wireless_rx_dma_start();
					ack_error_flush_restart();
					return;
				}
				end_ack_packet = false;
				wireless_rx_dma_start();
			}
			break;
			
			// eeprom write n up to 64 bytes
			case 0x0043:
			{
				// create data type
				EEPROM_Write_N_Request eeprom_write_request;
				ack_ok();
				//// set acknowledge response to ok
				//ctrl_ack_packet.bit.status_code = CTRL_ACK_OK;
				//ctrl_ack_packet.bit.crc = crc32(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg) - 4);
				//// send acknowledge packet
				//wireless_stream(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg));
				// wait for data request
				if (!wireless_read(eeprom_write_request.reg, sizeof(eeprom_write_request.reg))) {
					wireless_flush_restart();
					return;
				}
				
				// check packet is valid
				if (crc32(eeprom_write_request.reg, sizeof(eeprom_write_request.reg)) == CRC32_CHECK &&
				eeprom_write_request.bit.header == EEPROM_WRITE_N_REQUEST_HEADER) {
					// write data to eeprom
					//spi_eeprom_write_enable();
					//spi_eeprom_write_byte(eeprom_write_request.bit.address, eeprom_write_request.bit.data);
					spi_eeprom_write_n_s(eeprom_write_request.bit.address, eeprom_write_request.bit.data, eeprom_write_request.bit.size);
					//spi_eeprom_write_disable();
					ack_ok();
				}
				else {
					//LED_ON();
					//while(1);
					//ack_error();
					//wireless_flush();
					//wireless_rx_dma_start();
					ack_error_flush_restart();
					return;
				}
				end_ack_packet = false;
				wireless_rx_dma_start();
			}
			break;
			
			// set vec3 parameter
			case 0x0044:
			{
				// create data type
				Set_Vec3_Request set_request;
				ack_ok();
				//// set acknowledge response to ok
				//ctrl_ack_packet.bit.status_code = CTRL_ACK_OK;
				//ctrl_ack_packet.bit.crc = crc32(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg) - 4);
				//// send acknowledge packet
				//wireless_stream(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg));
				// wait for data request
				if (!wireless_read(set_request.reg, sizeof(set_request.reg))) {
					wireless_flush_restart();
					return;
				}
				
				// check packet is valid
				if (crc32(set_request.reg, sizeof(set_request.reg)) == CRC32_CHECK &&
					set_request.bit.header == SET_VEC3_REQUEST_HEADER) {
					if (set_request.bit.parameter > _NAV_PARAM_START) {
						nav_set_vec3((CTRL_Param) set_request.bit.parameter, set_request.bit.data);
					}
					else {
						control_set_value((CTRL_Param) set_request.bit.parameter, set_request.bit.data);
					}
					
					ack_ok();
				}
				else {
					//LED_ON();
					//while(1);
					//ack_error();
					//wireless_flush();
					//wireless_rx_dma_start();
					ack_error_flush_restart();
					return;
				}
				end_ack_packet = false;
				wireless_rx_dma_start();
			}
			break;
			
			// read vec3 parameter
			case 0x0045:
			{
				Read_Vec3_Request read_request;
				ack_ok();
				//// return ack packet
				//ctrl_ack_packet.bit.status_code = CTRL_ACK_OK;
				//ctrl_ack_packet.bit.crc = crc32(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg) - 4);
				//wireless_stream(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg));
				// wait for request
				if (!wireless_read(read_request.reg, sizeof(read_request.reg))) {
					wireless_flush_restart();
					return;
				}
				// check request is valid
				if (crc32(read_request.reg, sizeof(read_request.reg)) == CRC32_CHECK &&
				read_request.bit.header == READ_VEC3_REQUEST_HEADER) {
					ack_ok();
					Read_Vec3_Response read_packet;
					read_packet.bit.device_id = DEVICE_ID;
					if (read_request.bit.parameter > _NAV_PARAM_START) {
						nav_read_vec3((CTRL_Param) read_request.bit.parameter, read_packet.bit.data);
					}
					else {
						control_read_value((CTRL_Param) read_request.bit.parameter, read_packet.bit.data);
					}
					read_packet.bit.crc = crc32(read_packet.reg, sizeof(read_packet.reg) - 4);
					wireless_stream(read_packet.reg, sizeof(read_packet.reg));
				}
				else {
					//LED_ON();
					//while(1);
					//ack_error();
					//wireless_flush();
					//wireless_rx_dma_start();
					ack_error_flush_restart();
					return;
				}
				end_ack_packet = false;
				wireless_rx_dma_start();
			}
			break;
			
			// save vec3 parameter
			case 0x0046:
			{
				// create data type
				Save_Vec3_Request save_request;
				ack_ok();
				//// set acknowledge response to ok
				//ctrl_ack_packet.bit.status_code = CTRL_ACK_OK;
				//ctrl_ack_packet.bit.crc = crc32(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg) - 4);
				//// send acknowledge packet
				//wireless_stream(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg));
				// wait for data request
				if (!wireless_read(save_request.reg, sizeof(save_request.reg))) {
					wireless_flush_restart();
					return;
				}
				
				// check packet is valid
				if (crc32(save_request.reg, sizeof(save_request.reg)) == CRC32_CHECK &&
				save_request.bit.header == SAVE_VEC3_REQUEST_HEADER) {
					//control_save_value((CTRL_Param) save_request.bit.parameter);
					if (save_request.bit.parameter > _NAV_PARAM_START) {
						float value[3];
						nav_read_vec3((CTRL_Param) save_request.bit.parameter, value);
						control_write_value((CTRL_Param) save_request.bit.parameter, value);
					}
					else {
						//control_read_value((CTRL_Param) read_request.bit.parameter, &read_packet.bit.data);
						control_save_value((CTRL_Param) save_request.bit.parameter);
					}
					
					ack_ok();
				}
				else {
					//LED_ON();
					//while(1);
					//ack_error();
					//wireless_flush();
					//wireless_rx_dma_start();
					ack_error_flush_restart();
					return;
				}
				end_ack_packet = false;
				wireless_rx_dma_start();
			}
			break;
			
			// set scalar parameter
			case 0x0047:
			{
				Set_Scalar_Request set_request;
				ack_ok();
				//// return acknowledge packet
				//ctrl_ack_packet.bit.status_code = CTRL_ACK_OK;
				//ctrl_ack_packet.bit.crc = crc32(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg) - 4);
				//wireless_stream(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg));
				// wait for data request
				if (!wireless_read(set_request.reg, sizeof(set_request.reg))) {
					wireless_flush_restart();
					return;
				}
				// check request is valid
				if (crc32(set_request.reg, sizeof(set_request.reg)) == CRC32_CHECK &&
				set_request.bit.header == SET_SCALAR_REQUEST_HEADER) {
					if (set_request.bit.parameter > _NAV_PARAM_START) {
						nav_set_scalar((CTRL_Param) set_request.bit.parameter, &set_request.bit.data);
					}
					else {
						control_set_value((CTRL_Param) set_request.bit.parameter, &set_request.bit.data);
					}
					
					ack_ok();
				}
				else {
					//LED_ON();
					//while(1);
					//ack_error();
					//wireless_flush();
					//wireless_rx_dma_start();
					ack_error_flush_restart();
					return;
				}
				end_ack_packet = false;
				wireless_rx_dma_start();
			}
			break;
			
			// read scalar parameter
			case 0x0048:
			{
				Read_Scalar_Request read_request;
				ack_ok();
				//// return ack packet
				//ctrl_ack_packet.bit.status_code = CTRL_ACK_OK;
				//ctrl_ack_packet.bit.crc = crc32(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg) - 4);
				//wireless_stream(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg));
				// wait for request
				if (!wireless_read(read_request.reg, sizeof(read_request.reg))) {
					wireless_flush_restart();
					return;
				}
				// check request is valid
				if (crc32(read_request.reg, sizeof(read_request.reg)) == CRC32_CHECK &&
				read_request.bit.header == READ_SCALAR_REQUEST_HEADER) {
					// send okay ack packet
					ack_ok();
					//Read_Scalar_Response read_packet;
					//read_packet.bit.device_id = DEVICE_ID;
					//control_read_value((CTRL_Param) read_request.bit.parameter, &read_packet.bit.data);
					//read_packet.bit.crc = crc32(read_packet.reg, sizeof(read_packet.reg) - 4);
					//wireless_stream(read_packet.reg, sizeof(read_packet.reg));
					Read_Scalar_Response read_packet;
					read_packet.bit.device_id = DEVICE_ID;
					if (read_request.bit.parameter > _NAV_PARAM_START) {
						nav_read_scalar((CTRL_Param) read_request.bit.parameter, &read_packet.bit.data);
					}
					else {
						control_read_value((CTRL_Param) read_request.bit.parameter, &read_packet.bit.data);
					}
					read_packet.bit.crc = crc32(read_packet.reg, sizeof(read_packet.reg) - 4);
					wireless_stream(read_packet.reg, sizeof(read_packet.reg));
				}
				else {
					//LED_ON();
					//while(1);
					//ack_error();
					//wireless_flush();
					//wireless_rx_dma_start();
					ack_error_flush_restart();
					return;
				}
				end_ack_packet = false;
				wireless_rx_dma_start();
			}
			break;
			
			// save scalar parameter
			case 0x0049:
			{
				// create data type
				Save_Scalar_Request save_request;
				ack_ok();
				//// set acknowledge response to ok
				//ctrl_ack_packet.bit.status_code = CTRL_ACK_OK;
				//ctrl_ack_packet.bit.crc = crc32(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg) - 4);
				//// send acknowledge packet
				//wireless_stream(ctrl_ack_packet.reg, sizeof(ctrl_ack_packet.reg));
				// wait for data request
				if (!wireless_read(save_request.reg, sizeof(save_request.reg))) {
					wireless_flush_restart();
					return;
				}
								
				// check packet is valid
				if (crc32(save_request.reg, sizeof(save_request.reg)) == CRC32_CHECK &&
				save_request.bit.header == SAVE_SCALAR_REQUEST_HEADER) {
					////control_save_value((CTRL_Param) save_request.bit.parameter);
					if (save_request.bit.parameter > _NAV_PARAM_START) {
						float value;
						nav_read_scalar((CTRL_Param) save_request.bit.parameter, &value);
						control_write_value((CTRL_Param) save_request.bit.parameter, &value);
					}
					else {
						//control_read_value((CTRL_Param) read_request.bit.parameter, &read_packet.bit.data);
						control_save_value((CTRL_Param) save_request.bit.parameter);
					}
					
					ack_ok();
				}
				else {
					//LED_ON();
					//while(1);
					//ack_error();
					//wireless_flush();
					//wireless_rx_dma_start();
					ack_error_flush_restart();
					return;
				}
				end_ack_packet = false;
				wireless_rx_dma_start();
			}
			break;
			
			// system reset
			case 0x007F:
			end_ack_packet = false;
			NVIC_SystemReset();
			break;
			
			
			
			// NAV COMPUTER
			
			// case for nav self test
			case 0x0080:
			{
				ack_ok();
				// send command for self test
				nav_uart_send(0x80);
				
				nav_read(nav_selftest_packet.reg, sizeof(nav_selftest_packet.reg));
				
				if (crc32(nav_selftest_packet.reg, sizeof(nav_selftest_packet.reg)) != CRC32_CHECK) {
					//serial_print("CRC Check Failed\n");
					LED_ON();
					while(1);
				}
				
				wireless_stream(nav_selftest_packet.reg, sizeof(nav_selftest_packet.reg));
				end_ack_packet = false;
				wireless_rx_dma_start();
			}
			break;
			
			// command to send nav_data_packet
			case 0x0081:
			{
				ack_ok();
				wireless_stream(nav_data_packet.reg, sizeof(nav_data_packet.reg));
				end_ack_packet = false;
				wireless_rx_dma_start();
			}
			break;
			
			// calibrate magenetometer
			case 0x0082:
			nav_poll = false;
			nav_uart_send(0x86);
			break;
			
			// enable kalman filter
			case 0x0083:
			nav_uart_send(0x87);
			break;
			
			// disable kalman filter
			case 0x0084:
			nav_uart_send(0x88);
			break;
			
			// save magnetometer calibration
			case 0x0085:
			nav_save_vec3(_MAG_A_1);
			nav_save_vec3(_MAG_A_2);
			nav_save_vec3(_MAG_A_3);
			nav_save_vec3(_MAG_B);
			break;
			
			// enable kalman orientation update step
			case 0x0086:
			nav_uart_send(0x89);
			break;
			
			// disable kalman orientation update step
			case 0x0087:
			nav_uart_send(0x8A);
			break;
			
			// nav computer reset
			case 0x00FF:
			nav_uart_send(0xFF);
			//end_ack_packet = false;
			//wireless_rx_dma_start();
			break;
			
			default:
			{
				//delay_ms(1);
				serial_send(command);
				LED_ON();
				while(1);
			}
			break;
		}
		
//restart_dma:		
		
		//// restart DMA
		//wireless_rx_dma_start();
		////REG_PORT_OUTCLR1 = LED;
		
		if (end_ack_packet) {
			// restart dma
			wireless_rx_dma_start();
			ack_ok();
		}
	}
}


void init() {
	set_clock_48m();
	init_timer();
	start_timer();
	crc_init();
	//delay_ms(100);
	pwm_init_out();
	serial_init();
	wireless_init();
	nav_uart_init();
	spi_init();
	spi_eeprom_init();
	
	// set LED to output
	REG_PORT_DIRSET1 = LED;
	//LED_ON();
	
	// set SS pins high
	REG_PORT_DIRSET0 = PORT_PA25 | PORT_PA27 | PORT_PA28;
	REG_PORT_OUTSET0 = PORT_PA25 | PORT_PA27 | PORT_PA28;
	
	ctrl_ack_packet.bit.device_id = DEVICE_ID;
	
	delay_ms(200);
	
	wireless_rx_init_dma();
	pwm_init_in();
	control_load_values();
	reset_guidance();
	
	nav_poll = true;
	guidance_run = false;
	
	ack_ok();
}
