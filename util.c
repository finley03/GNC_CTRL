#include "util.h"


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