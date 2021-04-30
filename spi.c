#include "spi.h"


uint8_t spi_command(uint8_t opcode) {
	// check data register is ready
	while(SERCOM3->SPI.INTFLAG.bit.DRE == 0);
	
	// send data
	SERCOM3->SPI.DATA.reg = opcode;
	
	// wait until operation complete
	while(SERCOM3->SPI.INTFLAG.bit.TXC == 0);
	
	// read buffer
	return SERCOM3->SPI.DATA.reg;
}


void spi_init() {
	// SCLK = PA23
	// MOSI = PA22
	// MISO = PA24
	// SS = PA25
	
	// sercom3
	
	
	// provide bus clock to SERCOM3
	
	PM->APBCMASK.bit.SERCOM3_ = 1;
	
	
	
	
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID_SERCOM3_CORE;
	
	
	// configure CTRLA register
	
	//SERCOM3->SPI.CTRLA.bit.ENABLE = 0;
	
	while(GCLK->STATUS.bit.SYNCBUSY);
	
	const SERCOM_SPI_CTRLA_Type ctrla = {
		// MSB first
		.bit.DORD = 0,
		
		// set SPI mode 0
		.bit.CPHA = 0,
		.bit.CPOL = 0,
		
		// set frame to 'SPI frame'
		.bit.FORM = 0,
		
		// set mode to master
		.bit.MODE = 3,
		
		// set input to pad 2
		.bit.DIPO = 2,
		
		// set outputs DO to pad 0, SCK to pad 1
		.bit.DOPO = 0
	};
	
	SERCOM3->SPI.CTRLA.reg = ctrla.reg;
	
	
	// configure CTRLB register
	
	const SERCOM_SPI_CTRLB_Type ctrlb = {
		// enable receiver
		.bit.RXEN = 1,
		
		// set character size to 8 bits
		.bit.CHSIZE = 0,
		
		// set SS to software
		.bit.MSSEN = 0
	};
	
	SERCOM3->SPI.CTRLB.reg = ctrlb.reg;
	
	
	

	SERCOM3->SPI.BAUD.reg = 5; // 2
	
	
	
	// set pin multiplexing options
	
	const PORT_WRCONFIG_Type wrconfig = {
		// upper 16 pins
		.bit.HWSEL = 1,
		
		// enable pin configuration update
		.bit.WRPINCFG = 1,
		
		// enable configuration of pin multiplexing
		.bit.WRPMUX = 1,
		
		// define pin multiplexing mode (C)
		.bit.PMUX = 2,
		
		// enable pin multiplexing
		.bit.PMUXEN = 1,
		
		// apply to pins
		.bit.PINMASK = (uint16_t)((PORT_PA23 | PORT_PA22 | PORT_PA24) >> 16)
	};
	
	
	PORT->Group[0].WRCONFIG.reg = wrconfig.reg;


	// enable SERCOM SPI
	SERCOM3->SPI.CTRLA.bit.ENABLE = 1;
	
	
	// wait for synchronisation
	while(SERCOM3->SPI.SYNCBUSY.bit.ENABLE);
	
	//// set IMU handshake pin to input
	//REG_PORT_DIRCLR0 = (1 << 14);
	//PORT->Group[0].PINCFG[14].bit.INEN = 1;
}