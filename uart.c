#include "uart.h"


void serial_send(uint8_t data) {
	while(!SERCOM2->USART.INTFLAG.bit.DRE);
	
	SERCOM2->USART.DATA.reg = data;
}


void serial_print(char *addr) {
	while(*addr != 0) {
		serial_send(*addr);
		++addr;
	}
}


void serial_init() {
	// uses SERCOM2 (PA08 - PA11)
	
	// provide bus clock to SERCOM2
	PM->APBCMASK.bit.SERCOM2_ = 1;
	
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID_SERCOM2_CORE;
	
	while(GCLK->STATUS.bit.SYNCBUSY);
	
	
	SERCOM_USART_CTRLA_Type ctrla = {
		// set to LSB first
		.bit.DORD = 1,
		
		// set to asynchronous communication
		.bit.CMODE = 0,
		
		// set no parity
		.bit.FORM = 0,
		
		// set SERCOM PAD[3] to rx
		.bit.RXPO = 3,
		
		// set PAD[2] to tx
		.bit.TXPO = 1,
		
		// set internal clock
		.bit.MODE = 1
	};
	
	SERCOM2->USART.CTRLA.reg = ctrla.reg;
	
	
	SERCOM_USART_CTRLB_Type ctrlb = {
		// enable tx and rx
		.bit.RXEN = 1,
		.bit.TXEN = 1
	};
	
	SERCOM2->USART.CTRLB.reg = ctrlb.reg;
	
	while(SERCOM2->USART.SYNCBUSY.bit.CTRLB);
	
	
	// set baud to 3000000
	SERCOM2->USART.BAUD.reg = 0; // 45403
	
	
	PORT_WRCONFIG_Type wrconfig = {
		// lower 16 bits
		.bit.HWSEL = 0,
		
		// enable update
		.bit.WRPINCFG = 1,
		
		.bit.WRPMUX = 1,
		
		// peripheral multiplexing function D
		.bit.PMUX = 3,
		
		// enable pin multiplexing
		.bit.PMUXEN = 1,
		
		// select pins
		.bit.PINMASK = (uint16_t)((PORT_PA10 | PORT_PA11))
		
	};
	
	PORT->Group[0].WRCONFIG.reg = wrconfig.reg;
	
	
	// enable USART
	SERCOM2->USART.CTRLA.bit.ENABLE = 1;
	
	// wait for synchronisation
	while(SERCOM2->USART.SYNCBUSY.bit.ENABLE);
}


void nav_uart_init() {
	// uses SERCOM0 (PA08 - PA11)
	
	// provide bus clock to SERCOM0
	PM->APBCMASK.bit.SERCOM0_ = 1;
	
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID_SERCOM0_CORE;
	
	while(GCLK->STATUS.bit.SYNCBUSY);
	
	
	SERCOM_USART_CTRLA_Type ctrla = {
		// set to LSB first
		.bit.DORD = 1,
		
		// set to asynchronous communication
		.bit.CMODE = 0,
		
		// set no parity
		.bit.FORM = 0,
		
		// set SERCOM PAD[1] to rx
		.bit.RXPO = 1,
		
		// set PAD[0] to tx
		.bit.TXPO = 0,
		
		// set internal clock
		.bit.MODE = 1
	};
	
	SERCOM0->USART.CTRLA.reg = ctrla.reg;
	
	
	SERCOM_USART_CTRLB_Type ctrlb = {
		// enable tx and rx
		.bit.RXEN = 1,
		.bit.TXEN = 1
	};
	
	SERCOM0->USART.CTRLB.reg = ctrlb.reg;
	
	while(SERCOM0->USART.SYNCBUSY.bit.CTRLB);
	
	
	// set baud to 115200
	SERCOM0->USART.BAUD.reg = 0; // 63019
	
	
	PORT_WRCONFIG_Type wrconfig = {
		// lower 16 bits
		.bit.HWSEL = 0,
		
		// enable update
		.bit.WRPINCFG = 1,
		
		.bit.WRPMUX = 1,
		
		// pin multiplexing function C
		.bit.PMUX = 2,
		
		// enable pin multiplexing
		.bit.PMUXEN = 1,
		
		// select pins
		// no shift? because lower half of pins
		.bit.PINMASK = (uint16_t)((PORT_PA08 | PORT_PA09))
		
	};
	
	PORT->Group[0].WRCONFIG.reg = wrconfig.reg;
	
	
	// enable USART
	SERCOM0->USART.CTRLA.bit.ENABLE = 1;
	
	// wait for synchronisation
	while(SERCOM0->USART.SYNCBUSY.bit.ENABLE);
}


void nav_uart_send(uint8_t data) {
	while(!SERCOM0->USART.INTFLAG.bit.DRE);
	
	SERCOM0->USART.DATA.reg = data;
}