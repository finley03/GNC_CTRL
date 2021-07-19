#include "dma.h"


//void serial_rx_dma_start() {
	//// select channel
	//DMAC->CHID.reg = SERIAL_RX_CHANNEL;
	//
	//// clear buffered data
	//serial_flush();
	//// clear transfer complete interrupt bit
	////DMAC->CHINTFLAG.bit.TCMPL = 1;
	//DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_MASK;
	//// enable DMA on channel
	//DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
//}
//
//uint8_t serial_rx_dma_end() {
	//// select channel
	//DMAC->CHID.reg = SERIAL_RX_CHANNEL;
	//
	//// check dma transfer complete
	//if (DMAC->CHINTFLAG.bit.TCMPL) {
		//return 1;
	//}
	//else return 0;
//}
//
//void serial_rx_init_dma() {
	//// configure DMA descriptor
	//// set event output selection disabled
	//dma_descriptor[SERIAL_RX_CHANNEL].BTCTRL.bit.EVOSEL = 0;
	//
	//// set no action after block transfer finish
	//dma_descriptor[SERIAL_RX_CHANNEL].BTCTRL.bit.BLOCKACT = 0;
	//
	//// set beat size to one byte
	//dma_descriptor[SERIAL_RX_CHANNEL].BTCTRL.bit.BEATSIZE = 0;
	//
	//// disable source address incrementing
	//dma_descriptor[SERIAL_RX_CHANNEL].BTCTRL.bit.SRCINC = 0;
	//
	//// enable destination address incrementing
	//dma_descriptor[SERIAL_RX_CHANNEL].BTCTRL.bit.DSTINC = 1;
	//
	//// step size selection applies to destination
	//dma_descriptor[SERIAL_RX_CHANNEL].BTCTRL.bit.STEPSEL = 0;
	//
	//// set step size to size of beat
	//dma_descriptor[SERIAL_RX_CHANNEL].BTCTRL.bit.STEPSIZE = 0;
	//
	//// set descriptor to valid
	//dma_descriptor[SERIAL_RX_CHANNEL].BTCTRL.bit.VALID = 1;
	//
	//
	//// set number of bytes to transfer
	//dma_descriptor[SERIAL_RX_CHANNEL].BTCNT = sizeof(transfer_request.reg);
	//
	//// set source address
	//dma_descriptor[SERIAL_RX_CHANNEL].SRCADDR = (uint32_t) &(SERCOM1->USART.DATA.reg);
	//
	//// set destination address
	//dma_descriptor[SERIAL_RX_CHANNEL].DSTADDR = ((uint32_t) &(transfer_request.reg[0]) + sizeof(transfer_request.reg));
	//
	//
	//// tell DMAC where descriptors are
	//DMAC->BASEADDR.reg = (uint32_t) &(dma_descriptor[0]);
	//DMAC->WRBADDR.reg = (uint32_t) &(dma_descriptor_writeback[0]);
	//
	//// enable bus clocks
	//PM->AHBMASK.bit.DMAC_ = 1;
	//PM->APBBMASK.bit.DMAC_ = 1;
	//
	//// enable DMAC
	//DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);
	//
	//// set DMAC channel 0 for further configuration
	//DMAC->CHID.reg = SERIAL_RX_CHANNEL;
	//
	//// set DMAC trigger source, priority to second highest value
	//DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(SERCOM1_DMAC_ID_RX) | DMAC_CHCTRLB_LVL_LVL2;
	//
	//serial_rx_dma_start();
//}


void wireless_rx_dma_start() {
	// select channel
	DMAC->CHID.reg = WIRELESS_RX_CHANNEL;
	
	// clear buffered data
	wireless_flush();
	// clear transfer complete interrupt bit
	//DMAC->CHINTFLAG.bit.TCMPL = 1;
	DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_MASK;
	// enable DMA on channel
	DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

uint8_t wireless_rx_dma_end() {
	// select channel
	DMAC->CHID.reg = WIRELESS_RX_CHANNEL;
	
	// check dma transfer complete
	if (DMAC->CHINTFLAG.bit.TCMPL) {
		return 1;
	}
	else return 0;
}

void wireless_rx_init_dma() {
	// configure DMA descriptor
	// set event output selection disabled
	dma_descriptor[WIRELESS_RX_CHANNEL].BTCTRL.bit.EVOSEL = 0;
	
	// set no action after block transfer finish
	dma_descriptor[WIRELESS_RX_CHANNEL].BTCTRL.bit.BLOCKACT = 0;
	
	// set beat size to one byte
	dma_descriptor[WIRELESS_RX_CHANNEL].BTCTRL.bit.BEATSIZE = 0;
	
	// disable source address incrementing
	dma_descriptor[WIRELESS_RX_CHANNEL].BTCTRL.bit.SRCINC = 0;
	
	// enable destination address incrementing
	dma_descriptor[WIRELESS_RX_CHANNEL].BTCTRL.bit.DSTINC = 1;
	
	// step size selection applies to destination
	dma_descriptor[WIRELESS_RX_CHANNEL].BTCTRL.bit.STEPSEL = 0;
	
	// set step size to size of beat
	dma_descriptor[WIRELESS_RX_CHANNEL].BTCTRL.bit.STEPSIZE = 0;
	
	// set descriptor to valid
	dma_descriptor[WIRELESS_RX_CHANNEL].BTCTRL.bit.VALID = 1;
	
	
	// set number of bytes to transfer
	dma_descriptor[WIRELESS_RX_CHANNEL].BTCNT = sizeof(transfer_request.reg);
	
	// set source address
	dma_descriptor[WIRELESS_RX_CHANNEL].SRCADDR = (uint32_t) &(SERCOM1->USART.DATA.reg);
	
	// set destination address
	dma_descriptor[WIRELESS_RX_CHANNEL].DSTADDR = ((uint32_t) &(transfer_request.reg[0]) + sizeof(transfer_request.reg));
	
	
	// tell DMAC where descriptors are
	DMAC->BASEADDR.reg = (uint32_t) &(dma_descriptor[0]);
	DMAC->WRBADDR.reg = (uint32_t) &(dma_descriptor_writeback[0]);
	
	// enable bus clocks
	PM->AHBMASK.bit.DMAC_ = 1;
	PM->APBBMASK.bit.DMAC_ = 1;
	
	// enable DMAC
	DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);
	
	// set DMAC channel 0 for further configuration
	DMAC->CHID.reg = WIRELESS_RX_CHANNEL;
	
	// set DMAC trigger source, priority to second highest value
	DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(SERCOM1_DMAC_ID_RX) | DMAC_CHCTRLB_LVL_LVL2;
	
	wireless_rx_dma_start();
}