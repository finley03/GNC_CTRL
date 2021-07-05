#include "dma.h"

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
	//dma_descriptor[SERIAL_RX_CHANNEL].BTCTRL.bit.VALID = 0;
	//
	//
//}