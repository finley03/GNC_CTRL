// header file for direct memory access

#ifndef DMA_H
#define DMA_H

#include "util.h"

//#define DMA_CHANNELS 2
//
//typedef struct __attribute__((aligned(16))) {
	//union {
		//struct {
			//uint16_t VALID:1;
			//uint16_t EVOSEL:2;
			//uint16_t BLOCKACT:2;
			//uint16_t :3;
			//uint16_t BEATSIZE:2;
			//uint16_t SRCINC:1;
			//uint16_t DSTINC:1;
			//uint16_t STEPSEL:1;
			//uint16_t STEPSIZE:3;
		//} bit;
		//
		//uint16_t reg;
	//} BTCTRL;
	//
	//uint16_t BTCNT;
	//uint32_t SRCADDR;
	//uint32_t DSTADDR;
	//uint32_t DESCADDR;
//} DMA_DESCRIPTOR_Type;
//
//
//volatile DMA_DESCRIPTOR_Type dma_descriptor[DMA_CHANNELS];
//DMA_DESCRIPTOR_Type dma_descriptor_writeback[DMA_CHANNELS];
//
//// SERIAL RX on DMA channel 0
//#define SERIAL_RX_CHANNEL 0
//
//void serial_rx_init_dma();

#endif