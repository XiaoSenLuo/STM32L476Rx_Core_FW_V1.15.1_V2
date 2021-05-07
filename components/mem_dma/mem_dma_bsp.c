/*
 * mem_dma_bsp.c
 *
 *  Created on: 2021年1月17日
 *      Author: XIAOSENLUO
 */

#include "mem_dma_bsp.h"


void mem_dma_init(void){
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	LL_DMA_DisableChannel(MEM_DMA, MEM_DMA_CH);

//	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_MEMORY_TO_MEMORY);
//	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);
//	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);
//	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_INCREMENT);
//	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
//	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);
//	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_ConfigTransfer(MEM_DMA, MEM_DMA_CH, LL_DMA_DIRECTION_MEMORY_TO_MEMORY |
												LL_DMA_PRIORITY_HIGH              |
												LL_DMA_MODE_NORMAL                |
												LL_DMA_PERIPH_INCREMENT           |
												LL_DMA_MEMORY_INCREMENT           |
												LL_DMA_PDATAALIGN_BYTE            |
												LL_DMA_MDATAALIGN_BYTE);
}


