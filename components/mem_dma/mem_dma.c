/*
 * mem_dma.c
 *
 *  Created on: 2021年1月17日
 *      Author: XIAOSENLUO
 */

#include "mem_dma.h"


static volatile unsigned char mem_dma_status = 0;


void*    memcpy_dma (void *__restrict dst, const void *__restrict src, size_t s){
//    uint32_t dma_ch = 0;

//    dma_ch = (uint32_t)MEM_DMA + CHANNEL_OFFSET_TAB[LL_DMA_CHANNEL_1]; // 0x40020008UL
	NVIC_DisableIRQ(MEM_DMA_IRQ);
//	LL_DMA_DisableChannel(MEM_DMA, MEM_DMA_CH);
	CLEAR_BIT(((DMA_Channel_TypeDef *)0x40020008UL)->CCR, DMA_CCR_EN);

	LL_DMA_ConfigTransfer(MEM_DMA, MEM_DMA_CH, LL_DMA_DIRECTION_MEMORY_TO_MEMORY |
												LL_DMA_PRIORITY_HIGH              |
												LL_DMA_MODE_NORMAL                |
												LL_DMA_PERIPH_INCREMENT           |
												LL_DMA_MEMORY_INCREMENT           |
												LL_DMA_PDATAALIGN_BYTE            |
												LL_DMA_MDATAALIGN_BYTE);

//	LL_DMA_SetDataLength(MEM_DMA, MEM_DMA_CH, (uint32_t)s);
	WRITE_REG(((DMA_Channel_TypeDef *)(0x40020008UL))->CNDTR, s);   // 设置传输数据大小
//	LL_DMA_ConfigAddresses(MEM_DMA, MEM_DMA_CH, (uint32_t)src, (uint32_t)dst, LL_DMA_DIRECTION_MEMORY_TO_MEMORY);
	WRITE_REG(((DMA_Channel_TypeDef *)(0x40020008UL))->CPAR, (uint32_t)src);  // DMA1 Channel 1
	WRITE_REG(((DMA_Channel_TypeDef *)(0x40020008UL))->CMAR, (uint32_t)dst);
//	LL_DMA_SetPeriphRequest(MEM_DMA, LL_DMA_CHANNEL_1, LL_DMA_REQUEST_1);

//	dma_ch = ((uint32_t)MEM_DMA + DMA_CSELR_OFFSET);  // 0x400200a8UL
//    uint32_t tmp[2] = {0};
//	tmp[0] = DMA_CSELR_C1S << ((LL_DMA_CHANNEL_1) * 4U);  // 0x0F
//	tmp[1] = LL_DMA_REQUEST_1 << POSITION_VAL(DMA_CSELR_C1S << (LL_DMA_CHANNEL_1*4U));  // 0x01
//	MODIFY_REG(((DMA_Request_TypeDef *)((uint32_t)dma_ch))->CSELR,
//			tmp[0], tmp[1]);
    WRITE_REG(((DMA_Request_TypeDef *)((uint32_t)0x400200A8UL))->CSELR, (((DMA_Request_TypeDef *)((uint32_t)0x400200A8UL))->CSELR) | 0x01);   // 设置DMA请求
//	LL_DMA_EnableChannel(MEM_DMA, MEM_DMA_CH);
    SET_BIT(((DMA_Channel_TypeDef *)0x40020008UL)->CCR, DMA_CCR_EN);
//	while(LL_DMA_IsActiveFlag_TC1(MEM_DMA) == 0);
//	while(!(MEM_DMA->ISR & DMA_ISR_TCIF1));
	while(!(((DMA_TypeDef *)0x40020000UL)->ISR & DMA_ISR_TCIF1));
	LL_DMA_ClearFlag_TC1(MEM_DMA);
	return dst;
}

void*	 memset_dma (void * dst, int n, size_t s){
//	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	return NULL;
}

char 	*strcpy_dma (char *__restrict dst, const char *__restrict src){
    size_t l = 0;
    l = strlen(src);
	return memcpy_dma(dst, src, l);
}

char 	*strncpy_dma (char *__restrict dst, const char *__restrict src, size_t s){
//	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	size_t l = 0;
	l = strlen(src);
	l = (l > s) ? s : l;
	return memcpy_dma(dst, src, l);
}

void mem_dma_callback(void){

}
