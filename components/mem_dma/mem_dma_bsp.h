/*
 * mem_dma_bsp.h
 *
 *  Created on: 2021年1月17日
 *      Author: XIAOSENLUO
 */

#ifndef MEM_DMA_MEM_DMA_BSP_H_
#define MEM_DMA_MEM_DMA_BSP_H_


#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_bus.h"

#ifndef MEM_DMA
#define MEM_DMA                    DMA1
#endif
#ifndef MEM_DMA_CH
#define MEM_DMA_CH                 LL_DMA_CHANNEL_1
#endif
#ifndef MEM_DMA_IRQ
#define MEM_DMA_IRQ                DMA1_Channel1_IRQn  // 11
#endif

void mem_dma_init(void);

void mem_dma_callback(void);

#endif /* MEM_DMA_MEM_DMA_BSP_H_ */
