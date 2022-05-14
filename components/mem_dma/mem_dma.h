/*
 * mem_dma.h
 *
 *  Created on: 2021年1月17日
 *      Author: XIAOSENLUO
 */

#ifndef MEM_DMA_MEM_DMA_H_
#define MEM_DMA_MEM_DMA_H_

#include "stddef.h"
#include "stdint.h"
#include "string.h"
#include "mem_dma_bsp.h"

/**
 *  当 数据大小大于38byte时, 此函数才会有优势
 */
void*    memcpy_dma (void *__restrict, const void *__restrict, size_t);
void*	 memset_dma (void *, int, size_t);
char 	*strcpy_dma (char *__restrict, const char *__restrict);
char 	*strncpy_dma (char *__restrict, const char *__restrict, size_t);


#endif /* MEM_DMA_MEM_DMA_H_ */
