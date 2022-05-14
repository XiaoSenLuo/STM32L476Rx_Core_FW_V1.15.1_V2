/**
  ******************************************************************************
  * File Name          : SPI.h
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __spi_H
#define __spi_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/


/* USER CODE BEGIN Includes */
#include "stm32l4xx_ll_spi.h"
/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN Private defines */
#ifndef LL_SPI
#define LL_SPI                   1
#endif
#ifndef SPI_WRITE_USE_DMA
#define SPI_WRITE_USE_DMA        1
#endif
#ifndef SPI_READ_USE_DMA
#define SPI_READ_USE_DMA         1
#endif
/* USER CODE END Private defines */

#if(LL_SPI == 0)

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

#endif
/* USER CODE BEGIN Prototypes */


void spi_init(SPI_TypeDef *spi_index);
void spi_deinit(SPI_TypeDef *c);

static inline void spi_internal_slave_selete(SPI_TypeDef *spi_index, uint8_t selete){
	if(selete == 1) SET_BIT(spi_index->CR1, SPI_CR1_SSI);
	if(selete == 0) CLEAR_BIT(spi_index->CR1, SPI_CR1_SSI);
}

static inline void spi_set_clk_div(SPI_TypeDef *spi, uint32_t _div){
	LL_SPI_SetBaudRatePrescaler(SPI1, _div); // 时钟分频, spi时钟频率 = PCLK2 / baudrate_presacler;
}

#define _SPI1_CS(cs)

uint8_t SPI1_Write_Read(uint8_t* tx_ptr, uint8_t* rx_ptr, uint16_t s);
uint8_t SPI1_Write(uint8_t* ptr, uint16_t s);
uint8_t SPI1_Read(uint8_t* ptr, uint16_t s);

#if(LL_SPI)
uint32_t get_spi1_dma_status(void);

//uint8_t _spi1_tx_dma_config(uint8_t *ptr, uint16_t _s);
//uint8_t _spi1_rx_dma_config(uint8_t *ptr, uint16_t _s);

#else

void hal_spi1_rxcplt_callback(void);
void hal_spi1_txcplt_callback(void);

#endif

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ spi_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
