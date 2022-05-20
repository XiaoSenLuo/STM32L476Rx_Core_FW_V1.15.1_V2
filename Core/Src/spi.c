/**
  ******************************************************************************
  * File Name          : SPI.c
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

/* Includes ------------------------------------------------------------------*/


/* USER CODE BEGIN 0 */

#include "stm32l4xx_it.h"

#include "spi.h"


static SPI_HandleTypeDef hspi1 = { 0 };
static DMA_HandleTypeDef hdma_hspi1_tx = {0 };
static DMA_HandleTypeDef hdma_hspi1_rx = {0 };


static void dma_spi1_tx_isr_handler(void* ctx){
    HAL_DMA_IRQHandler((DMA_HandleTypeDef*)ctx);
}


static void dma_spi1_rx_isr_handler(void* ctx){
    HAL_DMA_IRQHandler((DMA_HandleTypeDef*)ctx);
}

static void spi1_isr_handler(void* ctx){
    HAL_SPI_IRQHandler((SPI_HandleTypeDef*)ctx);
}

void st_spi1_init(SPI_HandleTypeDef* *SpiHandle){
    GPIO_InitTypeDef  GPIO_InitStruct = {
            .Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
            .Mode = GPIO_MODE_AF_PP,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF5_SPI1,
            .Pull = GPIO_NOPULL,
    };
    __HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#if(0)
    hdma_hspi1_tx.Instance                 = DMA1_Channel3;
    hdma_hspi1_tx.Init.Request             = DMA_REQUEST_1;
    hdma_hspi1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_hspi1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_hspi1_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_hspi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_hspi1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_hspi1_tx.Init.Mode                = DMA_NORMAL;
    hdma_hspi1_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_DMA_Init(&hdma_hspi1_tx);
    __HAL_LINKDMA(&hspi1, hdmatx, hdma_hspi1_tx);

    hdma_hspi1_rx.Instance                 = DMA1_Channel2;
    hdma_hspi1_rx.Init.Request             = DMA_REQUEST_1;
    hdma_hspi1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_hspi1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_hspi1_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_hspi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_hspi1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_hspi1_rx.Init.Mode                = DMA_NORMAL;
    hdma_hspi1_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;

    HAL_DMA_Init(&hdma_hspi1_rx);
    __HAL_LINKDMA(&hspi1, hdmarx, hdma_hspi1_rx);

    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    HAL_NVIC_SetPriority(SPI1_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
#endif
    hspi1.Instance               = SPI1;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 7;
    hspi1.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    __HAL_RCC_SPI1_CLK_ENABLE();
    if(HAL_SPI_Init(&hspi1) == HAL_OK){
        *SpiHandle = &hspi1;
#if(0)
        ll_peripheral_isr_install(DMA1_Channel3_IRQn, spi1_dma_tx_isr_handler, hspi1.hdmatx);
        ll_peripheral_isr_install(DMA1_Channel2_IRQn, spi1_dma_rx_isr_handler, hspi1.hdmarx);
        ll_peripheral_isr_install(SPI1_IRQn, spi1_isr_handler, &hspi1);
#endif
    }
}



/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
