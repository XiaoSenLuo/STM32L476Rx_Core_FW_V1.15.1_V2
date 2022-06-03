/**
  ******************************************************************************
  * File Name          : SDMMC.c
  * Description        : This file provides code for the configuration
  *                      of the SDMMC instances.
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

#include "stm32l4xx_it.h"

#include "sdmmc.h"

static SD_HandleTypeDef hsd = { 0 };

#if(0)
static DMA_HandleTypeDef hdma_sdmmc1_tx = { 0 };
static DMA_HandleTypeDef hdma_sdmmc1_rx = { 0 };
#else
static DMA_HandleTypeDef hdma_sdmmc = { 0 };
#endif

static void dma_sdmmc_tx_isr_callback(void *ctx){
    HAL_DMA_IRQHandler((DMA_HandleTypeDef*)ctx);
}

static void dma_sdmmc_rx_isr_callback(void *ctx){
    HAL_DMA_IRQHandler((DMA_HandleTypeDef*)ctx);
}

static void dma_sdmmc_tx_rx_isr_callback(void *ctx){
    HAL_DMA_IRQHandler((DMA_HandleTypeDef*)ctx);
}

static void sdmmc_isr_callback(void *ctx){
    HAL_SD_IRQHandler((SD_HandleTypeDef*)ctx);
}


/* USER CODE BEGIN 0 */
int sdmmc_initialize(SD_HandleTypeDef* *hsd_handle){
    int err = 0, retry = 60;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
#if(0)
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {
            .PeriphClockSelection = RCC_PERIPHCLK_SDMMC1,
            .Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1,
            .PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK,
            .PLLSAI1.PLLSAI1M = 2,
            .PLLSAI1.PLLSAI1N = 12,
            .PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7,
            .PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2,
            .PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2,
            .PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE,
    };
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
#else
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {
            .PeriphClockSelection = RCC_PERIPHCLK_SDMMC1,
            .Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLL,
    };
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
#endif

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_SDMMC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /**SDMMC1 GPIO Configuration
PC8     ------> SDMMC1_D0
PC9     ------> SDMMC1_D1
PC10     ------> SDMMC1_D2
PC11     ------> SDMMC1_D3
PC12     ------> SDMMC1_CK
PD2     ------> SDMMC1_CMD
*/
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_15;  /// SD DETECT PIN
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

#if(0)
    /* SDMMC1 DMA Init */
    /* SDMMC1_TX Init */
    hdma_sdmmc1_tx.Instance = DMA2_Channel4;
    hdma_sdmmc1_tx.Init.Request = DMA_REQUEST_7;
    hdma_sdmmc1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_sdmmc1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sdmmc1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sdmmc1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sdmmc1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_sdmmc1_tx.Init.Mode = DMA_NORMAL;
    hdma_sdmmc1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_sdmmc1_tx);
    __HAL_LINKDMA(&hsd, hdmatx, hdma_sdmmc1_tx);
    ll_peripheral_isr_install(DMA2_Channel4_IRQn, dma_sdmmc_tx_isr_callback, hsd.hdmatx);
    HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

    /* SDMMC1_RX Init */

    hdma_sdmmc1_rx.Instance = DMA2_Channel5;
    hdma_sdmmc1_rx.Init.Request = DMA_REQUEST_7;
    hdma_sdmmc1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_sdmmc1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sdmmc1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sdmmc1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sdmmc1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_sdmmc1_rx.Init.Mode = DMA_NORMAL;
    hdma_sdmmc1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_sdmmc1_rx);
    __HAL_LINKDMA(&hsd, hdmarx, hdma_sdmmc1_rx);
    ll_peripheral_isr_install(DMA2_Channel5_IRQn, dma_sdmmc_rx_isr_callback, hsd.hdmarx);
    HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);
#else
    hdma_sdmmc.Instance = DMA2_Channel4;
    hdma_sdmmc.Init.Request = DMA_REQUEST_7;
    hdma_sdmmc.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_sdmmc.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sdmmc.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sdmmc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sdmmc.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_sdmmc.Init.Mode = DMA_NORMAL;
    hdma_sdmmc.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    HAL_DMA_Init(&hdma_sdmmc);

    __HAL_LINKDMA(&hsd, hdmatx, hdma_sdmmc);
    __HAL_LINKDMA(&hsd, hdmarx, hdma_sdmmc);

    ll_peripheral_isr_install(DMA2_Channel4_IRQn, dma_sdmmc_tx_rx_isr_callback, &hdma_sdmmc);

    HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

#endif
    hsd.Instance = SDMMC1;
    hsd.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass = SDMMC_CLOCK_BYPASS_ENABLE;
    hsd.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_ENABLE;
    hsd.Init.BusWide = SDMMC_BUS_WIDE_1B;
    hsd.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
    hsd.Init.ClockDiv = 0;

    if(sd_detect()){
        while(sd_detect() && (retry)){
            HAL_Delay(500);
            retry--;
        }
        if(!retry){
            err = 3; /* 3: Not Ready */
            goto end_section;
        }
        HAL_Delay(1000);  /// 等待稳定
    }

    ll_peripheral_isr_install(SDMMC1_IRQn, sdmmc_isr_callback, &hsd);
    HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);



    err = HAL_SD_Init(&hsd);
    if(err == 0){
        hsd.Init.BusWide = SDMMC_BUS_WIDE_4B;
        err = HAL_SD_ConfigWideBusOperation(&hsd, SDMMC_BUS_WIDE_4B);
        if(err) hsd.Init.BusWide = SDMMC_BUS_WIDE_1B;
    }else{
        err = 3; /* 3: Not Ready */
    }
    end_section:
    if(hsd_handle) *hsd_handle = (err == 0) ? &hsd : NULL;
    return err;
}


int sdmmc_deinitialize(SD_HandleTypeDef *hsd_handle){
    int err = 0;
    if(hsd_handle->Instance != SDMMC1) return 3;

    if(HAL_SD_DeInit(hsd_handle) != HAL_OK){
        err = 1;
    }

    HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Channel4_IRQn);

    HAL_GPIO_DeInit(GPIOC, (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12));
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

    HAL_DMA_DeInit(hsd_handle->hdmarx);
    HAL_DMA_DeInit(hsd_handle->hdmatx);

    __HAL_RCC_SDMMC1_CLK_DISABLE();

    return err;
}

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
