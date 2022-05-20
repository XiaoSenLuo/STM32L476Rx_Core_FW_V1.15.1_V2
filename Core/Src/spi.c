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

//#include "stm32l4xx_hal_spi.h"
//#include "stm32l4xx_hal_gpio.h"


#include <string.h>

#include "spi.h"


#if(1)


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

#if(0)
#include "bsp_ads127.h"
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    ads127_bsp_read_data_cplt_callback();
}
#endif
#endif


#if(0)

#if(LL_SPI == 0)


SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;


void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1){
  /* USER CODE BEGIN SPI1_MspInit 0 */
    __HAL_RCC_DMA1_CLK_ENABLE();
  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;   // 复用功能不能上拉, 否者无法使用
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#if(SPI_READ_USE_DMA)
    /* SPI1 DMA Init */
    /* SPI1_RX Init */
    hdma_spi1_rx.Instance = DMA1_Channel2;
    hdma_spi1_rx.Init.Request = DMA_REQUEST_1;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK){
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi1_rx);
#endif
#if(SPI_WRITE_USE_DMA)
    /* SPI1_TX Init */
    hdma_spi1_tx.Instance = DMA1_Channel3;
    hdma_spi1_tx.Init.Request = DMA_REQUEST_1;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK){
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi1_tx);
#endif
    /* SPI1 interrupt Init */
//#if(!(SPI_WRITE_USE_DMA && SPI_READ_USE_DMA))
    HAL_NVIC_SetPriority(SPI1_IRQn, SPI1_Priority, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
//#endif
  /* USER CODE BEGIN SPI1_MspInit 1 */
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, DMA1_Channel3_Priority, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, DMA1_Channel2_Priority, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);
#if(SPI_READ_USE_DMA)
    /* SPI1 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);
#endif
#if(SPI_WRITE_USE_DMA)
    HAL_DMA_DeInit(spiHandle->hdmatx);
#endif
    /* SPI1 interrupt Deinit */
//    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */
    HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);
  /* USER CODE END SPI1_MspDeInit 1 */
  }
}
#else
static void spi1_irq_handler(void){
	uint32_t _sr = 0, _cr2 = 0;
	_sr = SPI1->SR;
	_cr2 = SPI1->CR2;

	if((_sr & SPI_SR_TXE) && (_cr2 & SPI_CR2_TXEIE)){

	}
	if(!(_sr & SPI_SR_OVR) && (_sr & SPI_SR_RXNE) && (_cr2 & SPI_CR2_RXNEIE)){

	}
	if((_cr2 & SPI_CR2_ERRIE) && ((_sr & SPI_SR_MODF) || (_sr & SPI_SR_OVR) || (_sr & SPI_SR_FRE))){
		if(_sr & SPI_SR_MODF){
            LL_SPI_ClearFlag_MODF(SPI1);
		}
		if(_sr & SPI_SR_OVR){
            LL_SPI_ClearFlag_OVR(SPI1);
		}
		if(_sr & SPI_SR_FRE){
            LL_SPI_ClearFlag_FRE(SPI1);
		}
		LL_SPI_Disable(SPI1);
	}
}

#endif
/* USER CODE END 0 */

/* SPI1 init function */
static void spi1_irq_callback(void);
static void spi1_dma_rxcplt_callback(void);
static void spi1_dma_txcplt_callback(void);

void spi_init(SPI_TypeDef *spi_index){
#if(LL_SPI)
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint32_t _spi_div = LL_SPI_BAUDRATEPRESCALER_DIV8;
    LL_RCC_ClocksTypeDef rcc_clk = { 0 };
	if((uint32_t)spi_index == (uint32_t)SPI1){

		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

		GPIO_InitStruct.Pin = LL_GPIO_PIN_7 | LL_GPIO_PIN_6 | LL_GPIO_PIN_5;
		GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
		GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
		LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
		LL_RCC_GetSystemClocksFreq(&rcc_clk);
//		_spi_div = (rcc_clk.PCLK2_Frequency >32000000) ? (LL_SPI_BAUDRATEPRESCALER_DIV2) : (LL_SPI_BAUDRATEPRESCALER_DIV8);
		if(rcc_clk.PCLK2_Frequency <= 8000000) _spi_div = LL_SPI_BAUDRATEPRESCALER_DIV8;
		if((rcc_clk.PCLK2_Frequency > 8000000) && (rcc_clk.PCLK2_Frequency <= 16000000)) _spi_div = LL_SPI_BAUDRATEPRESCALER_DIV4;
		if((rcc_clk.PCLK2_Frequency > 16000000) && (rcc_clk.PCLK2_Frequency <= 32000000)) _spi_div = LL_SPI_BAUDRATEPRESCALER_DIV4;
        if((rcc_clk.PCLK2_Frequency) > 32000000) _spi_div = LL_SPI_BAUDRATEPRESCALER_DIV8;
		/* Configure SPI1 communication */
		LL_SPI_SetBaudRatePrescaler(SPI1, _spi_div); // 时钟分频, spi时钟频率 = PCLK2 / baudrate_presacler;
		LL_SPI_SetTransferDirection(SPI1,LL_SPI_FULL_DUPLEX);   // 全双工
		LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_2EDGE);         // 相位
		LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_LOW);     // 极性
		/* Reset value is LL_SPI_MSB_FIRST */
		LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);     // 高位先出
		LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);       // 数据宽度 8bit
		LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);               // 软件片选
		LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);  //8bit
		LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);               // 主机
		LL_SPI_DisableCRC(SPI1);
	//	LL_SPI_Enable(SPI1);

		HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, DMA1_Channel2_Priority, 0);
		HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, DMA1_Channel3_Priority, 0);

		LL_SPI_EnableIT_ERR(SPI1);
	}
#else
	uint32_t _clk2 = 0, _spi_div = SPI_BAUDRATEPRESCALER_8;
	_clk2 = HAL_RCC_GetPCLK2Freq();
	if(_clk2 <= 8000000) _spi_div = SPI_BAUDRATEPRESCALER_2;
	if((_clk2 > 8000000) && (_clk2 <= 16000000)) _spi_div = SPI_BAUDRATEPRESCALER_4;
	if((_clk2 > 16000000) && (_clk2 <= 32000000)) _spi_div = SPI_BAUDRATEPRESCALER_4;
    if(_clk2 > 32000000) _spi_div = SPI_BAUDRATEPRESCALER_8;
	  hspi1.Instance = SPI1;
	  hspi1.Init.Mode = SPI_MODE_MASTER;
	  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	  hspi1.Init.NSS = SPI_NSS_SOFT;
	  hspi1.Init.BaudRatePrescaler = _spi_div;
	  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	  hspi1.Init.CRCPolynomial = 7;
	  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	  if (HAL_SPI_Init(&hspi1) != HAL_OK)
	  {
	    Error_Handler();
	  }
#endif
	st_irq_handler_register(SPI1_IRQn, spi1_irq_callback);
}

void spi_deinit(SPI_TypeDef *spi_index){
	if((uint32_t)spi_index == (uint32_t)SPI1){
        LL_SPI_DeInit(spi_index);
	}
}

#if(LL_SPI)
static uint8_t _spi1_tx_dma_config(uint8_t *ptr, uint16_t _s){  // DMA1 Channel3; Request 1
	uint8_t ret = 0;
//	__IO uint32_t _dma_ch = 0, _dma_csl = 0;
	/* (1) Enable the clock of DMA1 and DMA1 */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	/* (2) Configure NVIC for DMA transfer complete/error interrupts */

	NVIC_EnableIRQ(DMA1_Channel3_IRQn);

	const uint32_t _dma_ch = (uint32_t)DMA1 + (uint32_t)(CHANNEL_OFFSET_TAB[LL_DMA_CHANNEL_3]);

	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
	LL_DMA_ClearFlag_GI3(DMA1);
	/* (3) Configure the DMA1_Channel2 functional parameters */
//	LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_3,
//			            LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
//						LL_DMA_PRIORITY_HIGH |
//						LL_DMA_MODE_NORMAL |
//						LL_DMA_PERIPH_NOINCREMENT |
//						LL_DMA_MEMORY_INCREMENT |
//						LL_DMA_PDATAALIGN_BYTE |
//						LL_DMA_MDATAALIGN_BYTE);

    WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CPAR, LL_SPI_DMA_GetRegAddr(SPI1)); // 目的地址
    WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CMAR, (uint32_t)ptr);               // 源地址
//	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, _s);
	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CNDTR, _s);

	if(LL_DMA_GetPeriphRequest(DMA1, LL_DMA_CHANNEL_3) != LL_DMA_REQUEST_1)
		LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_1);

	/* (5) Enable DMA interrupts complete/error */
//	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
//	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);
	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CCR,
			LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
			LL_DMA_PRIORITY_HIGH |
			LL_DMA_MODE_NORMAL |
			LL_DMA_PERIPH_NOINCREMENT |
			LL_DMA_MEMORY_INCREMENT |
			LL_DMA_PDATAALIGN_BYTE |
			LL_DMA_MDATAALIGN_BYTE |
			DMA_CCR_TCIE |
			DMA_CCR_TEIE |
			DMA_CCR_EN);
//	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
	return ret;
}

static uint8_t _spi1_rx_dma_config(uint8_t *ptr, uint16_t _s){   // DMA1 Channel2; Request 1
	uint8_t ret = 0;
//	__IO uint32_t _dma_ch = 0, _dma_csl = 0;
	/* (1) Enable the clock of DMA1 and DMA1 */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	const uint32_t _dma_ch = (uint32_t)DMA1 + (uint32_t)(CHANNEL_OFFSET_TAB[LL_DMA_CHANNEL_2]);
	/* (2) Configure NVIC for DMA transfer complete/error interrupts */

	NVIC_EnableIRQ(DMA1_Channel2_IRQn);

//	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
	CLEAR_BIT(((DMA_Channel_TypeDef *)(_dma_ch))->CCR, DMA_CCR_EN);

	LL_DMA_ClearFlag_GI2(DMA1);
	/* (3) Configure the DMA1_Channel2 functional parameters */
//	LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_2,
//			            LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
//						LL_DMA_PRIORITY_HIGH |
//						LL_DMA_MODE_NORMAL |
//						LL_DMA_PERIPH_NOINCREMENT |
//						LL_DMA_MEMORY_INCREMENT |
//						LL_DMA_PDATAALIGN_BYTE |
//						LL_DMA_MDATAALIGN_BYTE);

//	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2,
//			LL_SPI_DMA_GetRegAddr(SPI1),  // 源地址
//			(uint32_t)ptr,                // 目的地址
//			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CPAR, LL_SPI_DMA_GetRegAddr(SPI1)); // 源地址
    WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CMAR, (uint32_t)ptr);               // 目的地址
//	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, _s);
	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CNDTR, _s);

	if(LL_DMA_GetPeriphRequest(DMA1, LL_DMA_CHANNEL_2) != LL_DMA_REQUEST_1)
		LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMA_REQUEST_1);

	/* (5) Enable DMA interrupts complete/error */
//	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
//	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);
	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CCR,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
			LL_DMA_PRIORITY_HIGH |
			LL_DMA_MODE_NORMAL |
			LL_DMA_PERIPH_NOINCREMENT |
			LL_DMA_MEMORY_INCREMENT |
			LL_DMA_PDATAALIGN_BYTE |
			LL_DMA_MDATAALIGN_BYTE |
			DMA_CCR_TCIE |
			DMA_CCR_TEIE |
			DMA_CCR_EN);
//	LL_DMA_EnableChannel(DMA1,  LL_DMA_CHANNEL_2);
    return ret;
}


/* USER CODE BEGIN 1 */

#endif

uint8_t SPI1_Write(uint8_t* ptr, uint16_t s){
    uint8_t _ret = 0;

#if(LL_SPI)

    uint32_t _timeout = 8000000;
    CLEAR_BIT(SPI1->CR2, SPI_CR2_LDMATX);

	spi_wr_dma_status |= 0x01;
	st_irq_handler_register(DMA1_Channel3_IRQn, spi1_dma_txcplt_callback);
	_ret = _spi1_tx_dma_config(ptr, s);

	LL_SPI_EnableDMAReq_TX(SPI1);
	if(LL_SPI_IsEnabled(SPI1) == 0)
		LL_SPI_Enable(SPI1);
	while((spi_wr_dma_status & 0x01) && (_timeout--));
	if((_timeout == 0) && (spi_wr_dma_status & 0x01)) _ret = 255;
	while(LL_SPI_IsActiveFlag_BSY(SPI1));
	LL_SPI_ClearFlag_OVR(SPI1);
	LL_SPI_Disable(SPI1);
	LL_SPI_DisableDMAReq_TX(SPI1);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
	LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_3);
	LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_3);
    while(LL_SPI_GetRxFIFOLevel(SPI1)){
    	READ_REG(*((__IO uint16_t *)&SPI1->DR));   // 清空接受缓冲区
    }

#else
	spi_write_status |= SPI_STATUS_WRITE;
#if(SPI_WRITE_USE_DMA)
	st_irq_handler_register(DMA1_Channel3_IRQn, spi1_dma_txcplt_callback);
	_ret = HAL_SPI_Transmit_DMA(&hspi1, ptr, s);
#else
    st_irq_handler_register(SPI1_IRQn, NULL);
    ret = HAL_SPI_Transmit_IT(&hspi1, ptr, s);
#endif
    if(_ret == 0) while((spi_write_status & SPI_STATUS_WRITE) != 0);
    else _ret = 1;
#endif
    return _ret;
}

uint8_t SPI1_Read(uint8_t* ptr, uint16_t s){
    uint8_t _ret = 0;
#if(LL_SPI)
    memset(ptr, 0, s);
    _ret = SPI1_Write_Read(ptr, ptr, s);

#else
    spi_read_status |= SPI_STATUS_READ;
#if(SPI_READ_USE_DMA)
    st_irq_handler_register(DMA1_Channel2_IRQn, spi1_dma_rxcplt_callback);
    st_irq_handler_register(DMA1_Channel3_IRQn, spi1_dma_txcplt_callback);
    _ret = HAL_SPI_Receive_DMA(&hspi1, ptr, s);
#else
    ret = HAL_SPI_Receive_IT(&hspi1, ptr, s);
#endif
    if(_ret == 0) while((spi_read_status & SPI_STATUS_READ) != 0);
    else _ret = 1;
#endif
	return _ret;
}

uint8_t SPI1_Write_Read(uint8_t* tx_ptr, uint8_t* rx_ptr, uint16_t s){
    uint8_t ret = 0;

#if(LL_SPI)
//    register uint32_t _ftlvl = 3;
    uint32_t _timeout = 8000000;
	CLEAR_BIT(SPI1->CR2, SPI_CR2_LDMATX | SPI_CR2_LDMARX);

	if(rx_ptr != NULL){
	    while(LL_SPI_GetRxFIFOLevel(SPI1)) READ_REG(*((__IO uint16_t *)&SPI1->DR));
		spi_wr_dma_status |= 0x10;  // 读
		st_irq_handler_register(DMA1_Channel2_IRQn, spi1_dma_rxcplt_callback);
//		LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);  //8bit
		ret = _spi1_rx_dma_config(rx_ptr, s);
		LL_SPI_EnableDMAReq_RX(SPI1);
	}
	if(tx_ptr != NULL){
		spi_wr_dma_status |= 0x01;  // 写
		st_irq_handler_register(DMA1_Channel3_IRQn, spi1_dma_txcplt_callback);
		ret = _spi1_tx_dma_config(tx_ptr, s);
		if(LL_SPI_IsEnabled(SPI1) == 0) LL_SPI_Enable(SPI1);
		LL_SPI_EnableDMAReq_TX(SPI1);
	}
	if(tx_ptr != NULL){  // 等待发送完成
		while((spi_wr_dma_status & 0x01) && (_timeout--));
//		while(_ftlvl){
//			_ftlvl = LL_SPI_GetTxFIFOLevel(SPI1);
//		}
		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
		LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_3);
		LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_3);
	}
	if(rx_ptr != NULL){  // 等待接受完成
		while((spi_wr_dma_status & 0x10) && (_timeout--));
		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
		LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_2);
		LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_2);
	}
	LL_SPI_DisableDMAReq_RX(SPI1);
	LL_SPI_DisableDMAReq_TX(SPI1);
	if((_timeout == 0) && ((spi_wr_dma_status & 0x01) || (spi_wr_dma_status & 0x10))) ret = 255;
	while(LL_SPI_IsActiveFlag_BSY(SPI1));
	LL_SPI_ClearFlag_OVR(SPI1);
    LL_SPI_Disable(SPI1);

#else
    spi_read_status |= SPI_STATUS_READ;
    spi_write_status |= SPI_STATUS_WRITE;
#if(SPI_WRITE_USE_DMA && SPI_READ_USE_DMA)
    st_irq_handler_register(DMA1_Channel2_IRQn, spi1_dma_rxcplt_callback);
    st_irq_handler_register(DMA1_Channel3_IRQn, spi1_dma_txcplt_callback);
    ret = HAL_SPI_TransmitReceive_DMA(&hspi1, tx_ptr, rx_ptr, s);
#else
    ret = HAL_SPI_TransmitReceive_IT(&hspi1, tx_ptr, rx_ptr, s);
#endif
    if(ret == 0) while((spi_read_status & SPI_STATUS_READ) || (spi_write_status & SPI_STATUS_WRITE));
    else ret = 1;
#endif
//    _SPI1_CS(1);
    return ret;
}


#if(LL_SPI)
uint32_t get_spi1_dma_status(void){
	return spi_wr_dma_status;
}
#endif

void spi1_dma_txcplt_callback(void){        // DMA1 Channel 3, request 1
#if(LL_SPI)
	if(LL_DMA_IsActiveFlag_TC3(DMA1)){
		LL_DMA_ClearFlag_GI3(DMA1);
		spi_wr_dma_status &= (~SPI_STATUS_WRITE);
		spi_write_status &= (~SPI_STATUS_WRITE);
	}
#else
	HAL_DMA_IRQHandler(&hdma_spi1_tx);
#endif
}

void spi1_dma_rxcplt_callback(void){           // DMA1 Channel 2, request 1
#if(LL_SPI)
    if(LL_DMA_IsActiveFlag_TC2(DMA1)){
        LL_DMA_ClearFlag_GI2(DMA1);
        spi_wr_dma_status &= (~SPI_STATUS_READ);
        spi_read_status &= (~SPI_STATUS_READ);
    }
#else
    HAL_DMA_IRQHandler(&hdma_spi1_rx);
#endif
}

void spi1_irq_callback(void){
#if(LL_SPI)
	uint32_t _sr = 0, _cr2 = 0;
	_sr = SPI1->SR;
	_cr2 = SPI1->CR2;

	if((_sr & SPI_SR_TXE) && (_cr2 & SPI_CR2_TXEIE)){

	}
	if(!(_sr & SPI_SR_OVR) && (_sr & SPI_SR_RXNE) && (_cr2 & SPI_CR2_RXNEIE)){

	}
	if((_cr2 & SPI_CR2_ERRIE) && ((_sr & SPI_SR_MODF) || (_sr & SPI_SR_OVR) || (_sr & SPI_SR_FRE))){
		if(_sr & SPI_SR_MODF){
            LL_SPI_ClearFlag_MODF(SPI1);
		}
		if(_sr & SPI_SR_OVR){
            LL_SPI_ClearFlag_OVR(SPI1);
		}
		if(_sr & SPI_SR_FRE){
            LL_SPI_ClearFlag_FRE(SPI1);
		}
	}
#else
	HAL_SPI_IRQHandler(&hspi1);
#endif
}

#if(LL_SPI)

#else

void hal_spi1_rxcplt_callback(void){
	spi_read_status &= (~SPI_STATUS_READ);
}

void hal_spi1_txcplt_callback(void){
	spi_write_status &= (~SPI_STATUS_WRITE);
}

#if(LL_SPI == 0)

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
    if((uint32_t)(hspi->Instance) == (uint32_t)SPI1){
    	hal_spi1_rxcplt_callback();
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
    if((uint32_t)(hspi->Instance) == (uint32_t)SPI1){
    	hal_spi1_txcplt_callback();
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    if((uint32_t)(hspi->Instance) == (uint32_t)SPI1){
    	hal_spi1_rxcplt_callback();
    	hal_spi1_txcplt_callback();
    }
}
#endif
#endif
#endif



/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
