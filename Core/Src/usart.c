/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "main.h"
/* USER CODE END 0 */

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

#define UART3_IDLE                  0x08
#define UART4_IDLE                  0x10

static volatile uint8_t uart_idle = 0;


void uart_init(USART_TypeDef * in_uart, uint32_t _baurate){
	  LL_USART_InitTypeDef USART_InitStruct = {0};

	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

      if((uint32_t)in_uart == (uint32_t)USART3){
    	LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_PCLK1);   // 设置时钟源
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
//		  /**USART3 GPIO Configuration
//		  PC4   ------> USART3_TX
//		  PC5   ------> USART3_RX
//		  */
		  GPIO_InitStruct.Pin = LL_GPIO_PIN_4 | LL_GPIO_PIN_5;
		  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
		  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
		  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		  USART_InitStruct.BaudRate = _baurate;
		  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
		  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
		  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
		  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
		  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
		  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_8;
		  LL_USART_Init(USART3, &USART_InitStruct);

		  LL_USART_ConfigAsyncMode(USART3);
		  LL_USART_Enable(USART3);
		  /* Polling USART initialisation */
		  while((!(LL_USART_IsActiveFlag_TEACK(USART3))) || (!(LL_USART_IsActiveFlag_REACK(USART3)))){};
        HAL_NVIC_SetPriority(USART3_IRQn, USART3_Priority, 0);
        NVIC_EnableIRQ(USART3_IRQn);
	    LL_USART_ClearFlag_ORE(USART3);
	    if(!LL_USART_IsEnabledIT_RXNE(USART3)){
	    	LL_USART_EnableIT_RXNE(USART3);
	    }
	    if(!LL_USART_IsEnabledIT_ERROR(USART3)){
	    	LL_USART_EnableIT_ERROR(USART3);
	    }
    }
    if((uint32_t)in_uart == (uint32_t)UART4){
    	LL_RCC_SetUARTClockSource(LL_RCC_UART4_CLKSOURCE_PCLK1);
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	    /**UART4 GPIO Configuration
	    PA0     ------> UART4_TX
	    PA1     ------> UART4_RX
	    */
		  GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
		  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
		  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
		  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		  USART_InitStruct.BaudRate = _baurate;
		  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
		  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
		  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
		  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
		  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
		  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
		  LL_USART_Init(UART4, &USART_InitStruct);

		  LL_USART_ConfigAsyncMode(UART4);
		  LL_USART_Enable(UART4);
		  /* Polling USART initialisation */
		  while((!(LL_USART_IsActiveFlag_TEACK(UART4))) || (!(LL_USART_IsActiveFlag_REACK(UART4)))){};
		if(!LL_USART_IsEnabledIT_RXNE(UART4)){
			LL_USART_EnableIT_RXNE(UART4);
		//    	LL_USART_ReceiveData8(UART4);
		}
		if(!LL_USART_IsEnabledIT_ERROR(UART4)){
			LL_USART_EnableIT_ERROR(UART4);
		}
		HAL_NVIC_SetPriority(UART4_IRQn, UART4_Priority, 0);
		NVIC_EnableIRQ(UART4_IRQn);
    }
}

/* USER CODE BEGIN 1 */

void uart_stop(USART_TypeDef * in_uart){
    if((uint32_t)in_uart == (uint32_t)USART3){
    	LL_USART_DeInit(USART3);
    	LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_USART3);
    	LL_USART_Disable(USART3);
    	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
    	LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);
    	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_5, LL_GPIO_MODE_ANALOG);
    	LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);
    	NVIC_DisableIRQ(USART3_IRQn);
    }
    if((uint32_t)in_uart == (uint32_t)UART4){
    	LL_USART_DeInit(UART4);
    	LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_UART4);
    	LL_USART_Disable(UART4);
    	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ANALOG);
    	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);
    	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ANALOG);
    	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);
    	NVIC_DisableIRQ(UART4_IRQn);
    }
}

uint8_t uart_is_idle(USART_TypeDef* in_uart){
    return (uart_idle & UART3_IDLE);
}

#ifndef UART_TRANSFER_USE_DMA
#define UART_TRANSFER_USE_DMA      0
#endif


static __IO uint8_t uart_write_status = 0;  // bitx=0:OK, bitx=1:BUSY
static __IO uint8_t uart_read_status = 0;   // bitx=0:OK, bitx=1:BUSY
static __IO uint8_t uart_dma_write_status = 0;
static __IO uint8_t uart_readable = 0;      //
static __IO uint8_t uart_user_reading = 0;  // bitx=0:no read, bitx=1:reading
static __IO uint8_t* uart_rx_ptr = NULL;
static __IO uint16_t uart_rx_len = 0;
static __IO uint16_t uart_rx_len_index = 0;


#ifndef UART3_DATA_BUFFER_SIZE
#define UART3_DATA_BUFFER_SIZE                           128
#endif
/**
 * 命令格式:
 * 用户设置:
 *               发送: >set 变量 参数\r
 *用户获取:
 *              发送: >get 变量\r
 * 用户接受: <数据
 */
static uint8_t uart3_buffer_for_uart[UART3_DATA_BUFFER_SIZE];
static uint8_t uart3_buffer_for_user[UART3_DATA_BUFFER_SIZE];
static uint8_t* uart3_ptr_for_user = uart3_buffer_for_user;
static uint8_t* uart3_ptr_for_uart = uart3_buffer_for_uart;
static __IO uint8_t uart3_data_ready = 0;
static __IO uint16_t uart3_data_index = 0;
static __IO int8_t uart3_text_start = -1;
static __IO uint8_t uart3_user_reading = 0;

static uint8_t _uart_write(UART_HandleTypeDef* uart_handle, uint8_t* _ptr, uint16_t _s){
	uint32_t _ret = HAL_OK;

	_ret = HAL_UART_Transmit_IT(uart_handle, _ptr, _s);
//	_ret = HAL_UART_Transmit(uart_handle, _ptr, _s, 100000);
	if(_ret != HAL_OK){
		return 1;
	}else{
		return 0;
	}
}

static uint8_t _uart_read(UART_HandleTypeDef* uart_handle, uint8_t* _ptr, uint16_t _s){
	uint32_t _ret = HAL_OK;

	_ret = HAL_UART_Receive_IT(uart_handle, _ptr, _s);
//	_ret = HAL_UART_Receive(uart_handle, _ptr, _s, 0xFFFFFF);

	if(_ret != HAL_OK){
		return 1;
	}else{
		return 0;
	}
}

static uint8_t _uart3_tx_dma_config(uint8_t *_ptr, uint16_t _s){
	uint8_t ret = 0;

	// DMA1 Channel 2; request 2
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, DMA1_Channel2_Priority, 0);
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	/* (3) Configure the DMA functional parameters for transmission */
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);   // 配置参数要关闭 dma 通道
	LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_2,
						LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
						LL_DMA_PRIORITY_HIGH              |
						LL_DMA_MODE_NORMAL                |
						LL_DMA_PERIPH_NOINCREMENT         |
						LL_DMA_MEMORY_INCREMENT           |
						LL_DMA_PDATAALIGN_BYTE            |
						LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2,
						 (uint32_t)_ptr,
						 LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_TRANSMIT),
						 LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, _s);
	if(LL_DMA_GetPeriphRequest(DMA1, LL_DMA_CHANNEL_2) != LL_DMA_REQUEST_2)
		LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMA_REQUEST_2);
	/* (5) Enable DMA transfer complete/error interrupts  */
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);

	return ret;
}

static uint8_t _uart3_rx_dma_config(uint8_t *_ptr, uint16_t _s){
	uint8_t ret = 0;

	// DMA1 Channel 2; request 2
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, DMA1_Channel3_Priority, 0);
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	/* (3) Configure the DMA functional parameters for transmission */
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);   // 配置参数要关闭 dma 通道
	LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_3,
						LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
						LL_DMA_PRIORITY_HIGH              |
						LL_DMA_MODE_CIRCULAR              |
						LL_DMA_PERIPH_NOINCREMENT         |
						LL_DMA_MEMORY_INCREMENT           |
						LL_DMA_PDATAALIGN_BYTE            |
						LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3,
			             LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_RECEIVE),
						 (uint32_t)_ptr,
						 LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, _s);
	if(LL_DMA_GetPeriphRequest(DMA1, LL_DMA_CHANNEL_3) != LL_DMA_REQUEST_2)
		LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_2);
	/* (5) Enable DMA transfer complete/error interrupts  */
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);

	return ret;
}

static uint8_t _uart4_tx_dma_config(uint8_t *_ptr, uint16_t _s){
	uint8_t ret = 0;

	// DMA2 Channel 3; request 2
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
	HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, DMA2_Channel3_Priority, 0);
	NVIC_EnableIRQ(DMA2_Channel3_IRQn);
	/* (3) Configure the DMA functional parameters for transmission */
	LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_3);   // 配置参数要关闭 dma 通道
	LL_DMA_ConfigTransfer(DMA2, LL_DMA_CHANNEL_3,
						LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
						LL_DMA_PRIORITY_HIGH              |
						LL_DMA_MODE_NORMAL                |
						LL_DMA_PERIPH_NOINCREMENT         |
						LL_DMA_MEMORY_INCREMENT           |
						LL_DMA_PDATAALIGN_BYTE            |
						LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_3,
						 (uint32_t)_ptr,
						 LL_USART_DMA_GetRegAddr(UART4, LL_USART_DMA_REG_DATA_TRANSMIT),
						 LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_3, _s);
	if(LL_DMA_GetPeriphRequest(DMA1, LL_DMA_CHANNEL_3) != LL_DMA_REQUEST_2)
		LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_2);
	/* (5) Enable DMA transfer complete/error interrupts  */
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_3);
	LL_DMA_EnableIT_TE(DMA2, LL_DMA_CHANNEL_3);
	return ret;
}


static uint8_t _ll_uart_read_by_dma(USART_TypeDef *USARTx, uint8_t *_ptr, uint16_t _s){

	return 1;
}


uint8_t UART1_Write(uint8_t* ptr, uint16_t s){
    uint8_t ret = 0;
    uart_write_status |= 0x01;   // start transfer
    st_irq_handler_register(USART1_IRQn, NULL);
	ret = _uart_write(&huart1, ptr, s);
	if(ret == 0) while((uart_write_status & 0x01) != 0);
    return ret;
}

uint8_t UART1_Read(uint8_t* ptr, uint16_t s){
    uint8_t ret = 0;
    uart_read_status |= 0x01;   // start transfer
    st_irq_handler_register(USART1_IRQn, NULL);
	ret = _uart_read(&huart1, ptr, s);
	if(ret == 0) while((uart_read_status & 0x01) != 0);
    return ret;
}

static void _uart_start_tx(USART_TypeDef *USARTx){
	if((uint32_t)USARTx == (uint32_t)USART3){
		LL_USART_EnableDMAReq_TX(USART3);
		if(LL_USART_IsEnabled(USART3) == 0) LL_USART_Enable(USART3);
		LL_USART_ClearFlag_TC(USART3);
		LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
	}
	if((uint32_t)USARTx == (uint32_t)UART4){
		LL_USART_ClearFlag_TC(UART4);
		LL_USART_EnableDMAReq_TX(UART4);
		LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_3);
	}
}

uint8_t UART3_Write(uint8_t* ptr, uint16_t s){
	uint8_t ret = 0;
    uint16_t _len = 0;
    uint32_t _timeout = 8000000;
#if(UART3_USE_DMA)
    uart_dma_write_status |= 0x04;
    if(s == 0) _len = (uint16_t)strlen((char*)ptr);
    else _len = s;
    st_irq_handler_register(DMA1_Channel2_IRQn, uart3_dma_txcplt_callback);
	ret = _uart3_tx_dma_config(ptr, _len);
	_uart_start_tx(USART3);

	while(((uart_dma_write_status & 0x04) != 0) && (_timeout--));   // 等待传输完成
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
	LL_USART_DisableDMAReq_TX(USART3);
#else
    uart_write_status |= 0x04;   // start transfer

	ret = _uart_write(&huart3, ptr, s);
	if(ret == 0) while((uart_write_status & 0x04) != 0);
#endif
	return ret;
}

uint8_t UART3_Read(uint8_t* ptr, uint16_t s){
    size_t len = 0;
    uint8_t ret = 0;

#if(HAL_UART)
	uart_read_status |= 0x04;   // start transfer
    ret = _uart_read(&huart3, ptr, s);
    if(ret == 0) while((uart_read_status & 0x04) != 0);
#else
//    NVIC_EnableIRQ(USART3_IRQn);
//    LL_USART_ClearFlag_ORE(USART3);
    if(!LL_USART_IsEnabledIT_RXNE(USART3)){
    	LL_USART_EnableIT_RXNE(USART3);
//    	LL_USART_ReceiveData8(USART3);
    }
    if(uart3_data_ready){
    	uart3_user_reading = 1;
    	len = strlen((char*)uart3_ptr_for_user);
    	if(s < len + 1) ret = 255;
    	memcpy(ptr, uart3_ptr_for_user, len + 1);
    	uart3_data_ready = 0;
    	uart3_user_reading = 0;
    }else{
    	ret = 1;
    }
#endif
	return ret;
}

uint8_t UART4_Write(uint8_t* ptr, uint16_t s){
	uint8_t ret = 0;
	uint16_t _len = 0;
#if(UART4_USE_DMA)
	uart_dma_write_status |= 0x08;
	if(s == 0) _len = (uint16_t)strlen((char*)ptr);
	else _len = s;
    st_irq_handler_register(DMA2_Channel3_IRQn, uart4_dma_txcplt_callback);
	ret = _uart4_tx_dma_config(ptr, _len);
	_uart_start_tx(UART4);

	while((uart_dma_write_status & 0x08) != 0);   // 等待传输完成
	LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_3);
	LL_USART_DisableDMAReq_TX(UART4);
#else
	uint16_t start_tick = 0x0000ffff;
	uart_write_status |= 0x08;   // start transfer
    ret = _uart_write(&huart4, ptr, s);
    if(ret == 0) while(((uart_write_status & 0x08) !=  0) && ((start_tick--) > 0));
    if((uart_write_status & 0x08) && (!start_tick)) ret = 2;
#endif
    return ret;
}

uint8_t UART4_Read(uint8_t* ptr, uint16_t s){
	uint8_t ret = 0;
	uint16_t start_tick = 0x0000ffff;

#if(HAL_UART)
	uart_read_status |= 0x08;   // start transfer
    ret = _uart_read(&huart4, ptr, s);
    if(ret == 0) while((uart_read_status & 0x08) != 0);
#else
    if(((uart_read_status & 0x08) == 0) && (uart_rx_ptr == NULL)){
        uart_rx_ptr = ptr;
        uart_rx_len = s;
        uart_rx_len_index = 0;
        uart_read_status |= 0x08;
//        HAL_NVIC_SetPriority(UART4_IRQn, UART4_Priority, 0);
//        HAL_NVIC_EnableIRQ(UART4_IRQn);
        LL_USART_ClearFlag_ORE(UART4);
        start_tick = 0x0000FFFF;
        if(!LL_USART_IsEnabledIT_RXNE(UART4)){
        	LL_USART_EnableIT_RXNE(UART4);
    //    	LL_USART_ReceiveData8(UART4);
        }
        if(!LL_USART_IsEnabledIT_ERROR(UART4)){
        	LL_USART_EnableIT_ERROR(UART4);
        }
        while(((uart_read_status & 0x08) != 0) && ((start_tick--) > 0));
        if((uart_read_status & 0x08) && (!start_tick)){
        	ret = 2;
        }else{
        	ret = 0;
        }
        uart_rx_ptr = NULL;
        uart_rx_len_index = 0;
//		LL_USART_DisableIT_RXNE(UART4);
//		LL_USART_DisableIT_ERROR(UART4);
    }else{
    	ret = 1;
    }

#endif
    return ret;
}


#ifdef GPS_RECEIVE
#define GPS_DATA_BUFFER_SIZE           80

static uint8_t gps_buffer_for_uart[GPS_DATA_BUFFER_SIZE];
static uint8_t gps_buffer_for_user[GPS_DATA_BUFFER_SIZE];
static uint8_t* gps_ptr_for_user = gps_buffer_for_user;
static uint8_t* gps_ptr_for_uart = gps_buffer_for_uart;
static __IO uint8_t gps_data_ready = 0;
static __IO uint16_t gps_data_index = 0;
static __IO int8_t gps_text_start = -1;
static __IO uint8_t gps_user_reading = 0;

uint8_t UART_GPS_Receice(uint8_t* ptr, uint8_t s){
	size_t len = 0;
    uint8_t ret = 0;

    if(!LL_USART_IsEnabledIT_RXNE(UART4)){
    	LL_USART_EnableIT_RXNE(UART4);
//    	LL_USART_ReceiveData8(UART4);
    }
    if(!LL_USART_IsEnabledIT_ERROR(UART4)){
    	LL_USART_EnableIT_ERROR(UART4);
    }
    if(gps_data_ready){
    	gps_user_reading = 1;
    	len = strlen((char*)gps_ptr_for_user);
    	if(s < len + 1){
    		ret = 255;
    	}

        memcpy(ptr, gps_ptr_for_user, len + 1);
        gps_data_ready = 0;
        gps_user_reading = 0;
    }else{
    	ret = 1;
    }
    return ret;
}
#endif


static void _uart_error_handler(USART_TypeDef *USARTx){
	__IO uint32_t isr_reg;
	if(USARTx == USART3){
//		NVIC_DisableIRQ(USART3_IRQn);
	}else if(USARTx == UART4){
//		NVIC_DisableIRQ(UART4_IRQn);
	}
	isr_reg = LL_USART_ReadReg(USARTx, ISR);
	if(isr_reg & LL_USART_ISR_NE){
		LL_USART_ClearFlag_NE(USARTx);
	}else if(isr_reg & LL_USART_ISR_ORE){
		LL_USART_ClearFlag_ORE(USARTx);   // 清除 overrun 标志位
		LL_USART_ReceiveData8(USARTx);    // 丢弃未读数据
	}else if(isr_reg & LL_USART_ISR_FE){
        LL_USART_ClearFlag_FE(USARTx);
	}
}

void USART1_IRQHandler(void){
	HAL_UART_IRQHandler(&huart1);

    LL_USART_ClearFlag_ORE(USART1);
    LL_USART_EnableIT_RXNE(USART1);
}

void uart3_cmd_rx_callback(void){
    register uint32_t tmp_rx = '\0';
    uint8_t* tmp_ptr = NULL;

	tmp_rx = LL_USART_ReceiveData8(USART3);
    if(tmp_rx == CMD_TX_SIGN){  // '>'
    	uart3_text_start = 1;
    }
    if(tmp_rx == CMD_LF){ // '\n' 行尾
    	if(uart3_ptr_for_uart[uart3_data_index - 1] == CMD_CR){
    		uart3_data_index -= 1;
    		uart3_text_start = 0;
    	}else{
uart3_exti_ti:
    		uart3_text_start = -1;
    		uart3_data_index = 0;
//        		LL_USART_EnableIT_RXNE(USART3);
    		return; // 不是以 '\r\n' 结尾的命令, 退出
    	}
    }
    if(uart3_text_start == 1){
    	uart3_ptr_for_uart[uart3_data_index] = (uint8_t)tmp_rx;
    	uart3_data_index += 1;
    	if((UART3_DATA_BUFFER_SIZE - 2) < uart3_data_index){  // 缓冲区满
    		if((uart3_ptr_for_uart[uart3_data_index] == CMD_LF)
    				&& (uart3_ptr_for_uart[uart3_data_index - 1] == CMD_CR)){
    			uart3_text_start = 0;
    		}else{
                goto uart3_exti_ti;
    		}

    	}
    }
    if((uart3_text_start == 0)){    // 命令结束
    	uart3_ptr_for_uart[uart3_data_index] = '\0';
    	if(uart3_user_reading == 0){
    		tmp_ptr = uart3_ptr_for_user;
    		uart3_ptr_for_user = uart3_ptr_for_uart;
    		uart3_ptr_for_uart = tmp_ptr;
    		uart3_data_ready = 1;
    		uart3_text_start = -1;
    	}else{   // 用户正在读取数据, 不更新数据
    		uart3_text_start = -1;
    	}
    	uart3_data_index = 0;
    }
}


void USART3_IRQHandler(void){
    if(LL_USART_IsEnabledIT_RXNE(USART3) && LL_USART_IsActiveFlag_RXNE(USART3)){
        uart3_cmd_rx_callback();
    }else{
    	_uart_error_handler(USART3);
    }
}

void uart4_gps_rx_callback(void){
    register uint8_t tmp_rx = 0;
    uint8_t* tmp_ptr = NULL;

    tmp_rx = LL_USART_ReceiveData8(UART4);
    if(tmp_rx == '$') gps_text_start = 1;
    if(tmp_rx == CMD_CR) gps_text_start = 0;
    if(gps_text_start == 1){  // start
        gps_ptr_for_uart[gps_data_index] = (uint8_t)tmp_rx;
        gps_data_index += 1;
    }else if(gps_text_start == 0){   // end
    	gps_ptr_for_uart[gps_data_index] = (uint8_t)tmp_rx;
    	gps_ptr_for_uart[gps_data_index + 1] = '\n';
    	gps_ptr_for_uart[gps_data_index + 2] = '\0';
    	if(gps_user_reading == 0){
            tmp_ptr = gps_ptr_for_user;
            gps_ptr_for_user = gps_ptr_for_uart;
            gps_ptr_for_uart = tmp_ptr;
            gps_data_ready = 1;
            gps_text_start = -1;
    	}else{
    		gps_text_start = -1;
    	}
        gps_data_index = 0;
    }else{

    }
}

void UART4_IRQHandler(void){

	if(LL_USART_IsActiveFlag_RXNE(UART4) && LL_USART_IsEnabledIT_RXNE(UART4)){
		uart4_gps_rx_callback();
	}else{
//		HAL_UART_IRQHandler(&huart4);
		_uart_error_handler(UART4);
	}
//	LL_USART_EnableIT_RXNE(UART4);
}

#if(HAL_UART)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

	if(huart->Instance == USART1){
		uart_write_status &= (~0x01);
	}else if(huart->Instance == USART2){
		uart_write_status &= (~0x02);
	}else if(huart->Instance == USART3){
		uart_write_status &= (~0x04);
	}else if(huart->Instance == UART4){
		uart_write_status &= (~0x08);
	}else if(huart->Instance == UART5){
		uart_write_status &= (~0x10);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart->Instance == USART1){
		uart_read_status &= (~0x01);
	}else if(huart->Instance == USART2){
		uart_read_status &= (~0x02);
	}else if(huart->Instance == USART3){
		uart_read_status &= (~0x04);
	}else if(huart->Instance == UART4){
		uart_read_status &= (~0x08);
	}else if(huart->Instance == UART5){
		uart_read_status &= (~0x10);
	}
}
#endif

void uart3_dma_txcplt_callback(void){
		if(LL_DMA_IsActiveFlag_TC2(DMA1)){
		    LL_DMA_ClearFlag_GI2(DMA1);
		/* Call function Transmission complete Callback */
	        uart_dma_write_status &= (~0x04);
		}else if(LL_DMA_IsActiveFlag_TE2(DMA1)){
			LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
		}
}

void uart3_dma_rxcplt_callback(void){

}

void uart3_idle_callback(void){
    uart_idle |= UART3_IDLE;
}

void uart3_busy_callbcak(void){
	uart_idle &= (~UART3_IDLE);
}

//void UART4_DMA_Handler(void){
void uart4_dma_txcplt_callback(void){
	if(LL_DMA_IsActiveFlag_TC3(DMA2)){
	    LL_DMA_ClearFlag_GI3(DMA2);
	/* Call function Transmission complete Callback */
	    uart_dma_write_status &= (~0x08);
	}else if(LL_DMA_IsActiveFlag_TE3(DMA2)){
		LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_3);
	}

}



/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
