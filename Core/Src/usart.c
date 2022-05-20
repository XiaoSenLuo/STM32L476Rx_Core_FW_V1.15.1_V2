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


/* USER CODE BEGIN 0 */
#include "stm32l4xx_it.h"
#include "usart.h"

#define USART1_DATA_BUFFER_SIZE               (512)


typedef struct uart_transmit_s {
    uint8_t data[USART1_DATA_BUFFER_SIZE];
    int16_t length;
    int16_t tail;
    int16_t head;
}uart_transmit_t;

static uart_transmit_t rx_message = {
    .data = {0},
    .length = -1,
    .head = 0,
    .tail = 0,
};

static uart_transmit_t user_rx_message = {
        .data = {0},
        .length = -1,
        .head = 0,
        .tail = 0,
};

static uart_transmit_t tx_message = {
        .data = {0},
        .length = -1,
        .head = 0,
        .tail = 0,
};


static UART_HandleTypeDef huart1 = { 0 };
static DMA_HandleTypeDef hdma_usart1_rx = {0};
static DMA_HandleTypeDef hdma_usart1_tx = {0};


static void dma_usart1_tx_isr_handler(void* ctx){
    HAL_DMA_IRQHandler((DMA_HandleTypeDef*)ctx);
}

static void dma_usart1_rx_isr_handler(void* ctx){
    HAL_DMA_IRQHandler((DMA_HandleTypeDef*)ctx);
}

static void usart1_isr_handler(void* ctx){
    UART_HandleTypeDef* handle = (UART_HandleTypeDef*)ctx;
    uint32_t uart_isr_flag = READ_REG(handle->Instance->ISR);
    if(uart_isr_flag & UART_FLAG_IDLE){   /// 空闲
        __HAL_UART_CLEAR_FLAG(handle, UART_FLAG_IDLE);
        uint32_t pos = __HAL_DMA_GET_COUNTER(handle->hdmarx);
        rx_message.tail = USART1_DATA_BUFFER_SIZE - pos;  /// 计算队尾
        rx_message.length = rx_message.tail - rx_message.head;  /// 计算队列长度, 默认队尾大于队首
        if(rx_message.length < 0){ /// 队尾小于队首
            rx_message.length = USART1_DATA_BUFFER_SIZE + rx_message.length;

        }else{

        }
        user_rx_message.length = rx_message.length;
        rx_message.head = rx_message.tail;    /// 更新队首


    }

    HAL_UART_IRQHandler((UART_HandleTypeDef*)ctx);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

    __HAL_DMA_ENABLE_IT(huart->hdmarx, DMA_IT_TC);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart){

}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart){

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){

}

int usart1_initialize(UART_HandleTypeDef* *uart_handle, uint32_t baud){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){}

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**USART1 GPIO Configuration
        PB6     ------> USART1_TX
        PB7     ------> USART1_RX
   */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_2;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    HAL_DMA_Init(&hdma_usart1_rx);
    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_2;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    HAL_DMA_Init(&hdma_usart1_tx);
    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

    ll_peripheral_isr_install(DMA1_Channel4_IRQn, dma_usart1_tx_isr_handler, huart1.hdmatx);
    ll_peripheral_isr_install(DMA1_Channel5_IRQn, dma_usart1_rx_isr_handler, huart1.hdmarx);
    ll_peripheral_isr_install(USART1_IRQn, usart1_isr_handler, &huart1);

    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = baud;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_8;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    HAL_UART_Init(&huart1);

    if(uart_handle) *uart_handle = &huart1;
}

int usart1_start_receive(UART_HandleTypeDef *uart_handle){
    uint32_t err = 0;

    UNUSED(uart_handle);

    err = HAL_UART_Receive_DMA(&huart1, rx_message.data, USART1_DATA_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    return (int)err;
}

/* USER CODE END 0 */

#if(0)
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;


#define UART_TXDMA_TC   0x20
#define UART_TXDMA_HT   0x40

#define UART_RXDMA_TC   0x02
#define UART_RXDMA_HT   0x04



static volatile uint8_t uart_idle = 0;

static __IO uint8_t uart_write_status = 0;  // bitx=0:OK, bitx=1:BUSY
static __IO uint8_t uart_read_status = 0;   // bitx=0:OK, bitx=1:BUSY
static __IO uint8_t uart3_dma_status = UART_TXDMA_TC | UART_RXDMA_TC;   // Bit[7:4]:TX, Bit[0-3]:RX

#ifndef UART3_DATA_BUFFER_SIZE
#define UART3_DATA_BUFFER_SIZE                           128
#endif

#define CMD_DMA_CIRCULAR_BUFFER_SIZE                     48

static uint8_t uart3_buffer_for_uart[UART3_DATA_BUFFER_SIZE] = {0};
static uint8_t uart3_buffer_for_user[UART3_DATA_BUFFER_SIZE] = {0};
static uint8_t cmd_dma_circular_buffer[CMD_DMA_CIRCULAR_BUFFER_SIZE] = {0};

static int cmd_sentence_start = 0, cmd_sentence_end = 0;

static uint8_t* uart3_ptr_for_user = uart3_buffer_for_user;
static uint8_t* uart3_ptr_for_uart = uart3_buffer_for_uart;
static uint8_t* cmd_ptr_for_user = uart3_buffer_for_user;
static uint8_t* cmd_ptr_for_uart = uart3_buffer_for_uart;
static int cmd_data_index = 0;
static uint8_t cmd_sentence_lock = 0, cmd_data_ready = 0;

static __IO uint8_t uart3_data_ready = 0;
static __IO uint16_t uart3_data_index = 0;
static __IO int8_t uart3_text_start = -1;
static __IO uint8_t uart3_user_reading = 0;

static void uart3_dma_circular_rx_callback(void);
static void uart3_rx_idle_callback(void);


/**
 * 命令格式:
 * 用户设置:
 *               发送: >set 变量 参数\r
 *用户获取:
 *              发送: >get 变量\r
 * 用户接受: <数据
 */

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
//	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
//	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);

//	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, DMA1_Channel2_Priority, 0);
//	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
//    LL_DMA_ClearFlag_GI2(DMA1);
	return ret;
}

static uint8_t _uart3_rx_dma_config(uint8_t *_ptr, uint16_t _s){
	uint8_t ret = 0;

	// DMA1 Channel 2; request 2
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	st_irq_handler_register(DMA1_Channel3_IRQn, uart3_dma_circular_rx_callback);

	/* (3) Configure the DMA functional parameters for transmission */
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);   // 配置参数要关闭 dma 通道
	// 循环模式
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
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_ClearFlag_GI3(DMA1);
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	return ret;
}


static void _uart_start_tx(USART_TypeDef *USARTx){
	if((uint32_t)USARTx == (uint32_t)USART3){
		if(!LL_USART_IsEnabledDMAReq_TX(USART3)) LL_USART_EnableDMAReq_TX(USART3);
		if(!LL_USART_IsEnabled(USART3)) LL_USART_Enable(USART3);
		LL_USART_ClearFlag_TC(USART3);
//		LL_DMA_ClearFlag_GI2(DMA1);
		LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
	}
}

static void _uart_error_handler(USART_TypeDef *USARTx){
	__IO uint32_t isr_reg;
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


void cmd_uart_init(uint32_t _baurate){
	LL_USART_InitTypeDef USART_InitStruct = {0};
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	LL_USART_Disable(USART3);
	LL_USART_DeInit(USART3);
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

	LL_USART_ClearFlag_ORE(USART3);

	if(!LL_USART_IsEnabledIT_ERROR(USART3)){
	    LL_USART_EnableIT_ERROR(USART3);
	}
	if(!LL_USART_IsActiveFlag_IDLE(USART3)){
		LL_USART_EnableIT_IDLE(USART3);
	}

	HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
	NVIC_EnableIRQ(USART3_IRQn);

	_uart3_rx_dma_config(cmd_dma_circular_buffer, CMD_DMA_CIRCULAR_BUFFER_SIZE);
	st_irq_handler_register(DMA1_Channel3_IRQn, uart3_dma_circular_rx_callback);
	LL_USART_ClearFlag_TC(USART3);
	LL_USART_EnableDMAReq_RX(USART3);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
}

void cmd_uart_deinit(void){
	LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_USART3);
	LL_USART_Disable(USART3);
	LL_USART_DeInit(USART3);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_5, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);
	NVIC_DisableIRQ(USART3_IRQn);
	NVIC_DisableIRQ(DMA1_Channel3_IRQn);
}

void uart_init(USART_TypeDef * in_uart, uint32_t _baurate){
	  LL_USART_InitTypeDef USART_InitStruct = {0};

	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	  LL_USART_Disable(in_uart);

	  LL_USART_DeInit(in_uart);
    if((size_t)in_uart == (size_t)USART1){

    }
      if((uint32_t)in_uart == (uint32_t)USART3){
    	  memset(uart3_buffer_for_uart, 0, sizeof(uart3_buffer_for_uart));
    	  memset(uart3_buffer_for_user, 0, sizeof(uart3_buffer_for_user));

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
#ifndef UART_GPS_DMA_RX
		if(!LL_USART_IsEnabledIT_RXNE(UART4)){
			LL_USART_EnableIT_RXNE(UART4);
		//    	LL_USART_ReceiveData8(UART4);
		}
#endif
		if(!LL_USART_IsEnabledIT_ERROR(UART4)){
			LL_USART_EnableIT_ERROR(UART4);
		}
		if(!LL_USART_IsEnabledIT_IDLE(UART4)){
			LL_USART_EnableIT_IDLE(UART4);
			LL_USART_ClearFlag_IDLE(UART4);
		}
		HAL_NVIC_SetPriority(UART4_IRQn, UART4_Priority, 0);
		NVIC_EnableIRQ(UART4_IRQn);
#ifdef UART_GPS_DMA_RX
		_uart4_rx_dma_config(gps_buffer_for_uart, GPS_DATA_BUFFER_SIZE * 2);
		LL_USART_ClearFlag_TC(UART4);
		LL_USART_EnableDMAReq_RX(UART4);
		LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_5);
#endif
    }
}

void uart_deinit(USART_TypeDef * in_uart){
	  LL_USART_Disable(in_uart);

//	  LL_USART_DeInit(in_uart);
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

/* USER CODE BEGIN 1 */


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


uint8_t UART3_Write(uint8_t* ptr, uint16_t s){
	uint8_t ret = 0;
    uint16_t _len = 0;
    uint32_t _timeout = SystemCoreClock;
#if(UART3_USE_DMA)

//    while(((uart3_dma_status & UART_TXDMA_TC) == 0) && (_timeout--));   // 等待传输完成
//    if(_timeout == 0) return 1;

    CLEAR_BIT(uart3_dma_status, UART_TXDMA_TC); // 清除标志

    if(s == 0) _len = (uint16_t)strlen((char*)ptr);
    else _len = s;
    st_irq_handler_register(DMA1_Channel2_IRQn, uart3_dma_txcplt_callback);
	ret = _uart3_tx_dma_config(ptr, _len);
	_uart_start_tx(USART3);

	while(LL_DMA_IsActiveFlag_TC2(DMA1) == RESET);
	LL_DMA_ClearFlag_GI2(DMA1);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
//	while((uart3_dma_status & UART_TXDMA_TC) == 0);
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



bool UART3_Wait_For_TC(void){
    uint32_t _timeout = SystemCoreClock;

    while(((uart3_dma_status & UART_TXDMA_TC) == 0) && (_timeout--));   // 等待传输完成

    return true;
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
    if(LL_USART_IsEnabledIT_IDLE(USART3)){
    	LL_USART_ClearFlag_IDLE(USART3);
    	uart3_rx_idle_callback();
    }
}


void uart3_dma_txcplt_callback(void){
	if(LL_DMA_IsActiveFlag_TC2(DMA1)){
		LL_DMA_ClearFlag_GI2(DMA1);
	/* Call function Transmission complete Callback */
		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
		LL_USART_DisableDMAReq_TX(USART3);

		SET_BIT(uart3_dma_status, UART_TXDMA_TC);
	}else if(LL_DMA_IsActiveFlag_TE2(DMA1)){
		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
	}
}

void cmd_uart_swap_buffer(void){
   uint8_t* tmp = NULL;

   if(cmd_sentence_lock) return;
   cmd_sentence_lock = 1;
   tmp = cmd_ptr_for_uart;
   cmd_ptr_for_uart = cmd_ptr_for_user;
   cmd_ptr_for_user = tmp;

   cmd_data_ready = 1;
   cmd_sentence_lock = 0;
}

void uart3_dma_circular_rx_callback(void){
	uint8_t* ptr = NULL;
	int i = 0;

    if(LL_DMA_IsActiveFlag_HT3(DMA1)){
        LL_DMA_ClearFlag_HT3(DMA1);
        ptr = cmd_dma_circular_buffer;
    }
    if(LL_DMA_IsActiveFlag_TC3(DMA1)){
        LL_DMA_ClearFlag_TC3(DMA1);
        LL_DMA_ClearFlag_GI3(DMA1);
        ptr = &cmd_dma_circular_buffer[(CMD_DMA_CIRCULAR_BUFFER_SIZE >> 1)];
    }
    if(LL_DMA_IsActiveFlag_TE3(DMA1)){
    	LL_DMA_ClearFlag_TE3(DMA1);

    	LL_USART_Disable(USART3);
    	_uart3_rx_dma_config(cmd_dma_circular_buffer, CMD_DMA_CIRCULAR_BUFFER_SIZE);
    	LL_USART_Enable(USART3);
    	LL_USART_ClearFlag_TC(USART3);
    	LL_USART_EnableDMAReq_RX(USART3);
    	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);  // 重新启动
    	return;
    }

    for(i = 0; i < (CMD_DMA_CIRCULAR_BUFFER_SIZE >> 1); i++){
    	if((cmd_sentence_start < 0) && (ptr[i] != '>')) continue;
    	if(ptr[i] == '>'){  // 语句开始标志
    		cmd_sentence_start = i;
    		cmd_ptr_for_uart[cmd_data_index++] = ptr[i];
    		continue;
    	}
    	if((cmd_sentence_start >= 0) && (ptr[i] == '\n')){   // 语句开始并且有结束标志
    		cmd_sentence_end = i;
    		cmd_ptr_for_uart[cmd_data_index++] = ptr[i];
    		cmd_ptr_for_uart[cmd_data_index] = '\0';

        swap_section:
    		cmd_uart_swap_buffer();  // 交换缓冲区地址

    		cmd_data_index = 0;
    		cmd_sentence_start = -1;
    		cmd_sentence_end = -1;
    		continue;
    	}
        if((cmd_sentence_start >= 0) && (cmd_sentence_end < 0)) cmd_ptr_for_uart[cmd_data_index++] = ptr[i];  // 语句开始但是没有结束标志
    }
    memset(ptr, '\0', (CMD_DMA_CIRCULAR_BUFFER_SIZE >> 1));
}

bool cmd_sentence_is_ready(void){
	return cmd_data_ready;
}

uint8_t cmd_get_sentence(uint8_t *buf, uint16_t s){
	if(cmd_sentence_lock) return 2;
	if(cmd_data_ready == 0) return 1;
	cmd_sentence_lock = 1;
	strcpy((char*)buf, (const char*)cmd_ptr_for_user);
	cmd_data_ready = 0;
	cmd_sentence_lock = 0;
	return 0;
}

void uart3_rx_idle_callback(void){  // 总线空闲
	int i = 0;
	uint32_t cnt = 0;
    uint8_t *ptr = NULL;

	cnt = CMD_DMA_CIRCULAR_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3);  // 获取已经传输长度
	if((cnt == CMD_DMA_CIRCULAR_BUFFER_SIZE) || (cnt == 0) || (cnt == (CMD_DMA_CIRCULAR_BUFFER_SIZE >> 1))) return;  // 传输完成或者没有传输或者传输一半完成
	if(cnt < (CMD_DMA_CIRCULAR_BUFFER_SIZE >> 1)) ptr = cmd_dma_circular_buffer;  // 传输小于一半
	if(cnt > (CMD_DMA_CIRCULAR_BUFFER_SIZE >> 1)){
		ptr = &cmd_dma_circular_buffer[CMD_DMA_CIRCULAR_BUFFER_SIZE >> 1]; // 传输大于一半
		cnt -= (CMD_DMA_CIRCULAR_BUFFER_SIZE >> 1); // 仅处理大于的部分(已经触发HT中断)
	}

    for(i = 0; i < cnt; i++){
    	if((cmd_sentence_start < 0) && (ptr[i] != '>')) continue;
    	if(ptr[i] == '>'){  // 语句开始标志
    		cmd_sentence_start = i;
    		cmd_ptr_for_uart[cmd_data_index++] = ptr[i];
    		continue;
    	}
    	if((cmd_sentence_start >= 0) && (ptr[i] == '\n')){   // 语句开始并且有结束标志
    		cmd_sentence_end = i;
    		cmd_ptr_for_uart[cmd_data_index++] = ptr[i];
    		cmd_ptr_for_uart[cmd_data_index] = '\0';

        swap_section:
    		cmd_uart_swap_buffer();  // 交换缓冲区地址

    		cmd_data_index = 0;
    		cmd_sentence_start = -1;
    		cmd_sentence_end = -1;
    		continue;
    	}
    	if((cmd_sentence_start >= 0) && (cmd_sentence_end < 0)){
    		cmd_ptr_for_uart[cmd_data_index++] = ptr[i];  // 语句开始但是没有结束标志
    	}
    }
    memset(ptr, '\0', cnt);
}

void uart3_busy_callbcak(void){

}

#endif


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
