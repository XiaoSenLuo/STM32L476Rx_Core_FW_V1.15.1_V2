/**
  ******************************************************************************
  * File Name          : USART.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/


/* USER CODE BEGIN Includes */
#include "stm32l4xx_hal.h"

/* USER CODE END Includes */


int usart1_initialize(UART_HandleTypeDef* *uart_handle, uint32_t baud);

int usart1_start_receive(UART_HandleTypeDef *uart_handle);

#if(0)
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
#define HAL_UART             0
#define UART3_USE_DMA        1
#define UART4_USE_DMA        1
/* USER CODE END Private defines */

//void MX_UART4_Init(void);
//void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

void USART1_IRQHandler(void);
void UART4_IRQHandler(void);
void USART3_IRQHandler(void);

void uart_init(USART_TypeDef * in_uart, uint32_t _baurate);
void uart_deinit(USART_TypeDef * in_uart);

void uart3_dma_txcplt_callback(void);
void uart3_dma_rxcplt_callback(void);
void uart3_busy_callbcak(void);

void uart3_cmd_rx_callback(void);

uint8_t UART1_Write(uint8_t* ptr, uint16_t s);
uint8_t UART1_Read(uint8_t* ptr, uint16_t s);

uint8_t UART3_Write(uint8_t* ptr, uint16_t s);
bool UART3_Wait_For_TC(void);
uint8_t UART3_Read(uint8_t* ptr, uint16_t s);

uint8_t UART4_Write(uint8_t* ptr, uint16_t s);
uint8_t UART4_Read(uint8_t* ptr, uint16_t s);

void cmd_uart_init(uint32_t _baurate);
void cmd_uart_deinit(void);

bool cmd_sentence_is_ready(void);
uint8_t cmd_get_sentence(uint8_t *buf, uint16_t s);
#endif


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
