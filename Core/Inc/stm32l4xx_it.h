/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
 ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L4xx_IT_H
#define __STM32L4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void RCC_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel2_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);
void DMA1_Channel5_IRQHandler(void);
void DMA1_Channel6_IRQHandler(void);
void DMA1_Channel7_IRQHandler(void);
void ADC1_2_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM1_IRQHandler(void);
void SPI1_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void SDMMC1_IRQHandler(void);
//void UART4_IRQHandler(void);
void DMA2_Channel4_IRQHandler(void);
void DMA2_Channel3_IRQHandler(void);
void DMA2_Channel5_IRQHandler(void);
void LPTIM1_IRQHandler(void);
void LPTIM2_IRQHandler(void);
void FPU_IRQHandler(void);
/* USER CODE BEGIN EFP */
void RTC_Alarm_IRQHandler(void);
void OTG_FS_IRQHandler(void);

typedef void (*st_irq_handler_callback_t)(void);

typedef void (*isr_function_handle_t)(void* ctx);

void ll_peripheral_isr_install(IRQn_Type _irq_num, isr_function_handle_t fn, void *ctx);
void ll_peripheral_isr_uninstall(IRQn_Type _irq_num);
void ll_gpio_exti_isr_install(int gpio, isr_function_handle_t fn, void *ctx);
void ll_gpio_exti_isr_uninstall(int gpio);


void st_irq_handler_register(IRQn_Type _irq_num, void *_handler);
void st_exti_irq_handler_register(IRQn_Type _irq_num, uint32_t _exti_line, void *_handler);


/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32L4xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
