/**
  ******************************************************************************
  * File Name          : LPTIM.h
  * Description        : This file provides code for the configuration
  *                      of the LPTIM instances.
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
#ifndef __lptim_H
#define __lptim_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/


/* USER CODE BEGIN Includes */
#include "stm32l4xx_ll_lptim.h"
/* USER CODE END Includes */

extern LPTIM_HandleTypeDef hlptim1;
extern LPTIM_HandleTypeDef hlptim2;

/* USER CODE BEGIN Private defines */




/* USER CODE END Private defines */

void MX_LPTIM1_Init(void);
void MX_LPTIM2_Init(void);

/* USER CODE BEGIN Prototypes */

void lptim_init(LPTIM_TypeDef *in_lptim);
void lptim_deinit(LPTIM_TypeDef *in_lptim);
void lptim_start(LPTIM_TypeDef *in_lptim);
void lptim_stop(LPTIM_TypeDef *in_lptim);
void lptim_set_freq(LPTIM_TypeDef *in_lptim, uint32_t in_freq_mhz);
uint32_t lptim_get_freq(LPTIM_TypeDef *in_lptim);
void lptim_set_duty(LPTIM_TypeDef *in_lptim, uint8_t in_duty);

void lptim2_use_for_ads_clk(uint32_t in_clk_freq);

//void LPTIM1_IRQHandler(void);
//void LPTIM2_IRQHandler(void);

void lptim1_irq_callback(void);
void lptim2_irq_callback(void);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ lptim_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
