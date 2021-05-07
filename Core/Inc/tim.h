/**
  ******************************************************************************
  * File Name          : TIM.h
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#ifndef __tim_H
#define __tim_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/


/* USER CODE BEGIN Includes */
#include "stm32l4xx_ll_tim.h"
/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void tim_init(TIM_TypeDef *in_tim_index);
void tim_deinit(TIM_TypeDef *in_tim_index);

void tim_start(TIM_TypeDef *in_tim_index);
void tim_stop(TIM_TypeDef *in_tim_index);

void tim1_use_for_ads_clk(uint32_t in_clk_freq);

/**
 * in_channel: no use
 */
uint8_t tim_set_output_freq(TIM_TypeDef *in_tim, uint8_t in_channel_no_use, uint32_t in_freq_hz);


uint8_t tim_set_output_duty(TIM_TypeDef *in_tim, uint8_t in_channel, uint32_t in_duty);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
