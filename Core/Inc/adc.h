/**
  ******************************************************************************
  * File Name          : ADC.h
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
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
#ifndef __adc_H
#define __adc_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32l4xx_ll_adc.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

#ifndef ADC_USE_DMA
#define ADC_USE_DMA       0
#endif

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void st_adc_init(void);
void st_adc_deinit(void);
void st_adc_start(void);
void st_adc_stop(void);

void st_adc_wait_for_conver_cplt(void);
/* USER CODE BEGIN Prototypes */


uint32_t get_extern_analog_voltage(void);
uint16_t get_internal_voltage(void);
float get_internal_temp(void);

void st_adc1_irq_callback(void);

#if(ADC_USE_DMA)
void adc1_dma_rxcplt_callback(void);
void adc1_dma_err_callback(void);
void adc2_dma_rxcplt_callback(void);
void adc2_dma_err_callback(void);
#endif

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
