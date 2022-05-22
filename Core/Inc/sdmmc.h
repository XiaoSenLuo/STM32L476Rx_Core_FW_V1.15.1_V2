/**
  ******************************************************************************
  * File Name          : SDMMC.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __sdmmc_H
#define __sdmmc_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_gpio.h"

/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */
static inline uint8_t sd_detect(void){
    return (LL_GPIO_ReadInputPort(GPIOA) & GPIO_PIN_15) ? 1 : 0;
}

/* USER CODE BEGIN Prototypes */


int sdmmc_initialize(SD_HandleTypeDef* *hsd_handle);

int sdmmc_deinitialize(SD_HandleTypeDef *hsd_handle);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ sdmmc_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
