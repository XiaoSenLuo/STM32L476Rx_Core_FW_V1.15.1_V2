/**
 ******************************************************************************
  * @file    bsp_driver_sd.h (based on stm32l4r9i_eval_sd.h)
  * @brief   This file contains the common defines and functions prototypes for
  *          the bsp_driver_sd.c driver.
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
#ifndef __STM32L4_SD_H
#define __STM32L4_SD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

#if(1)

#else

#include "stdint.h"
#include "sdmmc.h"

int sd_initialize(uint8_t lun);
int sd_status(uint8_t lun);
int sd_read(uint8_t lun, uint8_t *buff, uint64_t sector, uint32_t count);
int sd_write(uint8_t lun, const uint8_t *buff, uint64_t sector, uint32_t count);
int sd_ioctl(uint8_t lun, uint8_t cmd, void *buff);

#endif

/* USER CODE END CallBacksSection_H */
#ifdef __cplusplus
}
#endif

#endif /* __STM32L4_SD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
