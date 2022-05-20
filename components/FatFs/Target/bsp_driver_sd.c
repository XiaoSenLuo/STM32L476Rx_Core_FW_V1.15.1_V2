/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    bsp_driver_sd.c for L4 (based on stm32l4r9i_eval_sd.c)
 * @brief   This file includes a generic uSD card driver.
 *          To be completed by the user according to the board used for the project.
 * @note    Some functions generated as weak: they can be overriden by
 *          - code in user files
 *          - or BSP code from the FW pack files
 *          if such files are added to the generated project (by the user).
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
/* USER CODE END Header */



#if(1)

#else

#include "bsp_driver_sd.h"
#include "sdmmc.h"

SD_HandleTypeDef *hsd_handle = NULL;

int sd_initialize(uint8_t lun){

}


int sd_status(uint8_t lun){

}


int sd_read(uint8_t lun, uint8_t *buff, uint64_t sector, uint32_t count){

}


int sd_write(uint8_t lun, const uint8_t *buff, uint64_t sector, uint32_t count){

}


int sd_ioctl(uint8_t lun, uint8_t cmd, void *buff){

}



#endif





/* USER CODE END AdditionalCode */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
