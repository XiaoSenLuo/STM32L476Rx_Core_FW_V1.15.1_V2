/**
  ******************************************************************************
  * File Name          : I2C.h
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
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
#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/


/* USER CODE BEGIN Includes */
//#include "stdint.h"
//#include "stddef.h"
#include "stm32l4xx_ll_i2c.h"

/* USER CODE END Includes */

//extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

//void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */


void i2c_init(I2C_TypeDef* in_i2c);
void i2c_deinit(I2C_TypeDef* in_i2c);

uint8_t I2C2_Write(uint8_t in_address, uint8_t in_reg, uint8_t* in_ptr, uint16_t in_size);
uint8_t I2C2_Read(uint8_t in_address, uint8_t in_reg, uint8_t* out_ptr, uint16_t in_size);

/**
 *     仅支持7位地址
 * @param in_i2c
 * @param out_address
 * @param out_device_num
 * @return
 */
uint8_t I2C_Find_Address(I2C_TypeDef* in_i2c, uint8_t *out_address, uint8_t *out_device_num);

//void i2c2_dma_txcplt_callback(void);
//void i2c2_dma_rxcplt_callback(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ i2c_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
