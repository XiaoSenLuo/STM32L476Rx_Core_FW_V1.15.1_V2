/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for
  *                      the gpio
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
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define IO_ADS1_CS_PORT                       GPIOA
#define IO_ADS1_CS_PIN                        LL_GPIO_PIN_4
#define IO_ADS1_DRDY_PORT                      GPIOB
#define IO_ADS1_DRDY_PIN                       LL_GPIO_PIN_1

/* USER CODE END Private defines */

#if(0)
void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

void gpio_pa4_change_mode(uint32_t in_mode);

void optimize_gpio_power(GPIO_TypeDef *in_gpio, uint32_t in_pin);

#endif

void gpio_all_set_analog(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
