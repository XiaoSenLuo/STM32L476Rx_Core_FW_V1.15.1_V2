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
#include "stm32l4xx_ll_gpio.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define PIN_MASK(pin_num)     (1UL << (pin_num))

#define IO_ADS_CS_PORT                       GPIOA
#define IO_ADS_CS_PIN                        (4)

#define IO_ADS_DRDY_PORT                     GPIOB
#define IO_ADS_DRDY_PIN                      (1)

#define IO_ADS_START_PORT                     GPIOC
#define IO_ADS_START_PIN                      (5)

#define IO_ADS_RESET_PORT                     GPIOC
#define IO_ADS_RESET_PIN                      (4)

#define IO_DET_CMD_PORT                         GPIOB
#define IO_DET_CMD_PIN                          (3)

#define IO_BZ_PORT                            GPIOA
#define IO_BZ_PIN                             (1)

#define IO_TS_PORT                            GPIOB
#define IO_TS_PIN                             (4)

#define IO_PPS_PORT                           GPIOA
#define IO_PPS_PIN                            (0)

#define IO_SD_DET_PORT                       GPIOA
#define IO_SD_DET_PIN                        (15)


/* USER CODE END Private defines */



void gpio_power_optimizer(GPIO_TypeDef *port, int32_t pin_num);


void gpio_all_set_analog(void);

void gpio_output_initialize(void);

void gpio_input_initialize(void);


static uint8_t inline gpio_get_pps_level(void){
    return ((LL_GPIO_ReadInputPort(IO_PPS_PORT) & PIN_MASK(IO_PPS_PIN)) ? 1 : 0);
}

static inline uint8_t gpio_cmd_detect(void){
    return ((LL_GPIO_ReadInputPort(IO_DET_CMD_PORT) & PIN_MASK(IO_DET_CMD_PIN)) ? 1 : 0);
}

static inline void gpio_buzzer_on(void){
    LL_GPIO_SetOutputPin(IO_BZ_PORT, PIN_MASK(IO_BZ_PIN));
}

static inline void gpio_buzzer_off(void){
    LL_GPIO_ResetOutputPin(IO_BZ_PORT, PIN_MASK(IO_BZ_PIN));
}

static inline void gpio_buzzer_toggle(void){
    LL_GPIO_TogglePin(IO_BZ_PORT, PIN_MASK(IO_BZ_PIN));
}

static inline void gpio_ts3a_mcu(void){
    LL_GPIO_ResetOutputPin(IO_TS_PORT, PIN_MASK(IO_TS_PIN));
}

static inline void gpio_ts3a_usb(void){
    LL_GPIO_SetOutputPin(IO_TS_PORT, PIN_MASK(IO_TS_PIN));
}

static inline uint8_t gpio_sd_detect(void){
    return ((LL_GPIO_ReadInputPort(IO_SD_DET_PORT) & PIN_MASK(IO_SD_DET_PIN)) ? 1 : 0);
}

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
