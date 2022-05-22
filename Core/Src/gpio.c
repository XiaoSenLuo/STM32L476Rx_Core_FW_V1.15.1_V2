/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN 0 */
#include "stm32l4xx_hal.h"
#include "gpio.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

void gpio_power_optimizer(GPIO_TypeDef *port, int32_t pin_num){
    GPIO_InitTypeDef GPIO_InitStructure = {
            .Alternate = 0,
            .Mode = GPIO_MODE_ANALOG,
            .Pin = 0x1UL << pin_num,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_LOW,
    };
    if((pin_num < 0) || (pin_num > 16)) return;

    HAL_GPIO_Init(port, &GPIO_InitStructure);
}

/* USER CODE BEGIN 2 */
void gpio_all_set_analog(void){
    GPIO_InitTypeDef GPIO_InitStructure = {
            .Alternate = 0,
            .Mode = GPIO_MODE_ANALOG,
            .Pin = 0x1FFD,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_LOW,
    };

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = 0xFFE7;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = 0x3FFF;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void gpio_output_initialize(void){
    GPIO_InitTypeDef GPIO_InitStructure = {
            .Alternate = 0,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pin = 1UL << IO_BZ_PIN,
            .Pull = GPIO_PULLDOWN,
            .Speed = GPIO_SPEED_LOW,
    };
    HAL_GPIO_Init(IO_BZ_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = 1UL << IO_TS_PIN;
    HAL_GPIO_Init(IO_TS_PORT, &GPIO_InitStructure);
}

void gpio_input_initialize(void){
    GPIO_InitTypeDef GPIO_InitStructure = {
            .Alternate = 0,
            .Mode = GPIO_MODE_INPUT,
            .Pin = 1UL << IO_DEC5V_PIN,
            .Pull = GPIO_PULLDOWN,
            .Speed = GPIO_SPEED_LOW,
    };
    HAL_GPIO_Init(IO_DEC5V_PORT, &GPIO_InitStructure);
}

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
