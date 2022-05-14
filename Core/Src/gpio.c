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
#include "gpio.h"
/* USER CODE BEGIN 0 */
#include "main.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

void gpio_pa4_change_mode(uint32_t in_mode){
	if(in_mode == LL_GPIO_MODE_OUTPUT){
		LL_GPIO_DisablePinAnalogControl(LED_GPIO_Port, LED_Pin);
		LL_GPIO_SetPinMode(LED_GPIO_Port, LED_Pin, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinOutputType(LED_GPIO_Port, LED_Pin, LL_GPIO_OUTPUT_PUSHPULL);
		LL_GPIO_SetPinSpeed(LED_GPIO_Port, LED_Pin, LL_GPIO_SPEED_FREQ_LOW);
	}
	if(in_mode == LL_GPIO_MODE_ANALOG){
		LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
		LL_GPIO_EnablePinAnalogControl(GPIOA, LL_GPIO_PIN_4);
	}
}

static void output_gpio_init(void){
    // LED
	LL_GPIO_SetPinMode(LED_GPIO_Port, LED_Pin, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinPull(LED_GPIO_Port, LED_Pin, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinOutputType(LED_GPIO_Port, LED_Pin, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(LED_GPIO_Port, LED_Pin, LL_GPIO_SPEED_FREQ_LOW);
	LED_OFF();

    // VBUS_EN
//	LL_GPIO_SetPinMode(VBUS_EN_GPIO_Port, VBUS_EN_Pin, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinPull(VBUS_EN_GPIO_Port, VBUS_EN_Pin, LL_GPIO_PULL_UP);
//	LL_GPIO_SetPinOutputType(VBUS_EN_GPIO_Port, VBUS_EN_Pin, LL_GPIO_OUTPUT_PUSHPULL);
//	LL_GPIO_SetPinSpeed(VBUS_EN_GPIO_Port, VBUS_EN_Pin, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetOutputPin(VBUS_EN_GPIO_Port, VBUS_EN_Pin);

    // VSD_EN
	LL_GPIO_SetPinMode(VSD_EN_GPIO_Port, VSD_EN_Pin, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinPull(VSD_EN_GPIO_Port, VSD_EN_Pin, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinOutputType(VSD_EN_GPIO_Port, VSD_EN_Pin, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(VSD_EN_GPIO_Port, VSD_EN_Pin, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetOutputPin(VSD_EN_GPIO_Port, VSD_EN_Pin);

    // ADS_CS
	LL_GPIO_SetPinMode(ADS_CS_GPIO_Port, ADS_CS_Pin, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinPull(ADS_CS_GPIO_Port, ADS_CS_Pin, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinOutputType(ADS_CS_GPIO_Port, ADS_CS_Pin, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(ADS_CS_GPIO_Port, ADS_CS_Pin, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetOutputPin(ADS_CS_GPIO_Port, ADS_CS_Pin);

    // VGPS_EN
	LL_GPIO_SetPinMode(VGPS_EN_GPIO_Port, VGPS_EN_Pin, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinPull(VGPS_EN_GPIO_Port, VGPS_EN_Pin, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinOutputType(VGPS_EN_GPIO_Port, VGPS_EN_Pin, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(VGPS_EN_GPIO_Port, VGPS_EN_Pin, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetOutputPin(VGPS_EN_GPIO_Port, VGPS_EN_Pin);

    // TS_IN1
	LL_GPIO_SetPinMode(TS_IN1_GPIO_Port, TS_IN1_Pin, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinPull(TS_IN1_GPIO_Port, TS_IN1_Pin, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinOutputType(TS_IN1_GPIO_Port, TS_IN1_Pin, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(TS_IN1_GPIO_Port, TS_IN1_Pin, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetOutputPin(TS_IN1_GPIO_Port, TS_IN1_Pin);

	// ADS Start Pin
	LL_GPIO_SetPinMode(ADS_START_GPIO_Port, ADS_START_Pin, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinPull(ADS_START_GPIO_Port, ADS_START_Pin, LL_GPIO_PULL_DOWN);
	LL_GPIO_SetPinOutputType(ADS_START_GPIO_Port, ADS_START_Pin, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(ADS_START_GPIO_Port, ADS_START_Pin, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_ResetOutputPin(ADS_START_GPIO_Port, ADS_START_Pin);

	// ADS Power Pin
	LL_GPIO_SetPinMode(V5_EN_GPIO_Port, V5_EN_Pin, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinPull(V5_EN_GPIO_Port, V5_EN_Pin, LL_GPIO_PULL_DOWN);
	LL_GPIO_SetPinOutputType(V5_EN_GPIO_Port, V5_EN_Pin, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(V5_EN_GPIO_Port, V5_EN_Pin, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_ResetOutputPin(V5_EN_GPIO_Port, V5_EN_Pin);

}

static void input_gpio_init(void){
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	 // SD_CD
	GPIO_InitStruct.Pin = SD_CD_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(SD_CD_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_4 | LL_GPIO_PIN_5;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

static void exit_gpio_init(void){
	GPIO_InitTypeDef GPIO_InitStruct;

	// EXTI_USBCD_Pin
	GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin = EXTI_USBCD_Pin;
	HAL_GPIO_Init(EXTI_USBCD_GPIO_Port, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI_USBCD_EXTI_IRQn, EXTI_USBCD_Priority, 0);
	HAL_NVIC_EnableIRQ(EXTI_USBCD_EXTI_IRQn);
   // EXTI_DRDY_Pin
	GPIO_InitStruct.Mode  = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin = EXTI_DRDY_Pin;
	HAL_GPIO_Init(EXTI_DRDY_GPIO_Port, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI_DRDY_EXTI_IRQn, EXTI_DRDY_Priority, 0);
	HAL_NVIC_EnableIRQ(EXTI_DRDY_EXTI_IRQn);
	// EXTI_PPS_Pin
	GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin = EXTI_PPS_Pin;
	HAL_GPIO_Init(EXTI_PPS_GPIO_Port, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI_PPS_EXTI_IRQn, EXTI_PPS_Priority, 0);
	HAL_NVIC_EnableIRQ(EXTI_PPS_EXTI_IRQn);
	// EXTI_IMU1_Pin
	GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin = EXTI_IMU1_Pin;
	HAL_GPIO_Init(EXTI_IMU1_GPIO_Port, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI_IMU1_EXTI_IRQn, EXTI_IMU1_Priority, 0);
	HAL_NVIC_EnableIRQ(EXTI_IMU1_EXTI_IRQn);
}

void ll_exti_it_gpio_init(GPIO_TypeDef * in_gpio_port, uint32_t in_exti_pin, uint32_t in_trigger){
	uint32_t _exti_port = 0, _exti_line = 0;

	LL_GPIO_SetPinMode(in_gpio_port, in_exti_pin, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(in_gpio_port, in_exti_pin, LL_GPIO_PULL_NO);

	if(LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_SYSCFG) == 0) LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

	_exti_port = ((((uint32_t)in_gpio_port - (uint32_t)AHB2PERIPH_BASE) >> 8) >> 2);
	_exti_line = (uint32_t)((0x000FU << ((in_exti_pin % 4) << 2)) << 16U | (in_exti_pin >> 2));

	LL_SYSCFG_SetEXTISource(_exti_port, _exti_line);

	if(_exti_line < 31){       // GPIO PIN < 16
		LL_EXTI_EnableIT_0_31(_exti_line);
        LL_EXTI_DisableEvent_0_31(_exti_line);
		switch(in_trigger){
		case LL_EXTI_TRIGGER_RISING:
			LL_EXTI_EnableRisingTrig_0_31(_exti_line);
			LL_EXTI_DisableFallingTrig_0_31(_exti_line);
			break;
		case LL_EXTI_TRIGGER_FALLING:
			LL_EXTI_DisableRisingTrig_0_31(_exti_line);
			LL_EXTI_EnableFallingTrig_0_31(_exti_line);
			break;
		case LL_EXTI_TRIGGER_RISING_FALLING:
			LL_EXTI_EnableFallingTrig_0_31(_exti_line);
			LL_EXTI_EnableRisingTrig_0_31(_exti_line);
			break;
		default:
			LL_EXTI_DisableFallingTrig_0_31(_exti_line);
			LL_EXTI_DisableRisingTrig_0_31(_exti_line);
			break;
		}
	}
}

void ll_exti_it_gpio_deinit(GPIO_TypeDef * in_gpio_port, uint32_t in_exti_pin){
	uint32_t _exti_port = 0, _exti_line = 0;

	if(LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_SYSCFG) == 0) LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

	_exti_port = ((((uint32_t)in_gpio_port - (uint32_t)AHB2PERIPH_BASE) >> 8) >> 2);
	_exti_line = (uint32_t)((0x000FU << ((in_exti_pin % 4) << 2)) << 16U | (in_exti_pin >> 2));

	if(_exti_line < 31){
		LL_EXTI_DisableIT_0_31(_exti_line);
		LL_EXTI_DisableEvent_0_31(_exti_line);
		LL_EXTI_DisableFallingTrig_0_31(_exti_line);
		LL_EXTI_DisableRisingTrig_0_31(_exti_line);
	}
}


void optimize_gpio_power(GPIO_TypeDef *in_gpio, uint32_t in_pin){
    uint32_t _pin = in_pin;
    uint32_t _pin_pos = 0, _pin_bit = 0;

    while((_pin >> _pin_pos) != 0x00){
    	_pin_bit = _pin & (0x00000001 << _pin_pos);
    	if(_pin_bit){
        	LL_GPIO_SetPinMode(in_gpio, _pin_bit, LL_GPIO_MODE_ANALOG);
        	LL_GPIO_SetPinPull(in_gpio, _pin_bit, LL_GPIO_PULL_NO);
    	}
    	_pin_pos++;
    }
}

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
     PA8   ------> RCC_MCO
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOH);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);

  output_gpio_init();
//  input_gpio_init();
//  exit_gpio_init();
//  optimize_gpio_power(GPIOA, LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11);
//  optimize_gpio_power(GPIOB, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 |LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_15);
//  optimize_gpio_power(GPIOC, LL_GPIO_PIN_1 | LL_GPIO_PIN_7);
}

/* USER CODE BEGIN 2 */
void gpio_all_set_analog(void){
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOH);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_13|LL_GPIO_PIN_15|LL_GPIO_PIN_0|LL_GPIO_PIN_1
						  |LL_GPIO_PIN_2|LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5
						  |LL_GPIO_PIN_6|LL_GPIO_PIN_7;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3
						  |LL_GPIO_PIN_9|LL_GPIO_PIN_10|LL_GPIO_PIN_11|LL_GPIO_PIN_12
						  |LL_GPIO_PIN_15;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_10
						  |LL_GPIO_PIN_11|LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14
						  |LL_GPIO_PIN_15|LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5
						  |LL_GPIO_PIN_6|LL_GPIO_PIN_7|LL_GPIO_PIN_8|LL_GPIO_PIN_9;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
