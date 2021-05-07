/**
  ******************************************************************************
  * File Name          : LPTIM.c
  * Description        : This file provides code for the configuration
  *                      of the LPTIM instances.
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
#include "lptim.h"

/* USER CODE BEGIN 0 */

#include "main.h"

/* USER CODE END 0 */

static __IO uint32_t lptim_freq[2] = {0}; // Unit: mHz
/* USER CODE BEGIN 1 */

void lptim_init(LPTIM_TypeDef *in_lptim){
    if((uint32_t)in_lptim == (uint32_t)LPTIM1){
    	LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_PCLK1);  //
    	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
		/* GPIO LPTIM1_IN1 configuration */

//		LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
//		LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_2, LL_GPIO_PULL_DOWN);
//		LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_HIGH);
//		LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_2, LL_GPIO_AF_1);

		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);
		LL_LPTIM_SetClockSource(LPTIM1, LL_LPTIM_CLK_SOURCE_INTERNAL);
		LL_LPTIM_SetPrescaler(LPTIM1, LL_LPTIM_PRESCALER_DIV64);  //  250KHz
		LL_LPTIM_ConfigOutput(LPTIM1, LL_LPTIM_OUTPUT_WAVEFORM_PWM, LL_LPTIM_OUTPUT_POLARITY_INVERSE);
		LL_LPTIM_SetCounterMode(LPTIM1, LL_LPTIM_COUNTER_MODE_INTERNAL);
		LL_LPTIM_Disable(LPTIM1);
    }
    if((uint32_t)in_lptim == (uint32_t)LPTIM2){
    	LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM2_CLKSOURCE_PCLK1);
		LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_LPTIM2);
		LL_LPTIM_Disable(LPTIM2);
		LL_LPTIM_SetClockSource(LPTIM2, LL_LPTIM_CLK_SOURCE_INTERNAL);
		LL_LPTIM_SetPrescaler(LPTIM2, LL_LPTIM_PRESCALER_DIV128);
//		LL_LPTIM_ConfigOutput(LPTIM2, LL_LPTIM_OUTPUT_WAVEFORM_PWM, LL_LPTIM_OUTPUT_POLARITY_REGULAR);
		LL_LPTIM_SetCounterMode(LPTIM2, LL_LPTIM_COUNTER_MODE_INTERNAL);

    }
}

void lptim_deinit(LPTIM_TypeDef *in_lptim){
	if((uint32_t)in_lptim == (uint32_t)LPTIM1){
		lptim_stop(LPTIM1);
		LL_LPTIM_DeInit(LPTIM1);
		LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_2, LL_GPIO_MODE_ANALOG);
	}
	if((uint32_t)in_lptim == (uint32_t)LPTIM2){
		lptim_stop(LPTIM2);
		LL_LPTIM_DeInit(LPTIM2);
		LL_GPIO_SetPinMode(ADS_CLK_GPIO_Port, ADS_CLK_Pin, LL_GPIO_MODE_ANALOG);
	}
}

void lptim2_use_for_ads_clk(uint32_t in_clk_freq){
	uint32_t _lptim_clk = 0, _div = 0, _lptim_cmr = 0, _lptim_arr = 0;
	lptim_deinit(LPTIM2);

	LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_NOCLOCK, LL_RCC_MCO1_DIV_16);

	LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM2_CLKSOURCE_PCLK1);
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_LPTIM2);

	LL_GPIO_SetPinMode(ADS_CLK_GPIO_Port, ADS_CLK_Pin, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinPull(ADS_CLK_GPIO_Port, ADS_CLK_Pin, LL_GPIO_PULL_DOWN);
	LL_GPIO_SetPinSpeed(ADS_CLK_GPIO_Port, ADS_CLK_Pin, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetAFPin_8_15(ADS_CLK_GPIO_Port, ADS_CLK_Pin, LL_GPIO_AF_14);

	LL_LPTIM_Disable(LPTIM2);
	LL_LPTIM_DisableIT_ARRM(LPTIM2);
//	LL_LPTIM_EnableIT_ARRM(LPTIM2);
	LL_LPTIM_DisableIT_CMPM(LPTIM2);
//	HAL_NVIC_SetPriority(LPTIM2_IRQn, LPTIM2_Priority, 0);
	NVIC_DisableIRQ(LPTIM2_IRQn);

	LL_LPTIM_SetClockSource(LPTIM2, LL_LPTIM_CLK_SOURCE_INTERNAL);
	LL_LPTIM_SetPrescaler(LPTIM2, LL_LPTIM_PRESCALER_DIV1);
	LL_LPTIM_ConfigOutput(LPTIM2, LL_LPTIM_OUTPUT_WAVEFORM_PWM, LL_LPTIM_OUTPUT_POLARITY_REGULAR);
	LL_LPTIM_SetCounterMode(LPTIM2, LL_LPTIM_COUNTER_MODE_INTERNAL);
	LL_LPTIM_Enable(LPTIM2);

	_lptim_clk = LL_RCC_GetLPTIMClockFreq(LL_RCC_LPTIM2_CLKSOURCE);
	_div = 1 << (LL_LPTIM_GetPrescaler(LPTIM2) >> 9);
//	_lptim_arr = _lptim_clk / _div / in_clk_freq - 1;
	_lptim_arr = __LL_TIM_CALC_ARR(_lptim_clk, _div - 1, in_clk_freq);
	_lptim_cmr = _lptim_arr >> 1;
	LL_LPTIM_SetAutoReload(LPTIM2, _lptim_arr);
    while(!LL_LPTIM_IsActiveFlag_ARROK(LPTIM2));
    LL_LPTIM_ClearFlag_ARROK(LPTIM2);
	LL_LPTIM_SetCompare(LPTIM2, _lptim_cmr);
	while(!LL_LPTIM_IsActiveFlag_CMPOK(LPTIM2));
	LL_LPTIM_ClearFlag_CMPOK(LPTIM2);
	LL_LPTIM_StartCounter(LPTIM2, LL_LPTIM_OPERATING_MODE_CONTINUOUS);
}

void lptim_start(LPTIM_TypeDef *in_lptim){
    if((uint32_t)in_lptim == (uint32_t)LPTIM1){
    	st_irq_handler_register(LPTIM1_IRQn, lptim1_irq_callback);
        HAL_NVIC_SetPriority(LPTIM1_IRQn, LPTIM1_Priority, 0);
        NVIC_EnableIRQ(LPTIM1_IRQn);
        LL_LPTIM_EnableIT_ARRM(LPTIM1);
        LL_LPTIM_EnableIT_CMPM(LPTIM1);
        if(LL_LPTIM_IsEnabled(LPTIM1) == 0) LL_LPTIM_Enable(LPTIM1);
        LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_CONTINUOUS);
    }
    if((uint32_t)in_lptim == (uint32_t)LPTIM2){
//    	st_irq_handler_register(LPTIM2_IRQn, lptim2_irq_callback);
//        HAL_NVIC_SetPriority(LPTIM2_IRQn, LPTIM2_Priority, 0);
//        NVIC_EnableIRQ(LPTIM2_IRQn);
//        LL_LPTIM_EnableIT_ARRM(LPTIM2);
//        LL_LPTIM_EnableIT_CMPM(LPTIM2);
        if(LL_LPTIM_IsEnabled(LPTIM2) == 0) LL_LPTIM_Enable(LPTIM2);
        LL_LPTIM_StartCounter(LPTIM2, LL_LPTIM_OPERATING_MODE_CONTINUOUS);
    }
}

void lptim_stop(LPTIM_TypeDef *in_lptim){
    if((uint32_t)in_lptim == (uint32_t)LPTIM1){
    	LL_LPTIM_Disable(LPTIM1);
//    	LL_LPTIM_DisableIT_ARRM(LPTIM1);
//    	LL_LPTIM_DisableIT_CMPM(LPTIM1);
    }
    if((uint32_t)in_lptim == (uint32_t)LPTIM2){
    	LL_LPTIM_Disable(LPTIM2);
//    	LL_LPTIM_DisableIT_ARRM(LPTIM2);
//    	LL_LPTIM_DisableIT_CMPM(LPTIM2);
    }
}


void lptim_set_freq(LPTIM_TypeDef *in_lptim, uint32_t in_freq_mhz){
    if((uint32_t)in_lptim == (uint32_t)LPTIM1){
    	if(LL_LPTIM_IsEnabled(LPTIM1) == 0) LL_LPTIM_Enable(LPTIM1);
    	uint32_t _lptim_clk = 0, _div = 0;
    	_lptim_clk = LL_RCC_GetLPTIMClockFreq(LL_RCC_LPTIM1_CLKSOURCE);
    	_div = 1 << (LL_LPTIM_GetPrescaler(LPTIM1) >> 9);
    	_lptim_clk /= _div;
    	if(in_freq_mhz < 1000){  // 小于 1Hz
//        	_lptim_clk *= 1000;
        	_div = _lptim_clk / in_freq_mhz * 1000;
    	}else{
    		in_freq_mhz /= 1000;
    		_div = _lptim_clk / in_freq_mhz;
    	}
    	if(_div > 0xFFFF) _div = 0xFFFF;
    	LL_LPTIM_SetAutoReload(LPTIM1, _div);
    	lptim_freq[0] = in_freq_mhz;
        while(!LL_LPTIM_IsActiveFlag_ARROK(LPTIM1));
        LL_LPTIM_ClearFlag_ARROK(LPTIM1);
    }
    if((uint32_t)in_lptim == (uint32_t)LPTIM2){
    	if(LL_LPTIM_IsEnabled(LPTIM2) == 0) LL_LPTIM_Enable(LPTIM2);
    	uint32_t _lptim_clk = 0, _div = 0;
    	_lptim_clk = LL_RCC_GetLPTIMClockFreq(LL_RCC_LPTIM2_CLKSOURCE);
    	_div = 1 << (LL_LPTIM_GetPrescaler(LPTIM2) >> 9);
    	_lptim_clk /= _div;
    	if(in_freq_mhz < 1000){  // 小于 1Hz
//        	_lptim_clk *= 1000;
        	_div = _lptim_clk / in_freq_mhz * 1000;
    	}else{
    		in_freq_mhz /= 1000;
    		_div = _lptim_clk / in_freq_mhz;
    	}
    	if(_div > 0xFFFF) _div = 0xFFFF;
//    	if(_div == 0) _div = 1;
    	LL_LPTIM_SetAutoReload(LPTIM2, _div);
    	lptim_freq[1] = in_freq_mhz;
        while(!LL_LPTIM_IsActiveFlag_ARROK(LPTIM2));
        LL_LPTIM_ClearFlag_ARROK(LPTIM2);
    }
}

uint32_t lptim_get_freq(LPTIM_TypeDef *in_lptim){
    if((uint32_t)in_lptim == (uint32_t)LPTIM1){
    	return lptim_freq[0];
    }
    if((uint32_t)in_lptim == (uint32_t)LPTIM2){
    	return lptim_freq[1];
    }
    return 0;
}

void lptim_set_duty(LPTIM_TypeDef *in_lptim, uint8_t in_duty){
	if(in_duty > 100) in_duty = 100;
    if((uint32_t)in_lptim == (uint32_t)LPTIM1){
    	uint32_t _lptim_arr = 0, _lptim_cmr = 0;
    	_lptim_arr = LL_LPTIM_GetAutoReload(LPTIM1);
    	_lptim_cmr = _lptim_arr * in_duty / 100;
    	LL_LPTIM_SetCompare(LPTIM1, _lptim_cmr);
    	while(!LL_LPTIM_IsActiveFlag_CMPOK(LPTIM1));
    	LL_LPTIM_ClearFlag_CMPOK(LPTIM1);
    }
    if((uint32_t)in_lptim == (uint32_t)LPTIM2){
    	uint32_t _lptim_arr = 0, _lptim_cmr = 0;
    	_lptim_arr = LL_LPTIM_GetAutoReload(LPTIM2);
    	_lptim_cmr = _lptim_arr * in_duty / 100;
    	LL_LPTIM_SetCompare(LPTIM2, _lptim_cmr);
    	while(!LL_LPTIM_IsActiveFlag_CMPOK(LPTIM2));
    	LL_LPTIM_ClearFlag_CMPOK(LPTIM2);
    }
}

__weak void lptim1_irq_callback(void){
	if(LL_LPTIM_IsActiveFlag_CMPM(LPTIM1)){
		LL_LPTIM_ClearFLAG_CMPM(LPTIM1);
	}
	if(LL_LPTIM_IsActiveFlag_ARRM(LPTIM1)){
		LL_LPTIM_ClearFLAG_ARRM(LPTIM1);
	}
}

__weak void lptim2_irq_callback(void){
	if(LL_LPTIM_IsActiveFlag_CMPM(LPTIM2)){
		LL_LPTIM_ClearFLAG_CMPM(LPTIM2);
	}
	if(LL_LPTIM_IsActiveFlag_ARRM(LPTIM2)){
		LL_LPTIM_ClearFLAG_ARRM(LPTIM2);
	}
}



/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
