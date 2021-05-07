/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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

#include "tim.h"
#include "main.h"


/* USER CODE BEGIN 0 */
void tim_init(TIM_TypeDef *in_tim_index){
    if((uint32_t)in_tim_index == (uint32_t)TIM1){
    	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    	LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);
    	LL_TIM_EnableARRPreload(TIM1);
    	LL_TIM_SetPrescaler(TIM1, __LL_TIM_CALC_PSC(SystemCoreClock, SystemCoreClock));
    }
}

void tim_deinit(TIM_TypeDef *in_tim_index){
	if((uint32_t)in_tim_index == (uint32_t)TIM1){
		LL_TIM_DeInit(TIM1);
		LL_GPIO_SetPinMode(ADS_CLK_GPIO_Port, ADS_CLK_Pin, LL_GPIO_MODE_ANALOG);
	}
}

void tim_start(TIM_TypeDef *in_tim_index){
	if((uint32_t)in_tim_index == (uint32_t)TIM1){
		LL_TIM_EnableCounter(TIM1);
	}
}

void tim_stop(TIM_TypeDef *in_tim_index){
	if((uint32_t)in_tim_index == (uint32_t)TIM1){
		LL_TIM_DisableCounter(TIM1);
	}
}

void tim1_use_for_ads_clk(uint32_t in_clk_freq){
	uint32_t _arr = 0;

	tim_deinit(TIM1);

    if(LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_TIM1) == 0)
    	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
//    if(LL_TIM_IsEnabledCounter(TIM1)) LL_TIM_DisableCounter(TIM1);
    LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);

    LL_GPIO_SetPinMode(ADS_CLK_GPIO_Port, ADS_CLK_Pin, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinPull(ADS_CLK_GPIO_Port, ADS_CLK_Pin, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinSpeed(ADS_CLK_GPIO_Port, ADS_CLK_Pin, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(ADS_CLK_GPIO_Port, ADS_CLK_Pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetAFPin_8_15(ADS_CLK_GPIO_Port, ADS_CLK_Pin, LL_GPIO_AF_1);

	LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);
	  /* Enable TIM2_ARR register preload. Writing to or reading from the         */
	  /* auto-reload register accesses the preload register. The content of the   */
	  /* preload register are transferred into the shadow register at each update */
	  /* event (UEV). */
 	LL_TIM_EnableARRPreload(TIM1);    // 使能预重装
	LL_TIM_SetPrescaler(TIM1, __LL_TIM_CALC_PSC(SystemCoreClock, SystemCoreClock));  // 设置分频为 0 ;
	_arr = __LL_TIM_CALC_ARR(SystemCoreClock, LL_TIM_GetPrescaler(TIM1), in_clk_freq);
	LL_TIM_SetAutoReload(TIM1, _arr);  // 设置重装值

//	LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
//	LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM1);

	// 设置输出
//	LL_TIM_SetOnePulseMode(TIM1, LL_TIM_ONEPULSEMODE_SINGLE);
	LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_ConfigOutput(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_LOW | LL_TIM_OCIDLESTATE_LOW);
	LL_TIM_OC_SetCompareCH1(TIM1, (LL_TIM_GetAutoReload(TIM1) >> 1));   // duty = 50%
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableAllOutputs(TIM1);

	LL_TIM_EnableCounter(TIM1);
	  /* Force update generation */
	LL_TIM_GenerateEvent_UPDATE(TIM1);
}

uint8_t tim_set_output_freq(TIM_TypeDef *in_tim, uint8_t in_channel_no_use, uint32_t in_freq_hz){
	uint32_t _arr = 0;
	in_channel_no_use = in_channel_no_use;

	if((uint32_t)in_tim == (uint32_t)TIM1){
		_arr = __LL_TIM_CALC_ARR(SystemCoreClock, LL_TIM_GetPrescaler(TIM1), in_freq_hz);
		LL_TIM_SetAutoReload(TIM1, _arr);  // 设置重装值
	}
	return 0;
}

uint8_t tim_set_output_duty(TIM_TypeDef *in_tim, uint8_t in_channel, uint32_t in_duty){
	uint32_t _arr = 0;
	uint32_t _cmr = 0;
	if((uint32_t)in_tim == (uint32_t)TIM1){
        _arr = LL_TIM_GetAutoReload(TIM1);
        _cmr = _arr * in_duty / 100;
        switch(in_channel){
        case LL_TIM_CHANNEL_CH1:
        	LL_TIM_OC_SetCompareCH1(TIM1, _cmr);
        	break;
        default:
        	break;
        }
	}
	return 0;
}

/* USER CODE END 0 */


/* TIM3 init function */


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
