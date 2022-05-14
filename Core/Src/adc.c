/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
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
#include "adc.h"
#include "main.h"

#include "stm32l4xx_hal.h"

/* USER CODE BEGIN 0 */
#ifndef ADC_CONVER_DATA_BUFFER_SIZE
#define ADC_CONVER_DATA_BUFFER_SIZE                         1
#endif

#ifndef VDDA_APPLI
#define VDDA_APPLI               ((uint32_t)3300)
#endif

static __IO uint8_t adc_data_index = 0;
static uint8_t adc_status = 0;
static __IO uint16_t adc_conver_data[ADC_CONVER_DATA_BUFFER_SIZE] = { 0 };

static __IO uint32_t nbADCData = 0;

static ADC_HandleTypeDef AdcHandle;

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef          GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* ADC Periph clock enable */
  __HAL_RCC_ADC_CLK_ENABLE();
  /* ADC Periph interface clock configuration */
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);
  /* Enable GPIO clock ****************************************/
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* ADC Channel GPIO pin configuration */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(ADC1_IRQn, 13, 0);
  HAL_NVIC_EnableIRQ(ADC1_IRQn);

}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{

  /*##-1- Reset peripherals ##################################################*/
	__HAL_RCC_ADC_FORCE_RESET();
	__HAL_RCC_ADC_RELEASE_RESET();
  /* ADC Periph clock disable
   (automatically reset all ADC's) */
  __HAL_RCC_ADC_CLK_DISABLE();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize the ADC Channel GPIO pin */
  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  /* Get the converted value of regular channel */
	adc_conver_data[0] = HAL_ADC_GetValue(AdcHandle);
	adc_status = 1;
}

/* USER CODE END 0 */


/* ADC1 init function */
void st_adc_init(void){
#if(0)
	LL_ADC_InitTypeDef ADC_InitStruct = {0};
	LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
	LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
	__IO uint32_t wait_loop_index = 0;

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_ANALOG);
	LL_GPIO_EnablePinAnalogControl(GPIOC, LL_GPIO_PIN_0);

	/* Note: On this STM32 device, ADC1 internal channel temperature sensor is mapped on GPIO pin PA.04 */
	/* Configure GPIO in analog mode to be used as ADC input */
//	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
	/* Connect GPIO analog switch to ADC input */
//	LL_GPIO_EnablePinAnalogControl(GPIOA, LL_GPIO_PIN_4);

	LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_SYSCLK);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);

	if(LL_ADC_IsEnabled(ADC1)) LL_ADC_Disable(ADC1);
	/** Common config
	*/
	ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_8B;
	ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
	ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
	LL_ADC_Init(ADC1, &ADC_InitStruct);

	ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
	ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
	ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
	ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
	ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;

	ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
	LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);

//	if(LL_ADC_GetLowPowerMode(ADC1) == LL_ADC_LP_AUTOWAIT){
//		LL_ADC_ConfigOverSamplingRatioShift(ADC1, LL_ADC_OVS_RATIO_16, LL_ADC_OVS_SHIFT_NONE);
//		LL_ADC_SetOverSamplingDiscont(ADC1, LL_ADC_OVS_REG_DISCONT);
//	}else{
//		LL_ADC_ConfigOverSamplingRatioShift(ADC1, LL_ADC_OVS_RATIO_16, LL_ADC_OVS_SHIFT_RIGHT_4);
//		LL_ADC_SetOverSamplingDiscont(ADC1, LL_ADC_OVS_REG_CONT);
//	}

//	LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), (LL_ADC_PATH_INTERNAL_TEMPSENSOR | LL_ADC_PATH_INTERNAL_VREFINT));
	LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE);
	/* Disable ADC deep power down (enabled by default after reset state) */
	LL_ADC_DisableDeepPowerDown(ADC1);
	/* Enable ADC internal voltage regulator */
	LL_ADC_EnableInternalRegulator(ADC1);
	/* Delay for ADC internal voltage regulator stabilization. */
	/* Compute number of CPU cycles to wait for, from delay in us. */
	/* Note: Variable divided by 2 to compensate partially */
	/* CPU processing cycles (depends on compilation optimization). */
	/* Note: If system core clock frequency is below 200kHz, wait time */
	/* is only a few CPU processing cycles. */

	wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
	while(wait_loop_index != 0){ wait_loop_index--; }

	ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
	ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
	LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
	/** Configure Regular Channel
	*/
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_640CYCLES_5);
	LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);

//	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_VREFINT);
//	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_247CYCLES_5);
////	LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_PATH_INTERNAL_VREFINT, LL_ADC_SINGLE_ENDED);
//
//	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_TEMPSENSOR);
//	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_247CYCLES_5);
////	LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_PATH_INTERNAL_TEMPSENSOR, LL_ADC_SINGLE_ENDED);

	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
	while(LL_ADC_IsCalibrationOnGoing(ADC1) != 0);
    wait_loop_index = ((LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32) >> 1);
    while(wait_loop_index != 0){ wait_loop_index--; }

	HAL_NVIC_SetPriority(ADC1_2_IRQn, (DMA1_Channel1_Priority - 1), 0);
	NVIC_EnableIRQ(ADC1_2_IRQn);

	st_irq_handler_register(ADC1_2_IRQn, adc1_irq_callback);

    LL_ADC_ClearFlag_OVR(ADC1);
    LL_ADC_EnableIT_OVR(ADC1);
//	LL_ADC_EnableIT_EOS(ADC1);
	LL_ADC_EnableIT_EOC(ADC1);
#endif

	ADC_ChannelConfTypeDef   sConfig;
	/*##-1- Configure the ADC peripheral #######################################*/
	AdcHandle.Instance          = ADC1;

	if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK)
	{
	/* ADC de-initialization Error */
	Error_Handler();
	}

	AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;          /* Asynchronous clock mode, input ADC clock not divided */
	AdcHandle.Init.Resolution            = ADC_RESOLUTION_8B;            /* 12-bit resolution for converted data */
	AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
	AdcHandle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
	AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
	AdcHandle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
	AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
	AdcHandle.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
	AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
	AdcHandle.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
	AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
	AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
	AdcHandle.Init.DMAContinuousRequests = DISABLE;                       /* DMA one-shot mode selected (not applied to this example) */
	AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
	AdcHandle.Init.OversamplingMode      = DISABLE;                       /* No oversampling */

	if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
	{
	/* ADC initialization Error */
	Error_Handler();
	}

	/*##-2- Configure ADC regular channel ######################################*/
	sConfig.Channel      = ADC_CHANNEL_1;                /* Sampled channel number */
	sConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
	sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;    /* Sampling time (number of clock cycles unit) */
	sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
	sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */
	sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */


	if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
	{
	/* Channel Configuration Error */
	Error_Handler();
	}

	/* Run the ADC calibration in single-ended mode */
	if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED) != HAL_OK)
	{
	/* Calibration Error */
	Error_Handler();
	}

	st_irq_handler_register(ADC1_2_IRQn, st_adc1_irq_callback);
}

void st_adc_deinit(void){
#if(0)
	LL_ADC_DeInit(ADC1);
	LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_ADC);
	LL_GPIO_DisablePinAnalogControl(GPIOC, LL_GPIO_PIN_0);
	LL_GPIO_DisablePinAnalogControl(GPIOA, LL_GPIO_PIN_4);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
#endif

	if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK)
	{
	/* ADC de-initialization Error */
	Error_Handler();
	}
}

/* USER CODE BEGIN 1 */

void st_adc_start(void){
#if(0)
	if(LL_ADC_IsEnabled(ADC1) == 0){
//		LL_ADC_DisableDeepPowerDown(ADC1);
//		LL_ADC_EnableInternalRegulator(ADC1);
		LL_ADC_Enable(ADC1);
		while(LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);
	}
	if((LL_ADC_IsDisableOngoing(ADC1) == 0) && (LL_ADC_REG_IsConversionOngoing(ADC1) == 0)){
		adc_status = 0;
		LL_ADC_REG_StartConversion(ADC1);
	}
#endif
	/*##-3- Start the conversion process #######################################*/
	adc_status = 0;
	if (HAL_ADC_Start_IT(&AdcHandle) != HAL_OK)
	{
		adc_status = 1;
	/* Start Conversation Error */
	Error_Handler();
	}
}

void st_adc_stop(void){
#if(0)
    if(LL_ADC_IsEnabled(ADC1)
    		&& (LL_ADC_IsDisableOngoing(ADC1) == 0)
			&& (LL_ADC_REG_IsStopConversionOngoing(ADC1) == 0)){
        LL_ADC_REG_StopConversion(ADC1);
    	LL_ADC_Disable(ADC1);
    	adc_data_index = 0;
    	adc_status = 0;
    }
#endif
	if (HAL_ADC_Stop_IT(&AdcHandle) != HAL_OK)
	{
	/* Start Conversation Error */
	Error_Handler();
	}
	adc_status = 0;
}

void st_adc_wait_for_conver_cplt(void){
	uint32_t _timeout = SystemCoreClock;
	while((_timeout--) && (adc_status == 1));
}

// return voltage: mV
uint32_t get_extern_analog_voltage(void){
#if(0)
	if(adc_status){
		uint32_t _extern_mvoltage = 0;
		_extern_mvoltage = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, adc_conver_data[0], LL_ADC_RESOLUTION_8B);
		return _extern_mvoltage;
	}
	return 0;
#endif
	return __HAL_ADC_CALC_DATA_TO_VOLTAGE(3300, adc_conver_data[0], ADC_RESOLUTION_8B);
}

uint16_t get_internal_voltage(void){

	return 0;
}

float get_internal_temp(void){
   if(adc_status){
       uint16_t _internal_temp = 0;
       _internal_temp = __LL_ADC_CALC_TEMPERATURE(VDDA_APPLI, adc_conver_data[1], LL_ADC_RESOLUTION_8B);
       return (float)_internal_temp;
   }
	return 25.00f;
}

static uint8_t con_times = 0;
static void adc1_eoc_callback(void){
//	adc_data_index += 1;
	adc_conver_data[0] += LL_ADC_REG_ReadConversionData32(ADC1); // 只有一个转换
	if((++con_times) < 8){
		st_adc_start();
	}else{
		con_times = 0;
		adc_conver_data[0] >>= 3;
		adc_status = 1;
	}

}

static void adc1_eos_callback(void){
    adc_data_index = 0;
    adc_status = 2;
}

void st_adc1_irq_callback(void){
#if(0)
	if(LL_ADC_IsActiveFlag_EOS(ADC1) != 0){
		LL_ADC_ClearFlag_EOS(ADC1);
		adc1_eos_callback();
	}
	if(LL_ADC_IsActiveFlag_EOC(ADC1)){
		LL_ADC_ClearFlag_EOC(ADC1);
		adc1_eoc_callback();
	}
	if(LL_ADC_IsActiveFlag_OVR(ADC1) != 0){
	/* Clear flag ADC group regular overrun */
	    LL_ADC_ClearFlag_OVR(ADC1);

	/* Call interruption treatment function */
		LL_ADC_DisableIT_OVR(ADC1);
	}
#endif

	HAL_ADC_IRQHandler(&AdcHandle);
}


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
