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
/* USER CODE BEGIN 0 */
#ifndef ADC_CONVER_DATA_BUFFER_SIZE
#define ADC_CONVER_DATA_BUFFER_SIZE                         1
#endif

#ifndef VDDA_APPLI
#define VDDA_APPLI               ((uint32_t)3300)
#endif

static __IO uint32_t adc_data_index = 0;
static uint8_t adc_status = 0;
static uint16_t adc_conver_data[ADC_CONVER_DATA_BUFFER_SIZE] = { 0 };

#if(ADC_USE_DMA)
static __IO int8_t adc_dma_status = 0;
static uint8_t _adc1_dma_config(uint16_t *in_ptr, uint16_t in_size);
static uint8_t _adc2_dma_config(uint16_t *in_ptr, uint16_t in_size);
#endif
/* USER CODE END 0 */


/* ADC1 init function */
void st_adc_init(void){
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

	HAL_NVIC_SetPriority(ADC1_2_IRQn, (DMA1_Channel1_Priority - 1), 0);
	NVIC_EnableIRQ(ADC1_2_IRQn);

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
#if(ADC_USE_DMA)
	ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
#else
	ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
#endif
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

#if(ADC_USE_DMA)
    _adc1_dma_config(adc_conver_data, ADC_CONVER_DATA_BUFFER_SIZE);
//    _adc2_dma_config(adc_conver_data, ADC_CONVER_DATA_BUFFER_SIZE);
#else
    LL_ADC_ClearFlag_OVR(ADC1);
    LL_ADC_EnableIT_OVR(ADC1);
	LL_ADC_EnableIT_EOS(ADC1);
	LL_ADC_EnableIT_EOC(ADC1);
#endif
}

void st_adc_deinit(void){
	LL_ADC_DeInit(ADC1);
	LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_ADC);
	LL_GPIO_DisablePinAnalogControl(GPIOC, LL_GPIO_PIN_0);
	LL_GPIO_DisablePinAnalogControl(GPIOA, LL_GPIO_PIN_4);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
}

/* USER CODE BEGIN 1 */

void st_adc_start(void){
	if(LL_ADC_IsEnabled(ADC1) == 0){
//		LL_ADC_DisableDeepPowerDown(ADC1);
//		LL_ADC_EnableInternalRegulator(ADC1);
		LL_ADC_Enable(ADC1);
		while(LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);
	}
#if(ADC_USE_DMA)
	adc_dma_status = 0;
	_adc1_dma_config(adc_conver_data, ADC_CONVER_DATA_BUFFER_SIZE);
#endif
	if((LL_ADC_IsDisableOngoing(ADC1) == 0) && (LL_ADC_REG_IsConversionOngoing(ADC1) == 0)){
		adc_status = 0;
		LL_ADC_REG_StartConversion(ADC1);
	}
}

void st_adc_stop(void){
    if(LL_ADC_IsEnabled(ADC1)
    		&& (LL_ADC_IsDisableOngoing(ADC1) == 0)
			&& (LL_ADC_REG_IsStopConversionOngoing(ADC1) == 0)){
        LL_ADC_REG_StopConversion(ADC1);
    	LL_ADC_Disable(ADC1);
#if(ADC_USE_DMA)
    	adc_dma_status = 0;
#endif
    	adc_data_index = 0;
    	adc_status = 0;
    }
}


uint16_t get_extern_analog_voltage(void){
#if(ADC_USE_DMA)
	if(adc_dma_status != 0){
	    if(adc_data_index && (adc_dma_status > 0)){
            uint16_t _extern_mvoltage = 0;
            _extern_mvoltage = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, adc_conver_data[0], LL_ADC_RESOLUTION_8B);
            return _extern_mvoltage;
	    }
	    if(adc_dma_status < 0){
	    	adc_data_index = 0;
	    	adc_dma_status = 0;
	    	_adc1_dma_config(adc_conver_data, ADC_CONVER_DATA_BUFFER_SIZE);
	    	st_adc_start();
	    }
	}
#else
	if(adc_status){
		uint16_t _extern_mvoltage = 0;
		_extern_mvoltage = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, adc_conver_data[0], LL_ADC_RESOLUTION_8B);
		return _extern_mvoltage;
	}
#endif
	return 0;
}

uint16_t get_internal_voltage(void){
#if(ADC_USE_DMA)
	if(adc_dma_status != 0){
	    if(adc_data_index && (adc_dma_status > 0)){
            uint16_t _internal_mvoltage = 0;
            _internal_mvoltage = __LL_ADC_CALC_VREFANALOG_VOLTAGE(adc_conver_data[1], LL_ADC_RESOLUTION_12B);
            return _internal_mvoltage;
	    }
	    if(adc_dma_status < 0){
	    	adc_data_index = 0;
	    	adc_dma_status = 0;
	    	_adc1_dma_config(adc_conver_data, ADC_CONVER_DATA_BUFFER_SIZE);
	    	st_adc_start();
	    }
	}
#else

#endif


	return 0;
}

float get_internal_temp(void){

#if(ADC_USE_DMA)
	if(adc_dma_status != 0){
	    if(adc_data_index && (adc_dma_status > 0)){
            uint16_t _internal_temp = 0;
            _internal_temp = __LL_ADC_CALC_TEMPERATURE(VDDA_APPLI, adc_conver_data[2], LL_ADC_RESOLUTION_12B);
            return (float)_internal_temp;
	    }
	    if(adc_dma_status < 0){
	    	adc_data_index = 0;
	    	adc_dma_status = 0;
	    	_adc1_dma_config(adc_conver_data, ADC_CONVER_DATA_BUFFER_SIZE);
	    	st_adc_start();
	    }
	}
#else
   if(adc_status){
       uint16_t _internal_temp = 0;
       _internal_temp = __LL_ADC_CALC_TEMPERATURE(VDDA_APPLI, adc_conver_data[1], LL_ADC_RESOLUTION_8B);
       return (float)_internal_temp;
   }

#endif
	return 25.00f;
}

static void adc1_eoc_callback(void){
//	adc_data_index += 1;
	adc_conver_data[adc_data_index++] = LL_ADC_REG_ReadConversionData32(ADC1);
	adc_status = 1;
}

static void adc1_eos_callback(void){
    adc_data_index = 0;
    adc_status = 2;
}

void adc1_irq_callback(void){
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
}

#if(ADC_USE_DMA)
static uint8_t _adc1_dma_config(uint16_t *in_ptr, uint16_t in_size){    // DMA1 Channel1; Request 0
    uint8_t ret = 0;
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, DMA1_Channel1_Priority, 0);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_ClearFlag_GI1(DMA1);
	/* (3) Configure the DMA1_Channel2 functional parameters */
	LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_1,
			            LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
						LL_DMA_PRIORITY_HIGH |
						LL_DMA_MODE_NORMAL |
						LL_DMA_PERIPH_NOINCREMENT |
						LL_DMA_MEMORY_INCREMENT |
						LL_DMA_PDATAALIGN_HALFWORD |
						LL_DMA_PDATAALIGN_HALFWORD);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
			LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
			(uint32_t)in_ptr,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, in_size);
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMA_REQUEST_0);

	/* (5) Enable DMA interrupts complete/error */
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);

	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	return ret;
}

static uint8_t _adc2_dma_config(uint16_t *in_ptr, uint16_t in_size){    // DMA1 Channel1; Request 0
    uint8_t ret = 0;
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, DMA1_Channel2_Priority, 0);
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
	LL_DMA_ClearFlag_GI1(DMA1);
	/* (3) Configure the DMA1_Channel2 functional parameters */
	LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_2,
			            LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
						LL_DMA_PRIORITY_HIGH |
						LL_DMA_MODE_CIRCULAR |
						LL_DMA_PERIPH_NOINCREMENT |
						LL_DMA_MEMORY_INCREMENT |
						LL_DMA_PDATAALIGN_HALFWORD |
						LL_DMA_PDATAALIGN_HALFWORD);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2,
			LL_ADC_DMA_GetRegAddr(ADC2, LL_ADC_DMA_REG_REGULAR_DATA),
			(uint32_t)in_ptr,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, in_size);
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMA_REQUEST_0);

	/* (5) Enable DMA interrupts complete/error */
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);

	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

	return ret;
}

void adc2_dma_rxcplt_callback(void){
	adc_data_index += 1;
	adc_dma_status = 1;
}

void adc2_dma_err_callback(void){
    adc_dma_status = -1;
    st_adc_stop();
}

void adc1_dma_rxcplt_callback(void){
	if(LL_DMA_IsActiveFlag_TC1(DMA1)){
		LL_DMA_ClearFlag_GI1(DMA1);
		adc_data_index += 1;
		adc_dma_status = 1;
	}
}

void adc1_dma_err_callback(void){
    adc_dma_status = -1;
    st_adc_stop();
}
#endif

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
