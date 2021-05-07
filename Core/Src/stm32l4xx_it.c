/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static st_irq_handler_callback_t systick_handler_callback = NULL;

static st_irq_handler_callback_t dma1_ch_handler_callback[8] = { NULL };
static st_irq_handler_callback_t dma2_ch_handler_callback[8] = { NULL };

static st_irq_handler_callback_t dma1_ch1_handler_callback = NULL;
static st_irq_handler_callback_t dma1_ch2_handler_callback = NULL;
static st_irq_handler_callback_t dma1_ch3_handler_callback = NULL;
static st_irq_handler_callback_t dma1_ch4_handler_callback = NULL;
static st_irq_handler_callback_t dma1_ch5_handler_callback = NULL;
static st_irq_handler_callback_t dma2_ch3_handler_callback = NULL;
static st_irq_handler_callback_t dma2_ch4_handler_callback = NULL;
static st_irq_handler_callback_t dma2_ch5_handler_callback = NULL;

static st_irq_handler_callback_t sdmmc1_handler_callback = NULL;
static st_irq_handler_callback_t spi1_handler_callback = NULL;
static st_irq_handler_callback_t lptim2_handler_callback = NULL;
static st_irq_handler_callback_t lptim1_handler_callback = NULL;

static st_irq_handler_callback_t exti_line_handler_callback[32] = { NULL };

static st_irq_handler_callback_t rtc_wkup_handler_callback = NULL;

static st_irq_handler_callback_t adc_handler_callback = NULL;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/


/* USER CODE BEGIN EV */
void st_irq_handler_register(uint32_t _irq_num, void *_handler){
	switch(_irq_num){
	case SysTick_IRQn:
		systick_handler_callback = (st_irq_handler_callback_t)_handler;
		break;
	case SDMMC1_IRQn:
		sdmmc1_handler_callback = (st_irq_handler_callback_t)_handler;
		break;
	case SPI1_IRQn:
		spi1_handler_callback = (st_irq_handler_callback_t)_handler;
		break;
	case DMA1_Channel2_IRQn:
		dma1_ch2_handler_callback = (st_irq_handler_callback_t)_handler;
		break;
	case DMA1_Channel3_IRQn:
		dma1_ch3_handler_callback = (st_irq_handler_callback_t)_handler;
		break;
	case DMA1_Channel4_IRQn:
		dma1_ch4_handler_callback = (st_irq_handler_callback_t)_handler;
		break;
	case DMA1_Channel5_IRQn:
		dma1_ch5_handler_callback = (st_irq_handler_callback_t)_handler;
		break;
	case DMA2_Channel3_IRQn:
		dma2_ch3_handler_callback = (st_irq_handler_callback_t)_handler;
		break;
	case DMA2_Channel4_IRQn:
		dma2_ch4_handler_callback = (st_irq_handler_callback_t)_handler;
		break;
	case DMA2_Channel5_IRQn:
		dma2_ch5_handler_callback = (st_irq_handler_callback_t)_handler;
		break;
	case LPTIM2_IRQn:
        lptim2_handler_callback = (st_irq_handler_callback_t)_handler;
		break;
	case RTC_WKUP_IRQn:
		rtc_wkup_handler_callback = (st_irq_handler_callback_t)_handler;
		break;
	case ADC1_2_IRQn:
		adc_handler_callback = (st_irq_handler_callback_t)_handler;
		break;
	default:
		break;
	}
}

void st_exti_irq_handler_register(uint32_t _irq_num, uint32_t _exti_line, void *_handler){
	UNUSED(_irq_num);
    if(_exti_line > 31) return;
	((*((st_irq_handler_callback_t*)(exti_line_handler_callback + _exti_line))))
			= (st_irq_handler_callback_t)_handler;
}

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();

  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  LED_ON();
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  LED_ON();
  NVIC_SystemReset();
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
   LED_ON();
   NVIC_SystemReset();
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
   LED_ON();
   NVIC_SystemReset();
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	if(systick_handler_callback)
		systick_handler_callback();
//	systick_callback();
  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RCC global interrupt.
  */
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

void RTC_Alarm_IRQHandler(void){

}

void RTC_WKUP_IRQHandler(void){
	if(LL_RTC_IsEnabledIT_WUT(RTC) && LL_RTC_IsActiveFlag_WUT(RTC)){
		LL_RTC_ClearFlag_WUT(RTC);
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20);
		if(rtc_wkup_handler_callback){
			rtc_wkup_handler_callback();
		}
	}
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */

void EXTI2_IRQHandler(void)  // GPS PPS Exit
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_2) != RESET){
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
    /* USER CODE BEGIN LL_EXTI_LINE_2 */
//    if(gps_pps_callback != NULL){
//    	gps_pps_callback();
//    }
//    st_irq_handler_callback_t _exti_callback = exti_line_handler_callback[2];
//    if(_exti_callback){
//    	_exti_callback();
//    }
    if(*(st_irq_handler_callback_t*)(exti_line_handler_callback + 2))
    	(*(st_irq_handler_callback_t*)(exti_line_handler_callback + 2))();
    /* USER CODE END LL_EXTI_LINE_2 */
  }
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_3) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);
    /* USER CODE BEGIN LL_EXTI_LINE_3 */
//    usb_dection_exti_callback();
//    st_irq_handler_callback_t _exti_callback = exti_line_handler_callback[3];
//    if(_exti_callback){
//    	_exti_callback();
//    }
    if(*(st_irq_handler_callback_t*)(exti_line_handler_callback + 3))
    	(*(st_irq_handler_callback_t*)(exti_line_handler_callback + 3))();
    /* USER CODE END LL_EXTI_LINE_3 */
  }
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
    /* USER CODE BEGIN LL_EXTI_LINE_4 */
//    st_irq_handler_callback_t _exti_callback = exti_line_handler_callback[4];
//    if(_exti_callback){
//    	_exti_callback();
//    }
    if(*(st_irq_handler_callback_t*)(exti_line_handler_callback + 4))
    	(*(st_irq_handler_callback_t*)(exti_line_handler_callback + 4))();
    /* USER CODE END LL_EXTI_LINE_4 */
  }
  /* USER CODE BEGIN EXTI4_IRQn 1 */


  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 interrupts.
  */
void DMA1_Channel1_IRQHandler(void){
//    uint32_t request = 0;
//    if(LL_DMA_IsActiveFlag_TC1(DMA1)){
//    	LL_DMA_ClearFlag_GI1(DMA1);
//    	request = 1;
//    }else if(LL_DMA_IsActiveFlag_TE1(DMA1)){
//    	LL_DMA_ClearFlag_TE1(DMA1);
//    	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
//        request = 2;
//    }
//    if(request == 1){
//        request = LL_DMA_GetPeriphRequest(DMA1, LL_DMA_CHANNEL_1);
//        switch(request){
//#if(ADC_USE_DMA)
//        case LL_DMA_REQUEST_0:  // adc dma request
//            adc1_dma_rxcplt_callback();
//        	break;
//#endif
//        }
//    }
//    if(request == 2){
//        request = LL_DMA_GetPeriphRequest(DMA1, LL_DMA_CHANNEL_1);
//        switch(request){
//#if(ADC_USE_DMA)
//        case LL_DMA_REQUEST_0:  // adc dma request
//            adc1_dma_err_callback();
//        	break;
//#endif
//        }
//    }
	if(dma1_ch1_handler_callback == NULL){

	}else{
		dma1_ch1_handler_callback();
	}
}


/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
//	uint32_t request = 0;
	if(dma1_ch2_handler_callback == NULL){

		return;
	}else{
		dma1_ch2_handler_callback();
	}

  /* USER CODE END DMA1_Channel2_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */
//	uint32_t request = 0;

    if(dma1_ch3_handler_callback == NULL){
        return;
    }else{
    	dma1_ch3_handler_callback();
    }

  /* USER CODE END DMA1_Channel3_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

	if(dma1_ch4_handler_callback)
		dma1_ch4_handler_callback();
  /* USER CODE END DMA1_Channel4_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

	if(dma1_ch5_handler_callback)
		dma1_ch5_handler_callback();
  /* USER CODE END DMA1_Channel5_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
  if(adc_handler_callback) adc_handler_callback();

  /* USER CODE END ADC1_2_IRQn 0 */

  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}


/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
    uint32_t _exti_pr = EXTI->PR1;
  /* USER CODE END EXTI9_5_IRQn 0 */
//  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_7) != RESET)
    if(_exti_pr & LL_EXTI_LINE_7)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_7);
    /* USER CODE BEGIN LL_EXTI_LINE_7 */
//    st_irq_handler_callback_t _exti_callback = exti_line_handler_callback[7];
//    if(_exti_callback){
//    	_exti_callback();
//    }
    if(*(st_irq_handler_callback_t*)(exti_line_handler_callback + 7))
    	(*(st_irq_handler_callback_t*)(exti_line_handler_callback + 7))();
    /* USER CODE END LL_EXTI_LINE_7 */
  }
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

void TIM1_IRQHandler(void){

}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
#if(LL_SPI)
	register uint32_t _sr = SPI1->SR;
	if(_sr & (SPI_SR_CRCERR | SPI_SR_MODF | SPI_SR_OVR | SPI_SR_FRE)){
		if(_sr & SPI_SR_CRCERR) LL_SPI_ClearFlag_CRCERR(SPI1);
		if(_sr & SPI_SR_FRE) LL_SPI_ClearFlag_FRE(SPI1);
		if(_sr & SPI_SR_MODF) LL_SPI_ClearFlag_MODF(SPI1);
		if(_sr & SPI_SR_OVR) LL_SPI_ClearFlag_OVR(SPI1);
	}
#else
    spi1_handler_callback();
#endif

  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	uint32_t _exti_pr = EXTI->PR1;
  /* USER CODE END EXTI15_10_IRQn 0 */
//	if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_12) != RESET)
	if(_exti_pr & LL_EXTI_LINE_12)
	{

		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
//		imu_exti_callback();
//	    st_irq_handler_callback_t _exti_callback = exti_line_handler_callback[12];
//	    if(_exti_callback){
//	    	_exti_callback();
//	    }
	    if(*(st_irq_handler_callback_t*)(exti_line_handler_callback + 12))
	    	(*(st_irq_handler_callback_t*)(exti_line_handler_callback + 12))();
	}
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13) != RESET){
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
    /* USER CODE BEGIN LL_EXTI_LINE_13 */

    /* USER CODE END LL_EXTI_LINE_13 */
  }
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles SDMMC1 global interrupt.
  */
void SDMMC1_IRQHandler(void)
{
  /* USER CODE BEGIN SDMMC1_IRQn 0 */

  /* USER CODE END SDMMC1_IRQn 0 */
	if(sdmmc1_handler_callback) sdmmc1_handler_callback();
  /* USER CODE BEGIN SDMMC1_IRQn 1 */

  /* USER CODE END SDMMC1_IRQn 1 */
}

///**
//  * @brief This function handles UART4 global interrupt.
//  */
//void UART4_IRQHandler(void)
//{
//  /* USER CODE BEGIN UART4_IRQn 0 */
//
//  /* USER CODE END UART4_IRQn 0 */
//  HAL_UART_IRQHandler(&huart4);
//  /* USER CODE BEGIN UART4_IRQn 1 */
//
//  /* USER CODE END UART4_IRQn 1 */
//}

void DMA2_Channel3_IRQHandler(void){
//	register uint32_t request = 0;
//	if(LL_DMA_IsActiveFlag_TC3(DMA2)){
//	    LL_DMA_ClearFlag_GI3(DMA2);
//	    request = 1;
//	/* Call function Transmission complete Callback */
//	}else if(LL_DMA_IsActiveFlag_TE3(DMA2)){
//		LL_DMA_ClearFlag_TE3(DMA1);
//		LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_3);
//	}
//	if(request == 1){
//		request = LL_DMA_GetPeriphRequest(DMA2, LL_DMA_CHANNEL_3);
//		switch(request){
//		case LL_DMA_REQUEST_1:
//			break;
//		case LL_DMA_REQUEST_2:
//			uart4_dma_txcplt_callback();
//			break;
//		}
//	}
	if(dma2_ch3_handler_callback == NULL){

	}else{
		dma2_ch3_handler_callback();
	}

}

/**
  * @brief This function handles DMA2 channel4 global interrupt.
  */
void DMA2_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel4_IRQn 0 */
//	register uint32_t request = 0;
//	register uint32_t direction = 0;
//	request = LL_DMA_GetPeriphRequest(DMA2, LL_DMA_CHANNEL_4);
//	switch(request){
//		case LL_DMA_REQUEST_7:
//            direction = LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_CHANNEL_4);
//            if(direction == LL_DMA_DIRECTION_PERIPH_TO_MEMORY){
//            	sdmmc1_dma_rxcplt_callback();
//            }
//            if(direction == LL_DMA_DIRECTION_MEMORY_TO_PERIPH){
//            	sdmmc1_dma_txcplt_callback();
//            }
//			break;
//		default:
//			break;
//	}
	if(dma2_ch4_handler_callback) dma2_ch4_handler_callback();
  /* USER CODE END DMA2_Channel4_IRQn 0 */

  /* USER CODE BEGIN DMA2_Channel4_IRQn 1 */

  /* USER CODE END DMA2_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel5 global interrupt.
  */
void DMA2_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel5_IRQn 0 */
    if(dma2_ch5_handler_callback) dma2_ch5_handler_callback();
  /* USER CODE END DMA2_Channel5_IRQn 0 */

  /* USER CODE BEGIN DMA2_Channel5_IRQn 1 */

  /* USER CODE END DMA2_Channel5_IRQn 1 */
}

void LPTIM1_IRQHandler(void)
{
  /* USER CODE BEGIN LPTIM1_IRQn 0 */
	if(lptim1_handler_callback) lptim1_handler_callback();
  /* USER CODE END LPTIM1_IRQn 0 */

  /* USER CODE BEGIN LPTIM1_IRQn 1 */

  /* USER CODE END LPTIM1_IRQn 1 */
}

/**
  * @brief This function handles LPTIM2 global interrupt.
  */
void LPTIM2_IRQHandler(void)
{
  /* USER CODE BEGIN LPTIM2_IRQn 0 */
	if(lptim2_handler_callback) lptim2_handler_callback();
  /* USER CODE END LPTIM2_IRQn 0 */

  /* USER CODE BEGIN LPTIM2_IRQn 1 */

  /* USER CODE END LPTIM2_IRQn 1 */
}


/**
  * @brief This function handles FPU global interrupt.
  */
void FPU_IRQHandler(void)
{
  /* USER CODE BEGIN FPU_IRQn 0 */

  /* USER CODE END FPU_IRQn 0 */
  /* USER CODE BEGIN FPU_IRQn 1 */

  /* USER CODE END FPU_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
