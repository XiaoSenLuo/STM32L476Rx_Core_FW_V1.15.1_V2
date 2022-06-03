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


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */



#include "stm32l4xx_ll_exti.h"

#include "stm32l4xx_it.h"
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


static isr_handle_def_t core_isr_handler[14] = {
        {.ctx = NULL, .isr_func_handle = NULL},
        {.ctx = NULL, .isr_func_handle = NULL},
        {.ctx = NULL, .isr_func_handle = NULL},
        {.ctx = NULL, .isr_func_handle = NULL},
        {.ctx = NULL, .isr_func_handle = NULL},
        {.ctx = NULL, .isr_func_handle = NULL},
        {.ctx = NULL, .isr_func_handle = NULL},
        {.ctx = NULL, .isr_func_handle = NULL},
        {.ctx = NULL, .isr_func_handle = NULL},
        {.ctx = NULL, .isr_func_handle = NULL},
        {.ctx = NULL, .isr_func_handle = NULL},
        {.ctx = NULL, .isr_func_handle = NULL},
        {.ctx = NULL, .isr_func_handle = NULL},
        {.ctx = NULL, .isr_func_handle = NULL},
};

static isr_handle_def_t peripheral_isr_handler[82] = {
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
};

static isr_handle_def_t gpio_exti_handler[16] = {
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
		{.ctx = NULL, .isr_func_handle = NULL},
        {.ctx = NULL, .isr_func_handle = NULL},
};

void ll_core_isr_install(IRQn_Type_t _irq_num, isr_function_handle_t fn, void *ctx){
    if((_irq_num >= 0) || (_irq_num < -14)) return;
    int irq_num = (int)_irq_num * -1;
    core_isr_handler[irq_num].ctx = ctx;
    core_isr_handler[irq_num].isr_func_handle = fn;
}

void ll_core_isr_uninstall(IRQn_Type_t _irq_num){
    if((_irq_num >= 0) || (_irq_num < -14)) return;
    int irq_num = (int)_irq_num * -1;
    core_isr_handler[irq_num].ctx = NULL;
    core_isr_handler[irq_num].isr_func_handle = NULL;
}

void ll_peripheral_isr_install(IRQn_Type_t _irq_num, isr_function_handle_t fn, void *ctx){
    if((_irq_num < 0) || (_irq_num > 81)) return;

    peripheral_isr_handler[_irq_num].ctx = ctx;
    peripheral_isr_handler[_irq_num].isr_func_handle = fn;
}

void ll_peripheral_isr_uninstall(IRQn_Type_t _irq_num){
    if((_irq_num < 0) || (_irq_num > 81)) return;

    peripheral_isr_handler[_irq_num].ctx = NULL;
    peripheral_isr_handler[_irq_num].isr_func_handle = NULL;
}

int gpio_mask2num(uint32_t mask){
    int i = 0;
    do{
        if(((mask)) & (1UL << i)){
            return i;
        }
        i += 1;
    }while(i < 16);
    return -1;
}

void ll_gpio_exti_isr_install(gpio_num_t gpio, isr_function_handle_t fn, void *ctx){
    if((gpio < 0) || (gpio > 15)) return;

    gpio_exti_handler[gpio].ctx = ctx;
    gpio_exti_handler[gpio].isr_func_handle = fn;
}

void ll_gpio_exti_isr_uninstall(gpio_num_t gpio){
    if((gpio < 0) || (gpio > 15)) return;

    gpio_exti_handler[gpio].ctx = NULL;
    gpio_exti_handler[gpio].isr_func_handle = NULL;
}

static const int GPIO_IRQn[16] = {
        EXTI0_IRQn,
        EXTI1_IRQn,
        EXTI2_IRQn,
        EXTI3_IRQn,
        EXTI4_IRQn,
        EXTI9_5_IRQn,
        EXTI9_5_IRQn,
        EXTI9_5_IRQn,
        EXTI9_5_IRQn,
        EXTI9_5_IRQn,
        EXTI15_10_IRQn,
        EXTI15_10_IRQn,
        EXTI15_10_IRQn,
        EXTI15_10_IRQn,
        EXTI15_10_IRQn,
        EXTI15_10_IRQn,
};

int gpio_get_irqn(gpio_num_t gpio_num){
    return GPIO_IRQn[gpio_num];
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/


/* USER CODE BEGIN EV */

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

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
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
	NVIC_SystemReset();
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
    if(core_isr_handler[SVCall_IRQn].isr_func_handle == NULL) return;
    core_isr_handler[SVCall_IRQn].isr_func_handle(core_isr_handler[SVCall_IRQn].ctx);
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
    if(core_isr_handler[DebugMonitor_IRQn].isr_func_handle == NULL) return;
    core_isr_handler[DebugMonitor_IRQn].isr_func_handle(core_isr_handler[DebugMonitor_IRQn].ctx);
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
    if(core_isr_handler[PendSV_IRQn].isr_func_handle == NULL) return;
    core_isr_handler[PendSV_IRQn].isr_func_handle(core_isr_handler[PendSV_IRQn].ctx);
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

  /* USER CODE END SysTick_IRQn 0 */
  if(core_isr_handler[SysTick_IRQn].isr_func_handle == NULL) return;
  core_isr_handler[SysTick_IRQn].isr_func_handle(core_isr_handler[SysTick_IRQn].ctx);
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt peripheral_isr_handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

void WWDG_IRQHandler(void){
    if(peripheral_isr_handler[WWDG_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[WWDG_IRQn].isr_func_handle(peripheral_isr_handler[WWDG_IRQn].ctx);
}

void PVD_PVM_IRQHandler(void){
    if(peripheral_isr_handler[PVD_PVM_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[PVD_PVM_IRQn].isr_func_handle(peripheral_isr_handler[PVD_PVM_IRQn].ctx);
}

void TAMP_STAMP_IRQHandler(void){
    if(peripheral_isr_handler[TAMP_STAMP_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[TAMP_STAMP_IRQn].isr_func_handle(peripheral_isr_handler[TAMP_STAMP_IRQn].ctx);
}

void RTC_WKUP_IRQHandler(void){
    if(peripheral_isr_handler[RTC_WKUP_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[RTC_WKUP_IRQn].isr_func_handle(peripheral_isr_handler[RTC_WKUP_IRQn].ctx);
}

void FLASH_IRQHandler(void){
    if(peripheral_isr_handler[FLASH_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[FLASH_IRQn].isr_func_handle(peripheral_isr_handler[FLASH_IRQn].ctx);
}

/**
  * @brief This function handles RCC global interrupt.
  */
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */
  if(peripheral_isr_handler[RCC_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[RCC_IRQn].isr_func_handle(peripheral_isr_handler[RCC_IRQn].ctx);
  /* USER CODE END RCC_IRQn 1 */
}

void EXTI0_IRQHandler(void){
    if((gpio_exti_handler[0].isr_func_handle == NULL) || (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0) == RESET)) return;

    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
    gpio_exti_handler[0].isr_func_handle(gpio_exti_handler[0].ctx);
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */

void EXTI1_IRQHandler(void){
    if((gpio_exti_handler[1].isr_func_handle == NULL) || (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_1) == RESET)) return;

    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    gpio_exti_handler[1].isr_func_handle(gpio_exti_handler[1].ctx);
}

void EXTI2_IRQHandler(void)  // GPS PPS Exit
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */

    /* USER CODE END LL_EXTI_LINE_2 */

  /* USER CODE BEGIN EXTI2_IRQn 1 */
    if((gpio_exti_handler[2].isr_func_handle == NULL) || (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_2) == RESET)) return;
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
    gpio_exti_handler[2].isr_func_handle(gpio_exti_handler[2].ctx);
  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */

    /* USER CODE END LL_EXTI_LINE_3 */

  /* USER CODE BEGIN EXTI3_IRQn 1 */
    if((gpio_exti_handler[3].isr_func_handle == NULL) || (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_3) == RESET)) return;
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);
    gpio_exti_handler[3].isr_func_handle(gpio_exti_handler[3].ctx);
  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
    /* USER CODE END LL_EXTI_LINE_4 */

  /* USER CODE BEGIN EXTI4_IRQn 1 */
    if((gpio_exti_handler[4].isr_func_handle == NULL) || (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4) == RESET)) return;
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
    gpio_exti_handler[4].isr_func_handle(gpio_exti_handler[4].ctx);

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 interrupts.
  */
void DMA1_Channel1_IRQHandler(void){
	if(peripheral_isr_handler[DMA1_Channel1_IRQn].isr_func_handle == NULL) return;

	peripheral_isr_handler[DMA1_Channel1_IRQn].isr_func_handle(peripheral_isr_handler[DMA1_Channel1_IRQn].ctx);
}


/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void){
  /* USER CODE END DMA1_Channel2_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */
	if(peripheral_isr_handler[DMA1_Channel2_IRQn].isr_func_handle == NULL) return;

	peripheral_isr_handler[DMA1_Channel2_IRQn].isr_func_handle(peripheral_isr_handler[DMA1_Channel2_IRQn].ctx);
  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void){
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */
	if(peripheral_isr_handler[DMA1_Channel3_IRQn].isr_func_handle == NULL) return;

	peripheral_isr_handler[DMA1_Channel3_IRQn].isr_func_handle(peripheral_isr_handler[DMA1_Channel3_IRQn].ctx);
  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void){
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */
	if(peripheral_isr_handler[DMA1_Channel4_IRQn].isr_func_handle == NULL) return;

	peripheral_isr_handler[DMA1_Channel4_IRQn].isr_func_handle(peripheral_isr_handler[DMA1_Channel4_IRQn].ctx);
  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void){
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */
	if(peripheral_isr_handler[DMA1_Channel5_IRQn].isr_func_handle == NULL) return;

	peripheral_isr_handler[DMA1_Channel5_IRQn].isr_func_handle(peripheral_isr_handler[DMA1_Channel5_IRQn].ctx);
  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void){
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */
	if(peripheral_isr_handler[DMA1_Channel6_IRQn].isr_func_handle == NULL) return;

	peripheral_isr_handler[DMA1_Channel6_IRQn].isr_func_handle(peripheral_isr_handler[DMA1_Channel6_IRQn].ctx);
  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void){
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */
	if(peripheral_isr_handler[DMA1_Channel7_IRQn].isr_func_handle == NULL) return;

	peripheral_isr_handler[DMA1_Channel7_IRQn].isr_func_handle(peripheral_isr_handler[DMA1_Channel7_IRQn].ctx);
  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 interrupts.
  */
void ADC1_2_IRQHandler(void){
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */

  /* USER CODE BEGIN ADC1_2_IRQn 1 */
    if(peripheral_isr_handler[ADC1_2_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[ADC1_2_IRQn].isr_func_handle(peripheral_isr_handler[ADC1_2_IRQn].ctx);
  /* USER CODE END ADC1_2_IRQn 1 */
}

void CAN1_TX_IRQhandler(void){

}

void CAN1_RX0_IRQHandler(void){


}

void CAN1_RX1_IRQHandler(void){

}

void CAN1_SCE_IRQHandler(void){

}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void){
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */

  /* USER CODE END LL_EXTI_LINE_7 */

  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
    register uint32_t exti_pr = EXTI->PR1;
    if((exti_pr & LL_EXTI_LINE_5) && (gpio_exti_handler[5].isr_func_handle)){
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_5);
    	gpio_exti_handler[5].isr_func_handle(gpio_exti_handler[5].ctx);
        return;
    }
    if((exti_pr & LL_EXTI_LINE_6) && (gpio_exti_handler[6].isr_func_handle)){
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_6);
    	gpio_exti_handler[6].isr_func_handle(gpio_exti_handler[6].ctx);
        return;
    }
    if((exti_pr & LL_EXTI_LINE_7) && (gpio_exti_handler[7].isr_func_handle)){
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_7);
    	gpio_exti_handler[7].isr_func_handle(gpio_exti_handler[7].ctx);
        return;
    }
    if((exti_pr & LL_EXTI_LINE_8) && (gpio_exti_handler[8].isr_func_handle)){
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
    	gpio_exti_handler[8].isr_func_handle(gpio_exti_handler[8].ctx);
        return;
    }
    if((exti_pr & LL_EXTI_LINE_9) && (gpio_exti_handler[9].isr_func_handle)){
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);
    	gpio_exti_handler[9].isr_func_handle(gpio_exti_handler[9].ctx);
        return;
    }
  /* USER CODE END EXTI9_5_IRQn 1 */
}

void TIM1_BRK_TIM15_IRQHandler(void){

}
void TIM1_UP_TIM16_IRQHandler(void){

}
void TIM1_TRG_COM_TIM17_IRQHandler(void){

}
void TIM1_CC_IRQHandler(void){

}

void TIM2_IRQHandler(void)
{
    /* USER CODE BEGIN TIM2_IRQn 0 */

    /* USER CODE END TIM2_IRQn 0 */

    /* USER CODE BEGIN TIM2_IRQn 1 */
    if(peripheral_isr_handler[TIM2_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[TIM2_IRQn].isr_func_handle(peripheral_isr_handler[TIM2_IRQn].ctx);
    /* USER CODE END TIM2_IRQn 1 */
}


void TIM3_IRQHandler(void){
    if(peripheral_isr_handler[TIM3_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[TIM3_IRQn].isr_func_handle(peripheral_isr_handler[TIM3_IRQn].ctx);
}


void TIM4_IRQHandler(void){
    if(peripheral_isr_handler[TIM4_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[TIM4_IRQn].isr_func_handle(peripheral_isr_handler[TIM4_IRQn].ctx);
}

void I2C1_EV_IRQHandler(void){
    if(peripheral_isr_handler[I2C1_EV_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[I2C1_EV_IRQn].isr_func_handle(peripheral_isr_handler[I2C1_EV_IRQn].ctx);
}

void I2C1_ER_IRQHandler(void){
    if(peripheral_isr_handler[I2C1_ER_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[I2C1_ER_IRQn].isr_func_handle(peripheral_isr_handler[I2C1_ER_IRQn].ctx);
}

void I2C2_EV_IRQHandler(void){
    if(peripheral_isr_handler[I2C2_EV_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[I2C2_EV_IRQn].isr_func_handle(peripheral_isr_handler[I2C2_EV_IRQn].ctx);
}

void I2C2_ER_IRQHandler(void){
    if(peripheral_isr_handler[I2C2_ER_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[I2C2_ER_IRQn].isr_func_handle(peripheral_isr_handler[I2C2_ER_IRQn].ctx);
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */

  /* USER CODE BEGIN SPI1_IRQn 1 */
    if(peripheral_isr_handler[SPI1_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[SPI1_IRQn].isr_func_handle(peripheral_isr_handler[SPI1_IRQn].ctx);
  /* USER CODE END SPI1_IRQn 1 */
}

void SPI2_IRQHandler(void)
{
    /* USER CODE BEGIN SPI1_IRQn 0 */

    /* USER CODE END SPI1_IRQn 0 */

    /* USER CODE BEGIN SPI1_IRQn 1 */
    if(peripheral_isr_handler[SPI2_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[SPI2_IRQn].isr_func_handle(peripheral_isr_handler[SPI2_IRQn].ctx);
    /* USER CODE END SPI1_IRQn 1 */
}

void USART1_IRQHandler(void){
    if(peripheral_isr_handler[USART1_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[USART1_IRQn].isr_func_handle(peripheral_isr_handler[USART1_IRQn].ctx);
}

void USART2_IRQHandler(void){
    if(peripheral_isr_handler[USART2_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[USART2_IRQn].isr_func_handle(peripheral_isr_handler[USART2_IRQn].ctx);
}

void USART3_IRQHandler(void){
    if(peripheral_isr_handler[USART3_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[USART3_IRQn].isr_func_handle(peripheral_isr_handler[USART3_IRQn].ctx);
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void){
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */

  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
    register uint32_t exti_pr = EXTI->PR1;
    if((exti_pr & LL_EXTI_LINE_10) && (gpio_exti_handler[10].isr_func_handle)){
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
    	gpio_exti_handler[10].isr_func_handle(gpio_exti_handler[10].ctx);
        return;
    }
    if((exti_pr & LL_EXTI_LINE_11) && (gpio_exti_handler[11].isr_func_handle)){
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_11);
    	gpio_exti_handler[11].isr_func_handle(gpio_exti_handler[11].ctx);
        return;
    }
    if((exti_pr & LL_EXTI_LINE_12) && (gpio_exti_handler[12].isr_func_handle)){
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
    	gpio_exti_handler[12].isr_func_handle(gpio_exti_handler[12].ctx);
        return;
    }
    if((exti_pr & LL_EXTI_LINE_13) && (gpio_exti_handler[13].isr_func_handle)){
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
    	gpio_exti_handler[13].isr_func_handle(gpio_exti_handler[13].ctx);
        return;
    }
    if((exti_pr & LL_EXTI_LINE_14) && (gpio_exti_handler[14].isr_func_handle)){
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_14);
    	gpio_exti_handler[14].isr_func_handle(gpio_exti_handler[14].ctx);
        return;
    }
    if((exti_pr & LL_EXTI_LINE_15) && (gpio_exti_handler[15].isr_func_handle)){
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
    	gpio_exti_handler[15].isr_func_handle(gpio_exti_handler[15].ctx);
        return;
    }
  /* USER CODE END EXTI15_10_IRQn 1 */
}

void RTC_Alarm_IRQHandler(void){

    if(peripheral_isr_handler[RTC_Alarm_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[RTC_Alarm_IRQn].isr_func_handle(peripheral_isr_handler[RTC_Alarm_IRQn].ctx);
}

void DFSDM1_FLT3_IRQHandler(void){
    if(peripheral_isr_handler[DFSDM1_FLT3_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[DFSDM1_FLT3_IRQn].isr_func_handle(peripheral_isr_handler[DFSDM1_FLT3_IRQn].ctx);
}

void TIM8_BRK_IRQHandler(void){
    if(peripheral_isr_handler[TIM8_BRK_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[TIM8_BRK_IRQn].isr_func_handle(peripheral_isr_handler[TIM8_BRK_IRQn].ctx);
}

void TIM8_UP_IRQHandler(void){
    if(peripheral_isr_handler[TIM8_UP_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[TIM8_UP_IRQn].isr_func_handle(peripheral_isr_handler[TIM8_UP_IRQn].ctx);
}

void TIM8_TRG_COM_IRQHandler(void){
    if(peripheral_isr_handler[TIM8_TRG_COM_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[TIM8_TRG_COM_IRQn].isr_func_handle(peripheral_isr_handler[TIM8_TRG_COM_IRQn].ctx);
}

void TIM8_CC_IRQHandler(void){
    if(peripheral_isr_handler[TIM8_CC_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[TIM8_CC_IRQn].isr_func_handle(peripheral_isr_handler[TIM8_CC_IRQn].ctx);
}

void ADC3_IRQHandler(void){
    if(peripheral_isr_handler[ADC3_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[ADC3_IRQn].isr_func_handle(peripheral_isr_handler[ADC3_IRQn].ctx);
}

void FMC_IRQHandler(void){
    if(peripheral_isr_handler[FMC_IRQn].isr_func_handle == NULL) return;

    peripheral_isr_handler[FMC_IRQn].isr_func_handle(peripheral_isr_handler[FMC_IRQn].ctx);
}

/**
  * @brief This function handles SDMMC1 global interrupt.
  */
void SDMMC1_IRQHandler(void)
{
  /* USER CODE BEGIN SDMMC1_IRQn 0 */

  /* USER CODE END SDMMC1_IRQn 0 */
  /* USER CODE BEGIN SDMMC1_IRQn 1 */
    if(peripheral_isr_handler[SDMMC1_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[SDMMC1_IRQn].isr_func_handle(peripheral_isr_handler[SDMMC1_IRQn].ctx);
  /* USER CODE END SDMMC1_IRQn 1 */
}

void TIM5_IRQHandler(void){
    if(peripheral_isr_handler[TIM5_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[TIM5_IRQn].isr_func_handle(peripheral_isr_handler[TIM5_IRQn].ctx);
}

void SPI3_IRQHandler(void){
    if(peripheral_isr_handler[SPI3_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[SPI3_IRQn].isr_func_handle(peripheral_isr_handler[SPI3_IRQn].ctx);
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  /* USER CODE BEGIN UART4_IRQn 1 */
    if(peripheral_isr_handler[UART4_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[UART4_IRQn].isr_func_handle(peripheral_isr_handler[UART4_IRQn].ctx);
  /* USER CODE END UART4_IRQn 1 */
}

void UART5_IRQHandler(void){
    if(peripheral_isr_handler[UART5_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[UART5_IRQn].isr_func_handle(peripheral_isr_handler[UART5_IRQn].ctx);
}

void TIM6_DAC_IRQHandler(void){
    if(peripheral_isr_handler[TIM6_DAC_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[TIM6_DAC_IRQn].isr_func_handle(peripheral_isr_handler[TIM6_DAC_IRQn].ctx);
}

void TIM7_IRQHandler(void){
    if(peripheral_isr_handler[TIM7_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[TIM7_IRQn].isr_func_handle(peripheral_isr_handler[TIM7_IRQn].ctx);
}

void DMA2_Channel1_IRQHandler(void){
    if(peripheral_isr_handler[DMA2_Channel1_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[DMA2_Channel1_IRQn].isr_func_handle(peripheral_isr_handler[DMA2_Channel1_IRQn].ctx);
}

void DMA2_Channel2_IRQHandler(void){
    if(peripheral_isr_handler[DMA2_Channel2_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[DMA2_Channel2_IRQn].isr_func_handle(peripheral_isr_handler[DMA2_Channel2_IRQn].ctx);
}

void DMA2_Channel3_IRQHandler(void){
    if(peripheral_isr_handler[DMA2_Channel3_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[DMA2_Channel3_IRQn].isr_func_handle(peripheral_isr_handler[DMA2_Channel3_IRQn].ctx);
}

/**
  * @brief This function handles DMA2 channel4 global interrupt.
  */
void DMA2_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel4_IRQn 0 */

  /* USER CODE END DMA2_Channel4_IRQn 0 */

  /* USER CODE BEGIN DMA2_Channel4_IRQn 1 */
    if(peripheral_isr_handler[DMA2_Channel4_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[DMA2_Channel4_IRQn].isr_func_handle(peripheral_isr_handler[DMA2_Channel4_IRQn].ctx);
  /* USER CODE END DMA2_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel5 global interrupt.
  */
void DMA2_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel5_IRQn 0 */

  /* USER CODE END DMA2_Channel5_IRQn 0 */

  /* USER CODE BEGIN DMA2_Channel5_IRQn 1 */
    if(peripheral_isr_handler[DMA2_Channel5_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[DMA2_Channel5_IRQn].isr_func_handle(peripheral_isr_handler[DMA2_Channel5_IRQn].ctx);
  /* USER CODE END DMA2_Channel5_IRQn 1 */
}

void DFSDM1_FLT0_IRQHandler(void){
    if(peripheral_isr_handler[DFSDM1_FLT0_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[DFSDM1_FLT0_IRQn].isr_func_handle(peripheral_isr_handler[DFSDM1_FLT0_IRQn].ctx);
}

void DFSDM1_FLT1_IRQHandler(void){
    if(peripheral_isr_handler[DFSDM1_FLT1_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[DFSDM1_FLT1_IRQn].isr_func_handle(peripheral_isr_handler[DFSDM1_FLT1_IRQn].ctx);
}

void DFSDM1_FLT2_IRQHandler(void){
    if(peripheral_isr_handler[DFSDM1_FLT2_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[DFSDM1_FLT2_IRQn].isr_func_handle(peripheral_isr_handler[DFSDM1_FLT2_IRQn].ctx);
}

void COMP_IRQHandler(void){
    if(peripheral_isr_handler[COMP_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[COMP_IRQn].isr_func_handle(peripheral_isr_handler[COMP_IRQn].ctx);
}

void LPTIM1_IRQHandler(void)
{
  /* USER CODE BEGIN LPTIM1_IRQn 0 */

  /* USER CODE END LPTIM1_IRQn 0 */

  /* USER CODE BEGIN LPTIM1_IRQn 1 */
    if(peripheral_isr_handler[LPTIM1_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[LPTIM1_IRQn].isr_func_handle(peripheral_isr_handler[LPTIM1_IRQn].ctx);
  /* USER CODE END LPTIM1_IRQn 1 */
}

/**
  * @brief This function handles LPTIM2 global interrupt.
  */
void LPTIM2_IRQHandler(void)
{
  /* USER CODE BEGIN LPTIM2_IRQn 0 */

  /* USER CODE END LPTIM2_IRQn 0 */

  /* USER CODE BEGIN LPTIM2_IRQn 1 */
    if(peripheral_isr_handler[LPTIM2_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[LPTIM2_IRQn].isr_func_handle(peripheral_isr_handler[LPTIM2_IRQn].ctx);
  /* USER CODE END LPTIM2_IRQn 1 */
}



/* USER CODE BEGIN 1 */

/**
  * @brief  This function handles USB-On-The-Go FS global interrupt request.
  * @param  None
  * @retval None
  */
void OTG_FS_IRQHandler(void)
{
    if(peripheral_isr_handler[OTG_FS_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[OTG_FS_IRQn].isr_func_handle(peripheral_isr_handler[OTG_FS_IRQn].ctx);
}

void DMA2_Channel6_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Channel5_IRQn 0 */

    /* USER CODE END DMA2_Channel5_IRQn 0 */

    /* USER CODE BEGIN DMA2_Channel5_IRQn 1 */
    if(peripheral_isr_handler[DMA2_Channel6_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[DMA2_Channel6_IRQn].isr_func_handle(peripheral_isr_handler[DMA2_Channel6_IRQn].ctx);
    /* USER CODE END DMA2_Channel5_IRQn 1 */
}

void DMA2_Channel7_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Channel5_IRQn 0 */

    /* USER CODE END DMA2_Channel5_IRQn 0 */

    /* USER CODE BEGIN DMA2_Channel5_IRQn 1 */
    if(peripheral_isr_handler[DMA2_Channel7_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[DMA2_Channel7_IRQn].isr_func_handle(peripheral_isr_handler[DMA2_Channel7_IRQn].ctx);
    /* USER CODE END DMA2_Channel5_IRQn 1 */
}


void LPUART1_IRQHandler(void){
    if(peripheral_isr_handler[LPUART1_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[LPUART1_IRQn].isr_func_handle(peripheral_isr_handler[LPUART1_IRQn].ctx);
}

void QUADSPI_IRQHandler(void){
    if(peripheral_isr_handler[QUADSPI_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[QUADSPI_IRQn].isr_func_handle(peripheral_isr_handler[QUADSPI_IRQn].ctx);
}

void I2C3_EV_IRQHandler(void){
    if(peripheral_isr_handler[I2C3_EV_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[I2C3_EV_IRQn].isr_func_handle(peripheral_isr_handler[I2C3_EV_IRQn].ctx);
}

void I2C3_ER_IRQHandler(void){
    if(peripheral_isr_handler[I2C3_ER_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[I2C3_ER_IRQn].isr_func_handle(peripheral_isr_handler[I2C3_ER_IRQn].ctx);
}

void SAI1_IRQHandler(void){
    if(peripheral_isr_handler[SAI1_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[SAI1_IRQn].isr_func_handle(peripheral_isr_handler[SAI1_IRQn].ctx);
}

void SAI2_IRQHandler(void){
    if(peripheral_isr_handler[SAI2_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[SAI2_IRQn].isr_func_handle(peripheral_isr_handler[SAI2_IRQn].ctx);
}

void SWPMI1_IRQHandler(void){
    if(peripheral_isr_handler[SWPMI1_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[SWPMI1_IRQn].isr_func_handle(peripheral_isr_handler[SWPMI1_IRQn].ctx);
}

void TSC_IRQHandler(void){
    if(peripheral_isr_handler[TSC_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[TSC_IRQn].isr_func_handle(peripheral_isr_handler[TSC_IRQn].ctx);
}

void LCD_IRQHandler(void){
    if(peripheral_isr_handler[LCD_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[LCD_IRQn].isr_func_handle(peripheral_isr_handler[LCD_IRQn].ctx);
}

void RNG_IRQHandler(void){
    if(peripheral_isr_handler[RNG_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[RNG_IRQn].isr_func_handle(peripheral_isr_handler[RNG_IRQn].ctx);
}

/**
  * @brief This function handles FPU global interrupt.
  */
void FPU_IRQHandler(void)
{
    /* USER CODE BEGIN FPU_IRQn 0 */

    /* USER CODE END FPU_IRQn 0 */
    /* USER CODE BEGIN FPU_IRQn 1 */
    if(peripheral_isr_handler[FPU_IRQn].isr_func_handle == NULL) return;
    peripheral_isr_handler[FPU_IRQn].isr_func_handle(peripheral_isr_handler[FPU_IRQn].ctx);
    /* USER CODE END FPU_IRQn 1 */
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
