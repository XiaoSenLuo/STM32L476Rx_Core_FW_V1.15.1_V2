//
// Created by XIAO on 2022/5/26.
//

#include "lptim.h"

static LPTIM_HandleTypeDef hlptim1 = { 0 };
static uint16_t pulse_num = 0, index = 0;

static void lptim1_isr_handler(void *ctx){
    LPTIM_HandleTypeDef *handle = (LPTIM_HandleTypeDef *)ctx;

}

void lptim1_initialize(LPTIM_HandleTypeDef* *lptim_handle, isr_function_handle_t fn, void *ctx){
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    uint32_t prio = 0;

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
    PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSE;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    __HAL_RCC_LPTIM1_CLK_ENABLE();

    hlptim1.Instance = LPTIM1;
    hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
    hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV128;
    hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
    hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
    hlptim1.Init.UpdateMode = LPTIM_UPDATE_ENDOFPERIOD;
    hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
    hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
    hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
    if((HAL_LPTIM_Init(&hlptim1) == HAL_OK) && (lptim_handle)){
        *lptim_handle = &hlptim1;
    }

    if(fn){
        ll_peripheral_isr_install(LPTIM1_IRQn, fn, ctx);
    }else{
        ll_peripheral_isr_install(LPTIM1_IRQn, lptim1_isr_handler, &hlptim1);
    }

    HAL_NVIC_SetPriority(LPTIM1_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(LPTIM1_IRQn);

}

void lptim1_deinitialize(LPTIM_HandleTypeDef* *lptim_handle){

}

void lptim1_start(LPTIM_HandleTypeDef *lptim_handle, uint16_t ht, uint16_t lt, uint16_t pulse){

}

void lptim1_stop(LPTIM_HandleTypeDef *lptim_handle){

}

