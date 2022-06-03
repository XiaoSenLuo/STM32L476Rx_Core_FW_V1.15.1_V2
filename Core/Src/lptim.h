//
// Created by XIAO on 2022/5/26.
//

#ifndef STM32L476RX_LPTIM_H
#define STM32L476RX_LPTIM_H



#include "stm32l4xx_it.h"

void lptim1_initialize(LPTIM_HandleTypeDef* *lptim_handle, isr_function_handle_t fn, void *ctx);

void lptim1_deinitialize(LPTIM_HandleTypeDef* *lptim_handle);

void lptim1_start(LPTIM_HandleTypeDef *lptim_handle, uint16_t ht, uint16_t lt, uint16_t pulse);
void lptim1_stop(LPTIM_HandleTypeDef *lptim_handle);

#endif //STM32L476RX_LPTIM_H
