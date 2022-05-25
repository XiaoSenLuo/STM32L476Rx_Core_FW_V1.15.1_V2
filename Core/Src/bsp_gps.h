//
// Created by XIAOSENLUO on 2022/5/25.
//

#ifndef STM32L476RX_BSP_GPS_H
#define STM32L476RX_BSP_GPS_H



#include "stm32l4xx_it.h"
#include "gpio.h"
#include "minmea.h"



void gps_pps_isr_install(GPIO_TypeDef * ppsPort, int32_t ppsPin, isr_function_handle_t fn, void *ctx);
void gps_pps_isr_uninstall(void);
void gps_pps_irq_disable(void);
void gps_pps_irq_enable(void);

#endif //STM32L476RX_BSP_GPS_H
