/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */



#include "stm32l4xx_it.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include "SEGGER_RTT.h"
#include "time.h"
#include "ctype.h"

#include "fatfs.h"
#include "ads127.h"
#include "spi.h"
#include "usart.h"
#include "adc.h"
#include "lptim.h"
#include "tim.h"
#include "i2c.h"
#include "gpio.h"
#include "rtc.h"
#include "mem_dma.h"
#include "st_crc.h"
#include "ini.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern TIM_HandleTypeDef htim2;   // use for timbase

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

typedef struct _sys_config_t{
    uint32_t gps_baudrate;
//    uint32_t gps_
}sys_config_t;

extern __IO uint32_t task_fild;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define EXTI_LINE0_Priority             0x0D
#define EXTI_LINE1_Priority             0x0D
#define EXTI_LINE2_Priority             0x0D
#define EXTI_LINE3_Priority             0x00
#define EXTI_LINE4_Priority             0x0D
#define EXTI_LINE5_Priority             0x0D
#define EXTI_LINE6_Priority             0x0D
#define EXTI_LINE7_Priority             0x04
#define EXTI_LINE8_Priority             0x0D
#define EXTI_LINE9_Priority             0x0D
#define EXTI_LINE10_Priority            0x0D
#define EXTI_LINE11_Priority            0x0D
#define EXTI_LINE12_Priority            0x0D
#define EXTI_LINE13_Priority            0x0D
#define EXTI_LINE14_Priority            0x0D
#define EXTI_LINE15_Priority            0x0D

#define SYSTEM_TICK_Priority            0x00


#define UART4_Priority                  0x02
#define USART1_Priority                 0x01
#define USART3_Priority                 0x01

#define LPTIM1_Priority                 0x0D
#define LPTIM2_Priority                 0x04

#define SDMMC1_Priority                 0x01

#define SPI1_Priority                   0x01

#define DMA2_Channel3_Priority          0x03
#define DMA2_Channel4_Priority          0x02       // SDMMC
#define DMA2_Channel5_Priority          0x02       // SDMMC
#define DMA1_Channel1_Priority          0x0D
#define DMA1_Channel2_Priority          0x00       // SPI1
#define DMA1_Channel3_Priority          0x00       // SPI1
#define DMA1_Channel4_Priority          0x03
#define DMA1_Channel5_Priority          0x03
#define DMA1_Channel6_Priority          0x03

#if((SDMMC1_Priority >= DMA2_Channel4_Priority) || (SDMMC1_Priority >= DMA2_Channel5_Priority))
#error "ERROR"
#endif

#define CTRL_WINDOWS_CR_LF                               "\r\n"
#ifndef CMD_TX_SIGN
#define CMD_TX_SIGN                                      '>'
#endif
#ifndef CMD_RX_SIGN
#define CMD_RX_SIGN                                      '<'
#endif
#ifndef CMD_SET_COMMAND
#define CMD_SET_COMMAND                                  "set"
#endif
#ifndef CMD_GET_COMMAND
#define CMD_GET_COMMAND                                  "get"
#endif
#ifndef CMD_CR
#define CMD_CR                                           '\r'
#endif
#ifndef CMD_LF
#define CMD_LF                                           '\n'
#endif


#define TASK_CMD                             0x00000001U    // 命令响应
#define TASK_GPS                             0x00000002U    // GPS授时
#define TASK_ADS                             0x00000004U    // ADS采集
#define TASK_RTCTIME                         0x00000008U    // 时钟校准
#define TASK_CMD_STOP                        0x00000010U    // 关闭命令相应, 关闭相应外设
#define TASK_GPS_STOP                        0x00000020U    // 已完成授时, 关闭相应外设
#define TASK_ADS_STOP                        0x00000040U    // 停止采集
//#define TASK_TIME_STOP                       0x00000080U    //

#define TASK_LED                             0x00000100U    // LED控制
#define TASK_TIM                             0x00000200U    // 定时器控制, 产生定时脉冲控制 ADS 采样率
#define TASK_USB_DECTION                     0x00000400U    // USB插入检测
#define TASK_BATTERY                         0x00000800U    // 电池电压监测
#define TASK_BATTERY_STOP                    0x00001000U

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
uint8_t cmd_printf(const char* fmt, ...);
uint8_t f_log_printf(FIL* _log_file, const char* fmt, ...);
/* USER CODE BEGIN EFP */


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
//#define EXTI_IMU2_Pin                              LL_GPIO_PIN_13
//#define EXTI_IMU2_GPIO_Port                        GPIOC
//#define EXTI_IMU2_EXTI_IRQn                        EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */
#define EXTI_IMU1_Pin                              LL_GPIO_PIN_12
#define EXTI_IMU1_GPIO_Port                        GPIOB
#define EXTI_IMU1_EXTI_IRQn                        EXTI15_10_IRQn
#define EXTI_IMU1_Priority                         EXTI_LINE12_Priority
#define EXTI_PPS_Pin                               LL_GPIO_PIN_2
#define EXTI_PPS_GPIO_Port                         GPIOA
#define EXTI_PPS_EXTI_IRQn                         EXTI2_IRQn
#define EXTI_PPS_Priority                          EXTI_LINE2_Priority
#define VGPS_EN_Pin                                LL_GPIO_PIN_3
#define VGPS_EN_GPIO_Port                          GPIOA
#define ADS_CLK_Pin                                LL_GPIO_PIN_8
#define ADS_CLK_GPIO_Port                          GPIOA
#define ADS_CS_Pin                                 LL_GPIO_PIN_6
#define ADS_CS_GPIO_Port                           GPIOB
#define ADS_START_Pin                              LL_GPIO_PIN_2
#define ADS_START_GPIO_Port                        GPIOB
#define EXTI_DRDY_Pin                              LL_GPIO_PIN_7
#define EXTI_DRDY_GPIO_Port                        GPIOB
#define EXTI_DRDY_EXTI_IRQn                        EXTI9_5_IRQn
#define EXTI_DRDY_Priority                         EXTI_LINE7_Priority
#define TS_IN1_Pin                                 LL_GPIO_PIN_15
#define TS_IN1_GPIO_Port                           GPIOA
#define VSD_EN_Pin                                 LL_GPIO_PIN_2
#define VSD_EN_GPIO_Port                           GPIOC
#define SD_CD_Pin                                  LL_GPIO_PIN_13
#define SD_CD_GPIO_Port                            GPIOC
#define VBUS_EN_Pin                                LL_GPIO_PIN_6
#define VBUS_EN_GPIO_Port                          GPIOC
#define V5_EN_Pin                                  LL_GPIO_PIN_5
#define V5_EN_GPIO_Port                            GPIOA
#define EXTI_USBCD_Pin                             LL_GPIO_PIN_3
#define EXTI_USBCD_GPIO_Port                       GPIOC
#define EXTI_USBCD_EXTI_IRQn                       EXTI3_IRQn
#define EXTI_USBCD_Priority                        EXTI_LINE3_Priority
#define LED_Pin                                    LL_GPIO_PIN_4
#define LED_GPIO_Port                              GPIOA
#define HSE_OSC_EN_Pin                             LL_GPIO_PIN_1
#define HSE_OSC_EN_Port                            GPIOH

#define LED_ON()                        LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin)
#define LED_OFF()                       LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin)
#define VSD_Control(value)              HAL_GPIO_WritePin(VSD_EN_GPIO_Port, VSD_EN_Pin, (!(value)))
#define VGPS_Control(value)             HAL_GPIO_WritePin(VGPS_EN_GPIO_Port, VGPS_EN_Pin, (!(value)))
#define VBUS_Control(value)             HAL_GPIO_WritePin(VBUS_EN_GPIO_Port, VBUS_EN_Pin, (value))
#define TF_Connect_To_USB()             LL_GPIO_ResetOutputPin(TS_IN1_GPIO_Port, TS_IN1_Pin)
#define TF_Connect_To_MCU()             LL_GPIO_SetOutputPin(TS_IN1_GPIO_Port, TS_IN1_Pin)
#define ADS_CS(value)                   HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, (value))
#define ADS_Start_Convert()             LL_GPIO_SetOutputPin(ADS_START_GPIO_Port, ADS_START_Pin)
#define ADS_Stop_Convert()              LL_GPIO_ResetOutputPin(ADS_START_GPIO_Port, ADS_START_Pin)

#define V5_ON()                         LL_GPIO_SetOutputPin(V5_EN_GPIO_Port, V5_EN_Pin)
#define V5_OFF()                        LL_GPIO_ResetOutputPin(V5_EN_GPIO_Port, V5_EN_Pin)

#define HSE_OSC_Control(value)          HAL_GPIO_WritePin(HSE_OSC_EN_Port, HSE_OSC_EN_Pin, (value))

#define HEX                             "0123456789ABCDEF"

#define RTC_TIMEOUT                     4
#define RTC_SD_WR_TIMEOUT               16
#define RTC_ADS_READ_TIMEOUT            8

void enter_sleep_mode(void);
void enter_power_mode(uint32_t power_mode);
int8_t get_battery_level(void);

void   mco_use_for_ads_clk(uint32_t in_clk_freq);


static inline void ads_drdy_exti_enable(void){
//	HAL_NVIC_SetPriority (EXTI_DRDY_EXTI_IRQn, EXTI_DRDY_Priority, 0);
//	NVIC_EnableIRQ(EXTI_DRDY_EXTI_IRQn);
	LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_7);
	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_7);
}

static inline void ads_drdy_exti_disable(void){
//    NVIC_DisableIRQ(EXTI_DRDY_EXTI_IRQn);
	LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_7);
	LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_7);
}

static inline void usb_dection_exti_enable(void){
	HAL_NVIC_SetPriority (EXTI_USBCD_EXTI_IRQn, EXTI_IMU1_Priority, 0);
	NVIC_EnableIRQ(EXTI_USBCD_EXTI_IRQn);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_3);
	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_3);
}

static inline void usb_dection_exti_disable(void){
//	HAL_NVIC_SetPriority (EXTI_USBCD_EXTI_IRQn, EXTI_USBCD_Priority, 0);
//	NVIC_DisableIRQ(EXTI_USBCD_EXTI_IRQn);
	LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_3);
	LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_3);
}

void usb_dection_exti_callback(void);

static inline uint8_t usb_is_connect(void){
    return (LL_GPIO_ReadInputPort(EXTI_USBCD_GPIO_Port) & EXTI_USBCD_Pin) ? 1 : 0;
}

static inline void imu_exti_enable(void){
	HAL_NVIC_SetPriority (EXTI_IMU1_EXTI_IRQn, EXTI_IMU1_Priority, 0);
	NVIC_EnableIRQ(EXTI_IMU1_EXTI_IRQn);
	LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_12);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_12);
}
static inline void imu_exti_disable(void){
//	HAL_NVIC_SetPriority (EXTI_IMU1_EXTI_IRQn, EXTI_IMU1_Priority, 0);
	NVIC_DisableIRQ(EXTI_IMU1_EXTI_IRQn);
	LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_12);
	LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_12);
}

void imu_exti_callback(void);

static inline void gps_pps_exti_enable(void){
	HAL_NVIC_SetPriority (EXTI_PPS_EXTI_IRQn, EXTI_PPS_Priority, 0);
	NVIC_EnableIRQ(EXTI_PPS_EXTI_IRQn);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_2);
//	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_2);
}
static inline void gps_pps_exti_disable(void){
//	HAL_NVIC_SetPriority (EXTI_PPS_EXTI_IRQn, EXTI_PPS_Priority, 0);
//	NVIC_DisableIRQ(EXTI_PPS_EXTI_IRQn);
	LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_2);
	LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_2);
}

void rtc_reset_ads_timeout(void);

__STATIC_INLINE void systick_disable(void){
	SysTick->CTRL &= (~(SysTick_CTRL_ENABLE_Msk));
}

static inline void systick_disable_ti_req(void){
	SysTick->CTRL &= (~SysTick_CTRL_TICKINT_Msk);
	NVIC_DisableIRQ(SysTick_IRQn);
}

static inline void systick_enable_ti_req(void){
	SysTick->CTRL |= (SysTick_CTRL_TICKINT_Msk);
	NVIC_EnableIRQ(SysTick_IRQn);
}

static inline void systick_enable(void){
	SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk);
}

static inline void systick_selete_clk_div(uint32_t in_div){
	LL_SYSTICK_SetClkSource(in_div);
}

static inline void systick_set_reload_value(uint32_t in_value){
	SysTick->LOAD = (uint32_t)(in_value - 1);
}

static inline void systick_set_counter(uint32_t in_value){
	SysTick->VAL = (uint32_t)in_value;
}

static inline void systick_set_priority(uint8_t in_priority){
	NVIC_SetPriority (SysTick_IRQn, in_priority);
}

static inline uint32_t systick_get_counter(void){
	return  (uint32_t)(SysTick->VAL);
}

//void systick_callback(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
