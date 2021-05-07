/**
  ******************************************************************************
  * File Name          : RTC.h
  * Description        : This file provides code for the configuration
  *                      of the RTC instances.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __rtc_H
#define __rtc_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/


/* USER CODE BEGIN Includes */
#include "stm32l4xx_ll_rtc.h"
#include "time.h"
/* USER CODE END Includes */

extern RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN Private defines */
#define RTC_WKUP_EXTI_LINE          LL_EXTI_LINE_20
#define RTC_ALARM_EXTI_LINE         LL_EXTI_LINE_18
/* USER CODE END Private defines */

void MX_RTC_Init(void);

/* USER CODE BEGIN Prototypes */

extern RTC_TimeTypeDef rtc_time;
extern RTC_DateTypeDef rtc_date;

uint8_t st_rtc_set_time(struct tm* _tm);
uint8_t st_rtc_get_time(struct tm* _tm);

uint8_t st_rtc_set_alarm(struct tm* in_tm);

/**
 *
 * @param in_timeout: 最大超时为 18 * 60 * 60 s
 * @param in_is_it
 * @return
 */
uint8_t st_rtc_config_wkup(uint16_t in_timeout, uint8_t in_is_it);

/**
 *
 * @param in_timeout: unit: s
 * @return
 */
uint8_t st_rtc_config_timeout(uint16_t in_timeout);

void st_rtc_alarm_callback(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ rtc_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
