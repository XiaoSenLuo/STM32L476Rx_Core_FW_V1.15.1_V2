/**
  ******************************************************************************
  * File Name          : RTC.c
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

/* Includes ------------------------------------------------------------------*/
#include "rtc.h"
#include "main.h"
/* USER CODE BEGIN 0 */

RTC_TimeTypeDef rtc_time;
RTC_DateTypeDef rtc_date;



static uint8_t rtc_enter_initmode(void);
static uint8_t rtc_exit_initmode(void);
static uint8_t rtc_waitfor_synchro(void);


/* USER CODE END 0 */

//RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void){

#if defined(USE_FULL_LL_DRIVER)

	LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
	LL_RCC_EnableRTC();
	LL_RTC_DisableWriteProtection(RTC);
	LL_RTC_TS_EnableInternalEvent(RTC);            // 当VCC掉电, 切换到Vbat供电备份时间戳
	LL_RTC_EnableInitMode(RTC);
	while (LL_RTC_IsActiveFlag_INIT(RTC) != 1){};

	LL_RTC_SetHourFormat(RTC, LL_RTC_HOURFORMAT_24HOUR);
	/* Set Asynch Prediv (value according to source clock) */
	LL_RTC_SetAsynchPrescaler(RTC, ((uint32_t)0x7F));
	/* Set Synch Prediv (value according to source clock) */
	LL_RTC_SetSynchPrescaler(RTC, ((uint32_t)0x00FF));

    rtc_exit_initmode();

	LL_RTC_EnableWriteProtection(RTC);

#endif

}

/* USER CODE BEGIN 1 */

static uint8_t rtc_enter_initmode(void){
	uint32_t _timeout = SystemCoreClock;

	LL_RTC_EnableInitMode(RTC);
	while ((_timeout) && (LL_RTC_IsActiveFlag_INIT(RTC) != 1)){ _timeout -= 1;};
    if((!_timeout) && (LL_RTC_IsActiveFlag_INIT(RTC) != 1))
    	return 1;
    else
    	return 0;
}

static uint8_t rtc_exit_initmode(void){
	uint8_t _ret = 0;
	LL_RTC_DisableInitMode(RTC);
    _ret = rtc_waitfor_synchro();
    return _ret;
}

static uint8_t rtc_waitfor_synchro(void){
	uint32_t _timeout = SystemCoreClock;

	LL_RTC_ClearFlag_RS(RTC);
	while((_timeout) && (LL_RTC_IsActiveFlag_RS(RTC) != 1)){ _timeout -= 1;};
	if((!_timeout) && (LL_RTC_IsActiveFlag_RS(RTC) != 1)) return 1;
	else return 0;
}

uint8_t st_rtc_set_time(struct tm* _tm){
    uint32_t res;

    if(_tm == NULL) return 1;

    LL_RTC_TimeTypeDef ll_rtc_time = { .TimeFormat = LL_RTC_HOURFORMAT_24HOUR };
    LL_RTC_DateTypeDef ll_rtc_date = { 0 };

    ll_rtc_time.Hours = __LL_RTC_CONVERT_BIN2BCD((uint8_t)_tm->tm_hour);
    ll_rtc_time.Minutes = __LL_RTC_CONVERT_BIN2BCD((uint8_t)_tm->tm_min);
    ll_rtc_time.Seconds = __LL_RTC_CONVERT_BIN2BCD((uint8_t)_tm->tm_sec);

    ll_rtc_time.TimeFormat = LL_RTC_HOURFORMAT_24HOUR;

    ll_rtc_date.Day = __LL_RTC_CONVERT_BIN2BCD((uint8_t)_tm->tm_mday);
    ll_rtc_date.Month = __LL_RTC_CONVERT_BIN2BCD((uint8_t)_tm->tm_mon);
    ll_rtc_date.WeekDay = __LL_RTC_CONVERT_BIN2BCD((uint8_t)_tm->tm_wday);
    ll_rtc_date.Year = __LL_RTC_CONVERT_BIN2BCD((uint8_t)_tm->tm_year + 1900 - 2000);

    LL_RTC_DisableWriteProtection(RTC);

    res = rtc_enter_initmode();
    if(res != 0){
    	res = 1;
    }else{

    	LL_RTC_DATE_Config(RTC, ll_rtc_date.WeekDay, ll_rtc_date.Day, ll_rtc_date.Month, ll_rtc_date.Year);
    	LL_RTC_TIME_Config(RTC, LL_RTC_TIME_FORMAT_AM_OR_24, ll_rtc_time.Hours, ll_rtc_time.Minutes, ll_rtc_time.Seconds);
    	res = rtc_exit_initmode();
//    	LL_RTC_BAK_SetRegister(RTC, LL_RTC_BKP_DR1, RTC_BKP_DATE_TIME_UPDTATED);
    }
    LL_RTC_EnableWriteProtection(RTC);
    return res;
}

uint8_t st_rtc_get_time(struct tm* _tm){
	uint8_t ret = 0;
    if(_tm == NULL) return 1;
	_tm->tm_sec = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC));
	_tm->tm_hour = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC));
	_tm->tm_min = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC));
	_tm->tm_mday = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetDay(RTC));
	_tm->tm_mon = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetMonth(RTC));
	_tm->tm_wday = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetWeekDay(RTC));
	_tm->tm_year = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetYear(RTC)) + 2000 - 1900;

	return 0;
}

uint8_t st_rtc_set_alarm(struct tm* in_tm){
	uint8_t ret = 0;

	LL_RTC_ALMA_Disable(RTC);

    if(ret == 0){
    	LL_RTC_ALMA_ConfigTime(RTC, LL_RTC_ALMA_TIME_FORMAT_AM, 0, 0, 0);
    }
	return ret;
}

uint8_t st_rtc_config_wkup(uint16_t in_timeout, uint8_t in_is_it){
	uint8_t ret = 0;
	uint32_t timeout = 8000000;
	LL_RTC_DisableWriteProtection(RTC);
	LL_RTC_WAKEUP_Disable(RTC);
	while ((LL_RTC_IsActiveFlag_WUTW(RTC) != 1) && (timeout)){
		timeout--;
	}
	if(timeout){
		LL_RTC_WAKEUP_SetAutoReload(RTC, in_timeout);
		LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);
	}else
		ret = 1;

	if(in_is_it){ // 使能 WUKP interrupt
		LL_RTC_ClearFlag_WUT(RTC);
		LL_RTC_EnableIT_WUT(RTC);
		NVIC_SetPriority(RTC_WKUP_IRQn, 7);
		NVIC_EnableIRQ(RTC_WKUP_IRQn);
		LL_EXTI_EnableIT_0_31(RTC_WKUP_EXTI_LINE);
		LL_EXTI_EnableRisingTrig_0_31(RTC_WKUP_EXTI_LINE);
	}else{
		LL_RTC_DisableIT_WUT(RTC);
		NVIC_DisableIRQ(RTC_WKUP_IRQn);
		LL_EXTI_DisableIT_0_31(RTC_WKUP_EXTI_LINE);
	}
	LL_RTC_EnableWriteProtection(RTC);
    return ret;
}

uint8_t st_rtc_config_timeout(uint16_t in_timeout){
	uint8_t ret = 0;
	uint32_t timeout = 8000000;
	LL_RTC_DisableWriteProtection(RTC);
	LL_RTC_WAKEUP_Disable(RTC); // 关闭
	while ((LL_RTC_IsActiveFlag_WUTW(RTC) != 1) && (timeout)){
		timeout--;
	}
	if(timeout){
		LL_RTC_WAKEUP_SetAutoReload(RTC, in_timeout);
		LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);
		LL_RTC_ClearFlag_WUT(RTC);
		LL_RTC_EnableIT_WUT(RTC);
		NVIC_SetPriority(RTC_WKUP_IRQn, 0);
		NVIC_EnableIRQ(RTC_WKUP_IRQn);
		LL_EXTI_EnableIT_0_31(RTC_WKUP_EXTI_LINE);
		LL_EXTI_EnableRisingTrig_0_31(RTC_WKUP_EXTI_LINE);
		LL_RTC_WAKEUP_Enable(RTC);   // 启动
	}else
		ret = 1;
	LL_RTC_EnableWriteProtection(RTC);
	return ret;
}

void RTC_SetUTCTime(uint32_t utcTime){

}

__weak void st_rtc_alarm_callback(void){

}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
