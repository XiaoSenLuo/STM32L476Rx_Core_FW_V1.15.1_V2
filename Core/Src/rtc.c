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
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_pwr.h"
/* USER CODE BEGIN 0 */

RTC_TimeTypeDef rtc_time;
RTC_DateTypeDef rtc_date;


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

    st_rtc_exit_initmode();

	LL_RTC_EnableWriteProtection(RTC);

#endif

}

/* USER CODE BEGIN 1 */

uint8_t st_rtc_enter_initmode(void){
	uint32_t _timeout = SystemCoreClock;

	LL_RTC_EnableInitMode(RTC);
	while ((_timeout) && (LL_RTC_IsActiveFlag_INIT(RTC) != 1)){ _timeout -= 1;};
    if((!_timeout) && (LL_RTC_IsActiveFlag_INIT(RTC) != 1))
    	return 1;
    else
    	return 0;
}

uint8_t st_rtc_exit_initmode(void){
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
    ll_rtc_date.Year = __LL_RTC_CONVERT_BIN2BCD((uint8_t)(_tm->tm_year + 1900 - 2000));

    LL_RTC_DisableWriteProtection(RTC);

    res = st_rtc_enter_initmode();
    if(res != 0){
    	res = 1;
    }else{
    	LL_RTC_TIME_Config(RTC, LL_RTC_TIME_FORMAT_AM_OR_24, ll_rtc_time.Hours, ll_rtc_time.Minutes, ll_rtc_time.Seconds);
    	LL_RTC_DATE_Config(RTC, ll_rtc_date.WeekDay, ll_rtc_date.Day, ll_rtc_date.Month, ll_rtc_date.Year);
    }
    res = st_rtc_exit_initmode();

    LL_RTC_EnableWriteProtection(RTC);
    return res;
}

uint8_t st_rtc_get_time(struct tm* _tm){
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

uint32_t st_rtc_get_subsecond(void){
    return LL_RTC_TIME_GetSubSecond(RTC);
}

uint8_t st_rtc_start_alarm(const char alarm, struct tm in_tm){
	uint8_t ret = 0;

	LL_RTC_DisableWriteProtection(RTC);
    if(st_rtc_enter_initmode() != 0){
        ret = 1;
        goto end_section;
    }

	if((alarm == 'A') || (alarm == 'a')){   // 闹钟A
		LL_RTC_ALMA_Disable(RTC);
		LL_RTC_ALMA_SetDay(RTC, __LL_RTC_CONVERT_BIN2BCD(in_tm.tm_mday));
	    LL_RTC_ALMA_ConfigTime(RTC, LL_RTC_ALMA_TIME_FORMAT_AM,
	    		__LL_RTC_CONVERT_BIN2BCD(in_tm.tm_hour), __LL_RTC_CONVERT_BIN2BCD(in_tm.tm_min), __LL_RTC_CONVERT_BIN2BCD(in_tm.tm_sec));
	    LL_RTC_ALMA_SetMask(RTC, LL_RTC_ALMA_MASK_NONE);
	    LL_RTC_ALMA_Enable(RTC);
	    LL_RTC_ClearFlag_ALRA(RTC);
	    LL_RTC_EnableIT_ALRA(RTC);
	}

	if((alarm == 'B') || (alarm == 'b')){   // 闹钟B
		LL_RTC_ALMB_Disable(RTC);
		LL_RTC_ALMA_SetDay(RTC, __LL_RTC_CONVERT_BIN2BCD(in_tm.tm_mday));
	    LL_RTC_ALMB_ConfigTime(RTC, LL_RTC_ALMB_TIME_FORMAT_AM,
	    		__LL_RTC_CONVERT_BIN2BCD(in_tm.tm_hour), __LL_RTC_CONVERT_BIN2BCD(in_tm.tm_min), __LL_RTC_CONVERT_BIN2BCD(in_tm.tm_sec));
	    LL_RTC_ALMB_SetMask(RTC, LL_RTC_ALMB_MASK_NONE);
	    LL_RTC_ALMB_Enable(RTC);
	    LL_RTC_ClearFlag_ALRB(RTC);
	    LL_RTC_EnableIT_ALRB(RTC);
	}

    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_18);
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_18);
	NVIC_SetPriority(RTC_Alarm_IRQn, 0x00);
	NVIC_EnableIRQ(RTC_Alarm_IRQn);

	ret = st_rtc_exit_initmode();
end_section:
	LL_RTC_EnableWriteProtection(RTC);
	return ret;
}

uint8_t st_rtc_stop_alarm(const char alarm){
	uint8_t ret = 0;

	LL_RTC_DisableWriteProtection(RTC);
    if(st_rtc_enter_initmode() != 0){
        ret = 1;
        goto end_section;
    }
	if((alarm == 'A') || (alarm == 'a')){   // 闹钟A
		LL_RTC_ALMA_Disable(RTC);
	    LL_RTC_ClearFlag_ALRA(RTC);
	    LL_RTC_DisableIT_ALRA(RTC);
	}

	if((alarm == 'B') || (alarm == 'b')){   // 闹钟B
		LL_RTC_ALMB_Disable(RTC);
	    LL_RTC_ClearFlag_ALRB(RTC);
	    LL_RTC_DisableIT_ALRB(RTC);
	}

	NVIC_DisableIRQ(RTC_Alarm_IRQn);

	ret = st_rtc_exit_initmode();
end_section:
	LL_RTC_EnableWriteProtection(RTC);
	return ret;
}




uint8_t st_rtc_config_wkup(uint16_t in_timeout, uint8_t in_is_it){
#define _RTC_TIMEOUT       8000000
	uint8_t ret = 0;
	uint32_t timeout = _RTC_TIMEOUT;
	LL_RTC_DisableWriteProtection(RTC);
	LL_RTC_WAKEUP_Disable(RTC);

	if(LL_RTC_IsActiveFlag_INIT(RTC) == 0){
		while((LL_RTC_IsActiveFlag_WUTW(RTC) == 0) && timeout){
             timeout -= 1;
		}
	}
	if(!timeout){
		LL_RTC_EnableWriteProtection(RTC);
		return 1;
	}
    timeout = _RTC_TIMEOUT;

	LL_RTC_ClearFlag_WUT(RTC);

	LL_EXTI_ClearFlag_0_31(RTC_WKUP_EXTI_LINE);
	if (LL_PWR_IsEnabledInternWU() == 0) LL_PWR_EnableInternWU(); /* Need to enable the Internal Wake-up line */
    if(in_timeout > 0xFFFF) in_timeout = 0xFFFF;
	if(in_timeout){
    	if(in_is_it){ //  interrupt 唤醒
    		LL_RTC_EnableIT_WUT(RTC);
    		NVIC_SetPriority(RTC_WKUP_IRQn, 0);  // 最高优先级, 防止无法唤醒
    		NVIC_EnableIRQ(RTC_WKUP_IRQn);
    		LL_EXTI_InitTypeDef wk = {.LineCommand = ENABLE, .Line_0_31 = RTC_WKUP_EXTI_LINE, .Line_32_63 = LL_EXTI_LINE_NONE, .Mode = LL_EXTI_MODE_IT, .Trigger = LL_EXTI_TRIGGER_RISING};
    		LL_EXTI_Init(&wk);
    	}else{         // event 唤醒
//    		LL_RTC_DisableIT_WUT(RTC);
    		LL_RTC_EnableIT_WUT(RTC);
    		NVIC_DisableIRQ(RTC_WKUP_IRQn);
    		LL_EXTI_InitTypeDef wk = {.LineCommand = ENABLE, .Line_0_31 = RTC_WKUP_EXTI_LINE, .Line_32_63 = LL_EXTI_LINE_NONE, .Mode = LL_EXTI_MODE_EVENT, .Trigger = LL_EXTI_TRIGGER_RISING};
    		LL_EXTI_Init(&wk);
    	}

    	while ((LL_RTC_IsActiveFlag_WUTW(RTC) != 1) && (timeout)){
    		timeout--;
    	}
    	if(timeout){
    		LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);
    		LL_RTC_WAKEUP_SetAutoReload(RTC, in_timeout);
    	}else
    		ret = 1;
    	LL_RTC_WAKEUP_Enable(RTC);
    	LL_RTC_ClearFlag_WUT(RTC);
    }else{
    	LL_RTC_DisableIT_WUT(RTC);
    	NVIC_DisableIRQ(RTC_WKUP_IRQn);
    }

    LL_RTC_EnableWriteProtection(RTC);
    return ret;
#undef _RTC_TIMEOUT
}

uint8_t st_rtc_config_wkup_stop(void){
#define _RTC_TIMEOUT       8000000

	uint32_t timeout = _RTC_TIMEOUT;
	LL_RTC_DisableWriteProtection(RTC);
	LL_RTC_WAKEUP_Disable(RTC);
	LL_RTC_ClearFlag_WUT(RTC);
	LL_EXTI_ClearFlag_0_31(RTC_WKUP_EXTI_LINE);
	NVIC_DisableIRQ(RTC_WKUP_IRQn);
	if(LL_RTC_IsActiveFlag_INIT(RTC) == 0){
		while((LL_RTC_IsActiveFlag_WUTW(RTC) == 0) && timeout){
             timeout -= 1;
		}
	}
	if(!timeout){
		LL_RTC_EnableWriteProtection(RTC);
		return 1;
	}
	LL_RTC_EnableWriteProtection(RTC);
	return 0;
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


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
