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
#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_pwr.h"


/* USER CODE BEGIN 0 */

static uint8_t rtc_waitfor_synchro(void);

#define RTC_CONFIG_BKP_FILD               (0xA2F8)

/* USER CODE END 0 */

static RTC_HandleTypeDef hrtc = { 0 };

/* RTC init function */

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

int rtc_initialize(RTC_HandleTypeDef_Handle *hrtc_handle){
    int err = 0;
    RCC_OscInitTypeDef        RCC_OscInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct = { 0 };

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 0x7F; /* LSE as RTC clock */
    hrtc.Init.SynchPrediv = 0x00FF; /* LSE as RTC clock */
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

    err = HAL_RTC_Init(&hrtc);
    if(hrtc_handle) *hrtc_handle = &hrtc;

    if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != RTC_CONFIG_BKP_FILD){
//        rtc_date_time_t dt = {.time.val = 0, .date.val = 0};
//        st_rtc_set_time_v2(&dt);
        RTC_DateTypeDef sdatestructure = {0};
        RTC_TimeTypeDef stimestructure = {0};

        /// 2022-6-9 星期四
        sdatestructure.Year = 0x22;
        sdatestructure.Month = RTC_MONTH_JUNE;
        sdatestructure.Date = 0x09;
        sdatestructure.WeekDay = RTC_WEEKDAY_THURSDAY;
        HAL_RTC_SetDate(&hrtc,&sdatestructure,RTC_FORMAT_BCD);

        /// 23:59:59
        stimestructure.Hours = 0x23;
        stimestructure.Minutes = 0x59;
        stimestructure.Seconds = 0x59;
        stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
        stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
        stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
        HAL_RTC_SetTime(&hrtc, &stimestructure, RTC_FORMAT_BCD);

        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, RTC_CONFIG_BKP_FILD);
    }else{
        __HAL_RCC_CLEAR_RESET_FLAGS();
    }

    return err;
}

uint8_t st_rtc_set_time(struct tm* _tm){
    uint32_t res = 0;

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

uint8_t st_rtc_set_time_v2(const rtc_date_time_t* time){
    uint32_t res = 0;

    if(time == NULL) return 1;
    uint32_t bcd_time = 0x00000000, bcd_date = 0x00002101;
    u32_st_rtc_time_bcd_format_handle bcd_time_handle = NULL;
    u32_st_rtc_date_bcd_format_handle bcd_date_handle = NULL;

    bcd_time_handle = (u32_st_rtc_time_bcd_format_handle)&bcd_time;
    bcd_date_handle = (u32_st_rtc_date_bcd_format_handle)&bcd_date;

    bcd_time_handle->sec = __LL_RTC_CONVERT_BIN2BCD(time->time.second);
    bcd_time_handle->min = __LL_RTC_CONVERT_BIN2BCD(time->time.minute);
    bcd_time_handle->hour = __LL_RTC_CONVERT_BIN2BCD(time->time.hour);
    bcd_time_handle->pm = 0;  /// 24 小时制

    bcd_date_handle->day = __LL_RTC_CONVERT_BIN2BCD(time->date.day);
    bcd_date_handle->wdu = __LL_RTC_CONVERT_BIN2BCD(time->date.weekday);
    bcd_date_handle->mon = __LL_RTC_CONVERT_BIN2BCD(time->date.month);
    bcd_date_handle->year = __LL_RTC_CONVERT_BIN2BCD(time->date.year - 2000);

    LL_RTC_DisableWriteProtection(RTC);
    res = st_rtc_enter_initmode();
    if(res != 0){
        res = 1;
    }else{
        RTC->TR = bcd_time;
        RTC->DR = bcd_date;
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


uint8_t st_rtc_get_time_v2(rtc_date_time_t* time){
    if(time == NULL) return 1;
    uint32_t bcd_time = 0, bcd_date = 0, ssr = 0;
    u32_st_rtc_time_bcd_format_handle bcd_time_handle = NULL;
    u32_st_rtc_date_bcd_format_handle bcd_date_handle = NULL;
    ssr = READ_REG(RTC->SSR);
    bcd_time = READ_REG(RTC->TR);
    bcd_date = READ_REG(RTC->DR);
    bcd_time_handle = (u32_st_rtc_time_bcd_format_handle)&bcd_time;
    bcd_date_handle = (u32_st_rtc_date_bcd_format_handle)&bcd_date;
    time->time.ssecond = 255 - (uint16_t)ssr;
    time->time.second = __LL_RTC_CONVERT_BCD2BIN(bcd_time_handle->sec);
    time->time.minute = __LL_RTC_CONVERT_BCD2BIN(bcd_time_handle->min);
    time->time.hour = __LL_RTC_CONVERT_BCD2BIN(bcd_time_handle->hour);
    time->time.pm = __LL_RTC_CONVERT_BCD2BIN(bcd_time_handle->pm);
    time->date.day = __LL_RTC_CONVERT_BCD2BIN(bcd_date_handle->day);
    time->date.weekday = __LL_RTC_CONVERT_BCD2BIN(bcd_date_handle->wdu);
    time->date.month = __LL_RTC_CONVERT_BCD2BIN(bcd_date_handle->mon);
    time->date.year = __LL_RTC_CONVERT_BCD2BIN(bcd_date_handle->year) + 2000;

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


static const char *time_str[60] = {
        "00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "31", "32", "33", "34", "35", "36", "37", "38", "39", "40", "41", "42", "43", "44", "45", "46", "47", "48", "49", "50", "51", "52", "53", "54", "55", "56", "57", "58", "59"
};

static const uint8_t month_max_day[13] = {0,31, 28, 31,30,31,30,31,31,30,31,30,31};

#include "stdio.h"

int rtc_time2str(const rtc_date_time_t *dt, char *str, size_t length){
    return snprintf(str, length, "%d-%s-%s-%s-%s-%s",
             dt->date.year, time_str[dt->date.month], time_str[dt->date.day],
             time_str[dt->time.hour], time_str[dt->time.minute], time_str[dt->time.second]);
}

int rtc_add_seconds(struct rtc_data_time_s *dt, uint8_t sec){
    uint8_t csec = 0, cmin = 0, chour = 0, cday = 0, cmon = 0, cyear = 0;

    dt->time.second += sec;
    if(dt->time.second > 59){
        csec = 1;
        dt->time.second = 0;
    }
    if(csec){
        dt->time.minute += csec;
        if(dt->time.minute > 59){
            cmin = 1;
            dt->time.minute = 0;
        }
    }
    if(cmin){
        dt->time.hour += cmin;
        if(dt->time.hour > 23){
            chour = 1;
            dt->time.hour = 0;
        }
    }
    if(chour){
        dt->date.day += chour;
        if(dt->date.day > month_max_day[dt->date.month]){
            cmon = 1;
            dt->date.day = 1;
        }
    }
    if(cday){
        dt->date.month += cday;
        if(dt->date.month > 12){
            cmon = 1;
            dt->date.month = 1;
        }
    }
    if(cmon){
        dt->date.year += cmon;
    }
}


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
