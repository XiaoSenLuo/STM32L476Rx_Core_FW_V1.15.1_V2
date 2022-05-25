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

#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_rtc.h"
#include "time.h"
/* USER CODE END Includes */

#define TM_DATE_Pos               (32)
#define TM_YEAR_Pos               (9)  // bit9:bit16
#define TM_YEAR_Msk               (0xFFUL << TM_YEAR_Pos)
#define TM_MONTH_Pos              (5)  // bit5:bit8
#define TM_MONTH_Msk              (0x0FUL << TM_MONTH_Pos)
#define TM_DAY_Pos                (0)       // bit0:bit4
#define TM_DAY_Msk                (0x1FUL << TM_DAY_Pos)

#define TM_TIME_Pos               (0)
#define TM_HOUR_Pos               (12) // bit12:bit16
#define TM_HOUR_Msk               (0x1FUL << TM_HOUR_Pos)
#define TM_MINUTE_Pos             (6)  // bit6:bit11
#define TM_MINUTE_Msk             (0x3FUL << TM_MINUTE_Pos)
#define TM_SECOND_Pos             (0)       // bit0-bit5
#define TM_SECOND_Msk             (0x3FUL << TM_SECOND_Pos)

static inline uint8_t tm_get_year(uint32_t date){
	return (uint8_t)((date & TM_YEAR_Msk) >> TM_YEAR_Pos);
}
static inline uint8_t tm_get_month(uint32_t date){
	return (uint8_t)((date & TM_MONTH_Msk) >> TM_MONTH_Pos);
}
static inline uint8_t tm_get_day(uint32_t date){
	return (uint8_t)((date & TM_DAY_Msk) >> TM_DAY_Pos);
}
static inline uint8_t tm_get_hour(uint32_t time){
	return (uint8_t)(((time) & TM_HOUR_Msk) >> TM_HOUR_Pos);
}
static inline uint8_t tm_get_minutes(uint32_t time){
	return (uint8_t)(((time) & TM_MINUTE_Msk) >> TM_MINUTE_Pos);
}
static inline uint8_t tm_get_second(uint32_t time){
	return (uint8_t)(((time) & TM_SECOND_Msk) >> TM_SECOND_Pos);
}

static inline void tm_set_date_time(struct tm* _tm, uint32_t date, uint32_t time){
	_tm->tm_year = tm_get_year(date);
	_tm->tm_mon = tm_get_month(date);
	_tm->tm_mday = tm_get_day(date);

	_tm->tm_hour = tm_get_hour(time);
	_tm->tm_min = tm_get_minutes(time);
	_tm->tm_sec = tm_get_second(time);
}

static inline uint64_t tm_get_date_time(struct tm _tm){
    uint64_t t = 0UL;
    uint64_t tmp = 0UL;
    tmp = _tm.tm_year;
    t |= (tmp << (TM_DATE_Pos + TM_YEAR_Pos));
    tmp = _tm.tm_mon;
    t |= (tmp << (TM_DATE_Pos + TM_MONTH_Pos));
    tmp = _tm.tm_mday;
    t |= (tmp << (TM_DATE_Pos + TM_DAY_Pos));

    tmp = _tm.tm_hour;
    t |= (tmp << (TM_TIME_Pos + TM_HOUR_Pos));
    tmp = _tm.tm_min;
    t |= (tmp << (TM_TIME_Pos + TM_MINUTE_Pos));
    tmp = _tm.tm_sec;
    t |= (tmp << (TM_TIME_Pos + TM_SECOND_Pos));
    return t;
}

/* USER CODE BEGIN Private defines */
#define RTC_WKUP_EXTI_LINE          LL_EXTI_LINE_20
#define RTC_ALARM_EXTI_LINE         LL_EXTI_LINE_18
/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */

uint8_t st_rtc_enter_initmode(void);
uint8_t st_rtc_exit_initmode(void);

typedef struct rtc_data_time_s {
    union {
        struct {
            uint32_t ssecond : 10;
            uint32_t second : 6;
            uint32_t minute : 6;
            uint32_t hour : 5;
            uint32_t pm : 1;
        };
        uint32_t val;
    }time;
    union {
        struct {
            uint32_t day : 5;
            uint32_t weekday : 3;
            uint32_t month : 4;
            uint32_t year : 16;
        };
        uint32_t val;
    }date;
}rtc_date_time_t;

typedef struct rtc_data_time_s * rtc_date_time_handle;

typedef union u32_st_rtc_time_bcd_format_s{
    struct{
        uint32_t sec : 7;
        uint32_t bit7 : 1;
        uint32_t min : 7;
        uint32_t bit15 : 1;
        uint32_t hour : 6;
        uint32_t pm : 1;
    };
    uint32_t val;
}u32_st_rtc_time_bcd_format_t;
typedef u32_st_rtc_time_bcd_format_t * u32_st_rtc_time_bcd_format_handle;

typedef union u32_st_rtc_date_bcd_format_s{
    struct{
        uint32_t day : 6;
        uint32_t bit6_7 : 2;
        uint32_t mon : 5;
        uint32_t wdu : 3;
        uint32_t year : 8;
    };
    uint32_t val;
}u32_st_rtc_date_bcd_format_t;
typedef u32_st_rtc_date_bcd_format_t * u32_st_rtc_date_bcd_format_handle;


static inline void st_rtc_time_convert(rtc_date_time_t *new_format, const struct tm *old_format){
    new_format->time.second = old_format->tm_sec;
    new_format->time.minute = old_format->tm_min;
    new_format->time.hour = old_format->tm_hour;
    new_format->date.day = old_format->tm_mday;
    new_format->date.weekday = old_format->tm_wday;
    new_format->date.month = old_format->tm_mon;
    new_format->date.year = old_format->tm_year;
}

uint8_t st_rtc_set_time(struct tm* _tm);
uint8_t st_rtc_get_time(struct tm* _tm);

uint8_t st_rtc_set_time_v2(const rtc_date_time_t* time);
uint8_t st_rtc_get_time_v2(rtc_date_time_t* time);

uint32_t st_rtc_get_subsecond(void);

uint8_t st_rtc_start_alarm(const char alarm, struct tm in_tm);

uint8_t st_rtc_stop_alarm(const char alarm);

/**
 *
 * @param in_timeout: 最大超时为 18 * 60 * 60 s
 * @param in_is_it
 * @return
 */
uint8_t st_rtc_config_wkup(uint16_t in_timeout, uint8_t in_is_it);

/**
 *
 * @return
 */
uint8_t st_rtc_config_wkup_stop(void);

/**
 *
 * @param in_timeout: unit: s
 * @return
 */
uint8_t st_rtc_config_timeout(uint16_t in_timeout);

typedef RTC_HandleTypeDef * RTC_HandleTypeDef_Handle;

int rtc_initialize(RTC_HandleTypeDef_Handle *hrtc_handle);

int rtc_time2str(const rtc_date_time_t *dt, char *str, size_t length);

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
