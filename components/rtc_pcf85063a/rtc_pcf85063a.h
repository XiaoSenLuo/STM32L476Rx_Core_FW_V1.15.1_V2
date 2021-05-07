/*
 * rtc_pcf85063a.h
 *
 *  Created on: 2021年1月12日
 *      Author: XIAOSENLUO
 */

#ifndef RTC_PCF85063A_RTC_PCF85063A_H_
#define RTC_PCF85063A_RTC_PCF85063A_H_

#include "stdint.h"
#include "stddef.h"
#include "time.h"


// uint8_t I2C2_Write(uint8_t in_address, uint8_t in_reg, uint8_t* in_ptr, uint16_t in_size);
typedef uint8_t (*pcf_read_func_t)(uint8_t , uint8_t , uint8_t* , uint16_t );
// uint8_t I2C2_Read(uint8_t in_address, uint8_t in_reg, uint8_t* out_ptr, uint16_t in_size);
typedef uint8_t (*pcf_write_func_t)(uint8_t , uint8_t , uint8_t* , uint16_t );

typedef struct _pcf_drv_t{
	uint8_t (*read)(uint8_t , uint8_t , uint8_t* , uint16_t );
	uint8_t (*write)(uint8_t , uint8_t , uint8_t* , uint16_t );
}pcf_drv_t;


#define PCF_REG_CONTROL_1                           0x00
#define PCF_REG_CONTROL_2                           0x01
#define PCF_REG_OFFSET                              0x02
#define PCF_REG_RAM                                 0x03
#define PCF_REG_SECONDS                             0x04
#define PCF_REG_MINUTES                             0x05
#define PCF_REG_HOURS                               0x06
#define PCF_REG_DAYS                                0x07
#define PCF_REG_WEEKDAYS                            0x08
#define PCF_REG_MONTHS                              0x09
#define PCF_REG_YEARS                               0x0A
#define PCF_REG_AL_SECONDS                          0x0B
#define PCF_REG_AL_MINUTES                          0x0C
#define PCF_REG_AL_HOURS                            0x0D
#define PCF_REG_AL_DAY                              0x0E
#define PCF_REG_AL_WEEKDAY                          0x0F
#define PCF_REG_TIMER_VALUE                         0x10
#define PCF_REG_TIMER_MODE                          0x11

// 内部晶振负载电容选择
#define PCF_SELETE_CAP_7PF                          0x00
#define PCF_SELETE_CAP_12PF_5                       0x01

// 时钟输出频率
#define PCF_CLKOUT_32768_HZ                         0x00
#define PCF_CLKOUT_16384_HZ                         0x01
#define PCF_CLKOUT_8196_HZ                          0x02
#define PCF_CLKOUT_4096_HZ                          0x03
#define PCF_CLKOUT_2048_HZ                          0x04
#define PCF_CLKOUT_1024_HZ                          0x05
#define PCF_CLKOUT_1_HZ                             0x06
#define PCF_CLKOUT_DISABLE                          0x07

// 设备地址
#define PCF_ADDRESS                                 0xA2

// 时间格式
#define PCF_TIME_FORMAT_12                          0x02
#define PCF_TIME_FORMAT_24                          0x00

// 校准模式
#define PCF_CAL_NORMAL_MODE                         0x00
#define PCF_CAL_COURSE_MODE                         0x80


#ifndef BCD2BIN
#define BCD2BIN(__VALUE__)                         ((uint8_t)(((uint8_t)((__VALUE__) & (uint8_t)0xF0U) >> (uint8_t)0x4U) * 10U + ((__VALUE__) & (uint8_t)0x0FU)))
#endif
#ifndef BIN2BCD
#define BIN2BCD(__VALUE__)                         ((uint8_t)((((__VALUE__) / 10U) << 4U) | ((__VALUE__) % 10U)))
#endif

typedef struct tm pcf_tm_t;

void rtc_pcf_drv_init(pcf_read_func_t read_func, pcf_write_func_t write_func);

/**
 *
 * @param in_cal_mode
 * @param in_offset: 有符号整型, bit[6] 是符号位
 * @return
 */
uint8_t rtc_pcf_calibrate_offset(uint8_t in_cal_mode, uint8_t in_offset);
uint8_t rtc_pcf_get_calibration(uint8_t *out_cal);

uint8_t rtc_pcf_selete_cap(uint8_t in_cap);
uint8_t rtc_pcf_clkout(uint8_t in_clkout);
uint8_t rtc_pcf_set_time_format(uint8_t in_fmt);
uint8_t rtc_pcf_get_time_format(void);

uint8_t rtc_pcf_get_date_time(pcf_tm_t *out_pcf_tm);
uint8_t rtc_pcf_set_date_time(pcf_tm_t *in_pcf_tm);

uint8_t rtc_pcf_set_date(pcf_tm_t *in_pcf_tm);
uint8_t rtc_pcf_get_date(pcf_tm_t *out_pcf_tm);

uint8_t rtc_pcf_set_time(pcf_tm_t *in_pcf_tm);
uint8_t rtc_pcf_get_time(pcf_tm_t *out_pcf_tm);



#endif /* RTC_PCF85063A_RTC_PCF85063A_H_ */
