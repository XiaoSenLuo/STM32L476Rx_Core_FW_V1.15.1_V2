/*
 * rtc_pcf85063a.c
 *
 *  Created on: 2021年1月12日
 *      Author: XIAOSENLUO
 */

#include "rtc_pcf85063a.h"

static pcf_drv_t pcf_drv = {.read = NULL, .write = NULL};

static uint8_t pcf_init = 0;
static uint8_t tm_fmt = 0;     // 0: 24 Format, 1: 12 Format

void rtc_pcf_drv_init(pcf_read_func_t read_func, pcf_write_func_t write_func){
	if(read_func != NULL){
        pcf_drv.read = read_func;
        pcf_init |= 0x01;
	}else{
		pcf_init &= (~0x01);
	}
	if(write_func != NULL){
		pcf_drv.write = write_func;
		pcf_init |= 0x02;
	}else{
		pcf_init &= (~0x02);
	}
}

uint8_t rtc_pcf_calibrate_offset(uint8_t in_cal_mode, uint8_t in_offset){
	uint8_t ret = 0;
    uint8_t _offset = 0;
    if((pcf_init & 0x02) == 0) return 1;
    _offset |= in_cal_mode;
    _offset |= in_offset;

    ret = pcf_drv.write(PCF_ADDRESS, PCF_REG_OFFSET, &_offset, 1);

	return ret;
}

uint8_t rtc_pcf_get_calibration(uint8_t *out_cal){
	uint8_t ret = 0;
	uint8_t _offset = 0;
	if((pcf_init & 0x01) == 0) return 1;
	ret = pcf_drv.read(PCF_ADDRESS, PCF_REG_OFFSET, &_offset, 1);

	if((ret == 0) && (out_cal != NULL)){
		*out_cal = _offset;
	}
    return ret;
}
uint8_t rtc_pcf_selete_cap(uint8_t in_cap){
	uint8_t ret = 0;
    uint8_t _control1 = 0;
    if(pcf_init == 0) return 1;
    ret = pcf_drv.read(PCF_ADDRESS, PCF_REG_CONTROL_1, &_control1, 1);
    if(ret) return ret;
    _control1 &= (~0x01);       // clear bit0
    _control1 |= (in_cap & 0x01);
    ret = pcf_drv.write(PCF_ADDRESS, PCF_REG_CONTROL_1, &_control1, 1);

	return ret;
}

uint8_t rtc_pcf_clkout(uint8_t in_clkout){
	uint8_t ret = 0;
    uint8_t _control2 = 0;
    if(pcf_init == 0) return 1;
    ret = pcf_drv.read(PCF_ADDRESS, PCF_REG_CONTROL_2, &_control2, 1);
    if(ret) return ret;
    _control2 &= (~PCF_CLKOUT_DISABLE);    // clear bit [2:0]
    _control2 |= (in_clkout & PCF_CLKOUT_DISABLE);                // modify bit [2:0]
    ret = pcf_drv.write(PCF_ADDRESS, PCF_REG_CONTROL_2, &_control2, 1);
	return ret;
}

uint8_t rtc_pcf_set_time_format(uint8_t in_fmt){
    uint8_t ret = 0;
    uint8_t _control1 = 0;
    if(pcf_init == 0) return 1;
    tm_fmt = in_fmt & 0x02;
    ret = pcf_drv.read(PCF_ADDRESS, PCF_REG_CONTROL_1, &_control1, 1);
    if(ret) return ret;
    _control1 &= (~0x02);  // clear bit 1;
    _control1 |= (in_fmt & 0x02);
    ret = pcf_drv.write(PCF_ADDRESS, PCF_REG_CONTROL_1, &_control1, 1);
    return ret;
}

uint8_t rtc_pcf_get_time_format(void){
	return tm_fmt;
}

uint8_t rtc_pcf_get_date_time(pcf_tm_t *out_pcf_tm){
    uint8_t ret = 0;
    if((pcf_init & 0x01) == 0) return 1;
    uint8_t _dt[7] = {0};

    ret = pcf_drv.read(PCF_ADDRESS, PCF_REG_SECONDS, _dt, 7);
    if((ret == 0) && (out_pcf_tm != NULL)){
    	out_pcf_tm->tm_sec = BCD2BIN(_dt[0]);
    	out_pcf_tm->tm_min = BCD2BIN(_dt[1]);
    	out_pcf_tm->tm_hour = BCD2BIN(_dt[2]);
    	out_pcf_tm->tm_mday = BCD2BIN(_dt[3]);
    	out_pcf_tm->tm_wday = BCD2BIN(_dt[4]);
    	out_pcf_tm->tm_mon = BCD2BIN(_dt[5]);
    	out_pcf_tm->tm_year = BCD2BIN(_dt[6]) + 2000 - 1900;
    }
    return ret;
}

uint8_t rtc_pcf_set_date_time(pcf_tm_t *in_pcf_tm){
    uint8_t ret = 0;
    if((pcf_init & 0x02) == 0) return 1;
    uint8_t _dt[7] = {0};

    if((in_pcf_tm != NULL)){
    	_dt[0] = BIN2BCD(in_pcf_tm->tm_sec);
    	_dt[1] = BIN2BCD(in_pcf_tm->tm_min);
    	_dt[2] = BIN2BCD(in_pcf_tm->tm_hour);
    	_dt[3] = BIN2BCD(in_pcf_tm->tm_mday);
    	_dt[4] = BIN2BCD(in_pcf_tm->tm_wday);
    	_dt[5] = BIN2BCD(in_pcf_tm->tm_mon);
    	_dt[6] = BIN2BCD(in_pcf_tm->tm_year + 1900 - 2000);
    }
	ret = pcf_drv.write(PCF_ADDRESS, PCF_REG_SECONDS, _dt, 7);
    return ret;
}

uint8_t rtc_pcf_set_date(pcf_tm_t *in_pcf_tm){
    uint8_t ret = 0;
    if((pcf_init & 0x02) == 0) return 1;
    uint8_t _dt[4] = {0};
    if(in_pcf_tm != NULL)
	_dt[0] = BIN2BCD(in_pcf_tm->tm_mday);
	_dt[1] = BIN2BCD(in_pcf_tm->tm_wday);
	_dt[2] = BIN2BCD(in_pcf_tm->tm_mon);
	_dt[3] = BIN2BCD(in_pcf_tm->tm_year + 1900 - 2000);

	ret = pcf_drv.write(PCF_ADDRESS, PCF_REG_DAYS, _dt, 4);
    return ret;
}

uint8_t rtc_pcf_get_date(pcf_tm_t *out_pcf_tm){
    uint8_t ret = 0;
    if((pcf_init & 0x01) == 0) return 1;
    uint8_t _dt[4] = {0};
    ret = pcf_drv.read(PCF_ADDRESS, PCF_REG_DAYS, _dt, 4);
    if((ret == 0) && (out_pcf_tm != NULL)){
    	out_pcf_tm->tm_mday = BCD2BIN(_dt[0]);
    	out_pcf_tm->tm_wday = BCD2BIN(_dt[1]);
    	out_pcf_tm->tm_mon = BCD2BIN(_dt[2]);
    	out_pcf_tm->tm_year = BCD2BIN(_dt[3]) + 2000 - 1900;
    }
    return ret;
}

uint8_t rtc_pcf_set_time(pcf_tm_t *in_pcf_tm){
    uint8_t ret = 0;
    if((pcf_init & 0x02) == 0) return 1;

    uint8_t _time[3] = {0};

    if(in_pcf_tm != NULL){
        _time[0] = BIN2BCD(in_pcf_tm->tm_sec);
        _time[1] = BIN2BCD(in_pcf_tm->tm_min);
        _time[2] = BIN2BCD(in_pcf_tm->tm_hour);
    }
    ret = pcf_drv.write(PCF_ADDRESS, PCF_REG_SECONDS, _time, 3);

    return ret;
}


uint8_t rtc_pcf_get_time(pcf_tm_t *out_pcf_tm){
    uint8_t ret = 0;
    if((pcf_init & 0x01) == 0) return 1;

    uint8_t _time[3] = {0};

    ret = pcf_drv.read(PCF_ADDRESS, PCF_REG_SECONDS, _time, 3);
    if((ret == 0 ) && (out_pcf_tm != NULL)){
		out_pcf_tm->tm_sec = BCD2BIN(_time[0]);
		out_pcf_tm->tm_min = BCD2BIN(_time[1]);
		out_pcf_tm->tm_hour = BCD2BIN(_time[2]);
    }
    return ret;
}


