/*
 * gps.c
 *
 *  Created on: 2020年11月21日
 *      Author: XIAOSENLUO
 */

#include "gps.h"
#include <stdarg.h>

extern void HAL_Delay(uint32_t Delay);


#define gps_delay_ms(ms)    HAL_Delay(ms)

#define GPS_ACK_TIME        500

//static char gps_sentence_buffer[MINMEA_MAX_LENGTH];
static void config_string(char* ptr, size_t cnt, const char* format, ...);

static uint8_t gps_driver_is_init = 0;

static gps_drv_t gps_drv = {.write = NULL, .read = NULL};

static uint8_t gps_pps_staus = 0;

static void config_string(char* ptr, size_t cnt, const char* format, ...){
	uint8_t cs, n;
	va_list arp;

	va_start(arp, format);
	memset(ptr, 0, cnt);
	n = vsnprintf(ptr, cnt, format, arp);
	va_end(arp);
	cs = minmea_checksum(ptr);
	snprintf(&ptr[n], (cnt - n), "%X\r\n", cs);
}


void gps_drv_init(void* func_write, void* func_read){
	if(func_write != NULL) gps_drv.write = func_write;
	if(func_read != NULL) gps_drv.read = func_read;
	gps_driver_is_init = 1;
}

uint8_t gps_update(uint8_t* out_data, uint8_t s){

    if(!gps_driver_is_init) return 1;

    uint8_t ret = 1;
//    uint32_t start_tick = 128;

    memset(out_data, 0, (size_t)s);
//    while(ret != 0 && (start_tick > 0)){
//    	start_tick--;
//    	if((start_tick % 32) == 0) ret = gps_drv.read(out_data, s);
//    }
    ret = gps_drv.read(out_data, s);
//    if((ret != 0) && (!start_tick)){
//    	ret = 2;
//    }
    return ret;
}

/**
 * 将当前配置信息保存到 FLASH 中， 即使接收机完全断电， FLASH 中的信息不丢失
 */
void gps_save_config_to_flash(void){
	if(!gps_driver_is_init) return;

    char pcas00[16];
    config_string(pcas00, sizeof(pcas00), "$PCAS00*");
    gps_drv.write((uint8_t*)pcas00, (uint32_t)strlen(pcas00));
    gps_delay_ms(GPS_ACK_TIME);
}


void gps_restart(uint8_t in_restart_mode){
	if(!gps_driver_is_init) return;
	char pcas[24];

    config_string(pcas, sizeof(pcas), "$PCAS10,%d*", in_restart_mode);
    gps_drv.write((uint8_t*)pcas, strlen(pcas));
    gps_delay_ms(GPS_ACK_TIME);
}

void gps_set_baudrate(uint32_t in_baudrate){
	char pcas[64];
	uint8_t _baudrate_index  = 1;

	switch(in_baudrate){
	case 4800:
		_baudrate_index = 0;
		break;
	case 9600:
		_baudrate_index = 1;
		break;
	case 19200:
		_baudrate_index = 2;
		break;
	case 38400:
		_baudrate_index = 3;
		break;
	case 57600:
		_baudrate_index = 4;
		break;
	case 115200:
		_baudrate_index = 5;
		break;
	default:
		_baudrate_index = 1;
		break;
	}
	config_string(pcas, sizeof(pcas), "$PCAS01,%d*", _baudrate_index);
	gps_drv.write((uint8_t*)pcas, strlen(pcas));
	gps_delay_ms(GPS_ACK_TIME);
}


/**
 * 配置GPS
 */
void gps_config(gps_config_t* gps_cfg){
	if(!gps_driver_is_init) return;
    char pcas[64];
    uint8_t ret = 0;
//    size_t s = sizeof(pcas);
    ret = ret;
    config_string(pcas, sizeof(pcas), "$PCAS01,%d*", gps_cfg->baudrate);  // 配置波特率
    ret = gps_drv.write((uint8_t*)pcas, strlen(pcas));
    gps_delay_ms(GPS_ACK_TIME);
    config_string(pcas, sizeof(pcas), "$PCAS02,%d*", gps_cfg->up_rate);   // 设置定位更新频率
    ret = gps_drv.write((uint8_t*)pcas, strlen(pcas));
    gps_delay_ms(GPS_ACK_TIME);
    if(gps_cfg->control_output == NULL){
    	config_string(pcas, sizeof(pcas), "$PCAS03,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,,,%d,%d,,,,%d*",
    			1,1,0,
				0,0,0,
				1,1,0,
				0,0,0,
				0);
    }else{
        config_string(pcas, sizeof(pcas), "$PCAS03,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,,,%d,%d,,,,%d*",
        		gps_cfg->control_output->nGGA, gps_cfg->control_output->nGLL, gps_cfg->control_output->nGSA,
    			gps_cfg->control_output->nGSV, gps_cfg->control_output->nRMC, gps_cfg->control_output->nVTG,
    			gps_cfg->control_output->nZDA, gps_cfg->control_output->nANT, gps_cfg->control_output->nDHV,
    			gps_cfg->control_output->nLPS, gps_cfg->control_output->nUTC, gps_cfg->control_output->nGST,
    			gps_cfg->control_output->nTIM
    			); // 设置输出语句, 顺序不可改变
    }
    ret = gps_drv.write((uint8_t*)pcas, strlen(pcas));
    gps_delay_ms(GPS_ACK_TIME);
    config_string(pcas, sizeof(pcas), "$PCAS04,%d*", gps_cfg->gps_mode);
    ret = gps_drv.write((uint8_t*)pcas, strlen(pcas));
    gps_delay_ms(GPS_ACK_TIME);
//    config_string(pcas, sizeof(pcas), "$PCAS05,%d*", gps_cfg->rev);
//    gps_drv.write(pcas, strlen(pcas));
//
//    config_string(pcas, sizeof(pcas), "$PCAS06,%d*", gps_cfg->info);
//    gps_drv.write(pcas, strlen(pcas));
}


uint8_t gps_check_ant(uint8_t* in_data){
	size_t len = 0;
    const char ant_ok[] = "$GPTXT,01,01,01,ANTENNA OK*35\r\n";
    if(in_data[0] != '$') return 2;
    len = strlen(in_data);
    if(strcmp((char*)in_data, ant_ok) == 0){
    	return 0;
    }else{
    	return 1;
    }
}

uint8_t gps_check_pps(int8_t in_status){
    if(in_status != -1){
    	gps_pps_staus = in_status;
    	return 0;
    }else{
    	return gps_pps_staus;
    }
}

int8_t gps_get_time_zone(struct minmea_float in_longitude){
    int8_t _time_zone = 0;
    int8_t _offset = 0;

    int32_t _ulon = (int32_t)in_longitude.value / in_longitude.scale / 100;
    _time_zone = (int8_t)(_ulon / 15);
    _offset = (_ulon % 15);

    if((_offset * 10) > 75){
    	_time_zone += ((in_longitude.value > 0) ? 1 : -1);
    }
    return _time_zone;
}


