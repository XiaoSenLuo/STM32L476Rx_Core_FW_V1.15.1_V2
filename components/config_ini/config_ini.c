/*
 * config_ini.c
 *
 *  Created on: 2020年12月11日
 *      Author: XIAOSENLUO
 */

#include "config_ini.h"
#include "stddef.h"
#include <ctype.h>

static const char* config_section[] = {
		"[system]",
		"[sd-tf]",
		"[ads127]",
		"[gps]",
		"[rtc]",
};

volatile uint32_t *system_config_buffer_ptr = NULL;
static const char* system_config_keyname[] = {
		"system_hse_value",
		"system_core_freq",
		"system_rco_div",
		"system_uart_baudrate",
		"system_cmd_connect_time",
		"system_battery_voltage",
		"system_battery_cutoff_voltage",
		"system_battery_thold_voltage",
		"system_battery_error_voltage",
};

static const char* sd_tf_config_keyname[] = {
		"sd_format",
        "sd_format_file_system",
		"sd_file_size_limit",
};

static const char* ads127_config_keyname[] = {
		"ads_conver_rate",
		"ads_mode",
		"ads_start_time",
		"ads_offset_calibration",
		"ads_gain_calibration",
		"ads_data_file_format",
		"ads_osr",
};

static const char* gps_config_keyname[] = {
		"gps_check",
		"gps_startup",
		"gps_next_startup_time",
		"gps_uart_baudrate",
		"gps_up_rate",
		"gps_mode",
};

static const char* rtc_config_keyname[] = {
		"rtc_ex_input_freq",
		"rtc_ex_cal_offset",
};

uint32_t *sys_config_init(void){
    uint32_t *ret = NULL;
    uint8_t sec_size = 0;
    uint8_t i = 0;
    size_t buff_size = 0;
    char* sec = NULL;
    char* *key = NULL;
    uint8_t key_size = 0;
    sec_size = get_section_size();
    for(i = 0; i < sec_size; i++){
    	sec = get_section_by_index(i);
    	key = get_keyname_by_section(sec, &key_size);
    	if(key != NULL) buff_size += key_size;
    	else continue;
    }
    ret = (uint32_t*)malloc(buff_size);
    return ret;
}

void sys_config_deinit(uint32_t *in_sys_config){
    free(in_sys_config);
    in_sys_config = NULL;
}

size_t get_section_size(void){
	return (size_t)(sizeof(config_section) / sizeof(char*));
}

char* get_section_by_index(uint8_t in_index){
	uint8_t _sec_size = 0;
	_sec_size = sizeof(config_section) / sizeof(char*);

	if(in_index < _sec_size){
		return (char*)config_section[in_index];
	}else{
		return NULL;
	}
}

char** get_keyname_by_section(char* in_section, uint8_t* out_keyname_size){
	uint8_t i = 0;
	uint8_t _sec_size = 0;

	_sec_size = sizeof(config_section) / sizeof(char*);

	for(i = 0; i < _sec_size; i++){
		if(strcmp(config_section[i], (char*)in_section) == 0){
			switch(i){
			case 0:  // [system]
				if(out_keyname_size != NULL){
					*out_keyname_size = sizeof(system_config_keyname) / sizeof(char*);
				}
                return (char**)system_config_keyname;
				break;
			case 1:  // [sd-tf]
				if(out_keyname_size != NULL){
					*out_keyname_size = sizeof(sd_tf_config_keyname) / sizeof(char*);
				}
                return (char**)sd_tf_config_keyname;
				break;
			case 2:  // [ads127]
				if(out_keyname_size != NULL){
					*out_keyname_size = sizeof(ads127_config_keyname) / sizeof(char*);
				}
                return (char**)ads127_config_keyname;
				break;
			case 3:  // [gps]
				if(out_keyname_size != NULL){
					*out_keyname_size = sizeof(gps_config_keyname) / sizeof(char*);
				}
                return (char**)gps_config_keyname;
				break;
			case 4:  // RTC
				if(out_keyname_size != NULL){
					*out_keyname_size = sizeof(rtc_config_keyname) / sizeof(char*);
				}
                return (char**)rtc_config_keyname;
				break;
			default:
				return NULL;
				break;
			}
	    }
	}
    return NULL;
}


uint8_t is_config_section(uint8_t* in_data){
    uint8_t vail = 0;
    uint8_t len = 0, i = 0;
    uint8_t fild = 0;

    len = (uint8_t)strlen((char*)in_data);

    if((len < 3) || (in_data[0] == ';') || (in_data[0] == '\r') || (in_data[0] == '\n')){
        vail = 1;
    }else{
        while((in_data[i] != '\0')){
            if((in_data[i] == '[') && (fild == 0)) fild = 1;
            if(in_data[i] != ' ' && !fild){
                vail = 1;
                break;
            }
            if((fild == 2) && ((in_data[i] != ' ') && (in_data[i] != '\r') && (in_data[i] != '\n'))){
                vail =1;
                break;
            }
            if((in_data[i] == ']') && (fild == 1)) fild = 2;
            i += 1;
        }
    }
    return vail;
}



uint8_t check_config_section(uint8_t* in_data, const char* in_section, uint8_t* out_section){
    uint8_t vail = 0;
    uint8_t len = 0;
    uint8_t start_i = 0, stop_i = 0, i = 0;
    uint8_t fild = 0;
    uint8_t c_sec_len = 0;
    uint8_t c_sec_fild = 0;

    len = (uint8_t)strlen((char*)in_data);

    if((len < 3) || (in_data[0] == ';') || (in_data[0] == '\r') || (in_data[0] == '\n')){
        vail = 1;
    }else{
        for(i = 0; i < len; i++){
            if((!fild) && (in_data[i] == '[')){
                start_i = i;
                fild = 1;
            }
            if((fild == 1) && (in_data[i] == ']')){
                stop_i = i;
                fild = 2;
            }
            if(((fild == 0) && (in_data[i] != ' '))
                || ((i == len - 1) && (stop_i == 0))
                || ((fild == 2) && (i > stop_i) && (in_data[i] != ' ') && (in_data[i] != '\r') && (in_data[i] != '\n'))
                ){
                vail = 1;
                break;
            }
        }
        if(!fild) vail = 1;
        if(vail == 0){
            if(in_section != NULL){
                if(memcmp(&in_data[start_i], in_section, (stop_i - start_i + 1)) != 0){
                    vail = 1;
                }
            }else{
            	for(i = 0; i < c_sec_len; i++){
                    if(memcmp(&in_data[start_i], (char*)config_section[i], (stop_i - start_i + 1)) == 0){
                        c_sec_fild = 1;
                    }
                    if((i == c_sec_len - 1) && (!c_sec_fild)){
                    	vail = 1;
                    }
            	}
            }
            if((vail == 0) && (out_section != NULL)){
                memcpy(out_section, &in_data[start_i], (stop_i - start_i + 1));
                out_section[stop_i - start_i + 1] = '\0';
            }
        }
    }
    return vail;
}


uint8_t get_config_value_by_keyname(uint8_t* in_data, const char* keyname, void* out_value, char opt){
    uint8_t vail = 0;
    uint8_t fild_index = 0;
    uint8_t i = 0;
    uint8_t len = 0, keyname_len = 0;
    char* value_str = NULL;
    char* _k = NULL;

    len = strlen((char*)in_data);
    if((in_data[len - 1] == '\r') || (in_data[len - 1] == '\n')) len -= 1;
    if((in_data[len - 1] == '\r') || (in_data[len - 1] == '\n')) len -= 1;
    keyname_len = strlen(keyname);
    _k = (char*)malloc(keyname_len + 1);
    memset(_k, 0, keyname_len + 1);
    memcpy(_k, keyname, keyname_len + 1);
    if(len <= keyname_len){
        vail = 1;
    }else{
        for(i = 0; i < len; i++){
            if(in_data[i] == '='){
            	fild_index = i;
            	break;
            }
            if((i == len - 1) && (!fild_index)) vail = 1;
        }
        if(vail == 0){
            if(memcmp(&in_data[fild_index - keyname_len], _k, keyname_len) == 0){  // 检查键名
                value_str = malloc(len - fild_index + 1);
                if(value_str == NULL) vail = 1;
                else{
                    memset(value_str, 0, len - fild_index);
//                    memcpy(value_str, &in_data[fild_index + 1], (size_t)(len - fild_index)); // '=' 后的数值
                    strcpy(value_str, (char*)&in_data[fild_index + 1]);
                    char* end_ptr = NULL;
                    char* c_out = NULL;
                    switch(opt){
                        case 'i': // 整数
                        	if(isdigit((unsigned char)value_str[0]) == 0){  // 不是数字
                        		vail = 1;
                        		break;
                        	}
                            *(int32_t*)out_value = strtol(value_str, &end_ptr, 10);  // 10 进制
                            if((*end_ptr == ' ') || (*end_ptr == '\r') || (*end_ptr == '\n')){

                            }else{
                            	vail = 1;
                            }
                            break;
                        case 'c': // 字符
                            c_out = (char*)out_value;
                            strcpy(c_out, value_str);
                            c_out[len - fild_index] = '\0';
                            break;
                        case 'f':  // 浮点数
                        	if(isdigit((unsigned char)value_str[0]) == 0){   // 不是数字
                        		vail = 1;
                        		break;
                        	}
                            *(float*)out_value = strtof(value_str, &end_ptr);
                            if((*end_ptr == ' ') || (*end_ptr == '\r') || (*end_ptr == '\n')){

                            }else{
                            	vail = 1;
                            }
                            break;
                        case 'd':  // 双精度浮点数
                        	if(isdigit((unsigned char)value_str[0]) == 0){   // 不是数字
                        		vail = 1;
                        		break;
                        	}
                            *(double*)out_value = strtod(value_str, &end_ptr);
                            if((*end_ptr == ' ') || (*end_ptr == '\r') || (*end_ptr == '\n')){

                            }else{
                            	vail = 1;
                            }
                            break;
                    }
                }
                free(value_str);
            }else{
                vail = 1;
            }
        }
    }
    free(_k);
    return vail;
}

