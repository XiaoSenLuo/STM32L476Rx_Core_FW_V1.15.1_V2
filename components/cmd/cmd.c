/*
 * cmd.c
 *
 *  Created on: Jan 10, 2022
 *      Author: XIAO
 */



#include "time.h"
#include "ctype.h"
#include "stdio.h"

#include "cmd.h"
#include "rtc.h"
#include "usart.h"
#include "config_ini.h"
#include "mem_dma.h"

//static uint8_t cmd_tx_buffer[128] = {'\0'};

void cmd_log_init(uint32_t baud){
//	uart_init(USART3, baud);
	cmd_uart_init(baud);
}

uint8_t cmd_write(uint8_t* data, uint16_t len){
//	if(!(cmd_status & CMD_STATUS_INIT)) return 1;
	if(LL_USART_IsEnabled(USART3) != 1) return 1;

    return UART3_Write(data, len);
}

uint8_t cmd_printf(const char* fmt, ...){
//	if(!(cmd_status & CMD_STATUS_INIT)) return 1;
	if(LL_USART_IsEnabled(USART3) != 1) return 1;

	uint8_t ret = 0, n = 0;
    char _fmt_str[128] = {'\0'};
    va_list arp;

    va_start(arp, fmt);
    n = vsnprintf(_fmt_str, sizeof(_fmt_str), fmt, arp);
    va_end(arp);
    ret = UART3_Write((uint8_t*)_fmt_str, n);
    return ret;
}

uint8_t cmd_log_printf(const char* fmt, ...){
//	if(!(cmd_status & CMD_STATUS_INIT)) return 1;
	if(LL_USART_IsEnabled(USART3) != 1) return 1;

	uint8_t ret = 1, n = 0;
    char _fmt_str[255] = {'\0'};
    va_list arp;

    uint32_t _tick = 0;
    struct tm _tm;

    _tick = st_rtc_get_subsecond();
    st_rtc_get_time(&_tm);
    n = snprintf(_fmt_str, sizeof(_fmt_str), "\n[%d-%d-%d-%d-%d-%d.%u]: ", _tm.tm_year + 1900, _tm.tm_mon, _tm.tm_mday, _tm.tm_hour, _tm.tm_min, _tm.tm_sec, (unsigned int)_tick);
    ret = UART3_Write((uint8_t*)_fmt_str, n);

    va_start(arp, fmt);
    n = vsnprintf(_fmt_str, sizeof(_fmt_str), fmt, arp);
    va_end(arp);
    ret = UART3_Write((uint8_t*)_fmt_str, n);

    return ret;
}

uint8_t cmd_scan(const char* in_data, char* out_cmd, char* out_val, char* out_data){
	// ">set val data"
	// ">get val"
	uint8_t _ret = 0;
	size_t _in_data_len = 0;
	uint16_t i = 0;
	char* _in = NULL;
	uint8_t fild_space = 0;

	fild_space = fild_space;

	if((in_data[0] != '>')) return 1;
//	if(islower((unsigned char)in_data[1] == 0)) return 1;
	else{
		_in_data_len = strlen(in_data);
		_in = &in_data[1];
		_in_data_len -= 1;
	}

	i = 0;
	while(_in[i] != '\0'){  // 获取命令
        if(isspace(_in[i]) == 0){ // 不是空格
            *out_cmd = _in[i];
            out_cmd += 1;
        }else{  // 空格, 结束
        	*out_cmd = '\0';
        	_in = &_in[i + 1];
        	fild_space = 1;
        	break;
        }
        i += 1;
        if(_in[i] == '\0'){
            *out_cmd = '\0';
            return 0;
        }
//        if((_in[i] == '\0') && (!fild_space)){
//            _ret = 1;
//        	return _ret;
//        }
	}
	i = 0;
	fild_space = 0;
	while(_in[i] != '\0'){    // 获取参数
        if(isspace(_in[i]) == 0){ // 不是空格
            *out_val = _in[i];
            out_val += 1;
        }else{  // 空格, 结束
        	*out_val = '\0';
        	_in = &_in[i + 1];
        	fild_space = 1;
        	break;
        }
        i += 1;
//        if((_in[i] == '\0') && (!fild_space)){
//            _ret = 1;
//        	return _ret;
//        }
        if(_in[i] == '\0'){
            *out_val = '\0';
            return 0;
        }
	}
	i = 0;
	fild_space = 0;
	while((_in[i] != '\0')){   // 获取参数值
        if(isspace(_in[i]) == 0){ // 不是空格
            *out_data = _in[i];
            out_data += 1;
        }else{  // 空格, 结束
        	*out_data = '\0';
        	fild_space = 1;
        	break;
        }
        i += 1;
        if(_in[i] == '\0') *out_data = '\0';
	}
	_ret = 0;
	return _ret;
}


void printf_help(void){
	  cmd_printf("<%s:OK\r\n", "help");
	  char* *_help_file = NULL;
	  uint32_t _lines = 0, _line_size = 0, i = 0;
	  _help_file = get_default_config_file();
	  _lines = get_default_config_lines();
	  for(i = 0; i < _lines; i++){
		  _line_size = strlen(_help_file[i]);
		  UART3_Write((uint8_t*)_help_file[i], _line_size);
	  }
}
