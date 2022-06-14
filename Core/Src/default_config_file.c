/*
 * default_config_file.c
 *
 *  Created on: 2020年12月11日
 *      Author: XIAOSENLUO
 */


static const char* default_config[] = {
		"; 配置文件, 行尾必须以CRLF控制符结尾\r\n",
		";'='前后不允许有空格\r\n",
		"[system]\r\n",
		";system_hse_value: 系统外部晶振中心频率, 单位Hz, 默认: 16000000\r\n",
		"system_hse_value=16000000\r\n",
		";system_core_freq: 系统核心频率, 单位Hz, 默认: 32000000\r\n",
		"system_core_freq=32000000\r\n",
		";system_rco_div: 系统时钟分频输出, 输出时钟为ADS采样单元提供采样时钟, 输出时钟频率=system_core_freq / system_rco_div, 可选参数[1,2,4,8,16] 默认: 8\r\n",
		"system_rco_div=8\r\n",
		";system_uart_baudrate: 系统串口波特率, 默认: 115200bps\r\n",
		"system_uart_baudrate=115200\r\n",
		";system_cmd_connect_time: 系统串口连接超时, Unit: s, 默认值:300\r\n",
		"system_cmd_connect_time=300\r\n",
		";system_battery_voltage: 电池额定电压, Unit: mV\r\n",
		"system_battery_voltage=14400\r\n",
		";system_battery_cutoff_voltage: 电池放电截止电压, Unit: mV\r\n",
		"system_battery_cutoff_voltage=8000\r\n",
		";system_battery_thold_voltage: 系统停止工作电压阈值: Unit: %\r\n",
		"system_battery_thold_voltage=2\r\n",
		";system_battery_error_voltage: 电池电压修正参数, 无单位, V = (Vadc * ((R1+R2)/R2+(system_battery_error_voltage / 1000.00f))\r\n",
		"system_battery_error_voltage=-50\r\n",
		"[sd]\r\n",
		";\r\n",
		";sd_format: 是否格式化, 默认: 0(否)\r\n",
		"sd_format=0\r\n",
		";sd_format_file_system: 文件系统, 可选参数[fat32[2],exfat[4]], 默认: 2, 此参数必须在\'sd_format=1\'时才有效\r\n",
		"sd_format_file_system=2\r\n",
		";sd_file_size_limit:文件大小上限限制, 当采用 FAT32 文件系统时, 必须小于4096,  默认: 1024. Unit:MB\r\n",
		"sd_file_size_limit=1024\r\n",
		"[ads]\r\n",
		";ads_conver_rate: 采样率(Hz), 默认: 2000\r\n",
		"ads_conver_rate=2000\r\n",
		";ads_mode: 采样模式, 2-高分辨率模式, 1-低功耗模式, 0-超低功耗模式, 默认: 0\r\n",
		"ads_mode=0\r\n",
		";ads_start_time: 开始采样时间, 单位秒(s), 数值等于距离授时时间的秒数 默认: 0, 授时成功立即开始采样\r\n",
		"ads_start_time=0\r\n",
		";ads_offset_calibration: 系统误差校准, 长度24Bit, 格式为二进制补码格式, 默认: 000000. 如果Bit[31] = 1, 使用内部自动校准程序 \r\n",
		"ads_offset_calibration=000000\r\n",
		";ads_gain_calibration: 系统增益校准, 长度16Bit, 格式为二进制补码格式, (<8000h gain>1), (=8000h gain=1), (>8000h gain<1), 默认: 8000. 如果Bit[31] = 1, 使用内部自动校准程序\r\n",
		"ads_gain_calibration=8000\r\n",
		";ads数据输出: (filter_output - ads_offset_calibration) * ads_gain_calibration / 8000h\r\n",
		";ads_data_file_format: 采样数据存储文件格式, 可选参数[0, 1], [0]: 文本文件, 文件后缀名为\'.txt\'; [1]: 二进制文件, 文件后缀名为\'.bin\'; 默认: 1\r\n",
		"ads_data_file_format=1\r\n",
		";ads_osr: 过采样系数, 详细请看 ADS127L 数据手册, 默认:32\r\n",
		"ads_osr=32\r\n",
		"[gps]\r\n",
		";gps_check: 开机是否等待GPS授时, 默认:1(等待)\r\n",
		"gps_check=1\r\n",
		";gps_startup: 是否启动gps模块, 默认: 1(启动)\r\n",
		"gps_startup=1\r\n",
		";gps_next_startup_time: gps定时启动授时, 单位秒(s), 数值等于距离上次启动授时时间, 默认: 604800(一周:7*24*60*60)\r\n",
		"gps_next_startup_time=604800\r\n",
		";gps_uart_baudrate: gps模块串口通信波特率, 默认: 9600bps\r\n",
		"gps_uart_baudrate=9600\r\n",
		";gps_up_rate: GPS定位更新时间间隔, 单位毫秒(ms), 默认: 1000\r\n",
		"gps_up_rate=1000\r\n",
		";gps_mode: GPS工作模式, 1-接受GPS信号, 2-接受BDS信号, 3-接受GPS+BDS信号, 4-接受GLONASS信号, 5-接受GPS+GLONASS信号, 6-接受BDS+GLONASS信号, 7-接受GPS+BDS+GLONASS信号, 默认: 2\r\n",
		"gps_mode=2\r\n",
		"[rtc]\r\n",
		";\r\n",
		"rtc_ex_input_freq=32768\r\n",
		";详细请查看 PCF85063A 数据手册\r\n",
		"rtc_ex_cal_offset=0x00\r\n",
		};


#include "stddef.h"

size_t get_default_config_lines(void){
	return (sizeof(default_config) / sizeof(char*));
}


char** get_default_config_file(void){
    return (char**)default_config;
}

#if(0)
#include "ff.h"

uint8_t create_default_config_file(const char *file_name){
	uint32_t _lines = 0, i = 0;
    FIL _file = {0};
    int res = 0;

	_lines = sizeof(default_config) / sizeof(char*);

	res = f_open(&_file, file_name, FA_CREATE_ALWAYS | FA_WRITE);

	if(res != 0) return (uint8_t)res;

	for(i = 0; i < _lines; i++){
		res = f_puts(default_config[i], &_file);
	}
	res = f_close(&_file);
    return (uint8_t)res;
}

#endif
