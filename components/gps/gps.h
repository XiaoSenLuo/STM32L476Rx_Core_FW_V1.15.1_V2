/*
 * gps.h
 *
 *  Created on: 2020年11月21日
 *      Author: XIAOSENLUO
 */

#ifndef GPS_H_
#define GPS_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>

#include "minmea/minmea.h"


#ifndef ATGM336H
#define ATGM336H                       1
#endif


#if defined(ATGM336H) && (ATGM336H == 1)

typedef struct {
    uint8_t (*write)   (uint8_t*, uint32_t);
    uint8_t (*read)    (uint8_t*, uint32_t);
}gps_drv_t;


typedef struct gps_ctrl_output{
    uint8_t nGGA;       /** GGA 输出频率, 语句输出频率是以定位更新率为基准的, n(0~9)表示每 n次定位输出一次, 0 表示不输出该语句, 空则保持原有配置。 **/
    uint8_t nGLL;       /** GLL 输出频率,  同 nGGA **/
    uint8_t nGSA;       /** GSA 输出频率,  同 nGGA **/
    uint8_t nGSV;       /** GSV 输出频率,  同 nGGA **/
    uint8_t nRMC;
    uint8_t nVTG;
    uint8_t nZDA;
    uint8_t nANT;
    uint8_t nDHV;
    uint8_t nLPS;
    uint8_t nUTC;
    uint8_t nGST;
    uint8_t nTIM;
}gps_ctrl_output_t;

typedef struct {
    uint8_t baudrate; /** 0=4800bps
                           1=9600bps (默认)
                           2=19200bps
                           3=38400bps
                           4=57600bps
                           5=115200bps  **/
    uint16_t up_rate;  /** 定位更新时间间隔， 单位为 ms。
                           1000=更新率为 1Hz， 每秒输出 1 个定位点
                           500=更新率为 2Hz， 每秒输出 2 个定位点
                           250=更新率为 4Hz， 每秒输出 4 个定位点
                           200=更新率为 5Hz， 每秒输出 5 个定位点
                           100=更新率为 10Hz， 每秒输出 10 个定位点  **/
    gps_ctrl_output_t* control_output;      /** 设置要求输出或停止输出的 NMEA 语句。
                                                NULL=默认配置  **/
    uint8_t gps_mode;  /** 1=GPS
                           2=BDS
                           3=GPS+BDS
                           4=GLONASS
                           5=GPS+GLONASS
                           6=BDS+GLONASS
                           7=GPS+BDS+GLONASS **/
    uint8_t rev;       /** 2 兼容 NMEA 4.1 以上版本
                                                                    5 兼容中国交通运输信息中心的 BDS/GPS 双模协议， 兼容 NMEA 2.3 以上版本， 兼容NMEA4.0 协议
                                                                    9 兼容单 GPS NMEA0183 协议， 兼容 NMEA 2.2 版本  **/
    uint8_t info;      /** 0=查询固件版本号
                           1=查询硬件型号及序列号
                           2=查询多模接收机的工作模式
                           3=查询产品的客户编号
                           5=查询升级代码信息  **/
    uint8_t restart;   /** 0=热启动。 不使用初始化信息， 备份存储中的所有数据有效。
                           1=温启动。 不使用初始化信息， 清除星历。
                           2=冷启动。 不使用初始化信息， 清除备份存储中除配置外的所有数据。
                           3=出厂启动。 清除内存所有数据， 并将接收机复位至出厂默认配置。    **/
}gps_config_t;

#define gps_power_down()   LL_GPIO_ResetOutputPin(VGPS_EN_GPIO_Port, VGPS_EN_Pin)
#define gps_power_up()     LL_GPIO_SetOutputPin(VGPS_EN_GPIO_Port, VGPS_EN_Pin)

void gps_drv_init(void* func_write, void* func_read);

/**
 * 将当前配置信息保存到 FLASH 中， 即使接收机完全断电， FLASH 中的信息不丢失
 */
void gps_save_config_to_flash(void);

void gps_restart(uint8_t in_restart_mode);
void gps_set_baudrate(uint32_t in_baudrate);


/**
 * Funcsion: check ANT is OK
 * return: 0-OK, 1-ANT is not connect
 */
uint8_t gps_check_ant(void);

bool gps_sentence_is_ready(void);

/**
 * 获取时区
 */
int8_t gps_get_time_zone(void);

/**
 * 配置GPS
 */
void gps_config(gps_config_t* gps_cfg);

//uint8_t gps_recieve_sentence(char* buffer);

typedef struct minmea_sentence_zda gps_time_t;
typedef struct minmea_sentence_gll gps_gll_t;

gps_time_t gps_get_time(void);

gps_gll_t gps_get_gll(void);

void gps_uart_init(uint32_t _baurate);

void gps_uart_deinit(void);

int gps_uart_get_sentence(uint8_t *ptr, uint16_t s);


#else
#error "GPS ERROR"
#endif

#endif /* GPS_H_ */
