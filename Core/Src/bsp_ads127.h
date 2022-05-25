/*
 * bsp_ads127.h
 *
 *  Created on: 2022年1月18日
 *      Author: XIAO
 */

#ifndef ADS127_BSP_ADS127_H_
#define ADS127_BSP_ADS127_H_

#include "ads127.h"
#include "ads127_driver.h"
#include "stdbool.h"


typedef struct ads_data_init_s{
    union{
        struct{
            uint8_t crcb : 1;             /// 指示状态字类型 0-CRC4, 1-CRC8
            uint8_t cs_enb : 1;           /// 指示状态字是否开启 0-使能, 1-关闭
            uint8_t bit2_7 : 6;
        };
        uint8_t val;
    }config;
    union{
        struct{
            uint8_t fsmode : 1;
            uint8_t format : 1;
            uint8_t filter : 2;
            uint8_t osr : 2;
            uint8_t hr : 1;
            uint8_t bit7 : 1;
        };
        uint8_t val;
    }mode;
    uint32_t clk;            /// 采样时钟
}ads_data_init_t;

#define ADS_DATA_INIT_DEFAULT()

typedef void (*ads_drdy_callback_t)(void*);

void ads127_bsp_drdy_isr_install(GPIO_TypeDef * drdyPort, int32_t drdyPin, ads_drdy_callback_t fn, void *ctx);

void ads127_bsp_enable_drdy(void);
void ads127_bsp_disable_drdy(void);

void ads127_bsp_read_init(const ads_data_init_t* init);

void ads127_bsp_read_data_from_isr(void* ctx);        /// 启动数据读取流程

void ads127_bsp_read_data_cplt_callback(void);   /// 数据读取完成回调, 此函数应该在SPI接收完成回调函数中调用


static inline void ads127_bsp_start_read_data(void){
    ads127_bsp_enable_drdy();
}

static inline void ads127_bsp_stop_read_data(void){
    ads127_bsp_disable_drdy();
}

void ads127_bsp_start_pin_initial(GPIO_TypeDef * startPort, int32_t startPin);

void ads127_bsp_start(void);
bool ads127_bsp_is_start(void);
void ads127_bsp_stop(void);

void ads127_bsp_reset_pin_initial(GPIO_TypeDef * resetPort, int32_t resetPin);

void ads127_bsp_reset(void);
void ads127_bsp_keep_reset(void);

//// data stream support

#include "time.h"
#include "rtc.h"

typedef struct ads_file_header_s {
    uint64_t counter;
    uint32_t length;
    rtc_date_time_t time;
}ads_file_header_t;

typedef struct ads_file_header_s * ads_file_header_handle;

uint32_t ads127_bsp_availablle(void* *ptr);
int ads127_bsp_lock_user(void);
int ads127_bsp_unlock_user(void);
uint32_t ads127_bsp_write_file(void *file);

uint32_t ads127_bsp_create_file(void *file);

#endif /* ADS127_BSP_ADS127_H_ */
