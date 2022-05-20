/*
 * bsp_ads127.h
 *
 *  Created on: 2022年1月18日
 *      Author: XIAO
 */

#ifndef ADS127_BSP_ADS127_H_
#define ADS127_BSP_ADS127_H_

#if(0)
#if(1)
/***
 * tx_ptr: tx buffer, if it is NULL, then will use default tx buffer
 * rx_ptr: rx buffer, if it is NULL, then will use default rx buffer
 * in_size: buffer size, if it is 0, then will use default size.
 */
uint8_t ads_read_data_by_dma_init(uint8_t *tx_ptr, uint8_t *rx_ptr, uint16_t in_size);
void ads_register_drdy_callback(void *callback_function);


void ads_rdata_spi_init(void);

/**
 * in_size: number of ads127
 *
 * out_cal_value: 补码
 *
 */
uint8_t ads_bsp_calibrate(uint32_t *out_cal_value, uint8_t in_size);

uint8_t ads_get_read_status(void);
void ads_read_data_complete(void);
void ads_bsp_selete_cs_by_index(uint8_t in_index);
void ads_bsp_pin_stop(void);
void ads_bsp_pin_start(void);
void ads_bsp_power(uint8_t in_status);


/**
 * FATFS支持
 */
uint8_t ads_save_data_to_file(void* in_file, void* in_parma);


uint8_t ads_user_read_fild(uint8_t in_fild);
uint8_t ads_data_is_buffer_full(void);

uint8_t* get_ads_data_buffer(void);

#else

uint8_t ads_read_data_by_dma_init(uint8_t *tx_ptr, uint8_t *rx_ptr, uint16_t in_size);

/**
 * in_size: number of ads127
 *
 * out_cal_value: 补码
 *
 */
uint8_t ads_bsp_calibrate(uint32_t *out_cal_value, uint8_t in_size);
void ads_bsp_selete_cs_by_index(uint8_t in_index);
void ads_bsp_pin_stop(void);
void ads_bsp_pin_start(void);
void ads_bsp_power(uint8_t in_status);

/**
 * FATFS支持
 */
uint8_t ads_save_data_to_file(void* in_file, void* in_parma);



void ads_config_spi_init(void);
void ads_rdata_spi_init(void);

#endif

#else

#include "stm32l4xx_ll_gpio.h"

void ads127_driver_initialaiz(SPI_HandleTypeDef *spihandle, GPIO_TypeDef * csPort, int32_t csPin);


typedef struct ads127_data_frame_s{
    struct{
        uint8_t tx_data[10];
        uint8_t rx_data[10];
    };
    struct{
        uint8_t cmd_length : 2;    /// range 0-2
        uint8_t data_length : 3;   /// range 1-8
    };
}ads127_data_frame_t;

typedef struct ads127_data_frame_s * ads127_data_frame_handle;

uint32_t ads127_data_frame(ads127_data_frame_handle handle);    /// 发送数据帧, 阻塞运行

typedef struct ads_data_init_s{
    union{
        struct{
            uint8_t crcb : 1;             /// 指示状态字类型 0-CRC4, 1-CRC8
            uint8_t cs_enb : 1;           /// 指示状态字是否开启 0-使能, 1-关闭
            uint8_t bit2_7 : 6;
        };
        uint8_t val;
    }config;
    uint32_t crate;          /// 目标采样率
    int16_t osr;             /// 过采样因子
    uint32_t clk;            /// 采样时钟
}ads_data_init_t;

#define ADS_DATA_INIT_DEFAULT()

void ads127_bsp_read_init(const ads_data_init_t* init);

void ads127_bsp_read_data_from_isr(void* ctx);        /// 启动数据读取流程, 非阻塞, ads127_bsp_read_data_cplt_callback()
                                             /// 应该被调用当数据读取完成时
void ads127_bsp_read_data_cplt_callback(void);   /// 数据读取完成回调, 此函数应该在SPI接收完成回调函数中调用

typedef void (*ads_drdy_callback_t)(void*);

void ads127_bsp_drdy_isr_install(GPIO_TypeDef * drdyPort, int32_t drdyPin, ads_drdy_callback_t fn, void *ctx);

void ads127_bsp_enable_drdy(void);
void ads127_bsp_disable_drdy(void);

void ads127_bsp_start_pin_initial(GPIO_TypeDef * startPort, int32_t startPin);

void ads127_bsp_start(void);
void ads127_bsp_stop(void);

void ads127_bsp_reset_pin_initial(GPIO_TypeDef * resetPort, int32_t resetPin);

void ads127_bsp_reset(void);


//// data stream support

int ads127_data_stream_write(void *fn, void *ctx);


#endif

#endif /* ADS127_BSP_ADS127_H_ */
