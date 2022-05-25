//
// Created by XIAOSENLUO on 2022/5/20.
//

#ifndef STM32L476RX_ADS127_DRIVER_H
#define STM32L476RX_ADS127_DRIVER_H


#include "stm32l4xx_hal.h"

void ads127_driver_initialize(SPI_HandleTypeDef *spihandle, GPIO_TypeDef * csPort, int32_t csPin);

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

void ads127_cs_set_level(uint8_t level);

uint32_t ads127_data_frame(ads127_data_frame_handle handle);    /// 发送数据帧, 阻塞运行



#endif //STM32L476RX_ADS127_DRIVER_H
