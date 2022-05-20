/*
 * ads127.c
 *
 *  Created on: 2020年11月30日
 *      Author: XIAOSENLUO
 */

#include "ads127.h"
#include <stdlib.h>
#include <string.h>


void ads127_command_reset(void){
    ads127_data_frame_t data_frame = {
            .data_length = 0,
            .cmd_length = 1,
            .tx_data = {ADS127_COMMAND_RESET, 0, 0, 0, 0},
            .rx_data = {0, 0, 0, 0, 0},
    };
    ads127_data_frame(&data_frame);
}

void ads127_command_start(void){
    ads127_data_frame_t data_frame = {
            .data_length = 0,
            .cmd_length = 1,
            .tx_data = {ADS127_COMMAND_START, 0, 0, 0, 0},
            .rx_data = {0, 0, 0, 0, 0},
    };
    ads127_data_frame(&data_frame);
}

void ads127_command_stop(void){
    ads127_data_frame_t data_frame = {
            .data_length = 0,
            .cmd_length = 1,
            .tx_data = {ADS127_COMMAND_STOP, 0, 0, 0, 0},
            .rx_data = {0, 0, 0, 0, 0},
    };
    ads127_data_frame(&data_frame);
}

uint32_t ads127_command_rdata(void){
    ads127_data_frame_t data_frame = {
            .data_length = 4,
            .cmd_length = 1,
            .tx_data = {ADS127_COMMAND_RDATA, 0, 0, 0, 0},
            .rx_data = {0, 0, 0, 0, 0},
    };
    return ads127_data_frame(&data_frame);
}

void ads127_configure(ads127_dev_t * device){
    ads127_data_frame_t data_frame = {
            .rx_data = {0, 0, 0, 0, 0},
            .cmd_length = 2,
            .data_length = 1,
            .tx_data = {ADS127_COMMAND_WREG_BASE | ADS127_REG_CONFIG, 0, device->config.val, 0, 0},
    };
    ads127_data_frame(&data_frame);

#if(0)
    data_frame.tx_data[0] = ADS_COMMAND_WREG_BASE | ADS_REG_OFC0;   /// 写命令 + 起始地址
    data_frame.tx_data[1] = 3 - 1;    /// 寄存器个数
    data_frame.tx_data[2] = device->ofc.ofc0;
    data_frame.tx_data[3] = device->ofc.ofc1;
    data_frame.tx_data[4] = device->ofc.ofc2;
    data_frame.data_length = 3;
    ads127_data_frame(&data_frame);

    data_frame.tx_data[0] = ADS_COMMAND_WREG_BASE | ADS_REG_FSC0;
    data_frame.tx_data[1] = 2 - 1;
    data_frame.tx_data[2] = device->fsc.fsc0;
    data_frame.tx_data[3] = device->fsc.fsc1;
    data_frame.tx_data[4] = 0;
    data_frame.data_length = 2;
    ads127_data_frame(&data_frame);
#endif
}

uint8_t ads127_get_configure(ads127_dev_t *device){
    uint32_t ret = 0;
    ads127_data_frame_t data_frame = {
            .rx_data = {0, 0, 0, 0, 0},
            .cmd_length = 2,
            .data_length = 1,
            .tx_data = {ADS127_COMMAND_RREG_BASE | ADS127_REG_CONFIG, 0, 0, 0, 0},
    };
    ret = ads127_data_frame(&data_frame);
    device->config.val = (uint8_t)ret;
    return (uint8_t)ret;
}

void ads127_configure_ofc(ads127_dev_t *device){
    ads127_data_frame_t data_frame = {
            .rx_data = {0, 0, 0, 0, 0},
            .cmd_length = 2,
            .data_length = 3,
            .tx_data = {ADS127_COMMAND_WREG_BASE | ADS127_REG_OFC0, 2, device->ofc.ofc0, device->ofc.ofc1, device->ofc.ofc2, 0},
    };
    ads127_data_frame(&data_frame);
}

uint32_t ads127_get_ofc(ads127_dev_t *device){
    uint32_t ret = 0;
    ads127_data_frame_t data_frame = {
            .rx_data = {0, 0, 0, 0, 0},
            .cmd_length = 2,
            .data_length = 3,
            .tx_data = {ADS127_COMMAND_RREG_BASE | ADS127_REG_OFC0, 2, 0, 0, 0},
    };
    ret = ads127_data_frame(&data_frame);
    device->ofc.val = ret;
    return (uint32_t)ret;
}

void ads127_configure_fsc(ads127_dev_t *device){
    ads127_data_frame_t data_frame = {
            .rx_data = {0, 0, 0, 0, 0},
            .cmd_length = 2,
            .data_length = 2,
            .tx_data = {ADS127_COMMAND_WREG_BASE | ADS127_REG_FSC0, 1, device->fsc.fsc0, device->fsc.fsc1, 0, 0},
    };
    ads127_data_frame(&data_frame);
}

uint16_t ads127_get_fsc(ads127_dev_t *device){
    uint32_t ret = 0;
    ads127_data_frame_t data_frame = {
            .rx_data = {0, 0, 0, 0, 0},
            .cmd_length = 2,
            .data_length = 2,
            .tx_data = {ADS127_COMMAND_RREG_BASE | ADS127_REG_FSC0, 1, 0, 0, 0},
    };
    ret = ads127_data_frame(&data_frame);
    device->fsc.val = (uint16_t)ret;
    return (uint16_t)ret;
}

uint8_t ads127_get_id(ads127_dev_t *device){
    uint32_t ret = 0;
    ads127_data_frame_t data_frame = {
            .rx_data = {0, 0, 0, 0, 0},
            .cmd_length = 2,
            .data_length = 1,
            .tx_data = {ADS127_COMMAND_RREG_BASE | ADS127_REG_ID, 0, 0, 0, 0},
    };
    ret = ads127_data_frame(&data_frame);
    device->id.val = (uint8_t)ret;
    return (uint8_t)ret;
}


uint8_t ads127_get_mode(ads127_dev_t * device){
    uint32_t ret = 0;
    ads127_data_frame_t data_frame = {
            .rx_data = {0, 0, 0, 0, 0},
            .cmd_length = 2,
            .data_length = 1,
            .tx_data = {ADS127_COMMAND_RREG_BASE | ADS127_REG_MODE, 0, 0, 0, 0},
    };
    ret = ads127_data_frame(&data_frame);
    device->mode.val = (uint8_t)ret;
    return (uint8_t)ret;
}

