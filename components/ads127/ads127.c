/*
 * ads127.c
 *
 *  Created on: 2020年11月30日
 *      Author: XIAOSENLUO
 */

#include "ads127.h"
#include <stdlib.h>
#include <string.h>


ads_drv_t ads_drv = {.write = NULL, .read = NULL, .write_read = NULL};


volatile uint8_t ads_nums = ADS_CNT;
volatile uint32_t ads_timestampe = 0;

__attribute__((weak)) void ads_drv_init(ads_write_func_t write_func, ads_read_func_t read_func, ads_write_read_func_t wr_func){
    if(write_func != NULL){
    	ads_drv.write = write_func;
    }else{
    	return;
    }
    if(read_func != NULL){
    	ads_drv.read = read_func;
    }else{
    	return;
    }
    if(wr_func != NULL){
    	ads_drv.write_read = wr_func;
    }
}



void ads_set_nums(uint8_t in_ads_num){
    ads_nums = (in_ads_num > ADS_CNT) ? ADS_CNT : in_ads_num;
}

#define __BUFFER_SIZE__             17

uint8_t ads_read_data_in_value(int32_t* data_ptr, uint8_t in_ads_num){

	uint8_t ret = 0;
    uint32_t complements[__BUFFER_SIZE__ / 2] = {0};
    uint8_t i = 0;

//    complements = (uint32_t*)malloc((size_t)in_ads_num * sizeof(uint32_t));
    if(complements != NULL){
    	ret = ads_read_data_in_complements(complements, NULL, in_ads_num);
    	if(ret != 0){
    		ret = 1;
    	}else{
    		for(i = 0; i < in_ads_num; i++){
    			data_ptr[i] = COM_TO_DEC(complements[i], 24);
    		}
    	}
//    	free(complements);
//    	complements = NULL;
    }
    return ret;
}

uint8_t ads_read_data_in_complements(uint32_t* data_ptr, uint8_t* out_status, uint8_t in_ads_num){
    uint8_t ret = 0;
    uint8_t _bytes[__BUFFER_SIZE__] = { 0 };
    uint16_t len = 0;
    uint8_t i = 0;
    uint8_t dw = 0;

    dw = ADS_STATUS_WORD_EN ? 4 : 3;
    len = dw;
	ret = ads_read_data_in_byte(_bytes, len);
	if(ret != 0){
		ret = 1;
	}else{
		for(i = 0; i < in_ads_num; i++){
			data_ptr[i] = ((uint32_t)_bytes[i * dw] << 16) | ((uint32_t)_bytes[i * dw + 1] << 8) | (uint32_t)_bytes[i * dw + 2] ;
			if((dw == 4) && (out_status)) out_status[i] = _bytes[i * dw + 3];
		}
	}
    return ret;
}

uint8_t ads_read_data_in_byte(uint8_t* data_ptr, uint8_t in_byte_num){
    uint8_t ret = 0;
    uint8_t _ads_buffer_tx[__BUFFER_SIZE__] = { 0 };
    uint8_t _ads_buffer_rx[__BUFFER_SIZE__] = { 0 };

    _ads_buffer_tx[0] = ADS_COMMAND_RDATA;
//#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)

    uint16_t ds = 0;
    ds = ((in_byte_num > (__BUFFER_SIZE__ - 1)) ? (__BUFFER_SIZE__ - 1) : in_byte_num) + 1;
    ret = ads_drv.write_read(_ads_buffer_tx, _ads_buffer_rx, ds);
    if(ret == 0){
    	memcpy(data_ptr, &_ads_buffer_rx[1], in_byte_num);
    }
#if(ADS_USE_WR_FUNC)
    #else
    ret = ads_drv.write(_ads_buffer_tx, 1);
    if(ret) return ret;     // 读写分开是因为 ADS127 有命令解码时间
    ret = ads_drv.read(data_ptr, in_byte_num);
    if(ret) return ret;
    ads_read_data_complete();
    memcpy(data_ptr, _ads_buffer_rx, in_byte_num);
#endif
//#else


//#endif
    return ret;
}

#ifndef ADS_READ_FUNC
#define ADS_READ_FUNC      1
#endif

uint8_t ads_get_device_characteristics(uint8_t* characteristics){
    uint8_t ret = 0;
#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)
    uint8_t rreg_command[2 + ADS_CNT] = {0};
    uint8_t id[ADS_CNT + 2] = {0};
#else
    uint8_t rreg_command[3] = {0};
    uint8_t id[3] = {0};
#endif
    rreg_command[0] = ADS_COMMAND_RREG_BASE | ADS_REG_ID; // start address at 0x00
    rreg_command[1] = 0x00; // read a reg

#if(ADS_USE_WR_FUNC)
#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)
    ret = ads_drv.write_read(rreg_command, id, 2 + ADS_CNT);
    if(ret == 0){
    	memcpy(characteristics, &id[2], ADS_CNT);
    }
#else
    ret = ads_drv.write_read(rreg_command, id, 3);
    if(ret == 0){
#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)
    	memcpy(characteristics, id, ADS_CNT);
#else
    	*characteristics = id[2];
#endif
    }
#endif
#else
    ret = ads_drv.write(rreg_command, 2);
    if(ret) return ret;
#if(ADS_READ_FUNC == 1)
#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)
    uint8_t i = 0;
    do{
        ret = ads_drv.read(&id[i], 1);
        if(ret) return ret;
        i += 1;
    }while(i < ADS_CNT);
#else
    ads_drv.delay(4);
    ret = ads_drv.read(id, 1);
    if(ret == 0)
    	*characteristics = id[0];
#endif
#endif
#if(ADS_READ_FUNC == 2)
    ret = ads_drv.read(id, ADS_CNT);
    if(ret) return ret;
#endif
#endif
	return ret;
}


uint8_t ads_cfg_sys(uint8_t cfg_sys){
    uint8_t ret = 0;
    uint8_t wreg_command[4] = {0};

    wreg_command[0] = (ADS_COMMAND_WREG_BASE) | ADS_REG_CONFIG;   // start address at 0x01
    wreg_command[1] = 0x00;    // write a reg
    wreg_command[2] = cfg_sys;  // config data

    ret = ads_drv.write(wreg_command, 2);
    if(ret) return ret;
    // 命令解码时间 >=  6 t_mclk
    ads_drv.delay(1);
    /**
     * datasheet rev.2016, section: 8.5.1.8.5, 8.5.1.8.6
     * .写完数据后增加适当数量时钟将数据从移位寄存器移送到配置寄存器
     */
    ret = ads_drv.write(&wreg_command[2], 2);

	return ret;
}

uint8_t ads_get_cfg_sys(uint8_t* cfg_sys){
	uint8_t ret = 0;
#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)
    uint8_t _cfg_sys[ADS_CNT + 2] = { 0 };
    uint8_t rreg_command[ADS_CNT + 2] = {0};
#else
    uint8_t _cfg_sys[3 + 1] = { 0 };
    uint8_t rreg_command[3 + 1] = {0};
#endif
    rreg_command[0] = ADS_COMMAND_RREG_BASE | ADS_REG_CONFIG; // start address at 0x01
    rreg_command[1] = 0x00; // read a reg
#if(ADS_USE_WR_FUNC)
#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)

#else
    ret = ads_drv.write_read(rreg_command, _cfg_sys, 3 + 1);
	if(ret == 0) {
        *cfg_sys = _cfg_sys[2];
	}
#endif
#else
    ret = ads_drv.write(rreg_command, 2);
    if(ret) return ret;
#if(ADS_READ_FUNC == 1)
#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)
    uint8_t i = 0;
    do{
        ret = ads_drv.read(&_cfg_sys[i], 1);
        if(ret) return ret;
        i += 1;
    }while(i < ADS_CNT);
    memcpy(cfg_sys, _cfg_sys, ADS_CNT);
#else
    ret = ads_drv.read(_cfg_sys, 1);
    if(ret == 0)
    	*cfg_sys = _cfg_sys[0];
#endif
#endif
#if(ADS_READ_FUNC == 2)
    ret = ads_drv.read(_cfg_sys, ADS_CNT);
    if(ret) return ret;
    memcpy(cfg_sys, _cfg_sys, ADS_CNT);
#endif

#endif
	return ret;
}


uint8_t ads_cfg_sys_offset_calibration(uint32_t offset_calibration){
    uint8_t ret = 0;
    uint8_t wreg_command[8] = { 0 };

    wreg_command[0] = ADS_COMMAND_WREG_BASE | ADS_REG_OFC0;  // start address at 0x02
    wreg_command[1] = 0x02; // write three regs
    wreg_command[2] = (uint8_t)offset_calibration;
    wreg_command[3] = (uint8_t)(offset_calibration >> 8);
    wreg_command[4] = (uint8_t)(offset_calibration >> 16);

    ret = ads_drv.write(wreg_command, 2);
    if(ret) return ret;
    ads_drv.delay(1);
    /**
     * datasheet rev.2016, section: 8.5.1.8.5, 8.5.1.8.6
     * .写完数据后增加适当数量时钟将数据从移位寄存器移送到配置寄存器
     */
    ret = ads_drv.write(&wreg_command[2], (3 + 1));

	return ret;
}

uint8_t ads_cfg_sys_gain_calibration(uint16_t gain_calibration){
    uint8_t ret = 0;
    uint8_t wreg_command[4 << 1] = {0};

    wreg_command[0] = ADS_COMMAND_WREG_BASE | ADS_REG_FSC0;  // start address at 0x05
    wreg_command[1] = 0x01; // write two regs
    wreg_command[2] = (uint8_t)gain_calibration;
    wreg_command[3] = (uint8_t)(gain_calibration >> 8);

    ret = ads_drv.write(wreg_command, 2);
    if(ret) return ret;
    ads_drv.delay(1);
    /**
     * datasheet rev.2016, section: 8.5.1.8.5, 8.5.1.8.6
     * .写完数据后增加适当数量时钟将数据从移位寄存器移送到配置寄存器
     */
    ret = ads_drv.write(&wreg_command[2], (2 + 1));

	return ret;
}

uint8_t ads_get_mode(uint8_t* mode){
    uint8_t ret = 0;
#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)
    uint8_t rreg_command[2 + ADS_CNT] = {0};
    uint8_t _mode[ADS_CNT + 2] = { 0 };
#else
    uint8_t rreg_command[3 + 1] = {0};
    uint8_t _mode[3 + 1] = { 0 };
#endif
    rreg_command[0] = ADS_COMMAND_RREG_BASE | ADS_REG_MODE; // start address at 0x07
    rreg_command[1] = 0x00; // read a reg
#if(ADS_USE_WR_FUNC)
#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)
#else
    ret = ads_drv.write_read(rreg_command, _mode, 3 + 1);
    if(ret == 0){
        *mode = _mode[2];
    }
#endif
#else
    ret = ads_drv.write(rreg_command, 2);
    if(ret) return ret;
#if(ADS_READ_FUNC == 1)
#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)
    uint8_t i =0;
    do{
        ret = ads_drv.read(&_mode[i], 1);
        if(ret) return ret;
        i += 1;
    }while(i < ADS_CNT);
    memcpy(mode, _mode, ADS_CNT);
#else
    ret = ads_drv.read(_mode, 1);
    if(ret == 0)
    	*mode = _mode[0];
#endif
#endif
#if(ADS_READ_FUNC == 2)
    ret = ads_drv.read(_mode, ADS_CNT);
    if(ret) return ret;
    memcpy(mode, _mode, ADS_CNT);
#endif

#endif
	return ret;
}

uint8_t ads_read_regs(uint8_t start_address, uint8_t end_address, uint8_t *pdata){
    uint8_t ret = 0;
#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)

#else
    uint8_t rreg_command[2] = { 0 };
    uint8_t rreg_value[16] = { 0 };

    rreg_command[0] = ADS_COMMAND_RREG_BASE | start_address; // start address at 0x07
    rreg_command[1] = end_address - start_address - 1;

    ret = ads_drv.write(rreg_command, (uint16_t)sizeof(rreg_command));
    if(ret) return ret;
    ads_drv.delay(1);
    ret = ads_drv.read(rreg_value, (rreg_command[1] + 1) + 1);
    if(ret) return ret;
    memcpy(pdata, rreg_value, (size_t)(rreg_command[1] + 1));
#endif
    return ret;
}


uint8_t ads_write_regs(uint8_t start_address, uint8_t end_address, uint8_t *pdata){
    uint8_t ret = 0;
#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)

#else
    uint8_t wreg_command[4 << 1] = {0};
    uint8_t wreg_value[16] = { 0 };

    wreg_command[0] = ADS_COMMAND_WREG_BASE | start_address;  // start address at 0x05
    wreg_command[1] = end_address - start_address - 1;    // write two regs
    ret = ads_drv.write(wreg_command, 2);
    if(ret) return ret;
    ads_drv.delay(1);
    memcpy(wreg_value, pdata, (size_t)(wreg_command[1] + 1));
    /**
     * datasheet rev.2016, section: 8.5.1.8.5, 8.5.1.8.6
     * .写完数据后增加适当数量时钟将数据从移位寄存器移送到配置寄存器
     */
    ret = ads_drv.write(wreg_value, ((wreg_command[1] + 1) + 1));
#endif
	return ret;
}


uint8_t ads_command_start(void){
    uint8_t ret = 0;
    uint8_t command = 0;

    command = ADS_COMMAND_START;
    ret = ads_drv.write(&command, 1);
    return ret;
}

uint8_t ads_command_stop(void){
    uint8_t ret = 0;
    uint8_t command = 0;

    command = ADS_COMMAND_STOP;
    ret = ads_drv.write(&command, 1);
    return ret;
}

uint8_t ads_command_reset(void){
    uint8_t ret = 0;
    uint8_t command = 0;

    command = ADS_COMMAND_RESET;
    ret = ads_drv.write(&command, 1);
    return ret;
}

uint8_t ads_command_rdata(void){
    uint8_t ret = 0;
    uint8_t command = 0;

    command = ADS_COMMAND_RDATA;
    ret = ads_drv.write(&command, 1);
    return ret;
}

