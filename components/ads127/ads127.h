/*
 * ads127.h
 *
 *  Created on: 2020年11月30日
 *      Author: XIAOSENLUO
 */

#ifndef ADS127_ADS127_H_
#define ADS127_ADS127_H_

#include "stdint.h"
#include "stddef.h"
#include "ads127_config.h"
#include "time.h"

typedef struct _ads_drv{
	uint8_t (*write)(uint8_t*, uint16_t);
	uint8_t (*read)(uint8_t*, uint16_t);
	uint8_t (*write_read)(uint8_t*, uint8_t*, uint16_t);
	void    (*delay)(uint32_t);
}ads_drv_t;


typedef uint8_t (*ads_write_func_t)(uint8_t*, uint16_t);
typedef uint8_t (*ads_read_func_t)(uint8_t*, uint16_t);
typedef uint8_t (*ads_write_read_func_t)(uint8_t*, uint8_t*, uint16_t);

#define ADS_COMMAND_RDATA                             0x12
#define ADS_COMMAND_RESET                             0x06
#define ADS_COMMAND_START                             0x08
#define ADS_COMMAND_STOP                              0x0A

#define ADS_COMMAND_RREG_BASE                         0x20
#define ADS_COMMAND_WREG_BASE                         0x40

#define ADS_REG_ID                                    0x00
#define ADS_REG_CONFIG                                0x01
#define ADS_REG_OFC0                                  0x02
#define ADS_REG_OFC1                                  0x03
#define ADS_REG_OFC2                                  0x04
#define ADS_REG_FSC0                                  0x05
#define ADS_REG_FSC1                                  0x06
#define ADS_REG_MODE                                  0x07


#define ADS_CFG_FSC_BIT                               0x40
#define ADS_CFG_OFC_BIT                               0x20
#define ADS_CFG_TOUTDEL_BIT                           0x08
#define ADS_CFG_SPITOUT_BIT                           0x04
#define ADS_CFG_CSENB_BIT                             0x02
#define ADS_CFG_CRCB_BIT                              0x01

#define ADS_CRC_4                                     0x03
#define ADS_CRC_8                                     0x07

#define ADS_DATA_READY                        (0x01)
#define ADS_DATA_BUF_FULL                     (0x10)
#define ADS_USER_READING                      (0x04)
#define ADS_TX_COMMAND                        (0x02)
#define ADS_RX_DATA                           (0x08)
#define ADS_READING                           (0x20)
#define ADS_READ_COMPLETE                     (0x40)


typedef struct _ads_data_typedef{
	uint8_t *data;
	uint32_t data_size;
	uint32_t timestamp;
	uint32_t index;
}ads_data_t;

typedef void (*ads_drdy_callback_t)(void);

static __inline int32_t complement_to_decimal(uint32_t com, uint8_t bits){
//	if(com & (0x01 << (n - 1))){    // 负数
//		return (signed int)(com | (~((1 << n) - 1)));
//	}else{
//		return (int32_t)(com);
//	}
	return ((com & (0x00000001 << (bits - 1))) ? (int32_t)(com | (~((1 << bits) - 1))) : (int32_t)(com));
}

static __inline uint32_t decimal_to_complement(int32_t dec, uint8_t bits){
    return (((dec) & (0x00000001 << (bits - 1))) ? (uint32_t)((((0x01 << bits) - 1) & (dec)) | (0x01 << (bits - 1))) : (uint32_t)(dec));
}

#define COM_TO_DEC(complements, n)  ((complements & (0x00000001 << (n - 1))) ? (int32_t)(complements | (~((1 << n) - 1))) : (complements))




void ads_drv_init(ads_write_func_t write_func, ads_read_func_t read_func, ads_write_read_func_t wr_func);

/**
 *     设置 ADS 数量
 */
void ads_set_nums(uint8_t in_ads_num);


/**
 * return: 0-success
 */
uint8_t ads_read_data_in_byte(uint8_t* data_ptr, uint8_t in_byte_num);
uint8_t ads_read_data_in_complements(uint32_t* data_ptr, uint8_t* out_status, uint8_t in_ads_num);
uint8_t ads_read_data_in_value(int32_t* data_ptr, uint8_t in_ads_num);


//uint8_t ads_data_buffer_init(uint8_t *in_ptr1, uint8_t *in_ptr2, uint32_t in_buffer_size);
//uint32_t ads_read_data_to_buffer(void);

uint8_t ads_get_device_characteristics(uint8_t* characteristics);
uint8_t ads_cfg_sys(uint8_t cfg_sys);
uint8_t ads_get_cfg_sys(uint8_t* cfg_sys);
uint8_t ads_cfg_sys_offset_calibration(uint32_t offset_calibration);
uint8_t ads_cfg_sys_gain_calibration(uint16_t gain_calibration);
uint8_t ads_get_mode(uint8_t* mode);
uint8_t ads_read_regs(uint8_t start_address, uint8_t end_address, uint8_t *pdata);
uint8_t ads_write_regs(uint8_t start_address, uint8_t end_address, uint8_t *pdata);

uint8_t ads_command_rdata(void);
uint8_t ads_command_start(void);
uint8_t ads_command_stop(void);
uint8_t ads_command_reset(void);

/***
 * tx_ptr: tx buffer, if it is NULL, then will use default tx buffer
 * rx_ptr: rx buffer, if it is NULL, then will use default rx buffer
 * in_size: buffer size, if it is 0, then will use default size.
 */
uint8_t ads_read_data_by_dma_init(uint8_t *tx_ptr, uint8_t *rx_ptr, uint16_t in_size);
void ads_register_drdy_callback(void *callback_function);

/**
 * in_size: number of ads127
 *
 * out_cal_value: 补码
 *
 */
uint8_t ads_bsp_calibrate(uint32_t *out_cal_value, uint8_t in_size);

uint8_t ads_get_read_status(void);
void ads_read_data_complete(void);
void ads_bsp_selete_cs_by_index(uint8_t in_index, uint8_t in_value);
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


#endif /* ADS127_ADS127_H_ */
