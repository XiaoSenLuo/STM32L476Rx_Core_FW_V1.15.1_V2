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

typedef struct ads127_dev_s{
    union{
    	struct{
    		uint8_t dev_id : 4;
    		uint8_t rev_id : 4;
    	};
    	uint8_t val;
    }id;
    union{
    	struct{
    		uint8_t crcb : 1;
    		uint8_t cs_enb : 1;
    		uint8_t spi_out : 1;
    		uint8_t tout_del : 1;
    		uint8_t ofc : 1;
    		uint8_t fsc : 1;
    		uint8_t bit6 : 1;
    		uint8_t bit7 : 1;
    	};
    	uint8_t val;
    }config;
    uint8_t ofc[3];
    uint8_t fsc[2];
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
}ads127_dev_t;

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

void ads_set_conver_rate(uint32_t rate, uint16_t osr);


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



#endif /* ADS127_ADS127_H_ */
