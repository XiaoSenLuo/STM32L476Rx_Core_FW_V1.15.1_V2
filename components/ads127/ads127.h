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
#include "time.h"


typedef union{
    struct{
        uint8_t fsmode : 1;
        uint8_t format : 1;
        uint8_t filter : 2;
        uint8_t osr : 2;
        uint8_t hr : 1;
        uint8_t bit7 : 1;
    };
    uint8_t val;
}ads127_dev_mode_u;

typedef ads127_dev_mode_u * ads127_dev_mode_handle;

typedef union{
    struct{
        uint32_t bin2com : 24;
        uint32_t status : 8;
    };
    uint32_t val;
}ads127_data_u;

typedef ads127_data_u * ads127_data_handle;

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
    union{
        struct{
            uint32_t ofc0 : 8;
            uint32_t ofc1 : 8;
            uint32_t ofc2 : 8;
        };
        uint32_t val;
    }ofc;
    union{
        struct{
            uint16_t fsc0 : 8;
            uint16_t fsc1 : 8;
        };
        uint16_t val;
    }fsc;
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

#define ADS127_DEFAULT_DEVICE()         { \
                                            .id.val = 0x03, \
                                            .config.val = 0x00, \
                                            .ofc.val = 0, \
                                            .fsc.val = 0, \
                                            .mode.val = 0x74, \
                                        }


#define ADS127_COMMAND_RDATA                             0x12
#define ADS127_COMMAND_RESET                             0x06
#define ADS127_COMMAND_START                             0x08
#define ADS127_COMMAND_STOP                              0x0A

#define ADS127_COMMAND_RREG_BASE                         0x20
#define ADS127_COMMAND_WREG_BASE                         0x40

#define ADS127_REG_ID                                    0x00
#define ADS127_REG_CONFIG                                0x01
#define ADS127_REG_OFC0                                  0x02
#define ADS127_REG_OFC1                                  0x03
#define ADS127_REG_OFC2                                  0x04
#define ADS127_REG_FSC0                                  0x05
#define ADS127_REG_FSC1                                  0x06
#define ADS127_REG_MODE                                  0x07




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


void ads127_command_reset(void);
void ads127_command_start(void);
void ads127_command_stop(void);
uint32_t ads127_command_rdata(void);

void ads127_configure(ads127_dev_t *device);
uint8_t ads127_get_configure(ads127_dev_t *device);

void ads127_configure_ofc(ads127_dev_t *device);
uint32_t ads127_get_ofc(ads127_dev_t *device);

void ads127_configure_fsc(ads127_dev_t *device);
uint16_t ads127_get_fsc(ads127_dev_t *device);

uint8_t ads127_get_id(ads127_dev_t *device);
uint8_t ads127_get_mode(ads127_dev_t *device);

static const int16_t ads_osr[4][4] = {
        {32, 64, 128, 256},
        {32, 64, 128, 256},
        {32, 128, 512, 2048},
        {-1, -1, -1, -1}
};

static inline int16_t ads127_get_osr(const ads127_dev_t *device){
    return ads_osr[device->mode.filter][device->mode.osr];
}

static inline uint8_t ads127_get_data_length(const ads127_dev_t *device){
    return (4 - device->config.cs_enb);
}


#endif /* ADS127_ADS127_H_ */
