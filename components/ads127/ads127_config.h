/*
 * ads127_config.h
 *
 *  Created on: 2021年3月3日
 *      Author: XIAOSENLUO
 */

#ifndef ADS127_ADS127_CONFIG_H_
#define ADS127_ADS127_CONFIG_H_

#define ADS_CASCADED_MODE                             (0)
#define ADS_DAISYCHAIN_MODE                           (1)
#define ADS_CONNECT_MODE                              ADS_CASCADED_MODE

#define ADS_REF_VOLTAGE                               2500 // Unit: mV
#define ADS_FS                                        (8388608)  // (1 << 23)
#define ADS_CNT                                       (2)

#ifndef ADS_STATUS_WORD
#define ADS_STATUS_WORD                               0                 // 状态字输出控制, 1: Disable, 0:Enable
#define ADS_STATUS_WORD_EN                            (!ADS_STATUS_WORD)
#endif

#define ADS_DATA_BUFFER_SIZE                          (32000)

#ifndef ADS_USE_WR_FUNC
#define ADS_USE_WR_FUNC                               1
#endif

#ifndef ADS_RXDMA_MODE_CIRCULAR
#define ADS_RXDMA_MODE_CIRCULAR                       0
#endif
#ifndef ADS_TXDMA_MODE_CIRCULAR
#define ADS_TXDMA_MODE_CIRCULAR                       0
#endif

#ifndef ADS_READ_DATA_IN_COMMAND
#define ADS_READ_DATA_IN_COMMAND                      0
#endif

#define ADS_DEFAULT_CONFIG                            (0x31 | (ADS_STATUS_WORD << 1)) // 使能SPI超时, 使能增益, 偏移设置, 状态字设置为CRC-8

typedef struct{
	uint32_t conver_rate;
	uint16_t osr;
}ads_config_t;

#endif /* ADS127_ADS127_CONFIG_H_ */
