/*
 * st_crc.h
 *
 *  Created on: 2021年1月31日
 *      Author: XIAOSENLUO
 */

#ifndef ST_CRC_ST_CRC_H_
#define ST_CRC_ST_CRC_H_

#include "stdint.h"

void st_crc_init(uint32_t in_pol, uint8_t in_crc_bit, uint32_t in_init_value);

uint32_t st_crc_calculate_msb(uint8_t *in_ptr, uint32_t in_size);
uint32_t st_crc_calculate_lsb(uint8_t *in_ptr, uint32_t in_size);
#endif /* ST_CRC_ST_CRC_H_ */
