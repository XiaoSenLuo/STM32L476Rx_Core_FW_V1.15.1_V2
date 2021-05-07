/*
 * st_crc.c
 *
 *  Created on: 2021年1月31日
 *      Author: XIAOSENLUO
 */

#include "st_crc.h"

#include "stm32l4xx_ll_crc.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"


void st_crc_init(uint32_t in_pol, uint8_t in_crc_bit, uint32_t in_init_value){
	uint32_t _crc_bit = 0;
	switch(in_crc_bit){
	case 7:
		_crc_bit = LL_CRC_POLYLENGTH_7B;
		break;
	case 8:
		_crc_bit = LL_CRC_POLYLENGTH_8B;
	    break;
	case 16:
		_crc_bit = LL_CRC_POLYLENGTH_16B;
		break;
	case 32:
		_crc_bit = LL_CRC_POLYLENGTH_32B;
		break;
	}
	  /* (1) Enable peripheral clock for CRC                   *********************/
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);

	  /* (2) Configure CRC functional parameters  ********************************/

	  /* Configure CRC calculation unit with user defined polynomial value, 8-bit long */
	LL_CRC_SetPolynomialCoef(CRC, in_pol);
	LL_CRC_SetPolynomialSize(CRC, _crc_bit);

	  /* Initialize default CRC initial value */
	  /* Reset value is LL_CRC_DEFAULT_CRC_INITVALUE */
	   LL_CRC_SetInitialData(CRC, in_init_value);

	  /* Set input data inversion mode : No inversion*/
	  /* Reset value is LL_CRC_INDATA_REVERSE_NONE */
	  // LL_CRC_SetInputDataReverseMode(CRC, LL_CRC_INDATA_REVERSE_NONE);

	  /* Set output data inversion mode : No inversion */
	  /* Reset value is LL_CRC_OUTDATA_REVERSE_NONE */
	  // LL_CRC_SetOutputDataReverseMode(CRC, LL_CRC_OUTDATA_REVERSE_NONE);
}


uint32_t st_crc_calculate_msb(uint8_t *in_ptr, uint32_t in_size){
	register uint32_t _data = 0;
	register uint32_t _index = 0;

//	LL_CRC_ResetCRCCalculationUnit(CRC);
	/* Compute the CRC of Data Buffer array*/
	for(_index = 0; _index < (in_size / 4); _index++){
	   _data = (uint32_t)((in_ptr[4 * _index] << 24) | (in_ptr[4 * _index + 1] << 16) | (in_ptr[4 * _index + 2] << 8) | in_ptr[4 * _index + 3]);
	   LL_CRC_FeedData32(CRC, _data);
	}
	/* Last bytes specific handling */
	if((in_size % 4) != 0){
	    if(in_size % 4 == 1){
	        LL_CRC_FeedData8(CRC, in_ptr[4 * _index]);
	    }
	    if(in_size % 4 == 2){
	        LL_CRC_FeedData16(CRC, (uint16_t)((in_ptr[4 * _index] << 8) | in_ptr[4 * _index + 1]));
	    }
	    if(in_size % 4 == 3){
	        LL_CRC_FeedData16(CRC, (uint16_t)((in_ptr[4 * _index] << 8) | in_ptr[4 * _index + 1]));
	        LL_CRC_FeedData8(CRC, in_ptr[4 * _index + 2]);
	    }
	}
	/* Return computed CRC value */
	return(LL_CRC_ReadData32(CRC));
}

uint32_t st_crc_calculate_lsb(uint8_t *in_ptr, uint32_t in_size){
	register uint32_t _data = 0;
	register uint32_t _index = 0;

	LL_CRC_ResetCRCCalculationUnit(CRC);
	/* Compute the CRC of Data Buffer array*/
	for(_index = 0; _index < (in_size / 4); _index++){
	   _data = (uint32_t)((in_ptr[4 * _index + 3] << 24) | (in_ptr[4 * _index + 2] << 16) | (in_ptr[4 * _index + 1] << 8) | in_ptr[4 * _index]);
	   LL_CRC_FeedData32(CRC, _data);
	}
	/* Last bytes specific handling */
	if((in_size % 4) != 0){
	    if(in_size % 4 == 1){
	        LL_CRC_FeedData8(CRC, in_ptr[4 * _index]);
	    }
	    if(in_size % 4 == 2){
	        LL_CRC_FeedData16(CRC, (uint16_t)((in_ptr[4 * _index + 1] << 8) | in_ptr[4 * _index]));
	    }
	    if(in_size % 4 == 3){
	    	LL_CRC_FeedData8(CRC, in_ptr[4 * _index + 2]);
	        LL_CRC_FeedData16(CRC, (uint16_t)((in_ptr[4 * _index + 1] << 8) | in_ptr[4 * _index]));

	    }
	}
	/* Return computed CRC value */
	return(LL_CRC_ReadData32(CRC));
}
