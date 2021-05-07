/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/


#include "i2c.h"

/* USER CODE BEGIN 0 */
#include "main.h"
/* USER CODE END 0 */


/* USER CODE BEGIN 1 */

#ifndef I2C_TIMING
#define I2C_TIMING           0x00707CBB
#endif

static uint8_t _i2c2_tx_dma_config(uint8_t *in_ptr, uint16_t in_size);
static uint8_t _i2c2_rx_dma_config(uint8_t *in_ptr, uint16_t in_size);

static void i2c2_dma_rxcplt_callback(void);
static void i2c2_dma_txcplt_callback(void);

//static __USED uint32_t i2c_time[] = {0x10403EFC/* 50K */, 0x00807CB9/* 100K */, 0x00300FBC/* 150K */,
//		0x00301184/* 200K */, 0x00300F66/* 250K */,
//		0x00302A36/* 300K */, 0x00301041/* 350K */,
//		0x00301035/* 400K */, 0x00300B28/* 1000K*/,
//};

void i2c_init(I2C_TypeDef* in_i2c){

	LL_I2C_InitTypeDef I2C_InitStruct = {0};

	if((uint32_t)I2C2 == (uint32_t)in_i2c){
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
		/* Configure SCL Pin as : Alternate function, High Speed, Open drain, Pull up */
		LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);
		LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_13, LL_GPIO_AF_4);
		LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_VERY_HIGH);
		LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_OPENDRAIN);
		LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_13, LL_GPIO_PULL_NO);

		/* Configure SDA Pin as : Alternate function, High Speed, Open drain, Pull up */
		LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_ALTERNATE);
		LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_14, LL_GPIO_AF_4);
		LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_14, LL_GPIO_SPEED_FREQ_VERY_HIGH);
		LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_14, LL_GPIO_OUTPUT_OPENDRAIN);
		LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_14, LL_GPIO_PULL_NO);

		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);
		LL_RCC_SetI2CClockSource(LL_RCC_I2C2_CLKSOURCE_PCLK1);
		LL_I2C_Disable(I2C2);
//		LL_I2C_SetTiming(I2C2, I2C_TIMING);     //

//		LL_I2C_EnableAutoEndMode(I2C2);
//		LL_I2C_DisableOwnAddress2(I2C2);
		LL_I2C_EnableGeneralCall(I2C2);
//		LL_I2C_EnableClockStretching(I2C2);
		I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
		I2C_InitStruct.Timing = 0x00301035;
		I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
		I2C_InitStruct.DigitalFilter = 0;
		I2C_InitStruct.OwnAddress1 = 0;
		I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
		I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
		LL_I2C_Init(I2C2, &I2C_InitStruct);
		LL_I2C_SetOwnAddress2(I2C2, 0, LL_I2C_OWNADDRESS2_NOMASK);

		HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, DMA1_Channel4_Priority, 0);
		HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, DMA1_Channel5_Priority, 0);
	}
}

void i2c_deinit(I2C_TypeDef* in_i2c){
	if((uint32_t)in_i2c == (uint32_t)I2C2){
		LL_I2C_DeInit(I2C2);
		LL_I2C_Disable(I2C2);
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_I2C2);
		LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_ANALOG);
		LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_13, LL_GPIO_PULL_NO);
		LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_ANALOG);
		LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_14, LL_GPIO_PULL_NO);
	}
}

static uint8_t _i2c2_tx_dma_config(uint8_t *in_ptr, uint16_t in_size){   // DMA1 Channel 4, request 3
	uint8_t ret = 0;
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	/* (1) Enable the clock of DMA1 and DMA1 */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	/* (2) Configure NVIC for DMA transfer complete/error interrupts */

	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	st_irq_handler_register(DMA1_Channel4_IRQn, i2c2_dma_txcplt_callback);

	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
	LL_DMA_ClearFlag_GI4(DMA1);
	/* (3) Configure the DMA1_Channel2 functional parameters */
	LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_4,
			            LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
						LL_DMA_PRIORITY_HIGH |
						LL_DMA_MODE_NORMAL |
						LL_DMA_PERIPH_NOINCREMENT |
						LL_DMA_MEMORY_INCREMENT |
						LL_DMA_PDATAALIGN_BYTE |
						LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4,
			(uint32_t)in_ptr,
			LL_I2C_DMA_GetRegAddr(I2C2, LL_I2C_DMA_REG_DATA_TRANSMIT),
			LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, in_size);
	if(LL_DMA_GetPeriphRequest(DMA1, LL_DMA_CHANNEL_4) != LL_DMA_REQUEST_3)
		LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_4, LL_DMA_REQUEST_3);

	/* (5) Enable DMA interrupts complete/error */
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_4);

	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
	return ret;
}

static uint8_t _i2c2_rx_dma_config(uint8_t *in_ptr, uint16_t in_size){   // DMA1 Channel 5, request 3
	uint8_t ret = 0;
	/* (1) Enable the clock of DMA1 and DMA1 */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	/* (2) Configure NVIC for DMA transfer complete/error interrupts */

	NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	st_irq_handler_register(DMA1_Channel5_IRQn, i2c2_dma_rxcplt_callback);

	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
	LL_DMA_ClearFlag_GI5(DMA1);
	/* (3) Configure the DMA1_Channel2 functional parameters */
	LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_5,
			            LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
						LL_DMA_PRIORITY_HIGH |
						LL_DMA_MODE_NORMAL |
						LL_DMA_PERIPH_NOINCREMENT |
						LL_DMA_MEMORY_INCREMENT |
						LL_DMA_PDATAALIGN_BYTE |
						LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_5,
			LL_I2C_DMA_GetRegAddr(I2C2, LL_I2C_DMA_REG_DATA_RECEIVE),  // 源地址
			(uint32_t)in_ptr,                // 目的地址
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, in_size);
	if(LL_DMA_GetPeriphRequest(DMA1, LL_DMA_CHANNEL_5) != LL_DMA_REQUEST_3)
		LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_5, LL_DMA_REQUEST_3);

	/* (5) Enable DMA interrupts complete/error */
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_5);

	LL_DMA_EnableChannel(DMA1,  LL_DMA_CHANNEL_5);

	return ret;
}

static volatile uint8_t i2c_dma_status = 0;

static uint8_t _i2c_request_memory_write(I2C_TypeDef* in_i2c, uint16_t _address, uint16_t _reg, uint8_t _reg_bit){
    uint8_t ret = 0;
    uint8_t _reg_size = 0;
    uint32_t _timeout = 32000000;

    if(_reg_bit == 16) _reg_size = 2;
    else _reg_size = 1;     // 8 bits

	LL_I2C_HandleTransfer(in_i2c, _address,
			LL_I2C_ADDRSLAVE_7BIT,
			_reg_size,
			LL_I2C_MODE_RELOAD, LL_I2C_GENERATE_START_WRITE);

	while(LL_I2C_IsActiveFlag_TXIS(in_i2c) == 0){
		if(LL_I2C_IsActiveFlag_NACK(in_i2c)){
			uint32_t __timeout = 32000000;
			while(LL_I2C_IsActiveFlag_STOP(in_i2c) == 0){
				__timeout -= 1;
				if(__timeout == 0) return 1;
			}
			LL_I2C_ClearFlag_NACK(in_i2c);
			LL_I2C_ClearFlag_STOP(in_i2c);

			if(LL_I2C_IsActiveFlag_TXIS(in_i2c) != 0){
				LL_I2C_TransmitData8(in_i2c, 0x00);
			}
			if(LL_I2C_IsActiveFlag_TXE(in_i2c) == 0){
				LL_I2C_ClearFlag_TXE(in_i2c);
			}
			// TODO  I2C_RESET_CR2(hi2c);
		}
		_timeout -= 1;
		if(_timeout == 0){
            return 1;
		}
	}
    if(_reg_bit == 8){   // 8 bit
    	LL_I2C_TransmitData8(in_i2c, (uint8_t)(_reg & 0x000000FF));
    }else{   // 16 bit

    }
    _timeout = 32000000;
    while(LL_I2C_IsActiveFlag_TCR(in_i2c) == 0){
        _timeout -= 1;
        if(_timeout == 0) return 1;
    }

    return ret;
}

static uint8_t _i2c_request_memory_read(I2C_TypeDef* in_i2c, uint16_t _address, uint16_t _reg, uint8_t _reg_bit){
    uint8_t ret = 0;
    uint8_t _reg_size = 0;
    uint32_t _timeout = 32000000;

    if(_reg_bit == 16) _reg_size = 2;
    else _reg_size = 1;     // 8 bits

	LL_I2C_HandleTransfer(in_i2c, _address,
			LL_I2C_ADDRSLAVE_7BIT,
			_reg_size,
			LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);

	while(LL_I2C_IsActiveFlag_TXIS(in_i2c) == 0){
		if(LL_I2C_IsActiveFlag_NACK(in_i2c)){
			uint32_t __timeout = 32000000;
			while(LL_I2C_IsActiveFlag_STOP(in_i2c) == 0){
				__timeout -= 1;
				if(__timeout == 0) return 1;
			}
			LL_I2C_ClearFlag_NACK(in_i2c);
			LL_I2C_ClearFlag_STOP(in_i2c);

			if(LL_I2C_IsActiveFlag_TXIS(in_i2c) != 0){
				LL_I2C_TransmitData8(in_i2c, 0x00);
			}
			if(LL_I2C_IsActiveFlag_TXE(in_i2c) == 0){
				LL_I2C_ClearFlag_TXE(in_i2c);
			}
			// TODO  I2C_RESET_CR2(hi2c);
		}
		_timeout -= 1;
		if(_timeout == 0){
            return 1;
		}
	}
    if(_reg_bit == 8){   // 8 bit
    	LL_I2C_TransmitData8(in_i2c, (uint8_t)(_reg & 0x000000FF));
    }else{   // 16 bit

    }
    _timeout = 32000000;
    while(LL_I2C_IsActiveFlag_TC(in_i2c) == 0){
        _timeout -= 1;
        if(_timeout == 0) return 1;
    }

    return ret;
}

static uint8_t _i2c_write(I2C_TypeDef* in_i2c, uint8_t _address, uint16_t _reg, uint8_t *_ptr, uint16_t _s){
    uint8_t ret = 0;

    if(LL_I2C_IsEnabled(in_i2c) == 0) LL_I2C_Enable(in_i2c);

    ret = _i2c_request_memory_write(in_i2c, _address, _reg, 8);
    if(ret != 0) return ret;

	i2c_dma_status |= 0x01;

	if((uint32_t)in_i2c == (uint32_t)I2C2) ret = _i2c2_tx_dma_config(_ptr, _s);

	LL_I2C_HandleTransfer(in_i2c, _address,
			LL_I2C_ADDRSLAVE_7BIT,
			_s,
			LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_NOSTARTSTOP);

	LL_I2C_EnableDMAReq_TX(in_i2c);
    while(i2c_dma_status & 0x01);
    while(LL_I2C_IsActiveFlag_STOP(in_i2c) == 0);
    LL_I2C_ClearFlag_STOP(in_i2c);
    LL_I2C_ClearFlag_NACK(in_i2c);
    LL_I2C_DisableDMAReq_TX(in_i2c);
    LL_I2C_Disable(in_i2c);
	return ret;
}

static uint8_t _i2c_read(I2C_TypeDef* in_i2c, uint8_t _address, uint16_t _reg, uint8_t *_ptr, uint16_t _s){
    uint8_t ret = 0;

    if(LL_I2C_IsEnabled(in_i2c) == 0) LL_I2C_Enable(in_i2c);
    ret = _i2c_request_memory_read(in_i2c, _address, _reg, 8);
    if(ret != 0) return ret;

	i2c_dma_status |= 0x10;

	if((uint32_t)in_i2c == (uint32_t)I2C2) ret = _i2c2_rx_dma_config(_ptr, _s);

	LL_I2C_HandleTransfer(in_i2c, _address,
			LL_I2C_ADDRSLAVE_7BIT,
			_s,
			LL_I2C_MODE_AUTOEND, I2C_GENERATE_START_READ);

	LL_I2C_EnableDMAReq_RX(in_i2c);
    while(i2c_dma_status & 0x10);
    while(LL_I2C_IsActiveFlag_STOP(in_i2c) == 0);
    LL_I2C_ClearFlag_STOP(in_i2c);
    LL_I2C_ClearFlag_NACK(in_i2c);
    LL_I2C_DisableDMAReq_RX(in_i2c);
    LL_I2C_Disable(in_i2c);
	return ret;
}

uint8_t I2C2_Write(uint8_t in_address, uint8_t in_reg, uint8_t* in_ptr, uint16_t in_size){
    uint8_t ret = 0;

    ret = _i2c_write(I2C2, in_address, in_reg, in_ptr, in_size);

    return ret;
}


uint8_t I2C2_Read(uint8_t in_address, uint8_t in_reg, uint8_t* out_ptr, uint16_t in_size){
	uint8_t ret = 0;

	ret = _i2c_read(I2C2, in_address, in_reg, out_ptr, in_size);

	return ret;
}

#ifndef I2C_TIMEOUT
#define I2C_TIMEOUT                           4000000
#endif

uint8_t I2C_Find_Address(I2C_TypeDef* in_i2c, uint8_t *out_address, uint8_t *out_device_num){
	uint8_t ret = 0;
    uint16_t i = 0;
    uint8_t _i2c_addr = 0;
    uint32_t tmp1 = 0, tmp2 = 0, timeout = I2C_TIMEOUT;
    uint8_t _num = 0;

	if(LL_I2C_IsEnabled(in_i2c) == 0) LL_I2C_Enable(in_i2c);
    while(i < 255){
    	i++;
    	_i2c_addr++;
    	ret = 0;
		LL_I2C_HandleTransfer(in_i2c, _i2c_addr,
			  LL_I2C_ADDRSLAVE_7BIT,
			  0,
			  LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
		tmp1 = LL_I2C_IsActiveFlag_STOP(in_i2c);
		tmp2 = LL_I2C_IsActiveFlag_NACK(in_i2c);
		timeout = I2C_TIMEOUT;
		while((tmp1 == 0) && (tmp2 == 0)){
			timeout -= 1;
			tmp1 = LL_I2C_IsActiveFlag_STOP(in_i2c);
			tmp2 = LL_I2C_IsActiveFlag_NACK(in_i2c);
			if(timeout == 0){
				ret = 1;
				break;
				//            	  return 1;
			}
		}
		if(ret){// 超时, 下一个地址
	    goto_next_address:
//			LL_I2C_HandleTransfer(in_i2c, _i2c_addr,
//				  LL_I2C_ADDRSLAVE_7BIT,
//				  0,
//				  LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_STOP);
//			tmp1 = LL_I2C_IsActiveFlag_STOP(in_i2c);
//			timeout = 8000000;
//			while(tmp1 == 0){
//				timeout -= 1;
//				tmp1 = LL_I2C_IsActiveFlag_STOP(in_i2c);
//				if(timeout == 0){
//				  ret = 1;
//				  break;
//				}
//			}
			LL_I2C_ClearFlag_NACK(in_i2c);
            LL_I2C_ClearFlag_STOP(in_i2c);
			continue;
		}
		if((LL_I2C_IsActiveFlag_NACK(in_i2c) == 0)){   // 地址正确. 从机返回ACK
			timeout = I2C_TIMEOUT;
			tmp1 = LL_I2C_IsActiveFlag_STOP(in_i2c);
			while(tmp1 == 0){
				timeout -= 1;
				tmp1 = LL_I2C_IsActiveFlag_STOP(in_i2c);
				if(timeout == 0){
				  ret = 1;
				  break;
				}
			}
            if(ret){
            	goto goto_next_address;
            }
            LL_I2C_ClearFlag_NACK(in_i2c);
            LL_I2C_ClearFlag_STOP(in_i2c);
            if(out_address != NULL) out_address[_num] = _i2c_addr;
            _num += 1;
            _i2c_addr++; // 跳过下一个地址, 因为下一个地址还是一样, 只是控制读写位由写请求变成读请求
            i++;
		}else{ // 地址错误, 从机返回 NACK
			timeout = I2C_TIMEOUT;
			tmp1 = LL_I2C_IsActiveFlag_STOP(in_i2c);
			while(tmp1 == 0){
				timeout -= 1;
				tmp1 = LL_I2C_IsActiveFlag_STOP(in_i2c);
				if(timeout == 0){
				  ret = 1;
				  break;
				//            		  return 1;
				}
			}
            if(ret){
            	goto goto_next_address;
            }
            LL_I2C_ClearFlag_STOP(in_i2c);
            LL_I2C_ClearFlag_NACK(in_i2c);
		}
    }
    if(out_device_num != NULL) *out_device_num = _num;
    return ret;
}


void i2c2_dma_txcplt_callback(void){
	if(LL_DMA_IsActiveFlag_TC4(DMA1)){
	  LL_DMA_ClearFlag_GI4(DMA1);
	  i2c_dma_status &= (~0x01);
	}
	if(LL_DMA_IsActiveFlag_TE4(DMA1)){
	  LL_DMA_ClearFlag_TE4(DMA1);
	  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
	}
}


void i2c2_dma_rxcplt_callback(void){
	if(LL_DMA_IsActiveFlag_TC5(DMA1)){
	  LL_DMA_ClearFlag_GI5(DMA1);
	  i2c_dma_status &= (~0x10);
	}
	if(LL_DMA_IsActiveFlag_TE5(DMA1)){
	  LL_DMA_ClearFlag_TE5(DMA1);
	  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
	}
}



/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
