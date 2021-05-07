/*
 * bsp_ads127.c
 *
 *  Created on: 2020年12月4日
 *      Author: XIAOSENLUO
 */


#include "main.h"

#include "ads127.h"

#define ADS_DATA_INDEX_SIZE                   (8)

#if(ADS_CONNECT_MODE == ADS_CASCADED_MODE)
#if(defined(ADS_STATUS_WORD))
#define ADS_DMA_TS                            ((ADS_STATUS_WORD_EN + 3))
#endif
#endif

#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)
#if(defined(ADS_STATUS_WORD))
#define ADS_DMA_TS                            ((ADS_CNT * (ADS_STATUS_WORD_EN + 3)))
#endif
#endif


#ifndef ADS_TX_RX_IN_ONE
#define ADS_TX_RX_IN_ONE                      (0x01)
#endif

static uint8_t ads_rdata_comm[(ADS_CNT << 2) << 2] __attribute__((section(".ram2"))) = { 0 };     // 发送临时缓冲区
static uint8_t ads_rdata_buffer[(ADS_CNT << 2) << 2] __attribute__((section(".ram2"))) = { 0 };   // 接收临时缓冲区
static uint8_t ads_data_buffer[ADS_DATA_BUFFER_SIZE + ADS_DATA_INDEX_SIZE] __attribute__((section(".ram2"))) = { 0 };    // RAM2内存区域做缓冲区
static uint8_t ads_data_buffer2[ADS_DATA_BUFFER_SIZE + ADS_DATA_INDEX_SIZE] __attribute__((section(".data"))) = { 0 };    // 缓冲区 2
static volatile uint32_t ads_data_length __attribute__((section(".ram2"))) = 0;
static volatile uint8_t ads_index __attribute__((section(".ram2"))) = 0;

static uint8_t *data_ptr __attribute__((section(".ram2"))) = NULL;
static uint8_t *user_ptr __attribute__((section(".ram2"))) = NULL;
static volatile uint8_t ads_only_rx_fild __attribute__((section(".ram2"))) = 0;
static volatile uint8_t ads_data_ready __attribute__((section(".ram2"))) = 0;
static volatile uint32_t ads_data_index __attribute__((section(".ram2"))) = 8;
static volatile uint8_t ads_user_fild __attribute__((section(".ram2"))) = 0;
static volatile uint8_t ads_read_status __attribute__((section(".ram2"))) = 0;

static ads_data_t ads_data_buf = {0};
static ads_data_t ads_user_buf = {0};

static volatile struct tm ads_data_timestamp = { 0 };
extern ads_drv_t ads_drv;

extern uint32_t ads_conver_rate;
extern uint32_t ads_osr;

static volatile uint8_t ads_conver_control __attribute__((section(".ram2"))) = 0;
static volatile uint32_t ads_drdy_count __attribute__((section(".ram2"))) = 0;

#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)
static void ads_send_rdata_command_cplt_callback(void);
static uint8_t ads_send_rdata_command(void);
#endif
static uint8_t _waitfor_spi1_alldone(void);
static uint8_t ads_start_receive_data(uint8_t *in_rx_ptr, uint16_t in_size);
static void ads_read_data_cplt_callback(void);
static void ads_data_ready_callback(void);


static void _ads_delay(uint32_t _delay){
	HAL_Delay(_delay);
}

void ads_drv_init(ads_write_func_t write_func, ads_read_func_t read_func, ads_write_read_func_t wr_func){

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
    ads_drv.delay = _ads_delay;
}

static uint8_t _waitfor_spi1_txcplt(void){
	uint32_t _timeout = 4000000;
	uint32_t _flag = 0;
    _flag = LL_SPI_GetTxFIFOLevel(SPI1);
    while(_flag && (_timeout--)){
    	_flag = READ_BIT(SPI1->SR, SPI_SR_FTLVL);
    }
    _flag = LL_SPI_IsActiveFlag_BSY(SPI1);
    while(_flag && (_timeout--)){
    	_flag = READ_BIT(SPI1->SR, SPI_SR_BSY);
    }
    if(_timeout) return 0;
    return 1;
}

static uint8_t _waitfor_spi1_rxcplt(void){
	uint32_t _timeout = 4000000;
	uint32_t _flag = 0;

    _flag = LL_SPI_IsActiveFlag_BSY(SPI1);
    while(_flag && (_timeout--)){
    	_flag = READ_BIT(SPI1->SR, SPI_SR_BSY);
    }
    _flag = LL_SPI_GetRxFIFOLevel(SPI1);
    while(_flag && (_timeout--)){
//    	READ_REG(*((__IO uint8_t *)&SPI1->DR));
    	_flag = READ_BIT(SPI1->SR, SPI_SR_FRLVL);
    }

    if(_timeout) return 0;
    return 1;
}

static uint8_t _waitfor_spi1_alldone(void){
	uint32_t _timeout = 4000;
	uint32_t _flag = 0;

    _flag = LL_SPI_GetTxFIFOLevel(SPI1);
    while(_flag && (_timeout--)){
    	_flag = READ_BIT(SPI1->SR, SPI_SR_FTLVL);
    }

    _flag = LL_SPI_IsActiveFlag_BSY(SPI1);
    while(_flag && (_timeout--)){
    	_flag = READ_BIT(SPI1->SR, SPI_SR_BSY);
    }
	_flag = LL_SPI_GetRxFIFOLevel(SPI1);
	while(_flag && (_timeout--)){
//    	READ_REG(*((__IO uint8_t *)&SPI1->DR));
		_flag = READ_BIT(SPI1->SR, SPI_SR_FRLVL);
	}
    if(_timeout) return 0;
    else{
		READ_REG(*((__IO uint16_t *)&SPI1->DR));
		READ_REG(*((__IO uint16_t *)&SPI1->DR));
    	return 1;
    }
}

static void ads_spi_irq_handler(void){
	uint32_t _sr = 0, _cr2 = 0;
	_sr = SPI1->SR;
	_cr2 = SPI1->CR2;

	if((_sr & SPI_SR_TXE) && (_cr2 & SPI_CR2_TXEIE)){

	}
	if(!(_sr & SPI_SR_OVR) && (_sr & SPI_SR_RXNE) && (_cr2 & SPI_CR2_RXNEIE)){

	}
	if((_cr2 & SPI_CR2_ERRIE) && ((_sr & SPI_SR_MODF) || (_sr & SPI_SR_OVR) || (_sr & SPI_SR_FRE))){
		if(_sr & SPI_SR_MODF){
			spi_internal_slave_selete(SPI1, SET);
            LL_SPI_ClearFlag_MODF(SPI1);
		}
		if(_sr & SPI_SR_OVR){
            LL_SPI_ClearFlag_OVR(SPI1);
		}
		if(_sr & SPI_SR_FRE){
            LL_SPI_ClearFlag_FRE(SPI1);
		}
	}
}

static uint8_t ads_start_receive_data(uint8_t *in_rx_ptr, uint16_t in_size){  // 接收数据使用DMA
	uint32_t ret = 0;

#if(ADS_TX_RX_IN_ONE)
    if(LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_SPI1) == 0) LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	if(LL_SPI_IsEnabled(SPI1)) LL_SPI_Disable(SPI1);
	while(LL_SPI_GetRxFIFOLevel(SPI1)) READ_REG(*((__IO uint16_t *)&SPI1->DR));
    LL_SPI_EnableIT_ERR(SPI1);
    // RX
	if(!LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_DMA1)) LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	uint32_t _dma_ch = (uint32_t)DMA1 + (uint32_t)(CHANNEL_OFFSET_TAB[LL_DMA_CHANNEL_2]);
//	NVIC_EnableIRQ(DMA1_Channel2_IRQn);

	CLEAR_BIT(((DMA_Channel_TypeDef *)(_dma_ch))->CCR, DMA_CCR_EN);
//	LL_DMA_ClearFlag_GI2(DMA1);
//    WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CPAR, LL_SPI_DMA_GetRegAddr(SPI1)); // 源地址
    WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CMAR, (uint32_t)in_rx_ptr);               // 目的地址

#if(ADS_CONNECT_MODE == ADS_CASCADED_MODE)
#if(ADS_READ_DATA_IN_COMMAND)
	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CNDTR, (ADS_DMA_TS + 1));
#else
	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CNDTR, ADS_DMA_TS);
#endif
#endif
//	if(LL_DMA_GetPeriphRequest(DMA1, LL_DMA_CHANNEL_2) != LL_DMA_REQUEST_1)
//		LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMA_REQUEST_1);
//    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);  // RX
	SET_BIT(((DMA_Channel_TypeDef *)(_dma_ch))->CCR, DMA_CCR_EN);
//	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CCR,
//			LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
//			LL_DMA_PRIORITY_VERYHIGH  |
//			LL_DMA_MODE_NORMAL        |
//			LL_DMA_PERIPH_NOINCREMENT |
//			LL_DMA_MEMORY_INCREMENT   |
//			LL_DMA_PDATAALIGN_BYTE    |
//			LL_DMA_MDATAALIGN_BYTE    |
//			DMA_CCR_TCIE              |
//			DMA_CCR_TEIE              |
//			DMA_CCR_EN);

	if(LL_SPI_IsEnabledDMAReq_RX(SPI1) == 0)
		LL_SPI_EnableDMAReq_RX(SPI1);

	// TX
	_dma_ch = (uint32_t)DMA1 + (uint32_t)(CHANNEL_OFFSET_TAB[LL_DMA_CHANNEL_3]);
//	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	CLEAR_BIT(((DMA_Channel_TypeDef *)(_dma_ch))->CCR, DMA_CCR_EN);
//	LL_DMA_ClearFlag_GI3(DMA1);
//    WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CPAR, LL_SPI_DMA_GetRegAddr(SPI1)); // 目的地址
//    WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CMAR, (uint32_t)ads_rdata_comm);    // 源地址
#if(ADS_CONNECT_MODE == ADS_CASCADED_MODE)
#if(ADS_READ_DATA_IN_COMMAND)
	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CNDTR, (ADS_DMA_TS + 1));
#else
	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CNDTR, ADS_DMA_TS);
#endif
#endif

//	if(LL_DMA_GetPeriphRequest(DMA1, LL_DMA_CHANNEL_3) != LL_DMA_REQUEST_1)
//		LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_1);
//	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3); // TX
	SET_BIT(((DMA_Channel_TypeDef *)(_dma_ch))->CCR, DMA_CCR_EN);
//	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CCR,
//			LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
//			LL_DMA_PRIORITY_HIGH      |
//			LL_DMA_MODE_NORMAL        |
//			LL_DMA_PERIPH_NOINCREMENT |
//			LL_DMA_MEMORY_INCREMENT   |
//			LL_DMA_PDATAALIGN_BYTE    |
//			LL_DMA_MDATAALIGN_BYTE    |
//			DMA_CCR_TCIE              |
//			DMA_CCR_TEIE              |
//			DMA_CCR_EN);

    if(LL_SPI_IsEnabledDMAReq_TX(SPI1) == 0)
    	LL_SPI_EnableDMAReq_TX(SPI1);

    if(LL_SPI_IsEnabled(SPI1) == 0)
    	LL_SPI_Enable(SPI1);
    return ret;
#endif
}

static void ads_cpoy_data_to_buffer(uint8_t *des_ptr, uint8_t *src_ptr, uint16_t _size){
	UNUSED(des_ptr);
    uint32_t len_buf = ADS_DATA_BUFFER_SIZE + ADS_DATA_INDEX_SIZE;

    if(len_buf - ads_data_index >= _size){
        memcpy((uint8_t*)(data_ptr + ads_data_index), (uint8_t*)(src_ptr), _size);  // 复制数据到缓存
        ads_data_index += _size;
    }
    if(ads_data_index >= (len_buf)){      // 缓冲区满

    	register uint8_t *tmp_ptr = NULL;
        if(ads_user_fild){              // 用户正在读取缓冲区, 此时禁止更新缓冲区数据
#if(ADS_TIMESTAMP)
        ads_data_index = 0;
#else
        ads_data_index = 8;
#endif
        	return;
        }
        // 更新缓冲区数据
    	ads_data_length = len_buf;  // 保存数据长度
        if(ads_data_ready & ADS_DATA_BUF_FULL) ads_data_ready &= (~ADS_DATA_BUF_FULL);
        tmp_ptr = data_ptr;
        data_ptr = user_ptr;
        user_ptr = tmp_ptr;
        ads_user_buf.timestamp = ads_data_buf.timestamp;
        ads_user_buf.index = ads_data_buf.index;
#if(ADS_TIMESTAMP)
        ads_data_index = 0;
#else
        ads_data_index = 8;
#endif
        ads_data_ready |= ADS_DATA_BUF_FULL;

        return;
    }
}

static void ads_read_data_cplt_callback(void){
    uint32_t _dma_isr = 0, _dma_ie = 0;
    uint32_t _dma_ch = (uint32_t)DMA1 + (uint32_t)(CHANNEL_OFFSET_TAB[LL_DMA_CHANNEL_2]);
    _dma_isr = DMA1->ISR;
    _dma_ie = ((DMA_Channel_TypeDef *)(_dma_ch))->CCR;

//	if(LL_DMA_IsActiveFlag_TE2(DMA1)){
	if((_dma_isr & DMA_ISR_TEIF2) && (_dma_ie & DMA_CCR_TEIE)){
		LL_DMA_ClearFlag_TE2(DMA1);
	}

#if(ADS_TX_RX_IN_ONE)
#if(ADS_RXDMA_MODE_CIRCULAR && ADS_TXDMA_MODE_CIRCULAR)
    if(_dma_isr & DMA_ISR_HTIF2){    // 数据接收一半
    	ads_bsp_selete_cs_by_index(1, RESET);
    	LL_DMA_ClearFlag_HT2(DMA1);
    }
    if(_dma_isr & DMA_ISR_HTIF3){
    	LL_DMA_ClearFlag_HT3(DMA1);
    }
#endif
//	if(LL_DMA_IsActiveFlag_TC2(DMA1)){   // RX Complete
    if((_dma_isr & DMA_ISR_TCIF2) && (_dma_ie & DMA_CCR_TCIE)){
		LL_DMA_ClearFlag_GI2(DMA1);
		ads_index += 1;
#if(ADS_CONNECT_MODE == ADS_CASCADED_MODE)
        // 读取第二个通道
		if(ads_index < ADS_CNT){
			_waitfor_spi1_rxcplt();
			LL_SPI_DisableDMAReq_RX(SPI1);
			LL_SPI_Disable(SPI1);
			ads_bsp_selete_cs_by_index(ads_index, RESET);
#if(ADS_READ_DATA_IN_COMMAND)
		    ads_start_receive_data(&ads_rdata_buffer[ADS_DMA_TS + 1], (ADS_DMA_TS));
#else
		    ads_start_receive_data(&ads_rdata_buffer[ADS_DMA_TS], (ADS_DMA_TS));
#endif
		}
		if(ads_index == ADS_CNT){   // 所有通道读取完成
			ads_index = 0;
			LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
			LL_SPI_DisableDMAReq_RX(SPI1);
			_waitfor_spi1_alldone();
			LL_SPI_Disable(SPI1);
			LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_SPI1);    // 关闭SPI时钟
			LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_DMA1); // 关闭DMA1时钟
			ads_read_status &= (~ADS_READING);
			ads_read_status |= (ADS_READ_COMPLETE);
            rtc_reset_ads_timeout();
#if(ADS_READ_DATA_IN_COMMAND)
#if(ADS_TIMESTAMP)
			ads_cpoy_data_to_buffer((uint8_t*)data_ptr, &ads_rdata_buffer[1], ADS_DMA_TS);     // 复制数据到缓冲区
			ads_cpoy_data_to_buffer((uint8_t*)data_ptr, &ads_rdata_buffer[ADS_DMA_TS + 2], ADS_DMA_TS);
#else
			ads_cpoy_data_to_buffer((uint8_t*)&data_ptr[ADS_DATA_INDEX_SIZE], &ads_rdata_buffer[1], ADS_DMA_TS);     // 复制数据到缓冲区
			ads_cpoy_data_to_buffer((uint8_t*)&data_ptr[ADS_DATA_INDEX_SIZE], &ads_rdata_buffer[ADS_DMA_TS + 2], ADS_DMA_TS);
#endif
#else
#if(ADS_TIMESTAMP)
			ads_cpoy_data_to_buffer((uint8_t*)data_ptr, ads_rdata_buffer, ADS_DMA_TS << 1);     // 复制数据到缓冲区
#else

			ads_cpoy_data_to_buffer((uint8_t*)&data_ptr[ADS_DATA_INDEX_SIZE], ads_rdata_buffer, ADS_DMA_TS << 1);     // 复制数据到缓冲区
#endif
#endif
//		    ads_drdy_exti_enable();
		}
#elif(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)

#endif
	}
#endif
}

#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)

static uint8_t ads_send_rdata_command(void){      // 发送一个命令
	uint32_t ret = 0;
	uint32_t _timeout = 4000000;
	if(LL_SPI_IsEnabledDMAReq_RX(SPI1)) LL_SPI_DisableDMAReq_RX(SPI1);
	if(LL_SPI_IsEnabledDMAReq_TX(SPI1)) LL_SPI_DisableDMAReq_TX(SPI1);
	ret = LL_SPI_GetTxFIFOLevel(SPI1);
    while((ret) && (_timeout--)){      // 还有数据帧未处理
    	ret = READ_BIT(SPI1->SR, SPI_SR_FTLVL);
    }
	ret = LL_SPI_IsActiveFlag_BSY(SPI1);
    while((ret) && (_timeout--)){      // 还有数据帧未处理
    	ret = READ_BIT(SPI1->SR, SPI_SR_BSY);
    }
    if(!_timeout) return 1;
    if(LL_SPI_IsEnabled(SPI1)) LL_SPI_Disable(SPI1);             // 终止
    LL_SPI_SetTransferDirection(SPI1, LL_SPI_FULL_DUPLEX);
    LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);         // 8bit
    LL_SPI_TransmitData8(SPI1, ADS_COMMAND_RDATA);
    SET_BIT(SPI1->CR2, SPI_CR2_TXEIE | SPI_CR2_ERRIE);

//    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
//    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

    LL_SPI_Enable(SPI1);  // 开始传输
    return 0;
}
#endif

static void ads_send_rdata_command_cplt_callback(void){
#if(ADS_CONNECT_MODE == ADS_CASCADED_MODE)
    uint32_t _dma_isr = 0, _dma_ie = 0;
    uint32_t _dma_ch = (uint32_t)DMA1 + (uint32_t)(CHANNEL_OFFSET_TAB[LL_DMA_CHANNEL_3]);
    _dma_isr = DMA1->ISR;
    _dma_ie = ((DMA_Channel_TypeDef *)(_dma_ch))->CCR;
//	if(LL_DMA_IsActiveFlag_TC3(DMA1)){   // TX Complete
    if((_dma_isr & DMA_ISR_TCIF3) && (_dma_ie & DMA_CCR_TCIE)){
		LL_DMA_ClearFlag_GI3(DMA1);
//		LL_DMA_ClearFlag_TC3(DMA1);
		if(ads_index >= ADS_CNT){
			LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
			LL_SPI_DisableDMAReq_TX(SPI1);
//			ads_drdy_exti_enable();
		}else{
//			ads_drdy_exti_disable();
		}
	}
//	if(LL_DMA_IsActiveFlag_TE3(DMA1)){
	if((_dma_isr & DMA_ISR_TEIF3) && (_dma_ie & DMA_CCR_TEIE)){
		LL_DMA_ClearFlag_TE3(DMA1);
	}

#endif
#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)
	uint32_t _sr = SPI1->SR;
	uint32_t _it = SPI1->CR2;
	uint32_t _timeout = 4000000;
	uint32_t _flag = 0;
	if(((_sr & SPI_SR_OVR) == 0) && (_sr & SPI_SR_RXNE) && (_it & SPI_CR2_RXNEIE)){
        CLEAR_BIT(SPI1->CR2, SPI_CR2_RXNEIE);
	}
	LL_SPI_IsActiveFlag_TXE(SPI1);
	if((_sr & SPI_SR_TXE) && (_it & SPI_CR2_TXEIE)){
//        LL_SPI_DisableIT_TXE(SPI1);
		CLEAR_BIT(SPI1->CR2, (SPI_CR2_TXEIE | SPI_CR2_ERRIE));
        _flag = LL_SPI_GetTxFIFOLevel(SPI1);
        while(_flag && (_timeout--)){
        	_flag = READ_BIT(SPI1->SR, SPI_SR_FTLVL);
        }
        _flag = LL_SPI_IsActiveFlag_BSY(SPI1);
        while(_flag && (_timeout--)){
        	_flag = READ_BIT(SPI1->SR, SPI_SR_BSY);
        }
        _flag = LL_SPI_GetRxFIFOLevel(SPI1);
        while(_flag && (_timeout--)){
        	READ_REG(*((__IO uint8_t *)&SPI1->DR));
        	_flag = READ_BIT(SPI1->SR, SPI_SR_FRLVL);
        }
        if(LL_SPI_IsEnabled(SPI1)) LL_SPI_Disable(SPI1);

        LL_SPI_EnableDMAReq_RX(SPI1);

        st_irq_handler_register(DMA1_Channel2_IRQn, ads_read_data_cplt_callback);
        st_irq_handler_register(DMA1_Channel3_IRQn, ads_read_data_cplt_callback);
        ads_start_receive_data(ads_rdata_buffer, (ADS_DMA_TS));
	}
#endif
}


static void ads_data_ready_callback(void){
	if((LL_GPIO_ReadInputPort(EXTI_DRDY_GPIO_Port) & EXTI_DRDY_Pin) == 0){  // 低电平
		ads_drdy_count += 1;
	    if((ads_conver_control) && (!(!(ads_drdy_count % ads_conver_control)))) return;

		if(ads_data_index <= 8){// 第一个数据打时间戳
			st_rtc_get_time((struct tm*)&ads_data_timestamp);
	        ads_data_buf.timestamp = ((uint32_t)((ads_data_timestamp.tm_year + 1900) - 2020) << 26) | ((uint32_t)ads_data_timestamp.tm_mon << 22) |
	    			((uint32_t)ads_data_timestamp.tm_yday << 17) |
					((uint32_t)ads_data_timestamp.tm_hour << 12) | ((uint32_t)ads_data_timestamp.tm_min << 6) | (ads_data_timestamp.tm_sec);
//	        ads_user_buf.timestamp = ads_data_buf.timestamp;
	        ads_data_buf.index += 1;
		}
	    if((!(ads_read_status & ADS_READING))){
	    	ads_read_status |= (ADS_READING);
	    	ads_read_status &= (~ADS_READ_COMPLETE);
	    	ads_bsp_selete_cs_by_index(0, RESET);
	        ads_start_receive_data(ads_rdata_buffer, (ADS_DMA_TS));
	    }
	}
}

void ads_register_drdy_callback(void *callback_function){
	st_exti_irq_handler_register(EXTI_DRDY_EXTI_IRQn, 7, callback_function);
}


uint8_t ads_read_data_by_dma_init(uint8_t *tx_ptr, uint8_t *rx_ptr, uint16_t in_size){
	uint8_t ret = 0;
#if(ADS_TX_RX_IN_ONE)

	uint32_t _rxdma_mode = (ADS_RXDMA_MODE_CIRCULAR) ? (LL_DMA_MODE_CIRCULAR | DMA_CCR_HTIE) : LL_DMA_MODE_NORMAL;
	uint32_t _txdma_mode = (ADS_TXDMA_MODE_CIRCULAR) ? (LL_DMA_MODE_CIRCULAR | DMA_CCR_HTIE) : LL_DMA_MODE_NORMAL;
    uint16_t _size = 0;

    memset((uint8_t*)ads_data_buffer, 0, sizeof(ads_data_buffer));
    memset((uint8_t*)ads_data_buffer2, 0, sizeof(ads_data_buffer2));
    memset(ads_rdata_comm, 0, sizeof(ads_rdata_comm));

#if(ADS_TIMESTAMP)
	data_ptr = &ads_data_buffer[ADS_DATA_INDEX_SIZE];
	user_ptr = &ads_data_buffer2[ADS_DATA_INDEX_SIZE];
	ads_data_index = 0;
#else
	data_ptr = ads_data_buffer;
	user_ptr = ads_data_buffer2;
	ads_data_index = 8;
#endif

	ads_index = 0;
	ads_user_fild = 0;
	ads_read_status = 0;
	ads_data_buf.data = ads_data_buffer;
	ads_data_buf.data_size = ADS_DATA_BUFFER_SIZE;
	ads_user_buf.data = ads_data_buffer2;
	ads_user_buf.data_size = ADS_DATA_BUFFER_SIZE;

    // 注册读取数据回调函数
    st_exti_irq_handler_register(EXTI_DRDY_EXTI_IRQn, 7, ads_data_ready_callback);

    if(ads_conver_rate < (128000 / ads_osr)){  // 采样率小于最小值, 设定最小采样率, 间隔取采样值
        ads_conver_control = (128000 / ads_osr / ads_conver_rate) + 1;
        ads_drdy_count = 0;
    }else{
    	ads_conver_control = 0;
        ads_drdy_count = 0;
    }
#if(ADS_READ_DATA_IN_COMMAND)
    if(in_size)
    	_size = in_size;
    else
    	_size = ADS_DMA_TS + 1;
    ads_rdata_comm[0] = ADS_COMMAND_RDATA;
#if(ADS_RXDMA_MODE_CIRCULAR && ADS_TXDMA_MODE_CIRCULAR)
    ads_rdata_comm[_size + 1] = ADS_COMMAND_RDATA;
    _size *= 2;
#endif
#else
    _size = ADS_DMA_TS;
#endif

    LL_SPI_Disable(SPI1);
    LL_SPI_DisableDMAReq_RX(SPI1);
    LL_SPI_DisableDMAReq_TX(SPI1);
    CLEAR_BIT(SPI1->CR2, SPI_CR2_TXEIE | SPI_CR2_RXNEIE);
    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);

    st_irq_handler_register(DMA1_Channel2_IRQn, ads_read_data_cplt_callback);  // RX
    st_irq_handler_register(DMA1_Channel3_IRQn, ads_send_rdata_command_cplt_callback);  // TX
    st_irq_handler_register(SPI1_IRQn, ads_spi_irq_handler);

	if(!LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_DMA1)) LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	uint32_t _dma_ch = (uint32_t)DMA1 + (uint32_t)(CHANNEL_OFFSET_TAB[LL_DMA_CHANNEL_2]);
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	// RX
	CLEAR_BIT(((DMA_Channel_TypeDef *)(_dma_ch))->CCR, DMA_CCR_EN);
	LL_DMA_ClearFlag_GI2(DMA1);
    WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CPAR, LL_SPI_DMA_GetRegAddr(SPI1)); // 源地址
    if(rx_ptr == NULL)
    	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CMAR, (uint32_t)ads_rdata_buffer);  // 目的地址
    else
    	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CMAR, (uint32_t)rx_ptr);  // 目的地址
	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CNDTR, _size);

	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMA_REQUEST_1);
//	uint32_t _dma_csl = ((uint32_t)((uint32_t)DMA1 + DMA_CSELR_OFFSET));
//	WRITE_REG(((DMA_Request_TypeDef *)(_dma_csl))->CSELR, (LL_DMA_REQUEST_1 << POSITION_VAL(DMA_CSELR_C1S << (LL_DMA_CHANNEL_2 << 2))));
	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CCR,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
			LL_DMA_PRIORITY_VERYHIGH  |
			_rxdma_mode               |
			LL_DMA_PERIPH_NOINCREMENT |
			LL_DMA_MEMORY_INCREMENT   |
			LL_DMA_PDATAALIGN_BYTE    |
			LL_DMA_MDATAALIGN_BYTE    |
			DMA_CCR_TCIE              |
			DMA_CCR_TEIE              );

    // TX
	_dma_ch = (uint32_t)DMA1 + (uint32_t)(CHANNEL_OFFSET_TAB[LL_DMA_CHANNEL_3]);
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	CLEAR_BIT(((DMA_Channel_TypeDef *)(_dma_ch))->CCR, DMA_CCR_EN);
	LL_DMA_ClearFlag_GI3(DMA1);
    WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CPAR, LL_SPI_DMA_GetRegAddr(SPI1)); // 目的地址
    if(tx_ptr == NULL)
    	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CMAR, (uint32_t)ads_rdata_comm);    // 源地址
    else
    	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CMAR, (uint32_t)tx_ptr);    // 源地址
	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CNDTR, _size);

//	_dma_csl = ((uint32_t)((uint32_t)DMA1 + DMA_CSELR_OFFSET));
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_1);
//	WRITE_REG(((DMA_Request_TypeDef *)(_dma_csl))->CSELR, (LL_DMA_REQUEST_1 << POSITION_VAL(DMA_CSELR_C1S << (LL_DMA_CHANNEL_3 << 2))));
	WRITE_REG(((DMA_Channel_TypeDef *)(_dma_ch))->CCR,
			LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
			LL_DMA_PRIORITY_VERYHIGH  |
			_txdma_mode                 |
			LL_DMA_PERIPH_NOINCREMENT |
			LL_DMA_MEMORY_INCREMENT   |
			LL_DMA_PDATAALIGN_BYTE    |
			LL_DMA_MDATAALIGN_BYTE    |
			DMA_CCR_TCIE              |
			DMA_CCR_TEIE              );

#else
	data_ptr = ads_data_buffer;
	user_ptr = ads_data_buffer2;

	memset((uint8_t*)ads_rdata_comm, 0, sizeof(ads_rdata_comm));
	ads_rdata_comm[0] = ADS_COMMAND_RDATA;   // 读取数据命令

    LL_SPI_Disable(SPI1);

	// 配置 DMA 读, 接受数据
    _spi1_rx_dma_config((uint8_t*)ads_rdata_buffer, ADS_DMA_TS);

	// 配置 DMA 写, 发读数据命令
    _spi1_tx_dma_config((uint8_t*)ads_rdata_comm, 1);

    ads_only_rx_fild = 1;
#endif
	return ret;
}

uint8_t ads_get_read_status(void){
    return ads_read_status;
}


void ads_stop_read_data_by_dma(void){
	LL_SPI_Disable(SPI1);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
	ads_only_rx_fild = 0;
}

uint8_t ads_data_is_buffer_full(void){
    return (ads_data_ready & ADS_DATA_BUF_FULL);
}

uint8_t ads_user_read_fild(uint8_t in_fild){
	ads_user_fild = in_fild;
	return ads_user_fild;
}

uint8_t ads_save_data_to_file(void* in_file, void* in_parma){
    uint8_t ret = 1;
    FIL *_fil_ptr = NULL;

    _fil_ptr = (FIL*)in_file;
    ads_user_fild = 0x01;

    if(*(uint8_t*)in_parma){     // 保存成 bin
#if(ADS_TIMESTAMP)
    	uint8_t _t[ADS_DATA_INDEX_SIZE] = {0};
    	*(uint32_t *)_t = ads_user_buf.timestamp;
    	*(uint32_t *)&_t[ADS_DATA_INDEX_SIZE >> 1] = _save_times;
    	ret = f_write(_fil_ptr, _t, sizeof(_t), NULL);
    	ret = f_write(_fil_ptr, (uint8_t*)user_ptr, ads_data_length, NULL);
#else
    	*(uint32_t *)&user_ptr[0] = ads_user_buf.timestamp;
    	*(uint32_t *)&user_ptr[4] = ads_user_buf.index;
    	ret = f_write(_fil_ptr, (uint8_t*)user_ptr, ads_data_length, NULL);
#endif
    }else{                       // 保存成 txt
    	uint8_t *_ascii_txt = NULL;

    	if(_ascii_txt == NULL){
    		return 1;
    	}else{
    		goto fatfs_sync_section;
    	}
    }
fatfs_sync_section:
    if(ads_data_ready & ADS_DATA_BUF_FULL) ads_data_ready &= (~ADS_DATA_BUF_FULL);   // 清除缓存区状态
    ads_user_fild = 0x00;
    ret = f_sync(_fil_ptr);

    return ret;
}


uint8_t* get_ads_data_buffer(void){
    if(ads_data_ready & ADS_DATA_BUF_FULL){  // 缓冲数据可用
    	return (uint8_t*)user_ptr;
    }
    return NULL;
}

static void ads_lptim2_handler(void){
	if(LL_LPTIM_IsActiveFlag_ARRM(LPTIM2)){
		LL_LPTIM_ClearFLAG_ARRM(LPTIM2);
		ADS_Start_Convert();
		NVIC_DisableIRQ(LPTIM2_IRQn);
	}
}

void ads_bsp_pin_start(void){
    uint32_t _ft = 0, _lt = 1;
    st_irq_handler_register(LPTIM2_IRQn, ads_lptim2_handler);
    if(LL_LPTIM_IsActiveFlag_ARRM(LPTIM2)) LL_LPTIM_ClearFLAG_ARRM(LPTIM2);
    _ft = _lt = HAL_GetTick();
    while((LL_LPTIM_IsActiveFlag_ARRM(LPTIM2) == 0) && ((_ft - _lt) < 3000)){
    	_lt = HAL_GetTick();
    }
    ADS_Start_Convert();
//    NVIC_EnableIRQ(LPTIM2_IRQn);
    while(LL_GPIO_IsOutputPinSet(ADS_START_GPIO_Port, ADS_START_Pin) != SET){
    	__NOP();
    }
    st_irq_handler_register(LPTIM2_IRQn, NULL);
}

void ads_bsp_pin_stop(void){
	ADS_Stop_Convert();
}

void ads_bsp_selete_cs_by_index(uint8_t in_index, uint8_t in_value){
	UNUSED(in_value);
    switch(in_index){
    case 0:
    	LL_GPIO_ResetOutputPin(ADS_CS_GPIO_Port, ADS_CS_Pin);
    	break;
    case 1:
    	LL_GPIO_SetOutputPin(ADS_CS_GPIO_Port, ADS_CS_Pin);
    	break;
    default:
    	LL_GPIO_ResetOutputPin(ADS_CS_GPIO_Port, ADS_CS_Pin);
    	break;
    }
}


void ads_bsp_power(uint8_t in_status){
	if(in_status == SET){
		V5_ON();
	}else{
		V5_OFF();
	}
}

static void ads_cal_data_ready_callback(void){
	if((LL_GPIO_ReadInputPort(EXTI_DRDY_GPIO_Port) & EXTI_DRDY_Pin) == 0){    // 低电平
	    if((!(ads_read_status & ADS_READING)) && (!ads_index)){
	    	ads_read_status |= (ADS_READING);
	    	ads_bsp_selete_cs_by_index(0, RESET);
	        ads_start_receive_data(ads_rdata_buffer, (ADS_DMA_TS));
	    }
	}
}

static void ads_cal_data_cplt_callback(void){
    uint32_t _dma_isr = 0, _dma_ie = 0;
    uint32_t _dma_ch = (uint32_t)DMA1 + (uint32_t)(CHANNEL_OFFSET_TAB[LL_DMA_CHANNEL_2]);
    _dma_isr = DMA1->ISR;
    _dma_ie = ((DMA_Channel_TypeDef *)(_dma_ch))->CCR;
#if(ADS_TX_RX_IN_ONE)
   // RX Complete
    if((_dma_isr & DMA_ISR_TCIF2) && (_dma_ie & DMA_CCR_TCIE)){
		LL_DMA_ClearFlag_GI2(DMA1);
		ads_index += 1;
#if(ADS_CONNECT_MODE == ADS_CASCADED_MODE)
        // 读取第二个通道
		if(ads_index < ADS_CNT){
			_waitfor_spi1_rxcplt();
			LL_SPI_DisableDMAReq_RX(SPI1);
			LL_SPI_Disable(SPI1);
			ads_bsp_selete_cs_by_index(ads_index, RESET);
#if(ADS_READ_DATA_IN_COMMAND)
			memcpy(&ads_data_buffer[ads_data_index], &ads_rdata_buffer[1], 3);      // 通道1
			ads_start_receive_data(ads_rdata_buffer, (ADS_DMA_TS));
#else
			memcpy((uint8_t*)&ads_data_buffer[ads_data_index], &ads_rdata_buffer[0], 3);
			ads_start_receive_data(ads_rdata_buffer, (ADS_DMA_TS));
#endif
		}
		if(ads_index == ADS_CNT){   // 所有通道读取完成
			ads_index = 0;
			LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
			LL_SPI_DisableDMAReq_RX(SPI1);
			_waitfor_spi1_alldone();
			LL_SPI_Disable(SPI1);
			ads_read_status &= (~ADS_READING);
#if(ADS_READ_DATA_IN_COMMAND)
			memcpy(&ads_data_buffer2[ads_data_index], &ads_rdata_buffer[1], 3);    // 通道2
			ads_data_index += 3;
			if(ads_data_index == (3 << 13)){
				ads_drdy_exti_disable();
				ads_data_ready |= ADS_DATA_BUF_FULL;
			}
#else
			memcpy((uint8_t*)&ads_data_buffer2[ads_data_index], &ads_rdata_buffer[0], 3);
			ads_data_index += 3;
			if(ads_data_index == (3 << 13)){
				ads_drdy_exti_disable();
				ads_data_ready |= ADS_DATA_BUF_FULL;
			}
#endif
		}
#elif(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)

#endif
	}
	if((_dma_isr & DMA_ISR_TEIF2) && (_dma_ie & DMA_CCR_TEIE)){
		LL_DMA_ClearFlag_TE2(DMA1);
	}

#endif
}
uint8_t ads_bsp_calibrate(uint32_t *out_cal_value, uint8_t in_size){
	uint8_t ret = 0;
    uint32_t cal_complements[ADS_CNT] = { 0 };
    uint32_t i = 0;

    if((cal_complements != NULL)){
    	ads_data_index = 0;
        ads_read_data_by_dma_init(ads_rdata_comm, ads_rdata_buffer, 0);
        st_exti_irq_handler_register(EXTI_DRDY_EXTI_IRQn, 7, ads_cal_data_ready_callback);
        st_irq_handler_register(DMA1_Channel2_IRQn, ads_cal_data_cplt_callback);  // RX
        st_irq_handler_register(DMA1_Channel3_IRQn, ads_send_rdata_command_cplt_callback);  // TX
        st_irq_handler_register(SPI1_IRQn, ads_spi_irq_handler);
    	ads_bsp_pin_start();
		while(ads_data_is_buffer_full() == 0){
			__NOP();
		}
		ads_bsp_pin_stop();
		st_exti_irq_handler_register(EXTI_DRDY_EXTI_IRQn, 7, NULL);
//		spi_set_clk_div(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV256);  // 低速读取设备配置
		i = 0;
        do{
        	cal_complements[0] += ((ads_data_buffer[i] << 16) | (ads_data_buffer[i + 1] << 8) | (ads_data_buffer[i + 2]));
            i += 3;
        }while(i < ads_data_index);
        i = 0;
        do{
        	cal_complements[1] += ((ads_data_buffer2[i] << 16) | (ads_data_buffer2[i + 1] << 8) | (ads_data_buffer2[i + 2]));
            i += 3;
        }while(i < ads_data_index);
        cal_complements[0] >>= 13;
        cal_complements[1] >>= 13;
//        i = 0;
//        do{
//      	    ads_bsp_selete_cs_by_index(i, RESET);
//      	    ads_cfg_sys_offset_calibration(cal_complements[i]);
//            i += 1;
//        }while(i < ADS_CNT);
        if(out_cal_value != NULL){
        	out_cal_value[0] = cal_complements[0];
        	out_cal_value[1] = cal_complements[1];
        }
        ads_data_index = 0;
    }
    return ret;
}
