/*
 * bsp_ads127.c
 *
 *  Created on: 2020年12月4日
 *      Author: XIAOSENLUO
 */


#include "main.h"

#include "ads127.h"
#include "bsp_ads127.h"
#include "stm32l4xx_it.h"

#include "stdbool.h"

#if(0)

#if(1)
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

static volatile uint8_t _lock = 0;

static inline void lock_buffer(void){
	_lock = 1;
}

static inline void unlock_buffer(void){
    _lock = 0;
}

static inline bool is_lock_buffer(void){
	return _lock;
}

static ads_data_t ads_data_buf = {0};
static ads_data_t ads_user_buf = {0};

static volatile struct tm ads_data_timestamp = { 0 };
extern ads_drv_t ads_drv;

ads_config_t ads_config = {.conver_rate = 2000, .osr = 32};


static volatile uint8_t ads_conver_control __attribute__((section(".ram2"))) = 0;
volatile uint32_t ads_drdy_count __attribute__((section(".ram2"))) = 0;

#if(ADS_CONNECT_MODE == ADS_DAISYCHAIN_MODE)
static void ads_send_rdata_command_cplt_callback(void);
static uint8_t ads_send_rdata_command(void);
#endif
static uint8_t _waitfor_spi1_alldone(void);
static uint8_t ads_start_receive_data(uint8_t *in_rx_ptr, uint16_t in_size);
static void ads_read_data_cplt_callback(void);
static void ads_data_ready_callback(void);


void ads_set_conver_rate(uint32_t rate, uint16_t osr){
	ads_config.conver_rate = rate;
	ads_config.osr = osr;
    uint32_t clk = 0;

	if((ads_config.conver_rate * ads_config.osr) <= 100000){   // 不满足最低 100KHz
		if(ads_config.conver_rate * ads_config.osr == 0){
		    clk = 0;   // 关闭采样时钟
		}else{
			clk = 128000;   // 采样主时钟  >=100KHz
		}
	}else clk = ads_config.conver_rate * ads_config.osr;

	lptim2_use_for_ads_clk(clk);    // 设置采样时钟频率

    if(ads_config.conver_rate < (clk / ads_config.osr)){  // 采样率小于最小值, 设定最小采样率, 间隔取采样值
        ads_conver_control = (clk / ads_config.osr / ads_config.conver_rate) + 1;
    }else{
    	ads_conver_control = 0;
    }

	ads_drdy_count = 0;
}


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
			ads_bsp_selete_cs_by_index(ads_index);
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
	    if((ads_conver_control) && (ads_drdy_count % ads_conver_control)) return;

		if(ads_data_index <= 8){// 第一个数据打时间戳
			st_rtc_get_time((struct tm*)&ads_data_timestamp);
	        ads_data_buf.timestamp = ((uint32_t)((ads_data_timestamp.tm_year + 1900) - 2020) << 26) | ((uint32_t)ads_data_timestamp.tm_mon << 22) |
	    			((uint32_t)ads_data_timestamp.tm_mday << 17) |
					((uint32_t)ads_data_timestamp.tm_hour << 12) | ((uint32_t)ads_data_timestamp.tm_min << 6) | (ads_data_timestamp.tm_sec);
//	        ads_user_buf.timestamp = ads_data_buf.timestamp;
	        ads_data_buf.index += 1;
		}
	    if((!(ads_read_status & ADS_READING))){
	    	ads_read_status |= (ADS_READING);
	    	ads_read_status &= (~ADS_READ_COMPLETE);
	    	ads_bsp_selete_cs_by_index(0);
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

    if(!ads_data_is_buffer_full()) return 255;
    if(_fil_ptr->obj.fs == NULL) return FR_INVALID_OBJECT;
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
#define TXT_BUFF_SIZE          2048
    	char _ascii_txt[TXT_BUFF_SIZE + 1];
    	uint8_t *ptr = &user_ptr[0];
    	uint32_t len = 0, ads_len = ads_data_length, data_size = 0;
    	len = data_to_str(ptr, 8, _ascii_txt, TXT_BUFF_SIZE + 1);
    	ret = f_write(_fil_ptr, _ascii_txt, len, NULL);
    	ads_len -= 8;
        ptr += 8;
        do{
            if(ads_len >= (TXT_BUFF_SIZE >> 1)){
            	data_size = (TXT_BUFF_SIZE >> 1);
                ads_len -= (TXT_BUFF_SIZE >> 1);
            }else{
            	data_size = ads_len;
                ads_len -= ads_len;
            }
            len = data_to_str(ptr, data_size, _ascii_txt, TXT_BUFF_SIZE + 1);  // HEX----->TXT: 一个数据占两个字节
            ptr += data_size;          // 偏移数据地址
            ret = f_write(_fil_ptr, _ascii_txt, len, NULL);
        }while(ads_len);
#undef TXT_BUFF_SIZE
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

void ads_bsp_selete_cs_by_index(uint8_t in_index){
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
	    	ads_bsp_selete_cs_by_index(0);
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
			ads_bsp_selete_cs_by_index(ads_index);
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


#else

extern ads_drv_t ads_drv;
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


static uint8_t rdata_buffer[8] = {0};


void ads_bsp_pin_stop(void){
	ADS_Stop_Convert();
}
void ads_bsp_pin_start(void){
	if(LL_LPTIM_IsActiveFlag_ARRM(LPTIM2)) LL_LPTIM_ClearFLAG_ARRM(LPTIM2);
    while(LL_LPTIM_IsActiveFlag_ARRM(LPTIM2) != SET);
    ADS_Start_Convert();
    while(LL_GPIO_IsOutputPinSet(ADS_START_GPIO_Port, ADS_START_Pin) != SET){
    	__NOP();
    }
}
void ads_bsp_power(uint8_t in_status){
	if(in_status == SET){
		V5_ON();
	}else{
		V5_OFF();
	}
}

void ads_set_conver_rate(uint32_t rate, uint16_t osr){

    uint32_t clk = 0;

	if((rate * osr) <= 100000){   // 不满足最低 100KHz
		if(rate * osr == 0){
		    clk = 0;   // 关闭采样时钟
		}else{
			clk = 128000;   // 采样主时钟  >=100KHz
		}
	}else clk = rate * osr;

	lptim2_use_for_ads_clk(clk);    // 设置采样时钟频率

//    if(ads_config.conver_rate < (clk / ads_config.osr)){  // 采样率小于最小值, 设定最小采样率, 间隔取采样值
////        ads_conver_control = (clk / ads_config.osr / ads_config.conver_rate) + 1;
//    }else{
////    	ads_conver_control = 0;
//    }

//	ads_drdy_count = 0;
}

void ads_bsp_selete_cs_by_index(uint8_t in_index){
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


static void spi1_irq_handler(void){
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

static void spi1_tx_dma_irq_handler(void){

}

static uint8_t _index = 0;
static void spi1_rx_dma_irq_handler(void){
    if(LL_DMA_IsActiveFlag_HT2(DMA1)){
//    	LL_SPI_Disable(SPI1);
//    	ads_bsp_selete_cs_by_index(1);
    	LL_DMA_ClearFlag_HT2(DMA1);
//    	READ_REG(*((__IO uint16_t *)&SPI1->DR));
//    	LL_SPI_Enable(SPI1);
    }
    if(LL_DMA_IsActiveFlag_TC2(DMA1)){
    	LL_SPI_Disable(SPI1);
    	LL_DMA_ClearFlag_GI2(DMA1);
    	_index += 1;
		uint8_t tmp[8];
		memcpy(tmp, rdata_buffer, 4);
    	if(_index == 1){
        	ads_bsp_selete_cs_by_index(1);
        	LL_DMA_ClearFlag_GI2(DMA1);
        	LL_SPI_Enable(SPI1);
    	}

//    	uint8_t tmp[8];
//    	memcpy(tmp, rdata_buffer, 4);
//    	READ_REG(*((__IO uint16_t *)&SPI1->DR));
//    	memset(rdata_buffer, 0, 8);
    }
}

static void ads_drdy_irq_handler(void){
	if((LL_GPIO_ReadInputPort(EXTI_DRDY_GPIO_Port) & EXTI_DRDY_Pin) == 0){
		ads_bsp_selete_cs_by_index(0);
		READ_REG(*((__IO uint16_t *)&SPI1->DR));  // 清空
		_index = 0;

        LL_SPI_Enable(SPI1);
	}
}

static void _spi1_tx_dma_config(uint8_t *ptr, uint16_t _s){  // DMA1 Channel3; Request 1

	/* (1) Enable the clock of DMA1 and DMA1 */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	/* (2) Configure NVIC for DMA transfer complete/error interrupts */

	NVIC_EnableIRQ(DMA1_Channel3_IRQn);

	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
	LL_DMA_ClearFlag_GI3(DMA1);
	/* (3) Configure the DMA1_Channel2 functional parameters */
	LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_3,
			            LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
						LL_DMA_PRIORITY_HIGH |
						LL_DMA_MODE_NORMAL |
						LL_DMA_PERIPH_NOINCREMENT |
						LL_DMA_MEMORY_INCREMENT |
						LL_DMA_PDATAALIGN_BYTE |
						LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3,
			(uint32_t)ptr,  // 源地址
			LL_SPI_DMA_GetRegAddr(SPI1),                // 目的地址
			LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, _s);


	if(LL_DMA_GetPeriphRequest(DMA1, LL_DMA_CHANNEL_3) != LL_DMA_REQUEST_1)
		LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_1);

	/* (5) Enable DMA interrupts complete/error */
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);

	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
}

static void _spi1_rx_dma_config(uint8_t *ptr, uint16_t _s){   // DMA1 Channel2; Request 1
	/* (1) Enable the clock of DMA1 and DMA1 */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	/* (2) Configure NVIC for DMA transfer complete/error interrupts */
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

	LL_DMA_ClearFlag_GI2(DMA1);
	/* (3) Configure the DMA1_Channel2 functional parameters */
	LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_2,
			            LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
						LL_DMA_PRIORITY_HIGH |
						LL_DMA_MODE_CIRCULAR |
						LL_DMA_PERIPH_NOINCREMENT |
						LL_DMA_MEMORY_INCREMENT |
						LL_DMA_PDATAALIGN_BYTE |
						LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2,
			LL_SPI_DMA_GetRegAddr(SPI1),  // 源地址
			(uint32_t)ptr,                // 目的地址
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, _s);

	if(LL_DMA_GetPeriphRequest(DMA1, LL_DMA_CHANNEL_2) != LL_DMA_REQUEST_1)
		LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMA_REQUEST_1);

	/* (5) Enable DMA interrupts complete/error */

	LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_2);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);

	LL_DMA_EnableChannel(DMA1,  LL_DMA_CHANNEL_2);
}


void ads_config_spi_init(void){
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_SPI_DeInit(SPI1);

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

	/* Configure SPI1 communication */
	LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV8); // 时钟分频, spi时钟频率 = PCLK2 / baudrate_presacler;
	LL_SPI_SetTransferDirection(SPI1,LL_SPI_FULL_DUPLEX);   // 全双工
	LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_2EDGE);         // 相位
	LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_LOW);     // 极性
	/* Reset value is LL_SPI_MSB_FIRST */
	LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);     // 高位先出
	LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);       // 数据宽度 8bit
	LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);               // 软件片选
	LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);  //8bit
	LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);               // 主机
	LL_SPI_DisableCRC(SPI1);
//	LL_SPI_Enable(SPI1);

	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);

	LL_SPI_EnableIT_ERR(SPI1);
	st_irq_handler_register(SPI1_IRQn, spi1_irq_handler);
}



void ads_rdata_spi_init(void){
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_SPI_DeInit(SPI1);

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

	/* Configure SPI1 communication */
	LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV8); // 时钟分频, spi时钟频率 = PCLK2 / baudrate_presacler;
	LL_SPI_SetTransferDirection(SPI1,LL_SPI_SIMPLEX_RX);   // Only Rx
	LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_2EDGE);         // 相位
	LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_LOW);     // 极性
	/* Reset value is LL_SPI_MSB_FIRST */
	LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);     // 高位先出
	LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);       // 数据宽度 8bit
	LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);               // 软件片选
	LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);  //8bit
	LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);               // 主机
	LL_SPI_DisableCRC(SPI1);

	LL_SPI_EnableDMAReq_RX(SPI1);

	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);

	LL_SPI_EnableIT_ERR(SPI1);
	st_irq_handler_register(SPI1_IRQn, spi1_irq_handler);
}

uint8_t ads_read_data_by_dma_init(uint8_t *tx_ptr, uint8_t *rx_ptr, uint16_t in_size){

	ads_rdata_spi_init();

    st_exti_irq_handler_register(EXTI_DRDY_EXTI_IRQn, 7, ads_drdy_irq_handler);
    st_irq_handler_register(DMA1_Channel2_IRQn, spi1_rx_dma_irq_handler);  // RX
    st_irq_handler_register(DMA1_Channel3_IRQn, spi1_tx_dma_irq_handler);



    _spi1_rx_dma_config(rdata_buffer, 4);
	return 0;
}

uint8_t ads_save_data_to_file(void* in_file, void* in_parma){

	return 1;
}

#endif
#else

#include "stm32l4xx_hal.h"
#include "stm32l4xx_it.h"
#include "stm32l4xx_hal_exti.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_dma.h"
#include "stm32l4xx_hal_spi.h"

static SPI_HandleTypeDef *ads_spi_handle = NULL;
static GPIO_TypeDef *adsCSPort = NULL;
static int32_t adsCSPin = -1;

void ads127_driver_initialaiz(SPI_HandleTypeDef *spihandle, GPIO_TypeDef * csPort, int32_t csPin){
    uint8_t test[4] = {0, 0, 0, 0};
    ads_spi_handle = spihandle;

    if((csPort != NULL) && (csPin >= 0)){
        GPIO_InitTypeDef GPIO_InitStructure = {
                .Alternate = 0,
                .Mode = GPIO_MODE_OUTPUT_PP,
                .Pin = csPin,
                .Pull = GPIO_PULLUP,
                .Speed = GPIO_SPEED_LOW,
        };
        HAL_GPIO_Init(csPort, &GPIO_InitStructure);
        HAL_GPIO_WritePin(csPort, csPin, 1);
        adsCSPort = csPort;
        adsCSPin = csPin;
    }
    if(HAL_SPI_TransmitReceive(spihandle, test, test, sizeof test, 1000) == HAL_OK){
        ads_spi_handle = spihandle;
    }
}

static void ads127_cs_set_level(uint8_t level){
    if((adsCSPin < 0) || (adsCSPort == NULL)) return;
    if(level){
        LL_GPIO_SetOutputPin(adsCSPort, adsCSPin);
    }else{
        LL_GPIO_ResetOutputPin(adsCSPort, adsCSPin);
    }
}


static void ads127_bsp_spi_transmit_receive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t timeout){
    uint32_t             txallowed = 1U, sr = 0UL;

    timeout = timeout;

    hspi->pRxBuffPtr  = (uint8_t *)pRxData;
    hspi->RxXferCount = Size;
    hspi->RxXferSize  = Size;
    hspi->pTxBuffPtr  = (uint8_t *)pTxData;
    hspi->TxXferCount = Size;
    hspi->TxXferSize  = Size;

    CLEAR_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
//    if((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(hspi);
//    }
    while ((hspi->TxXferCount > 0U) || (hspi->RxXferCount > 0U)){
        if((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)) && (hspi->TxXferCount > 0U) && (txallowed == 1U)){
            if (hspi->TxXferCount > 1U){
                hspi->Instance->DR = *((uint16_t *)hspi->pTxBuffPtr);
                hspi->pTxBuffPtr += sizeof(uint16_t);
                hspi->TxXferCount -= 2U;
            }else{
                *(__IO uint8_t *)&hspi->Instance->DR = (*hspi->pTxBuffPtr);
                hspi->pTxBuffPtr++;
                hspi->TxXferCount--;
            }
            /* Next Data is a reception (Rx). Tx not allowed */
            txallowed = 0U;
        }
        if((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) && (hspi->RxXferCount > 0U)){
            if (hspi->RxXferCount > 1U){
                *((uint16_t *)hspi->pRxBuffPtr) = (uint16_t)hspi->Instance->DR;
                hspi->pRxBuffPtr += sizeof(uint16_t);
                hspi->RxXferCount -= 2U;
                if (hspi->RxXferCount <= 1U){
                    /* Set RX Fifo threshold before to switch on 8 bit data size */
                    SET_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
                }
            }else{
                (*(uint8_t *)hspi->pRxBuffPtr) = *(__IO uint8_t *)&hspi->Instance->DR;
                hspi->pRxBuffPtr++;
                hspi->RxXferCount--;
            }
            /* Next Data is a Transmission (Tx). Tx is allowed */
            txallowed = 1U;
        }
    }
    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));
    __HAL_SPI_DISABLE(hspi);
    //// 检查 发送 FTLVL
    while((hspi->Instance->SR & SPI_SR_FTLVL) != SPI_FTLVL_EMPTY){

    }

    /// 检查 BSY 标志位
    while ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY) ? SET : RESET) != RESET){

    }

    /// 检查 接收 FRLVL
    while((hspi->Instance->SR & SPI_SR_FRLVL) != SPI_FRLVL_EMPTY){
        READ_REG(*((__IO uint8_t *)&hspi->Instance->DR));
    }

    sr = hspi->Instance->SR;
    if(sr & SPI_FLAG_MODF) __HAL_SPI_CLEAR_MODFFLAG(hspi);
    if(sr & SPI_FLAG_OVR) __HAL_SPI_CLEAR_OVRFLAG(hspi);
}

uint32_t ads127_data_frame(ads127_data_frame_handle handle){
#if(0)
    if(ads_spi_handle == NULL) return 0xFFFFFFFFUL;
    HAL_SPI_TransmitReceive_DMA(ads_spi_handle, handle->tx_data, handle->rx_data, handle->cmd_length + handle->data_length);
    __HAL_DMA_DISABLE_IT(ads_spi_handle->hdmatx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(ads_spi_handle->hdmarx, DMA_IT_HT);
    while(ads_spi_handle->hdmatx->State == HAL_DMA_STATE_BUSY);
    while(ads_spi_handle->hdmarx->State == HAL_DMA_STATE_BUSY);
    while(ads_spi_handle->State == HAL_SPI_STATE_BUSY_TX_RX);
    while(ads_spi_handle->State != HAL_SPI_STATE_READY);
#else
    ads127_cs_set_level(0);
    HAL_SPI_TransmitReceive(ads_spi_handle, handle->tx_data, handle->rx_data, handle->cmd_length + handle->data_length, 1000);   /// 阻塞读取
    ads127_cs_set_level(1);
#endif
    return *(uint32_t*)(&handle->rx_data[handle->cmd_length]);
}

typedef struct ads_data_ptr_s{
    struct{
        uint8_t *user;
        uint8_t *isr;
        uint32_t length;
    }ptr;
    uint32_t index;
    struct{
        const uint32_t size;   /// cache 长度
        uint8_t step;    /// 每次读取字节长度
    }limit;

}ads_data_ptr_t;

typedef struct ads_read_ctrl_s{
    union{
        struct{
            uint8_t counter : 4;
            uint8_t response : 4;
        };
        uint8_t val;
    }count;
    ads_data_ptr_t data;
    uint32_t current_counter;
    union{
        struct{
            uint8_t save : 1;
            uint8_t user_read : 1;
            uint8_t swap : 1;
        };
        uint8_t val;
    }action;
}ads_read_ctrl_t;

#define ADS_DATA_CACHE_SIZE                     (4 * 8000)

static uint8_t ads_data_cache1[ADS_DATA_CACHE_SIZE] __attribute__((section(".ram2"))) = { 0 };
static uint8_t ads_data_cache2[ADS_DATA_CACHE_SIZE] __attribute__((section(".ram1"))) = { 0 };

static ads_read_ctrl_t ads_read_ctrl = {
        .count.val = 0,
        .data = {
            .ptr.user = ads_data_cache1,
            .ptr.isr = ads_data_cache2,
            .ptr.length = 0,
            .index = 0,
            .limit.size = ADS_DATA_CACHE_SIZE,
            .limit.step = 4,
        },
        .current_counter  =0,
        .action.val = 0,
};


void ads127_bsp_read_init(const ads_data_init_t* init){
    uint8_t step = 4;

    step -= init->config.cs_enb;
    ads_read_ctrl.data.limit.step = step;
    ads_read_ctrl.data.index = 0;
    ads_read_ctrl.count.response = (init->crate == UINT32_MAX) ? 0 : (init->clk / init->osr / init->crate) - 1;
    ads_read_ctrl.current_counter  = 0;
    ads_read_ctrl.count.counter = 0;
    ads_read_ctrl.action.val = 0;
}

void swap_cache_address(ads_read_ctrl_t * ctrl){
    uint8_t *tmp = ctrl->data.ptr.isr;
    if(ctrl->action.user_read){
        ctrl->data.index = 0;   /// 重置数据指针索引
        return;  /// 用户正在读取, 禁止交换地址
    }
    ctrl->data.ptr.length = ctrl->data.index;    /// 数据长度
    ctrl->data.index = 0;             //// 重置数据指针索引
    ctrl->action.swap = 1;
    ctrl->data.ptr.isr = ctrl->data.ptr.user;
    ctrl->data.ptr.user = tmp;
    ctrl->action.swap = 0;
    ctrl->action.save = 1;  /// 可以保存数据了, 此时用户应该读取 ptr->user指向的内存, 大小为 ptr.length
}

void ads127_bsp_read_data_from_isr(void * ctx){
    UNUSED(ctx);
    if(ads_read_ctrl.count.counter != ads_read_ctrl.count.response){
        ads_read_ctrl.count.counter += 1;
        return;  /// 未达到响应条件, 返回
    }
    ads_read_ctrl.current_counter += 1;
    ads_read_ctrl.count.counter = 0;  /// 重置计数器

    if(ads_spi_handle == NULL) return;
    if((ads_read_ctrl.data.index + ads_read_ctrl.data.limit.step) > ads_read_ctrl.data.limit.size){  /// 检查剩余空间
        /// 剩余空间不够, 交换cache
        swap_cache_address(&ads_read_ctrl);
    }
    uint8_t tx[4] = {0, 0, 0, 0};  //// 无需发送命令

    if(adsCSPin >= 0) LL_GPIO_ResetOutputPin(adsCSPort, adsCSPin);
#if(0)
    HAL_SPI_TransmitReceive_DMA(ads_spi_handle, tx, &ads_read_ctrl.data.ptr.isr[ads_read_ctrl.data.index], ads_read_ctrl.data.limit.step);   /// 非阻塞读取
    __HAL_DMA_DISABLE_IT(ads_spi_handle->hdmatx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(ads_spi_handle->hdmarx, DMA_IT_HT);
#else
    ads127_bsp_spi_transmit_receive(ads_spi_handle, tx, &ads_read_ctrl.data.ptr.isr[ads_read_ctrl.data.index], ads_read_ctrl.data.limit.step, 100);   /// 阻塞读取
    ads_read_ctrl.data.index += ads_read_ctrl.data.limit.step;
    if(adsCSPin >= 0) LL_GPIO_SetOutputPin(adsCSPort, adsCSPin);
#endif
}

void ads127_bsp_read_data_cplt_callback(void){
#if(0)
    ads_read_ctrl.data.index += ads_read_ctrl.data.limit.step;
    if(adsCSPin >= 0) LL_GPIO_SetOutputPin(adsCSPort, adsCSPin);
#endif
}

static GPIO_TypeDef *adsDRDYPinPort = NULL, *adsStartPinPort = NULL, *adsResetPinPort = NULL;
static int32_t adsDRDYPin = -1, adsStartPin = -1, adsResetPin = -1;

static int adsDRDY_IRQn = -255;

void ads127_bsp_drdy_isr_install(GPIO_TypeDef * drdyPort, int32_t drdyPin, ads_drdy_callback_t fn, void *ctx){

    GPIO_InitTypeDef GPIO_InitStructure = {
            .Alternate = 0,
            .Mode = GPIO_MODE_IT_FALLING,
            .Pin = drdyPin,
            .Pull = LL_GPIO_PULL_UP,
            .Speed = GPIO_SPEED_LOW,
    };
    if(drdyPort && (drdyPin >= 0)){
        HAL_GPIO_Init(drdyPort, &GPIO_InitStructure);

        int gpio = gpio_mask2num(drdyPin);
        adsDRDY_IRQn = GPIO_IRQn[gpio];
        HAL_NVIC_SetPriority(adsDRDY_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(adsDRDY_IRQn);
        ll_gpio_exti_isr_install((gpio_num_t)gpio, fn, ctx);
    }
    adsDRDYPin = drdyPin;
    adsDRDYPinPort = drdyPort;
}

void ads127_bsp_enable_drdy(void){
    if(adsDRDY_IRQn >= 0) NVIC_EnableIRQ(adsDRDY_IRQn);
}

void ads127_bsp_disable_drdy(void){
    if(adsDRDY_IRQn >= 0) NVIC_DisableIRQ(adsDRDY_IRQn);
}

void ads127_bsp_start_pin_initial(GPIO_TypeDef * startPort, int32_t startPin){
    if((startPort != NULL) && (startPin >= 0)){
        GPIO_InitTypeDef GPIO_InitStructure = {
                .Alternate = 0,
                .Mode = GPIO_MODE_OUTPUT_PP,
                .Pin = startPin,
                .Pull = LL_GPIO_PULL_DOWN,
                .Speed = GPIO_SPEED_LOW,
        };
        HAL_GPIO_Init(startPort, &GPIO_InitStructure);
        HAL_GPIO_WritePin(startPort, startPin, 0);
        adsStartPinPort = startPort;
        adsStartPin = startPin;
    }
}

void ads127_bsp_start(void){
    if((adsStartPinPort != NULL) && (adsStartPin >= 0)){
        HAL_GPIO_WritePin(adsStartPinPort, adsStartPin, 1);
    }
}

void ads127_bsp_stop(void){
    if((adsStartPinPort != NULL) && (adsStartPin >= 0)){
        HAL_GPIO_WritePin(adsStartPinPort, adsStartPin, 0);
    }
}


void ads127_bsp_reset_pin_initial(GPIO_TypeDef * resetPort, int32_t resetPin){
    if((resetPort != NULL) && (resetPin >= 0)){
        GPIO_InitTypeDef GPIO_InitStructure = {
                .Alternate = 0,
                .Mode = GPIO_MODE_OUTPUT_PP,
                .Pin = resetPin,
                .Pull = LL_GPIO_PULL_DOWN,
                .Speed = GPIO_SPEED_LOW,
        };
        HAL_GPIO_Init(resetPort, &GPIO_InitStructure);
        HAL_GPIO_WritePin(resetPort, resetPin, 0);
        adsResetPinPort = resetPort;
        adsResetPin = resetPin;
    }
}

void ads127_bsp_reset(void){
    if((adsResetPinPort != NULL) && (adsResetPin >= 0)){
        HAL_GPIO_WritePin(adsResetPinPort, adsResetPin, 0);
        HAL_Delay(1000);
        HAL_GPIO_WritePin(adsResetPinPort, adsResetPin, 1);
        HAL_Delay(1000);
    }
}


#if(0)
#include "ff.h"
#endif

int ads127_data_stream_write(void *fn, void *ctx){
    if(fn == NULL) return 1;
    if(ads_read_ctrl.action.save == 0) return 2;



}

#endif
