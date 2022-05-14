/*
 * gps.c
 *
 *  Created on: 2020年11月21日
 *      Author: XIAOSENLUO
 */

#include "gps.h"
#include <stdarg.h>

#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_it.h"



extern void HAL_Delay(uint32_t Delay);


#define gps_delay_ms(ms)    HAL_Delay(ms)

#define GPS_ACK_TIME        500

//static char gps_sentence_buffer[MINMEA_MAX_LENGTH];
static void config_string(char* ptr, size_t cnt, const char* format, ...);

static uint8_t gps_driver_is_init = 0;

static gps_drv_t gps_drv = {.write = NULL, .read = NULL};

static uint8_t gps_pps_staus = 0;

static gps_time_t gps_time = {0};
static volatile int8_t ant_statue = -1;
static volatile int8_t gps_time_zone = 0;
static gps_gll_t gps_gll = {0};

static void config_string(char* ptr, size_t cnt, const char* format, ...){
	uint8_t cs, n;
	va_list arp;

	va_start(arp, format);
	memset(ptr, 0, cnt);
	n = vsnprintf(ptr, cnt, format, arp);
	va_end(arp);
	cs = minmea_checksum(ptr);
	snprintf(&ptr[n], (cnt - n), "%X\r\n", cs);
}


void gps_drv_init(void* func_write, void* func_read){
	if(func_write != NULL) gps_drv.write = func_write;
	if(func_read != NULL) gps_drv.read = func_read;
	gps_driver_is_init = 1;
}

/**
 * 将当前配置信息保存到 FLASH 中， 即使接收机完全断电， FLASH 中的信息不丢失
 */
void gps_save_config_to_flash(void){
	if(!gps_driver_is_init) return;

    char pcas00[16];
    config_string(pcas00, sizeof(pcas00), "$PCAS00*");
    gps_drv.write((uint8_t*)pcas00, (uint32_t)strlen(pcas00));
    gps_delay_ms(GPS_ACK_TIME);
}


void gps_restart(uint8_t in_restart_mode){
	if(!gps_driver_is_init) return;
	char pcas[24];

    config_string(pcas, sizeof(pcas), "$PCAS10,%d*", in_restart_mode);
    gps_drv.write((uint8_t*)pcas, strlen(pcas));
    gps_delay_ms(GPS_ACK_TIME);
}

void gps_set_baudrate(uint32_t in_baudrate){
	char pcas[64];
	uint8_t _baudrate_index  = 1;

	switch(in_baudrate){
	case 4800:
		_baudrate_index = 0;
		break;
	case 9600:
		_baudrate_index = 1;
		break;
	case 19200:
		_baudrate_index = 2;
		break;
	case 38400:
		_baudrate_index = 3;
		break;
	case 57600:
		_baudrate_index = 4;
		break;
	case 115200:
		_baudrate_index = 5;
		break;
	default:
		_baudrate_index = 1;
		break;
	}
	config_string(pcas, sizeof(pcas), "$PCAS01,%d*", _baudrate_index);
	gps_drv.write((uint8_t*)pcas, strlen(pcas));
	gps_delay_ms(GPS_ACK_TIME);
}


/**
 * 配置GPS
 */
void gps_config(gps_config_t* gps_cfg){
	if(!gps_driver_is_init) return;
    char pcas[64];
    uint8_t ret = 0;
//    size_t s = sizeof(pcas);
    ret = ret;
    config_string(pcas, sizeof(pcas), "$PCAS01,%d*", gps_cfg->baudrate);  // 配置波特率
    ret = gps_drv.write((uint8_t*)pcas, strlen(pcas));
    gps_delay_ms(GPS_ACK_TIME);
    config_string(pcas, sizeof(pcas), "$PCAS02,%d*", gps_cfg->up_rate);   // 设置定位更新频率
    ret = gps_drv.write((uint8_t*)pcas, strlen(pcas));
    gps_delay_ms(GPS_ACK_TIME);
    if(gps_cfg->control_output == NULL){
    	config_string(pcas, sizeof(pcas), "$PCAS03,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,,,%d,%d,,,,%d*",
    			1,1,0,
				0,0,0,
				1,1,0,
				0,0,0,
				0);
    }else{
        config_string(pcas, sizeof(pcas), "$PCAS03,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,,,%d,%d,,,,%d*",
        		gps_cfg->control_output->nGGA, gps_cfg->control_output->nGLL, gps_cfg->control_output->nGSA,
    			gps_cfg->control_output->nGSV, gps_cfg->control_output->nRMC, gps_cfg->control_output->nVTG,
    			gps_cfg->control_output->nZDA, gps_cfg->control_output->nANT, gps_cfg->control_output->nDHV,
    			gps_cfg->control_output->nLPS, gps_cfg->control_output->nUTC, gps_cfg->control_output->nGST,
    			gps_cfg->control_output->nTIM
    			); // 设置输出语句, 顺序不可改变
    }
    ret = gps_drv.write((uint8_t*)pcas, strlen(pcas));
    gps_delay_ms(GPS_ACK_TIME);
    config_string(pcas, sizeof(pcas), "$PCAS04,%d*", gps_cfg->gps_mode);
    ret = gps_drv.write((uint8_t*)pcas, strlen(pcas));
    gps_delay_ms(GPS_ACK_TIME);
//    config_string(pcas, sizeof(pcas), "$PCAS05,%d*", gps_cfg->rev);
//    gps_drv.write(pcas, strlen(pcas));
//
//    config_string(pcas, sizeof(pcas), "$PCAS06,%d*", gps_cfg->info);
//    gps_drv.write(pcas, strlen(pcas));
}


int8_t gps_get_time_zone(void){
	return gps_time_zone;
}

bool gps_parse_gll(gps_gll_t *frame, const char* str){
	bool ret = false;

	if(str[0] != '$') return false;

    if(strncmp((const char*)(str + 3), "GLL", 3) == 0){
        ret = minmea_parse_gll((struct minmea_sentence_gll*)frame, str);
    }
    return ret;
}

int8_t gps_parse_time_zone(gps_gll_t *frame){
    int8_t _time_zone = 0;
    int8_t _offset = 0;

    int32_t _ulon = (int32_t)frame->longitude.value / frame->longitude.scale / 100;
    _time_zone = (int8_t)(_ulon / 15);
    _offset = (_ulon % 15);

    if((_offset * 10) > 75){
    	_time_zone += ((frame->longitude.value > 0) ? 1 : -1);
    }
    return _time_zone;
}


bool gps_parse_zda(gps_time_t *frame, const char* str){
	bool ret = false;

	if(str[0] != '$') return false;

    if(strncmp((const char*)(str + 3), "ZDA", 3) == 0){
        ret = minmea_parse_zda((struct minmea_sentence_zda*)frame, str);
    }
    return ret;
}

bool gps_parse_ant(const char* str){
    char type[6] = {0};
    char info[16] = {0};
    int xx = 0, yy = 0, zz = 0;

    if(str[0] != '$') return false;
    if(strncmp((const char*)(str + 3), "TXT", 3) != 0) return false;

    if(!minmea_scan(str, "tiiis",
            type,
            &xx,
            &yy,
            &zz,
             info))
        return false;
    if (strcmp(info, "ANTENNA OK"))
        return false;
    return true;
}

uint8_t gps_check_ant(void){
    return (uint8_t)ant_statue;
}

gps_time_t gps_get_time(void){
	return gps_time;
}

gps_gll_t gps_get_gll(void){
	return gps_gll;
}

/***************************************************** GPS UART ***************************************************/
#define UART_GPS_DMA_RX
#define GPS_DATA_BUFFER_SIZE           80
#define GPS_DMA_CIRCULAR_BUFFER_SIZE   48

static uint8_t gps_buffer_for_uart[GPS_DATA_BUFFER_SIZE];
static uint8_t gps_buffer_for_user[GPS_DATA_BUFFER_SIZE];

static uint8_t* gps_ptr_for_user = gps_buffer_for_user;
static uint8_t* gps_ptr_for_uart = gps_buffer_for_uart;

static __IO uint8_t gps_data_ready = 0;
static __IO uint16_t gps_data_index = 0;
static volatile int gps_sentence_start = -1, gps_sentence_end = -1;

static uint8_t gps_dma_circular_buffer[GPS_DMA_CIRCULAR_BUFFER_SIZE] = {0};
static __IO uint8_t gps_sentence_lock = 0;

static uint8_t _uart4_tx_dma_config(uint8_t *_ptr, uint16_t _s){
	uint8_t ret = 0;

	// DMA2 Channel 3; request 2
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

	/* (3) Configure the DMA functional parameters for transmission */
	LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_3);   // 配置参数要关闭 dma 通道
	LL_DMA_ConfigTransfer(DMA2, LL_DMA_CHANNEL_3,
						LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
						LL_DMA_PRIORITY_HIGH              |
						LL_DMA_MODE_NORMAL                |
						LL_DMA_PERIPH_NOINCREMENT         |
						LL_DMA_MEMORY_INCREMENT           |
						LL_DMA_PDATAALIGN_BYTE            |
						LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_3,
						 (uint32_t)_ptr,
						 LL_USART_DMA_GetRegAddr(UART4, LL_USART_DMA_REG_DATA_TRANSMIT),
						 LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_3, _s);
	if(LL_DMA_GetPeriphRequest(DMA2, LL_DMA_CHANNEL_3) != LL_DMA_REQUEST_2)
		LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_2);
	/* (5) Enable DMA transfer complete/error interrupts  */
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_3);
	LL_DMA_EnableIT_TE(DMA2, LL_DMA_CHANNEL_3);
	LL_DMA_ClearFlag_GI3(DMA2);
	HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 3, 0);
	NVIC_EnableIRQ(DMA2_Channel3_IRQn);
	return ret;
}

/**
 *  循环接收模式
 */
static uint8_t _uart4_rx_dma_config(uint8_t *_ptr, uint16_t _s){
	uint8_t ret = 0;

	// DMA2 Channel 3; request 2
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

	/* (3) Configure the DMA functional parameters for transmission */
	LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_5);   // 配置参数要关闭 dma 通道
	LL_DMA_ConfigTransfer(DMA2, LL_DMA_CHANNEL_5,
			            LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
						LL_DMA_PRIORITY_HIGH              |
						LL_DMA_MODE_CIRCULAR                |
						LL_DMA_PERIPH_NOINCREMENT         |
						LL_DMA_MEMORY_INCREMENT           |
						LL_DMA_PDATAALIGN_BYTE            |
						LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_5,
			            LL_USART_DMA_GetRegAddr(UART4, LL_USART_DMA_REG_DATA_RECEIVE),
						(uint32_t)_ptr,
						LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_5, _s);
	if(LL_DMA_GetPeriphRequest(DMA2, LL_DMA_CHANNEL_5) != LL_DMA_REQUEST_2)
		LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_5, LL_DMA_REQUEST_2);
	/* (5) Enable DMA transfer complete/error interrupts  */
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_5);
	LL_DMA_EnableIT_HT(DMA2, LL_DMA_CHANNEL_5);
	LL_DMA_EnableIT_TE(DMA2, LL_DMA_CHANNEL_5);
	LL_DMA_ClearFlag_GI5(DMA2);
	HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 2, 0);
	NVIC_EnableIRQ(DMA2_Channel5_IRQn);
	return ret;
}

uint8_t gps_uart_transfer(uint8_t* ptr, uint16_t s){
	uint8_t ret = 0;
	uint16_t i = 0;
#if(UART4_USE_DMA)

#else

#endif
    while(i < s){
    	while(!LL_USART_IsActiveFlag_TXE(UART4));
    	if(i == s -1){
    		LL_USART_ClearFlag_TC(UART4);
    	}
    	LL_USART_TransmitData8(UART4, ptr[i++]);
    }
    while(!LL_USART_IsActiveFlag_TC(UART4));
    LL_USART_ClearFlag_TC(UART4);
    return ret;
}


static void _uart_error_handler(USART_TypeDef *USARTx){
	__IO uint32_t isr_reg;
	isr_reg = LL_USART_ReadReg(USARTx, ISR);
	if(isr_reg & LL_USART_ISR_NE){
		LL_USART_ClearFlag_NE(USARTx);
	}else if(isr_reg & LL_USART_ISR_ORE){
		LL_USART_ClearFlag_ORE(USARTx);   // 清除 overrun 标志位
		LL_USART_ReceiveData8(USARTx);    // 丢弃未读数据
	}else if(isr_reg & LL_USART_ISR_FE){
        LL_USART_ClearFlag_FE(USARTx);
	}
}

static void uart4_rx_idle_handler(void){
	int i = 0;
	uint32_t cnt = 0;
    uint8_t *ptr = NULL;

	cnt = GPS_DMA_CIRCULAR_BUFFER_SIZE - LL_DMA_GetDataLength(DMA2, LL_DMA_CHANNEL_5);  // 获取已经传输长度
	if((cnt == GPS_DMA_CIRCULAR_BUFFER_SIZE) || (cnt == 0) || (cnt == (GPS_DMA_CIRCULAR_BUFFER_SIZE >> 1))) return;  // 传输完成或者没有传输或者完成传输一半
	if(cnt < (GPS_DMA_CIRCULAR_BUFFER_SIZE >> 1)) ptr = gps_dma_circular_buffer;  // 传输小于一半
	if(cnt > (GPS_DMA_CIRCULAR_BUFFER_SIZE >> 1)){
		ptr = &gps_dma_circular_buffer[GPS_DMA_CIRCULAR_BUFFER_SIZE >> 1]; // 传输大于一半
		cnt -= (GPS_DMA_CIRCULAR_BUFFER_SIZE >> 1); // 仅处理大于的部分(已经触发HT中断)
	}

    for(i = 0; i < cnt; i++){
    	if((gps_sentence_start < 0) && (ptr[i] != '$')) continue;
    	if(ptr[i] == '$'){  // 语句开始标志
    		gps_sentence_start = i;
    		gps_ptr_for_uart[gps_data_index++] = ptr[i];
    		continue;
    	}
    	if((gps_sentence_start >= 0) && (ptr[i] == '\n')){   // 语句开始并且有结束标志
    		gps_sentence_end = i;
    		gps_ptr_for_uart[gps_data_index++] = ptr[i];
    		gps_ptr_for_uart[gps_data_index] = '\0';

    		ant_statue = gps_parse_ant((const char*)gps_ptr_for_uart);  // 检查天线状态
    		if(gps_parse_zda(&gps_time, (const char*)gps_ptr_for_uart)) goto swap_section;  // 解析 GPS ZDA, 得到时间
            if(gps_parse_gll(&gps_gll, (const char*)gps_ptr_for_uart)) gps_time_zone = gps_parse_time_zone(&gps_gll);

        swap_section:
    		gps_uart_swap_buffer();  // 交换缓冲区地址

    		gps_data_index = 0;
    		gps_sentence_start = -1;
    		gps_sentence_end = -1;
    		continue;
    	}
        if((gps_sentence_start >= 0) && (gps_sentence_end < 0)) gps_ptr_for_uart[gps_data_index++] = ptr[i];  // 语句开始但是没有结束标志
    }

    memset(ptr, '\0', cnt);
}

#if(0)
void UART4_IRQHandler(void){

	if(LL_USART_IsActiveFlag_RXNE(UART4) && LL_USART_IsEnabledIT_RXNE(UART4)){
//		uart4_gps_rx_callback();
	}else{
		_uart_error_handler(UART4);
	}
	if(LL_USART_IsActiveFlag_IDLE(UART4)){
		LL_USART_ClearFlag_IDLE(UART4);
		uart4_rx_idle_handler();
	}
}
#endif

//void UART4_DMA_Handler(void){
void uart4_dma_txcplt_callback(void){
	if(LL_DMA_IsActiveFlag_TC3(DMA2)){
	    LL_DMA_ClearFlag_GI3(DMA2);
	/* Call function Transmission complete Callback */

	}else if(LL_DMA_IsActiveFlag_TE3(DMA2)){
		LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_3);
	}
}


void gps_uart_swap_buffer(void){
   uint8_t* tmp = NULL;

   if(gps_sentence_lock) return;
   gps_sentence_lock = 1;
   tmp = gps_ptr_for_uart;
   gps_ptr_for_uart = gps_ptr_for_user;
   gps_ptr_for_user = tmp;

   gps_data_ready = 1;
   gps_sentence_lock = 0;
}

bool gps_sentence_is_ready(void){
	return gps_data_ready;
}

void uart4_dma_circular_rx_callback(void){
	uint8_t* ptr = NULL;
	int i = 0;
    if(LL_DMA_IsActiveFlag_HT5(DMA2)){
        LL_DMA_ClearFlag_HT5(DMA2);
        ptr = gps_dma_circular_buffer;
    }
    if(LL_DMA_IsActiveFlag_TC5(DMA2)){
        LL_DMA_ClearFlag_TC5(DMA2);
        LL_DMA_ClearFlag_GI5(DMA2);
        ptr = &gps_dma_circular_buffer[(GPS_DMA_CIRCULAR_BUFFER_SIZE >> 1)];
    }
    if(LL_DMA_IsActiveFlag_TE5(DMA2)){
    	LL_DMA_ClearFlag_TE5(DMA2);

    	LL_USART_Disable(UART4);
    	_uart4_rx_dma_config(gps_dma_circular_buffer, GPS_DMA_CIRCULAR_BUFFER_SIZE);
    	LL_USART_Enable(UART4);
    	LL_USART_ClearFlag_TC(UART4);
    	LL_USART_EnableDMAReq_RX(UART4);
    	LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_5);  // 重新启动
    	return;
    }

    for(i = 0; i < (GPS_DMA_CIRCULAR_BUFFER_SIZE >> 1); i++){
    	if((gps_sentence_start < 0) && (ptr[i] != '$')) continue;
    	if(ptr[i] == '$'){  // 语句开始标志
    		gps_sentence_start = i;
    		gps_ptr_for_uart[gps_data_index++] = ptr[i];
    		continue;
    	}
    	if((gps_sentence_start >= 0) && (ptr[i] == '\n')){   // 语句开始并且有结束标志
    		gps_sentence_end = i;
    		gps_ptr_for_uart[gps_data_index++] = ptr[i];
    		gps_ptr_for_uart[gps_data_index] = '\0';

    		ant_statue = gps_parse_ant((const char*)gps_ptr_for_uart);  // 检查天线状态
    		if(gps_parse_zda(&gps_time, (const char*)gps_ptr_for_uart)) goto swap_section;  // 解析 GPS ZDA, 得到时间
            if(gps_parse_gll(&gps_gll, (const char*)gps_ptr_for_uart)) gps_time_zone = gps_parse_time_zone(&gps_gll);

        swap_section:
    		gps_uart_swap_buffer();  // 交换缓冲区地址

    		gps_data_index = 0;
    		gps_sentence_start = -1;
    		gps_sentence_end = -1;

    		continue;
    	}
        if((gps_sentence_start >= 0) && (gps_sentence_end < 0)) gps_ptr_for_uart[gps_data_index++] = ptr[i];  // 语句开始但是没有结束标志
    }
    memset(ptr, '\0', (GPS_DMA_CIRCULAR_BUFFER_SIZE >> 1));
}

int gps_uart_get_sentence(uint8_t *ptr, uint16_t s){
	if(gps_sentence_lock) return 2;
	if(gps_data_ready == 0) return 1;
	gps_sentence_lock = 1;
	strcpy((char*)ptr, (const char*)gps_ptr_for_user);
	gps_data_ready = 0;
	gps_sentence_lock = 0;
	return 0;
}


void gps_uart_init(uint32_t _baudrate){

	LL_USART_InitTypeDef USART_InitStruct = {0};
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	LL_USART_Disable(UART4);

	LL_USART_DeInit(UART4);

	LL_RCC_SetUARTClockSource(LL_RCC_UART4_CLKSOURCE_PCLK1);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    /**UART4 GPIO Configuration
    PA0     ------> UART4_TX
    PA1     ------> UART4_RX
    */
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
	  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  USART_InitStruct.BaudRate = _baudrate;
	  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	  LL_USART_Init(UART4, &USART_InitStruct);

	  LL_USART_ConfigAsyncMode(UART4);
	  LL_USART_Enable(UART4);
	  /* Polling USART initialisation */
	while((!(LL_USART_IsActiveFlag_TEACK(UART4))) || (!(LL_USART_IsActiveFlag_REACK(UART4)))){};
//#ifndef UART_GPS_DMA_RX
//	if(!LL_USART_IsEnabledIT_RXNE(UART4)){
//		LL_USART_EnableIT_RXNE(UART4);
//	//    	LL_USART_ReceiveData8(UART4);
//	}
//#endif
	if(!LL_USART_IsEnabledIT_ERROR(UART4)){
		LL_USART_EnableIT_ERROR(UART4);
	}
	if(!LL_USART_IsEnabledIT_IDLE(UART4)){
		LL_USART_EnableIT_IDLE(UART4);
		LL_USART_ClearFlag_IDLE(UART4);
	}
	HAL_NVIC_SetPriority(UART4_IRQn, 2, 0);
	NVIC_EnableIRQ(UART4_IRQn);
#ifdef UART_GPS_DMA_RX
	_uart4_rx_dma_config(gps_dma_circular_buffer, GPS_DMA_CIRCULAR_BUFFER_SIZE);
	st_irq_handler_register(DMA2_Channel5_IRQn, uart4_dma_circular_rx_callback);
	LL_USART_ClearFlag_TC(UART4);
	LL_USART_EnableDMAReq_RX(UART4);
	LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_5);
#endif
}


void gps_uart_deinit(void){
	LL_USART_DeInit(UART4);
	LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_UART4);
	LL_USART_Disable(UART4);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);
	NVIC_DisableIRQ(UART4_IRQn);
	NVIC_DisableIRQ(DMA2_Channel5_IRQn);
}
