/*
 * bsp_ads127.c
 *
 *  Created on: 2020年12月4日
 *      Author: XIAOSENLUO
 */

#if(1)


#include "stm32l4xx_it.h"
#include "stm32l4xx_ll_gpio.h"

#include "ads127_driver.h"
#include "bsp_ads127.h"

static uint32_t ads127_bsp_spi_transmit_receive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t timeout){
    uint32_t txallowed = 1U, sr = 0UL, err = 0;

//    __HAL_RCC_SPI1_CLK_ENABLE();

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
    while(((hspi->Instance->SR & SPI_SR_FTLVL) != SPI_FTLVL_EMPTY) && timeout){
        timeout--;
    }

    /// 检查 BSY 标志位
    while (((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY) ? SET : RESET) != RESET) && timeout){
        timeout--;
    }

    /// 检查 接收 FRLVL
    while(((hspi->Instance->SR & SPI_SR_FRLVL) != SPI_FRLVL_EMPTY) && timeout){
        READ_REG(*((__IO uint8_t *)&hspi->Instance->DR));
        timeout--;
    }
    if(hspi->ErrorCode || (!timeout)){
        err = 1;
    }
    sr = hspi->Instance->SR;
    if(sr & SPI_FLAG_MODF) __HAL_SPI_CLEAR_MODFFLAG(hspi);
    if(sr & SPI_FLAG_OVR) __HAL_SPI_CLEAR_OVRFLAG(hspi);

//    __HAL_RCC_SPI1_CLK_DISABLE();
    return err;
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
    uint64_t current_counter;
    union{
        struct{
            uint8_t save : 1;
            uint8_t user_read : 1;
            uint8_t swap : 1;
        };
        uint8_t val;
    }action;
}ads_read_ctrl_t;


#define ADS_DATA_CACHE_OFFSET                   (32)
#define ADS_DATA_CACHE_SIZE                     ((4 * 8000) + (ADS_DATA_CACHE_OFFSET))

static uint8_t ads_data_cache1[ADS_DATA_CACHE_SIZE] __attribute__((section(".ram2"))) = { 0 };
static uint8_t ads_data_cache2[ADS_DATA_CACHE_SIZE] __attribute__((section(".ram1"))) = { 0 };

static ads_read_ctrl_t ads_read_ctrl = {
        .count.val = 0,
        .data = {
            .ptr.user = ads_data_cache1,
            .ptr.isr = ads_data_cache2,
            .ptr.length = 0,
            .index = ADS_DATA_CACHE_OFFSET,
            .limit.size = ADS_DATA_CACHE_SIZE,
            .limit.step = 4,
        },
        .current_counter  =0,
        .action.val = 0,
};

static ads_file_header_t *file_header = NULL;

void ads127_bsp_read_init(const ads_data_init_t* init){
    uint8_t step = 4;

    step -= init->config.cs_enb;
    ads_read_ctrl.data.limit.step = step;
    ads_read_ctrl.data.index = ADS_DATA_CACHE_OFFSET;
    ads_read_ctrl.count.response = (init->crate == UINT32_MAX) ? 0 : (init->clk / init->osr / init->crate) - 1;
    ads_read_ctrl.current_counter  = 0;
    ads_read_ctrl.count.counter = 0;
    ads_read_ctrl.action.val = 0;
}

void swap_cache_address(ads_read_ctrl_t * ctrl){
    uint8_t *tmp = ctrl->data.ptr.isr;
    ads_file_header_t *header = (ads_file_header_handle)ctrl->data.ptr.isr;;

    if(ctrl->action.user_read){
        ctrl->data.index = ADS_DATA_CACHE_OFFSET;   /// 重置数据指针索引
        return;  /// 用户正在读取, 禁止交换地址
    }

    ctrl->action.swap = 1;
    ctrl->action.save = 0;

    ctrl->data.ptr.length = ctrl->data.index;    /// 数据长度
    ctrl->data.index = ADS_DATA_CACHE_OFFSET;             /// 重置数据指针索引

    header->counter += 1;                                /// 记录采样缓冲索引
    header->length = ctrl->data.ptr.length;              /// 记录采样数据长度, 此信息会写入文件头

    ctrl->data.ptr.isr = ctrl->data.ptr.user;
    ctrl->data.ptr.user = tmp;

    ctrl->action.swap = 0;
    ctrl->action.save = 1;  /// 可以保存数据了, 此时用户应该读取 ptr->user指向的内存, 大小为 ptr.length
}

void ads127_bsp_read_data_from_isr(void * ctx){
    SPI_HandleTypeDef *spi_handle = (SPI_HandleTypeDef *)ctx;

    if(ads_read_ctrl.count.counter != ads_read_ctrl.count.response){
        ads_read_ctrl.count.counter += 1;
        return;  /// 未达到响应条件, 返回
    }
    ads_file_header_t *header = (ads_file_header_handle)ads_read_ctrl.data.ptr.isr;

    ads_read_ctrl.current_counter += 1;
    ads_read_ctrl.count.counter = 0;  /// 重置计数器
    if(spi_handle == NULL) return;
    if((ads_read_ctrl.data.index + ads_read_ctrl.data.limit.step) > ads_read_ctrl.data.limit.size){  /// 检查剩余空间
        /// 剩余空间不够, 交换 cache
        swap_cache_address(&ads_read_ctrl);
    }

    if(ads_read_ctrl.data.index == ADS_DATA_CACHE_OFFSET){
        struct tm time = { 0 };
        st_rtc_get_time(&time);   /// 获取首次时间
        st_rtc_time_convert(&header->time, &time);
        header->time.time.msecond = st_rtc_get_subsecond();
    }

    uint8_t tx[4] = {0, 0, 0, 0};  //// 无需发送命令, 当 DRDY 变低时, 采样数据已经移入移位寄存器中

    ads127_cs_set_level(0);
#if(0)
    HAL_SPI_TransmitReceive_DMA(ads_spi_handle, tx, &ads_read_ctrl.data.ptr.isr[ads_read_ctrl.data.index], ads_read_ctrl.data.limit.step);   /// 非阻塞读取
    __HAL_DMA_DISABLE_IT(ads_spi_handle->hdmatx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(ads_spi_handle->hdmarx, DMA_IT_HT);
#else
    ads127_bsp_spi_transmit_receive(spi_handle, tx, &ads_read_ctrl.data.ptr.isr[ads_read_ctrl.data.index], ads_read_ctrl.data.limit.step, 80000000);   /// 阻塞读取
    ads_read_ctrl.data.index += ads_read_ctrl.data.limit.step;
    ads127_cs_set_level(1);
#endif
}

__weak void ads127_bsp_read_data_cplt_callback(void){
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
            .Pin = 1UL << drdyPin,
            .Pull = LL_GPIO_PULL_UP,
            .Speed = GPIO_SPEED_LOW,
    };
    if(drdyPort && (drdyPin >= 0)){
        HAL_GPIO_Init(drdyPort, &GPIO_InitStructure);

        int gpio = (gpio_num_t)drdyPin;
        adsDRDY_IRQn = gpio_get_irqn(gpio);
        ll_gpio_exti_isr_install((gpio_num_t)gpio, fn, ctx);
        HAL_NVIC_SetPriority(adsDRDY_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(adsDRDY_IRQn);
    }
    adsDRDYPin = 1UL << drdyPin;
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
                .Pin = 1UL << startPin,
                .Pull = LL_GPIO_PULL_DOWN,
                .Speed = GPIO_SPEED_LOW,
        };
        HAL_GPIO_Init(startPort, &GPIO_InitStructure);
        HAL_GPIO_WritePin(startPort, 1UL << startPin, 0);
        adsStartPinPort = startPort;
        adsStartPin = 1UL << startPin;
    }
}

void ads127_bsp_start(void){
    if((adsStartPinPort != NULL) && (adsStartPin >= 0)){
//        HAL_GPIO_WritePin(adsStartPinPort, adsStartPin, 1);
        LL_GPIO_SetOutputPin(adsStartPinPort, adsStartPin);
    }
}

void ads127_bsp_stop(void){
    if((adsStartPinPort != NULL) && (adsStartPin >= 0)){
//        HAL_GPIO_WritePin(adsStartPinPort, adsStartPin, 0);
        LL_GPIO_ResetOutputPin(adsStartPinPort, adsStartPin);
    }
}


void ads127_bsp_reset_pin_initial(GPIO_TypeDef * resetPort, int32_t resetPin){
    if((resetPort != NULL) && (resetPin >= 0)){
        GPIO_InitTypeDef GPIO_InitStructure = {
                .Alternate = 0,
                .Mode = GPIO_MODE_OUTPUT_PP,
                .Pin = 1UL << resetPin,
                .Pull = LL_GPIO_PULL_DOWN,
                .Speed = GPIO_SPEED_LOW,
        };
        HAL_GPIO_Init(resetPort, &GPIO_InitStructure);
        HAL_GPIO_WritePin(resetPort, 1UL << resetPin, 0);
        adsResetPinPort = resetPort;
        adsResetPin = 1UL << resetPin;
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

uint32_t ads127_bsp_availablle(void){
    if(ads_read_ctrl.action.save && !ads_read_ctrl.action.swap){
        return ads_read_ctrl.data.ptr.length;
    }else{
        return 0;
    }
}

#if(1)
#include "ff.h"

uint32_t ads127_bsp_write_file(void *file){
    FIL* file_ptr = (FIL*)(file);
    uint16_t write = 0;
    int err = 0;
    if(ads_read_ctrl.action.swap || !ads_read_ctrl.action.save) return 0;
    ads_read_ctrl.action.user_read = 1;   /// 标记读标志位
    ads_read_ctrl.action.save = 0;        /// 清除保存标志位
    err = f_write(file_ptr, ads_read_ctrl.data.ptr.user, ads_read_ctrl.data.ptr.length, &write);
    ads_read_ctrl.action.user_read = 0;
    ads_read_ctrl.action.save = 0;        /// 清除保存标志位
    if(err == FR_OK){
        err = f_sync(file_ptr);
    }
#if(0)
    for(int i = 0; i < (ADS_DATA_CACHE_OFFSET / 4); i++){
        *(uint32_t*)(ads_read_ctrl.data.ptr.user + i * 4) = 0;
    }
#endif
    return write;
}

#endif

#endif
