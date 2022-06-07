/*
 * bsp_ads127.c
 *
 *  Created on: 2020年12月4日
 *      Author: XIAOSENLUO
 */

#if(1)

#include "string.h"
#include "stm32l4xx_it.h"
#include "stm32l4xx_ll_gpio.h"

#include "ads127_driver.h"
#include "ads127.h"
#include "bsp_ads127.h"


void (*can_save_callback)(void*) = NULL;
void *can_save_callback_ctx = NULL;

static uint32_t ads127_bsp_spi_transmit_receive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t timeout){
    uint32_t txallowed = 1U, sr = 0UL, err = 0;

    hspi->pRxBuffPtr  = (uint8_t *)pRxData;
    hspi->RxXferCount = Size;
    hspi->RxXferSize  = Size;
    hspi->pTxBuffPtr  = (uint8_t *)pTxData;
    hspi->TxXferCount = Size;
    hspi->TxXferSize  = Size;

    CLEAR_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
    if((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(hspi);
    }
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
    if(sr & SPI_FLAG_OVR){
        __HAL_SPI_CLEAR_OVRFLAG(hspi);
    }
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
    uint8_t lock;
}ads_data_ptr_t;

typedef struct ads_read_ctrl_s{
#if(0)
    union{
        struct{
            uint8_t counter : 4;
            uint8_t response : 4;
        };
        uint8_t val;
    }count;
#endif
    ads_data_ptr_t data;
    uint64_t current_counter;
    union{
        struct{
            uint8_t save : 1;
        };
        uint8_t val;
    }action;
    ads_file_header_t file_header;
}ads_read_ctrl_t;


static uint8_t ads_data_cache1[ADS_DATA_CACHE_SIZE] __attribute__((section(".ram2"))) = { 0 };
static uint8_t ads_data_cache2[ADS_DATA_CACHE_SIZE] __attribute__((section(".ram1"))) = { 0 };

static ads_read_ctrl_t ads_read_ctrl = {
#if(0)
        .count.val = 0,
#endif
        .data = {
            .ptr = {.user = ads_data_cache1, .isr = ads_data_cache2, .length = 0},
            .index = ADS_DATA_CACHE_OFFSET,
            .limit = {.size = ADS_DATA_CACHE_SIZE, .step = 4},
        },
        .current_counter  =0,
        .action.val = 0,
        .file_header = { 0 , 0, { .time.val = 0, .date.val = 0} },
};


void   ads127_bsp_read_init(const ads_data_init_t* init, void (*fn)(void *), void * ctx){
    ads_read_ctrl.data.limit.step = 4 - init->config.cs_enb;
    ads_read_ctrl.data.index = ADS_DATA_CACHE_OFFSET;
#if(0)
    ads_read_ctrl.count.response = 0;
    ads_read_ctrl.count.counter = 0;
#endif
    ads_read_ctrl.current_counter  = 0;
    ads_read_ctrl.action.val = 0;

    if(fn){
        can_save_callback = fn;
        can_save_callback_ctx = ctx;
    }
}

void swap_cache_address(ads_read_ctrl_t * ctrl){
    uint8_t *tmp = NULL;
    ctrl->file_header.counter += 1;

    if(ctrl->data.lock){
        ctrl->data.index = ADS_DATA_CACHE_OFFSET;   /// 重置数据指针索引
        return;  /// 用户正在读取, 禁止交换地址
    }

    ctrl->data.lock = 1;
    ctrl->action.save = 0;
    tmp = ctrl->data.ptr.isr;
    ctrl->data.ptr.length = ctrl->data.index;            /// 数据长度
    ctrl->file_header.length = ctrl->data.index;
    ctrl->data.index = ADS_DATA_CACHE_OFFSET;             /// 重置数据指针索引
    ctrl->data.ptr.isr = ctrl->data.ptr.user;
    ctrl->data.ptr.user = tmp;
    /// 生成文件头信息
    ads_file_header_t *header = (ads_file_header_handle)ctrl->data.ptr.user;
    header->time = ctrl->file_header.time;
    header->counter = ctrl->file_header.counter;
    header->length = ctrl->file_header.length;

    ctrl->action.save = 1;  /// 可以保存数据了, 此时用户应该读取 ptr->user指向的内存, 大小为 ptr.length
    ctrl->data.lock = 0;
}

void ads127_bsp_read_data_from_isr(void * ctx){
    SPI_HandleTypeDef *spi_handle = (SPI_HandleTypeDef *)ctx;
#if(0)
    if(ads_read_ctrl.count.counter != ads_read_ctrl.count.response){
        ads_read_ctrl.count.counter += 1;
        return;  /// 未达到响应条件, 返回
    }
#endif
    ads_read_ctrl.current_counter += 1;
#if(0)
    ads_read_ctrl.count.counter = 0;  /// 重置计数器
#endif
    if(spi_handle == NULL) return;
    if((ads_read_ctrl.data.index + ads_read_ctrl.data.limit.step) > ads_read_ctrl.data.limit.size){  /// 检查剩余空间
        /// 剩余空间不够, 交换 cache
        swap_cache_address(&ads_read_ctrl);
        if(can_save_callback) can_save_callback(can_save_callback_ctx);
    }

    if(ads_read_ctrl.data.index == ADS_DATA_CACHE_OFFSET){
        st_rtc_get_time_v2(&ads_read_ctrl.file_header.time);   /// 获取首次时间
    }

    uint8_t tx[8] = {0, 0, 0, 0, 0, 0, 0, 0};  //// 无需发送命令, 当 DRDY 变低时, 采样数据已经移入移位寄存器中

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

void ads127_bsp_drdy_pin_initial(GPIO_TypeDef * drdyPort, int32_t drdyPin){
    GPIO_InitTypeDef GPIO_InitStructure = {
            .Alternate = 0,
            .Mode = GPIO_MODE_IT_FALLING,
            .Pin = 1UL << drdyPin,
            .Pull = GPIO_PULLUP,
            .Speed = GPIO_SPEED_LOW,
    };
    if(drdyPort && (drdyPin >= 0)){
        HAL_GPIO_Init(drdyPort, &GPIO_InitStructure);
        int gpio = (gpio_num_t)drdyPin;
        adsDRDY_IRQn = gpio_get_irqn(gpio);

        adsDRDYPin = drdyPin;
        adsDRDYPinPort = drdyPort;
    }
}

void ads127_bsp_drdy_isr_install(GPIO_TypeDef * drdyPort, int32_t drdyPin, ads_drdy_callback_t fn, void *ctx){
    UNUSED(fn);
    GPIO_InitTypeDef GPIO_InitStructure = {
            .Alternate = 0,
            .Mode = GPIO_MODE_IT_FALLING,
            .Pin = 1UL << drdyPin,
            .Pull = GPIO_PULLUP,
            .Speed = GPIO_SPEED_LOW,
    };
    if(drdyPort && (drdyPin >= 0)){
        HAL_GPIO_Init(drdyPort, &GPIO_InitStructure);

        int gpio = (gpio_num_t)drdyPin;
        adsDRDY_IRQn = gpio_get_irqn(gpio);
        ll_gpio_exti_isr_install((gpio_num_t)gpio, &ads127_bsp_read_data_from_isr, ctx);
        HAL_NVIC_SetPriority(adsDRDY_IRQn, 4, 0);
        HAL_NVIC_EnableIRQ(adsDRDY_IRQn);
    }
    adsDRDYPin = drdyPin;
    adsDRDYPinPort = drdyPort;
}

void ads127_bsp_drdy_isr_uninstall(void){
    if(!adsDRDYPinPort && (adsDRDYPin < 0)) return;
    HAL_NVIC_DisableIRQ(adsDRDY_IRQn);
    ll_gpio_exti_isr_uninstall(adsDRDYPin);
    GPIO_InitTypeDef GPIO_InitStructure = {
            .Alternate = 0,
            .Mode = GPIO_MODE_ANALOG,
            .Pin = 1UL << adsDRDYPin,
            .Pull = GPIO_PULLUP,
            .Speed = GPIO_SPEED_LOW,
    };
    HAL_GPIO_Init(adsDRDYPinPort, &GPIO_InitStructure);
    adsDRDYPin = -1;
    adsDRDYPinPort = NULL;
    adsDRDY_IRQn = -255;
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
                .Pull = GPIO_PULLDOWN,
                .Speed = GPIO_SPEED_LOW,
        };
        HAL_GPIO_Init(startPort, &GPIO_InitStructure);
        HAL_GPIO_WritePin(startPort, 1UL << startPin, 0);
        adsStartPinPort = startPort;
        adsStartPin = startPin;
    }
}

void ads127_bsp_start(void){
    if((adsStartPinPort != NULL) && (adsStartPin >= 0)){
        LL_GPIO_SetOutputPin(adsStartPinPort, 1UL << adsStartPin);
    }
}

bool ads127_bsp_is_start(void){
    if((adsStartPinPort != NULL) && (adsStartPin >= 0)){
        return ((LL_GPIO_ReadOutputPort(adsStartPinPort) & (1UL << adsStartPin)) ? true : false);
    }
    return false;
}

void ads127_bsp_stop(void){
    if((adsStartPinPort != NULL) && (adsStartPin >= 0)){
        LL_GPIO_ResetOutputPin(adsStartPinPort, 1UL << adsStartPin);
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
        adsResetPin = resetPin;
    }
}

void ads127_bsp_keep_reset(void){
    if((adsResetPinPort != NULL) && (adsResetPin >= 0)){
        HAL_GPIO_WritePin(adsResetPinPort, 1UL << adsResetPin, 0);
    }
}

void ads127_bsp_reset(void){
    if((adsResetPinPort != NULL) && (adsResetPin >= 0)){
        HAL_GPIO_WritePin(adsResetPinPort, 1UL << adsResetPin, 0);
        HAL_Delay(1000);
        HAL_GPIO_WritePin(adsResetPinPort, 1UL << adsResetPin, 1);
        HAL_Delay(1000);
    }
}

uint32_t ads127_bsp_availablle(void* *ptr){
    if((!ads_read_ctrl.data.lock) && ads_read_ctrl.action.save){
        if(ptr) *ptr = (void *)ads_read_ctrl.data.ptr.user;
        return ads_read_ctrl.data.ptr.length;
    }
    return 0;
}

void ads127_bsp_clear_save_fild(void){
    ads_read_ctrl.action.save = 0;
}

int ads127_bsp_lock_user(void){
    if(ads_read_ctrl.data.lock) return ads_read_ctrl.data.lock;
    ads_read_ctrl.data.lock = 1;
    return 0;
}

int ads127_bsp_unlock_user(void){
    ads_read_ctrl.data.lock = 0;
    return ads_read_ctrl.data.lock;
}

#if(1)
#include "fatfs.h"
#include "string.h"
#include "log.h"

uint32_t ads127_bsp_write_file(void *file){

    uint32_t write = 0;
    int err = 0;
    if(ads_read_ctrl.data.lock || !ads_read_ctrl.action.save) return 0;   /// 检查是否可以写入文件
    ads_read_ctrl.data.lock = 1;   /// 加锁

    err = f_write((FIL*)file, ads_read_ctrl.data.ptr.user, ads_read_ctrl.data.ptr.length, &write);
    if(err == FR_OK){
        f_sync((FIL*)file);
    }else{
        char *err_str = NULL;
        err_str = fs_error_string(NULL, 0, err);
        log_printf("数据文件写入错误[ERROR: %s]\n", err_str);
    }
    ads_read_ctrl.data.ptr.length -= write;
    ads_read_ctrl.action.save = 0;
    ads_read_ctrl.data.lock = 0;
    return err;
}

int ads127_bsp_get_file_header(void *out){
    if(ads_read_ctrl.data.lock) return ads_read_ctrl.data.lock;
    if(out == NULL) return 2;
    memcpy(out, &ads_read_ctrl.file_header, sizeof(ads_read_ctrl.file_header));
    return 0;
}

uint32_t ads127_bsp_create_file(void *file, void *out){
#define HEX "0123456789ABCDEF"
    uint32_t s = 0, err = 0;
    char path[64] = {'\0'};
    rtc_date_time_t time = { 0 };

    if((file == NULL) || (((FIL*)file)->obj.fs)) return FR_INVALID_OBJECT;
    strncpy(path, sd_root_path, 64);
    s = strlen(path);
    if(path[s - 1] != '/') path[s++] = '/';
    for(int i = 0; i < 3; i++){
        uint32_t uid = READ_REG(*((uint32_t *)(UID_BASE + 4UL * i)));
        for(int j = 0; j < 8; j++){
            path[s] = HEX[(uid << (j << 2) & 0xF0000000) >> 28];
            s = s + 1;
        }
    }
    path[s] = '\0';
    err = f_mkdir(path);
    if((err != FR_OK) && (err != FR_EXIST)) return err;
    path[s++] = '/';
    path[s] = '\0';
    st_rtc_get_time_v2(&time);
    rtc_time2str(&time, &path[s], sizeof(path) - s);
    strcat(path, ".bin");
#if(0)
    err = fs_create_file((FIL*)file, path);
#else
    err = f_open((FIL*)file, path, FA_CREATE_ALWAYS | FA_WRITE);
#endif
//    log_printf("%s\n", path);
    if(out) memcpy(out, &time, sizeof(time));
    return err;
#undef HEX
}


uint32_t ads127_bsp_offset_calibration(void){
    uint32_t ofc = 0;
#if(1)
#define CALCULATION_COUNT_SHIFT      4
    uint8_t *ptr = NULL;
    uint8_t config = 0, count = (1UL << CALCULATION_COUNT_SHIFT);
    uint32_t tick = 0, start_tick = 0;
    uint64_t sum = 0;
    ads_data_init_t init = {{0}, {0}, {0}};
    config = ads127_get_configure(NULL);

    init.config.val = config;
    ads127_bsp_read_init(&init, NULL, NULL);
#if(0)
    ads_read_ctrl.count.response = 0;
    ads_read_ctrl.count.counter = 0;
#endif

    ads127_bsp_drdy_isr_install(adsDRDYPinPort, adsDRDYPin, &ads127_bsp_read_data_from_isr, ads127_driver_handle());
    ads127_command_start();
    do{
        sum = 0;

        tick = start_tick = HAL_GetTick();
        do{
            tick = HAL_GetTick();
        }while((ads127_bsp_availablle(&ptr) == 0) && (tick - start_tick < 3000));
        if(tick - start_tick > 3000){
            return 0;
        }
        ads127_bsp_lock_user();
        ptr = (uint8_t *)(ptr + ADS_DATA_CACHE_OFFSET);
        for(int i = 0; i < (ADS_DATA_CACHE_SIZE - ADS_DATA_CACHE_OFFSET);){
            sum += ((((uint32_t)ptr[i]) << 16) | (((uint32_t)ptr[i + 1]) << 8) | (((uint32_t)ptr[i + 2])));
            i += ads_read_ctrl.data.limit.step;
        }
        ads127_bsp_clear_save_fild();
        ads127_bsp_unlock_user();
        ofc += sum / ((ADS_DATA_CACHE_SIZE - ADS_DATA_CACHE_OFFSET) / ads_read_ctrl.data.limit.step);
    }while(--count);
    ads127_command_stop();

    ofc >>= CALCULATION_COUNT_SHIFT;
#undef CALCULATION_COUNT
#endif

    return ofc;
}

uint16_t ads127_bsp_gain_calibration(uint32_t expected_code){   /// TODO 增益校准
    uint16_t fsc = 0x8000;
    uint8_t *ptr = NULL;
    uint8_t config = 0;
    uint32_t tick = 0, start_tick = 0, actual_code = 0;
    uint64_t sum = 0;
    int32_t expected_voltage = 0, actual_voltage = 0;
    float gain_error = 0.000000000000f;

    config = ads127_get_configure(NULL);
    ads_read_ctrl.data.limit.step = 4 - ((config & 0x02) >> 2);
    ads_read_ctrl.data.index = ADS_DATA_CACHE_OFFSET;
#if(0)
    ads_read_ctrl.count.response = 0;
    ads_read_ctrl.count.counter = 0;
#endif
    ads_read_ctrl.current_counter  = 0;
    ads_read_ctrl.action.val = 0;

    ads127_bsp_drdy_isr_install(adsDRDYPinPort, adsDRDYPin, &ads127_bsp_read_data_from_isr, ads127_driver_handle());

    ads127_command_start();
    tick = start_tick = HAL_GetTick();
    do{
        tick = HAL_GetTick();
    }while((ads127_bsp_availablle(&ptr) == 0) && (tick - start_tick < 1000));
    if(tick - start_tick > 1000){
        return fsc;
    }
    ads127_command_stop();
    ads127_bsp_lock_user();
    ptr += ADS_DATA_CACHE_OFFSET;
    for(int i = 0; i < (ADS_DATA_CACHE_SIZE - ADS_DATA_CACHE_OFFSET);){
        sum += (((uint32_t)ptr[i]) << 16) | (((uint32_t)ptr[i + 1]) << 8) | (((uint32_t)ptr[i + 2]));
        i += ads_read_ctrl.data.limit.step;
    }
    ads127_bsp_unlock_user();

    actual_code = sum / ((ADS_DATA_CACHE_SIZE - ADS_DATA_CACHE_OFFSET) >> 2);
    expected_voltage = ((int32_t)(expected_code << 8)) >> 8;
    actual_voltage = ((int32_t)(actual_code << 8)) >> 8;

    gain_error = (float)expected_voltage / (float)actual_voltage;

    fsc = (uint16_t)(gain_error * 0.000030517578f + 0.500000000000f);

    return fsc;
}


#endif

#endif
