//
// Created by XIAOSENLUO on 2022/5/20.
//

#include "ads127_driver.h"
#include "stm32l4xx_ll_gpio.h"


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

void ads127_cs_set_level(uint8_t level){
    if((adsCSPin < 0) || (adsCSPort == NULL)) return;
    if(level){
        LL_GPIO_SetOutputPin(adsCSPort, adsCSPin);
    }else{
        LL_GPIO_ResetOutputPin(adsCSPort, adsCSPin);
    }
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
