/**
 ******************************************************************************
  * @file    bsp_driver_sd.h (based on stm32l4r9i_eval_sd.h)
  * @brief   This file contains the common defines and functions prototypes for
  *          the bsp_driver_sd.c driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L4_SD_H
#define __STM32L4_SD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

//#include "stdint.h"
#include "stm32l4xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief SD Card information structure
  */
#ifndef BSP_SD_CardInfo
#define BSP_SD_CardInfo HAL_SD_CardInfoTypeDef
#endif
/* Exported constants --------------------------------------------------------*/
/**
  * @brief  SD status structure definition
  */
#define   MSD_OK                        ((uint8_t)0x00)
#define   MSD_ERROR                     ((uint8_t)0x01)
#define   MSD_ERROR_SD_NOT_PRESENT      ((uint8_t)0x02)

/**
  * @brief  SD transfer state definition
  */
#define   SD_TRANSFER_OK                ((uint8_t)0x00)
#define   SD_TRANSFER_BUSY              ((uint8_t)0x01)
#define   SD_TRANSFER_ERROR             ((uint8_t)0x02)

#define SD_DATATIMEOUT           ((uint32_t)100000000)

#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)

#ifdef OLD_API
/* kept to avoid issue when migrating old projects. */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
#else
/* USER CODE BEGIN BSP_H_CODE */

#define SDMMC_TRANSFER_USE_DMA              1   // 1: 使用DMA传输   0: 不使用DMA传输

#if(SDMMC_TRANSFER_USE_DMA == 1)
#define SDMMC_SHARE_DMA                     1   // 1: SDMMC发送与接受使用同一个DMA通道    0: SDMMC发送接收使用不同DMA通道
#endif

#define SD_PRESENT               ((uint8_t)0x01)  /* also in bsp_driver_sd.h */
#define SD_NOT_PRESENT           ((uint8_t)0x00)  /* also in bsp_driver_sd.h */
#define SD_DETECT_PIN            SD_CD_Pin      // SD卡插入检测管脚
#define SD_DETECT_GPIO_PORT      SD_CD_GPIO_Port
#define __SD_DETECT_GPIO_CLK_ENABLE()    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

/* DMA definitions for SD DMA transfer */
#define __DMAx_TxRx_CLK_ENABLE            __HAL_RCC_DMA2_CLK_ENABLE

#if(SDMMC_SHARE_DMA == 1)
#define SD_DMAx_Tx_STREAM                 DMA2_Channel4
#else
#define SD_DMAx_Tx_STREAM                 DMA2_Channel5
#endif
#define SD_DMAx_Rx_STREAM                 DMA2_Channel4

#if(SDMMC_SHARE_DMA == 1)
#define SD_DMAx_Tx_IRQn                   DMA2_Channel4_IRQn
#else
#define SD_DMAx_Tx_IRQn                   DMA2_Channel5_IRQn
#endif
#define SD_DMAx_Rx_IRQn                   DMA2_Channel4_IRQn

//#define BSP_SD_DMA_Tx_IRQHandler          DMA2_Channel5_IRQHandler
//#define BSP_SD_DMA_Rx_IRQHandler          DMA2_Channel4_IRQHandler
//#define BSP_SD_IRQHandler                 SDMMC1_IRQHandler


/* Exported functions --------------------------------------------------------*/
uint8_t BSP_SD_Init(void);
uint8_t BSP_SD_DeInit(void);
/**
 *
 * @param in_div
 *              >= 0, SDMMC_CK = SMDMMC_CLK / (in_div + 2)
 *              < 0, 将根据总线频率更改
 * @return
 */
uint8_t BSP_SD_ChangeSpeed(int16_t in_div);

void BSP_SD_Enable(void);
void BSP_SD_Disable(void);

uint8_t BSP_SD_ITConfig(void);
void    BSP_SD_DetectIT(void);
void    BSP_SD_DetectCallback(void);
uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks);
uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks);
uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr);
uint8_t BSP_SD_GetCardState(void);
void    BSP_SD_GetCardInfo(BSP_SD_CardInfo *CardInfo);
uint8_t BSP_SD_IsDetected(void);

#if(SDMMC_TRANSFER_USE_DMA)
void    sdmmc1_irq_callback(void);
void    sdmmc1_dma_txcplt_callback(void);
void    sdmmc1_dma_rxcplt_callback(void);

//void    BSP_SD_IRQHandler(void);
//void    BSP_SD_DMA_Tx_IRQHandler(void);
//void    BSP_SD_DMA_Rx_IRQHandler(void);
#endif


/* USER CODE END BSP_H_CODE */
#endif
/* USER CODE BEGIN CallBacksSection_H */
/* These __weak functions can be surcharged by application code in case the current settings
   (eg. interrupt priority, callbacks implementation) need to be changed for specific application needs */
void    BSP_SD_AbortCallback(void);
void    BSP_SD_WriteCpltCallback(void);
void    BSP_SD_ReadCpltCallback(void);
/* USER CODE END CallBacksSection_H */
#ifdef __cplusplus
}
#endif

#endif /* __STM32L4_SD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
