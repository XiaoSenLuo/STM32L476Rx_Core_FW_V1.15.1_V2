/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sd_diskio.c
  * @brief   SD Disk I/O driver
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
/* USER CODE END Header */

/* Note: code generation based on sd_diskio_template_bspv1.c v2.1.4
   as "Use dma template" is disabled. */

/* USER CODE BEGIN firstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END firstSection*/

/* Includes ------------------------------------------------------------------*/
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "bsp_driver_sd.h"
#include "diskio.h"
#include "stm32l4xx_ll_dma.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* use the default SD timout as defined in the platform BSP driver*/
#if defined(SDMMC_DATATIMEOUT)
#define SD_TIMEOUT SDMMC_DATATIMEOUT
#endif

#define SD_DEFAULT_BLOCK_SIZE 512

/*
 * Depending on the use case, the SD card initialization could be done at the
 * application level: if it is the case define the flag below to disable
 * the BSP_SD_Init() call in the SD_Initialize() and add a call to
 * BSP_SD_Init() elsewhere in the application.
 */
/* USER CODE BEGIN disableSDInit */
/* #define DISABLE_SD_INIT */
/* USER CODE END disableSDInit */

/* Private variables ---------------------------------------------------------*/
/* Disk status */

int SD_initialize(uint8_t lun);
int SD_status(uint8_t lun);
int SD_read(uint8_t lun, uint8_t *buff, uint32_t sector, uint16_t count);
int SD_write(uint8_t lun, const uint8_t *buff, uint32_t sector, uint16_t count);
int SD_ioctl(uint8_t lun, uint8_t cmd, void *buff);


/* Private function prototypes -----------------------------------------------*/
static int SD_CheckStatus(uint8_t lun);
#if(0)
int SD_initialize (uint8_t);
int SD_status (uint8_t);
int SD_read (uint8_t, uint8_t*, uint32_t, uint16_t);
int SD_write (uint8_t, const uint8_t*, uint32_t, uint16_t);
int SD_ioctl (uint8_t, uint8_t, void*);
#endif

/* USER CODE BEGIN beforeFunctionSection */
/* can be used to modify / undefine following code or add new code */

static SD_HandleTypeDef *hsd_handle = NULL;
static volatile uint8_t sd_state = STA_NOINIT;
/* USER CODE END beforeFunctionSection */

/* Private functions ---------------------------------------------------------*/

static int SD_CheckStatus(uint8_t lun){
#if(0)
  if(BSP_SD_GetCardState() == MSD_OK)
  {
    Stat &= ~STA_NOINIT;
  }
#endif
    UNUSED(lun);
    if(hsd_handle == NULL) return 3;  /* 3: Not Ready */
//    uint32_t card_state = 0;
//    card_state = HAL_SD_GetCardState(&hsd_handle);
//
//    if ((card_state == HAL_SD_CARD_TRANSFER)){
//        return 0;
//    }else if((card_state == HAL_SD_CARD_SENDING) ||
//             (card_state == HAL_SD_CARD_RECEIVING) ||
//             (card_state == HAL_SD_CARD_PROGRAMMING))
//    {
//        return (1);   /// busy
//    }else{
//        return(2);   /// error
//    }
    return ((HAL_SD_GetCardState(hsd_handle) == HAL_SD_CARD_TRANSFER ) ? 0 : 1);
    return 0;
}

/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval int: Operation status
  */
int SD_initialize(uint8_t lun)
{
#if(0)
#if !defined(DISABLE_SD_INIT)

  if(BSP_SD_Init() == MSD_OK)
  {
    Stat = SD_CheckStatus(lun);
  }

#else
  Stat = SD_CheckStatus(lun);
#endif
#endif
    int err = STA_NOINIT;
    err = sdmmc_initialize(&hsd_handle);
    if(err == 0) err = SD_CheckStatus(lun);

  return err;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval int: Operation status
  */
int SD_status(uint8_t lun)
{
  return SD_CheckStatus(lun);
}

/* USER CODE BEGIN beforeReadSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeReadSection */
/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval int: Operation result
  */

int SD_read(uint8_t lun, uint8_t *buff, uint32_t sector, uint16_t count)
{
#if(0)
    int res = RES_ERROR;
  uint8_t ret = 0;

  if(BSP_SD_IsDetected() != SD_PRESENT) return RES_NOTRDY;
#if(SDMMC_TRANSFER_USE_DMA)
  ret = BSP_SD_ReadBlocks_DMA((uint32_t*)buff, (uint32_t) (sector), count);
#else
  ret = BSP_SD_ReadBlocks((uint32_t*)buff, (uint32_t) (sector), count, SD_TIMEOUT);
#endif
  if(ret == MSD_OK)
  {
    /* wait until the read operation is finished */
    while(BSP_SD_GetCardState()!= MSD_OK)
    {
    }
    res = RES_OK;
  }
#endif
    int err = 0;
    UNUSED(lun);
    uint32_t timeout = SD_TIMEOUT;
    if(sd_detect()) return 3;
#if(1)
    LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_4);
    LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_4, LL_DMA_REQUEST_7);
    hsd_handle->hdmarx->Init.Direction = DMA_PERIPH_TO_MEMORY;
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    err = HAL_SD_ReadBlocks_DMA(hsd_handle, buff, sector, count);
    if(err){
        err = 1; /* 1: R/W Error */
        goto end_section;
    }
    while((hsd_handle->hdmarx->State != HAL_DMA_STATE_READY) && (timeout)){
        timeout--;
    }
    while((hsd_handle->State != HAL_SD_STATE_READY) && (timeout)){
        timeout--;
    }
    while(SD_CheckStatus(lun) && (timeout)){
        timeout--;
    }
    if((hsd_handle->ErrorCode) || (!timeout)){
        err = 1;
        goto end_section;
    }
#else
    err = HAL_SD_ReadBlocks(hsd_handle, buff, sector, count, 1000);
#endif
    end_section:
  return err;
}

/* USER CODE BEGIN beforeWriteSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeWriteSection */
/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval int: Operation result
  */

int SD_write(uint8_t lun, const uint8_t *buff, uint32_t sector, uint16_t count)
{
#if(0)
    int res = RES_ERROR;
  uint8_t ret = MSD_OK;

  if(BSP_SD_IsDetected() != SD_PRESENT) return RES_NOTRDY;
#if(SDMMC_TRANSFER_USE_DMA)
  ret = BSP_SD_WriteBlocks_DMA((uint32_t*)buff, (uint32_t)(sector), count);
#else
  ret = BSP_SD_WriteBlocks((uint32_t*)buff, (uint32_t)(sector), count, SD_TIMEOUT);
#endif
  if(ret == MSD_OK)
  {
	/* wait until the Write operation is finished */
    while(BSP_SD_GetCardState() != MSD_OK)
    {
    }
    res = RES_OK;
  }
#endif
    int err = 0;
    uint32_t timeout = SD_TIMEOUT;
//    UNUSED(lun);
    if(sd_detect()) return 3;
#if(1)
    LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_4);
    LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_4, LL_DMA_REQUEST_7);
    hsd_handle->hdmatx->Init.Direction = DMA_MEMORY_TO_PERIPH;
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    err = HAL_SD_WriteBlocks_DMA(hsd_handle, buff, sector, count);
    if(err){
        err = 1; /* 1: R/W Error */
        goto end_section;
    }
    while((hsd_handle->hdmatx->State != HAL_DMA_STATE_READY) && (timeout)){
        timeout--;
    }
    while((hsd_handle->State != HAL_SD_STATE_READY) && (timeout)){
        timeout--;
    }
    while(SD_CheckStatus(lun) && (timeout)){
        timeout--;
    }
    if((hsd_handle->ErrorCode) || (!timeout)){
        err = 1;
        goto end_section;
    }
#else
    err = HAL_SD_WriteBlocks(hsd_handle, buff, sector, count, 1000);
#endif
    end_section:
  return err;
}


/* USER CODE BEGIN beforeIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeIoctlSection */
/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval int: Operation result
  */

#include "diskio.h"

int SD_ioctl(uint8_t lun, uint8_t cmd, void *buff)
{

#if(1)
    int res = RES_ERROR;
    HAL_SD_CardInfoTypeDef CardInfo = { 0 };
    if(sd_detect()) return RES_NOTRDY;
  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;

  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
      HAL_SD_GetCardInfo(hsd_handle, &CardInfo);
    *(uint32_t *)buff = CardInfo.LogBlockNbr;
    res = RES_OK;
    break;

  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
      HAL_SD_GetCardInfo(hsd_handle, &CardInfo);
    *(uint16_t *)buff = CardInfo.LogBlockSize;
    res = RES_OK;
    break;

  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
      HAL_SD_GetCardInfo(hsd_handle, &CardInfo);
    *(uint32_t *)buff = CardInfo.LogBlockSize;
    res = RES_OK;
    break;
  default:
    res = RES_PARERR;
  }
    return res;
#endif

}


int sd_driver_register(char* out_path){
    int err = 0;
    Diskio_drvTypeDef io_drv = {
            .disk_initialize = SD_initialize,
            .disk_ioctl = SD_ioctl,
            .disk_read = SD_read,
            .disk_write = SD_write,
            .disk_status = SD_status,
    };
    err = FATFS_LinkDriver(&io_drv, out_path);

//    err = sdmmc_initialize(&hsd_handle);
    return err;
}
/* USER CODE BEGIN afterIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END afterIoctlSection */

/* USER CODE BEGIN lastSection */
/* can be used to modify / undefine previous code or add new code */
/* USER CODE END lastSection */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

