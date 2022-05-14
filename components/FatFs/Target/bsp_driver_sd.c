/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    bsp_driver_sd.c for L4 (based on stm32l4r9i_eval_sd.c)
 * @brief   This file includes a generic uSD card driver.
 *          To be completed by the user according to the board used for the project.
 * @note    Some functions generated as weak: they can be overriden by
 *          - code in user files
 *          - or BSP code from the FW pack files
 *          if such files are added to the generated project (by the user).
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

#ifdef OLD_API
/* kept to avoid issue when migrating old projects. */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
#else
/* USER CODE BEGIN FirstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END FirstSection */
/* Includes ------------------------------------------------------------------*/

#include "bsp_driver_sd.h"
#include "main.h"

#define SDMMC_WAIT_TIMEOUT        30000  // 30s

/* Extern variables ---------------------------------------------------------*/

static void    BSP_SD_MspInit(SD_HandleTypeDef *hsd, void *Params);
static void    BSP_SD_MspDeInit(SD_HandleTypeDef *hsd, void *Params);

SD_HandleTypeDef hsd1 = { 0 };

/* USER CODE BEGIN BeforeInitSection */
/* can be used to modify / undefine following code or add code */
static DMA_HandleTypeDef hdma_rx = { 0 };
static DMA_HandleTypeDef hdma_tx = { 0 };

static DMA_HandleTypeDef hdma_sdmmc1 = { 0 };

static __IO int8_t sd_dma_status = 0;    //  0:end, 1:start, -1:error

static HAL_StatusTypeDef _sdmmc1_dma_rx_config(SD_HandleTypeDef *hsd);

static HAL_StatusTypeDef _sdmmc1_dma_tx_config(SD_HandleTypeDef *hsd);
static uint8_t SD_CheckReadWtiteOperation(SD_HandleTypeDef *hsd, uint32_t timeout);


static HAL_StatusTypeDef _sdmmc1_dma_rx_config(SD_HandleTypeDef *hsd)
{
  HAL_StatusTypeDef status = HAL_ERROR;

//  __DMAx_TxRx_CLK_ENABLE();
  /* Configure DMA Rx parameters */
  hdma_rx.Init.Request             = DMA_REQUEST_7;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_rx.Init.Mode                = DMA_NORMAL;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  hdma_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;

  hdma_rx.Instance = SD_DMAx_Rx_STREAM;

  /* Associate the DMA handle */
  __HAL_LINKDMA(hsd, hdmarx, hdma_rx);

  /* Stop any ongoing transfer and reset the state*/
//  HAL_DMA_Abort(&hdma_rx);
//
//  /* Deinitialize the Channel for new transfer */
//  HAL_DMA_DeInit(&hdma_rx);

  /* Configure the DMA Channel */
  status = HAL_DMA_Init(&hdma_rx);

  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(SD_DMAx_Rx_IRQn, DMA2_Channel4_Priority, 0);
  NVIC_EnableIRQ(SD_DMAx_Rx_IRQn);

  return (status);
}

/**
  * @brief Configure the DMA to transmit data to the SD card
  * @retval
  *  HAL_ERROR or HAL_OK
  */
static HAL_StatusTypeDef _sdmmc1_dma_tx_config(SD_HandleTypeDef *hsd)
{
  HAL_StatusTypeDef status = HAL_ERROR;

//  __DMAx_TxRx_CLK_ENABLE();
  /* Configure DMA Tx parameters */
  hdma_tx.Init.Request             = DMA_REQUEST_7;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  hdma_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;

  hdma_tx.Instance = SD_DMAx_Tx_STREAM;

  /* Associate the DMA handle */
  __HAL_LINKDMA(hsd, hdmatx, hdma_tx);

//  /* Stop any ongoing transfer and reset the state*/
//  HAL_DMA_Abort(&hdma_tx);
//
//  /* Deinitialize the Channel for new transfer */
//  HAL_DMA_DeInit(&hdma_tx);

  /* Configure the DMA Channel */
  status = HAL_DMA_Init(&hdma_tx);

  /* NVIC configuration for DMA transfer complete interrupt */
#if(SDMMC_SHARE_DMA == 0)
  HAL_NVIC_SetPriority(SD_DMAx_Tx_IRQn, DMA2_Channel5_Priority, 0);
#else
  HAL_NVIC_SetPriority(SD_DMAx_Tx_IRQn, DMA2_Channel4_Priority, 0);
#endif
  HAL_NVIC_EnableIRQ(SD_DMAx_Tx_IRQn);

  return (status);
}

static uint8_t SD_CheckReadWtiteOperation(SD_HandleTypeDef *hsd, uint32_t timeout){
    uint32_t _firsttick = 0, _lasttick = 1;
    uint8_t ret = MSD_OK;

    UNUSED(hsd);

    _firsttick = HAL_GetTick();
    while(
    		(sd_dma_status != 0)
			&& (((_lasttick - _firsttick) != timeout) || ((_lasttick - _firsttick) != (-1 * timeout)))
    ){
        _lasttick = HAL_GetTick();
    }
    if(sd_dma_status == 0) ret = MSD_OK;
    else ret = MSD_ERROR;
    return ret;
}


//#include "SEGGER_RTT.h"
extern void Error_Handler(void);

void BSP_SD_MspInit(SD_HandleTypeDef *hsd, void *Params){
	if(hsd->Instance == SDMMC1){
		GPIO_InitTypeDef gpioinitstruct = {0};
//		RCC_PeriphCLKInitTypeDef  RCC_PeriphClkInit;
//		HAL_StatusTypeDef hal_ret;
		/* Prevent unused argument(s) compilation warning */
		UNUSED(Params);

		LL_RCC_SetSDMMCClockSource(LL_RCC_SDMMC1_CLKSOURCE_PLLSAI1);

		/* Enable SDMMC1 clock */
		__HAL_RCC_SDMMC1_CLK_ENABLE();

		/* Enable DMA2 clocks */
		__DMAx_TxRx_CLK_ENABLE();

		/* Enable GPIOs clock */
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();
		__SD_DETECT_GPIO_CLK_ENABLE();

		/* Common GPIO configuration */
		gpioinitstruct.Mode      = GPIO_MODE_AF_PP;
		gpioinitstruct.Pull      = GPIO_PULLUP;
		gpioinitstruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
		gpioinitstruct.Alternate = GPIO_AF12_SDMMC1;

		/* GPIOC configuration */
		gpioinitstruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;

		HAL_GPIO_Init(GPIOC, &gpioinitstruct);

		/* GPIOD configuration */
		gpioinitstruct.Pin = GPIO_PIN_2;
		HAL_GPIO_Init(GPIOD, &gpioinitstruct);

		//  /* NVIC configuration for SDMMC1 interrupts */
		HAL_NVIC_SetPriority(SDMMC1_IRQn, SDMMC1_Priority, 0);
		NVIC_EnableIRQ(SDMMC1_IRQn);

		#if(SDMMC_TRANSFER_USE_DMA == 1)

		hdma_sdmmc1.Init.Request             = DMA_REQUEST_7;
		hdma_sdmmc1.Init.Direction           = DMA_MEMORY_TO_PERIPH;
		hdma_sdmmc1.Init.PeriphInc           = DMA_PINC_DISABLE;
		hdma_sdmmc1.Init.MemInc              = DMA_MINC_ENABLE;
		hdma_sdmmc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_sdmmc1.Init.Mode                = DMA_NORMAL;
		hdma_sdmmc1.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
		hdma_sdmmc1.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
		hdma_sdmmc1.Instance = SD_DMAx_Tx_STREAM;

		if (HAL_DMA_Init(&hdma_sdmmc1) != HAL_OK){

		}
		/* Several peripheral DMA handle pointers point to the same DMA handle.
		Be aware that there is only one channel to perform all the requested DMAs. */
		/* Be sure to change transfer direction before calling
		HAL_SD_ReadBlocks_DMA or HAL_SD_WriteBlocks_DMA. */
		__HAL_LINKDMA(hsd,hdmarx,hdma_sdmmc1);
		__HAL_LINKDMA(hsd,hdmatx,hdma_sdmmc1);

		  HAL_NVIC_SetPriority(SD_DMAx_Rx_IRQn, DMA2_Channel4_Priority, 0);
		  HAL_NVIC_EnableIRQ(SD_DMAx_Rx_IRQn);

		#if(SDMMC_SHARE_DMA == 0)
		/* DMA initialization should be done here but , as there is only one channel for RX and TX it is configured and done directly when required*/
		hal_ret = SD_DMAConfigRx(hsd);
		if(hal_ret != HAL_OK){
		  Error_Handler();
		}
		hal_ret = SD_DMAConfigTx(hsd);
		if(hal_ret != HAL_OK){
		  Error_Handler();
		}
		#endif
		#endif
	}
}

void BSP_SD_MspDeInit(SD_HandleTypeDef *hsd, void *Params)
{

  /* Prevent unused argument(s) compilation warning */
  UNUSED(Params);

  /* Disable all interrupts */
  HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
//  HAL_NVIC_DisableIRQ(SD_DETECT_IRQn);
  HAL_NVIC_DisableIRQ(SD_DMAx_Rx_IRQn);
  HAL_NVIC_DisableIRQ(SD_DMAx_Tx_IRQn);

  /* De-initialize all pins */
  HAL_GPIO_DeInit(GPIOC, (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12));
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

  /* De-initialize DMA channel */
  HAL_DMA_DeInit(hsd->hdmatx);
  HAL_DMA_DeInit(hsd->hdmarx);

  /* Deactivate clock of SDMMC */
  __HAL_RCC_SDMMC1_CLK_DISABLE();

}

void HAL_SD_MspInit(SD_HandleTypeDef *hsd){
	BSP_SD_MspInit(hsd, NULL);
}

void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd){

	LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_SDMMC1);
	LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_SDMMC1);

	BSP_SD_MspDeInit(hsd, NULL);
}


/* USER CODE END BeforeInitSection */
/**
  * @brief  Initializes the SD card device.
  * @retval SD status
  */
uint8_t BSP_SD_Init(void)
{
	uint8_t sd_state = MSD_OK;
	HAL_StatusTypeDef hal_ret = 0;
	uint32_t _sdmmc_div = 0, _sdmmc_clk = 0;
	LL_RCC_ClocksTypeDef rcc_clk = { 0 };

	LL_RCC_SetSDMMCClockSource(LL_RCC_SDMMC1_CLKSOURCE_PLLSAI1);

	LL_RCC_GetSystemClocksFreq(&rcc_clk);

    _sdmmc_clk = LL_RCC_GetSDMMCClockFreq(LL_RCC_SDMMC1_CLKSOURCE);

	_sdmmc_div = (rcc_clk.PCLK2_Frequency < _sdmmc_clk) ? 2 : 0;
    switch(rcc_clk.PCLK2_Frequency){
    case 8000000:
    	_sdmmc_div = 20;
    	break;
    case 16000000:
    	_sdmmc_div = 10;
    	break;
    case 32000000:
    	_sdmmc_div = 10;
    	break;
    case 64000000:
    	_sdmmc_div = 10;
    	break;
    }

	/* uSD device interface configuration */
	hsd1.Instance = SDMMC1;
	hsd1.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
	hsd1.Init.ClockBypass         = SDMMC_CLOCK_BYPASS_DISABLE;
	hsd1.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_ENABLE;
	hsd1.Init.BusWide             = SDMMC_BUS_WIDE_1B;
	hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
//	hsd1.Init.ClockDiv            = SDMMC_TRANSFER_CLK_DIV;  // 48MHz / (div + 2)
	hsd1.Init.ClockDiv            = _sdmmc_div;
	/* Initialize IO functionalities used by SD detect pin */

	/* Check if the SD card is plugged in the slot */
	if(BSP_SD_IsDetected() != SD_PRESENT){
		return MSD_ERROR_SD_NOT_PRESENT;
	}
	/* Msp SD initialization */
    st_irq_handler_register(SDMMC1_IRQn, sdmmc1_irq_callback);

	/* HAL SD initialization */
	hal_ret = HAL_SD_Init(&hsd1);
	if(hal_ret != HAL_OK){
		hal_ret = hsd1.ErrorCode;
		sd_state = MSD_ERROR;
		Error_Handler();
	}else{
		sd_state = MSD_OK;
	}

	/* Configure SD Bus width */
	if(sd_state == MSD_OK){
	/* Enable wide operation */
		hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
		sd_state = HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B);
	}

	return  sd_state;
}

uint8_t BSP_SD_DeInit(void){
	uint32_t ret = 0;

	ret = HAL_SD_DeInit(&hsd1);

	if(ret == HAL_OK) return 0;
	else return 1;
}

/* USER CODE BEGIN AfterInitSection */


uint8_t BSP_SD_ChangeSpeed(int16_t in_div){
	uint8_t ret = 0;
	if(in_div < 0){
		uint32_t _sdmmc_div = 0, _sdmmc_clk = 0;
		LL_RCC_ClocksTypeDef rcc_clk = { 0 };

		LL_RCC_SetSDMMCClockSource(LL_RCC_SDMMC1_CLKSOURCE_PLLSAI1);

		LL_RCC_GetSystemClocksFreq(&rcc_clk);

	    _sdmmc_clk = LL_RCC_GetSDMMCClockFreq(LL_RCC_SDMMC1_CLKSOURCE);

		_sdmmc_div = (rcc_clk.PCLK2_Frequency < _sdmmc_clk) ? 2 : 0;
	    switch(rcc_clk.PCLK2_Frequency){
	    case 8000000:
	    	_sdmmc_div = 22;
	    	break;
	    case 16000000:
	    	_sdmmc_div = 22;
	    	break;
	    case 32000000:
	    	_sdmmc_div = 10;
	    	break;
	    case 64000000:
	    	_sdmmc_div = 10;
	    	break;
	    default:
	    	_sdmmc_div = 10;
	    	break;
	    }
	    hsd1.Init.ClockDiv = _sdmmc_div;
	}else{
		hsd1.Init.ClockDiv = in_div;
	}
	ret = SDMMC_Init(hsd1.Instance, hsd1.Init);
	return ret;
}

void BSP_SD_Enable(void){
	__HAL_SD_ENABLE(&hsd1);
}



void BSP_SD_Disable(void){
	__HAL_SD_DISABLE(&hsd1);
}

/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END AfterInitSection */

/* USER CODE BEGIN InterruptMode */
/**
  * @brief  Configures Interrupt mode for SD detection pin.
  * @retval Returns 0
  */
uint8_t BSP_SD_ITConfig(void){
  /* Code to be updated by the user or replaced by one from the FW pack (in a stmxxxx_sd.c file) */
	  /* Code to be updated by the user or replaced by one from the FW pack (in a stmxxxx_sd.c file) */
    __SD_DETECT_GPIO_CLK_ENABLE();
	  /* SD Card detect pin configuration */
//	LL_GPIO_SetPinMode(SD_DETECT_GPIO_PORT, SD_DETECT_PIN, LL_GPIO_MODE_INPUT);
//    LL_GPIO_SetPinPull(SD_DETECT_GPIO_PORT, SD_DETECT_PIN, LL_GPIO_PULL_UP);
	return 0;
}

/* USER CODE END InterruptMode */

/* USER CODE BEGIN BeforeReadBlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeReadBlocksSection */
/**
  * @brief  Reads block(s) from a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read
  * @param  Timeout: Timeout for read operation
  * @retval SD status
  */
__weak uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  uint8_t sd_state = MSD_OK;

  sd_dma_status = 1;
  sd_state = HAL_SD_ReadBlocks(&hsd1, (uint8_t *)pData, ReadAddr, NumOfBlocks, Timeout);
  if (sd_state != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }else{
	  sd_state = MSD_OK;
//    sd_state = SD_CheckReadWtiteOperation(NULL, SDMMC_WAIT_TIMEOUT);
  }

  return sd_state;
}

/* USER CODE BEGIN BeforeWriteBlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeWriteBlocksSection */
/**
  * @brief  Writes block(s) to a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of SD blocks to write
  * @param  Timeout: Timeout for write operation
  * @retval SD status
  */
__weak uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  uint32_t sd_state = MSD_OK;

  sd_dma_status = 1;
  sd_state = HAL_SD_WriteBlocks(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks, Timeout);
  if (sd_state != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }
  if(sd_state == HAL_OK){
	  sd_state = MSD_OK;
//    sd_state = SD_CheckReadWtiteOperation(NULL, SDMMC_WAIT_TIMEOUT);
  }
  return sd_state;
}

/* USER CODE BEGIN BeforeReadDMABlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeReadDMABlocksSection */
/**
  * @brief  Reads block(s) from a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read
  * @retval SD status
  */
__weak uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
  uint8_t sd_state = MSD_OK;

#if(SDMMC_SHARE_DMA)
//  hsd1.hdmatx = NULL;
//  if(hsd1.hdmarx == NULL){
//	  sd_state = SD_DMAConfigRx(&hsd1);
//	  if(sd_state == HAL_OK) sd_state = MSD_OK;
//	  else sd_state = MSD_ERROR;
//  }

//	  __HAL_DMA_DISABLE(hsd1.hdmarx);
    st_irq_handler_register(DMA2_Channel4_IRQn, sdmmc1_dma_rxcplt_callback);
	LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_4);   // 关闭 dma 通道, 修改参数
	LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_4, LL_DMA_REQUEST_7);
	LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);  // 修改传输方向(修改寄存器)
	hdma_sdmmc1.Init.Direction = DMA_PERIPH_TO_MEMORY;  // 修改传输方向, 此参数会在 HAL_SD_ReadBlocks_DMA 或者 HAL_SD_ReadBlocks_DMA 内部调用, 但是并不会修改实际寄存器
//	  __HAL_LINKDMA(&hsd1, hdmarx, hdma_sdmmc1);
#endif
  /* Read block(s) in DMA transfer mode */
  sd_dma_status = 1;
  if(sd_state == MSD_OK) sd_state = HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)pData, ReadAddr, NumOfBlocks);
  if (sd_state != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }else{
    sd_state = SD_CheckReadWtiteOperation(NULL, SDMMC_WAIT_TIMEOUT);
  }

  return sd_state;
}

/* USER CODE BEGIN BeforeWriteDMABlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeWriteDMABlocksSection */
/**
  * @brief  Writes block(s) to a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of SD blocks to write
  * @retval SD status
  */
__weak uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
  uint8_t sd_state = MSD_OK;

#if(SDMMC_SHARE_DMA)
//  hsd1.hdmarx = NULL;
//  if(hsd1.hdmatx == NULL){
//	  sd_state = SD_DMAConfigTx(&hsd1);
//	  if(sd_state == HAL_OK) sd_state = MSD_OK;
//	  else sd_state = MSD_ERROR;
//  }
//    __HAL_DMA_DISABLE(hsd1.hdmarx);
    st_irq_handler_register(DMA2_Channel4_IRQn, sdmmc1_dma_txcplt_callback);
	  LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_4);
	  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_4, LL_DMA_REQUEST_7);
	  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	  hdma_sdmmc1.Init.Direction = DMA_MEMORY_TO_PERIPH;
//	  __HAL_LINKDMA(&hsd1, hdmatx, hdma_sdmmc1);
#endif
  /* Write block(s) in DMA transfer mode */
  sd_dma_status = 1;
  if(sd_state == MSD_OK) sd_state = HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks);
  if (sd_state != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }else{
	  sd_state = SD_CheckReadWtiteOperation(NULL, SDMMC_WAIT_TIMEOUT);
  }

  return sd_state;
}

/* USER CODE BEGIN BeforeEraseSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeEraseSection */
/**
  * @brief  Erases the specified memory area of the given SD card.
  * @param  StartAddr: Start byte address
  * @param  EndAddr: End byte address
  * @retval SD status
  */
__weak uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr)
{
  uint8_t sd_state = MSD_OK;

  if (HAL_SD_Erase(&hsd1, StartAddr, EndAddr) != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }

  return sd_state;
}

/* USER CODE BEGIN BeforeGetCardStateSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeGetCardStateSection */

/**
  * @brief  Gets the current SD card data status.
  * @param  None
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  SD_TRANSFER_OK: No data transfer is acting
  *            @arg  SD_TRANSFER_BUSY: Data transfer is acting
  */
__weak uint8_t BSP_SD_GetCardState(void)
{
  return ((HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER ) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}

/**
  * @brief  Get SD information about specific SD card.
  * @param  CardInfo: Pointer to HAL_SD_CardInfoTypedef structure
  * @retval None
  */
__weak void BSP_SD_GetCardInfo(BSP_SD_CardInfo *CardInfo)
{
  /* Get SD card Information */
  HAL_SD_GetCardInfo(&hsd1, CardInfo);
}

/* USER CODE BEGIN BeforeCallBacksSection */
/* can be used to modify previous code / undefine following code / add code */

void sdmmc1_irq_callback(void){
	HAL_SD_IRQHandler(&hsd1);
}

#if(SDMMC_SHARE_DMA == 0)

void    BSP_SD_DMA_Tx_IRQHandler(void){
    HAL_DMA_IRQHandler(hsd1.hdmatx);
}
#endif

void sdmmc1_dma_rxcplt_callback(void){
#if(SDMMC_SHARE_DMA == 1)
	if((hsd1.Context == (SD_CONTEXT_DMA | SD_CONTEXT_READ_SINGLE_BLOCK)) ||
	   (hsd1.Context == (SD_CONTEXT_DMA | SD_CONTEXT_READ_MULTIPLE_BLOCK)))
	{
		HAL_DMA_IRQHandler(hsd1.hdmarx);
	}
#elif(SDMMC_SHARE_DMA == 0)
	HAL_DMA_IRQHandler(hsd1.hdmarx);
#endif
}

void sdmmc1_dma_txcplt_callback(void){
#if(SDMMC_SHARE_DMA == 1)
    if((hsd1.Context == (SD_CONTEXT_DMA | SD_CONTEXT_WRITE_SINGLE_BLOCK)) ||
		     (hsd1.Context == (SD_CONTEXT_DMA | SD_CONTEXT_WRITE_MULTIPLE_BLOCK)))
	{
		HAL_DMA_IRQHandler(hsd1.hdmatx);
	}
#elif(SDMMC_SHARE_DMA == 0)
	HAL_DMA_IRQHandler(hsd1.hdmarx);
#endif
}

/* USER CODE END BeforeCallBacksSection */
/**
  * @brief SD Abort callbacks
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{
	sd_dma_status = -1;
//  BSP_SD_AbortCallback();
}

/**
  * @brief Tx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
	sd_dma_status = 0;
//  BSP_SD_WriteCpltCallback();
}

/**
  * @brief Rx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
	sd_dma_status = 0;
//  BSP_SD_ReadCpltCallback();
}

/* USER CODE BEGIN CallBacksSection_C */
/**
  * @brief BSP SD Abort callback
  * @retval None
  * @note empty (up to the user to fill it in or to remove it if useless)
  */
__weak void BSP_SD_AbortCallback(void)
{
  sd_dma_status = -1;
}

/**
  * @brief BSP Tx Transfer completed callback
  * @retval None
  * @note empty (up to the user to fill it in or to remove it if useless)
  */
__weak void BSP_SD_WriteCpltCallback(void)
{
//  sd_dma_status &= (~0x02);
	sd_dma_status = 0;
}

/**
  * @brief BSP Rx Transfer completed callback
  * @retval None
  * @note empty (up to the user to fill it in or to remove it if useless)
  */
__weak void BSP_SD_ReadCpltCallback(void)
{
//  sd_dma_status &= (~0x01);
	sd_dma_status = 0;
}
/* USER CODE END CallBacksSection_C */
#endif

/**
 * @brief  Detects if SD card is correctly plugged in the memory slot or not.
 * @param  None
 * @retval Returns if SD is detected or not
 */


/* USER CODE BEGIN AdditionalCode */
/* user code can be inserted here */
uint8_t BSP_SD_IsDetected(void){
  return !(SD_DETECT_GPIO_PORT->ODR & SD_DETECT_PIN);
}
/* USER CODE END AdditionalCode */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
