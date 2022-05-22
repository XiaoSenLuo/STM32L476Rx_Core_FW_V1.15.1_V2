/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "stm32l4xx_it.h"
#include "stm32l4xx_hal.h"

#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_wwdg.h"
#include "stm32l4xx_ll_iwdg.h"


#include <math.h>
#include "sdmmc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "rtc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmd.h"
#include "gps.h"
#include "fatfs.h"
#include "config_ini.h"
#include "bsp_ads127.h"
#include "mem_dma.h"
#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


#define LOG_PATH    "log.txt"
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static SPI_HandleTypeDef_Handle spi1_handle = NULL;
static FATFS *fs = NULL;
static FIL ads_data_file = { 0 };


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef enum{
    SYS_CORE_FREQ_32M = 32000000UL,
    SYS_CORE_FREQ_36M = 36000000UL,
    SYS_CORE_FREQ_48M = 48000000UL,
    SYS_CORE_FREQ_64M = 64000000UL,
    SYS_CORE_FREQ_72M = 72000000UL,
    SYS_CORE_FREQ_80M = 80000000UL,
}SYS_Core_Freq_t;

static uint8_t LL_SystemClock_Config(uint32_t in_clock_index, uint32_t in_clk_freq);
static void SystemClockHSE_Config(SYS_Core_Freq_t in_clk_freq);
static void Clock48_Domain_Cofing(void);
static void RTCClockLSE_Config(void);

typedef enum{
    MCO_FREQ_1M = 10000000UL,
    MCO_FREQ_2M = 2000000UL,
	MCO_FREQ_4M = 4000000UL,
	MCO_FREQ_8M = 8000000UL,
	MCO_FREQ_16M = 16000000UL
}mco_freq_t;

static void mco_enable(mco_freq_t mco_feq){

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_8, LL_GPIO_PULL_DOWN);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_0);

    switch(mco_feq){
        case MCO_FREQ_1M:
            LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_HSE, LL_RCC_MCO1_DIV_16);
            break;
        case MCO_FREQ_2M:
            LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_HSE, LL_RCC_MCO1_DIV_8);
            break;
        case MCO_FREQ_4M:
            LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_HSE, LL_RCC_MCO1_DIV_4);
            break;
        case MCO_FREQ_8M:
            LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_HSE, LL_RCC_MCO1_DIV_2);
            break;
        case MCO_FREQ_16M:
            LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_HSE, LL_RCC_MCO1_DIV_1);
            break;
        default:
            LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_NOCLOCK, LL_RCC_MCO1_DIV_16);
            break;
    }
}

static void mco_disable(void){
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_8, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ANALOG);

    LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_NOCLOCK, LL_RCC_MCO1_DIV_16);
}

static void gps_pps_isr_handler(void *ctx){

}

static void gps_pps_isr_install(GPIO_TypeDef * ppsPort, int32_t ppsPin, isr_function_handle_t fn, void *ctx){

    int gpio_num = -1, irqn = -255;
    GPIO_InitTypeDef GPIO_InitStructure = {
            .Alternate = 0,
            .Mode = GPIO_MODE_IT_RISING,
            .Pin = 1UL << ppsPin,
            .Pull = LL_GPIO_PULL_UP,
            .Speed = GPIO_SPEED_LOW,
    };
    if((ppsPort == NULL) || (ppsPin < 0)) return;
    HAL_GPIO_Init(ppsPort, &GPIO_InitStructure);

    gpio_num = gpio_mask2num(ppsPin);
    irqn = gpio_get_irqn(gpio_num);

    ll_gpio_exti_isr_install(gpio_num, fn, ctx);

    HAL_NVIC_SetPriority(irqn, 0, 0);
    HAL_NVIC_EnableIRQ(irqn);
}

static void gps_pps_isr_uninstall(GPIO_TypeDef * ppsPort, int32_t ppsPin, isr_function_handle_t fn, void *ctx){
    int gpio_num = -1, irqn = -255;
    GPIO_InitTypeDef GPIO_InitStructure = {
            .Alternate = 0,
            .Mode = GPIO_MODE_ANALOG,
            .Pin = 1UL << ppsPin,
            .Pull = LL_GPIO_PULL_UP,
            .Speed = GPIO_SPEED_LOW,
    };
    UNUSED(ctx);
    if((ppsPort == NULL) || (ppsPin < 0)) return;
    HAL_GPIO_Init(ppsPort, &GPIO_InitStructure);
    gpio_num = (gpio_num_t)ppsPin;
    irqn = gpio_get_irqn(gpio_num);
    HAL_NVIC_DisableIRQ(irqn);
    ll_gpio_exti_isr_uninstall(gpio_num);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void){
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */
    if(0){
      uint32_t _system_clock_count = 4000000;
      while(_system_clock_count--);
    }

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    gpio_all_set_analog();
    rtc_initialize(NULL);
    SystemClockHSE_Config(SYS_CORE_FREQ_72M);
    HAL_Init();
    mem_dma_init();
    gpio_output_initialize();
    gpio_input_initialize();

    /* USER CODE BEGIN Init */
    mco_enable(MCO_FREQ_16M);

    if(1){
        struct tm time = {0};
        st_rtc_get_time(&time);
    }

    if(0){
        UART_HandleTypeDef *uart1_handle = NULL;
        usart1_initialize(&uart1_handle, 9600);

        const uint8_t test_tx[] = {'T', 'E', 'S', 'T', ' ', 'U', 'A', 'R', 'T', '\n'};
        usart1_write_bytes(uart1_handle, (void*)test_tx, sizeof(test_tx), 1000);
        usart1_start_receive(uart1_handle);
        while(1){
            uint8_t test[128], tl = 0;
            tl = usart1_read_bytes(uart1_handle, test, 128, 1000);
            if(tl) usart1_write_bytes(uart1_handle, test, tl, 1000);
        }
    }
    if(0){
        UART_HandleTypeDef *lpuart1_handle = NULL;
        lpuart1_initialize(&lpuart1_handle, 9600);

        lpuart1_start_receive(lpuart1_handle);
        while(1){
            uint8_t test[1024], tl = 0;
            tl = lpuart1_read_bytes(lpuart1_handle, test, 1024, 1000);
            UNUSED(tl);
        }
    }
    if(0){
        SD_HandleTypeDef *sd_handle = NULL;
        int err = 0;
        err = sdmmc_initialize(&sd_handle);
        HAL_Delay(100);

    }
    if(1){

        int err = 0;
        err = FATFS_SD_Init(&fs);
#if(0)
        err = f_mount(&fatfs, sd_root_path, 0);
        if(!err){
            err = f_open(file, "0:/test.txt", FA_WRITE | FA_CREATE_ALWAYS);
        }
        f_close(file);
#endif
        FS_FileOperations();
        err = f_open(&ads_data_file, "data.bin", FA_CREATE_ALWAYS | FA_WRITE);
        if(err != FR_OK){
            while(1){
                HAL_Delay(1000);
            }
        }
    }

    if(0){
        st_spi1_init(&spi1_handle);
        ads127_bsp_reset_pin_initial(GPIOC, 4);
        ads127_bsp_start_pin_initial(GPIOC, 5);

        ads127_bsp_reset();
        ads127_driver_initialaiz(spi1_handle, GPIOA, 4);
        if(0){
            ads127_dev_t device = {
                    0
            };
            ads127_get_id(&device);
            ads127_get_id(&device);
            ads127_get_mode(&device);
            HAL_Delay(100);
        }
        if(0){
            ads127_dev_t device = {
                    .config.val = 0x00,
            };
            uint32_t ret = 0;
            ads127_configure(&device);
            device.config.val = 0;
            ret = ads127_get_configure(&device);
            HAL_Delay(ret);
        }
        if(0){
            ads127_dev_t device = {
                    .ofc.val = 0x00112233,
            };
            ads127_configure_ofc(&device);
            device.ofc.val = 0;
            ads127_get_ofc(&device);
            HAL_Delay(100);
        }
        if(0){
            ads127_dev_t device = {
                    .fsc.val = 0x1122,
            };
            ads127_configure_fsc(&device);
            device.fsc.val = 0;
            ads127_get_fsc(&device);
            HAL_Delay(100);
        }
        HAL_Delay(100);

        ads127_dev_t device = ADS127_DEFAULT_DEVICE();
        ads127_get_id(&device);
        ads127_get_mode(&device);
        ads127_get_configure(&device);
        ads_data_init_t read_init = {
                .crate = 0,
                .osr = 0,
                .config = 0,
                .clk = MCO_FREQ_16M,
        };
        read_init.config.val = device.config.val;
        read_init.crate = UINT32_MAX;  /// 最大采样率
        read_init.osr = ads127_get_osr(&device);  /// 获取过采样因子
        ads127_bsp_read_init(&read_init);
        ads127_bsp_drdy_isr_install(GPIOB, 1, ads127_bsp_read_data_from_isr, spi1_handle);
        ads127_bsp_enable_drdy();
        ads127_bsp_start();
    }

    /* USER CODE END Init */

    /* Configure the system clock */


    while(1){
        if(ads127_bsp_availablle() == 0) continue;

        ads127_bsp_write_file(&ads_data_file);
    }

    /* USER CODE END SysInit */
    end_system_reset:
    NVIC_SystemReset();
    /* Initialize all configured peripherals */

    /* USER CODE BEGIN 2 */


    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}


/* USER CODE BEGIN 4 */

/**
 *
 */

void SystemClockHSE_Config(SYS_Core_Freq_t in_clk_freq){
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
    while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1){};
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    /* -1- Select MSI as system clock source to allow modification of the PLL configuration */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
    /* Initialization Error */
//    Error_Handler();
    }
    HAL_Init();
    /* -2- Enable HSE  Oscillator, select it as PLL source and finally activate the PLL */
    RCC_OscInitStruct.OscillatorType        = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState              = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLSource         = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLState          = RCC_PLL_ON;

    if(in_clk_freq == SYS_CORE_FREQ_32M){
      RCC_OscInitStruct.PLL.PLLM              = 2;  // 16M / M
      RCC_OscInitStruct.PLL.PLLN              = 16; // 16 / M * N
      RCC_OscInitStruct.PLL.PLLP              = 7;
      RCC_OscInitStruct.PLL.PLLQ              = 4;
      RCC_OscInitStruct.PLL.PLLR              = 4;
    //		  RCC_OscInitStruct.PLL.PLLSource         = RCC_PLLSOURCE_HSE;
    //		  RCC_OscInitStruct.PLL.PLLState          = RCC_PLL_ON;
    }
    if(in_clk_freq == SYS_CORE_FREQ_36M){
        RCC_OscInitStruct.PLL.PLLM              = 2;  // 16M / M
        RCC_OscInitStruct.PLL.PLLN              = 36; // 16 / M * N
        RCC_OscInitStruct.PLL.PLLP              = 7;
        RCC_OscInitStruct.PLL.PLLQ              = 6;
        RCC_OscInitStruct.PLL.PLLR              = 8;
        //		  RCC_OscInitStruct.PLL.PLLSource         = RCC_PLLSOURCE_HSE;
        //		  RCC_OscInitStruct.PLL.PLLState          = RCC_PLL_ON;
    }
    if(in_clk_freq == SYS_CORE_FREQ_48M){
      RCC_OscInitStruct.PLL.PLLM              = 2;  // 16M / M
      RCC_OscInitStruct.PLL.PLLN              = 36; // 16 / M * N
      RCC_OscInitStruct.PLL.PLLP              = 7;
      RCC_OscInitStruct.PLL.PLLQ              = 6;
      RCC_OscInitStruct.PLL.PLLR              = 6;
    //		  RCC_OscInitStruct.PLL.PLLSource         = RCC_PLLSOURCE_HSE;
    //		  RCC_OscInitStruct.PLL.PLLState          = RCC_PLL_ON;
    }
    if(in_clk_freq == SYS_CORE_FREQ_64M){
        RCC_OscInitStruct.PLL.PLLM              = 2;  // 16M / M
        RCC_OscInitStruct.PLL.PLLN              = 16; // 16 / M * N
        RCC_OscInitStruct.PLL.PLLP              = 7;
        RCC_OscInitStruct.PLL.PLLQ              = 4;
        RCC_OscInitStruct.PLL.PLLR              = 2;
        //		  RCC_OscInitStruct.PLL.PLLSource         = RCC_PLLSOURCE_HSE;
        //		  RCC_OscInitStruct.PLL.PLLState          = RCC_PLL_ON;
    }
    if(in_clk_freq == SYS_CORE_FREQ_72M){
        RCC_OscInitStruct.PLL.PLLM              = 2;  // 16M / M
        RCC_OscInitStruct.PLL.PLLN              = 36; // 16 / M * N
        RCC_OscInitStruct.PLL.PLLP              = 7;
        RCC_OscInitStruct.PLL.PLLQ              = 6;
        RCC_OscInitStruct.PLL.PLLR              = 4;
        //		  RCC_OscInitStruct.PLL.PLLSource         = RCC_PLLSOURCE_HSE;
        //		  RCC_OscInitStruct.PLL.PLLState          = RCC_PLL_ON;
    }
    if(in_clk_freq == SYS_CORE_FREQ_80M){
        RCC_OscInitStruct.PLL.PLLM              = 2;  // 16M / M
        RCC_OscInitStruct.PLL.PLLN              = 20; // 16 / M * N
        RCC_OscInitStruct.PLL.PLLP              = 7;
        RCC_OscInitStruct.PLL.PLLQ              = 4;
        RCC_OscInitStruct.PLL.PLLR              = 2;
        //		  RCC_OscInitStruct.PLL.PLLSource         = RCC_PLLSOURCE_HSE;
        //		  RCC_OscInitStruct.PLL.PLLState          = RCC_PLL_ON;
    }
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    /* Initialization Error */
//        Error_Handler();
    }

    /* -3- Select the PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
    RCC_ClkInitStruct.ClockType       = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
//    if(READ_BIT(RCC->CR, RCC_CR_PLLRDY)) RCC_ClkInitStruct.SYSCLKSource    = RCC_SYSCLKSOURCE_PLLCLK;
//    else{
//      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
//    }
    RCC_ClkInitStruct.SYSCLKSource    = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider   = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider  = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider  = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
    /* Initialization Error */
//    Error_Handler();
    }

    /* -4- Optional: Disable MSI Oscillator (if the MSI is no more needed by the application)*/
    RCC_OscInitStruct.OscillatorType  = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState        = RCC_MSI_OFF;
    RCC_OscInitStruct.PLL.PLLState    = RCC_PLL_NONE;  /* No update on PLL */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
    /* Initialization Error */
//    Error_Handler();
    }

    HAL_Init();
}

void Clock48_Domain_Cofing(void){
#if(0)
	RCC_PLLSAI1InitTypeDef PllSAI1ClkInit = {0};
	PllSAI1ClkInit.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
	PllSAI1ClkInit.PLLSAI1M = 2;
	PllSAI1ClkInit.PLLSAI1N = 12;
	PllSAI1ClkInit.PLLSAI1P = RCC_PLLP_DIV7;
	PllSAI1ClkInit.PLLSAI1Q = RCC_PLLQ_DIV2;
	PllSAI1ClkInit.PLLSAI1R = RCC_PLLR_DIV2;
	PllSAI1ClkInit.PLLSAI1Source = RCC_PLLSOURCE_HSE;

	if(HAL_RCCEx_EnablePLLSAI1(&PllSAI1ClkInit) != HAL_OK){
		Error_Handler();
	}
#endif
}

void RTCClockLSE_Config(void){
#if(1)
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_PWR_EnableBkUpAccess();
	uint32_t _timeout = 0;
    if(LL_RCC_LSE_IsReady() == 0){   // 防止重复初始化
    	_timeout = SystemCoreClock;
        LL_RCC_ForceBackupDomainReset();
        LL_RCC_ReleaseBackupDomainReset();
		LL_RCC_LSE_EnableBypass();
		LL_RCC_LSE_Enable();
		/* Wait till LSE is ready */
		while((LL_RCC_LSE_IsReady() != 1) && (_timeout--)){};
		LL_RCC_ClearFlag_LSERDY();
//		LL_RCC_LSE_EnableCSS();
		if((!_timeout) && (LL_RCC_LSE_IsReady() != 1)){
			LL_RCC_LSI_Enable();
			LL_RCC_LSE_Disable();
			while(LL_RCC_LSI_IsReady() != 1);
		}else{
			LL_RCC_LSI_Disable();
		}
    }
#else
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){
        Error_Handler();
    }

#endif
}


/* USER CODE END 4 */


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
    assert_failed(__FILE__, __LINE__);
//    while(1);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//     SEGGER_RTT_printf(0, "Error: %s on line %d\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
