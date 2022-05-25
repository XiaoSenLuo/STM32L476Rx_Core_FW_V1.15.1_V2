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
#include "stm32l4xx_ll_rtc.h"

#include <math.h>
#include "sdmmc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "rtc.h"
#include "log.h"
#include "stateMachine.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmd.h"
#include "bsp_gps.h"
#include "fatfs.h"
#include "config_ini.h"
#include "bsp_ads127.h"
#include "mem_dma.h"
#include "main.h"
#include "system_typedef.h"

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

#define TIME_FILE_PATH               "0:/time.txt"


static system_config_t system_config = {
        .system = {
                .uart_baudrate = 921600,
        },
        .gps = {
                .baudrate = 9600,
        },
        .ads = {
                .ctrl = {.conver_rate = UINT32_MAX, .file.val = 1 << 2},
        },
        .sd = {
                .fm = {.val = 4},
                .file_limit = 1,
        },
        .rtc = {
                0
        },
};

static int system_error_code = 0;
static system_event_t sys_event = {
        .type = eTYPE_NOEVENT,
        .event = NOEVENT,
        .event_data = NULL,
};

struct analog_domain_data_s{
    FATFS *fs;
    char file_path[48];
    FIL ads_data_file;
    SPI_HandleTypeDef_Handle ads_spi_handle[2];
    ads127_dev_t device;
    system_config_t *sys_config;
    uint32_t clk;
};

static struct analog_domain_data_s analog_data_domain = {
        .fs = NULL,
        .ads_data_file = { 0 },
        .ads_spi_handle = {NULL, NULL},
        .file_path = {'\0'},
        .device = ADS127_DEFAULT_DEVICE(),
        .sys_config = &system_config,
        .clk = 16000000,
};

struct gps_domain_data_s {
    UART_HandleTypeDef *gps_uart_handle;
    FIL time_log_file;
    uint32_t pps_ok;
    system_config_t *sys_config;
};

static struct gps_domain_data_s gps_data_domain = {
        .gps_uart_handle = NULL,
        .time_log_file = { 0 },
        .pps_ok = 0,
        .sys_config = &system_config,
};

struct cmd_domain_data_s {
    UART_HandleTypeDef *cmd_uart_handle;
    system_config_t *sys_config;
};

static struct cmd_domain_data_s cmd_data_domain = {
        .cmd_uart_handle = NULL,
        .sys_config = &system_config,
};

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


static void core_get_device_uid(char* id, uint8_t len){
#define HEX "0123456789ABCDEF"
    uint32_t uid[3] = {0, 0, 0};
    uint8_t i = 0, j = 0;
    for(i = 0; i < 3; i++){
        uid[i] = READ_REG(*((uint32_t *)(UID_BASE + i * 4U)));
        for(j = 0; j < 8; j++){
            if(i * 8 + j < (len - 1)) id[(i << 3) + j] = HEX[(uid[i] << (j << 2) & 0xF0000000) >> 28];   // DEC >> HEX
            else{
                id[len - 1] = '\0';
                return;
            }
        }
    }
    id[(i << 3) + j + 1] = '\0';
#undef HEX
}


static void gpio_buzzer_warning(uint32_t keep_time, uint32_t on_time, uint32_t off_time){
    uint32_t timeout = keep_time, start_tick = 0, tick = 0, prio = on_time + off_time;
    start_tick = HAL_GetTick();
    tick = start_tick;
    do{

        tick = HAL_GetTick();
    }while((tick - start_tick) < timeout);
}

static void core_enter_sleep_mode(void){
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    LL_LPM_EnableSleep();
    LL_LPM_EnableSleepOnExit();
//	CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPONEXIT_Msk));  // 立即进入Sleep
//	SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPONEXIT_Msk));    // 响应完所有中断再进入中断
//	CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
    __WFI();
}

static void core_enter_power_mode(uint32_t power_mode){
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    if (LL_PWR_IsActiveFlag_SB() != 0){
        LL_PWR_ClearFlag_SB();
    }

    if((power_mode == LL_PWR_MODE_STANDBY) || (power_mode == LL_PWR_MODE_SHUTDOWN)){
        LL_PWR_ClearFlag_WU();
        LL_PWR_ClearFlag_WU5();
        LL_PWR_ClearFlag_WU4();
        LL_PWR_ClearFlag_WU3();
        LL_PWR_ClearFlag_WU2();
        LL_PWR_ClearFlag_WU1();
        LL_RTC_ClearFlag_WUT(RTC);
        LL_RTC_ClearFlag_ALRB(RTC);
        LL_RTC_ClearFlag_ALRA(RTC);
        LL_RTC_ClearFlag_TS(RTC);
        LL_RTC_ClearFlag_ITS(RTC);
    }

    LL_PWR_SetPowerMode(power_mode);
    LL_LPM_EnableDeepSleep();
    LL_LPM_EnableSleepOnExit();

    /* This option is used to ensure that store operations are completed */
#if defined ( __CC_ARM)
    __force_stores();
#endif
    /* Request Wait For Interrupt */
    __WFI();
}


static void core_mco_enable(mco_freq_t mco_feq){

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

static void core_mco_disable(void){
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_8, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ANALOG);

    LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_NOCLOCK, LL_RCC_MCO1_DIV_16);
}

static void gps_pps_isr_handler(void *ctx){
    gpio_buzzer_toggle();
}

static void gpio_dec5v_isr_handler(void *ctx){

}

static void gpio_dec5v_isr_install(GPIO_TypeDef * dec5VPort, int32_t dec5VPin, isr_function_handle_t fn, void *ctx){
    int gpio_num = -1, irqn = -255;
    GPIO_InitTypeDef GPIO_InitStructure = {
            .Alternate = 0,
            .Mode = GPIO_MODE_IT_RISING_FALLING | GPIO_MODE_INPUT,
            .Pin = 1UL << dec5VPin,
            .Pull = GPIO_PULLDOWN,
            .Speed = GPIO_SPEED_LOW,
    };
    if((dec5VPort == NULL) || (dec5VPin < 0)) return;
    HAL_GPIO_Init(dec5VPort, &GPIO_InitStructure);

    gpio_num = gpio_mask2num(dec5VPin);
    irqn = gpio_get_irqn(gpio_num);

    ll_gpio_exti_isr_install(gpio_num, fn, ctx);

    HAL_NVIC_SetPriority(irqn, 0, 0);
    HAL_NVIC_EnableIRQ(irqn);
}

static void gpio_dec5v_isr_uninstall(GPIO_TypeDef * dec5VPort, int32_t dec5VPin){
    int gpio_num = -1, irqn = -255;
    GPIO_InitTypeDef GPIO_InitStructure = {
            .Alternate = 0,
            .Mode = GPIO_MODE_ANALOG,
            .Pin = 1UL << dec5VPin,
            .Pull = GPIO_PULLDOWN,
            .Speed = GPIO_SPEED_LOW,
    };

    if((dec5VPort == NULL) || (dec5VPin < 0)) return;
    HAL_GPIO_Init(dec5VPort, &GPIO_InitStructure);
    gpio_num = (gpio_num_t)dec5VPin;
    irqn = gpio_get_irqn(gpio_num);
    HAL_NVIC_DisableIRQ(irqn);
    ll_gpio_exti_isr_uninstall(gpio_num);
}


static void enter_analog_state_initialize(void *currentStateData, struct event *event, void *newStateData);
static void enter_read_cmd_uart_state_initialize(void *currentStateData, struct event *event, void *newStateData);
static void enter_gps_time_state_initialize(void *currentStateData, struct event *event, void *newStateData);

static struct event get_event(void *ctx);
static bool check_event(void *check_event, struct event *event);
static void entry_idle_state_handler(void *state_data, struct event *event);
static void entry_read_cmd_uart_state_handler(void *state_data, struct event *event);
static void entry_parse_cmd_state_handler(void *state_data, struct event *event);
static void entry_gps_time_state_handler(void *state_data, struct event *event);
static void entry_analog_initialize_state_handler(void* state_data, struct event *event);
static void entry_analog_write_file_state_handler(void* state_data, struct event *event);

static void exit_analog_write_file_sate_handler(void* state_data, struct event *event);
static void exit_gps_time_state_handler(void* state_data, struct event *event);

static struct state read_event_state, idle_state, read_cmd_uart_state, parse_cmd_state, gps_time_state, analog_initialize_state, analog_write_file_state, error_state;

static struct state read_event_state = {
        .parentState = NULL,
        .entryState = NULL,
        .transitions = (struct transition[]){
            {  /// 流程 0
                eTYPE_ANALOG, (void *)(intptr_t)(EVENT_ANALOG_STOP), &check_event, NULL, &analog_initialize_state
                }, /// 流程0 必须比 流程1 优先
            {  /// 流程 1
                eTYPE_ANALOG, (void *)(intptr_t)(EVENT_ANALOG_BUFFER_FULL), &check_event, NULL, &analog_write_file_state,
                },
            {
                    eTYPE_USB, (void *)(intptr_t)EVENT_UART2USB_DATA_READY, &check_event, NULL, &read_cmd_uart_state,
                },
            {
                eTYPE_USB, (void*)(intptr_t)EVENT_UART2USB_CONNECT, &check_event, NULL, &read_event_state,
                },
            {
                eTYPE_GPS, (void *)(intptr_t)EVENT_GPS_PPS, &check_event, &gps_time_state
                },
            {
                eTYPE_SD, NULL, &check_event, NULL, &read_event_state,
                },
            {
                eTYPE_NOEVENT, NULL, NULL, NULL, NULL, &read_event_state,
                },
            },
        .numTransitions = 7,
        .entryAction = NULL,
        .exitAction = NULL,
        .data = &sys_event,
};

static struct state idle_state = {
        .parentState = &read_event_state,
        .entryState = NULL,
        .transitions = (struct transition[]){

            },
        .numTransitions = 0,
        .entryAction = &entry_idle_state_handler,
        .exitAction = NULL,
        .data = NULL,
};

/*!> 读取串口数据 */
static struct state read_cmd_uart_state = {
        .parentState = &read_event_state,
        .entryState = NULL,
        .transitions = (struct transition[]){

        },
        .numTransitions = 1,
        .entryAction = &entry_read_cmd_uart_state_handler,
        .exitAction = NULL,
        .data = NULL,
};

/*!> 命令解析状态 */
static struct state parse_cmd_state = {
        .parentState = &read_event_state,
        .entryState = NULL,
        .transitions = NULL,
        .numTransitions = 0,
        .entryAction = &entry_parse_cmd_state_handler,
        .exitAction = NULL,
        .data = NULL,
};

/*!> GPS 授时状态 */
static struct state gps_time_state = {
        .parentState = &read_event_state,
        .entryState = NULL,
        .transitions = (struct transition[]){
            eTYPE_GPS, NULL, NULL, NULL, &read_event_state
        },
        .numTransitions = 1,
        .entryAction = &entry_gps_time_state_handler,
        .exitAction = &exit_gps_time_state_handler,
        .data = NULL,
};

static struct state analog_initialize_state = {
        .parentState = &read_event_state,
        .entryState = NULL,
        .transitions = (struct transition[]){
            eTYPE_ANALOG, NULL, NULL, NULL, &analog_write_file_state
        },
        .numTransitions = 1,
        .entryAction = entry_analog_initialize_state_handler,
        .exitAction = NULL,
        .data = &analog_data_domain,
};

/*!> 采样数据写入文件 */
static struct state analog_write_file_state = {
        .parentState = &read_event_state,
        .entryState = NULL,
        .transitions = (struct transition[]){
                {eTYPE_ANALOG, NULL, NULL, NULL, &read_event_state},
        },
        .numTransitions = 1,
        .entryAction = &entry_analog_write_file_state_handler,
        .exitAction = &exit_analog_write_file_sate_handler,
        .data = &analog_data_domain,
};

/*!> 错误状态 */
static struct state error_state = {
        .parentState = &read_event_state,
        .entryState = NULL,
        .transitions = (struct transition[]){

        },
        .numTransitions = 1,
        .entryAction = NULL,
        .exitAction = NULL,
        .data = NULL,
};

struct stateMachine fsm = {
        .currentState = NULL,
        .previousState = NULL,
        .errorState = NULL,
};


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void){
    /* USER CODE BEGIN 1 */
    uint32_t err = 0;
    /* USER CODE END 1 */
    if(1){  /// \brief 等待供电稳定
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

    ads127_bsp_reset_pin_initial(IO_ADS_RESET_PORT, IO_ADS_RESET_PIN);
    ads127_bsp_start_pin_initial(IO_ADS_START_PORT, IO_ADS_START_PIN);

    gpio_ts3a_mcu();
    if(gpio_cmd_detect()){
        usart1_initialize(&cmd_data_domain.cmd_uart_handle, 921600);
    }

    /* USER CODE BEGIN Init */

    if(0){
        rtc_date_time_t time = { 0 };
        st_rtc_get_time_v2(&time);
    }

    if(gpio_sd_detect()){   /*!< 检测 SD 卡 */
        uint32_t timeout = 30000, start_tick = 0, tick = 0;
        start_tick = HAL_GetTick();
        tick = start_tick;
        do{
            if(gpio_sd_detect() == 0){
                HAL_Delay(1000);
                if(gpio_sd_detect() == 0){
                    gpio_buzzer_off();
                    break;
                }
            }
            tick = HAL_GetTick();
            if(((tick - start_tick) % 500) == 0) gpio_buzzer_toggle();
        }while((tick - start_tick < timeout));
        if((tick - start_tick >= timeout)){
            gpio_buzzer_off();
            core_enter_power_mode(LL_PWR_MODE_SHUTDOWN);
        }
        while(1);
    }
    err = FATFS_SD_Init(&analog_data_domain.fs);  /// \brief 挂载 SD
    if(err){
        gpio_buzzer_on();
        HAL_Delay(5000);
        gpio_buzzer_off();
        HAL_NVIC_SystemReset();
    }
    log_file_create(NULL, NULL);   /// 在默认位置创建日志文件
    log_printf("\n");

    if(1){  /// \brief 检测 Analog 采样 ADS127L01
        analog_data_domain.clk = MCO_FREQ_16M;
        core_mco_enable(analog_data_domain.clk);
        st_spi1_init(&analog_data_domain.ads_spi_handle[0]);
        ads127_bsp_reset();
        ads127_driver_initialize(analog_data_domain.ads_spi_handle[0], IO_ADS_CS_PORT , IO_ADS_CS_PIN);
        uint32_t device_id = 0;
        device_id = ads127_get_id(&analog_data_domain.device);
        if(device_id & 0x03){
            log_printf("检测到 ADC Chip, 设备ID: %#x. \n", device_id);
            ads127_bsp_keep_reset();
            core_mco_disable();
        }else{
            log_printf("无法检测 ADC Chip! \n");
            gpio_buzzer_on();
            HAL_Delay(5000);
            gpio_buzzer_off();
            log_file_close();
            HAL_GPIO_WritePin(IO_ADS_RESET_PORT, PIN_MASK(IO_ADS_RESET_PIN), 0);
            core_mco_disable();
            HAL_NVIC_SystemReset();
        }
    }
    /// \brief 自检通过
    if(0){
        uint32_t timeout = 30000, tick = 0, start_tick = 0;
        start_tick = HAL_GetTick();
        tick = start_tick;
        do{
            if(gpio_cmd_detect()){
                /// TODO 检测到 USB 插入, DoSomething
            }
            tick = HAL_GetTick();
            if(tick - start_tick >= timeout){
                /// TODO 没有检测到 USB 插入, DoSomething
            }
        }while(tick - start_tick < timeout);
    }

    /// \brief 进入有限状态机
    /// TODO FSM 变换
#if(0)

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
    if(0){

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
        SPI_HandleTypeDef *spi1_handle = ads_spi_handle[0];
        st_spi1_init(&spi1_handle);
        ads127_bsp_reset_pin_initial(GPIOC, 4);
        ads127_bsp_start_pin_initial(GPIOC, 5);

        ads127_bsp_reset();
        ads127_driver_initialize(spi1_handle, GPIOA, 4);
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

#endif
    volatile struct event fsm_event = {
            .type = 0,
            .data = 0,
    };

    stateM_init(&fsm, &read_event_state, &error_state);
    /* USER CODE END Init */
//    enter_analog_state_initialize(NULL, &(struct event){.type = eTYPE_ANALOG, .data = 0}, &analog_data_domain);
    /* Configure the system clock */

    while(1){
//        fsm_event.type = eTYPE_ANALOG;
        fsm_event = get_event(NULL);
        stateM_handleEvent(&fsm, &fsm_event);
//        entry_analog_write_file_state_handler(&analog_data_domain, &fsm_event);
    }

    /* USER CODE END SysInit */

    NVIC_SystemReset();
    /* Initialize all configured peripherals */

    /* USER CODE BEGIN 2 */
    return 0;

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
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) return;
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


static void enter_read_cmd_uart_state_initialize(void *currentStateData, struct event *event, void *newStateData){
    UNUSED(currentStateData);

    struct cmd_domain_data_s * data_domain = (struct cmd_domain_data_s *)newStateData;
    if(data_domain->cmd_uart_handle == NULL){
        usart1_initialize(&data_domain->cmd_uart_handle, data_domain->sys_config->system.uart_baudrate);
        usart1_start_receive(data_domain->cmd_uart_handle);
    }
}

static void enter_gps_time_state_initialize(void *currentStateData, struct event *event, void *newStateData){
    UNUSED(currentStateData);

    struct gps_domain_data_s * data_domain = (struct gps_domain_data_s *)newStateData;
    if(data_domain->gps_uart_handle == NULL){
        lpuart1_initialize(&data_domain->gps_uart_handle, data_domain->sys_config->gps.baudrate);
        lpuart1_start_receive(&data_domain->gps_uart_handle);  /// 开始 接收
    }
}

static void enter_analog_state_initialize(void *currentStateData, struct event *event, void *newStateData){
    UNUSED(currentStateData);

    if(event->type != eTYPE_ANALOG) return;
    if(ads127_bsp_is_start()) return;

    struct analog_domain_data_s * data_domain = (struct analog_domain_data_s *)newStateData;

    if(data_domain->ads_spi_handle[0] == NULL){
        rtc_date_time_t time = {0};
        uint8_t tmp = 0;
        st_spi1_init(&data_domain->ads_spi_handle[0]);
    }
        ads127_bsp_reset_pin_initial(IO_ADS_RESET_PORT, IO_ADS_RESET_PIN);
        ads127_bsp_start_pin_initial(IO_ADS_START_PORT, IO_ADS_START_PIN);
        ads127_bsp_reset();
        ads127_driver_initialize(analog_data_domain.ads_spi_handle[0], IO_ADS_CS_PORT , IO_ADS_CS_PIN);
    core_mco_enable(data_domain->clk);
        ads127_get_id(&data_domain->device);
        ads127_get_mode(&data_domain->device);

#if(0)
        strncpy(data_domain->file_path, sd_root_path, sizeof(data_domain->file_path)); ///
        core_get_device_uid(&data_domain->file_path[strlen(sd_root_path)], sizeof(data_domain->file_path) - strlen(sd_root_path));
        fs_create_dir(data_domain->file_path);
        strcat(data_domain->file_path, "/");
        tmp = strlen(data_domain->file_path);
        st_rtc_get_time_v2(&time);
        rtc_time2str(&time, &data_domain->file_path[tmp], tmp - 2);
        strcat(data_domain->file_path, ".txt");
        fs_create_file(&data_domain->ads_data_file, data_domain->file_path);
#else
        ads127_bsp_create_file(&data_domain->ads_data_file);
#endif

        ads_data_init_t init = { 0 };
        init.clk = data_domain->clk;
        init.config.val = data_domain->device.config.val;
        init.mode.val = data_domain->device.mode.val;
        ads127_bsp_read_init(&init);

        ads127_bsp_drdy_isr_install(IO_ADS_DRDY_PORT, IO_ADS_DRDY_PIN, ads127_bsp_read_data_from_isr, data_domain->ads_spi_handle[0]);

        ads127_bsp_start();
//    }
}

static struct event get_event(void *ctx){
    struct event event = {
            .type = eTYPE_NOEVENT,
            .data = NULL,
    };
    if(gpio_cmd_detect()){
        event.type = eTYPE_USB;
        event.data = &cmd_data_domain;
        if(gpio_get_pps_level()){       /// 优先级 比 CMD 高
            event.type = eTYPE_GPS;
            event.data = &gps_data_domain;
        }
    }else{
        event.type = eTYPE_ANALOG;
        event.data = (void *)&analog_data_domain;
    }
    return event;
}

static bool check_event(void *check_event, struct event *event){
    system_event_def_t check = (system_event_def_t)(intptr_t)check_event;
    bool ret = false;

    switch(check){
        case EVENT_UART2USB_DATA_READY:
            if(usart1_bytes_available(NULL)) ret = true;
            break;
        case EVENT_ANALOG_BUFFER_FULL:
            if(ads127_bsp_availablle(NULL)) ret = true;
            break;
        case EVENT_ANALOG_STOP:
            ret = ads127_bsp_is_start();
            ret = !ret;
            break;
        case EVENT_UART2USB_CONNECT:
            break;
        case EVENT_UART2USB_DISCONNECT:
            break;
        case EVENT_GPS_PPS:
            if(gpio_get_pps_level()) ret = true;
            break;
        case EVENT_SD_CONNECT:
            break;
        case EVENT_SD_DISCONNECT:
            break;
        case NOEVENT:
        default:
            ret = true;
            break;
    }
    return ret;
}

static void entry_idle_state_handler(void *state_data, struct event *event){

}

static void entry_read_cmd_uart_state_handler(void *state_data, struct event *event){

}

static void entry_parse_cmd_state_handler(void *state_data, struct event *event){

}

static void entry_gps_time_state_handler(void *state_data, struct event *event){
#define NMEA_BUFFER_SIZE               1024
    struct gps_domain_data_s * data_domain = (struct gps_domain_data_s *)state_data;
    char *buf = NULL, bak = '\0', ok = 0;
    int rb = 0, parse_res = 0, sfild = -1, pfild = -1, sec_add = 0, sec_fild = 0;
    uint32_t tick = 0, start_tick = 0;
    struct minmea_sentence_zda zda = { 0 };
    rtc_date_time_t time = { 0 };
    uint32_t bcd_time = 0x00000000, bcd_date = 0x00002101;
    u32_st_rtc_time_bcd_format_handle bcd_time_handle = NULL;
    u32_st_rtc_date_bcd_format_handle bcd_date_handle = NULL;
    FIL time_file = { 0 };

    bcd_time_handle = &bcd_time;
    bcd_date_handle = &bcd_date;

    gps_pps_irq_disable(); /// 停止响应 PPS IRQ

    if(buf == NULL) buf = (char *)malloc(NMEA_BUFFER_SIZE);
    start_tick = HAL_GetTick();
    tick = start_tick;

    LL_RTC_DisableWriteProtection(RTC);
    st_rtc_enter_initmode();

    do{
        tick = HAL_GetTick();
    }while(gpio_get_pps_level() && ((tick - start_tick) <= 500)); /// 等待 PPS 信号变低, PPS 宽度 100ms
    if((tick - start_tick) > 1000) goto end_section;
    rb = lpuart1_read_bytes(NULL, buf, NMEA_BUFFER_SIZE, 1100);   /// 读取 NMEA 数据
    if(!rb) goto end_section;
    buf[rb] = '\0';
    for(int i = 0; i < rb; i++){
        if(sfild == -1){
            if((buf[i] == '$') && (strncmp("ZDA", &buf[i + 3], 3) == 0)) sfild = i;
        }else{
            if((buf[i] == '\r') && (buf[i + 1] == '\n')) pfild = i + 1;
            break;
        }
    }

    if((sfild < 0) || (pfild < 0)) goto end_section;
    bak = buf[pfild + 1];
    buf[pfild + 1] = '\0';
    parse_res = (int)minmea_parse_zda(&zda, &buf[sfild]);
    buf[pfild + 1] = bak;
    if(!parse_res) goto end_section;
    /// 计算下一秒
    if(gpio_get_pps_level() == 0){ /// 下一秒还未到来, 加一秒
        sec_add = 1;
        sec_fild = 59;
    }else{   /// 下一秒已经到来, 加两秒
        add_2_section:
        sec_add = 2;
        sec_fild = 58;
        HAL_Delay(150);   /// 跳过 PPS 脉冲
    }

    add_section:
    time.time.ssecond = zda.time.microseconds;
    time.time.second = (zda.time.seconds == sec_fild) ? 0 : zda.time.seconds + sec_add;
    time.time.minute = (zda.time.seconds == sec_fild) ? ((zda.time.minutes == 59) ? 0 : (zda.time.minutes + 1)) : zda.time.minutes;
    time.time.hour = (zda.time.seconds == sec_fild) ? ((zda.time.minutes == 59) ? ((zda.time.hours == 23) ? 0 : (zda.time.hours + 1)) : zda.time.hours) : zda.time.hours;  /// XXX:进位到小时结束, 天数进位可能性太小, 不继续进位了, 权当隐藏 BUG
    time.date.day = zda.date.day;
    time.date.month = zda.date.month;
    time.date.year = zda.date.year;

    bcd_time_handle->sec = __LL_RTC_CONVERT_BIN2BCD(time.time.second);
    bcd_time_handle->min = __LL_RTC_CONVERT_BIN2BCD(time.time.minute);
    bcd_time_handle->hour = __LL_RTC_CONVERT_BIN2BCD(time.time.hour);
    bcd_time_handle->pm = 0;  /// 24 小时制

    bcd_date_handle->day = __LL_RTC_CONVERT_BIN2BCD(time.date.day);
    bcd_date_handle->wdu = __LL_RTC_CONVERT_BIN2BCD(time.date.weekday);
    bcd_date_handle->mon = __LL_RTC_CONVERT_BIN2BCD(time.date.month);
    bcd_date_handle->year = __LL_RTC_CONVERT_BIN2BCD(time.date.year - 2000);

    if(gpio_get_pps_level()){   /// 下一秒已经过去, 重新计算
        goto add_2_section;
//        while(gpio_get_pps_level());
//        while(gpio_get_pps_level() == 0);
    }

    if((gpio_get_pps_level() == 0)){
        while(gpio_get_pps_level() == 0);  /// 下一秒还未到来
    }

    RTC->TR = bcd_time;
    RTC->DR = bcd_date;
    ok = 1;

    end_section:
    st_rtc_exit_initmode();
    LL_RTC_EnableWriteProtection(RTC);

    if(ok){
        buf[0] = '\0';
        rb = f_open(&time_file, TIME_FILE_PATH, FA_OPEN_ALWAYS | FA_OPEN_APPEND);
        if((rb == FR_OK) || (rb == FR_EXIST)){
            rb = rtc_time2str(&time, buf, NMEA_BUFFER_SIZE);
            buf[rb++] = '\n';
            f_write(&time_file, buf, rb, NULL);
            f_close(&time_file);
        }
    }

    if(buf) free(buf);
    gps_pps_irq_enable();

#undef NMEA_BUFFER_SIZE
}

static void entry_analog_initialize_state_handler(void* state_data, struct event *event){
    gps_pps_isr_uninstall(); /// 停止响应 PPS
    lpuart1_deinitialize(NULL);
    lpuart1_deinitialize(NULL);              /// 关闭 GPS 串口

    usart1_stop_receive(NULL);
    usart1_deinitialize(NULL);

    enter_analog_state_initialize(state_data, event, state_data);
}

static void entry_analog_write_file_state_handler(void* state_data, struct event *event){
    struct analog_domain_data_s * data_domain = (struct analog_domain_data_s *)state_data;
    uint32_t bl = 0;
    bl = ads127_bsp_availablle(NULL);
    if(bl) ads127_bsp_write_file(&data_domain->ads_data_file);
}

static void exit_analog_write_file_sate_handler(void* state_data, struct event *event){
    struct analog_domain_data_s * data_domain = (struct analog_domain_data_s *)state_data;
    uint64_t file_size = 0;
    if(!data_domain->ads_data_file.obj.fs) return;
    file_size = f_size(&data_domain->ads_data_file);
    if(file_size >= 32000 * 1000){
        f_close(&data_domain->ads_data_file);
        ads127_bsp_create_file(&data_domain->ads_data_file);
    }
}

static void exit_gps_time_state_handler(void* state_data, struct event *event){
    struct gps_domain_data_s * data_domain = (struct gps_domain_data_s *)state_data;
    if(data_domain->gps_uart_handle){
        lpuart1_stop_receive(data_domain->gps_uart_handle);
        lpuart1_deinitialize(&data_domain->gps_uart_handle);
    }
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
