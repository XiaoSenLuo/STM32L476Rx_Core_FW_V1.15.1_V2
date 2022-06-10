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

#include "bsp_gps.h"
#include "fatfs.h"
#include "bsp_ini.h"
#include "bsp_ads127.h"
#include "mem_dma.h"
#include "main.h"
#include "system_typedef.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define TIME_FILE_PATH               "0:/time.txt"

typedef enum{
    SYS_CORE_FREQ_32M = 32000000UL,
    SYS_CORE_FREQ_36M = 36000000UL,
    SYS_CORE_FREQ_48M = 48000000UL,
    SYS_CORE_FREQ_64M = 64000000UL,
    SYS_CORE_FREQ_72M = 72000000UL,
    SYS_CORE_FREQ_80M = 80000000UL,
}SYS_Core_Freq_t;

typedef enum{
    MCO_FREQ_1M = 10000000UL,
    MCO_FREQ_2M = 2000000UL,
    MCO_FREQ_4M = 4000000UL,
    MCO_FREQ_8M = 8000000UL,
    MCO_FREQ_16M = 16000000UL
}mco_freq_t;


typedef union {
    struct {
        uint32_t self_test : 1;
        uint32_t analog : 1;
        uint32_t analog_init : 1;
        uint32_t cmd : 1;
        uint32_t cmd_init : 1;
        uint32_t gps : 1;
        uint32_t gps_init : 1;
        uint32_t buzzer : 1;
        uint32_t pvd_below : 1;
    };
    uint32_t val;
}system_fild_t;

static system_fild_t sys_fild = {.val = 0};

static system_config_t system_config = {
        .system = {
                .uart_baudrate = 921600,
        },
        .gps = {
                .baudrate = 9600,
        },
        .ads = {
                .clk = MCO_FREQ_16M,
                .conver_rate = UINT32_MAX,
                .file.val = 60 << 2,
        },
        .sd = {
                .fm = {.val = 4},
                .file_size_limit = 1,
        },
        .rtc = {
        },
};

static FATFS* sd_fs_handle = NULL;

#define ANALOG_BUZZER_NUMBER          10
struct analog_domain_data_s{
    FATFS* *fs;
    FIL ads_data_file;
    char dir[32];
    rtc_date_time_t file_create_time;
    SPI_HandleTypeDef_Handle ads_spi_handle[2];
    ads127_dev_t device;
    system_config_t *sys_config;
    uint32_t clk;
    uint64_t th;
    system_fild_t *fild;
    int error;
    uint16_t err_counter;
    uint8_t buzzer_number;

};

static struct analog_domain_data_s analog_data_domain = {
        .fs = &sd_fs_handle,
        .ads_data_file = { {0} },
        .file_create_time = {.time.val = 0, .date.val = 0},
        .ads_spi_handle = {NULL, NULL},
        .device = ADS127_DEFAULT_DEVICE(),
        .sys_config = &system_config,
        .clk = 16000000,
        .th = 62500 * 4 * 60,
        .fild = &sys_fild,
        .error = 0,
        .err_counter = 0,
        .buzzer_number = ANALOG_BUZZER_NUMBER,
};

struct gps_domain_data_s {
    FATFS* *fs;
    UART_HandleTypeDef *gps_uart_handle;
    uint32_t pps;
    system_config_t *sys_config;
    system_fild_t *fild;
    int error;
};


static struct gps_domain_data_s gps_data_domain = {
        .fs = &sd_fs_handle,
        .gps_uart_handle = NULL,
        .pps = 0,
        .sys_config = &system_config,
        .fild = &sys_fild,
};

struct cmd_domain_data_s {
    FATFS* *fs;
    UART_HandleTypeDef *cmd_uart_handle;
    system_config_t *sys_config;
    system_fild_t *fild;
    int error;
};

static struct cmd_domain_data_s cmd_data_domain = {
        .fs = &sd_fs_handle,
        .cmd_uart_handle = NULL,
        .sys_config = &system_config,
        .fild = &sys_fild,
};


struct idle_domain_data_s {
    FATFS* *fs;
    struct analog_domain_data_s * analog_domain;
    struct gps_domain_data_s * gps_domain;
    struct cmd_domain_data_s * cmd_domain;
    system_config_t *sys_config;
    uint32_t start_tick;
    system_fild_t *fild;
};


static struct idle_domain_data_s idle_data_domain = {
        .fs = &sd_fs_handle,
    .analog_domain = &analog_data_domain,
    .gps_domain = &gps_data_domain,
    .cmd_domain = &cmd_data_domain,
    .sys_config = &system_config,
    .start_tick = 0,
    .fild = &sys_fild,
};


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t LL_SystemClock_Config(uint32_t in_clock_index, uint32_t in_clk_freq);
static void SystemClockHSE_Config(SYS_Core_Freq_t in_clk_freq);
static void Clock48_Domain_Cofing(void);
static void RTCClockLSE_Config(void);
static void pvd_pvm_isr_handler(void *ctx);

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

}

static void core_iwdg_initialize(uint32_t ms_timeout){
    UNUSED(ms_timeout);
    IWDG_HandleTypeDef iwdg = {.Instance = IWDG, .Init.Prescaler = IWDG_PRESCALER_256, .Init.Window = IWDG_WINDOW_DISABLE, .Init.Reload = 3750};

    HAL_IWDG_Init(&iwdg);
}

static inline void core_iwdg_refresh(void){
    WRITE_REG(IWDG->KR, IWDG_KEY_RELOAD);
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
    struct gps_domain_data_s * data_domain = (struct gps_domain_data_s *)ctx;
    data_domain->pps += 1;
}

static void gps_nmea_parse_zda_handler(uint8_t ch, void *ctx){
    /// $GNZDA,124729.000,30,05,2022,00,00*47\r\n\0
    gps_nmea_parse_zda_t *zda = (gps_nmea_parse_zda_t *)ctx;

    zda->sentence[zda->char_num++] = ch;
    if(zda->sentence[0] != '$'){
        zda->char_num = 0;
        return;
    }
    if((zda->char_num == 6) && (strncmp("ZDA", &zda->sentence[3], 3) != 0)){
        zda->char_num = 0;
        return;
    }
    if((zda->sentence[0] == '$') && (zda->sentence[zda->char_num - 2] == '\r') && (ch == '\n')){
        zda->sentence[zda->char_num] = '\0';
//        zda->parse = minmea_parse_zda(&zda->zda, zda->sentence);
        zda->parse = 1;
    }
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


static void analog_state_transition_action(void *currentStateData, struct event *event, void *newStateData);
static void read_cmd_uart_state_transition_action(void *currentStateData, struct event *event, void *newStateData);
static void gps_time_state_transition_action(void *currentStateData, struct event *event, void *newStateData);
static void idle_state_transition_action(void *currentStateData, struct event *event, void *newStateData);

static bool check_event(void *check_event, struct event *event);
static void entry_self_test_state_handler(void *state_data, struct event *event);

static void entry_idle_state_handler(void *state_data, struct event *event);
static void entry_read_cmd_uart_state_handler(void *state_data, struct event *event);
static void entry_parse_cmd_state_handler(void *state_data, struct event *event);
static void entry_gps_time_state_handler(void *state_data, struct event *event);
static void entry_analog_state_handler(void* state_data, struct event *event);
static void entry_analog_write_file_state_handler(void* state_data, struct event *event);
static void entry_error_state_handler(void* state_data, struct event *event);

static void exit_analog_write_file_sate_handler(void* state_data, struct event *event);
static void exit_gps_time_state_handler(void* state_data, struct event *event);

static struct state start_sate, self_test_state, idle_state, read_cmd_uart_state, parse_cmd_state, gps_time_state,  analog_state, analog_write_file_state, error_state;

static struct state start_sate = {
        .parentState = NULL,
        .entryState = &self_test_state,
        .transitions = (struct transition[]){
                {
                        eTYPE_SELF, NULL, NULL, NULL, &self_test_state,
                },
                {
                        eTYPE_NOEVENT, NULL, NULL, NULL, &idle_state,
                },
        },
        .numTransitions = 2,
        .entryAction = NULL,
        .exitAction = NULL,
        .data = &idle_data_domain,
};

static struct state self_test_state = {
        .parentState = &start_sate,
        .entryState = NULL,
        .transitions = (struct transition[]){
                {
                    eTYPE_SELF, NULL, NULL, NULL, &self_test_state,
                },
                {
                    eTYPE_NOEVENT, NULL, NULL, NULL, &idle_state,
                },
                {
                    eTYPE_ERROR, NULL, NULL, NULL, &error_state
                },
        },
        .numTransitions = 3,
        .entryAction = &entry_self_test_state_handler,
        .exitAction = NULL,
        .data = &idle_data_domain,
};

static struct state idle_state = {
        .parentState = &start_sate,
        .entryState = NULL,
        .transitions = (struct transition[]){
                {
                        eTYPE_ANALOG, (void*)(intptr_t)EVENT_ANALOG_BUFFER_FULL, &check_event, NULL, &analog_write_file_state,
                },
                {
                        eTYPE_ANALOG, NULL, NULL, &analog_state_transition_action, &analog_state,
                },
                {
                        eTYPE_CMD, (void*)(intptr_t)EVENT_UART2USB_DATA_READY, &check_event, NULL, &read_cmd_uart_state,
                },
                {     /// TODO 处理 USB 插入
                        eTYPE_USB, NULL, NULL, NULL, &idle_state,
                },
                {
                        eTYPE_SD, NULL, NULL, NULL, &idle_state,
                },
                {
                        eTYPE_GPS, NULL, NULL, gps_time_state_transition_action, &gps_time_state,
                },
                {
                        eTYPE_ERROR, NULL, NULL, NULL, &error_state,
                },
                {
                        eTYPE_NOEVENT, NULL, NULL, &idle_state_transition_action, &idle_state,
                },
        },
        .numTransitions = 8,
        .entryAction = &entry_idle_state_handler,
        .exitAction = NULL,
        .data = &idle_data_domain,
};

/*!> 读取串口数据 */
static struct state read_cmd_uart_state = {
        .parentState = &idle_state,
        .entryState = NULL,
        .transitions = (struct transition[]){
                {eTYPE_CMD, NULL, NULL, NULL, &parse_cmd_state},
                {eTYPE_NOEVENT, NULL, NULL, &idle_state_transition_action, &idle_state},
        },
        .numTransitions = 2,
        .entryAction = &entry_read_cmd_uart_state_handler,
        .exitAction = NULL,
        .data = &cmd_data_domain,
};

/*!> 命令解析状态 */
static struct state parse_cmd_state = {
        .parentState = &read_cmd_uart_state,
        .entryState = NULL,
        .transitions = (struct transition[]){
                {eTYPE_NOEVENT, NULL, NULL, &idle_state_transition_action, &idle_state},
        },
        .numTransitions = 1,
        .entryAction = &entry_parse_cmd_state_handler,
        .exitAction = NULL,
        .data = &cmd_data_domain,
};

/*!> GPS 授时状态 */
static struct state gps_time_state = {
        .parentState = &idle_state,
        .entryState = NULL,
        .transitions = (struct transition[]){
            {eTYPE_NOEVENT, NULL, NULL, &idle_state_transition_action, &idle_state},
        },
        .numTransitions = 1,
        .entryAction = &entry_gps_time_state_handler,
        .exitAction = &exit_gps_time_state_handler,
        .data = &gps_data_domain,
};

static struct state analog_state = {
        .parentState = &idle_state,
        .entryState = NULL,
        .transitions = (struct transition[]){
            {eTYPE_ANALOG, (void *)(intptr_t)(EVENT_ANALOG_BUFFER_FULL), &check_event, NULL, &analog_write_file_state},
            {eTYPE_NOEVENT, NULL, NULL, &idle_state_transition_action, &idle_state},
        },
        .numTransitions = 2,
        .entryAction = &entry_analog_state_handler,
        .exitAction = NULL,
        .data = &analog_data_domain,
};

/*!> 采样数据写入文件 */
static struct state analog_write_file_state = {
        .parentState = &analog_state,
        .entryState = NULL,
        .transitions = (struct transition[]){
                {eTYPE_NOEVENT, NULL, NULL, &idle_state_transition_action, &idle_state},
        },
        .numTransitions = 1,
        .entryAction = &entry_analog_write_file_state_handler,
        .exitAction = NULL,
        .data = &analog_data_domain,
};

/*!> 错误状态 */
static struct state error_state = {
        .parentState = &idle_state,
        .entryState = NULL,
        .transitions = (struct transition[]){
                {eTYPE_NOEVENT, NULL, NULL, NULL, &idle_state},
        },
        .numTransitions = 1,
        .entryAction = &entry_error_state_handler,
        .exitAction = NULL,
        .data = &idle_data_domain,
};

static struct event fsm_event = {
        .type = eTYPE_SELF,
        .data = 0,
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
        uint32_t _system_clock_count = 20000000;
        while(_system_clock_count--);
    }

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    HAL_Delay(2000);
    gpio_all_set_analog();
    SystemClockHSE_Config(SYS_CORE_FREQ_72M);
    HAL_Init();
    mem_dma_init();
    gpio_output_initialize();
    gpio_input_initialize();

    err = rtc_initialize(NULL);
    if(err != HAL_OK){
        gpio_buzzer_on();
        HAL_Delay(10000);
        HAL_NVIC_SystemReset();
        while(1);
    }

    gpio_buzzer_on();
    HAL_Delay(100);
    gpio_buzzer_off();

    gps_pps_isr_install(IO_PPS_PORT, IO_PPS_PIN, &gps_pps_isr_handler, &gps_data_domain);

    ll_peripheral_isr_install(PVD_PVM_IRQn, &pvd_pvm_isr_handler, &sys_fild);  /// \brief 注册PVD 回调函数

    core_iwdg_initialize(30000);
    /// \brief 进入有限状态机
    /// TODO FSM 变换
#if(0)
    if(0){
        UART_HandleTypeDef  *cmd_uart_handle = NULL;
//        usart1_initialize(&cmd_uart_handle, 921600);
//        usart1_write_bytes(cmd_uart_handle, "test\n", 5, 100);

        gps_pps_isr_install(IO_PPS_PORT, IO_PPS_PIN, gps_pps_isr_handler, NULL);
        gps_pps_irq_disable();
//        while(gpio_get_pps_level() == 0);
        gps_time_state_transition_action(&idle_data_domain, NULL, &gps_data_domain);

        while(1){
            while(gpio_get_pps_level() == 0);
            gps_nmea_parse_zda_t zda = {.sentence = {'\0'}, .char_num = 0, .parse = 0,};
            lpuart1_install_char_process_handler(&gps_nmea_parse_zda_handler, &zda);
            lpuart1_start_receive(NULL);
        	while(zda.parse == 0);
            lpuart1_stop_receive(NULL);
            lpuart1_install_char_process_handler(NULL, NULL);
//            HAL_Delay(1000);
        };
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
//            tl = lpuart1_read_bytes(lpuart1_handle, test, 1024, 1000);
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
        FATFS * fs = NULL;
        FIL ads_data_file;
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
        SPI_HandleTypeDef *spi1_handle = NULL;
        st_spi1_init(&spi1_handle);
        ads127_bsp_reset_pin_initial(IO_ADS_RESET_PORT, IO_ADS_RESET_PIN);

        core_mco_enable(MCO_FREQ_16M);
        ads127_bsp_reset();
        ads127_driver_initialize(spi1_handle, IO_ADS_CS_PORT, IO_ADS_CS_PIN);
        ads127_dev_t device = {
                0
        };
        if(1){
            ads127_get_id(&device);
            ads127_get_id(&device);
            ads127_get_mode(&device);
            HAL_Delay(100);
        }
        if(1){
            uint32_t ret = 0;
            ads127_configure(&device);
            device.config.fsc = 1;
            device.config.ofc = 1;
            ads127_configure(&device);
            ret = ads127_get_configure(&device);
            HAL_Delay(ret);
        }
        if(1){
            device.ofc.val = 0x00ffff20;
            ads127_configure_ofc(&device);
            device.ofc.val = 0;
            ads127_get_ofc(&device);
            HAL_Delay(100);
        }
        if(1){
            device.fsc.val = 0x1122;
            ads127_configure_fsc(&device);
            device.fsc.val = 0;
            ads127_get_fsc(&device);
            HAL_Delay(100);
        }
        HAL_Delay(100);

        ads127_get_id(&device);
        ads127_get_mode(&device);
        ads127_get_configure(&device);
        ads_data_init_t read_init = {
                .config = 0,
                .clk = MCO_FREQ_16M,
        };
        read_init.config.val = device.config.val;
        ads127_bsp_read_init(&read_init, NULL, NULL);
        ads127_bsp_drdy_isr_install(GPIOB, 1, ads127_bsp_read_data_from_isr, spi1_handle);
        ads127_bsp_enable_drdy();
        ads127_bsp_start();
    }

#endif

    stateM_init(&fsm, &start_sate, &error_state);
    /* USER CODE END Init */

    /* Configure the system clock */
    idle_data_domain.start_tick = HAL_GetTick();
    while(1){
        if((!sys_fild.analog) && sys_fild.self_test){
            uint32_t tick = HAL_GetTick();
            struct state *cs = NULL;
            cs = stateM_currentState(&fsm);
            if(((tick - idle_data_domain.start_tick) > (10 * 1000)) && (cs == &idle_state)){
                sys_fild.analog = 1;
                fsm_event.type = eTYPE_ANALOG;
            }
        }
        stateM_handleEvent(&fsm, &fsm_event);
        core_iwdg_refresh();
    }
    /* USER CODE END SysInit */
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
    return 0;
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

static void pvd_pvm_isr_handler(void *ctx){

    /* Check PWR exti flag */
    if(__HAL_PWR_PVD_EXTI_GET_FLAG() != 0x0U){
        uint32_t pvdo = 0;
        pvdo = __HAL_PWR_GET_FLAG(PWR_FLAG_PVDO);

        if(pvdo && ctx){
            system_fild_t *fild = (system_fild_t *)ctx;
            fild->pvd_below = 1;
        }
        /* Clear PVD exti pending bit */
        __HAL_PWR_PVD_EXTI_CLEAR_FLAG();
    }
}

static void read_cmd_uart_state_transition_action(void *currentStateData, struct event *event, void *newStateData){
    UNUSED(currentStateData);
    UNUSED(event);
    struct cmd_domain_data_s * data_domain = (struct cmd_domain_data_s *)newStateData;
    if(data_domain->cmd_uart_handle == NULL){
        usart1_initialize(&data_domain->cmd_uart_handle, data_domain->sys_config->system.uart_baudrate);
        usart1_start_receive(data_domain->cmd_uart_handle);
    }
}


static void idle_state_transition_action(void *currentStateData, struct event *event, void *newStateData){
    UNUSED(currentStateData);
    UNUSED(newStateData);
    struct idle_domain_data_s * data_domain = (struct idle_domain_data_s *)newStateData;
    event->type = eTYPE_NOEVENT;
    if(data_domain->fild->pvd_below){
        event->type = eTYPE_ERROR;
        goto return_section;
    }
    if(gpio_get_pps_level()){
        event->type = eTYPE_GPS;
        goto return_section;
    }

    if(ads127_bsp_availablle(NULL) > 0){
        event->type = eTYPE_ANALOG;
        goto return_section;
    }

    if(usart1_bytes_available(data_domain->cmd_domain->cmd_uart_handle)){
        event->type = eTYPE_CMD;
        goto return_section;
    }

    if(gpio_cmd_detect()){
        event->type = eTYPE_USB;
        goto return_section;
    }

    return_section:
    return;
}

static void gps_time_state_transition_action(void *currentStateData, struct event *event, void *newStateData){
    UNUSED(currentStateData);
    UNUSED(event);

    struct gps_domain_data_s * data_domain = (struct gps_domain_data_s *)newStateData;
    struct idle_domain_data_s * idle_domain = (struct idle_domain_data_s *)currentStateData;

    if((data_domain->gps_uart_handle == NULL)){
        lpuart1_initialize(&data_domain->gps_uart_handle, data_domain->sys_config->gps.baudrate);
    }else{
#if(0)
        if(lpuart1_is_start_receive(data_domain->gps_uart_handle) == false){
            lpuart1_start_receive(data_domain->gps_uart_handle);  /// 开始 接收
        }
#else

#endif
    }
    if(ads127_bsp_is_start()){     /// 检查采样情况
        ads127_bsp_stop();
        core_mco_disable();
        if(ads127_bsp_availablle(NULL)){
            ads127_bsp_write_file(&idle_domain->analog_domain->ads_data_file);
            f_close(&idle_domain->analog_domain->ads_data_file);
        }
    }
    idle_domain->fild->analog = 0;
    gpio_buzzer_off();
}

static void analog_state_transition_action(void *currentStateData, struct event *event, void *newStateData){
    if(stateM_currentState(&fsm) == &idle_state){
        struct idle_domain_data_s * idle_domain = (struct idle_domain_data_s *)currentStateData;

        if(idle_domain->gps_domain->gps_uart_handle){
            lpuart1_stop_receive(idle_domain->gps_domain->gps_uart_handle);
            lpuart1_deinitialize(&idle_domain->gps_domain->gps_uart_handle);
        }
        if(idle_domain->cmd_domain->cmd_uart_handle){
            usart1_stop_receive(idle_domain->cmd_domain->cmd_uart_handle);
            usart1_deinitialize(&idle_domain->cmd_domain->cmd_uart_handle);
        }
    }
}

static bool check_event(void *check_event, struct event *event){
    system_event_def_t check = (system_event_def_t)(void *)(intptr_t)check_event;
    bool ret = false;

    if(event->type == eTYPE_ANALOG){
        switch(check){
            case EVENT_ANALOG_BUFFER_FULL:
                if(ads127_bsp_availablle(NULL) > 0) ret = true;
                break;
            case EVENT_ANALOG_STOP:
                ret = ads127_bsp_is_start();
                ret = !ret;
                break;
            default:
                ret = false;
                break;
        }
    }else if(event->type == eTYPE_USB){
        struct cmd_domain_data_s *handle = (struct cmd_domain_data_s *)event->data;
        switch(check){
            case EVENT_UART2USB_DATA_READY:
                if(usart1_bytes_available(NULL)) ret = true;
                break;
            case EVENT_UART2USB_CONNECT:
                if(handle->cmd_uart_handle == NULL) ret = true;
                break;
            case EVENT_UART2USB_DISCONNECT:
                if(handle->cmd_uart_handle) ret = true;
                break;
            default:
                ret = false;
                break;
        }
    }else if(event->type == eTYPE_GPS){
        switch(check){
            case EVENT_GPS_PPS:
                if(gpio_get_pps_level()) ret = true;
                break;
            default:
                ret = false;
                break;
        }
    }else if(event->type == eTYPE_SD){
        switch(check){
            case EVENT_SD_CONNECT:
                break;
            case EVENT_SD_DISCONNECT:
                break;
            default:
                ret = false;
                break;
        }
    }else if(event->type == eTYPE_ERROR){
        ret = true;
    }else{
        ret = false;
    }
    return ret;
}

static void entry_self_test_state_handler(void *state_data, struct event *event){

    struct idle_domain_data_s * data_domain = (struct idle_domain_data_s *)state_data;
    int err = 0;

    data_domain->fild->self_test = 0;

    ads127_bsp_reset_pin_initial(IO_ADS_RESET_PORT, IO_ADS_RESET_PIN);
    ads127_bsp_start_pin_initial(IO_ADS_START_PORT, IO_ADS_START_PIN);
    ads127_bsp_drdy_pin_initial(IO_ADS_DRDY_PORT, IO_ADS_DRDY_PIN);

    gpio_ts3a_mcu();

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
    err = FATFS_SD_Init(&sd_fs_handle);  /// \brief 挂载 SD
    if(err){
        gpio_buzzer_on();
        HAL_Delay(5000);
        gpio_buzzer_off();
        HAL_NVIC_SystemReset();
        while(1);
    }
    log_file_create(NULL, NULL);   /// 在默认位置创建日志文件

    if(1){
        err = parse_ini_file_from_sd(sd_root_path, data_domain->sys_config);
        if(err){
            log_printf("无法加载配置文件,以默认配置运行\n");
        }
        log_printf("系统串口波特率: %lubps, GPS串口波特率:%lubps, 采样时钟:%luHz, 采样文件限制类型:%d, 采样文件限制数值%d\n", data_domain->sys_config->system.uart_baudrate, data_domain->sys_config->gps.baudrate, data_domain->sys_config->ads.clk, data_domain->sys_config->ads.file.limit_type, data_domain->sys_config->ads.file.limit);
        save_ini_file_to_sd(sd_root_path, data_domain->sys_config);
    }

    if(1){  /// \brief 检测 Analog 采样 ADS127L01
        analog_data_domain.clk = data_domain->sys_config->ads.clk;
        ads127_bsp_stop();
        core_mco_enable(analog_data_domain.clk);
        st_spi1_init(&analog_data_domain.ads_spi_handle[0]);
        ads127_bsp_reset();
        ads127_driver_initialize(analog_data_domain.ads_spi_handle[0], IO_ADS_CS_PORT , IO_ADS_CS_PIN);
        uint32_t device_id = 0;
        device_id = ads127_get_id(&analog_data_domain.device);
        if(device_id & 0x03){
            FIL cal = { 0 };
            char cal_txt[32] = {'\0'};
            uint32_t ofc = 0, fsc = 0x8000;

            ads127_get_mode(&data_domain->analog_domain->device);
            HAL_Delay(1);
            ads127_get_configure(&data_domain->analog_domain->device);

            log_printf("检测到 ADC Chip, 设备ID: %#x. \n", device_id);
            log_printf("ADC 硬件配置[采样时钟 %luHz 过采样因子 %d 数据接口 %d,%d 运行模式 %d 数字滤波器 %d 偏移校准值 %#X 增益校准值 %#X]\n", data_domain->analog_domain->clk, ads127_get_osr(&data_domain->analog_domain->device), data_domain->analog_domain->device.mode.format, data_domain->analog_domain->device.mode.fsmode, data_domain->analog_domain->device.mode.hr, data_domain->analog_domain->device.mode.filter, data_domain->sys_config->ads.offset_calibration, data_domain->sys_config->ads.gain_calibration);
#if(1)
            if((data_domain->sys_config->ads.offset_calibration != UINT32_MAX)) goto config_offset_section;
            ofc = ads127_bsp_offset_calibration();
            if(ofc == 0){
                log_printf("偏移校准失败.\n");
                gpio_buzzer_on();
                HAL_Delay(1000);
            }else{
                data_domain->sys_config->ads.offset_calibration = ofc;
                save_ini_file_to_sd(sd_root_path, data_domain->sys_config);
                log_printf("偏移校准成功[OFFSET:%#X].\n", ofc);
            }
            config_offset_section:
            data_domain->analog_domain->device.ofc.val = data_domain->sys_config->ads.offset_calibration;

#define ADS_GAIN_CALIBRATION_FILE_PATH             "calibration/gain.txt"
            if((data_domain->sys_config->ads.gain_calibration != UINT32_MAX)) goto config_gain_sectiion;
            f_chdir(sd_root_path);
            err = f_open(&cal, ADS_GAIN_CALIBRATION_FILE_PATH, FA_READ | FA_OPEN_EXISTING);
            if(err){
                goto config_gain_sectiion;
            }
            err = (uint32_t)f_gets(cal_txt, sizeof cal_txt, &cal);
            if(err == 0){
                err = 1;
                f_close(&cal);
                goto config_gain_sectiion;
            }else{
                err = 0;
            }
            f_close(&cal);
            fsc = strtoul(cal_txt, NULL, 0);   /// 16进制数字, 补码格式
            fsc = ads127_bsp_gain_calibration(fsc);
            if(fsc == 0x8000){
                log_printf("增益校准失败.\n");
                gpio_buzzer_on();
                HAL_Delay(1000);
                goto config_gain_sectiion;
            }else{
                data_domain->sys_config->ads.gain_calibration = fsc;
                save_ini_file_to_sd(sd_root_path, data_domain->sys_config);
                log_printf("增益校准成功[OFFSET:%#X].\n", fsc);
            }
            config_gain_sectiion:
            if(err){
                log_printf("无法加载增益校准配置.\n");
            }

            data_domain->analog_domain->device.fsc.val = (uint16_t)data_domain->sys_config->ads.gain_calibration;

#undef ADS_GAIN_CALIBRATION_FILE_PATH
#endif

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
            while(1);
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
    log_printf("系统自检通过.\n");
    data_domain->fild->self_test = 1;      /// 自检通过
    event->type = eTYPE_NOEVENT;
}

static void entry_idle_state_handler(void *state_data, struct event *event){
    idle_state_transition_action(state_data, event, state_data);
}

static void entry_read_cmd_uart_state_handler(void *state_data, struct event *event){
    struct cmd_domain_data_s *data_domain = (struct cmd_domain_data_s *)state_data;
    uint8_t *buffer = NULL;
    uint16_t rb = 0;
    rb = usart1_bytes_available(data_domain->cmd_uart_handle);
    if(!rb) return;
    buffer = (uint8_t *)malloc(rb + 1);
    rb = usart1_read_bytes(data_domain->cmd_uart_handle, buffer, rb + 1, 0);

    cmd_data_frame_handle handle = (cmd_data_frame_handle)buffer;

    free(buffer);
    event->type = eTYPE_NOEVENT;
}

static void entry_parse_cmd_state_handler(void *state_data, struct event *event){
    UNUSED(state_data);
    event->type = eTYPE_NOEVENT;
}

static void entry_gps_time_state_handler(void *state_data, struct event *event){
#define NMEA_BUFFER_SIZE               100
    struct gps_domain_data_s *data_domain = (struct gps_domain_data_s *)state_data;
#if(0)
    char buf[NMEA_BUFFER_SIZE + 1] = {'\0'}, nmea_zda[NMEA_BUFFER_SIZE] = {'\0'};
    int sfild = -1, pfild = -1, rb = 0, ni = 0;
    struct minmea_sentence_zda zda = { 0 };
#else
    gps_nmea_parse_zda_t zda_parse = {.sentence = {'\0'}, .char_num = 0, .parse = 0, .zda = {{0}, {0}, 0, 0}};
#endif
    char ok = 0;
    int parse_res = 0;
    uint32_t tick = 0, start_tick = 0, pps = 0;

    rtc_date_time_t time = { 0 };
    uint32_t bcd_time = 0x00000000, bcd_date = 0x00002101;
    u32_st_rtc_time_bcd_format_handle bcd_time_handle = NULL;
    u32_st_rtc_date_bcd_format_handle bcd_date_handle = NULL;

    bcd_time_handle = (u32_st_rtc_time_bcd_format_handle)&bcd_time;
    bcd_date_handle = (u32_st_rtc_date_bcd_format_handle)&bcd_date;

    /// NMEA报文在 PPS 脉冲上升沿前发送完毕
#if(0)

#else
    lpuart1_install_char_process_handler(&gps_nmea_parse_zda_handler, &zda_parse);
    lpuart1_start_receive(data_domain->gps_uart_handle);  /// 开始 接收
#endif
    LL_RTC_DisableWriteProtection(RTC);
    st_rtc_enter_initmode();

    start_tick = HAL_GetTick();
    tick = start_tick;

    do{
        tick = HAL_GetTick();
    }while(gpio_get_pps_level() && ((tick - start_tick) <= 500)); /// 等待 PPS 信号变低, PPS 宽度 100ms
    if((tick - start_tick) > 500) goto end_section;
    pps = data_domain->pps;
#if(0)
    do{
        rb = lpuart1_read_bytes(data_domain->gps_uart_handle, buf, NMEA_BUFFER_SIZE, 500);   /// 读取 NMEA 数据
        if(!rb) break;
        buf[rb] = '\0';
        for(int i = 0; i < rb; i++){
            if(sfild == -1){
                if((buf[i] == '$') && (strncmp("ZDA", &buf[i + 3], 3) == 0)){
                    sfild = i;
                    strncpy(nmea_zda, &buf[i], 6);
                    ni += 6;
                }
            }else{
                nmea_zda[ni++] = buf[i];
                if((buf[i] == '\n') && (buf[i - 1] == '\r')){
                    pfild = i;
                    break;
                }
            }
        }
    }while(pfild <= 0);

    lpuart1_stop_receive(data_domain->gps_uart_handle);

    if((sfild < 0) || (pfild < 0)) goto end_section;
    nmea_zda[ni + 1] = '\0';
#else

    start_tick = HAL_GetTick();
    tick = start_tick;
    do{
        tick = HAL_GetTick();
    }while((zda_parse.parse == 0) && ((tick - start_tick) <= 1500));

    lpuart1_stop_receive(data_domain->gps_uart_handle);
    lpuart1_install_char_process_handler(NULL, NULL);

    if((tick - start_tick) > 1500) goto end_section;

    parse_res = (int)minmea_parse_zda(&zda_parse.zda, zda_parse.sentence);

#endif

    if(!parse_res) goto end_section;

    time.time.ssecond = zda_parse.zda.time.microseconds;
    time.time.second = zda_parse.zda.time.seconds;
    time.time.minute = zda_parse.zda.time.minutes;
    time.time.hour = zda_parse.zda.time.hours;
    time.time.hour += 8;
    time.date.day = zda_parse.zda.date.day;
    time.date.month = zda_parse.zda.date.month;
    time.date.year = zda_parse.zda.date.year;

//    if(gpio_get_pps_level() == 0){
//        while(gpio_get_pps_level() == 0);
//    }
//    while(gpio_get_pps_level() == 1);

//    rtc_add_seconds(&time, data_domain->pps - pps + 1);

    bcd_time_handle->sec = __LL_RTC_CONVERT_BIN2BCD(time.time.second);
    bcd_time_handle->min = __LL_RTC_CONVERT_BIN2BCD(time.time.minute);
    bcd_time_handle->hour = __LL_RTC_CONVERT_BIN2BCD(time.time.hour);
    bcd_time_handle->pm = 0;  /// 24 小时制

    bcd_date_handle->day = __LL_RTC_CONVERT_BIN2BCD(time.date.day);
    bcd_date_handle->wdu = __LL_RTC_CONVERT_BIN2BCD(time.date.weekday);
    bcd_date_handle->mon = __LL_RTC_CONVERT_BIN2BCD(time.date.month);
    bcd_date_handle->year = __LL_RTC_CONVERT_BIN2BCD(time.date.year - 2000);

    gps_pps_irq_disable(); /// 停止响应 PPS IRQ

    if((gpio_get_pps_level() == 0)){
        while(gpio_get_pps_level() == 0);  /// 等待当前时间 PPS 脉冲
    }

    RTC->TR = bcd_time;
    RTC->DR = bcd_date;
    ok = 1;
    end_section:
    st_rtc_exit_initmode();
    LL_RTC_EnableWriteProtection(RTC);

    if(ok){
        FIL time_file = { 0 };
        gpio_buzzer_on();
        int err = f_open(&time_file, TIME_FILE_PATH, FA_WRITE | FA_OPEN_ALWAYS | FA_OPEN_APPEND);
        if((err == FR_OK) || (err == FR_EXIST)){
            char buf[32] = {'\0'};
            err = rtc_time2str(&time, buf, 64);
            buf[err++] = '\n';
            f_write(&time_file, zda_parse.sentence, zda_parse.char_num, NULL);
            f_write(&time_file, buf, err, NULL);
            f_close(&time_file);
        }
        HAL_Delay(10000);
        gpio_buzzer_off();
    }
    gps_pps_irq_enable();
    event->type = eTYPE_NOEVENT;
#undef NMEA_BUFFER_SIZE
}

static void entry_analog_state_handler(void* state_data, struct event *event){

    if(event->type != eTYPE_ANALOG) return;
    if(ads127_bsp_is_start()) return;

    struct analog_domain_data_s * data_domain = (struct analog_domain_data_s *)state_data;
    ads_data_init_t init = { 0 };
    uint32_t err = 0, ds = 0, rate = 0, osr = 256;
    char name[64] = { '\0' };

    gpio_buzzer_on();
    core_mco_enable(data_domain->clk);
    if(data_domain->ads_spi_handle[0] == NULL){
        st_spi1_init(&data_domain->ads_spi_handle[0]);
    }
#if(1)
    if(1){
        uint8_t tconfig = 0;
        uint32_t tofc = 0, tfsc = 0;
        ads127_configure_ofc(&data_domain->device);
        HAL_Delay(1);
        tofc = ads127_get_ofc(&data_domain->device);
        HAL_Delay(1);
        ads127_configure_fsc(&data_domain->device);
        HAL_Delay(1);
        tfsc = ads127_get_fsc(&data_domain->device);
        HAL_Delay(1);
        data_domain->device.config.ofc = 1;
        data_domain->device.config.fsc = 0;
        ads127_configure(&data_domain->device);
        HAL_Delay(1);
        tconfig = ads127_get_configure(&data_domain->device);

        log_printf("ADC Chip:[配置:%#x][偏移校准:%#x][增益校准:%#x]\n", tconfig, tofc, tfsc);
    }

#endif
    if(data_domain->ads_data_file.obj.fs){
        f_close(&data_domain->ads_data_file);
    }
    err = ads127_bsp_create_file(&data_domain->ads_data_file, &data_domain->file_create_time);
    if((err != FR_OK) && (err != FR_EXIST)){
        ads127_bsp_stop();
        core_mco_disable();
        fs_error_string(name, sizeof(name), err);
        log_printf("无法创建采样数据文件, 停止采样.[ERROR:%s]\n", name);
        gpio_buzzer_on();
        HAL_Delay(3000);
        gpio_buzzer_off();
        HAL_NVIC_SystemReset();
        return;
    }
    ds = rtc_time2str(&data_domain->file_create_time, name, sizeof(name));
    strncpy(&name[ds], ".bin", sizeof(name) - ds);
    log_printf("采样数据文件 %s 创建成功\n", name);

    init.clk = data_domain->clk;
    init.config.val = data_domain->device.config.val;
    init.mode.val = data_domain->device.mode.val;
    ads127_bsp_read_init(&init, NULL, NULL);

    osr = ads127_get_osr(&data_domain->device);
    osr = (osr != -1) ? osr : 256;
    rate = (data_domain->clk) / osr;
    data_domain->th = data_domain->sys_config->ads.file.limit_type ? (data_domain->sys_config->ads.file.limit << 10) : (rate * 4 * data_domain->sys_config->ads.file.limit);   /// 计算文件限制大小

    ads127_bsp_drdy_isr_install(IO_ADS_DRDY_PORT, IO_ADS_DRDY_PIN, ads127_bsp_read_data_from_isr, ads127_driver_handle());

    ads127_bsp_start();

    if(!ads127_bsp_availablle(NULL)){
        event->type = eTYPE_NOEVENT;
    }
    data_domain->fild->buzzer = 1;
    data_domain->buzzer_number = ANALOG_BUZZER_NUMBER;
    log_printf("采样开始......\n");
    gpio_buzzer_off();
}

static void entry_analog_write_file_state_handler(void* state_data, struct event *event){
    struct analog_domain_data_s *data_domain = (struct analog_domain_data_s *)state_data;
    uint32_t bl = 0, err = 0;
//    bl = ads127_bsp_availablle(NULL);
//    if(bl){
        err = ads127_bsp_write_file(&data_domain->ads_data_file);
        if(err){
            data_domain->error = err;
            data_domain->err_counter += 1;
            gpio_buzzer_toggle();
            if(data_domain->err_counter > 125){
                event->type = eTYPE_ERROR;
                return;
            }
        }else{
            data_domain->error = 0;
            if(data_domain->err_counter) data_domain->err_counter -= 1;
        }
        exit_analog_write_file_sate_handler(state_data, event);
//    }
//    idle_state_transition_action(state_data, event, idle_state.data);
    event->type = eTYPE_NOEVENT;
}

static void entry_error_state_handler(void* state_data, struct event *event){
    struct idle_domain_data_s * data_domain = (struct idle_domain_data_s *)state_data;
    if(event->type != eTYPE_ERROR) return;

    if(data_domain->fild->pvd_below){
        if(ads127_bsp_is_start()){
            ads127_bsp_stop();
            core_mco_disable();
            f_close(&data_domain->analog_domain->ads_data_file);
        }
        log_printf("电池电压不足, 系统关机\n");
        log_file_close();
        core_enter_power_mode(LL_PWR_MODE_SHUTDOWN);
        while(1);
    }
    if(stateM_previousState(&fsm) == (&analog_write_file_state)){
        char *err_str = NULL;
        err_str = fs_error_string(NULL, NULL, data_domain->analog_domain->error);
        log_printf("模拟采样发生错误[ERROR:%s]\n", err_str);
        log_printf("即将重启设备\n");
        log_file_close();
        f_close(&data_domain->analog_domain->ads_data_file);
    }else{
        log_printf("状态机运行发生错误, 前状态为 %#X \n.", (uint32_t)stateM_previousState(&fsm));
        log_printf("即将重启设备\n");
        log_file_close();
        f_close(&data_domain->analog_domain->ads_data_file);
    }
    HAL_NVIC_SystemReset();
    event->type = eTYPE_NOEVENT;
    while(1);
}

static void exit_analog_write_file_sate_handler(void* state_data, struct event *event){
    struct analog_domain_data_s *data_domain = (struct analog_domain_data_s *)state_data;
    uint64_t file_size = 0;

    char name[64] = { '\0' };
    uint32_t ds = 0, err = 0;
    if(!data_domain->ads_data_file.obj.fs) return;

    file_size = f_size(&data_domain->ads_data_file);
    if(file_size < data_domain->th) return;
    if((data_domain->fild->buzzer) && (data_domain->buzzer_number)){
        gpio_buzzer_on();
        data_domain->buzzer_number--;
    }

    f_close(&data_domain->ads_data_file);
    err = ads127_bsp_create_file(&data_domain->ads_data_file, &data_domain->file_create_time);
    if((err != FR_OK) && (err != FR_EXIST)){
        ads127_bsp_stop();
        core_mco_disable();
        fs_error_string(name, sizeof(name), err);
        log_printf("无法创建采样数据文件, 停止采样[ERROR: %s]\n", name);
        log_file_close();
        HAL_Delay(3000);
        gpio_buzzer_off();
        HAL_NVIC_SystemReset();
        return;
    }
    ds = rtc_time2str(&data_domain->file_create_time, name, sizeof(name));
    strncpy(&name[ds], ".bin", sizeof(name) - ds);
    log_printf("采样数据文件 %s 创建成功\n", name);
    gpio_buzzer_off();
}

static void exit_gps_time_state_handler(void* state_data, struct event *event){
    UNUSED(event);
    struct gps_domain_data_s *data_domain = (struct gps_domain_data_s *)state_data;
    if(data_domain->gps_uart_handle){
        lpuart1_stop_receive(data_domain->gps_uart_handle);
        lpuart1_deinitialize(&data_domain->gps_uart_handle);
        data_domain->gps_uart_handle = NULL;
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
    assert_failed((uint8_t *)__FILE__, (uint32_t)__LINE__);

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

#pragma clang diagnostic pop
