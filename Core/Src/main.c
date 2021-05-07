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
#include "main.h"
//#include "adc.h"
#include "dma.h"

//#include "i2c.h"
//#include "lptim.h"
//#include "rtc.h"
//#include "sdmmc.h"
//#include "spi.h"
//#include "tim.h"
//#include "usart.h"
//#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "gps.h"
//#include "fatfs.h"
#include "led_blink.h"
#include "config_ini.h"
#include "rtc_pcf85063a.h"
#include "imu_bhi160.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define MSI_TO_MSI                     0x11U
#define MSI_TO_HSE                     0x12U
#define MSI_TO_HSI                     0x13U
#define HSE_TO_HSE                     0x22U
#define HSE_TO_MSI                     0x21U
#define HSE_TO_HSI                     0x23U
#define HSI_TO_HSI                     0x33U
#define HSI_TO_HSE                     0x32U
#define HSI_TO_MSI                     0x31U

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SD_CONNECT_TO_MCU              0x01
#define SD_CONNECT_TO_USB              0x00

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CMD_STATUS_INIT                0x01
#define CMD_STATUS_CONNECT             0x02

#define GPS_STATUS_INIT                0x01
#define GPS_STATUS_ANT_OK              0x02
#define GPS_STATUS_PPS_OK              0x04
#define GPS_STATUS_TIME                0x08

#define ADS_STATUS_INIT                0x01
#define ADS_STATUS_START               0x02
#define ADS_STATUS_RESET               0x04
#define ADS_STATUS_STOP                0x08

#define TIM_STATUS_INIT                0x01
#define TIM_STATUS_SET                 0x02
#define TIM_STATUS_START               0x04
#define TIM_STATUS_STOP                0x08

#define ADC_STATUS_INIT                0x01
#define ADC_STATUS_START               0x02
#define ADC_STATUS_STOP                0x04

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

#define RAM2_START_ADDR               0x10000000U

/* USER CODE BEGIN PV */

static __aligned(4) uint8_t workBuffer[FF_MAX_SS] = { 0 };
static FATFS SDFatFS;    /* File system object for SD logical drive */
static FIL log_file = { 0 };
static FIL ads_data_file = { 0 };
static FIL ads_file_name[128] = { 0 };
static const char* ads_data_file_fmt_suffix[] = {".txt", ".bin"};

__IO uint64_t gps_get_unix_timestamp[2] = {0};
__IO uint32_t gps_UTCTime[2] = {0};
__IO float gps_latitude[2] = {0.000f};   // 纬度
__IO float gps_longitude[2] = {0.000f};  // 经度

uint32_t system_cmd_buffer_size = 128;


// 以下变量顺序不能随意改变
// System Config
__IO uint32_t system_hse_value __attribute__((section(".data"))) = 16000000;
__IO uint32_t system_core_freq __attribute__((section(".data"))) = 32000000;
__IO uint32_t system_rco_div __attribute__((section(".data"))) = 8;
__IO uint32_t system_uart_baudrate __attribute__((section(".data"))) = 115200;
__IO uint32_t system_cmd_connect_time __attribute__((section(".data"))) = 300;        // 5分钟
__IO uint32_t system_battery_voltage __attribute__((section(".data"))) = 14400;       // 电池额定电压
__IO uint32_t system_battery_cutoff_voltage __attribute__((section(".data"))) = 7000; // 放电截至电压
__IO uint32_t  system_battery_thold_voltage __attribute__((section(".data"))) = 2;     // 停止工作电压阈值, 2%
__IO int32_t  system_battery_error_voltage __attribute__((section(".data"))) = -50;   // 电池电压修正参数
// SD-TF Config
__IO uint32_t sd_format __attribute__((section(".data"))) = 0;
__IO uint32_t sd_format_file_system __attribute__((section(".data"))) = 0x02;
__IO uint32_t sd_file_size_limit __attribute__((section(".data"))) = 4096;
// ADS Config
__IO uint32_t ads_conver_rate __attribute__((section(".data"))) = 2000;
__IO uint32_t ads_mode __attribute__((section(".data"))) = 0;
__IO uint32_t ads_start_time __attribute__((section(".data"))) = 0;
__IO uint32_t ads_offset_calibration __attribute__((section(".data"))) = 0;
__IO uint32_t ads_gain_calibration __attribute__((section(".data")))= 0;
__IO uint32_t ads_data_file_format __attribute__((section(".data")))= 1;
__IO uint32_t ads_osr __attribute__((section(".data"))) = 32;
// GPS Config
// index:0-start, 1-stop
__IO uint32_t gps_check __attribute__((section(".data"))) = 1;
__IO uint32_t gps_startup __attribute__((section(".data"))) = 1;
__IO uint32_t gps_next_startup_time __attribute__((section(".data"))) = 604800;
__IO uint32_t gps_uart_baudrate __attribute__((section(".data"))) = 9600;
__IO uint32_t gps_up_rate __attribute__((section(".data"))) = 1000;
__IO uint32_t gps_mode __attribute__((section(".data"))) = 2;
// RTC Config
__IO uint32_t rtc_ex_input_freq __attribute__((section(".data"))) = 32768;
__IO uint32_t rtc_ex_cal_offset __attribute__((section(".data"))) = 0x06;   // 此值使用 LL_RTC_BKP_DR0 备份寄存器备份
__IO uint32_t rtc_ex_i2c_speed = 0x00807CB9;

__IO uint32_t task_fild = 0, task_fild_bak = 0;

static int8_t local_time_zone = 127;

static __IO uint8_t led_control = 0;
static __IO float led_high_time = 0.030f; /* time = (led_high_time * 1000 / led_blink_freq) ms */
static __IO float led_blink_freq = 1.000f;

static __IO uint8_t adc_control = 0;

__IO uint8_t ads_control = 0;

static __IO uint8_t usb_dection = 0;

static __IO uint8_t gps_status = 0;
static __IO uint32_t gps_pps_cunt = 0;

static __IO uint8_t cmd_status = 0;

#define SYS_CAL_CTRL_SET                   0x10
#define SYS_CAL_CTRL_START                 0x08
#define SYS_CAL_CTRL_STOP                  0x04
#define SYS_CAL_CTRL_TEST                  0x02
#define SYS_CAL_STATUS_CPLT                0x01

#define SYS_CAL_SIZE                       (33)
static __IO uint8_t system_calibration_ctrl = 0x00;                    // 此值使用 LL_RTC_BKP_DR1 备份寄存器备份
static uint32_t *system_calibrate_offset = NULL;    // 使用 LL_RTC_BKP_DR2 备份寄存器备份偏移值
                                                    // 使用LL_RTC_BKP_DR3 存储 1ms 定时校准值

static __IO uint32_t system_utc_time = 0;
static __IO uint32_t system_tick_cunt = 0;
/* USER CODE END PV */

/**
 * ini parse
 */
typedef struct {
    #define INI_CFG(s, n, v_default) int32_t s##_##n;
    #include "system_config.def"
} configuration_t;


configuration_t config __attribute__((section(".data"))) = {
    #define INI_CFG(s, n, v_default) v_default,
    #include "system_config.def"
};

int f_ini_parse(const char* filename, ini_handler handler, void* user);
int f_ini_parse_file(FIL* file, ini_handler handler, void* user);
int f_ini_dump_file(FIL* file, ini_handler handler, void* user);

/* process a line of the INI file, storing valid values into config struct */
int ini_parse_handler(void *user, const char *section, const char *name, const char *value);

int ini_parse_handler(void *user, const char *section, const char *name, const char *value){
	configuration_t* pconfig = (configuration_t*)user;
    char key_name [32] = { 0 };
    strcpy(key_name, &name[strlen(section) + 1]);
#define INI_CFG(s, n, v_default) if((strcmp(section, #s) == 0) && (strcmp(key_name, #n) == 0)) \
	                                 pconfig->s##_##n = atol(value);

   #include "system_config.def"

   return 1;
}

int f_ini_parse_file(FIL* file, ini_handler handler, void* user){
	return ini_parse_stream((ini_reader)f_gets, file, handler, user);
}

int f_ini_parse(const char* filename, ini_handler handler, void* user){
	FIL file = {0};
	int error = -1;

	error = f_open(&file, filename, FA_READ | FA_OPEN_EXISTING);
	if(error != 0) return error;
	error = f_ini_parse_file(&file, handler, user);
	f_close(&file);
	return error;
}

int parse_ini_file_from_sd(const char* _root_path, ini_handler handler, void* user){
	char dir_path[32] = {'\0'};
	FIL _config_file = { 0 };
	int res = FR_OK;
	strcpy(dir_path, _root_path);  // 0:/
	strcat(dir_path, "config");    // 0:/config
	res = f_chdir(_root_path);     // 切换到根目录

	res = f_mkdir(dir_path);
    if(!((res == FR_OK) || (res == FR_EXIST))) return -1;

    res = f_chdir(dir_path);  // 切换到配置目录
	res = f_open(&_config_file, "config.ini", FA_OPEN_EXISTING | FA_READ);
	if((res != FR_OK) && (res == FR_NO_FILE)){  // // config.ini文件不存在
		res = f_open(&_config_file, "default_config.ini", FA_READ | FA_OPEN_EXISTING);   // 尝试打开默认配置文件
		if((res != FR_OK) && (res == FR_NO_FILE)){  // 文件不存在则创建
			res = create_default_config_file("default_config.ini");
			if(res != 0) return res;
			res = f_ini_parse("default_config.ini", handler, user);
			return res;
		}
	}
    res = f_ini_parse_file(&_config_file, handler, user);
    res = f_close(&_config_file);
	res = f_chdir(_root_path);     // 切换到根目录
	return res;
}

int f_ini_dump_file(FIL* file, ini_handler handler, void* user){
	FRESULT res = FR_OK;

    configuration_t* pconfig = (configuration_t*)user;
    char key_value [128] = { '\0' }, section[16] = {'\0'};

    UNUSED(handler);

#define INI_CFG(s, n, v_default) if(strncmp(&section[1], #s, strlen(#s))) \
	                             { \
                                     sprintf(section, "[%s]\r\n", #s); \
                                     res = f_write(file, section, strlen(section), NULL); \
	                             } \
                                 sprintf(key_value, "%s_%s = %ld\r\n", #s, #n, pconfig->s##_##n); \
                                 res = f_write(file, key_value, strlen(key_value), NULL);

    #include "system_config.def"

    return (int)res;
}

int save_ini_file_to_sd(const char* _root_path, ini_handler handler, void* user){
    char dir_path[32] = {'\0'}, tmp_path[32] = {'\0'};
//    DIR config_dir = { 0 };
    int res = FR_OK;
    FIL _config_file = { 0 };
    f_getcwd(tmp_path, 32);

    strcpy(dir_path, _root_path);  // 0:/
    strcat(dir_path, "config/config.ini");    // 0:/config
    res = f_open(&_config_file, dir_path, FA_CREATE_ALWAYS | FA_WRITE);
    if(res == FR_DISK_ERR) return res;
    if((res != FR_OK) && !((res == FR_NO_PATH) || (res == FR_NO_FILE))){
        res = f_chdir(_root_path);     // 切换到根目录
    	res = f_mkdir("config");
    	if(!((res == FR_OK) || (res == FR_EXIST) || (res == FR_DISK_ERR))) return res;   // 目录不存在 并且无法创建目录
    	res = f_open(&_config_file, dir_path, FA_CREATE_ALWAYS | FA_WRITE);
    }
    if(res) return res;
    res = f_ini_dump_file(&_config_file, handler, user);
    res += f_close(&_config_file);
    res += f_chdir(tmp_path);     // 切换到根目录
    return res;
}

int find_chr(const char* str, char c){
	int pos = -1;
	do{
		pos += 1;
		if(*(str + pos) == c) return pos;
	}while(*(str+pos) != '\0');
	return -1;
}

int parse_iso8601_time_from_string(const char *str, struct tm* _tm){  // yyyy-mm-ddThh:mm:ss
   char tmp[8] = {'\0'};
   int pos = -1;
   unsigned char fild = 0x00;
   char t = 0;
   char i = 0, k = 0;

   do{
	   pos += 1;
	   t = (char)*(str + pos);
       if((t >= '0') && (t <= '9')){  // 数字
    	   tmp[i++] = t;
           continue;
       }else if((t == '-') || (t == ':') || (t == 'T') || (t == '\0') || (t == '\r') || (t == '\n')){
    	   tmp[i++] = '\0';
           fild += 1;
           k = strlen(tmp);
           switch(fild)
           {
           case 1:
               if(k == 4) _tm->tm_year = atoi(tmp) - 1900;
        	   break;
           case 2:
        	   if(k == 2) _tm->tm_mon = atoi(tmp);
        	   break;
           case 3:
        	   if(k == 2) _tm->tm_mday = atoi(tmp);
        	   break;
           case 4:
        	   if(k == 2) _tm->tm_hour = atoi(tmp);
        	   break;
           case 5:
        	   if(k == 2) _tm->tm_min = atoi(tmp);
        	   break;
           case 6:
        	   if(k == 2) _tm->tm_sec = atoi(tmp);
        	   break;
           }
           i = 0;
       }
   }while(t != '\0');
   return 0;
}

int32_t get_config(void* user, const char* section, const char* name){
	configuration_t* pconfig = (configuration_t*)user;

#define INI_CFG(s, n, v_default) if((strcmp(section, #s) == 0) && (strcmp(name, #n) == 0)) \
	                                 return pconfig->s##_##n;

    #include "system_config.def"

    return 1;
}

int32_t set_config(void* user, const char* section, const char* name, int32_t value){
	configuration_t* pconfig = (configuration_t*)user;

#define INI_CFG(s, n, v_default) if((strcmp(section, #s) == 0) && (strcmp(name, #n) == 0)) \
                                     pconfig->s##_##n = value;

    #include "system_config.def"

    return 1;
}

/**
 * ini parse end
 */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
static void LL_SystemClock_Config(uint32_t in_clock_index, uint32_t in_clk_freq);

/**
 * opt: 0: read, 1: modify
 */
static void read_config_from_sd(const char* _root_path, char* _config_section, void* _config, uint8_t opt);

static void save_config_to_sd(const char* _root_path);
static uint8_t cmd_scan(char* in_data, char* out_cmd, char* out_val, char* out_data);
static void sd_switch_connect(uint8_t in_device);
static uint32_t set_config_value(char* in_config_name, char* in_value);
static void pericdic_close_ads_file(void);
static void reset_sd(void);

static void gps_pps_cunt_callback(void);
static void sys_ms_calibration_callback(void);

static volatile uint32_t rtc_sd_timeout_count = 0;
static volatile uint32_t rtc_rads_timeout_count = 0;
static void rtc_timeout_callback(void);
void rtc_reset_sd_timeout(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
  /* USER CODE BEGIN 1 */
    volatile uint32_t ret = 0;

    char *cmd_buffer = NULL;   // 命令缓冲区
    char *cmd = NULL;
    char *cmd_val = NULL;
    char *cmd_data = NULL;
    uint32_t cmd_connect_tick = 0;
    uint32_t cmd_connect_time = 0;
//    uint32_t cmd_connect_timeout_bak = 0;
    char *gps_text_buffer = NULL;
  uint8_t gps_to_cmd = 0;
  uint8_t gps_ant_status = 0;

  uint8_t led_status = 0;
  uint8_t ads_status = 0;
//  uint8_t lptim_status = 0;
  uint8_t adc_status = 0;
  uint32_t adc_start_tick[2] = { 0 };

  uint32_t led_prio_start_tick = 0, led_last_tick = 0;
  int8_t battery_level = 0;
//  float mcu_temp = 25.00f;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */


  /* USER CODE BEGIN Init */
//  SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  /* USER CODE END Init */

  /* Configure the system clock */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOH);
  LL_GPIO_SetPinMode(GPIOH, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOH, LL_GPIO_PIN_1, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_ResetOutputPin(GPIOH, LL_GPIO_PIN_1);

  LL_AHB1_GRP1_DisableClockStopSleep(~(LL_AHB1_GRP1_PERIPH_DMA2 | LL_AHB1_GRP1_PERIPH_DMA1 | LL_AHB1_GRP1_PERIPH_SRAM1 | LL_AHB1_GRP1_PERIPH_FLASH));
  LL_AHB1_GRP1_EnableClockStopSleep(LL_AHB1_GRP1_PERIPH_DMA2 | LL_AHB1_GRP1_PERIPH_DMA1 | LL_AHB1_GRP1_PERIPH_SRAM1 | LL_AHB1_GRP1_PERIPH_FLASH);
  LL_AHB2_GRP1_DisableClockStopSleep(~(LL_AHB2_GRP1_PERIPH_GPIOA | LL_AHB2_GRP1_PERIPH_GPIOB | LL_AHB2_GRP1_PERIPH_GPIOC
		  | LL_AHB2_GRP1_PERIPH_GPIOD | LL_AHB2_GRP1_PERIPH_GPIOE | LL_AHB2_GRP1_PERIPH_GPIOF | LL_AHB2_GRP1_PERIPH_GPIOH
		  | LL_AHB2_GRP1_PERIPH_SRAM2));
  LL_AHB2_GRP1_EnableClockStopSleep(LL_AHB2_GRP1_PERIPH_GPIOA | LL_AHB2_GRP1_PERIPH_GPIOB | LL_AHB2_GRP1_PERIPH_GPIOC
		  | LL_AHB2_GRP1_PERIPH_GPIOD | LL_AHB2_GRP1_PERIPH_GPIOE | LL_AHB2_GRP1_PERIPH_GPIOF | LL_AHB2_GRP1_PERIPH_GPIOH
		  | LL_AHB2_GRP1_PERIPH_SRAM2);

  LL_AHB3_GRP1_DisableClockStopSleep(LL_AHB3_GRP1_PERIPH_ALL);
//  LL_AHB3_GRP1_EnableClockStopSleep();
  LL_APB1_GRP1_DisableClockStopSleep(~(LL_APB1_GRP1_PERIPH_PWR | LL_APB1_GRP1_PERIPH_LPTIM1));
  LL_APB1_GRP1_EnableClockStopSleep(LL_APB1_GRP1_PERIPH_PWR | LL_APB1_GRP1_PERIPH_LPTIM1);
  LL_APB1_GRP2_DisableClockStopSleep(~LL_APB1_GRP2_PERIPH_LPTIM2);
  LL_APB1_GRP2_EnableClockStopSleep(LL_APB1_GRP2_PERIPH_LPTIM2);
  LL_APB2_GRP1_DisableClockStopSleep(~(LL_APB2_GRP1_PERIPH_SPI1 | LL_APB2_GRP1_PERIPH_SDMMC1 | LL_APB2_GRP1_PERIPH_SYSCFG));
  LL_APB2_GRP1_EnableClockStopSleep(LL_APB2_GRP1_PERIPH_SPI1 | LL_APB2_GRP1_PERIPH_SDMMC1 | LL_APB2_GRP1_PERIPH_SYSCFG);

  LL_SystemClock_Config(MSI_TO_MSI, system_core_freq);   // 默认系统时钟
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  if(LL_RCC_IsEnabledRTC() == 0){
	  MX_RTC_Init();
  }

  MX_GPIO_Init();
  usb_dection_exti_disable();
  imu_exti_disable();

  systick_disable();
  systick_disable_ti_req();
  systick_selete_clk_div(LL_SYSTICK_CLKSOURCE_HCLK);
  systick_set_priority(0);
  systick_set_reload_value((1 << 24) - 1);

  mem_dma_init();

  HAL_Init();

  ads_bsp_power(RESET);
  ads_drdy_exti_disable();

#define TEST_GPIO    0
#if(TEST_GPIO)
  VGPS_Control(1);
  LED_ON();
  LED_OFF();
  sd_switch_connect(SD_CONNECT_TO_USB);
  ads_bsp_power(RESET);
  ads_bsp_power(SET);
  ads_bsp_selete_cs_by_index(0, RESET);
  ads_bsp_selete_cs_by_index(1, RESET);
  while(1);
#endif
  uint8_t _i2c_address[2] = { 0 };
  uint8_t _i2c_device_num = 0;

  i2c_init(I2C2);

  ret = I2C_Find_Address(I2C2, _i2c_address, &_i2c_device_num);
  if((ret == 0) && (_i2c_device_num)){
	  for(ret = 0; ret < _i2c_device_num; ret++){
		  if(_i2c_address[ret] == PCF_ADDRESS){
	          rtc_pcf_drv_init(I2C2_Read, I2C2_Write);
	          rtc_pcf_clkout(PCF_CLKOUT_DISABLE);
//	          rtc_pcf_clkout(PCF_CLKOUT_32768_HZ);
	          pcf_tm_t tm = {0};
	          rtc_pcf_get_date_time(&tm);
	          st_rtc_set_time(&tm);
		  }
		  if(_i2c_address[ret] == (0x28 << 1)){  // BHI160
			  int32_t bhi_ret = -1;
			  st_exti_irq_handler_register(EXTI15_10_IRQn, 12, imu_bhy_int_callback);
			  imu_exti_enable();

//			  bhi_ret = bhy_driver_init(&bhy1_fw);
//			  while(LL_GPIO_ReadInputPort(EXTI_IMU1_GPIO_Port) & EXTI_IMU1_Pin);
//			  while(!(LL_GPIO_ReadInputPort(EXTI_IMU1_GPIO_Port) & EXTI_IMU1_Pin));

//			  demo_sensor();

		  }else{
			  imu_exti_disable();
			  optimize_gpio_power(EXTI_IMU1_GPIO_Port, EXTI_IMU1_Pin);
		  }
	  }
  }else{
	  i2c_deinit(I2C2);
  }
  /* USER CODE BEGIN SysInit */
  sd_switch_connect(SD_CONNECT_TO_MCU);
  HAL_Delay(500);
  BSP_SD_ITConfig();

  ret = 0;
  if(FATFS_LinkDriver(&SD_Driver, sd_root_path) == 0){  // sd_root_path: "0:/"
	  if(BSP_SD_IsDetected() == SD_PRESENT){
//		  if(BSP_SD_Init() == MSD_OK){
			  ret = f_mount(&SDFatFS, (TCHAR const*)sd_root_path, 0);   // 立即挂载
        	  if(ret != FR_OK){
            	  if(ret == FR_NO_FILESYSTEM){
            		  MKFS_PARM mkfs_parm = { 0 };
            	  fatfs_no_file_system_section:
            		  mkfs_parm.fmt = FM_FAT32;
            		  ret = f_mkfs(sd_root_path, &mkfs_parm, (uint8_t*)workBuffer, sizeof(workBuffer));
            	  }
        	  }
        	  ret = FS_FileOperations();
        	  if(ret) goto fatfs_no_file_system_section;
        	  if(ret != 0){
        		  HAL_Delay(5 * 1000);

        		  goto end_system_reset;
        	  }
        	  ret = f_mkdir("log"); // 创建日志目录
        	  if(!((ret == FR_OK) || (ret == FR_EXIST))){
        		  HAL_Delay(5 * 1000);

        		  goto end_system_reset;
        	  }
              char _lp[16] = { 0 };
              strcpy(_lp, sd_root_path);
              strcat(_lp, "log/log.txt");

        	  ret = f_open(&log_file, _lp, FA_CREATE_ALWAYS | FA_WRITE);
        	  ret = f_log_printf(&log_file, "正在读取配置文件......\n");
        	  ret = f_close(&log_file);

//        	  read_config_from_sd(sd_root_path, NULL, NULL, 0);

        	  ret = parse_ini_file_from_sd(sd_root_path, ini_parse_handler, &config);
        	  if(ret){
            	  ret = f_open(&log_file, _lp, FA_CREATE_ALWAYS | FA_WRITE);
            	  ret = f_log_printf(&log_file, "配置文件读取失败!将以默认配置运行!\n");
            	  ret = f_close(&log_file);
        	  }
        	  ret = save_ini_file_to_sd(sd_root_path, NULL, &config);

        	  // 获取剩余容量, 决定是否格式化
        	  uint64_t fre_clust = 0, fre_sect = 0;
        	  uint64_t tot_sect = 0;
        	  FATFS *_fs = NULL;
        	  ret = f_getfree(sd_root_path, &fre_clust, &_fs);
              tot_sect = (((_fs->n_fatent - 2) * _fs->csize) >> 1) >> 10;        // Unit: MB
              fre_sect = ((fre_clust * _fs->csize) >> 1) >> 10;                  // Unit: MB
              if(((tot_sect > (128 << 10)) && (fre_sect < (28 << 10))) || (tot_sect < (8 << 10))){
                  goto sd_format_section;
              }
        	  if(sd_format){    // 格式化
        		  MKFS_PARM mkfs_parm = { 0 };
              sd_format_section:
				  switch(sd_format_file_system){
				  case 2:
					  mkfs_parm.fmt = FM_FAT32;
					  break;
				  case 4:
					  mkfs_parm.fmt = FM_EXFAT;
					  break;
				  default:
					  mkfs_parm.fmt = FM_FAT32;
					  break;
				  }
        		  ret = f_mkfs(sd_root_path, &mkfs_parm, (uint8_t*)workBuffer, sizeof(workBuffer));
        		  if(ret == FR_OK){   // 格式化成功
                      ret = f_chdir(sd_root_path);         // 切换到根目录
                      ret = f_mkdir("log");             // 创建日志目录
                      ret = f_open(&log_file, _lp, FA_CREATE_ALWAYS | FA_WRITE);
                      if(fre_sect < (8 << 10)) ret = f_log_printf(&log_file, "[!!!!!警告!!!!!]容量过小, 请更换SD卡\n");
                      ret = f_log_printf(&log_file, "格式化完成...\n");
                	  if((ads_conver_rate > 16000) || (ads_conver_rate < 0)){
                		  uint32_t _bak = ads_conver_rate;
                		  ads_conver_rate = 2000;
                          f_log_printf(&log_file, "[!!!!!警告!!!!!]ADC不支持采样率%uHz, 已更改为 %uHz\n", _bak, ads_conver_rate);
                	  }
                      ret = f_close(&log_file);
//                      save_config_to_sd(sd_root_path);   // 保存配置到文件
                        save_ini_file_to_sd(sd_root_path, NULL, &config);
        		  }
        	  }
        	  ret = f_unmount((TCHAR const*)sd_root_path);
//		  }
	  }
  }
//  LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_SYSCLK, LL_RCC_MCO1_DIV_8);

  if((ads_conver_rate <= 4000)){
	  LL_SystemClock_Config(MSI_TO_HSE, 16000000);
  }else
	  if((ads_conver_rate > 4000) && (ads_conver_rate <= 8000)){
	  LL_SystemClock_Config(MSI_TO_HSE, 32000000);
  }else
	  if((ads_conver_rate > 8000) && (ads_conver_rate <= 16000)){
	  LL_SystemClock_Config(MSI_TO_HSE, 64000000);
  }else{

  }
  ret = HAL_Init();

  ret = BSP_SD_ChangeSpeed(10);
  if(ret == MSD_OK){   // 重新初始化
	  uint32_t _sdmmc_clk = 0, _sdmmc_div = 0;
	  _sdmmc_clk = LL_RCC_GetSDMMCClockFreq(LL_RCC_SDMMC1_CLKSOURCE);
	  _sdmmc_div = (SDMMC1->CLKCR) & 0x000000FF;
	  _sdmmc_clk /= (_sdmmc_div + 2);
	  ret = f_mount(&SDFatFS, (TCHAR const*)sd_root_path, 0);          // 重新挂载
	  if(log_file.obj.fs == NULL) ret = f_open(&log_file, "0:/log/log.txt", FA_OPEN_EXISTING | FA_WRITE | FA_OPEN_APPEND);  // 打开文件, 并把读写指针移到文件末尾, 追加数据
      ret = f_log_printf(&log_file, "核心频率 =%uHz更改完成, SD 重新初始化完成, 速度为%ubps...\n", SystemCoreClock, _sdmmc_clk);
  }
  if((LL_RCC_LSE_IsReady() == 0) && (LL_RCC_LSI_IsReady())){
	  ret = f_log_printf(&log_file, "外部RTC时钟无法启动, 请检查RTC电池, 启用内部RTC时钟\n");
  }
  rtc_ex_cal_offset = 0x06;
  if(rtc_ex_cal_offset == 0x00){ // PCF时钟校准配置丢失
	  rtc_ex_cal_offset = LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR0);   // 读取上一次配置
  }else{   // PCF校准值已更新, 更新备份寄存器的校准值
	  LL_RTC_BAK_SetRegister(RTC, LL_RTC_BKP_DR0, (uint32_t)rtc_ex_cal_offset);
  }
  if(rtc_ex_cal_offset){
	  uint8_t _offset = 0;
	  ret = rtc_pcf_get_calibration(&_offset);   // 读取PCF的校准值
	  if(_offset != rtc_ex_cal_offset){
		  ret = rtc_pcf_calibrate_offset(PCF_CAL_NORMAL_MODE, rtc_ex_cal_offset);  // 写入新的校准值
	  }
  }

  st_exti_irq_handler_register(EXTI3_IRQn, 3, usb_dection_exti_callback);
  usb_dection_exti_enable();

  if((gps_check) && (gps_startup)){
	  task_fild |= TASK_GPS;
  }else{
	  task_fild &= (~(TASK_GPS | TASK_GPS_STOP));
	  int32_t _sys_cal_vlaue = 0;
	  uint32_t _systick_reload = 0;
	  _sys_cal_vlaue = LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR2);   // 读取 HSE 校准值
      _systick_reload = (((SystemCoreClock - _sys_cal_vlaue) / 1000) - 1); // 1ms
  }
  if(usb_is_connect()){  // USB 插入
	  CLEAR_BIT(task_fild, TASK_ADS);
      SET_BIT(task_fild, TASK_USB_DECTION | TASK_BATTERY | TASK_CMD);
  }else{

      if(ads_start_time == 0){
    	  task_fild = TASK_ADS;
      }
  }

  led_prio_start_tick = HAL_GetTick();
  cmd_connect_tick = (uint32_t)(HAL_GetTick() / 1000);  // 毫秒
  cmd_connect_time = cmd_connect_tick + (system_cmd_connect_time); // 连接超时5分钟
//  cmd_connect_timeout_bak = system_cmd_connect_time;

  while(1){
loop_section:
      if(task_fild & TASK_LED){
    	  uint32_t _led_tick = 0;
    	  _led_tick = HAL_GetTick();
    	  if((_led_tick != led_last_tick) && ((uint32_t)(led_blink_freq * 1000))){
    		  led_last_tick = _led_tick;
        	  if((led_last_tick >= (led_prio_start_tick + (uint32_t)(1000.0f / led_blink_freq))))
        		  led_prio_start_tick = led_last_tick;   // 周期开始
    		  if((led_last_tick < (led_prio_start_tick + (uint32_t)(1000.0f / led_blink_freq * (led_high_time)))))
    		  {  // LED 亮
    			  if(!led_status){
    				  led_status = 1;
    				  LED_ON();
    			  }
    		  }else{
    			  if(led_status){
    				  led_status = 0;
    				  LED_OFF();
    			  }
    		  }
    	  }
      }
      if(task_fild & TASK_USB_DECTION){
			if(usb_is_connect()){  // 高电平,  USB插入
				usb_dection = 1;
			//		task_fild |= TASK_USB_DECTION;
			}else{ // 低电平, USB拔出
				usb_dection = 0;
			//		task_fild |= TASK_USB_DECTION;
			}
    	  if(usb_dection){  // USB 插入
    		  if(cmd_status & CMD_STATUS_INIT){
    			  cmd_printf("USB connected to system\r\n");
    		  }
    		  task_fild_bak = task_fild;
    		  task_fild = (TASK_CMD | TASK_GPS | TASK_BATTERY);
    		  if(log_file.obj.fs != 0){
    			  f_log_printf(&log_file, "USB connected to system\n");

    		  }
    		  if(ads_data_file.obj.fs != 0){
    			  f_sync(&ads_data_file);
    		  }
    		  sd_switch_connect(SD_CONNECT_TO_USB);
//    		  task_fild &= (~TASK_USB_DECTION);
    		  HAL_Delay(5000);
    	  }else{           // USB 拔出
    		  if(cmd_status & CMD_STATUS_INIT){
    			  cmd_printf("USB disconnected from system\r\n");
    		  }
    		  task_fild = (task_fild_bak ? task_fild_bak : task_fild);
    		  sd_switch_connect(SD_CONNECT_TO_MCU);
    		  f_log_printf(&log_file, "USB disconnected from system\n");
//    		  task_fild &= (~TASK_USB_DECTION);
    	  }
//    	  task_fild &= (~TASK_USB_DECTION);
    	  CLEAR_BIT(task_fild, TASK_USB_DECTION);
      }

      if(task_fild & TASK_BATTERY){
    	  if((adc_status & ADC_STATUS_INIT) == 0){

//    		  gpio_pa4_change_mode(LL_GPIO_MODE_ANALOG);

    		  st_irq_handler_register(ADC1_2_IRQn, adc1_irq_callback);

    		  st_adc_init();
    		  adc_status |= ADC_STATUS_INIT;
    		  adc_control |= ADC_STATUS_START;
    		  task_fild &= (~TASK_LED);

    		  st_adc_start();
    		  adc_start_tick[0] = HAL_GetTick();
    	  }else{    // ADC初始化完成
    		  adc_start_tick[1] = HAL_GetTick();
    		  if((adc_start_tick[1] - adc_start_tick[0]) >= 1000){  // 采样率 1Hz
                  battery_level = get_battery_level();
//        		  mcu_temp = get_internal_temp();
//            	  if(cmd_status & CMD_STATUS_INIT){
//            		  cmd_printf("<system_battery_level %d\r\n", battery_level);
//            		  cmd_printf("<system_temperature %.1f\r\n", mcu_temp);
//            	  }
        		  st_adc_start();
        		  adc_start_tick[0] = HAL_GetTick();
    		  }
              if(adc_control & ADC_STATUS_STOP){
            	  st_adc_stop();
//            	  st_adc_deinit();
            	  adc_control &= (~ADC_STATUS_STOP);
//            	  task_fild &= (~TASK_BATTERY);
              }
              if(battery_level < system_battery_thold_voltage){
//            	  task_fild = TASK_LED | TASK_CMD;
            	  if(cmd_status & CMD_STATUS_INIT){
//            		  ret = UART3_Write("Battery \r\n", 0);
            		  ret = cmd_printf("Battery Power Warning\r\n");
            	  }
        		  led_blink_freq = 2.00f;
        		  led_high_time = 0.03f;
              }
    	  }
      }
      if(task_fild & TASK_BATTERY_STOP){
    	  if(adc_status){
        	  st_adc_stop();
        	  st_adc_deinit();
        	  adc_status = 0;
        	  adc_control = 0;
        	  task_fild &= (~TASK_BATTERY);
        	  if((task_fild & TASK_ADS) == 0){
        		  task_fild |= TASK_LED;
        	  }
        	  gpio_pa4_change_mode(LL_GPIO_MODE_OUTPUT);
    	  }
    	  CLEAR_BIT(task_fild, TASK_BATTERY | TASK_BATTERY_STOP);
      }

      if(task_fild & TASK_CMD){
		  if((cmd_status & CMD_STATUS_INIT) == 0){  // 未初始化, 准备初始化
			  if(!usb_dection) f_log_printf(&log_file, "初始化系统上位机通讯串口\n");
			  if(system_uart_baudrate) uart_init(USART3, system_uart_baudrate);   // 初始化 system 串口
			  else{
				  uart_init(USART3, 115200);   // 初始化 system 串口
			  }
			  if(cmd_buffer == NULL){
				  cmd_buffer = (char*)malloc((size_t)system_cmd_buffer_size);
				  cmd = (char*)malloc(16);
				  cmd_val = (char*)malloc(32);
				  cmd_data = (char*)malloc(32);
				  memset(cmd_buffer, '\0', system_cmd_buffer_size);
				  memset(cmd, '\0', 16);
				  memset(cmd_val, '\0', 32);
				  memset(cmd_data, '\0', 32);
			  }
			  if(!usb_dection) f_log_printf(&log_file, "系统上位机通讯串口初始化完成, 波特率为 %d bps, 超时 %d 秒\n", system_uart_baudrate, system_cmd_connect_time);

			  while(!usb_is_connect()); // 等待USB插入
              HAL_Delay(3000);          // 等待 5s, 留时间上位机打开串口
              struct tm _tm;
              st_rtc_get_time(&_tm);
			  ret = cmd_printf("[%d-%d-%d-%d-%d-%d:%u]:CMD\r\n", _tm.tm_year + 1900, _tm.tm_mon, _tm.tm_mday, _tm.tm_hour, _tm.tm_min, _tm.tm_sec, HAL_GetTick());
			  cmd_status |= CMD_STATUS_INIT;  // 初始化完成
		  }
		  if((cmd_status & CMD_STATUS_INIT)){
			  cmd_connect_tick = (uint32_t)(HAL_GetTick() / 1000);

			  if((cmd_status & CMD_STATUS_CONNECT) && (cmd_connect_tick > cmd_connect_time)){
				  task_fild |= (TASK_CMD_STOP);   // 连接超时,  关闭
				  task_fild &= (~TASK_CMD);
				  if(!usb_dection){
					  f_log_printf(&log_file, "系统上位机通讯串口连接超时, 即将关闭\n");
				  }
				  continue;
			  }
			  memset(cmd_buffer, '\0', system_cmd_buffer_size);
			  ret = UART3_Read((uint8_t*)cmd_buffer, system_cmd_buffer_size);
			  if(ret == 0){
				  memset(cmd, '\0', 16);
				  memset(cmd_val, '\0', 32);
				  memset(cmd_data, '\0', 32);
				  ret = cmd_scan(cmd_buffer, cmd, cmd_val, cmd_data);
				  if((!ret) && (strcmp(cmd, "connect") == 0)){ // connect 命令
					  // TODO
					  task_fild |= TASK_CMD;
					  cmd_status |= CMD_STATUS_CONNECT;    // 标记连接
					  cmd_connect_tick = (uint32_t)(HAL_GetTick() / 1000);  // 秒
					  cmd_connect_time = cmd_connect_tick + (system_cmd_connect_time); // 连接超时5分钟

					  ret = cmd_printf("<OK\r\n");
				  }
				  if((!ret) && (strcmp(cmd, "disconnect") == 0)){
					  task_fild |= (TASK_CMD_STOP);   // 关闭
					  task_fild &= (~TASK_CMD);
					  if(!usb_dection){
						  f_log_printf(&log_file, "系统上位机通讯串口即将关闭\n");

					  }
					  continue;
				  }
				  if((!ret) && (strcmp(cmd, "help") == 0)){  // help 命令
					  // TODO
					  char* *_help_file = NULL;
					  uint32_t _lines = 0, _line_size = 0, i = 0;
					  _help_file = get_default_config_file();
					  _lines = get_default_config_lines();
					  for(i = 0; i < _lines; i++){
						  _line_size = strlen(_help_file[i]);
						  ret = UART3_Write((uint8_t*)_help_file[i], _line_size);
						  if(ret) continue;
					  }
				  }
				  if((!ret) && (cmd_status & CMD_STATUS_CONNECT) && (strcmp(cmd, "set") == 0)){   // set 命令
					  // TODO
//                      set_config_value(cmd_val, cmd_data);
                      int pos = 0;
                      pos = find_chr(cmd_val, '_');  // 找出 section 结束位置,  e.g. section_
					  if(pos >= 0){
						  *(cmd_val + pos) = '\0';
						  set_config(&config, &cmd_val[0], &cmd_val[pos + 1], atol(cmd_data));
						  ret = cmd_printf("<OK\r\n");
					  }else{ //
                          if(strcmp(cmd_val, "time") == 0){  // 设置时间,  >set time 2000-1-1-0-0-0
                              struct tm _tm;
                              parse_iso8601_time_from_string(cmd_data, &_tm);
                              st_rtc_set_time(&_tm);
                              ret = cmd_printf("<OK\r\n");
//                              st_rtc_get_time(&_tm);
//                              ret = cmd_printf("%d-%d-%d-%d-%d-%d\r\n", _tm.tm_year + 1900, _tm.tm_mon, _tm.tm_mday, _tm.tm_hour, _tm.tm_min, _tm.tm_sec);
                          }
					  }
				  }
				  if((!ret) && (cmd_status & CMD_STATUS_CONNECT) && (strcmp(cmd, "get") == 0)){   // get 命令
					  // TODO
					  if(strcmp(cmd_val, "system_battery_level") == 0){ // 电池电量
                          int8_t _battery_level = 0;
                          _battery_level = get_battery_level();
                          ret = cmd_printf("<system_battery_level=%d\r\n", _battery_level);
					  }else if(strcmp(cmd_val, "gps_ant_status") == 0){

						  ret = cmd_printf("<gps_ant_status=%d\r\n", (gps_status & GPS_STATUS_ANT_OK));
					  }
					  else{
                          int32_t _config = 0;
//                          _config = set_config_value(cmd_val, NULL);
                          int pos = 0;
                          pos = find_chr(cmd_val, '_');
                          if(pos >= 0){
                        	  *(cmd_val + pos) = '\0';
                              _config = get_config(&config, &cmd_val[0], &cmd_val[pos + 1]);
                              cmd_printf("<%s_%s=%ld\r\n", cmd_val, &cmd_val[pos + 1], _config);
                          }
					  }
				  }
				  if((!ret) & (cmd_status & CMD_STATUS_INIT) && (strcmp(cmd, "restart") == 0)){
					  NVIC_SystemReset();   // 重启
				  }
				  if((!ret) && (cmd_status & CMD_STATUS_CONNECT) && (strcmp(cmd, "save") == 0)){  // save 命令, 保存设置到文件
                      // TODO
//					  save_config_to_sd(sd_root_path);
					  TF_Connect_To_MCU();
					  HAL_Delay(1000);
					  VSD_Control(SET);
					  ret = save_ini_file_to_sd(sd_root_path, NULL, &config);
	                  TF_Connect_To_USB();
	                  HAL_Delay(1000);
	                  VSD_Control(SET);
					  if(!ret) cmd_printf("<OK\r\n");
	                  else cmd_printf("<FAIL\r\n");
				  }
				  if((!ret) && (cmd_status & CMD_STATUS_CONNECT) && (strcmp(cmd, "gps") == 0)){  // 转发 GPS 数据
					  // TODO
					  gps_to_cmd = !gps_to_cmd;
				  }
			  }
		  }
      }
      if(task_fild & TASK_CMD_STOP){
		  if((cmd_status & CMD_STATUS_INIT)){
			  free(cmd_buffer);
			  cmd_buffer = NULL;
			  free(cmd);
			  cmd = NULL;
			  free(cmd_val);
			  cmd_val = NULL;
			  free(cmd_data);
			  cmd_data = NULL;
			  uart_stop(USART3);
			  cmd_status = 0;
		  }
//		  save_config_to_sd(sd_root_path);  // 保存配置到文件
		  save_ini_file_to_sd(sd_root_path, NULL, &config);

//		  task_fild &= (~(TASK_CMD_STOP | TASK_CMD));   // 删除任务
		  CLEAR_BIT(task_fild, TASK_CMD_STOP | TASK_CMD);
      }

      if(task_fild & TASK_GPS){

		  if((gps_status & GPS_STATUS_INIT) == 0){  // GPS 串口未初始化, 准备初始化
			  if(gps_startup) VGPS_Control(1);           // GPS 模块上电
			  if(gps_uart_baudrate) uart_init(UART4, gps_uart_baudrate);  // 初始化串口
			  else{
				  uart_init(UART4, 9600);
			  }
			  gps_drv_init(&UART4_Write, &UART_GPS_Receice);   // 初始化 GPS 驱动
			  gps_config_t gps_cfg = {
					  .baudrate = 1,
					  .up_rate = 1000,
					  .gps_mode = 2,
					  .control_output = NULL,
			  };
			  gps_config(&gps_cfg);         // 初始化 GPS 模块
//			  gps_save_config_to_flash();
			  gps_text_buffer = (char*)malloc(80);
			  memset(gps_text_buffer, 0, 80);
			  gps_status |= GPS_STATUS_INIT;   // 初始化完成
			  if(!usb_dection){
				  f_log_printf(&log_file, "GPS和通讯串口初始化完成\n");

			  }
			  if(cmd_status & CMD_STATUS_INIT){
				  cmd_printf("GPS和通讯串口初始化完成\r\n");
			  }
			  GPIO_InitTypeDef GPIO_InitStruct;
			  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
			  GPIO_InitStruct.Pull  = GPIO_PULLUP;
			  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			  GPIO_InitStruct.Pin = EXTI_PPS_Pin;
			  HAL_GPIO_Init(EXTI_PPS_GPIO_Port, &GPIO_InitStruct);

			  st_exti_irq_handler_register(EXTI_PPS_EXTI_IRQn, 2, gps_pps_cunt_callback);

			  gps_pps_exti_enable();
		  }
		  if((gps_status & GPS_STATUS_INIT)){

			  ret = gps_update((uint8_t*)gps_text_buffer, 80);
			  if(!ret){
				  if(gps_to_cmd){
					  cmd_printf("%s", gps_text_buffer);
				  }else{
					  if(!ret){
						  if(memcmp(gps_text_buffer, "$GPTXT", 6) == 0){  // 检测天线
							  ret = gps_check_ant((uint8_t*)gps_text_buffer);
							  if((ret == 0) && ((gps_status & GPS_STATUS_ANT_OK) == 0)){  // 天线存在
								  if(((gps_status & GPS_STATUS_ANT_OK) == 0) && (!usb_dection)){
									  f_log_printf(&log_file, "GPS 天线插入\n");
									  gps_ant_status = 0x01;
								  }
								  gps_status |= GPS_STATUS_ANT_OK;         // 天线插入
								  if(cmd_status & CMD_STATUS_INIT){
									  ret = cmd_printf("GPS ANT OK\r\n");
								  }
							  }
							  if(ret){   // 天线不存在
								  if(!usb_dection){
									  f_log_printf(&log_file, "GPS 天线未检测到\n");
								  }
								  if(cmd_status & CMD_STATUS_INIT){
									  ret = cmd_printf("GPS ANT NOT EXIST\r\n");
								  }
								  gps_status &= (~GPS_STATUS_ANT_OK);      // 天线未插入
							  }
						  }
						  // TODO
						  if(gps_status & GPS_STATUS_ANT_OK){
							  if(memcmp(&gps_text_buffer[3], "ZDA", 3) == 0){   // 获取时间信息
								  struct minmea_sentence_zda m_zda = {0};
								  ret = minmea_parse_zda(&m_zda, gps_text_buffer);
								  if(ret){
									  if((m_zda.time.seconds < 0) & ((local_time_zone < 13) || local_time_zone > -13)){
										  gps_status &= (~GPS_STATUS_TIME);  // 标记授时失败
									  }else{   // 数据有效
		                                  struct tm _tm = { 0 };
		                                  _tm.tm_year = m_zda.date.year - 1900;
		                                  _tm.tm_mon = m_zda.date.month;
		                                  _tm.tm_yday = m_zda.date.day;
		                                  _tm.tm_hour = m_zda.time.hours + local_time_zone;
		                                  _tm.tm_min = m_zda.time.minutes;
		                                  _tm.tm_sec = m_zda.time.seconds;
		                                  if(_tm.tm_hour > 24){
		                                	  _tm.tm_mday += 1;
		                                	  _tm.tm_hour -= 24;
		                                  }
		                                  if(_tm.tm_hour < 0){
		                                	  _tm.tm_mday -= 1;
		                                	  _tm.tm_hour += 24;
		                                  }
		                                  if(!(gps_status & GPS_STATUS_TIME)){
		                                	  ret = rtc_pcf_set_date_time((pcf_tm_t*)&_tm);
		                                	  if(ret) // PCF 写入错误, 写入内部RTC
		                                		  st_rtc_set_time(&_tm);
		                                	  gps_status |= GPS_STATUS_TIME;  // 标记授时成功
		                                  }
		                                  if(!usb_dection){
		        							  f_log_printf(&log_file, "GPS 授时: %d-%d-%d:%d:%d:%d.%d\n", m_zda.date.year, m_zda.date.month, m_zda.date.day,
		        										  m_zda.time.hours + local_time_zone, m_zda.time.minutes, m_zda.time.seconds, m_zda.time.microseconds);

		                                  }
										  if(cmd_status & CMD_STATUS_INIT){
											  ret = cmd_printf("time:%d-%d-%d:%d:%d:%d.%d\r\n",
													  m_zda.date.year, m_zda.date.month, m_zda.date.day,
													  m_zda.time.hours + local_time_zone, m_zda.time.minutes, m_zda.time.seconds, m_zda.time.microseconds);
										  }
									  }
								  }
							  }
							  if(memcmp(&gps_text_buffer[3], "GLL", 3) == 0){  // 获取经纬度
								  struct minmea_sentence_gll m_gll = {0};
								  ret = minmea_parse_gll(&m_gll, gps_text_buffer);
								  if(ret){
									  double latitude = 0.0f, longitude = 0.0f;
			                          latitude = (double)m_gll.latitude.value / (double)m_gll.latitude.scale / 100.000000f;
			                          longitude = (double)m_gll.longitude.value / (double)m_gll.longitude.scale / 100.000000f;

			                          if(!isnan(latitude) && !isnan(longitude)){  // 数据有效
			                        	  local_time_zone = gps_get_time_zone(m_gll.longitude);
			                        	  if(!usb_dection){
			                        		  f_log_printf(&log_file, "获取地理位置, 纬度: %f°, 经度: %f°, 所在时区: %d(正: 东, 负: 西)\n", latitude, longitude, local_time_zone);

			                        	  }
				                          if(cmd_status & CMD_STATUS_INIT){
				                        	  ret = cmd_printf("latitude:%f, longitude:%f\r\n", latitude, longitude);
				                          }
		                              }
								  }
							  }
							  if((gps_status & GPS_STATUS_PPS_OK) == 0){
								  // TODO
								  if(gps_pps_cunt > 32){
									  gps_status |= GPS_STATUS_PPS_OK;
									  ret = cmd_printf("GPS PPS OK\r\n");
								  }else{
									  gps_status &= (~GPS_STATUS_PPS_OK);
								  }
							  }
							  if(!(system_calibration_ctrl & SYS_CAL_STATUS_CPLT)
									  && (gps_status & GPS_STATUS_PPS_OK) && (gps_status & GPS_STATUS_TIME)){  // PPS脉冲正常, 授时正常, 测量系统外部晶振频率
		                          uint32_t _systick_reload_value = 0;
		                          int32_t _sys_hse_cal = 0;
		                          uint32_t i = 0;
		                          size_t _cal_buf_size = SYS_CAL_SIZE + 1;

		                          NVIC_DisableIRQ(TIM2_IRQn);
		                          system_calibrate_offset = (uint32_t*)malloc(_cal_buf_size * sizeof(uint32_t));   // 开辟缓冲区
		                          if(system_calibrate_offset == NULL) continue;

		                          // 测试频率, 大概测量, LSB: (1 / (SystemCoreClock / 8))s
		                          systick_disable();
		//                          systick_disable_ti_req();
		                          systick_selete_clk_div(LL_SYSTICK_CLKSOURCE_HCLK_DIV8);
		                          _systick_reload_value = (SystemCoreClock >> 3) - 1;  // 定时 1s
		                          systick_set_reload_value(_systick_reload_value);
		                          systick_enable_ti_req();
		                          systick_set_counter(0);

		                          i = system_tick_cunt;
		                          memset(system_calibrate_offset, 0, _cal_buf_size * sizeof(uint32_t));
		                          system_calibration_ctrl = (SYS_CAL_CTRL_START | SYS_CAL_CTRL_SET);
		                          while(!(system_calibration_ctrl & SYS_CAL_CTRL_STOP));  // 等待结束
		                          systick_disable();
		                          gps_pps_exti_disable();                        // 关闭 GPS PPS 中断响应
		                          system_calibration_ctrl = 0x00;
		                          if(i < system_tick_cunt){                              // 频率偏快
		                        	  system_calibrate_offset[(gps_pps_cunt) % SYS_CAL_SIZE] = _systick_reload_value - system_calibrate_offset[(gps_pps_cunt) % SYS_CAL_SIZE];    // 获取偏移值
		                        	  _sys_hse_cal = 1;
		                          }else{                                                 // 频率偏慢
		                        	  _sys_hse_cal = -1;
		                          }
		                          cmd_printf("HSE OFFSET(TEST): %16u\n", system_calibrate_offset[(gps_pps_cunt) % SYS_CAL_SIZE]);

		                          // 测量频率, 精确测量, LSB: (1 / SystemCoreClock)s
		                          memset(system_calibrate_offset, 0, _cal_buf_size * sizeof(uint32_t));

		                          systick_selete_clk_div(LL_SYSTICK_CLKSOURCE_HCLK);
		                          _systick_reload_value = (SystemCoreClock >> 1) - 1;        // 定时0.5s
		                          systick_set_reload_value(_systick_reload_value);
		                          systick_disable_ti_req();
		                          systick_set_counter(0);

		                          gps_pps_exti_enable();
		                          system_calibration_ctrl = (SYS_CAL_CTRL_START | SYS_CAL_CTRL_SET);
		                          i = SYS_CAL_SIZE;
		                          do{
		                        	  i -= 1;
		                              while(!(system_calibration_ctrl & SYS_CAL_CTRL_STOP)){};  // 等待结束
		                              system_calibration_ctrl &= (~SYS_CAL_CTRL_STOP);          // 清除结束标志位, 再次开始测量
		                          }while(i);
		                          systick_disable();
		                          gps_pps_exti_disable();                                       // 关闭 GPS PPS 中断响应
		                          system_calibration_ctrl = SYS_CAL_STATUS_CPLT;
		                          system_calibrate_offset[_cal_buf_size - 1] = system_calibrate_offset[0];
		                          system_calibrate_offset[0] = 0U;
		                          i = _cal_buf_size;        // 索引范围: 1~(_cal_buf_size-1)
		                          do{                       // 计算每次的偏移值, 因为记录的偏移值是前面的偏移值的累加值
		                        	  i -= 1;
		//                        	  cmd_printf("HSE OFFSET[%u]:%10u\n", i, system_calibrate_offset[i]);
		                        	  if(_sys_hse_cal < 0) system_calibrate_offset[i] -= system_calibrate_offset[i-1];  // 频率偏慢, 偏移值 = 计数值, 并且累加
		                        	  else{         // 频率偏快, 偏移值 = 重载值 - 计数值, 并且累加
		//                        		  system_calibrate_offset[i] = _systick_reload_value - system_calibrate_offset[i];
		//                        		  system_calibrate_offset[i-1] = _systick_reload_value - system_calibrate_offset[i-1];
		//                        		  system_calibrate_offset[i] -= system_calibrate_offset[i-1];
		                        	      system_calibrate_offset[i] = system_calibrate_offset[i-1] - system_calibrate_offset[i];
		                        	  }
		                        	  cmd_printf("HSE OFFSET[%u]:%16u\n", i, system_calibrate_offset[i]);
		                          }while(i != 1);
		                          for(i = 2; i < _cal_buf_size; i++){
		                        	  system_calibrate_offset[0] += system_calibrate_offset[i];
		                          }
		                          system_calibrate_offset[0] /= (SYS_CAL_SIZE - 1);          // 计算平均值
		                          _sys_hse_cal *= system_calibrate_offset[0];
		                          LL_RTC_BAK_SetRegister(RTC, LL_RTC_BKP_DR2, _sys_hse_cal);     // 存储校准值
		                          cmd_printf("HSE OFFSET(FINAL): %16d\n", _sys_hse_cal);
		                          free(system_calibrate_offset);
		                          system_calibrate_offset = NULL;

		                          // 校准 systick 定时  1ms, 校准值用 LL_RTC_BKP_DR3寄存器存储
		                          int32_t _systick_ms_cal = 0;
		                          system_calibrate_offset = (uint32_t *)malloc(_cal_buf_size * 2 * sizeof(uint32_t));
		                          if(system_calibrate_offset == NULL) goto calibration_end_section;
		                          else memset(system_calibrate_offset, 0, _cal_buf_size * 2 * sizeof(uint32_t));
		                     calibrate_ms_section:
		                          _systick_reload_value = (((SystemCoreClock - _sys_hse_cal) / 1000) + _systick_ms_cal) - 1;
		                          systick_set_reload_value(_systick_reload_value);
		                          systick_enable_ti_req();
		                          systick_set_counter(0);
		                          system_calibration_ctrl = SYS_CAL_CTRL_START | SYS_CAL_CTRL_SET;
		                          i = SYS_CAL_SIZE;
		                          do{
		                        	  i -= 1;
		                              while(!(system_calibration_ctrl & SYS_CAL_CTRL_STOP)){};  // 等待结束
		                              system_calibration_ctrl &= (~SYS_CAL_CTRL_STOP);          // 清除结束标志位, 再次开始测量
		                          }while(i);
		                          systick_disable();
		                          gps_pps_exti_disable();                                       // 关闭 GPS PPS 中断响应
		                          system_calibration_ctrl = SYS_CAL_STATUS_CPLT;
		                          while(!(system_calibration_ctrl & SYS_CAL_CTRL_STOP));
		                     calibration_end_section:
		                          NVIC_EnableIRQ(TIM2_IRQn);
		                          gps_startup = 0;
		                          gps_check = 0;
		                          save_ini_file_to_sd(sd_root_path, NULL, &config);
		                          task_fild |= TASK_GPS_STOP;
							  }
						  }
					  }
				  }
			  }
		  }
      }
      if(task_fild & TASK_GPS_STOP){
    	  if(gps_status){
        	  VGPS_Control(0);           // GPS 模块掉电
              uart_stop(UART4);
              free(gps_text_buffer);
              gps_text_buffer = NULL;
              if(!usb_dection) f_log_printf(&log_file, "GPS 关闭\n");
              gps_status = 0;
              task_fild &= (~(TASK_GPS | TASK_GPS_STOP));
              gps_pps_exti_disable();
              LL_GPIO_SetPinMode(EXTI_PPS_GPIO_Port, EXTI_PPS_Pin, LL_GPIO_MODE_ANALOG);
              LL_GPIO_SetPinPull(EXTI_PPS_GPIO_Port, EXTI_PPS_Pin, LL_GPIO_PULL_NO);
    	  }
    	  CLEAR_BIT(task_fild, TASK_GPS_STOP | TASK_GPS);
      }

      if(task_fild & TASK_ADS){
          if((ads_status & ADS_STATUS_INIT) == 0){ // ads 没有初始化, 准备初始化

              ads_drdy_exti_disable();
              ads_bsp_pin_stop();
              ads_bsp_power(SET);
        	  if(ads_conver_rate <= 4000) lptim2_use_for_ads_clk(4000 * ads_osr);   // 采样主时钟  >=100KHz
        	  else lptim2_use_for_ads_clk(ads_conver_rate * ads_osr);
        	  spi_init(SPI1);
        	  ads_drv_init(SPI1_Write, SPI1_Read, SPI1_Write_Read);   // 初始化通讯接口驱动
        	  spi_set_clk_div(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV256);  // 低速读取设备配置

        	  uint8_t tmp[8] = {0};
        	  ads_command_reset();
              HAL_Delay(500);

        	  uint8_t i = 0;
              do{
            	  ads_bsp_selete_cs_by_index(i, RESET);
                  ret = ads_read_regs(ADS_REG_ID, ADS_REG_ID + 7, tmp);
            	  ads_cfg_sys(ADS_DEFAULT_CONFIG);
            	  ret = ads_read_regs(0x00, 0x00 + 7, tmp);
            	  ret = f_log_printf(&log_file, "ADS127 Channel[%d] Config: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n", i, tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6]);
                  i += 1;
              }while(i < ADS_CNT);
        	  char _file_name[16] = {0};
        	  ret = f_chdir(sd_root_path);
              strcpy(_file_name, "ads127");
              ret = f_mkdir(_file_name);   // 创建目录
              if(!((ret == FR_OK) || (ret == FR_EXIST))){
            	  goto end_system_reset;
              }
              ret = f_chdir(_file_name);   // 进入目录
        	  uint16_t _t = 4;
              uint32_t ads_cal[ADS_CNT] = { 0 };
        	  if(ads_offset_calibration & 0x80000000){
              ads_calibrate_section:
        		  spi_set_clk_div(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV8);    // 高速读取数据
        		  ads_drdy_exti_enable();
        		  ret = ads_bsp_calibrate(ads_cal, ADS_CNT);
        		  ads_drdy_exti_disable();
        		  spi_set_clk_div(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV256);  // 低速读取设备配置
        	        i = 0;
        	        do{
        	      	    ads_bsp_selete_cs_by_index(i, RESET);
        	      	    ads_cfg_sys_offset_calibration(ads_cal[i]);
        	            i += 1;
        	        }while(i < ADS_CNT);
        		  if((ret != 0) & (_t)){
        			  _t -= 1;
        			  goto ads_calibrate_section;
        		  }else{
                      FIL ads_cal_file;
                      ret = f_open(&ads_cal_file, "ads127_calibrate_value.txt", FA_CREATE_ALWAYS | FA_WRITE);
                      for(i = 0; i < ADS_CNT; i++){
                    	  ads_bsp_selete_cs_by_index(i, RESET);
                          ret = ads_read_regs(ADS_REG_ID, ADS_REG_ID + 7, tmp);
                          ads_cal[i] = tmp[ADS_REG_OFC0] | (tmp[ADS_REG_OFC1] << 8) | (tmp[ADS_REG_OFC2] << 16);
                    	  ret = f_log_printf(&ads_cal_file, "ADS127 calibrate value of Channel[%d]: %XH\n", i, ads_cal[i]);
                      }
                      f_close(&ads_cal_file);
        		  }
        	  }else{
      	        i = 0;
      	        do{
      	      	    ads_bsp_selete_cs_by_index(i, RESET);
      	      	    ads_cfg_sys_offset_calibration(ads_offset_calibration);
      	            i += 1;
      	        }while(i < ADS_CNT);
        	  }
        	  if(ads_gain_calibration & 0x80000000){

        	  }else{
        	        i = 0;
        	        do{
        	      	    ads_bsp_selete_cs_by_index(i, RESET);
        	      	    ads_cfg_sys_gain_calibration(ads_gain_calibration);
        	            i += 1;
        	        }while(i < ADS_CNT);
        	  }

              ret = f_mkdir("data");
              if(!((ret == FR_OK) || (ret == FR_EXIST))){
            	  goto end_system_reset;
              }
              ret = f_chdir("data");  // 进入目录

              struct tm _tm = { 0 };
              st_rtc_get_time(&_tm);
              sprintf(_file_name, "ads_%d-%d-%d-%d-%d-%d", _tm.tm_year, _tm.tm_mon, _tm.tm_mday, _tm.tm_hour, _tm.tm_min, _tm.tm_sec);
              strcat(_file_name, ads_data_file_fmt_suffix[ads_data_file_format]);
//        	  ret = f_open(&ads_data_file, _file_name, FA_CREATE_ALWAYS | FA_WRITE);
              ret = fs_create_file(&ads_data_file, NULL, _file_name);
              if(!ret){
            	  char _path[FF_LFN_BUF] = {0};
            	  ret = f_getcwd(_path, FF_LFN_BUF);
            	  f_log_printf(&log_file, "ADS 采集数据文件创建成功: %s/%s\n", _path, _file_name);
              }

        	  ads_bsp_pin_stop();
        	  spi_set_clk_div(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV8);    // 高速读取数据

        	  ret = (system_core_freq / system_rco_div / 1000000);
        	  while(ret--);

              ads_read_data_by_dma_init(NULL, NULL, 0);
              ads_drdy_exti_enable();
//              ads_bsp_pin_start();

              st_rtc_config_timeout(RTC_TIMEOUT);

              ads_status |= ADS_STATUS_INIT;

        	  CLEAR_BIT(task_fild, TASK_LED);
        	  SET_BIT(task_fild, TASK_CMD_STOP | TASK_GPS_STOP | TASK_BATTERY_STOP | TASK_ADS);
//        	  goto loop_section;
        	  VBUS_Control(RESET);
          }
          else{
        	  uint8_t _n = 0, _m = 5;
        	  uint8_t _format = 1;
        	  uint32_t _ads_data_file_limit = ((sd_file_size_limit << 10) << 10);            // Unit: Byte
        	  _format = (ads_data_file_format) ? 1 : 0;

              ads_bsp_pin_start();

        	  while(task_fild & TASK_ADS){    // 循环读取数据
				  if(ads_data_is_buffer_full()){  // 读取数据
                      if(rtc_sd_timeout_count >= (RTC_SD_WR_TIMEOUT / RTC_TIMEOUT)){
                    	  ret = BSP_SD_Init();
                    	  if(ret){

                    		 goto end_system_reset;
                    	  }
                      }
					  if(f_size(&ads_data_file) > (_ads_data_file_limit - ADS_DATA_BUFFER_SIZE)){ // 文件达到最大限制
						  if(ads_data_file.obj.fs){
							  ret = f_close(&ads_data_file);
							  if(!ret){
								  f_log_printf(&log_file, "文件达到限制, 结束写入成功, 即将创建新文件.\n");
							  }
						  }
						  uint64_t fre_clust = 0, fre_sect = 0;
						  FATFS *_fs = NULL;
						  char _file_name[128] = {0};
						  struct tm _tm = { 0 };
					  free_sd_space_section:
						  ret = f_getfree(sd_root_path, &fre_clust, &_fs);
						  fre_sect = ((fre_clust * _fs->csize) >> 1);                        // Unit: KB
						  if(fre_sect < (_ads_data_file_limit >> 10)){                       // 剩余容量小于文件大小上限
							  char _patt[12];
							  strcpy(_patt, "ads_*");
							  strcat(_patt, ads_data_file_fmt_suffix[_format]);
							  fs_del_file(NULL, _patt, DEL_FILE_BY_MODIFYTIME_UP); // 删除修改时间最早的数据文件
                              f_log_printf(&log_file, "SD卡空间不足, 已删除");
							  goto free_sd_space_section;
						  }
						  // 剩余容量大于文件大小上限
						  st_rtc_get_time(&_tm);
						  sprintf(_file_name, "ads_%d-%d-%d-%d-%d-%d", _tm.tm_year, _tm.tm_mon, _tm.tm_mday, _tm.tm_hour, _tm.tm_min, _tm.tm_sec);
						  strcat(_file_name, ads_data_file_fmt_suffix[_format]);
						  ret = fs_create_file(&ads_data_file, NULL, _file_name);
			              if(!ret){
			            	  char _path[FF_LFN_BUF] = {0};
			            	  ret = f_getcwd(_path, FF_LFN_BUF);
			            	  f_log_printf(&log_file, "ADS 采集数据文件创建成功: %s/%s\n", _path, _file_name);
			              }
					  }
					  if(_format){
						  if(ads_data_file.obj.fs){
							  if(LL_GPIO_GetPinMode(LED_GPIO_Port, LED_Pin) != LL_GPIO_MODE_OUTPUT){
								  gpio_pa4_change_mode(LL_GPIO_MODE_OUTPUT);
							  }
							  if(((++_n) >= _m) || (!_n)){
								  _n = 1;
								  LED_ON();
							  }
							  ret = ads_save_data_to_file(&ads_data_file, (void*)&_format);
							  if(!ret){
								  rtc_reset_sd_timeout();
								  LED_OFF();
								  if(LL_GPIO_GetPinMode(LED_GPIO_Port, LED_Pin) == LL_GPIO_MODE_OUTPUT)
									  optimize_gpio_power(LED_GPIO_Port, LED_Pin);
							  }
						  }
					  }
				  }
				  if(ads_get_read_status() & ADS_READ_COMPLETE){
	//				  enter_sleep_mode();
				  }
        	  }
          }
      }
      if(task_fild & TASK_ADS_STOP){
          ads_bsp_pin_stop();
    	  ads_status = 0;
    	  lptim_deinit(LPTIM2);
    	  ads_bsp_power(RESET);
    	  spi_deinit(SPI1);
    	  if(ads_data_file.obj.fs) ret = f_close(&ads_data_file);
//    	  task_fild &= (~(TASK_ADS | TASK_ADS_STOP));
    	  CLEAR_BIT(task_fild, TASK_ADS | TASK_ADS_STOP);
      }
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
static void system_pll_param_calc(uint32_t in_src_freq, uint32_t in_dst_freq, uint32_t *out_pll_m, uint32_t *out_pll_n, uint32_t *out_pll_r){
    uint8_t n = 0, m = 0, r = 0;
    uint8_t _r[] = {2, 4, 6, 8};
    uint32_t _out_pll_m_clk = 0, _out_pll_clk = 0;
    for(m = 0; m <= 8; ++m){
    	if(out_pll_m == NULL) m = 1;            // 固定 PLL_M, 此时输入频率是 PLL_M 输出频率
        _out_pll_m_clk = in_src_freq / m;
        for(n = 8; n <= 86; ++n){
        	for(r = 0; r < (sizeof(_r) / sizeof(uint8_t)); r++){
            	_out_pll_clk = _out_pll_m_clk * n / _r[r];
            	if(_out_pll_clk == in_dst_freq){
            	    if(out_pll_m != NULL) *out_pll_m = m;
            	    if(out_pll_n != NULL) *out_pll_n = n;
            	    if(out_pll_r != NULL) *out_pll_r = _r[r];
            	    return;
            	}else if(_out_pll_clk < in_dst_freq) break;
            }
        	if((_out_pll_m_clk * n / _r[sizeof(_r) / sizeof(uint8_t) - 1]) > in_dst_freq) break;
        }
    }
}

static void LL_SystemClock_Config(uint32_t in_clock_index, uint32_t in_clk_freq){
    uint32_t _sys_core_freq = in_clk_freq, _sys_hse_value = system_hse_value;
    uint32_t _sys_pll_n = 8, _sys_pll_r = 2, _sys_pll_m = 2;
    uint32_t _sys_msi_freq_range[] = {100000, 200000, 400000, 800000, 1000000, 2000000, 4000000, 8000000, 16000000, 24000000, 32000000, 48000000};
	uint32_t _timeout = SystemCoreClock;

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_PWR_EnableBkUpAccess();
    if(LL_RCC_LSE_IsReady() == 0){   // 防止重复初始化
    	_timeout = SystemCoreClock;
        LL_RCC_ForceBackupDomainReset();
        LL_RCC_ReleaseBackupDomainReset();
		LL_RCC_LSE_EnableBypass();
		LL_RCC_LSE_Enable();
		/* Wait till LSE is ready */
		while((LL_RCC_LSE_IsReady() != 1) && _timeout){ _timeout--; };
		LL_RCC_ClearFlag_LSERDY();
//		LL_RCC_LSE_EnableCSS();
		if(!_timeout){
			LL_RCC_LSI_Enable();
			LL_RCC_LSE_Disable();
			while(LL_RCC_LSI_IsReady() != 1);
		}else{
			LL_RCC_LSI_Disable();
		}
    }
	if(in_clock_index == (MSI_TO_MSI)){  // MSI
		_timeout = SystemCoreClock;
		LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
		while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1){};
		LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	config_msi_as_sysclk:
		LL_RCC_MSI_Enable();
		while(LL_RCC_MSI_IsReady() != 1){};
		LL_RCC_ClearFlag_MSIRDY();
		LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);
		while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_MSI){};
		LL_RCC_MSI_EnableRangeSelection();
		LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_7);
//		LL_RCC_MSI_SetCalibTrimming(0);
		/* Main PLL configuration and activation */
        LL_RCC_PLLSAI1_Disable();
        LL_RCC_PLLSAI1_DisableDomain_48M();
        while(LL_RCC_PLLSAI1_IsReady() != 0){}; // 等待 PLLSAI1 停止
		LL_RCC_PLL_Disable();
		while(LL_RCC_PLL_IsReady() != 0) {};  // 等待 PLL 停止
		LL_RCC_PLL_DisableDomain_SYS();
		_sys_pll_m = LL_RCC_PLLM_DIV_1;
		_sys_pll_r = LL_RCC_PLLR_DIV_2;
        _sys_pll_n = _sys_core_freq * (2 * ((_sys_pll_r >> 25) + 1)) * ((_sys_pll_m >> 4) + 1) / _sys_msi_freq_range[(LL_RCC_MSI_GetRange() >> 4)];
		LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, _sys_pll_m, _sys_pll_n, _sys_pll_r);
		LL_RCC_PLL_Enable();
		LL_RCC_PLL_EnableDomain_SYS();
		while(LL_RCC_PLL_IsReady() != 1) {};
		LL_RCC_PLLSAI1_ConfigDomain_48M(LL_RCC_PLLSOURCE_MSI, _sys_pll_m, 12, LL_RCC_PLLSAI1Q_DIV_2);
		LL_RCC_PLLSAI1_EnableDomain_48M();
		LL_RCC_PLLSAI1_Enable();
		/* Wait till PLLSAI1 is ready */
		while(LL_RCC_PLLSAI1_IsReady() != 1){};
		LL_RCC_ClearFlag_PLLSAI1RDY();
		/* Sysclk activation on the main PLL */
		LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
		while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL){};

		/* Set APB1 & APB2 prescaler*/
		LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
		LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
		LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

		LL_SetSystemCoreClock(_sys_core_freq);

		LL_RCC_HSI_Disable();
		while(LL_RCC_HSI_IsReady() != 0);
		LL_RCC_HSE_Disable();
		while(LL_RCC_HSE_IsReady() != 0);
		LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_NOCLOCK, LL_RCC_MCO1_DIV_8);
		HSE_OSC_Control(RESET);
        return;
	}else if(in_clock_index == (HSI_TO_HSI)){ // HSI
		_timeout = SystemCoreClock;
		LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
		while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1){};
		LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	config_hsi_as_sysclk:
		LL_RCC_HSI_Enable();
		while(LL_RCC_HSI_IsReady() != 1);
		LL_RCC_ClearFlag_HSIRDY();

		LL_RCC_PLLSAI1_Disable();
		while(LL_RCC_PLLSAI1_IsReady() != 0){}; // 等待 PLLSAI1 停止
		LL_RCC_PLLSAI1_DisableDomain_48M();
		LL_RCC_PLL_Disable();
		while(LL_RCC_PLL_IsReady() != 0) {};  // 等待 PLL 停止
		LL_RCC_PLL_DisableDomain_SYS();
		_sys_pll_m = LL_RCC_PLLM_DIV_2;
		_sys_pll_r = LL_RCC_PLLR_DIV_2;
		_sys_pll_n = _sys_core_freq * (2 * ((_sys_pll_r >> 25) + 1)) * ((_sys_pll_m >> 4) + 1) / (HSI_VALUE);
		LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, _sys_pll_m, _sys_pll_n, _sys_pll_r);  // 64M
		LL_RCC_PLL_Enable();
		LL_RCC_PLL_EnableDomain_SYS();
		while(LL_RCC_PLL_IsReady() != 1){};
		LL_RCC_ClearFlag_PLLRDY();
		/* Sysclk activation on the main PLL */
		LL_RCC_PLLSAI1_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSI, _sys_pll_m, 12, LL_RCC_PLLSAI1Q_DIV_2);  // 48MHz
		LL_RCC_PLLSAI1_EnableDomain_48M();
		LL_RCC_PLLSAI1_Enable();
		/* Wait till PLLSAI1 is ready */
		while(LL_RCC_PLLSAI1_IsReady() != 1){};
		LL_RCC_ClearFlag_PLLSAI1RDY();
		/* Sysclk activation on the main PLL */
		LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
		while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL){};
		/* Set APB1 & APB2 prescaler*/
		LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
		LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
		LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
		LL_SetSystemCoreClock(_sys_core_freq);

		LL_RCC_MSI_Disable();
		while(LL_RCC_MSI_IsReady() != 0);
		LL_RCC_HSE_Disable();
		while(LL_RCC_HSE_IsReady() != 0);
		LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_NOCLOCK, LL_RCC_MCO1_DIV_8);
		HSE_OSC_Control(RESET);
		return;
	}else if(in_clock_index == (HSE_TO_HSE)){  // HSE
		_timeout = SystemCoreClock;
		LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
		while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1){};
		LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	config_hse_as_sysclk:
		LL_RCC_HSE_EnableCSS();        // 启用备用时钟功能, 此时需要一个运行稳定的备用时钟做切换, 默认 MSI
		LL_RCC_HSE_EnableBypass();
		LL_RCC_HSE_Enable();
		while((LL_RCC_HSE_IsReady() != 1) && (_timeout)){};
		if((_timeout == 0)){
			goto config_msi_as_sysclk;
		}
		LL_RCC_ClearFlag_HSERDY();

		LL_RCC_PLLSAI1_Disable();   // 配置 PLL 前必须先停止 PLLSAIx
		while(LL_RCC_PLLSAI1_IsReady() != 0){}; // 等待 PLLSAI1 停止
		LL_RCC_PLLSAI1_DisableDomain_48M();
		LL_RCC_PLL_Disable();
		LL_RCC_PLL_DisableDomain_SYS();
		while(LL_RCC_PLL_IsReady() != 0) {};  // 等待 PLL 停止

		if(in_clk_freq == _sys_hse_value){
			LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);
			while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE){};
		}else{
			_sys_pll_m = LL_RCC_PLLM_DIV_2;
			_sys_pll_r = LL_RCC_PLLR_DIV_2;
			_sys_pll_n = _sys_core_freq * (2 * ((_sys_pll_r >> 25) + 1)) * ((_sys_pll_m >> 4) + 1) / (system_hse_value);
			system_pll_param_calc(system_hse_value / ((_sys_pll_m >> 4) + 1), _sys_core_freq, NULL, &_sys_pll_n, &_sys_pll_r);
			_sys_pll_r = ((_sys_pll_r >> 1) - 1) << 25;
			LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, _sys_pll_m, _sys_pll_n, _sys_pll_r);
			LL_RCC_PLL_EnableDomain_SYS();
			LL_RCC_PLL_Enable();
			while(LL_RCC_PLL_IsReady() != 1) {};  // 等待 PLL 启动
			LL_RCC_ClearFlag_PLLRDY();
			LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
			while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL){};
		}

		LL_RCC_PLLSAI1_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, _sys_pll_m, 12, LL_RCC_PLLSAI1Q_DIV_2);
		LL_RCC_PLLSAI1_EnableDomain_48M();
		LL_RCC_PLLSAI1_Enable();
		/* Wait till PLLSAI1 is ready */
		while(LL_RCC_PLLSAI1_IsReady() != 1){};  // 等待 PLLSAI1 启动
		LL_RCC_ClearFlag_PLLSAI1RDY();
		/* Sysclk activation on the main PLL */

		/* Set APB1 & APB2 prescaler*/
		LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
		LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
		LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
		LL_SetSystemCoreClock(_sys_core_freq);

		LL_RCC_MSI_Disable();
		while(LL_RCC_MSI_IsReady() != 0);
		LL_RCC_HSI_Disable();
		while(LL_RCC_HSI_IsReady() != 0);

		LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_NOCLOCK, LL_RCC_MCO1_DIV_8);
		return;
	}else{
		_timeout = SystemCoreClock;
		if(LL_RCC_MSI_IsReady() == 0) LL_RCC_MSI_Enable();
		while(LL_RCC_MSI_IsReady() != 1){};
		LL_RCC_ClearFlag_MSIRDY();
		LL_RCC_MSI_EnableRangeSelection();
		LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_1);
		LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);
		while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI){};
		LL_RCC_MSI_DisablePLLMode();
		if((in_clock_index == MSI_TO_HSE) || (in_clock_index == HSI_TO_HSE)){
			HSE_OSC_Control(SET);
			goto config_hse_as_sysclk; // 切换到 HSE
		}
		if((in_clock_index == MSI_TO_HSI) || (in_clock_index == HSE_TO_HSI)) goto config_hsi_as_sysclk; // 切换到 HSI
		return;
	}
}

static void read_config_from_sd(const char* _root_path, char* _config_section, void* _config, uint8_t opt){  // 废弃函数
    char* dir_path = NULL;
    uint8_t i = 0;
    DIR config_dir = { 0 };
    FRESULT res = FR_OK;
    FIL config_file = { 0 };

	char *config_section_bak = NULL;
	size_t sec_size = 0;
	char* *config_keyname_bak = NULL;
	int8_t back_keyname = -1;
	uint8_t keyname_size = 0;
    int8_t *fild_bak = NULL;
    int8_t *fild_keyname_bak = NULL;
    char *config_line = NULL;
    size_t line_size = 256;

    int8_t is_section = -1;
    int32_t u_value = 0;
    char c_value[16] = { 0 };

    _config = _config;
    _config_section = _config_section;

    dir_path = (char*)malloc(32);
    if(dir_path != NULL) memset(dir_path, 0, 32);
    else{
    	return;
    }

    strcpy(dir_path, _root_path);  // 0:/
    strcat(dir_path, "config");    // 0:/config

    res = f_chdir(_root_path);     // 切换到根目录

    res = f_opendir(&config_dir, dir_path);
    if(res != FR_OK){
    	if(res == FR_NO_PATH){    // 目录不存在
            res = f_mkdir(dir_path);
    	}
    }else
        res = f_closedir(&config_dir);
    if(0){

    }else{
    	config_line = (char*)malloc(line_size);
    	if((config_line == NULL)) return;

    	char _config_path[64];
    	memset(_config_path, 0, 64);
    	strcpy(_config_path, dir_path);
    	strcat(_config_path, "/config.txt");  // 0:/config/config.txt
    	res = f_open(&config_file, _config_path, FA_OPEN_EXISTING | FA_READ);
    	if(res != FR_OK){  // 文件打开错误
            if(res == FR_NO_FILE){// config.txt文件不存在, 将创建默认default_config.txt
                memset(_config_path, 0, 64);
                strcpy(_config_path, dir_path);
                strcat(_config_path, "/default_config.txt");
                res = f_open(&config_file, _config_path, FA_READ | FA_OPEN_EXISTING);   // 尝试打开默认配置文件
                if(res != FR_OK){  // 文件不存在则创建
                	res = f_open(&config_file, _config_path, FA_WRITE | FA_CREATE_ALWAYS);
                    if(res == FR_OK){
                        char* *_default_file = NULL;
                        uint32_t _lines = 0, _line_size = 0;
                        _default_file = get_default_config_file();
                        _lines = get_default_config_lines();
                        for(i = 0; i < _lines; i++){
                        	_line_size = strlen(_default_file[i]);
                            memcpy_dma(config_line, _default_file[i], _line_size);
                        	f_write(&config_file, config_line, _line_size, NULL);
                        }
                        f_close(&config_file);
                        res = f_open(&config_file, _config_path, FA_OPEN_EXISTING | FA_READ); // 重新打开刚刚创建的新文件
                        if(res == FR_OK){
                        	goto read_exist_config__file;
                        }
                    }
                }else{  // 默认配置文件存在
                	goto read_exist_config__file;
                }
            }
    	}else{   // 文件存在
read_exist_config__file:

            sec_size = get_section_size();
            fild_bak = (int8_t*)malloc(sec_size);
            for(i = 0; i < sec_size; i++){
            	fild_bak[i] = i;
            }
    		while(!f_eof(&config_file)){
                memset(config_line, 0, line_size);
                f_gets(config_line, line_size, &config_file);   // read a line
                if((config_line[0] == ';') || (config_line[0] == '\r') || (config_line[0] == '\n')) continue;  // comments
                else{
                    for(i = 0; i < sec_size; i++){
                    	if((fild_bak[i] != -1)){
                    		config_section_bak = get_section_by_index(i);
                        	if((check_config_section((uint8_t*)config_line, config_section_bak, NULL) == 0)){
                        		is_section = i;    // 标记匹配的 section
                        		back_keyname = i;  // 标记需要备份的配置变量列表
                                fild_bak[i] = -1;  // 标记已读取 section
                                break;
                        	}
                    	}
                    }
                    switch(is_section){  // 读取对应 section 的配置变量
                    case -1:   // no section
                    	break;
                    case 0: // [system]
                        if(back_keyname == -1){  // read config value
                            for(i = 0; i < keyname_size; i++){
                            	if((fild_keyname_bak[i] != -1)
                            			&& (get_config_value_by_keyname((uint8_t*)config_line, config_keyname_bak[i], &u_value, 'i') == 0)){ // 匹配完成

                            		fild_keyname_bak[i] = -1; // 标记已读取
                                    switch(i){
                                    case 0:
                                    	system_hse_value = u_value;
                                    	break;
                                    case 1:
                                    	system_core_freq = u_value;
                                    	break;
                                    case 2:
                                    	system_rco_div = u_value;
                                    	break;
                                    case 3:
                                    	system_uart_baudrate = u_value;
                                    	break;
                                    case 4:
                                    	system_cmd_connect_time = u_value;
                                    	break;
                                    case 5:
                                    	system_battery_voltage = u_value;
                                    	break;
                                    case 6:
                                    	system_battery_cutoff_voltage = u_value;
                                    	break;
                                    case 7:
                                    	system_battery_thold_voltage = u_value;
                                    	break;
                                    case 8:
                                    	system_battery_error_voltage = (int32_t)u_value;
                                    	break;
                                    }
                                    break;  // 终止循环
                            	}
                            }
                        }
                        if(back_keyname == 0){   // 备份
                        	config_keyname_bak = get_keyname_by_section(config_section_bak, (uint8_t*)&keyname_size);
                        	if(fild_keyname_bak != NULL){
                        		free(fild_keyname_bak);
                        		fild_keyname_bak = NULL;
                        	}
                        	fild_keyname_bak = (int8_t*)malloc(keyname_size);
                        	for(i = 0; i < keyname_size; i++){
                        		fild_keyname_bak[i] = i;
                        	}
                        	back_keyname = -1;  // 备份完成
                        	break;
                        }
                    	break;
                    case 1: // [sd-tf]

                        if(back_keyname == -1){
                        	for(i = 0; i < keyname_size; i++){
                        		if((fild_keyname_bak[i] != -1)
                        				&& (get_config_value_by_keyname((uint8_t*)config_line, config_keyname_bak[i], c_value, 'c') == 0)){ // 匹配完成
                        			fild_keyname_bak[i] = -1; // 标记已读取
                                    switch(i){
                                    case 0:
                                        sd_format = strtol(c_value, NULL, 10);
                                    	break;
                                    case 1:
                                    	if((strcmp(c_value, "fat32\n") == 0) || (strcmp(c_value, "fat32") == 0) || (c_value[0] == '2')){
                                    		sd_format_file_system = 0x02;
                                    	}else if((strcmp(c_value, "exfat\n") == 0) || (strcmp(c_value, "exfat") == 0) || (c_value[0] == '4')){
                                    		sd_format_file_system = 0x04;
                                    	}else{
                                    		break;
                                    	}
                                    	break;
                                    case 2:
                                    	sd_file_size_limit = strtol(c_value, NULL, 10);
                                    	break;
                                    }
                                    break; // 终止循环
                        		}
                        	}
                        }
                        if(back_keyname == 1){   // 备份
                        	config_keyname_bak = get_keyname_by_section(config_section_bak, (uint8_t*)&keyname_size);
                        	if(fild_keyname_bak != NULL){
                        		free(fild_keyname_bak);
                        		fild_keyname_bak = NULL;
                        	}
                        	fild_keyname_bak = (int8_t*)malloc(keyname_size);
                        	for(i = 0; i < keyname_size; i++){
                        		fild_keyname_bak[i] = i;
                        	}
                        	back_keyname = -1;  // 备份完成
                        	break;
                        }
                    	break;
                    case 2: // [ads127]
                        if(back_keyname == -1){
                            for(i = 0; i < keyname_size; i++){
                            	if((fild_keyname_bak[i] != -1)
                            			&& (get_config_value_by_keyname((uint8_t*)config_line, config_keyname_bak[i], &u_value, 'i') == 0)){
                            		fild_keyname_bak[i] = -1; // 标记已读取
                            		switch(i){
                            		case 0:
                            			ads_conver_rate = u_value;
                            			break;
                            		case 1:
                            			ads_mode = (uint8_t)u_value;
                            			break;
                            		case 2:
                            			ads_start_time = u_value;
                            		    break;
                            		case 3:
                            			ads_offset_calibration = u_value;
                            			break;
                            		case 4:
                            			ads_gain_calibration = u_value;
                            			break;
                            		case 5:
                            			ads_data_file_format = u_value;
                            			break;
                            		case 6:
                            			ads_osr = u_value;
                            			break;
                            		}
                            		break;  // 终止循环
                            	}
                            }
                        }
                        if(back_keyname == 2){   // 备份
                        	config_keyname_bak = get_keyname_by_section(config_section_bak, (uint8_t*)&keyname_size);
                        	if(fild_keyname_bak != NULL){
                        		free(fild_keyname_bak);
                        		fild_keyname_bak = NULL;
                        	}
                        	fild_keyname_bak = (int8_t*)malloc(keyname_size);
                        	for(i = 0; i < keyname_size; i++){
                        		fild_keyname_bak[i] = i;
                        	}
                        	back_keyname = -1;  // 备份完成
                        	break;
                        }
                    	break;
                    case 3: // [gps]
                        if(back_keyname == -1){
                        	for(i = 0; i < keyname_size; i++){
                        		if((fild_keyname_bak[i] != -1)
                        				&& (get_config_value_by_keyname((uint8_t*)config_line, config_keyname_bak[i], &u_value, 'i') == 0)){
                        			fild_keyname_bak[i] = -1; // 标记已读取
                        			switch(i){
                        			case 0:
                        				gps_check = (uint8_t)u_value;
                        				break;
                        			case 1:
                        				gps_startup = (uint8_t)u_value;
                        				break;
                        			case 2:
                        				gps_next_startup_time = u_value;
                        				break;
                        			case 3:
                        				gps_uart_baudrate = u_value;
                        				break;
                        			case 4:
                        				gps_up_rate = u_value;
                        				break;
                        			case 5:
                        				gps_mode = (uint8_t)u_value;
                        				break;
                        			}
                        			break;
                        		}
                        	}
                        }
                        if(back_keyname == 3){   // 备份
                        	config_keyname_bak = get_keyname_by_section(config_section_bak, (uint8_t*)&keyname_size);
                        	if(fild_keyname_bak != NULL){
                        		free(fild_keyname_bak);
                        		fild_keyname_bak = NULL;
                        	}
                        	fild_keyname_bak = (int8_t*)malloc(keyname_size);
                        	for(i = 0; i < keyname_size; i++){
                        		fild_keyname_bak[i] = i;
                        	}
                        	back_keyname = -1;  // 备份完成
                        	break;
                        }
                    	break;
                    case 4: // [rtc]
                        if(back_keyname == -1){
                        	for(i = 0; i < keyname_size; i++){
                        		if((fild_keyname_bak[i] != -1)
                        				&& (get_config_value_by_keyname((uint8_t*)config_line, config_keyname_bak[i], &u_value, 'i') == 0)){
                        			fild_keyname_bak[i] = -1; // 标记已读取
                        			switch(i){
                        			case 0:
                        				rtc_ex_input_freq = (uint32_t)u_value;
                        				break;
                        			case 1:
                        				rtc_ex_cal_offset = (uint8_t)u_value;
                        				break;
                        			case 2:
                        				break;
                        			}
                        			break;
                        		}
                        	}
                        }
                        if(back_keyname == 4){   // 备份
                        	config_keyname_bak = get_keyname_by_section(config_section_bak, (uint8_t*)&keyname_size);
                        	if(fild_keyname_bak != NULL){
                        		free(fild_keyname_bak);
                        		fild_keyname_bak = NULL;
                        	}
                        	fild_keyname_bak = (int8_t*)malloc(keyname_size);
                        	for(i = 0; i < keyname_size; i++){
                        		fild_keyname_bak[i] = i;
                        	}
                        	back_keyname = -1;  // 备份完成
                        	break;
                        }
                    	break;
                    case 5:
                    	break;
                    }
                }
    		}
        	free(fild_bak);
        	fild_bak = NULL;
        	free(fild_keyname_bak);
        	fild_keyname_bak = NULL;
    	}
    	f_close(&config_file);
    }
    free(config_line);
    config_line = NULL;
    free(dir_path);
    dir_path = NULL;
}

static void save_config_to_sd(const char* _root_path){  // 废弃函数
    char* dir_path = NULL;

    DIR config_dir = { 0 };
    FRESULT res = FR_OK;
    FIL config_file = { 0 };
    char _config_path[64];

    char *config_line = NULL;
    size_t line_size = 256;

    dir_path = (char*)malloc(32);
    if(dir_path != NULL) memset(dir_path, 0, 32);
    else{
    	return;
    }

    strcpy(dir_path, _root_path);  // 0:/
    strcat(dir_path, "config");    // 0:/config

    res = f_chdir(_root_path);     // 切换到根目录
    res = f_opendir(&config_dir, dir_path);
    if(res != FR_OK){
    	if(res == FR_NO_PATH){    // 目录不存在
            res = f_mkdir(dir_path);
    	}
    }
    f_closedir(&config_dir);

	memset(_config_path, 0, 64);
	strcpy(_config_path, dir_path);
	strcat(_config_path, "/config.txt");  // 0:/config/config.txt
    res = f_open(&config_file, _config_path, FA_CREATE_ALWAYS | FA_WRITE);
    if(res == FR_OK){
    	uint32_t *_saddress = (uint32_t *)&system_hse_value;    // 获取配置变量起始地址
    	uint32_t _i_val = 0;
        static char _c_val[16];
        uint32_t _size = 0, _k = 0;
        uint8_t i = 0, j = 0;
        size_t k = 0;
    	char *config_section_bak = NULL;
    	size_t sec_size = 0;
    	char* *config_keyname_bak = NULL;
    	uint8_t keyname_size = 0;

        config_line = (char*)malloc(line_size);
        if(config_line == NULL) return;
        memset(config_line, 0, line_size);
        sec_size = get_section_size();
        for(i = 0; i < sec_size; i++){
        	config_section_bak = get_section_by_index(i);   // 获取 section
        	strcpy(&config_line[0], config_section_bak);
        	strcat(config_line, "\r\n");
        	_size = (uint32_t)strlen(config_line);
        	f_write(&config_file, config_line, _size, NULL);
        	config_keyname_bak = get_keyname_by_section(config_section_bak, &keyname_size);
        	for(j = 0; j < keyname_size; j++){
        		k = strlen(config_keyname_bak[j]);
                memcpy_dma(config_line, config_keyname_bak[j], k);  // 复制变量到内存
                strcpy(&config_line[k], "=");
                k += 1;
                _i_val = *(uint32_t*)(_saddress++);     // 获取配置变量地址, 取值
                if(_i_val < 0) _k = sprintf(_c_val, "%d\r\n", (int)_i_val);
                else _k = sprintf(_c_val, "%u\r\n", _i_val);
                strcpy(&config_line[k], _c_val);
                _size = k + _k;
                f_write(&config_file, config_line, _size, NULL);
        	}
        }
        f_close(&config_file);
        free(config_line);
        config_line = NULL;
    }
    if(dir_path != NULL) free(dir_path);
    dir_path = NULL;
}


static uint8_t cmd_scan(char* in_data, char* out_cmd, char* out_val, char* out_data){
	// ">set val data"
	// ">get val"
	uint8_t _ret = 0;
	size_t _in_data_len = 0;
	uint16_t i = 0;
	char* _in = NULL;
	uint8_t fild_space = 0;

	fild_space = fild_space;

	if((in_data[0] != CMD_TX_SIGN)) return 1;
//	if(islower((unsigned char)in_data[1] == 0)) return 1;
	else{
		_in_data_len = strlen((char*)in_data);
		_in = &in_data[1];
		_in_data_len -= 1;
	}

	i = 0;
	while(_in[i] != '\0'){  // 获取命令
        if(isspace(_in[i]) == 0){ // 不是空格
            *out_cmd = _in[i];
            out_cmd += 1;
        }else{  // 空格, 结束
        	*out_cmd = '\0';
        	_in = &_in[i + 1];
        	fild_space = 1;
        	break;
        }
        i += 1;
        if(_in[i] == '\0'){
            *out_cmd = '\0';
            return 0;
        }
//        if((_in[i] == '\0') && (!fild_space)){
//            _ret = 1;
//        	return _ret;
//        }
	}
	i = 0;
	fild_space = 0;
	while(_in[i] != '\0'){    // 获取参数
        if(isspace(_in[i]) == 0){ // 不是空格
            *out_val = _in[i];
            out_val += 1;
        }else{  // 空格, 结束
        	*out_val = '\0';
        	_in = &_in[i + 1];
        	fild_space = 1;
        	break;
        }
        i += 1;
//        if((_in[i] == '\0') && (!fild_space)){
//            _ret = 1;
//        	return _ret;
//        }
        if(_in[i] == '\0'){
            *out_val = '\0';
            return 0;
        }
	}
	i = 0;
	fild_space = 0;
	while((_in[i] != '\0')){   // 获取参数值
        if(isspace(_in[i]) == 0){ // 不是空格
            *out_data = _in[i];
            out_data += 1;
        }else{  // 空格, 结束
        	*out_data = '\0';
        	fild_space = 1;
        	break;
        }
        i += 1;
        if(_in[i] == '\0') *out_data = '\0';
	}
	_ret = 0;
	return _ret;
}

static uint32_t set_config_value(char* in_config_name, char* in_value){   // 废弃函数
    uint8_t ret = 0;
    uint8_t i = 0;
    char* _in_str = NULL;
    char _sec_name[16];

    _in_str = in_config_name;
    while((_in_str[i] != '_') && (_in_str[i] != '\0')){
        _sec_name[i] = _in_str[i];
        i += 1;
    }
    _sec_name[i] = '\0';
    if(strcmp(_sec_name, "system") == 0){
        if(strcmp(in_config_name, "system_hse_value") == 0){
        	if(in_value != NULL) system_hse_value = strtol(in_value, NULL, 10);
        	else return system_hse_value;
        }else if(strcmp(in_config_name, "system_core_freq") == 0){
        	if(in_value != NULL) system_core_freq = strtol(in_value, NULL, 10);
        	else return system_core_freq;
        }else if(strcmp(in_config_name, "system_uart_baudrate") == 0){
        	if(in_value != NULL){
            	system_uart_baudrate = strtol(in_value, NULL, 10);       // 更改系统串口波特率
            	uart_stop(USART3);
            	uart_init(USART3, system_uart_baudrate);
        	}else{
        		return system_uart_baudrate;
        	}
        }
    }else if(strcmp(_sec_name, "sd") == 0){
        if(strcmp(in_config_name, "sd_format") == 0){
        	if(in_value != NULL) sd_format = strtol(in_value, NULL, 10);
        	else return sd_format;
        }else if(strcmp(in_config_name, "sd_format_file_system") == 0){
        	if(in_value != NULL){
            	if(strcmp(in_value, "fat32") == 0){
            		sd_format_file_system = 0x02;
            	}else if(strcmp(in_value, "exfat") == 0){
            		sd_format_file_system = 0x04;
            	}
        	}else{
        		return sd_format_file_system;
        	}
        }else if(strcmp(in_config_name, "sd_file_size_limit") == 0){
        	if(in_value) sd_file_size_limit = strtol(in_value, NULL, 10);
        	else return sd_file_size_limit;
        }
    }else if(strcmp(_sec_name, "ads") == 0){
        if(strcmp(in_config_name, "ads_conver_rate") == 0){
        	if(in_value != NULL) ads_conver_rate = strtol(in_value, NULL, 10);
        	else return ads_conver_rate;
        }else if(strcmp(in_config_name, "ads_mode") == 0){
        	if(in_value != NULL) ads_mode = strtol(in_value, NULL, 10);
        	else return ads_mode;
        }else if(strcmp(in_config_name, "ads_start_time") == 0){
        	if(in_value != NULL) ads_start_time = strtol(in_value, NULL, 10);
        	else return ads_start_time;
        }else if(strcmp(in_config_name, "ads_offset_calibration") == 0){
        	if(in_value != NULL) ads_offset_calibration = strtol(in_value, NULL, 10);
        	else return ads_offset_calibration;
        }else if(strcmp(in_config_name, "ads_gain_calibration") == 0){
        	if(in_value != NULL) ads_gain_calibration = strtol(in_value, NULL, 10);
        	else return ads_gain_calibration;
        }
    }else if(strcmp(_sec_name, "gps") == 0){
        if(strcmp(in_config_name, "gps_check") == 0){
        	if(in_value != NULL) gps_check = strtol(in_value, NULL, 10);
        	else return gps_check;
        }else if(strcmp(in_config_name, "gps_startup") == 0){
        	if(in_value != NULL) gps_startup = strtol(in_value, NULL, 10);
        	else return gps_startup;
        }else if(strcmp(in_config_name, "gps_next_startup_time") == 0){
        	if(in_value != NULL) gps_next_startup_time = strtol(in_value, NULL, 10);
        	else return gps_next_startup_time;
        }else if(strcmp(in_config_name, "gps_uart_baudrate") == 0){         // 更改GPS串口波特率
        	if(in_value != NULL){
            	gps_uart_baudrate = strtol(in_value, NULL, 10);
            	gps_set_baudrate(gps_uart_baudrate);
            	uart_stop(UART4);
            	uart_init(UART4, gps_uart_baudrate);
        	}else return gps_uart_baudrate;

        }else if(strcmp(in_config_name, "gps_up_rate") == 0){
        	if(in_value != NULL) gps_up_rate = strtol(in_value, NULL, 10);
        	else return gps_up_rate;
        }else if(strcmp(in_config_name, "gps_mode") == 0){
        	if(in_value != NULL) gps_mode = strtol(in_value, NULL, 10);
        	else return gps_mode;
        }
    }
    return ret;
}

void mco_use_for_ads_clk(uint32_t in_clk_freq){
	// MCO Pin
	LL_GPIO_SetPinMode(ADS_CLK_GPIO_Port, ADS_CLK_Pin, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinOutputType(ADS_CLK_GPIO_Port, ADS_CLK_Pin, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(ADS_CLK_GPIO_Port, ADS_CLK_Pin, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinPull(ADS_CLK_GPIO_Port, ADS_CLK_Pin, LL_GPIO_PULL_NO);
	LL_GPIO_SetAFPin_8_15(ADS_CLK_GPIO_Port, ADS_CLK_Pin, LL_GPIO_AF_0);

	LL_RCC_ClocksTypeDef rcc_clk = { 0 };
	uint32_t _div = 1;
	LL_RCC_GetSystemClocksFreq(&rcc_clk);
	in_clk_freq = 4000000;
    _div = rcc_clk.SYSCLK_Frequency / in_clk_freq;
    if(_div == 1) _div = LL_RCC_MCO1_DIV_1;
    if(_div == 2) _div = LL_RCC_MCO1_DIV_2;
    if((_div > 2) && (_div <= 4)) _div = LL_RCC_MCO1_DIV_4;
    if((_div > 4) && (_div <= 8)) _div = LL_RCC_MCO1_DIV_8;
    if((_div > 8)) _div = LL_RCC_MCO1_DIV_16;
	LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_SYSCLK, _div);   // 输出 ADC 采样时钟 4MHz
}

void lptim1_irq_callback(void){
	if(LL_LPTIM_IsActiveFlag_CMPM(LPTIM1)){
		LL_LPTIM_ClearFLAG_CMPM(LPTIM1);
	    if((LL_GPIO_ReadOutputPort(ADS_START_GPIO_Port) & ADS_START_Pin) == 0){
	        LL_GPIO_SetOutputPin(ADS_START_GPIO_Port, ADS_START_Pin);        // 拉高 start 引脚, 开始转换
	    }
	}
	if(LL_LPTIM_IsActiveFlag_ARRM(LPTIM1)){
		LL_LPTIM_ClearFLAG_ARRM(LPTIM1);
	    if((ads_control & ADS_STATUS_START) == 0){
	//    	ads_control = ADS_STATUS_START;
	    }
	}
}


void usb_dection_exti_callback(void){
	if(task_fild & TASK_ADS){
		if(usb_is_connect()){  // 高电平,  USB插入
			ads_drdy_exti_disable();
			task_fild |= (TASK_USB_DECTION | TASK_ADS_STOP);
			task_fild &= (~TASK_ADS);
		}
	}else{
		task_fild |= TASK_USB_DECTION;
	}
}

void imu_exti_callback(void){
	imu_bhy_int_callback();
}

void gps_pps_cunt_callback(void){
	register uint32_t _systick_value = SysTick->VAL;

	gps_pps_cunt += 1;
	if((system_calibration_ctrl & SYS_CAL_CTRL_SET)){
		if(system_calibration_ctrl & SYS_CAL_CTRL_START){
			systick_enable();
			system_calibration_ctrl &= (~SYS_CAL_CTRL_START);
			gps_pps_cunt = 0;
			return;
		}else{
			if(system_calibrate_offset) system_calibrate_offset[(gps_pps_cunt) % SYS_CAL_SIZE] = _systick_value; // 获取计数值
			system_calibration_ctrl |= SYS_CAL_CTRL_STOP;
			return;
		}
	}
}

void systick_callback(void){
    // TODO
    system_tick_cunt += 1;
}

void sys_ms_calibration_callback(void){
	register uint32_t _systick_value = SysTick->VAL;
    register uint32_t _systick_cunt = system_tick_cunt;
    gps_pps_cunt += 1;
    if(system_calibration_ctrl & SYS_CAL_CTRL_SET){
    	if(system_calibration_ctrl & SYS_CAL_CTRL_START){
    		systick_enable();
    		system_calibration_ctrl &= (~SYS_CAL_CTRL_START);
    		gps_pps_cunt = 0;
    		system_tick_cunt = 0;
    		return;
    	}else{
    		if(system_calibrate_offset){
    			system_calibrate_offset[(gps_pps_cunt) % SYS_CAL_SIZE] = _systick_value; // 获取计数值
    			system_calibrate_offset[SYS_CAL_SIZE + (gps_pps_cunt) % SYS_CAL_SIZE] = _systick_cunt;  // 获取毫秒数
    		}
    		system_calibration_ctrl |= SYS_CAL_CTRL_STOP;
    		return;
    	}
    }
}

uint8_t cmd_printf(const char* fmt, ...){
	if(!(cmd_status & CMD_STATUS_INIT)) return 1;
	uint8_t ret = 0, n = 0;
    char _fmt_str[128];
    va_list arp;
    va_start(arp, fmt);
    n = vsnprintf(_fmt_str, 128, fmt, arp);
    va_end(arp);
    ret = UART3_Write((uint8_t*)_fmt_str, n);
    return ret;
}

uint8_t f_log_printf(FIL* _log_file, const char* fmt, ...){
	if((_log_file == NULL) || (_log_file->obj.fs == NULL)) return 1;
	uint8_t ret = 1;
	va_list arp;
    char _time[256] = { 0 };
    uint32_t _l = 0, _tick = 0;
    struct tm _tm;

    _tick = HAL_GetTick();
    st_rtc_get_time(&_tm);
    _l = snprintf(_time, sizeof(_time), "%d-%d-%d-%d-%d-%d.%u: ", _tm.tm_year + 1900, _tm.tm_mon, _tm.tm_mday, _tm.tm_hour, _tm.tm_min, _tm.tm_sec, _tick);

    ret = f_write(_log_file, _time, _l, NULL);
    va_start(arp, fmt);
    _l = vsnprintf(_time, sizeof(_time), fmt, arp);
    va_end(arp);
    ret = f_write(_log_file, _time, _l, NULL);
    ret = f_sync(_log_file);
    return ret;
}

static void sd_switch_connect(uint8_t in_device){
	if(in_device){   // MCU
		// 断电
//		VSD_Control(RESET);
//		VBUS_Control(RESET);
		TF_Connect_To_MCU();
		HAL_Delay(1000);
		// 上电
		VSD_Control(SET);
	}else{           // USB
		// 断电
//		VSD_Control(RESET);
//		VBUS_Control(RESET);
		TF_Connect_To_USB();
		HAL_Delay(1000);
		// 上电
		VBUS_Control(SET);
		VSD_Control(SET);
	}
}

#define R1         200.00f
#define R2         47.00f
int8_t get_battery_level(void){
	int8_t _battery_level = -1;
    uint16_t _ex = 0;
    float _exd = 0.00f;
    _ex = get_extern_analog_voltage();
    _exd = _ex * ((float)((R1 + R2) / R2) + (system_battery_error_voltage / 1000.00f));
    _battery_level = (int8_t)(((_exd - system_battery_cutoff_voltage) / (system_battery_voltage - system_battery_cutoff_voltage)) * 100 + 0.50f);
	return _battery_level;
}

void enter_sleep_mode(void){
	CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPONEXIT_Msk));  // 立即进入Sleep
//	SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPONEXIT_Msk));    // 响应完所有中断再进入中断
	CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
	__WFI();
}

void enter_power_mode(uint32_t power_mode){

	VSD_Control(RESET);
	if (LL_PWR_IsActiveFlag_SB() != 0){
		LL_PWR_ClearFlag_SB();
	}
	if (LL_PWR_IsActiveFlag_WU2() != 0){
		LL_PWR_ClearFlag_WU2();
	}
	LL_PWR_DisableWakeUpPin(LL_PWR_WAKEUP_PIN2);
	LL_PWR_ClearFlag_WU();
//	LL_PWR_SetWakeUpPinPolarityLow(LL_PWR_WAKEUP_PIN2);
//	LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PIN2);
	LL_PWR_SetPowerMode(power_mode);
	LL_LPM_EnableDeepSleep();

	  /* This option is used to ensure that store operations are completed */
	#if defined ( __CC_ARM)
	  __force_stores();
	#endif
	  /* Request Wait For Interrupt */
	  __WFI();
}


static void rtc_timeout_callback(void){
	rtc_sd_timeout_count += 1;
	rtc_rads_timeout_count += 1;
	if(!(rtc_sd_timeout_count % (RTC_SD_WR_TIMEOUT / RTC_TIMEOUT))){        // SD卡写入超时, 可能SD出现问题, 复位SD卡
		ads_drdy_exti_disable();
		reset_sd();
		ads_drdy_exti_enable();
	}

	if(!(rtc_rads_timeout_count % (RTC_ADS_READ_TIMEOUT / RTC_TIMEOUT))){    // ADS数据读取超时
		ads_drdy_exti_disable();
		spi_deinit(SPI1);
        spi_init(SPI1);
        ads_read_data_by_dma_init(NULL, NULL, 0);
        ads_drdy_exti_enable();
	}
}

void rtc_reset_sd_timeout(void){
	rtc_sd_timeout_count = 0;
}

void rtc_reset_ads_timeout(void){
	rtc_rads_timeout_count = 0;
}

static void reset_sd(void){
    BSP_SD_DeInit();
}

static uint32_t save_ads_file_times = 0;
static void pericdic_close_ads_file(void){
     if(save_ads_file_times != ads_get_save_times()){
    	 save_ads_file_times = ads_get_save_times();
     }else{  // 文件保存出现问题
    	 reset_sd();
     }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
     SEGGER_RTT_printf(0, "Error: %s on line %d\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
