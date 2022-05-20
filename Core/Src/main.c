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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmd.h"
#include "gps.h"
#include "fatfs.h"
#include "config_ini.h"

#include "bsp_ads127.h"

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
#if(0)
#define CMD_STATUS_INIT                0x01
#define CMD_STATUS_CONNECT             0x02

#define GPS_STATUS_INIT                0x01
#define GPS_STATUS_ANT_OK              0x02
#define GPS_STATUS_ANT_NO_EXIST        0x20
#define GPS_STATUS_PPS_OK              0x04
#define GPS_STATUS_TIME                0x08

#define ADS_STATUS_INIT                0x01
#define ADS_STATUS_START               0x02
#define ADS_STATUS_RESET               0x04
#define ADS_STATUS_STOP                0x08
#define ADS_STATUS_DELAY               0x10


#define TIM_STATUS_INIT                0x01
#define TIM_STATUS_SET                 0x02
#define TIM_STATUS_START               0x04
#define TIM_STATUS_STOP                0x08

#define ADC_STATUS_INIT                0x01
#define ADC_STATUS_START               0x02
#define ADC_STATUS_STOP                0x04

#define WAKEUP_FROM_RUN_MODE           0x00
#define WAKEUP_FROM_SLEEP_MODE         0x01
#define WAKEUP_FROM_LP_RUN_MODE        0x02
#define WAKEUP_FROM_LP_SLEEP_MODE      0x04
#define WAKEUP_FROM_STOP01_MODE        0x08
#define WAKEUP_FROM_STOP2_MODE         0x10
#define WAKEUP_FROM_STANDBY_MODE       0x20
#define WAKEUP_FROM_SHUTDOWN_MODE      0x40

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

#define RAM2_START_ADDR               0x10000000U

/* USER CODE BEGIN PV */

static __aligned(4) uint8_t workBuffer[FF_MAX_SS] = { 0 };
static FATFS SDFatFS;    /* File system object for SD logical drive */

const static uint32_t HSE_FREQ = 16000000;

static FIL log_file = { 0 };
//static FIL ads_data_file = { 0 };
//static char ads_file_name[FF_MAX_LFN] = {'\0'};

typedef struct {
	FIL file;
    char file_name[FF_MAX_LFN];
    char suffix[5];
}f_file_t;

f_file_t f_ads_data_file = {0};


static const char* ads_data_file_fmt_suffix[] = {".txt", ".bin"};

__IO uint32_t task_fild = TASK_ILDE, task_fild_bak = 0;
__IO uint64_t task_ilde = 0;

//static int8_t local_time_zone = 127;

static __IO uint8_t led_control = 0;
static __IO float led_high_time = 0.030f; /* time = (led_high_time * 1000 / led_blink_freq) ms */
static __IO float led_blink_freq = 1.000f;

static __IO uint8_t adc_control = 0;

__IO uint8_t ads_control = 0;

static __IO uint8_t cmd_status = 0;

#define SYS_CAL_CTRL_SET                   0x10
#define SYS_CAL_CTRL_START                 0x08
#define SYS_CAL_CTRL_STOP                  0x04
#define SYS_CAL_CTRL_TEST                  0x02
#define SYS_CAL_STATUS_CPLT                0x01

#define SYS_CAL_SIZE                       (33)
static __IO uint8_t system_calibration_ctrl = 0x00;                    // 此值使用 LL_RTC_BKP_DR1 备份寄存器备份
    // 使用 LL_RTC_BKP_DR2 备份寄存器备份偏移值
                                                    // 使用LL_RTC_BKP_DR3 存储 1ms 定时校准值

static __IO uint32_t system_utc_time = 0;
static __IO uint32_t system_tick_cunt = 0;
/* USER CODE END PV */

/**
 * ini parse
 */
typedef void (*config_changed_callback_t)(void);

typedef struct {
    #define INI_CFG(s, n, v_default) int32_t s##_##n;
    #include "system_config.def"
#if defined(CONFIG_CALLBACK) && (CONFIG_CALLBACK)
    #define INI_CFG(s, n, v_default) config_changed_callback_t s##_##n##_cb;    // 定义设置改变回调函数指针
    #include "system_config.def"
#endif
} configuration_t;

configuration_t config __attribute__((section(".data"))) = {
    #define INI_CFG(s, n, v_default) .s##_##n = v_default,
    #include "system_config.def"
#if defined(CONFIG_CALLBACK) && (CONFIG_CALLBACK)
    #define INI_CFG(s, n, v_default) .s##_##n##_cb = NULL,
    #include "system_config.def"
#endif
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
#define INI_CFG(s, n, v_default) if((strcmp(section, #s) == 0) && (strcmp(key_name, #n) == 0)){ \
	                                 pconfig->s##_##n = atol(value); \
	                                 if(pconfig->s##_##n##_cb != NULL) pconfig->s##_##n##_cb(); \
	                                 return 0; \
                                 }

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
    uint32_t len = 0;
    UNUSED(handler);

#define INI_CFG(s, n, v_default) if(strncmp(&section[1], #s, strlen(#s))) \
	                             { \
                                     len = snprintf(section, sizeof(section), "[%s]\r\n", #s); \
                                     f_write(file, section, len, NULL); \
	                             } \
                                 len = snprintf(key_value, sizeof(key_value), "%s_%s = %ld\r\n", #s, #n, pconfig->s##_##n); \
                                 f_write(file, key_value, len, NULL);

    #include "system_config.def"

    return (int)res;
}

int save_ini_file_to_sd(const char* _root_path, ini_handler handler, void* user){
    char dir_path[32] = {'\0'}, tmp_path[32] = {'\0'};
    int res = FR_OK;
    FIL _config_file = { 0 };
    f_getcwd(tmp_path, 32);

    strcpy(dir_path, _root_path);  // 0:/
    strcat(dir_path, "config/config.ini");    // 0:/config
    res = f_open(&_config_file, dir_path, FA_CREATE_ALWAYS | FA_WRITE);
    if(res == FR_DISK_ERR) return res;
    if((res != FR_OK)){
        res = f_chdir(_root_path);     // 切换到根目录
    	res = f_mkdir("config");
    	if(!((res == FR_OK) || (res == FR_EXIST))) return res;   // 目录不存在 并且无法创建目录
    	res = f_open(&_config_file, dir_path, FA_CREATE_ALWAYS | FA_WRITE);
    }
    if(res) return res;
    res = f_ini_dump_file(&_config_file, handler, user);
    res += f_close(&_config_file);
    res += f_chdir(tmp_path);     // 切换到根目录
    return res;
}

int parse_iso8601_time_from_string(const char *str, struct tm* _tm){  // yyyy-mm-ddThh:mm:ss
   char tmp[8] = {'\0'};
   int pos = -1;
   unsigned char fild = 0x00;
   char t = 0;
   int i = 0, k = 0;

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
        	   if(k <= 2) _tm->tm_mon = atoi(tmp);
        	   break;
           case 3:
        	   if(k <= 2) _tm->tm_mday = atoi(tmp);
        	   break;
           case 4:
        	   if(k <= 2) _tm->tm_hour = atoi(tmp);
        	   break;
           case 5:
        	   if(k <= 2) _tm->tm_min = atoi(tmp);
        	   break;
           case 6:
        	   if(k <= 2) _tm->tm_sec = atoi(tmp);
        	   break;
           }
           i = 0;
       }
   }while(t != '\0');
   return 0;
}

uint8_t save_time_to_file(const char* _root_path, struct tm _tm, const char* prefix){

	char time_str[24] = {'\0'};
	char _path[48] = {'\0'};
	uint8_t ret = 0;

	format_time_to_string(time_str, sizeof(time_str), _tm);   // 格式化时间字符串

    snprintf(_path, sizeof(_path), "%s/%s-%s", _root_path, prefix, time_str);

	FIL time_file = {0};
	ret = f_open(&time_file, _path, FA_WRITE | FA_CREATE_ALWAYS);
	if(ret != FR_OK) return ret;
	ret = f_write(&time_file, time_str, strlen(time_str), NULL);
    f_close(&time_file);
	return ret;
}


int32_t get_config(void* user, const char* section, const char* name){
	if(user == NULL) goto return_section;
	configuration_t* pconfig = (configuration_t*)user;

#define INI_CFG(s, n, v_default) if((strncmp(section, #s, strlen(#s)) == 0) && (strncmp(name, #n, strlen(#n)) == 0)){ \
	                                 return pconfig->s##_##n; \
                                 }

    #include "system_config.def"
return_section:
    return -1;
}

int32_t set_config(void* user, const char* section, const char* name, int32_t value){
	if(user == NULL) goto return_section;
	configuration_t* pconfig = (configuration_t*)user;

#if defined(CONFIG_CALLBACK) && (CONFIG_CALLBACK)
#define INI_CFG(s, n, v_default) if((strcmp(section, #s) == 0) && (strcmp(name, #n) == 0)){ \
                                     pconfig->s##_##n = value; \
                                     if(pconfig->s##_##n##_cb != NULL) pconfig->s##_##n##_cb(); \
                                     return 0; \
                                 }
#else
#define INI_CFG(s, n, v_default) if((strcmp(section, #s) == 0) && (strcmp(name, #n) == 0)){ \
                                     pconfig->s##_##n = value; \
                                     return 0; \
                                 }
#endif

    #include "system_config.def"

	return_section:
	return -1;
}

#if defined(CONFIG_CALLBACK) && (CONFIG_CALLBACK)
void registe_config_callback(void* user, const char* section, const char* name, void* cb){
	if(user == NULL) return;
	configuration_t* pconfig = (configuration_t*)user;

#define INI_CFG(s, n, v_default) if((strcmp(section, #s) == 0) && (strcmp(name, #n) == 0)){ \
                                     pconfig->s##_##n##_cb = (config_changed_callback_t)cb; \
                                 }
    #include "system_config.def"

}

void unregiste_config_callback(void* user, const char* section, const char* name){
	if(user == NULL) return;
	configuration_t* pconfig = (configuration_t*)user;

#define INI_CFG(s, n, v_default) if((strcmp(section, #s) == 0) && (strcmp(name, #n) == 0)){ \
                                     pconfig->s##_##n##_cb = NULL; \
                                 }
    #include "system_config.def"
}
#endif


uint8_t save_tmp_file_name(const char* _path, const char* in_file_name){
	uint8_t ret = 0;
    FIL fp = {0};
    char _p[FF_MAX_LFN / 2] = {'\0'};

//    if(_path) strcpy(_p, _path);
//    else f_getcwd(_p, sizeof(_p));

    strcat(_p, "tmp_file_name.txt");

    ret = f_open(&fp, _p, FA_WRITE | FA_CREATE_ALWAYS);
    if((ret != FR_OK)) return ret;
    f_printf(&fp, "%s\n", in_file_name);
    f_close(&fp);
    return ret;
}

uint8_t get_tmp_file_name(const char* _path, char* out_file_name){
	uint8_t ret = 0;
    FIL fp = {0};
    char _p[FF_MAX_LFN / 2] = {'\0'};
    char *rc = NULL;

//    if(_path) strcpy(_p, _path);
//    else f_getcwd(_p, sizeof(_p));

    strcat(_p, "tmp_file_name.txt");

    ret = f_open(&fp, _p, FA_READ | FA_OPEN_EXISTING);
    if(ret != FR_OK){
    	out_file_name[0] = '\0';
    	return 1;
    }
    rc = f_gets(out_file_name, 32, &fp);
    if(rc == NULL) out_file_name[0] = '\0';
    f_close(&fp);
    return (rc ? 0 : 1);
}

uint8_t remove_tmp_file_name(const char* _path, const char* in_file_name){
	uint8_t ret = 0;
    FIL fp = {0}, new_fp = {0};
    char _p[FF_MAX_LFN / 2] = {'\0'};
    char _d[FF_MAX_LFN / 2] = {'\0'};
    char *rc = NULL;
    if(_path) strcpy(_p, _path);
    else f_getcwd(_p, sizeof(_p));

    strcat(_p, "/tmp_file_name");
    ret = f_open(&new_fp, _p, FA_WRITE | FA_CREATE_ALWAYS);   // 打开新文件
    if(ret != FR_OK) return ret;
    strcat(_p, ".txt");
    ret = f_open(&fp, _p, FA_READ | FA_OPEN_EXISTING);   // 打开已经存在的文件
    if(ret != FR_OK){f_close(&new_fp); return ret;};

    while(1){
    	rc = f_gets(_d, sizeof(_d), &fp);
    	if(!((rc != NULL) && !(f_eof(&new_fp)))) break;
        if(strcmp(_d, in_file_name) != 0){
        	f_printf(&new_fp, "%s\n", &new_fp);       // 复制数据
        }
    }

    f_close(&fp);
    f_close(&new_fp);
    if(rc){
        f_unlink(_p);    // 删除旧文件
        strncpy(_d, _p, strlen(_p) - 4);
        f_rename(_d, _p);        // 重命名新文件
    }else{
        strncpy(_d, _p, strlen(_p) - 4);
        f_unlink(_d);    // 删除新文件
    }

    return rc ? 0 : 1;
}

void ads_test_cb(void){
	uint8_t test = (uint8_t)get_config(&config, "ads", "test");
	if(test){   // 测试开始
		CLEAR_BIT(task_fild, TASK_ADS_STOP);
//		SET_BIT(task_fild, TASK_ADS_TEST);
	}
	if(!test){        // 测试结束
        CLEAR_BIT(task_fild, TASK_ADS_TEST);
//        SET_BIT(task_fild, TASK_ADS_STOP);
	}
}

#define ADS_START_TIME_BKP_DR                   LL_RTC_BKP_DR0

void ads_start_time_cb(void){
    uint32_t ads_start_time = 0, _new = 0;
    _new = (uint32_t)get_config(&config, "ads", "start_time");
//    ads_start_time = LL_RTC_BAK_GetRegister(RTC, ADS_START_TIME_BKP_DR);

//    if(ads_start_time != _new){  // 更新
        LL_RTC_BAK_SetRegister(RTC, ADS_START_TIME_BKP_DR, _new);
//    }
}

#define ADS_WORK_TIME_BKP_DR                   (LL_RTC_BKP_DR0 + 1)

void ads_work_time_cb(void){
	int32_t wt = 0, _new = 0;
	wt = (int32_t)LL_RTC_BAK_GetRegister(RTC, ADS_WORK_TIME_BKP_DR);
	_new = (int32_t)get_config(&config, "ads", "work_time");
    if(wt != _new){
    	LL_RTC_BAK_SetRegister(RTC, ADS_WORK_TIME_BKP_DR, (uint32_t)_new);
    }
}

#define ADS_SHUTDOWN_TIME_BKP_DR              (LL_RTC_BKP_DR0 + 2)

void ads_shutdown_time_cb(void){
	int32_t st = 0, _new = 0;
	st = (int32_t)LL_RTC_BAK_GetRegister(RTC, ADS_SHUTDOWN_TIME_BKP_DR);
	_new = (int32_t)get_config(&config, "ads", "shutdown_time");
    if(st != _new){
    	LL_RTC_BAK_SetRegister(RTC, ADS_SHUTDOWN_TIME_BKP_DR, (uint32_t)_new);
    }
}


#define ADS_CONVER_RATE_BKP_DR               (LL_RTC_BKP_DR0 + 3)

void ads_conver_rate_cb(void){
	uint32_t cr = 0, _new = 0;
//	cr = (uint32_t)LL_RTC_BAK_GetRegister(RTC, ADS_CONVER_RATE_BKP_DR);
	_new = (uint32_t)get_config(&config, "ads", "conver_rate");
//    if(cr != _new){
    	LL_RTC_BAK_SetRegister(RTC, ADS_CONVER_RATE_BKP_DR, (uint32_t)_new);
//    }
}

#define ADS_SHUTDOWN_TIME_BAK_BKP_DR         (LL_RTC_BKP_DR0 + 4)

static void backup_ads_shutdown_time(void){
    register uint32_t _st = 0;
    _st = LL_RTC_BAK_GetRegister(RTC, ADS_SHUTDOWN_TIME_BKP_DR);
    LL_RTC_BAK_SetRegister(RTC, ADS_SHUTDOWN_TIME_BAK_BKP_DR, _st);
}

static void rtc_alarm_cb(void){
	CLEAR_BIT(task_fild, TASK_ADS);

	backup_ads_shutdown_time();

//	SET_BIT(task_fild, TASK_ADS_STOP | TASK_ENTER_LP_MODE);
}

static void gps_startup_cb(void){
    if(get_config(&config, "gps", "startup") > 0){
        SET_BIT(task_fild, TASK_GPS);
    }else{
    	CLEAR_BIT(task_fild, TASK_GPS);
    }
}

void system_get_device_uid(char* id, uint8_t len){
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


uint8_t file_to_cmd(const char* path){
    FIL test_file;
    uint8_t ret = 0;

    ret = f_open(&test_file, path, FA_READ | FA_OPEN_EXISTING);
    if(ret != FR_OK){
    	return ret;
    }
    uint32_t fsz = f_size(&test_file);
    if(fsz == 0) goto end_section;
    uint32_t rbt = 0, rbta = 0;
    uint8_t  rbuf[512] = {0};
    do{
  	    ret = f_read(&test_file, rbuf, sizeof(rbuf), (unsigned int*)&rbt);
        if(ret != FR_OK) break;
        cmd_write(rbuf, rbt);
  	    rbta += rbt;
  	    HAL_Delay(10);
    }while(rbta != fsz);
end_section:
    f_close(&test_file);
    return ret;
}



/**
 * ini parse end
 */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */


static void wwdg_init(uint16_t timeout /* second */){
#if defined(DEBUG)
	LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_APB1_GRP1_WWDG_STOP);
#endif
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_WWDG);

	LL_RCC_ClocksTypeDef rcc_clk = { 0 };
	LL_RCC_GetSystemClocksFreq(&rcc_clk);

	LL_WWDG_SetPrescaler(WWDG, LL_WWDG_PRESCALER_8);

//	uint32_t wwdg_clk = ((rcc_clk.PCLK1_Frequency) >> 12 >> 3);

	LL_WWDG_SetWindow(WWDG,0x7E);
	LL_WWDG_SetCounter(WWDG, 0X7E);
	LL_WWDG_Enable(WWDG);
}


static void iwdg_init(uint16_t timeout /* second */){
#define IWDG_TIMEOUT_MAX    32
#if defined(DEBUG)
	LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_APB1_GRP1_IWDG_STOP);
#endif
	if(LL_RCC_IsActiveFlag_IWDGRST()){
		/* clear IWDG reset flag */
		LL_RCC_ClearResetFlags();
	}
	LL_RCC_LSI_Enable();
	while (LL_RCC_LSI_IsReady() != 1)
	{
	}
	/* Configure the IWDG with window option disabled */
	/* ------------------------------------------------------- */
	/* (1) Enable the IWDG by writing 0x0000 CCCC in the IWDG_KR register */
	/* (2) Enable register access by writing 0x0000 5555 in the IWDG_KR register */
	/* (3) Write the IWDG prescaler by programming IWDG_PR from 0 to 7 - LL_IWDG_PRESCALER_4 (0) is lowest divider*/
	/* (4) Write the reload register (IWDG_RLR) */
	/* (5) Wait for the registers to be updated (IWDG_SR = 0x0000 0000) */
	/* (6) Refresh the counter value with IWDG_RLR (IWDG_KR = 0x0000 AAAA) */
	LL_IWDG_Enable(IWDG);                             /* (1) */
	LL_IWDG_EnableWriteAccess(IWDG);                  /* (2) */
	// 32000 / 256

	LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_256);/* (3) */
	uint16_t _timeout = (timeout * 0xFFF) / IWDG_TIMEOUT_MAX; // (s)
	if(timeout > IWDG_TIMEOUT_MAX) _timeout = 0xFFF;
	LL_IWDG_SetReloadCounter(IWDG, _timeout);            /* (4) */
	while (LL_IWDG_IsReady(IWDG) != 1)                /* (5) */
	{
	}
	LL_IWDG_ReloadCounter(IWDG);                      /* (6) */

#undef IWDG_TIMEOUT_MAX
}


static void iwdg_update(void){
	LL_IWDG_ReloadCounter(IWDG);
}


#if defined(NOT_USE) && (NOT_USE)
/**
 * opt: 0: read, 1: modify
 */
static void read_config_from_sd(const char* _root_path, char* _config_section, void* _config, uint8_t opt);
static void save_config_to_sd(const char* _root_path);
static uint32_t set_config_value(char* in_config_name, char* in_value);
#endif



static void sd_switch_connect(uint8_t in_device);
static void pericdic_close_ads_file(void);
static void reset_sd(void);

static uint32_t gps_pps_counter = 0;
static void gps_pps_cunt_callback(void);
static void gps_time_good_callback(void);

static void tim2_irq_update_handler(void);

#endif

#if defined(USE_USB)
USBD_HandleTypeDef USBD_Device;
#endif
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
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_8, LL_GPIO_PULL_DOWN);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ANALOG);

    LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_HSE, LL_RCC_MCO1_DIV_2);
}

typedef struct __SPI_HandleTypeDef * SPI_HandleTypeDef_Handle;

static SPI_HandleTypeDef_Handle spi1_handle = NULL;
static FATFS *fs = NULL;

static FIL ads_data_file = { 0 };

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

    RTCClockLSE_Config();

    SystemClockHSE_Config(SYS_CORE_FREQ_72M);

    Clock48_Domain_Cofing();

    HAL_Init();

    gpio_all_set_analog();

    /* USER CODE BEGIN Init */
    mco_enable(MCO_FREQ_16M);

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

    if(1){

        st_spi1_init(&spi1_handle);
        ads127_bsp_reset_pin_initial(GPIOC, GPIO_PIN_4);
        ads127_bsp_start_pin_initial(GPIOC, GPIO_PIN_5);

        ads127_bsp_reset();
        ads127_driver_initialaiz(spi1_handle, GPIOA, GPIO_PIN_4);
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
        ads127_bsp_drdy_isr_install(GPIOB, GPIO_PIN_1, ads127_bsp_read_data_from_isr, spi1_handle);
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


#if(0)

#if defined(NOT_USE) && (NOT_USE)
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

                            		uint8_t pos_ = find_chr(config_keyname_bak[i], '_');
                            		char ckn[32];
                            		strncpy(ckn, config_keyname_bak[i], 32);
                            		ckn[pos] = '\0';
                                    set_config(_config, ckn, &ckn[pos + 1], u_value);

//                                    switch(i){
//                                    case 0:
//                                    	system_hse_value = u_value;
//                                    	break;
//                                    case 1:
//                                    	system_core_freq = u_value;
//                                    	break;
//                                    case 2:
//                                    	system_rco_div = u_value;
//                                    	break;
//                                    case 3:
//                                    	system_uart_baudrate = u_value;
//                                    	break;
//                                    case 4:
//                                    	system_cmd_connect_time = u_value;
//                                    	break;
//                                    case 5:
//                                    	system_battery_voltage = u_value;
//                                    	break;
//                                    case 6:
//                                    	system_battery_cutoff_voltage = u_value;
//                                    	break;
//                                    case 7:
//                                    	system_battery_thold_voltage = u_value;
//                                    	break;
//                                    case 8:
//                                    	system_battery_error_voltage = (int32_t)u_value;
//                                    	break;
//                                    }
//                                    break;  // 终止循环
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
                            		uint8_t pos_ = find_chr(config_keyname_bak[i], '_');
                            		char ckn[32];
                            		strncpy(ckn, config_keyname_bak[i], 32);
                            		ckn[pos] = '\0';
                                    set_config(_config, ckn, &ckn[pos + 1], u_value);
//                            		switch(i){
//                            		case 0:
//                            			ads_conver_rate = u_value;
//                            			break;
//                            		case 1:
//                            			ads_mode = (uint8_t)u_value;
//                            			break;
//                            		case 2:
//                            			ads_start_time = u_value;
//                            		    break;
//                            		case 3:
//                            			ads_offset_calibration = u_value;
//                            			break;
//                            		case 4:
//                            			ads_gain_calibration = u_value;
//                            			break;
//                            		case 5:
//                            			ads_data_file_format = u_value;
//                            			break;
//                            		case 6:
//                            			ads_osr = u_value;
//                            			break;
//                            		}
//                            		break;  // 终止循环
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
                            		uint8_t pos_ = find_chr(config_keyname_bak[i], '_');
                            		char ckn[32];
                            		strncpy(ckn, config_keyname_bak[i], 32);
                            		ckn[pos] = '\0';
                                    set_config(_config, ckn, &ckn[pos + 1], u_value);
//                        			switch(i){
//                        			case 0:
//                        				gps_check = (uint8_t)u_value;
//                        				break;
//                        			case 1:
//                        				gps_startup = (uint8_t)u_value;
//                        				break;
//                        			case 2:
//                        				gps_next_startup_time = u_value;
//                        				break;
//                        			case 3:
//                        				gps_uart_baudrate = u_value;
//                        				break;
//                        			case 4:
//                        				gps_up_rate = u_value;
//                        				break;
//                        			case 5:
//                        				gps_mode = (uint8_t)u_value;
//                        				break;
//                        			}
//                        			break;
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
                            		uint8_t pos_ = find_chr(config_keyname_bak[i], '_');
                            		char ckn[32];
                            		strncpy(ckn, config_keyname_bak[i], 32);
                            		ckn[pos] = '\0';
                                    set_config(_config, ckn, &ckn[pos + 1], u_value);
//                        			switch(i){
//                        			case 0:
//                        				rtc_ex_input_freq = (uint32_t)u_value;
//                        				break;
//                        			case 1:
//                        				rtc_ex_cal_offset = (uint8_t)u_value;
//                        				break;
//                        			case 2:
//                        				break;
//                        			}
//                        			break;
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
#endif

#if defined(NOT_USE) && (NOT_USE)
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
#endif


#if defined(NOT_USE) && (NOT_USE)
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
#endif

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


void usb_dection_exti_callback(void){
	SET_BIT(task_fild, TASK_USB_DECTION);
}

void imu_exti_callback(void){
//	imu_bhy_int_callback();
}

void gps_pps_cunt_callback(void){
	gps_pps_counter += 1;
}

void gps_time_good_callback(void){
	LED_ON();
	HAL_Delay(5000);
	LED_OFF();
}

void systick_callback(void){
    // TODO
    system_tick_cunt += 1;
}


uint8_t f_log_printf(FIL* _log_file, const char* fmt, ...){
	if((_log_file == NULL) || (_log_file->obj.fs == NULL)) return 1;
    if(where_sd_connect() != SD_CONNECT_TO_MCU) return 2;

	uint8_t ret = 1;
	va_list arp;
    char _time[256] = { 0 };
    uint32_t _l = 0, _tick = 0;
    struct tm _tm;

    _tick = HAL_GetTick();
    st_rtc_get_time(&_tm);
    _l = snprintf(_time, sizeof(_time), "\n[%d-%d-%d-%d-%d-%d.%u]: ", _tm.tm_year + 1900, _tm.tm_mon, _tm.tm_mday, _tm.tm_hour, _tm.tm_min, _tm.tm_sec, (unsigned int)_tick);

    ret = f_write(_log_file, _time, _l, NULL);
    va_start(arp, fmt);
    _l = vsnprintf(_time, sizeof(_time), fmt, arp);
    va_end(arp);
    if(_time[_l - 1] != '\n'){          // 检查换行符
    	if(_l < sizeof(_time) - 1){
        	_time[_l] = '\n';
        	_l += 1;
    	}else{
    		_time[_l - 1] = '\n';
    	}

    }
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
//		HAL_Delay(500);
		// 上电
		VSD_Control(SET);
	}else{           // USB
		// 断电
//		VSD_Control(RESET);
//		VBUS_Control(RESET);
		TF_Connect_To_USB();
		HAL_Delay(500);
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
    float system_battery_cutoff_voltage = 0.0f, system_battery_voltage = 0.0f, system_battery_error_voltage = 0.0f;

    system_battery_cutoff_voltage = (float)get_config(&config, "system", "battery_cutoff_voltage");
    system_battery_voltage = (float)get_config(&config, "system", "battery_voltage");
    system_battery_error_voltage = (float)get_config(&config, "system", "battery_error_voltage");

    _ex = get_extern_analog_voltage();
    _exd = _ex * ((float)((R1 + R2) / R2) + (system_battery_error_voltage / 1000.00f));
    _battery_level = (int8_t)(((_exd - system_battery_cutoff_voltage) / (system_battery_voltage - system_battery_cutoff_voltage)) * 100 + 0.50f);
	return _battery_level;
}

void enter_sleep_mode(void){
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	LL_LPM_EnableSleep();
	LL_LPM_EnableSleepOnExit();
//	CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPONEXIT_Msk));  // 立即进入Sleep
//	SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPONEXIT_Msk));    // 响应完所有中断再进入中断
//	CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
	__WFI();
}

void enter_power_mode(uint32_t power_mode){
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	VSD_Control(RESET);
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


//static void rtc_timeout_callback(void){
//	rtc_sd_timeout_count += 1;
//	rtc_rads_timeout_count += 1;
//	if(!(rtc_sd_timeout_count % (RTC_SD_WR_TIMEOUT / RTC_TIMEOUT))){        // SD卡写入超时, 可能SD出现问题, 复位SD卡
//		ads_drdy_exti_disable();
//		reset_sd();
//		ads_drdy_exti_enable();
//	}
//
//	if(!(rtc_rads_timeout_count % (RTC_ADS_READ_TIMEOUT / RTC_TIMEOUT))){    // ADS数据读取超时
//		ads_drdy_exti_disable();
//		spi_deinit(SPI1);
//        spi_init(SPI1);
//        ads_read_data_by_dma_init(NULL, NULL, 0);
//        ads_drdy_exti_enable();
//	}
//}
//
//void rtc_reset_sd_timeout(void){
//	rtc_sd_timeout_count = 0;
//}
//
//void rtc_reset_ads_timeout(void){
//	rtc_rads_timeout_count = 0;
//}

static void reset_sd(void){
//    BSP_SD_DeInit();
}

static uint32_t save_ads_file_times = 0;
static void pericdic_close_ads_file(void){
//     if(save_ads_file_times != ads_get_save_times()){
//    	 save_ads_file_times = ads_get_save_times();
//     }else{  // 文件保存出现问题
//    	 reset_sd();
//     }
}
#endif

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
