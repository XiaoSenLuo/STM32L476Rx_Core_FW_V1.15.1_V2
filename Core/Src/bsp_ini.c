//
// Created by XIAOSENLUO on 2022/6/3.
//


#include "ini.h"
#include "ff.h"
#include "bsp_ini.h"


//#define INI_DEFAULT_FILE_PATH         "0:/config/default.ini"
#define INI_FILE_PATH                 "0:/config/config.ini"

#if(0)
typedef struct {
#define INI_CFG(s, n, v_default) int32_t s##_##n;
#include "system_config.def"
#if defined(CONFIG_CALLBACK) && (CONFIG_CALLBACK)
    #define INI_CFG(s, n, v_default) config_changed_callback_t s##_##n##_cb;    // 定义设置改变回调函数指针
    #include "system_config.def"
#endif
#undef INI_CFG
} configuration_t;

configuration_t config __attribute__((section(".data"))) = {
#define INI_CFG(s, n, v_default) .s##_##n = v_default,
#include "system_config.def"
#if defined(CONFIG_CALLBACK) && (CONFIG_CALLBACK)
        #define INI_CFG(s, n, v_default) .s##_##n##_cb = NULL,
    #include "system_config.def"
#endif
#undef INI_CFG
};
#endif


static int f_ini_parse(const char* filename, ini_handler handler, void* user);
static int f_ini_parse_file(FIL* file, ini_handler handler, void* user);
static int f_ini_dump_file(FIL* file, void* user);

/* process a line of the INI file, storing valid values into config struct */
static int ini_parse_handler(void *user, const char *section, const char *name, const char *value);

static int ini_parse_handler(void *user, const char *section, const char *name, const char *value){
    system_config_t* pconfig = (system_config_t*)user;

#define INI_CFG(s, n, v_default) if((strncmp(section, #s, strlen(#s)) == 0) && (strncmp(name, #n, strlen(#n)) == 0)){ \
	                                 *(uint32_t *)(&pconfig->s.n) = strtoul(value, NULL, 0); \
	                                 return 1; \
                                 }
#include "system_config.def"
#undef INI_CFG
    return 0; /* unknown section/name, error */
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

int parse_ini_file_from_sd(const char* _root_path, void* user){
    char dir_path[32] = {'\0'};
    FIL _config_file = { 0 };
    int res = FR_OK;

//    res = f_chdir(_root_path);     // 切换到根目录
//    if(res) goto return_section;
    strcpy(dir_path, _root_path);  // 0:/
    strcat(dir_path, "config");
    res = f_mkdir(dir_path);
    if((res != FR_OK) && (res != FR_EXIST)) goto return_section;
    strncpy(dir_path, INI_FILE_PATH, 32);
    res = f_open(&_config_file, dir_path, FA_OPEN_EXISTING | FA_READ);
    if((res != FR_OK) && ((res == FR_NO_FILE))){  // // config.ini文件不存在
        res = f_open(&_config_file, dir_path, FA_WRITE | FA_CREATE_ALWAYS);
        if(res) goto return_section;
        f_ini_dump_file(&_config_file, user);
        f_close(&_config_file);
        res = f_open(&_config_file, dir_path, FA_OPEN_EXISTING | FA_READ);
        if(res) goto return_section;
    }
    res = f_ini_parse_file(&_config_file, &ini_parse_handler, user);
    f_close(&_config_file);

    return_section:
//    res = f_chdir(_root_path);     // 切换到根目录
    return res;
}

static int f_ini_dump_file(FIL* file, void* user){
    int res = FR_OK;

    system_config_t * pconfig = (system_config_t  *)user;
    uint32_t len = 0, num = 0, isection = 0;
    char *text_ini = NULL;
    text_ini = (char *)malloc(1024);
    memset(text_ini, '\0', 1024);
/// [section]
/// key=value;
#define INI_CFG(s, n, v_default) do{ \
                                     if(strncmp(&text_ini[isection + 1], #s, strlen(#s))) \
                                     { \
                                         len = snprintf(&text_ini[num], 1024 - num, "[%s]\n", #s); \
                                         isection = num; \
                                         num += len;  \
                                     } \
                                     len = snprintf(&text_ini[num], 1024 - num, "%s = %lu\n", #n, *(uint32_t  *)(&pconfig->s.n)); \
                                     num += len;                              \
                                 }while(0);

#include "system_config.def"
#undef INI_CFG
    f_write(file, text_ini, num, NULL);
    free(text_ini);
    return (int)res;
}

int save_ini_file_to_sd(const char* _root_path, void* user){
    char dir_path[32] = {'\0'}, tmp_path[32] = {'\0'};
    int res = FR_OK;
    FIL _config_file = { 0 };

    strcpy(dir_path, INI_FILE_PATH);    // 0:/config
    memcpy(dir_path, _root_path, strlen(_root_path));  // 0:/
    res = f_open(&_config_file, dir_path, FA_CREATE_ALWAYS | FA_WRITE);
    if((res != FR_OK) && (res == FR_NO_FILE)) return res;
    if((res != FR_OK)){
        char dd[10] = {'\0'};
        strcpy(dd, _root_path);
        strcat(dd, "config");
        res = f_mkdir(dd);
        if(!((res == FR_OK) || (res == FR_EXIST))) return res;   // 目录不存在 并且无法创建目录
        res = f_open(&_config_file, dir_path, FA_CREATE_ALWAYS | FA_WRITE);
    }
    if(res) return res;
    res = f_ini_dump_file(&_config_file, user);
    res += f_close(&_config_file);
    return res;
}

