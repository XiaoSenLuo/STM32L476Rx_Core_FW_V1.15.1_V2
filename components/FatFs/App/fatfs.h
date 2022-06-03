/**
  ******************************************************************************
  * @file   fatfs.h
  * @brief  Header for fatfs applications
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
#ifndef __fatfs_H
#define __fatfs_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "ff.h" /* defines SD_Driver as external */
#include "sd_diskio.h"
#include "ff_gen_drv.h"
/* USER CODE BEGIN Includes */



#define DEL_FILE_BY_MODIFYTIME_UP                    0
#define DEL_FILE_BY_MODIFYTIME_DOWN                  1
/* USER CODE END Includes */

//extern uint8_t retSD; /* Return value for SD */
extern char sd_root_path[4]; /* SD logical drive path */
//extern FATFS SDFatFS; /* File system object for SD logical drive */
//extern FIL SDFile; /* File object for SD */

int FATFS_SD_Init(FATFS* *fs);

int FATFS_SD_DeInit(FATFS* *fs);

/* USER CODE BEGIN Prototypes */
int FS_FileOperations(void);


/**
 *
 * @param out_error_str
 * @param len
 * @param in_error_code
 */
char* fs_error_string(char* out_error_str, uint8_t len, int in_error_code);

/**
 *
 * @param in_path: 指定路径, 如果为 NULL并且文件系统为FAT32, 则查找目前工作目录下的文件, 如果文件系统为EXFAT, 则查找根目录下的文件
 * @param out_file_info: 用于存储文件信息的数组
 * @param out_file_num: 查找到的文件数量
 *
 * @return
 *           0: 查找成功
 */
uint8_t fs_scan_file(const char *in_path, FILINFO *out_file_info, uint32_t *out_file_num);

/**
 *
 * @param in_path
 * @param pattern
 * @param out_file_info
 * @param opt
 * @return
 */
uint8_t fs_find_file(const char *in_path, const char* pattern, FILINFO *out_file_info, char opt);

//uint8_t fs_write_file(FIL* file, const void* data, uint32_t btw);
//
//uint8_t fs_write_string(FIL* file, const char* str);
//
//uint8_t fs_read_file(FIL* file, void* buf, uint32_t btr);


/**
 *
 * @param in_path
 * @param pattern
 * @param opt
 * @return
 */
uint8_t fs_del_file(const char *in_path, const char* pattern, uint8_t opt);

uint8_t fs_create_file(FIL *in_file, const char *in_file_name);

uint8_t fs_create_dir(const char *path);

uint32_t fs_get_total_space(void);

uint32_t fs_get_free_space(void);

/* USER CODE END Prototypes */
#ifdef __cplusplus
}
#endif
#endif /*__fatfs_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
