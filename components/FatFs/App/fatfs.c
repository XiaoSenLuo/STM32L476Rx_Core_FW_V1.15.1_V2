/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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

#include "fatfs.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "utils.h"

char sd_root_path[4] = {'\0'};   /* SD logical drive path */

#define check_FRESULT(ret) {if((ret) != 0) return ret;}

/* USER CODE BEGIN Variables */
#include "stm32l4xx_ll_rtc.h"
/* USER CODE END Variables */

FATFS sd_fatfs = {0};
#if(0)
__aligned(4) uint8_t workBuffer[FF_MAX_SS] = { 0 };
#endif
int FATFS_SD_Init(FATFS* *fs)
{
#if(1)
  /*## FatFS: Link the SD driver ###########################*/
    uint8_t ret = 0;

    /* USER CODE BEGIN Init */
  /* additional user code for init */
   ret = sd_driver_register(sd_root_path);
   if(!ret)
       ret = f_mount(&sd_fatfs, (TCHAR const*)sd_root_path, 1);   // 立即挂载
#if(0)
       if(ret & FR_NO_FILESYSTEM){
        MKFS_PARM mkfs_parm = {.fmt = FM_FAT32, .n_root = 0};
        ret = f_mkfs(sd_root_path, &mkfs_parm, workBuffer, FF_MAX_SS);
        check_FRESULT(ret);
        ret = f_unmount(sd_root_path);
        ret = f_mount(&sd_fatfs, (TCHAR const*)sd_root_path, 1);   // 立即挂载
   }
#endif
   *fs = &sd_fatfs;
	return ret;
#endif
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */

	DWORD ret = 0;

	LL_RTC_TimeTypeDef ll_rtc_time = { .TimeFormat = LL_RTC_HOURFORMAT_24HOUR };
	LL_RTC_DateTypeDef ll_rtc_date = { 0 };

	ll_rtc_time.Seconds = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC));
	ll_rtc_time.Minutes = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC));
	ll_rtc_time.Hours = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC));

	ll_rtc_date.Day = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetDay(RTC));
	ll_rtc_date.Month = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetMonth(RTC));
//	ll_rtc_date.WeekDay = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetWeekDay(RTC));
	ll_rtc_date.Year = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetYear(RTC));

	ret = ((DWORD)((ll_rtc_date.Year + 2000) - 1980) << 25) | ((DWORD)ll_rtc_date.Month << 21) | ((DWORD)ll_rtc_date.Day << 16)
	      | ((DWORD)ll_rtc_time.Hours << 11) | ((DWORD)ll_rtc_time.Minutes << 5) | (ll_rtc_time.Seconds >> 1);
	return ret;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */
int FS_FileOperations(void)
{
  FRESULT res = FR_OK; /* FatFs function common result code */
  uint32_t byteswritten = 0; /* File write/read counts */

  char rtext[32]; /* File read buffer */
  DWORD fre_clust;
  uint32_t fre_sect, tot_sect;
  FIL SDFile;       /* File object for SD */
  FATFS* fs;
  int8_t ret = 0;
  /* Register the file system object to the FatFs module */

//  uint32_t _fn = 0;
  DIR _dir = { 0 };
//                          fs_scan_file(NULL, NULL, &_fn);  // 查找文件数量
  res = f_chdir(sd_root_path);
  if(res) return res;
  FILINFO _fi = { 0 };
  res = f_findfirst(&_dir, &_fi, "", "*.txt");
  while((res == FR_OK) && (_fi.fname[0])){
	  res = f_findnext(&_dir, &_fi);
  }
  f_closedir(&_dir);
  if(res == FR_OK){
    /* Create and Open a new text file object with write access */
	  res = f_mkdir("sd");
	  res = f_mkdir("sd/info");
	  if(!((res == FR_OK) || (res == FR_EXIST))){
		  return 1;
	  }
	  res = f_chdir("sd/info");
	  if(res != FR_OK) return 1;

	  res = f_open(&SDFile, "test.txt", FA_WRITE);
      if(res != FR_OK){
        if(res != FR_NO_FILE) return 1;
        res = f_open(&SDFile, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);
        if(res != FR_OK) return 1;
      }
      char wtext[64]; /* File write buffer */
      memset(wtext, 0, sizeof(wtext));
      strcpy(wtext, "test sd");
      byteswritten = strlen(wtext);
      res = f_write(&SDFile, wtext, byteswritten, NULL);
      if(res != FR_OK) return 1;
      res = f_close(&SDFile);
      if(res != FR_OK) return 1;
      res = f_open(&SDFile, "test.txt", FA_READ);
      if(res != FR_OK) return 1;
      memset(rtext, 0, sizeof(rtext));
      f_gets((TCHAR*)rtext, (int)sizeof(rtext), &SDFile);
      ret = strcmp(rtext, wtext);
      if(ret != 0) ret =  1;
      f_close(&SDFile);
      if(ret != 0) return (uint8_t)ret;
      res = f_getfree(sd_root_path, &fre_clust, &fs);
      if(res != FR_OK) return res;

      tot_sect = ((fs->n_fatent - 2) * fs->csize) >> 1 >> 10 >> 10;  // GB
      fre_sect = (fre_clust * fs->csize) >> 1 >> 10 >> 10;  // GB
      memset(wtext, 0, sizeof(wtext));
      sprintf(wtext, "总容量: %d GiB\r\n剩余容量: %d GiB\r\n", tot_sect, fre_sect);
      res = f_open(&SDFile, "sd info.txt", FA_CREATE_ALWAYS | FA_WRITE);
      if(res != FR_OK) return res;
      f_write(&SDFile, wtext, (uint16_t)strlen(wtext), NULL);
      f_close(&SDFile);
      f_chdir(sd_root_path);  // 返回根目录
  }
  return (uint8_t)res;
}

static const char* err_str[] = {
		"FR_OK",
		"FR_DISK_ERR",
		"FR_INT_ERR",
		"FR_NOT_READY",
		"FR_NO_FILE",
		"FR_NO_PATH",
		"FR_INVALID_NAME",
		"FR_DENIED",
		"FR_EXIST",
		"FR_INVALID_OBJECT",
		"FR_WRITE_PROTECTED",
		"FR_INVALID_DRIVE",
		"FR_NOT_ENABLED",
		"FR_NO_FILESYSTEM",
		"FR_MKFS_ABORTED",
		"FR_TIMEOUT",
		"FR_LOCKED",
		"FR_NOT_ENOUGH_CORE",
		"FR_TOO_MANY_OPEN_FILES",
		"FR_INVALID_PARAMETER",
};

void fs_error_string(char* out_error_str, uint8_t len, int in_error_code){
    if(in_error_code > 19 - 1) return;
    strncpy(out_error_str, err_str[in_error_code], len);
}

uint8_t fs_scan_file(const char *in_path, FILINFO *out_file_info, uint32_t *out_file_num){

	FRESULT ret = 0;
	DIR _dir;
	FILINFO _fno;
    char *_path = NULL;

    if((in_path == NULL)){
    	_path = (char*)malloc(FF_LFN_BUF);
    	f_getcwd(_path, FF_LFN_BUF);
    }else{
    	_path = (char*)malloc(strlen(in_path) + 1);
    	strcpy(_path, in_path);
    }
	ret = f_opendir(&_dir, _path);
	if(ret == FR_OK){
		while(1){
			ret = f_readdir(&_dir, &_fno);
			if((ret != FR_OK) || _fno.fname[0] == 0) break;  // 发生错误或者目录索引结束
			if((_fno.fattrib & AM_DIR)){                 // 这是子目录
//				if(opt == 1){
//					if(scan_dir_path[strlen(scan_dir_path) - 1] != '/') strcat(scan_dir_path, "/");
//					strcat(scan_dir_path, _fno.fname);
//	                ret = fs_scan_file(scan_dir_path, out_file_info, out_file_num, opt);
//	                if(ret != FR_OK) break;
//
//				}
			}else{                                    // (_fno.fattrib & AM_ARC) 文件
                if(out_file_info != NULL){
                	memcpy(out_file_info, &_fno, sizeof(FILINFO));
                	out_file_info += 1;
                }
                if(out_file_num != NULL) (*out_file_num) += 1;
			}
		}
		f_closedir(&_dir);
	}
	free(_path);
	_path = NULL;
	return ret;
}

uint8_t fs_find_file(const char *in_path, const char* pattern, FILINFO *out_file_info, char opt){
	FRESULT ret = 0;
	DIR _dir;
	FILINFO _fno = {0};
	uint32_t _data = 0, _time = 0, _size = 0;
    char _path[FF_LFN_BUF] = {0};

    if((in_path == NULL)){
    	f_getcwd(_path, FF_LFN_BUF);
    }else{
    	strcpy(_path, in_path);
    }

    if(opt == 't'){
		_time = UINT32_MAX;
		_data = UINT32_MAX;
    }
    if(opt == 's'){
    	_size = UINT32_MAX;
    }
	ret = f_findfirst(&_dir, &_fno, _path, pattern);
	while((ret == FR_OK) && (_fno.fname[0])){
		if(_fno.fattrib & AM_DIR) goto find_next_section;
        switch(opt){
        case '*':
        	memcpy(out_file_info, &_fno, sizeof(FILINFO));
        	break;
        case 'T': // 时间排序(最新)
        	if(_fno.fdate > _data){
        	T_true_section:
        		_data = _fno.fdate;
        		_time = _fno.ftime;
        		memcpy(out_file_info, &_fno, sizeof(FILINFO));
        		goto find_next_section;
        	}
        	if((_fno.fdate == _data) && (_fno.ftime >= _time)){
                goto T_true_section;
        	}
        	break;
        case 't': // 时间排序(最早)
        	if(_fno.fdate < _data){
        	t_true_section:
        		_data = _fno.fdate;
        		_time = _fno.ftime;
        		memcpy(out_file_info, &_fno, sizeof(FILINFO));
        		goto find_next_section;
        	}
        	if((_fno.fdate == _data) && (_fno.ftime <= _time)){
                goto t_true_section;
        	}
        	break;
        case 'S': // 大小排序(最大)
			if(_fno.fsize > _size){
				_size = _fno.fsize;
				memcpy(out_file_info, &_fno, sizeof(FILINFO));
			}
        	break;
        case 's':  // (最小)
			if(_fno.fsize < _size){
				_size = _fno.fsize;
				memcpy(out_file_info, &_fno, sizeof(FILINFO));
			}
        	break;
        default:
        	goto return_section;
        	break;
        }

	find_next_section:
	    ret = f_findnext(&_dir, &_fno);
	}
return_section:
	ret += f_closedir(&_dir);
	return ret;
}

uint8_t fs_del_file(const char *in_path, const char* pattern, uint8_t opt){
	FRESULT ret = 0;
	FILINFO _fno = {0};
	char _path[FF_LFN_BUF] = {0};

	strcpy(_path, in_path);
    ret = fs_find_file(_path, pattern,  &_fno, opt);
    if((ret == FR_OK) && _fno.fname[0]){
    	strcat(_path, "/");
    	strcat(_path, _fno.fname);
    	ret = f_unlink(_path);
    }
    if((ret == FR_OK) && _fno.fname[0] == '\0') ret = FR_NO_FILE;

    return ret;
}

/**
 * 文件名可以包含绝对路径也可以只是文件名， 但是路径不支持相对路径
 */
uint8_t fs_create_file(FIL *in_file, const char *in_file_name){
	FRESULT ret = 0;
    int pos1 = -1;
    char dir[128];

    pos1 = find_chr(in_file_name, '/');
    if(pos1 > 0){  // 包含路径
    	pos1 = find_last_index_of_chr(in_file_name, '/');
        strncpy(dir, in_file_name, pos1);
        ret = fs_create_dir(dir);
        if(ret != FR_OK || ret != FR_EXIST) return ret;
        goto create_file_section;
    }else{
    	goto create_file_section;
    }
create_file_section:
    ret = f_open(in_file, in_file_name, FA_CREATE_ALWAYS | FA_WRITE);
    return ret;
}

uint8_t fs_create_dir(const char *path){  // path = 0:/dir1/dir2/dir3
    uint8_t ret = 0;
    int pos2 = 0, pos1 = 0;
    char dir_name[FF_MAX_LFN >> 1] = {'\0'};

    pos2 = find_chr(&path[pos2], '/');

    while(pos2 > 0){
        strncat(dir_name, &path[pos1], pos2);
        ret = f_mkdir(dir_name);
        if(ret != FR_OK && ret != FR_EXIST) return ret;
//        strcat(dir_name, "/");
        pos1 += pos2;
        pos2 = find_chr(&path[pos1 + 1], '/') + 1;
    }
    ret = f_mkdir(path);  // 最后一级目录

    return ret;
}

uint32_t fs_get_total_space(void){
	uint64_t fre_clust = 0;
	uint32_t tot_sect = 0;
	FATFS *_fs = NULL;
	f_getfree(sd_root_path, (DWORD*)&fre_clust, &_fs);
	tot_sect = (((_fs->n_fatent - 2) * _fs->csize) >> 1) >> 10;        // Unit: MB

	return tot_sect;
}

uint32_t fs_get_free_space(void){
	uint64_t fre_clust = 0;
	uint32_t fre_sect = 0;
	FATFS *_fs = NULL;
	f_getfree(sd_root_path, (DWORD*)&fre_clust, &_fs);

	fre_sect = ((fre_clust * _fs->csize) >> 1) >> 10;                  // Unit: MB

	return fre_sect;
}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
