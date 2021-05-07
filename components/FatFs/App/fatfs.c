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

uint8_t retSD;    /* Return value for SD */
char sd_root_path[4];   /* SD logical drive path */



/* USER CODE BEGIN Variables */
#include "stm32l4xx_ll_rtc.h"
/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, sd_root_path);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
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

	ll_rtc_time.Hours = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC));
	ll_rtc_time.Minutes = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC));
	ll_rtc_time.Seconds = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC));
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
uint8_t FS_FileOperations(void)
{
  FRESULT res = FR_OK; /* FatFs function common result code */
  uint32_t byteswritten = 0; /* File write/read counts */

  char rtext[32]; /* File read buffer */
  DWORD fre_clust, fre_sect, tot_sect;
  FIL SDFile;       /* File object for SD */
  FATFS* fs;
  int8_t ret = 0;
  /* Register the file system object to the FatFs module */

//  uint32_t _fn = 0;
  DIR _dir = { 0 };
//                          fs_scan_file(NULL, NULL, &_fn);  // 查找文件数量
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
      if(res != FR_OK){
    	  return 1;
      }
      res = f_open(&SDFile, "sd info.txt", FA_CREATE_ALWAYS | FA_WRITE);
      if(res != FR_OK) return 1;
      tot_sect = (fs->n_fatent - 2) * fs->csize;
      fre_sect = fre_clust * fs->csize;
      memset(wtext, 0, sizeof(wtext));
      sprintf(wtext, "总容量: %.3f GiB\r\n剩余容量: %.3f GiB\r\n", ((double)tot_sect / 2.00f / 1024.00f / 1024.00f), ((double)fre_sect / 2.00f / 1024.00f / 1024.00f));
      f_write(&SDFile, wtext, (uint16_t)strlen(wtext), NULL);
      f_close(&SDFile);
      f_chdir(sd_root_path);  // 返回根目录
  }
  return (uint8_t)res;
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


uint8_t fs_del_file(const char *in_path, const char* pattern, uint8_t opt){
	FRESULT ret = 0;
	DIR _dir;
	FILINFO _fno = {0}, _fno_del = {0};
    char _path[FF_LFN_BUF] = {0};


    if((in_path == NULL)){
//    	_path = (char*)malloc(FF_LFN_BUF);
    	f_getcwd(_path, FF_LFN_BUF);
    }else{
//    	_path = (char*)malloc(strlen(in_path) + 1);
    	strcpy(_path, in_path);
    }
	ret = f_findfirst(&_dir, &_fno, _path, pattern);
	switch(opt){
	case DEL_FILE_BY_MODIFYTIME_UP: // 删除修改时间最早的文件
		_fno_del.ftime = UINT16_MAX;
		_fno_del.fdate = UINT16_MAX;
        while((ret == FR_OK) && (_fno.fname[0])){
           if((_fno.fdate + _fno.ftime) < (_fno_del.fdate + _fno_del.ftime)){
        	   _fno_del.fdate = _fno.fdate;
        	   _fno_del.ftime = _fno.ftime;
              memcpy(&_fno_del, &_fno, sizeof(FILINFO));
           }
           ret = f_findnext(&_dir, &_fno);
        }
        strcat(_path, _fno_del.fname);
        ret = f_unlink(_path);
		break;
	case DEL_FILE_BY_MODIFYTIME_DOWN:
		break;
	default:
		break;
	}
	f_closedir(&_dir);
//	free(_path);
//	_path = NULL;
	return ret;
}


uint8_t fs_create_file(FIL *in_file, const char *in_path, const char *in_file_name){
	FRESULT ret = 0;

    if((in_path == NULL)){
//    	_path = (char*)malloc(FF_LFN_BUF);
//    	f_getcwd(_path, FF_LFN_BUF);
//    	strcpy(_path, in_file_name);
    	ret = f_open(in_file, in_file_name, FA_CREATE_ALWAYS | FA_WRITE);
    	return ret;
    }else{
        char *_path = NULL;
    	_path = (char*)malloc(strlen(in_path) + strlen(in_file_name) + 2);
    	strcpy(_path, in_path);
    	if(_path[strlen(_path) - 1] != '/') strcat(_path, "/");
    	strcat(_path, in_file_name);
    	ret = f_open(in_file, _path, FA_CREATE_ALWAYS | FA_WRITE);
    	free(_path);
    	_path = NULL;
    }
    return ret;
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
