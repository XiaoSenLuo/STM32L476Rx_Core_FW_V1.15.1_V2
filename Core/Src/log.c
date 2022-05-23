//
// Created by XIAO on 2022/5/23.
//

#include "log.h"
#include "ff.h"
#include "fatfs.h"
#include "usart.h"
#include "rtc.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#ifndef LOG_BUFFER_SIZE
#define LOG_BUFFER_SIZE                   512
#endif

static const char log_default_dir[] = "0:/log";
static char root[4] = {'0', ':', '/', '\0'};
static FIL log_file = { 0 };
static char *log_buffer = NULL;

int log_file_create(const char *dir, void* *file){
    int err = 0;
    rtc_date_time_t time = { 0 };
    char path[64] = {'\0'};
    size_t ll = 0;
    if(dir == NULL){
        err = fs_create_dir(log_default_dir);
        strcat(path, log_default_dir);
    }else{
        err = fs_create_dir(dir);
        strcat(path, dir);
    }
    if((err != FR_OK) && (err != FR_EXIST)) goto end_section;

    ll = strlen(path);
    if(path[ll - 1] != '/'){
        strcat(path, "/");
        ll += 1;
    }
    strcat(path, "log-");
    ll += 4;
    st_rtc_get_time_v2(&time);
    ll += rtc_time2str(&time, &path[ll], 64 - ll);
    strcat(path, ".txt");

    if(log_file.obj.fs){
        f_close(&log_file);
    }
    err = f_open(&log_file, path, FA_WRITE | FA_CREATE_ALWAYS);
    if((err != FR_OK) && (err != FR_EXIST)) goto end_section;
    if(log_buffer == NULL) log_buffer = (char *)malloc(LOG_BUFFER_SIZE);
    if(file) *file = &log_file;
    end_section:
    return err;
}


int log_file_close(void){
    int err = 0;
    if(log_file.obj.fs){
        err = f_close(&log_file);
    }
    if(err != FR_OK) goto end_section;

    end_section:
    if(log_buffer){
        free(log_buffer);
        log_buffer = NULL;
    }
    return err;
}


int log_printf(const char *fmt, ...){
    int err = 0;
    rtc_date_time_t time = { 0 };
    size_t ll = 0;
    va_list arp;

    if(log_file.obj.fs == NULL){
        rtc_date_time_t time = { 0 };
        char path[32] = {'\0'};
        err = log_file_create(log_default_dir, NULL);
        if((err != FR_OK) && (err != FR_EXIST)) goto end_section;
    }
    log_buffer[0] = '[';
    ll += 1;
    st_rtc_get_time_v2(&time);
    ll = rtc_time2str(&time, &log_buffer[ll], LOG_BUFFER_SIZE);
    strcat(&log_buffer[ll], "]: ");
    ll += 2;
    va_start(arp, fmt);
    ll += vsniprintf(&log_buffer[ll], LOG_BUFFER_SIZE - ll, fmt, arp);
    va_end(arp);

    err = f_write(&log_file, log_buffer, ll + 1, NULL);
    if(!err) err = f_sync(&log_file);

    end_section:
    return err;
}


