/*
 * st_test_flash.c
 *
 *  Created on: 2020年11月30日
 *      Author: XIAOSENLUO
 */


#include "st_spiffs_api.h"

extern spiffs st_spiffs_fs;

void st_spiffs_test(void){
    int32_t res = SPIFFS_OK;
    spiffs* test_fs = NULL;
    uint8_t test_write_char[] = "test stm32l476rg";
    uint8_t test_read_char[32] = {0};
    int32_t len = sizeof(test_write_char);
    test_fs = &st_spiffs_fs;
    spiffs_file test_fh;

    test_fh = SPIFFS_open(test_fs, "test.txt", SPIFFS_CREAT | SPIFFS_TRUNC | SPIFFS_RDWR, 0);
    if(test_fh < 0){
        return;
    }
    res = SPIFFS_write(test_fs, test_fh, (uint8_t*)test_write_char, len);
    if(res < 0){
    	return;
    }
    res = SPIFFS_close(test_fs, test_fh);
    if(res < 0){
    	return;
    }
    test_fh = SPIFFS_open(test_fs, "test.txt", SPIFFS_RDWR, 0);
    if(test_fh < 0){
    	return;
    }
    len = sizeof(test_read_char);
    res = SPIFFS_read(test_fs, test_fh, test_read_char, len);
    if(res < 0){
    	return;
    }
    res = SPIFFS_close(test_fs, test_fh);
    if(res < 0){
    	return;
    }
}


