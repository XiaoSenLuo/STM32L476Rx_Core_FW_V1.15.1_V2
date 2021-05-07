/*
 * st_spiffs_drv.c
 *
 *  Created on: 2020年11月24日
 *      Author: XIAOSENLUO
 */


#include <st_spiffs_drv.h>
#include "main.h"


#define PHYS_FLASH_SIZE       (1*1024*1024) // 1MB
// default test spiffs file system size
#define SPIFFS_FLASH_SIZE     (32*1024)      // 32KB
// default test spiffs file system offset in emulated spi flash
#define SPIFFS_PHYS_ADDR      (4*1024*1024)
// default test sector size
//#define SECTOR_SIZE         65536  // 64KB
// default test logical block size
#define LOG_BLOCK_SIZE           (16*1024) // 16KB
// default test logical page size
#define LOG_PAGE_SIZE            (2048) // 2KB
// default test number of filedescs
#define DEFAULT_NUM_FD            16
// default test number of cache pages
#define DEFAULT_NUM_CACHE_PAGES   8

spiffs st_spiffs_fs;

static uint8_t st_spiffs_work_buf[LOG_PAGE_SIZE * 2];
static uint8_t st_spiffs_fds[32 * 4];
static uint8_t st_spiffs_cache_buf[(LOG_PAGE_SIZE + 32) * 4];

s32_t st_flash_read(u32_t addr, u32_t cnt, u8_t *dst);
s32_t st_flash_write(u32_t addr, u32_t cnt, u8_t *src);
s32_t st_flash_erase(u32_t addr, u32_t cnt);

static uint32_t st_get_page_address(uint32_t addr){
	uint32_t page_addr = 0;
	if(addr < (FLASH_BASE + FLASH_BANK_SIZE)){
		page_addr = (addr - FLASH_BASE) / FLASH_PAGE_SIZE;
	}else{
		page_addr = (addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
	}
	return page_addr;
}

static uint32_t st_get_bank_index(uint32_t addr){
    uint32_t bank_index = 0;

    /* No Bank swap */
    if(READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0){
    	if(addr < (FLASH_BASE + FLASH_BANK_SIZE)){
    		bank_index = FLASH_BANK_1;
    	}else{
    		bank_index = FLASH_BANK_2;
    	}
    }else{   /* Bank swap */
    	if(addr < (FLASH_BASE + FLASH_BANK_SIZE)){
    		bank_index = FLASH_BANK_2;
    	}else{
    		bank_index = FLASH_BANK_1;
    	}
    }
    return bank_index;
}


s32_t st_flash_read(u32_t addr, u32_t cnt, u8_t *dst){

    uint32_t tmp_data = 0;
    uint32_t cnt_index = 0;
    uint8_t cnt_i = 0;

    if(addr < st_spiffs_fs.cfg.phys_addr) return SPIFFS_ERR_IX_MAP_BAD_RANGE;
    if((addr + cnt) > (st_spiffs_fs.cfg.phys_addr + st_spiffs_fs.cfg.phys_size)) return SPIFFS_ERR_IX_MAP_BAD_RANGE;

    do{
    	cnt_i = cnt_index % 4;
    	if(cnt_i == 0){
    		tmp_data = *(__IO uint32_t *)addr;
    		dst[cnt_index] = (uint8_t)tmp_data;
    		addr += 4;
    	}else{
    		dst[cnt_index] = (uint8_t)(tmp_data >> (cnt_i << 3));
    	}
    	cnt_index += 1;
    }while(cnt_index != cnt);
    return SPIFFS_OK;
}

s32_t st_flash_write(u32_t addr, u32_t cnt, u8_t *src){
	uint32_t res = HAL_OK;
	uint64_t tmp_data = 0;
	uint32_t cnt_index = 0;
	uint8_t  cnt_i = 0;

	if(addr > (st_spiffs_fs.cfg.phys_addr + st_spiffs_fs.cfg.phys_size)
			|| (addr < st_spiffs_fs.cfg.phys_addr)){
		return SPIFFS_ERR_IX_MAP_BAD_RANGE;
	}

	res = HAL_FLASH_Unlock();
	if(res != HAL_OK){
		return SPIFFS_ERR_INTERNAL;
	}
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	do{
		cnt_i = cnt_index % 8;
		if((cnt_i == 7)){
			tmp_data |= ((uint64_t)src[cnt_index]) << (cnt_i << 3);
			res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, tmp_data);
			addr += 8;
			tmp_data = 0;
		}else{
            tmp_data |= ((uint64_t)src[cnt_index]) << (cnt_i << 3);
            if(cnt_index == (cnt - 1)){
            	res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, tmp_data);
            	addr += (cnt_index % 8);
            }
		}
		cnt_index += 1;
	}while((cnt_index != cnt) && (addr < (st_spiffs_fs.cfg.phys_addr + st_spiffs_fs.cfg.phys_size)));
	/* Process Unlocked */
	HAL_FLASH_Lock();
    if(res == HAL_OK){
    	return SPIFFS_OK;
    }else{
    	return SPIFFS_ERR_INTERNAL;
    }
}

s32_t st_flash_erase(u32_t addr, u32_t cnt){

	uint32_t first_page = 0;
	uint32_t bank_index = 0;
    uint32_t page_number = 0;
    uint32_t res = HAL_OK;
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PAGEError = 0;

    first_page = st_get_page_address(addr);
    page_number = st_get_page_address(addr + cnt) - first_page + 1;
    bank_index = st_get_bank_index(addr);

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks = bank_index;
    EraseInitStruct.Page = first_page;
    EraseInitStruct.NbPages = page_number;

	HAL_FLASH_Unlock();
	/* Clear OPTVERR bit set on virgin samples */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    res = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
    if(res != HAL_OK){
    	PAGEError = PAGEError;
    	SPIFFS_DBG("Erase %dth Page Fail!\r\n", PAGEError);
    	return SPIFFS_ERR_ERASE_FAIL;
    }
    HAL_FLASH_Lock();
    return SPIFFS_OK;
}



void    st_spiffs_init(spiffs* fs_t){
    spiffs_config cfg;
    int32_t res = SPIFFS_OK;

    cfg.log_block_size = 4096;  // 4KB
    cfg.log_page_size = 2048;   // 2KB
    cfg.phys_addr = ADDR_FLASH_PAGE_496;
    cfg.phys_size = 16 * 2048; // 32KB
    cfg.phys_erase_block = 2048;
    cfg.hal_read_f = st_flash_read;
    cfg.hal_write_f = st_flash_write;
    cfg.hal_erase_f = st_flash_erase;

    res = SPIFFS_mount(&st_spiffs_fs,
    		&cfg,
			st_spiffs_work_buf,
			st_spiffs_fds,
			sizeof(st_spiffs_fds),
			st_spiffs_cache_buf,
			sizeof(st_spiffs_cache_buf),
			NULL);
    if(res != SPIFFS_OK){

    }
    SPIFFS_unmount(&st_spiffs_fs);

    res = SPIFFS_format(&st_spiffs_fs);
    if(res != 0){

    }
    res = SPIFFS_mount(&st_spiffs_fs,
    		&cfg,
			st_spiffs_work_buf,
			st_spiffs_fds,
			sizeof(st_spiffs_fds),
			st_spiffs_cache_buf,
			sizeof(st_spiffs_cache_buf),
			NULL);
    if(res != 0){

    }
    if(fs_t != NULL) fs_t = &st_spiffs_fs;
    else fs_t = fs_t;
}
