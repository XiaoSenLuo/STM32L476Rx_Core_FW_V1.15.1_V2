/*
 * config_ini.h
 *
 *  Created on: 2020年12月11日
 *      Author: XIAOSENLUO
 */

#ifndef CONFIG_INI_CONFIG_INI_H_
#define CONFIG_INI_CONFIG_INI_H_

#include "stdint.h"
#include "string.h"
#include "stdlib.h"


/**
 *  初始化系统变量配置, 并返回配置变量缓冲区, 变量顺序与配置名称保存一直
 * @return
 */
uint32_t *sys_config_init(void);
void sys_config_deinit(uint32_t *in_sys_config);


/***
 * in_data: 以 "\r\n" 结尾
 * in_section: 以 "\r\b" 结尾, 包含 "[]"
 */

uint8_t is_config_section(uint8_t* in_data);

/**
 *
 * @return 返回配置节个数
 */
size_t get_section_size(void);

/**
 *
 * @param in_index 配置节索引
 * @return 配置节
 */
char* get_section_by_index(uint8_t in_index);

/**
 * function: 检查 in_data 是否与  in_section 匹配, 如果 out_section 不为空, 则返回 section
 *           同时检查语句是否符合 section 语法
 */
uint8_t check_config_section(uint8_t* in_data, const char* in_section, uint8_t* out_section);

/**
 *
 * @param in_section 配置节
 * @param out_section_size 返回键表大小
 * @return 返回配置节键表
 */
char** get_keyname_by_section(char* in_section, uint8_t* out_keyname_size);

/**
 * opt: 'i'-整型
 *      'c'-字符
 *      'f'-浮点数
 *      'd'-双精度浮点型
 */
uint8_t get_config_value_by_keyname(uint8_t* in_data, const char* keyname, void* out_value, char opt);

size_t get_default_config_lines(void);
char** get_default_config_file(void);

uint8_t create_default_config_file(const char *file_name);


#endif /* CONFIG_INI_CONFIG_INI_H_ */
