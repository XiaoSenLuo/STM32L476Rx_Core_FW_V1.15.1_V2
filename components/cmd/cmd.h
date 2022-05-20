/*
 * cmd.h
 *
 *  Created on: Jan 10, 2022
 *      Author: XIAO
 */

#ifndef CMD_CMD_H_
#define CMD_CMD_H_

#include "stdint.h"
#include "string.h"
#include "stdarg.h"
#include "stdbool.h"


void cmd_log_init(uint32_t baud);
uint8_t cmd_write(uint8_t* data, uint16_t len);
uint8_t cmd_printf(const char* fmt, ...);
uint8_t cmd_log_printf(const char* fmt, ...);


uint8_t cmd_scan(const char* in_data, char* out_cmd, char* out_val, char* out_data);
bool cmd_parse_connect(const char* sentence);
bool cmd_parse_get(const char* sentence);


void printf_help(void);


#endif /* CMD_CMD_H_ */
