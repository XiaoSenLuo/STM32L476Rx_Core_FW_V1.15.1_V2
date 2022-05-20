/*
 * utils.h
 *
 *  Created on: 2021年6月9日
 *      Author: XIAOSENLUO
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "time.h"

int find_chr(const char* str, const char c);
int find_last_index_of_chr(const char* str, const char c);

int replace_last_index_of_chr(char* str, const char c1, const char c2);

void string_format(char* out, const char* format, ...);

void format_date_to_string(char *out_str, uint8_t len, struct tm _tm);

void format_time_to_string(char *out_str, uint8_t len, struct tm _tm);

uint32_t data_to_str(uint8_t* in_data, uint32_t in_size, char* out_str, uint32_t in_str_len);


void add_time(struct tm* tt, const int add_second);

#endif /* INC_UTILS_H_ */
