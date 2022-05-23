//
// Created by XIAO on 2022/5/23.
//

#ifndef STM32L476RX_LOG_H
#define STM32L476RX_LOG_H

#include "stdarg.h"


int log_file_create(const char *dir, void* *file);
int log_file_close(void);

int log_printf(const char *fmt, ...);

#endif //STM32L476RX_LOG_H
