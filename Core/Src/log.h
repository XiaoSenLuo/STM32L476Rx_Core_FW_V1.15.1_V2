//
// Created by XIAO on 2022/5/23.
//

#ifndef LOG_H
#define LOG_H

#include "stdarg.h"


int log_file_create(const char *dir, void* *file);
int log_file_close(void);

int log_printf(const char *fmt, ...);

//int log_flush(void);

#endif //LOG_H
