//
// Created by XIAOSENLUO on 2022/6/3.
//

#ifndef BSP_INI_H
#define BSP_INI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "system_typedef.h"


int parse_ini_file_from_sd(const char* _root_path, void* user);
int save_ini_file_to_sd(const char* _root_path, void* user);



#ifdef __cplusplus
}
#endif
#endif //BSP_INI_H
