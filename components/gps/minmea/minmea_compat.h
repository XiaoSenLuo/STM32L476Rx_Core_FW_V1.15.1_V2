/*
 * Copyright © 2017 Kosma Moczek <kosma@cloudyourcar.com>
 * This program is free software. It comes without any warranty, to the extent
 * permitted by applicable law. You can redistribute it and/or modify it under
 * the terms of the Do What The Fuck You Want To Public License, Version 2, as
 * published by Sam Hocevar. See the COPYING file for more details.
 */


#ifndef _MINMEA_COMPAT_H_
#define _MINMEA_COMPAT_H_

//#define inline __inline

/**
 * 获取时间戳函数
 **/

#define timegm mktime


//time_t _mkgmtime(struct tm* timeptr){
//
//}



#endif

/* vim: set ts=4 sw=4 et: */
