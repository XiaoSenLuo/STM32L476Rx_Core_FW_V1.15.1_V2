/*
 * system_typedef.h
 *
 *  Created on: 2022年5月12日
 *      Author: XIAO
 */

#ifndef INC_SYSTEM_TYPEDEF_H_
#define INC_SYSTEM_TYPEDEF_H_

//#include "ads127.h"

#include "stdint.h"

typedef struct sys_config_s{
    struct{
    	uint32_t hse_value;
    	uint32_t core_freq;
    	uint32_t rco_div;
    	uint32_t uart_baudrate;
    	uint32_t cmd_connect_timeout;
    	uint32_t battery_voltage;
    	uint32_t battery_cutoff_voltage;
    	uint32_t battery_thold_voltage;
    	uint32_t battery_error_voltage;
    }system;
    struct{
        union{
        	struct{
        		uint32_t format : 1;
        		uint32_t file_system : 3;
        	};
        	uint32_t val;
        }fm;
        uint32_t file_limit;
    }sd;
    struct{
        struct{
        	uint32_t conver_rate;
        	uint32_t start_time;
        	int32_t work_time;
        	int32_t shutdown_time;
        	union{
        		struct{
        			uint32_t format : 1;
        		};
        		uint32_t val;
        	}file;
        }ctrl;
        uint32_t ofc;
        uint32_t fsc;
    }ads;
    struct{
        uint32_t baudrate;
    }gps;
    struct{

    }rtc;
}sys_config_t;


typedef union {
    struct {
        uint32_t usb2uart : 1;
        uint32_t gps_pps : 4;
        uint32_t sd_cd : 1;
        uint32_t usb_dec : 1;
    };
    uint32_t val;
}system_state_t;

#endif /* INC_SYSTEM_TYPEDEF_H_ */
