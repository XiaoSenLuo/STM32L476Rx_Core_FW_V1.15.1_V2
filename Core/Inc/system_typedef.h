/*
 * system_typedef.h
 *
 *  Created on: 2022年5月12日
 *      Author: XIAO
 */

#ifndef INC_SYSTEM_TYPEDEF_H_
#define INC_SYSTEM_TYPEDEF_H_

#include "stdint.h"


typedef enum {
    eTYPE_NOEVENT = 0,
    eTYPE_USB = 1,
    eTYPE_GPS,
    eTYPE_ANALOG,
    eTYPE_SD,
}system_event_type_t;

typedef enum {
    NOEVENT = 0,
    EVENT_UART2USB_CONNECT = 1,
    EVENT_UART2USB_DISCONNECT,
    EVENT_UART2USB_DATA_READY,
    EVENT_GPS_PPS,
    EVENT_ANALOG_STOP,
    EVENT_ANALOG_BUFFER_FULL,
    EVENT_SD_CONNECT,
    EVENT_SD_DISCONNECT,
}system_event_def_t;

typedef struct system_event_s{
    system_event_type_t type;
    system_event_def_t event;
    void *event_data;
}system_event_t;

typedef struct system_event_s * system_event_handle;

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
        			uint32_t format : 1;     // 0:bin, 1:txt
                    int32_t limit_type : 1;  // 0:time, 1:size
                    uint32_t limit : 8;      // unit:分钟或者MB
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
}system_config_t;

typedef union u32_cmd_format_u{
    struct{
        uint32_t bit0_16 : 16;
        uint32_t cmd : 8;
        uint32_t cmd_type : 8;
    };
    uint32_t val;
};
typedef union u32_cmd_format_u * u32_cmd_format_handle;

struct u80_cmd_frame_head_info_s{
    uint32_t frame_head;
    uint8_t id;
    uint32_t cmd;   /// \brief u32_cmd_format_t
    uint8_t length;
};
typedef struct u80_cmd_frame_head_info_s * cmd_frame_head_info_handle;

struct cmd_data_frame_s {
    struct u80_cmd_frame_head_info_s head;
    void *data;
}cmd_data_frame_t;
typedef struct cmd_data_frame_s * cmd_data_frame_handle;


#endif /* INC_SYSTEM_TYPEDEF_H_ */
