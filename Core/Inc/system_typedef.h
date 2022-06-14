/*
 * system_typedef.h
 *
 *  Created on: 2022年5月12日
 *      Author: XIAO
 */

#ifndef SYSTEM_TYPEDEF_H_
#define SYSTEM_TYPEDEF_H_

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    eTYPE_ERROR = -1,
    eTYPE_NOEVENT = 0,
    eTYPE_SELF,
    eTYPE_USB,
    eTYPE_CMD,
    eTYPE_GPS,
    eTYPE_ANALOG,
    eTYPE_SD,
}system_event_type_t;

typedef enum {
    NOEVENT = 0,
    EVENT_UART2USB_CONNECT,
    EVENT_UART2USB_DISCONNECT,
    EVENT_UART2USB_DATA_READY,
    EVENT_GPS_PPS,
    EVENT_ANALOG_STOP,
    EVENT_ANALOG_BUFFER_FULL,
    EVENT_SD_CONNECT,
    EVENT_SD_DISCONNECT,
}system_event_def_t;


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
        uint32_t file_size_limit;
    }sd;
    struct{
        uint32_t clk;
        uint32_t conver_rate __attribute__((deprecated));
        uint32_t start_time;
        uint32_t work_time;
        uint32_t shutdown_time;
        union{
            struct{
                uint32_t format : 1 __attribute__((deprecated));     // 0:bin, 1:txt
                uint32_t limit_type : 1;  // 0:time, 1:size
                uint32_t limit : 16;      // unit:秒或者KB
            };
            uint32_t val;
        }file;
        uint32_t offset_calibration;
        uint32_t gain_calibration;
    }ads;
    struct{
        uint32_t baudrate;
    }gps;
    struct{
        union {
            struct{
                uint32_t enable : 1;
                uint32_t output_freq : 1;
            };
            uint32_t val;
        }calibration;
    }rtc;
}system_config_t;

typedef union u32_cmd_format_u{
    struct{
        uint32_t bit0_16 : 16;
        uint32_t cmd : 8;
        uint32_t cmd_type : 8;
    };
    uint32_t val;
}u32_cmd_format_t;
typedef union u32_cmd_format_u * u32_cmd_format_handle;

struct u80_cmd_frame_head_info_s{
    uint32_t frame_head;
    uint8_t id;
    uint32_t cmd;   /// \brief u32_cmd_format_t
    uint8_t length;
};
typedef struct u80_cmd_frame_head_info_s * cmd_frame_head_info_handle;

typedef struct cmd_data_frame_s {
    struct u80_cmd_frame_head_info_s head;
    void *data;
}cmd_data_frame_t;
typedef struct cmd_data_frame_s * cmd_data_frame_handle;

#ifdef __cplusplus
}
#endif
#endif /* SYSTEM_TYPEDEF_H_ */
