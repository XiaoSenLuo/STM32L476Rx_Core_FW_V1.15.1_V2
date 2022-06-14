//
// Created by XIAOSENLUO on 2022/5/25.
//

#ifndef BSP_GPS_H
#define BSP_GPS_H



#include "stm32l4xx_it.h"
#include "gpio.h"
#include "minmea.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gps_nmea_parse_zda_s{
    struct minmea_sentence_zda zda;
    char sentence[48];
    char char_num;
    char parse;
}gps_nmea_parse_zda_t;

void gps_pps_isr_install(GPIO_TypeDef * ppsPort, int32_t ppsPin, isr_function_handle_t fn, void * ctx);

void gps_pps_isr_uninstall(void);

void gps_pps_irq_disable(void);

void gps_pps_irq_enable(void);

typedef struct gps_ctrl_output{
    uint8_t nGGA;       /** GGA 输出频率, 语句输出频率是以定位更新率为基准的, n(0~9)表示每 n次定位输出一次, 0 表示不输出该语句, 空则保持原有配置。 **/
    uint8_t nGLL;       /** GLL 输出频率,  同 nGGA **/
    uint8_t nGSA;       /** GSA 输出频率,  同 nGGA **/
    uint8_t nGSV;       /** GSV 输出频率,  同 nGGA **/
    uint8_t nRMC;
    uint8_t nVTG;
    uint8_t nZDA;
    uint8_t nANT;
    uint8_t nDHV;
    uint8_t nLPS;
    uint8_t res1;
    uint8_t res2;
    uint8_t nUTC;
    uint8_t nGST;
    uint8_t res3;
    uint8_t res4;
    uint8_t res5;
    uint8_t nTIM;
}gps_ctrl_output_t;

void gps_config_output(const gps_ctrl_output_t * out);

#ifdef __cplusplus
}
#endif

#endif //BSP_GPS_H
