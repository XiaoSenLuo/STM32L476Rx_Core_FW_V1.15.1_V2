/*
 * led_blink.h
 *
 *  Created on: Dec 5, 2020
 *      Author: XIAOSENLUO
 */

#ifndef LED_BLINK_LED_BLINK_H_
#define LED_BLINK_LED_BLINK_H_



#include "stdint.h"

//typedef struct _led_drv_t{
//	void (*led_on)(void);
//	void (*led_off)(void);
//}led_drv_t;
//
//
//typedef struct _led_blink_cfg_t{
//	float led_blink_freq;
//	uint8_t led_blink_times;
//	uint8_t led_next_blink_delay;
//	uint8_t led_blink_cnt;
//	uint8_t led_on_duty;
//}led_blink_cfg_t;


//void led_blink_drv_init(void* led_on_func, void* led_off_func);

//void led_blink(void);

//void led_set_blink(led_blink_cfg_t* led_cfg);

void led_blink_start(void);
void led_blink_stop(void);
void led_set_blink_times(uint8_t times);
void led_blink_on(void);
void led_blink_off(void);

#endif /* LED_BLINK_LED_BLINK_H_ */
