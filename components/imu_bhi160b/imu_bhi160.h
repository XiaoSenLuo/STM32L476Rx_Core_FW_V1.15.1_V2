/*
 * imu_bhi160.h
 *
 *  Created on: 2021年1月19日
 *      Author: XIAOSENLUO
 */

#ifndef IMU_BHI160B_IMU_BHI160_H_
#define IMU_BHI160B_IMU_BHI160_H_

#include "bhy_support.h"
#include "bhy_uc_driver.h"

#define BHI160B
#define IMU_EXTI_LINE   1

void imu_bhy_int_callback(void);

void demo_sensor(void);

#endif /* IMU_BHI160B_IMU_BHI160_H_ */
