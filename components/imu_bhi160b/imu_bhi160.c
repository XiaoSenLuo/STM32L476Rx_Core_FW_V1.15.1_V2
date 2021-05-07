/*
 * imu_bhi160.c
 *
 *  Created on: 2021年3月25日
 *      Author: XIAOSENLUO
 */

#include "imu_bhi160.h"
#include "stdint.h"
#include "stdio.h"

#include "main.h"

#if(defined(BHI160B))
// #include "firmware\Bosch_PCB_7183_di03_BMI160-7183_di03.2.1.11696_170103.h"
#include "firmware\bosch_pcb_7183_di03_bmi160-7183_di03-2-1-11696_20180502.h"
#elif(defined(BHI160A))
#include "firmware\Bosch_PCB_7183_di01_BMI160-7183_di01.2.1.10836_170103.h"
#elif(defined(BHI160B_BMM150))
#include "firmware\Bosch_PCB_7183_di03_BMI160_BMM150-7183_di03.2.1.11696_170103.h"
#elif(defined(BHI160A_BMP280))
#include "firmware\Bosch_PCB_7183_di01_BMI160_BMP280-7183_di01.2.1.10836.h"
#else
#error "please include a hearer file!"
#endif


/* should be greater or equal to 69 bytes, page size (50) + maximum packet size(18) + 1 */
#define FIFO_SIZE                      300
#define MAX_PACKET_LENGTH              18
#define TICKS_IN_ONE_SECOND            32000.0F


#define debug_log     printf
#define BHY_INT       (EXTI_IMU1_Pin)


static __IO uint8_t imu_int_io_level = 0;

//#define ioport_get_pin_level(bhy_int) LL_GPIO_ReadInputPort(EXTI_IMU1_GPIO_Port & (bhy_int))

static uint8_t ioport_get_pin_level(uint32_t in_bhy_int){
	in_bhy_int = in_bhy_int;

#if(IMU_EXTI_LINE)
//	return (LL_GPIO_ReadInputPort(EXTI_IMU1_GPIO_Port) & EXTI_IMU1_Pin);
	return imu_int_io_level;
#else
	uint8_t int_status[8] = {0};

    bhy_get_interrupt_status(&int_status[0],
    		&int_status[1], &int_status[2], &int_status[3],
			&int_status[4], &int_status[5], &int_status[6]);
    return int_status[0];
#endif
}

void imu_bhy_int_callback(void){
    if(LL_GPIO_ReadInputPort(EXTI_IMU1_GPIO_Port) & EXTI_IMU1_Pin){   // HIGH
    	imu_int_io_level = 1;
    }
    if(!(LL_GPIO_ReadInputPort(EXTI_IMU1_GPIO_Port) & EXTI_IMU1_Pin)){  // LOW
    	imu_int_io_level = 0;
    }
}

/* system timestamp */
uint32_t bhy_timestamp = 0;

uint8_t fifo[FIFO_SIZE];

/*!
 * @brief This function is time stamp callback function that process data in fifo.
 *
 * @param[in]   new_timestamp
 */
static void timestamp_callback(bhy_data_scalar_u16_t *new_timestamp)
{
    /* updates the system timestamp */
    bhy_update_system_timestamp(new_timestamp, &bhy_timestamp);
}

/*!
 * @brief This function is  callback function for acquring sensor datas
 *
 * @param[in]   sensor_data
 * @param[in]   sensor_id
 */
static void sensors_callback_acc(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
    float   time_stamp    = 0;
    uint8_t sensor_type   = 0;
    int16_t x_raw         = 0;
    int16_t y_raw         = 0;
    int16_t z_raw         = 0;
    float   x_data        = 0;
    float   y_data        = 0;
    float   z_data        = 0;

    /* Since a timestamp is always sent before every new data, and that the callbacks   */
    /* are called while the parsing is done, then the system timestamp is always equal  */
    /* to the sample timestamp. (in callback mode only)                                 */
    time_stamp = (float)(bhy_timestamp) / TICKS_IN_ONE_SECOND;

//    debug_log("sensor_id = %d\n", sensor_id);

    switch(sensor_id)
    {
        case VS_ID_ACCELEROMETER:
        case VS_ID_ACCELEROMETER_WAKEUP:
            x_raw  = sensor_data->data_vector.x;
            y_raw  = sensor_data->data_vector.y;
            z_raw  = sensor_data->data_vector.z;
            /* The resolution is  15bit ,the default range is 4g, actual acceleration equals: raw_data/(exp(2,15) == 32768) */
            x_data = (float)x_raw / 32768.0f * 4.0f;
            y_data = (float)y_raw / 32768.0f * 4.0f;
            z_data = (float)z_raw / 32768.0f * 4.0f;

//            debug_log("Time:%6.3fs acc %f %f %f\n", time_stamp, x_data, y_data, z_data);
            break;

        default:
//            debug_log("unknown id = %d\n", sensor_id);
            break;
    }
}


/*!
 * @brief This function is used to run bhy hub
 */
void demo_sensor(void)
{
    int8_t ret;

    /* BHY Variable*/
    uint8_t                    *fifoptr           = NULL;
    uint8_t                    bytes_left_in_fifo = 0;
    uint16_t                   bytes_remaining    = 0;
    uint16_t                   bytes_read         = 0;
    bhy_data_generic_t         fifo_packet;
    bhy_data_type_t            packet_type;
    BHY_RETURN_FUNCTION_TYPE   result;
    int8_t                    bhy_mapping_matrix_init[3*3]   = {0};
    int8_t                    bhy_mapping_matrix_config[3*3] = {0,1,0,-1,0,0,0,0,1};

    /* To get the customized version number in firmware, it is necessary to read Parameter Page 2, index 125 */
    /* to get this information. This feature is only supported for customized firmware. To get this customized */
    /* firmware, you need to contact your local FAE of Bosch Sensortec. */
    //struct cus_version_t      bhy_cus_version;

//    uint8_t ioport_level = 0;
//    ioport_level = ioport_get_pin_level(0);

    /* init the bhy chip */
    result = bhy_driver_init((const uint8_t*)&bhy_firmware_image);
    if(result)
    {
//        debug_log("Fail to init bhy\n");
    }

    /* wait for the bhy trigger the interrupt pin go down and up again */
    while (ioport_get_pin_level(BHY_INT));
//    ioport_level = ioport_get_pin_level(BHY_INT);
    while (!ioport_get_pin_level(BHY_INT));

    uint16_t ram_ver = 0;
    uint8_t int_status[8] = {0}, chip_status[8] = {0};
    struct accel_physical_status_t acc_status;
    struct sensor_information_wakeup_t info_wakeup;
    struct gyro_physical_status_t gyr_status;

//    bhy_delay_msec(100);
//    bhy_get_ram_version(&ram_ver);
//    while(!ram_ver);
//    bhy_get_interrupt_status(&int_status[0],
//    		&int_status[1], &int_status[2], &int_status[3],
//			&int_status[4], &int_status[5], &int_status[6]);
//    bhy_get_chip_status(&chip_status[0],
//    		&chip_status[1], &chip_status[2], &chip_status[3], &chip_status[4]);

    /* To get the customized version number in firmware, it is necessary to read Parameter Page 2, index 125 */
    /* to get this information. This feature is only supported for customized firmware. To get this customized */
    /* firmware, you need to contact your local FAE of Bosch Sensortec. */
    //bhy_read_parameter_page(BHY_PAGE_2, PAGE2_CUS_FIRMWARE_VERSION, (uint8_t*)&bhy_cus_version, sizeof(struct cus_version_t));
    //debug_log("cus version base:%d major:%d minor:%d\n", bhy_cus_version.base, bhy_cus_version.major, bhy_cus_version.minor);

    /* config mapping matrix, for customer platform, this remapping matrix need to be changed */
    /* according to 'Application Note Axes remapping of BHA250(B) /BHI160(B)' document.       */
    bhy_mapping_matrix_get(PHYSICAL_SENSOR_INDEX_ACC, bhy_mapping_matrix_init);
    bhy_mapping_matrix_set(PHYSICAL_SENSOR_INDEX_ACC, bhy_mapping_matrix_config);
    bhy_mapping_matrix_get(PHYSICAL_SENSOR_INDEX_ACC, bhy_mapping_matrix_init);

    /* install time stamp callback */
    bhy_install_timestamp_callback(VS_WAKEUP, timestamp_callback);
    bhy_install_timestamp_callback(VS_NON_WAKEUP, timestamp_callback);

//    bhy_set_meta_event();

    /* install the callback function for parse fifo data */
    result = bhy_install_sensor_callback(VS_TYPE_ACCELEROMETER, VS_WAKEUP, sensors_callback_acc);
    if(result)
    {
//        debug_log("Fail to install sensor callback\n");
    }

    /* enables the virtual sensor */
    result = bhy_enable_virtual_sensor(VS_TYPE_ACCELEROMETER, VS_WAKEUP, 10, 10, VS_FLUSH_SINGLE, 0, 0);
    if(result)
    {
//        debug_log("Fail to enable sensor id=%d\n", VS_TYPE_ACCELEROMETER);
    }
    bhy_delay_msec(1000);
	bhy_get_chip_status(&chip_status[0],
			&chip_status[1], &chip_status[2], &chip_status[3], &chip_status[4]);
	bhy_get_physical_sensor_status(&acc_status, &gyr_status, NULL);
	bhy_get_wakeup_sensor_information(BHY_PARAMETER_REQUEST_READ_PARAMETER_33, &info_wakeup);

	bhy_get_interrupt_status(&int_status[0],
			&int_status[1], &int_status[2], &int_status[3],
			&int_status[4], &int_status[5], &int_status[6]);

    /* continuously read and parse the fifo */
    while(1)
    {
        /* wait until the interrupt fires */
        /* unless we already know there are bytes remaining in the fifo */

		bhy_get_chip_status(&chip_status[0],
				&chip_status[1], &chip_status[2], &chip_status[3], &chip_status[4]);
		bhy_get_physical_sensor_status(&acc_status, &gyr_status, NULL);
		bhy_get_wakeup_sensor_information(BHY_PARAMETER_REQUEST_READ_PARAMETER_33, &info_wakeup);

        while (!ioport_get_pin_level(BHY_INT) && !bytes_remaining)
        {
    		bhy_get_interrupt_status(&int_status[0],
    				&int_status[1], &int_status[2], &int_status[3],
    				&int_status[4], &int_status[5], &int_status[6]);
        	bhy_delay_msec(50);
        }

        result = bhy_read_fifo(fifo + bytes_left_in_fifo, FIFO_SIZE - bytes_left_in_fifo, &bytes_read, &bytes_remaining);
        bytes_read           += bytes_left_in_fifo;
        fifoptr              = fifo;
        packet_type          = BHY_DATA_TYPE_PADDING;

        do
        {
            /* this function will call callbacks that are registered */
            result = bhy_parse_next_fifo_packet(&fifoptr, &bytes_read, &fifo_packet, &packet_type);

            /* prints all the debug packets */
//            if (packet_type == BHY_DATA_TYPE_DEBUG)
//            {
//                bhy_print_debug_packet(&fifo_packet.data_debug, bhy_printf);
//            }

            /* the logic here is that if doing a partial parsing of the fifo, then we should not parse  */
            /* the last 18 bytes (max length of a packet) so that we don't try to parse an incomplete   */
            /* packet */
        } while ((result == BHY_SUCCESS) && (bytes_read > (bytes_remaining ? MAX_PACKET_LENGTH : 0)));

        bytes_left_in_fifo = 0;

        if (bytes_remaining)
        {
            /* shifts the remaining bytes to the beginning of the buffer */
            while (bytes_left_in_fifo < bytes_read)
            {
                fifo[bytes_left_in_fifo++] = *(fifoptr++);
            }
        }
    }
}

/** @}*/




