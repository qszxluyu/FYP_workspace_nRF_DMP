/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "app_uart.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "app_mpu.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "boards.h"
#include "nrf_drv_mpu.h"

//#include "math.h"

#include "dmpkey.h"
#include "dmpmap.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "eMPL_outputs.h"
#include "fusion_9axis.h"
#include "quaternion_supervisor.h"
#include "ml_math_func.h"
#include "data_builder.h"
//#include "fast_no_motion.h"
#include "mpl.h"

#include "nrf_drv_inv_dmp.h"



//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 1024                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

#define MPU_ADDRESS     			0x68 
#define MPU_AK89XX_MAGN_ADDRESS     0x0C

#define DEFAULT_MPU_HZ  (20)


// General application timer settings.
APP_TIMER_DEF(timer1);

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

struct platform_data_s {
		signed char orientation[9];
};

static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};

struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};
static struct hal_s hal = {0};

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}



#ifdef ENABLE_LOOPBACK_TEST
/** @brief Function for setting the @ref ERROR_PIN high, and then enter an infinite loop.
 */
static void show_error(void)
{

    bsp_board_leds_on();
    while (true)
    {
        // Do nothing.
    }
}


/** @brief Function for testing UART loop back.
 *  @details Transmitts one character at a time to check if the data received from the loopback is same as the transmitted data.
 *  @note  @ref TX_PIN_NUMBER must be connected to @ref RX_PIN_NUMBER)
 */
static void uart_loopback_test()
{
    uint8_t * tx_data = (uint8_t *)("\r\nLOOPBACK_TEST\r\n");
    uint8_t   rx_data;

    // Start sending one byte and see if you get the same
    for (uint32_t i = 0; i < MAX_TEST_DATA_BYTES; i++)
    {
        uint32_t err_code;
        while (app_uart_put(tx_data[i]) != NRF_SUCCESS);

        nrf_delay_ms(10);
        err_code = app_uart_get(&rx_data);

        if ((rx_data != tx_data[i]) || (err_code != NRF_SUCCESS))
        {
            show_error();
        }
    }
    return;
}


#endif

void uart_config(void)
{
	  uint32_t err_code;

    bsp_board_leds_init();

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_ENABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);
}

void mpu_setup(void)
{
		printf("\r\nMPU setup start... \r\n");
	
    ret_code_t ret_code;
    // Initiate MPU driver
    ret_code = mpu_init();

		printf("\r\nMPU init complete! \r\n");	
	
    APP_ERROR_CHECK(ret_code); // Check for errors in return value
    
    // Setup and configure the MPU with intial values
    mpu_config_t p_mpu_config = MPU_DEFAULT_CONFIG(); // Load default values
    p_mpu_config.smplrt_div = 19;   // Change sampelrate. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). 19 gives a sample rate of 50Hz
    p_mpu_config.accel_config.afs_sel = AFS_2G; // Set accelerometer full scale range
		p_mpu_config.gyro_config.fs_sel = GFS_2000DPS; //Set gyroscope full scale range 
    ret_code = mpu_config(&p_mpu_config); // Configure the MPU with above values
    APP_ERROR_CHECK(ret_code); // Check for errors in return value 
}

void magn_setup()
{
		ret_code_t ret_code;
	
		// Setup and configure the MPU Magnetometer
		mpu_magn_config_t p_mpu_magn_config;
		p_mpu_magn_config.resolution = OUTPUT_RESOLUTION_16bit; // Set output resolution
		p_mpu_magn_config.mode = CONTINUOUS_MEASUREMENT_100Hz_MODE;	//Set measurement mode
		
		ret_code = mpu_magnetometer_init(&p_mpu_magn_config);
		APP_ERROR_CHECK(ret_code); // Check for errors in return value

}

// Function starting the internal LFCLK oscillator.
// This is needed by RTC1 which is used by the application timer
// (When SoftDevice is enabled the LFCLK is always running and this is not needed).
static void lfclk_request(void)
{
    uint32_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

// Timeout handler for the repeated timer
static void timer_a_handler(void * p_context)
{
    //nrf_gpio_pin_toggle(LED_2);
		//do nothing
}

// Create timers
static void create_timers()
{   
    uint32_t err_code;

    // Create timers
    err_code = app_timer_create(&timer1,
                                APP_TIMER_MODE_REPEATED,
                                timer_a_handler);
    APP_ERROR_CHECK(err_code);
}

static void tap_cb(unsigned char direction, unsigned char count)
{

}

static void android_orient_cb(unsigned char orientation)
{
	
}

/**
 * @brief Function for main application entry.
 */	
int main(void)
{		
		uint32_t err_code;
	  /*
		//variables carry out the reading from MPU9250
		accel_values_t acc_values;
		gyro_values_t gyro_values;
		magn_values_t magn_values;
		*/
	
		//uint8_t TempReading[3]={0};
	
		unsigned long timestamp;
		
		long *quat_data;
		int8_t accuracy;
		unsigned long inv_timestamp;
	
		//set up uart
		uart_config();
		
		//set up mpu acc & gyro
		mpu_setup();
		
		//set up mpu magn
		magn_setup();

		/*
		//i2c r/w function test
		uint8_t test_data[3]={0,0,0};
		err_code=i2c_write_porting(0x68,0x13, 3, test_data);
		err_code=mpu_twi_read_test(0x68, 0x13, 3, TempReading);
		APP_ERROR_CHECK(err_code);
		printf("Test result: %d ; %d ; %d \n", TempReading[0],TempReading[1],TempReading[2]);
		*/
		
		// Request LF clock.
    lfclk_request();
		// Initialize the application timer module.
   	err_code = app_timer_init();
		APP_ERROR_CHECK(err_code);
	
		create_timers();
	
		err_code = app_timer_start(timer1, APP_TIMER_TICKS(360000), NULL);
		APP_ERROR_CHECK(err_code);
		
		//set up logger module
		//err_code = NRF_LOG_INIT(NULL);
		//APP_ERROR_CHECK(err_code);
		
		//DMP MPL setting
		inv_error_t inv_err_code;
		
		/*********************mpu initiation******************************/
		struct int_param_s int_param;
    inv_err_code = mpu_init_inv(&int_param);
    if (inv_err_code) {
        printf("Could not initialize the mpu.\n");
    }
		/*******************end of mpu initation**************************/
		
		/*********mpl initiation*************/
		
		inv_err_code = inv_init_mpl();
		
		if (inv_err_code == INV_SUCCESS){
        printf("MPL initiated\n");			
		}
		
		/******end of mpl initiation*********/
		
		/************enable modules************/
		
		inv_enable_quaternion();
		
		//inv_enable_9x_sensor_fusion();
		
		inv_err_code = inv_enable_eMPL_outputs();
		if(inv_err_code){
				printf("enable_eMPL_outputs failed! /n");
		}
		
		/*********end of enable modules********/
		
		/*********mpl starting*************/
		
		inv_err_code = inv_start_mpl();
		
    if (inv_err_code == INV_ERROR_NOT_AUTHORIZED) {
        while (1) {
           printf("Not authorized.\n");
					 nrf_delay_ms(1000000);
        }
    }
    if (inv_err_code) {
        printf("Could not start the MPL.\n");
				nrf_delay_ms(1000000);
    }
		if (inv_err_code == INV_SUCCESS){
        printf("MPL starts!.\n");			
		}
		
		/******end of mpl starting*********/
		
		
		/*********weak up all sensors************/
		
		mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
		
		/********end of weak up all sensors******/
		
		
		/*************initialize the DMP**************/
		
		inv_err_code = dmp_load_motion_driver_firmware();
		if (inv_err_code != 0) {
			printf("Could not download DMP!!! Error Code: %d \n", inv_err_code);
    }else{
			printf("DMP downloaded! \n");
		}
		
		dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
		
		dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);
		
		hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;		
		
	  dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    inv_set_quat_sample_rate(1000000L / DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
		
		/**********end of initialize the DMP**********/
		
		/**********start functions************/
		/*
		inv_err_code = inv_start_9x_sensor_fusion();
		if (inv_err_code == INV_SUCCESS){
        printf("9x sensor fusion starts!.\n");			
		}
		*/
		
		inv_err_code = inv_start_quaternion();
		if (inv_err_code == INV_SUCCESS){
        printf("quaternion starts!.\n");			
		}
		
    //inv_stop_9x_sensor_fusion();
    
		
		//NRF_LOG_FLUSH();
		
		while (true)
    {

				
			  // Read accelerometer sensor values
        //err_code = mpu_read_accel(&acc_values);
        //APP_ERROR_CHECK(err_code);
			
				// Read gyroscope sensor values
				//err_code = mpu_read_gyro(&gyro_values);
				//APP_ERROR_CHECK(err_code);
				
				// Read Magnetometer sensor values
				//err_code = mpu_read_magnetometer(&magn_values, NULL);
				//APP_ERROR_CHECK(err_code);
			
				millis(&timestamp);
			
				inv_err_code = inv_get_sensor_type_quat(quat_data, &accuracy, (inv_time_t*)&inv_timestamp);
				
        // Clear terminal and print values
        //printf("\033[3;1HSample # %d\r\nX: %06d\r\nY: %06d\r\nZ: %06d", ++sample_number, acc_values.x, acc_values.y, acc_values.z);
				//printf("Accel Data: X: %06d ; Y: %06d ; Z: %06d ; \n ", acc_values.x, acc_values.y, acc_values.z);
				
				//printf("Gyro Data: X: %06d ; Y: %06d ; Z: %06d ; \n ", gyro_values.x, gyro_values.y, gyro_values.z);
				
				//printf("Magn Data: X: %06d ; Y: %06d ; Z: %06d ; \n ", magn_values.x, magn_values.y, magn_values.z);
				
				printf("Timestamp: T: %ld ;  \n ", timestamp);
				
				printf("Raw quat data: %ld ; Updated?: %d ; \n ", *quat_data, inv_err_code);
				
				nrf_gpio_pin_toggle(LED_1);
        nrf_delay_ms(100);
				
				//app_timer_pause();
				
								/*
        uint8_t cr;
        while (app_uart_get(&cr) != NRF_SUCCESS);
        while (app_uart_put(cr) != NRF_SUCCESS);

        if (cr == 'q' || cr == 'Q')
        {
            printf(" \r\nExit!\r\n");

            while (true)
            {
                // Do nothing.
            }
        }
				*/
    }
		
		

}


/** @} */
