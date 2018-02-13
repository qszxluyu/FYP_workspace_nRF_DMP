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
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "app_mpu.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "boards.h"


#include "math.h"

#include "inv_mpu.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"

#include "nrf_drv_inv_dmp.h"

//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 1024                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */


// General application timer settings.
APP_TIMER_DEF(timer1);

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
    p_mpu_config.accel_config.afs_sel = AFS_16G; // Set accelerometer full scale range
		p_mpu_config.gyro_config.fs_sel = GFS_1000DPS; //Set gyroscope full scale range 
    ret_code = mpu_config(&p_mpu_config); // Configure the MPU with above values
    APP_ERROR_CHECK(ret_code); // Check for errors in return value 
}

void magn_setup()
{
		ret_code_t ret_code;
		
		
		//Enable bypass mode
	/*
		mpu_int_pin_cfg_t bypass_config;
		bypass_config.i2c_bypass_en = 1;
		ret_code = mpu_int_cfg_pin(&bypass_config); // Set bypass mode
		APP_ERROR_CHECK(ret_code); // Check for errors in return value
	*/	
	
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

/**
 * @brief Function for main application entry.
 */	
int main(void)
{		
		uint32_t err_code;
		//variables carry out the reading from MPU9250
		accel_values_t acc_values;
		gyro_values_t gyro_values;
		magn_values_t magn_values;
	
		unsigned long timestamp;
	
		//set up uart
		uart_config();
		
		//printf("\r\nTest1: MPU twi communication \r\n");
		
		//set up mpu acc & gyro
		mpu_setup();

		//printf("\r\nMPU setup complete! \r\n");	
		
		//set up mpu magn
		magn_setup();
	
		// Request LF clock.
    lfclk_request();
		// Initialize the application timer module.
   	err_code = app_timer_init();
		APP_ERROR_CHECK(err_code);
	
		create_timers();
	
		err_code = app_timer_start(timer1, APP_TIMER_TICKS(216000), NULL);
		APP_ERROR_CHECK(err_code);		
	

    while (true)
    {
				
			  // Read accelerometer sensor values
        err_code = mpu_read_accel(&acc_values);
        APP_ERROR_CHECK(err_code);
			
				// Read gyroscope sensor values
				err_code = mpu_read_gyro(&gyro_values);
				APP_ERROR_CHECK(err_code);
				
				// Read Magnetometer sensor values
				err_code = mpu_read_magnetometer(&magn_values, NULL);
				APP_ERROR_CHECK(err_code);
			
				millis(&timestamp);
				
        // Clear terminal and print values
        //printf("\033[3;1HSample # %d\r\nX: %06d\r\nY: %06d\r\nZ: %06d", ++sample_number, acc_values.x, acc_values.y, acc_values.z);
				printf("Accel Data: X: %06d ; Y: %06d ; Z: %06d ; \n ", acc_values.x, acc_values.y, acc_values.z);
				
				printf("Gyro Data: X: %06d ; Y: %06d ; Z: %06d ; \n ", gyro_values.x, gyro_values.y, gyro_values.z);
				
				printf("Magn Data: X: %06d ; Y: %06d ; Z: %06d ; \n ", magn_values.x, magn_values.y, magn_values.z);
				
				printf("Timestamp: T: %ld ;  \n ", timestamp);
				
				nrf_gpio_pin_toggle(LED_1);
        nrf_delay_ms(50);
				
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
