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
#include "nrf_drv_gpiote.h"
#include "nrf_drv_common.h"
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
#include "fast_no_motion.h"
#include "compass_vec_cal.h"
#include "mag_disturb.h"
#include "hal_outputs.h"
#include "mpl.h"

#include "nrf_drv_inv_dmp.h"

#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "softdevice_handler.h"
#include "nrf_ble_gatt.h"
#include "ble_advdata.h"
#include "ble_nus_c.h"


//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 1024                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

#define MPU_ADDRESS     			0x68 
#define MPU_AK89XX_MAGN_ADDRESS     0x0C

#define DEFAULT_MPU_HZ  (20)

#define PIN_IN ARDUINO_7_PIN

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define COMPASS_READ_MS (100)

/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

enum t_axisOrder {
	X_AXIS, // 0
	Y_AXIS, // 1
	Z_AXIS  // 2
};

// General application timer settings.
APP_TIMER_DEF(timer1);

volatile unsigned char rx_new;
unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

struct platform_data_s {
		signed char orientation[9];
};

static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0,-1, 0,
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
		NRF_LOG_RAW_INFO("\r\nMPU setup start... \r\n");
		
    ret_code_t ret_code;
    // Initiate MPU driver
    ret_code = mpu_init();

		NRF_LOG_RAW_INFO("\r\nMPU init complete! \r\n");	
	
		//NRF_LOG_FLUSH();
	
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

static void pin_in_read(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
		hal.new_gyro = 1;
		//Do nothing
}

static inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7) {
	NRF_LOG_RAW_INFO("Self Test Passed!\r\n");
        /*
				NRF_LOG_RAW_INFO("accel: %7.4f %7.4f %7.4f\r\n",
                    accel[0]/65536.f,
                    accel[1]/65536.f,
                    accel[2]/65536.f);
        NRF_LOG_RAW_INFO("gyro: %7.4f %7.4f %7.4f\r\n",
                    gyro[0]/65536.f,
                    gyro[1]/65536.f,
                    gyro[2]/65536.f);
        */
				/* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
        /*
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 2048.f; //convert to +-16G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
#else
        /* Push the calibrated data to the MPL library.
         *
         * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
    	unsigned short accel_sens;
    	float gyro_sens;

		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		inv_set_accel_bias(accel, 3);
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
		inv_set_gyro_bias(gyro, 3);
#endif
    }
    else {
            if (!(result & 0x1))
                NRF_LOG_RAW_INFO("Gyro failed.\r\n");
            if (!(result & 0x2))
                NRF_LOG_RAW_INFO("Accel failed.\r\n");
            if (!(result & 0x4))
                NRF_LOG_RAW_INFO("Compass failed.\r\n");
     }

}

static void tap_cb(unsigned char direction, unsigned char count)
{	
		//Do nothing
}

static void android_orient_cb(unsigned char orientation)
{
		//Do nothing
}

static void gyro_data_ready_cb(void)
{
    //hal.new_gyro = 1;
		//Do nothing, pin_in_read() does the job of this part
}

static void GPIO_setup()
{
		uint32_t err_code = nrf_drv_gpiote_init();
		APP_ERROR_CHECK(err_code);
	
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;
	
	  err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, pin_in_read);
    APP_ERROR_CHECK(err_code);		

		nrf_drv_gpiote_in_event_enable(PIN_IN, true);	
}

/* Get data from MPL.
 * TODO: Add return values to the inv_get_sensor_type_xxx APIs to differentiate
 * between new and stale data.
 */
static void read_from_mpl(void)
{
    long data[9];
    int8_t accuracy;
    unsigned long timestamp;
    float float_data[3] = {0};
		double quat_print[4] = {0};
		
		if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)){
				
				quat_print[0]= data[0] * 1.0 / (1<<30);
				quat_print[1]= data[1] * 1.0 / (1<<30);			
				quat_print[2]= data[2] * 1.0 / (1<<30);
				quat_print[3]= data[3] * 1.0 / (1<<30);
				
				NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER ",", NRF_LOG_FLOAT(quat_print[0]));
				NRF_LOG_FLUSH();
				NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER ",", NRF_LOG_FLOAT(quat_print[1]));
				NRF_LOG_FLUSH();
				NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER ",", NRF_LOG_FLOAT(quat_print[2]));
				NRF_LOG_FLUSH();
				NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER ",", NRF_LOG_FLOAT(quat_print[3]));
				NRF_LOG_FLUSH();
				
    } else{
			return;
		}
		
		if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp)){
			
				NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER ",", NRF_LOG_FLOAT(float_data[0]));
				NRF_LOG_FLUSH();
				NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER ",", NRF_LOG_FLOAT(float_data[1]));
				NRF_LOG_FLUSH();	
				NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(float_data[2]));
				NRF_LOG_FLUSH();
			
		}
		
}

/**
 * @brief Function for main application entry.
 */	
int main(void)
{		
		uint32_t err_code;
	
	  unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
		unsigned long timestamp;
		struct int_param_s int_param;
	
	  unsigned char new_compass = 0;
		unsigned short compass_fsr;
			
		//set up uart
		uart_config();
	
		//set up GPIOTE
		GPIO_setup();
		
		//set up mpu acc & gyro
		mpu_setup();
		//NRF_LOG_FLUSH();
		
		//set up mpu magn
		magn_setup();


		//set up NRF logger module
		APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

		
		//i2c r/w function test
		/*
		uint8_t test_data[3]={0,0,0};
		err_code=i2c_write_porting(0x68,0x13, 3, test_data);
		err_code=mpu_twi_read_test(0x68, 0x13, 3, TempReading);
		APP_ERROR_CHECK(err_code);
		NRF_LOG_RAW_INFO("Test result: %d ; %d ; %d \r\n", TempReading[0],TempReading[1],TempReading[2]);
		*/
		
		// Request LF clock.
    lfclk_request();
		// Initialize the application timer module.
   	err_code = app_timer_init();
		APP_ERROR_CHECK(err_code);
	
		create_timers();
	
		err_code = app_timer_start(timer1, APP_TIMER_TICKS(360000), NULL);
		APP_ERROR_CHECK(err_code);
		
		
		/************************* DMP MPL setting ***********************/
		inv_error_t inv_err_code;
		
		/*********************mpu initiation******************************/
		
    inv_err_code = mpu_init_inv(&int_param);
    if (inv_err_code) {
				NRF_LOG_RAW_INFO("Could not initialize the mpu.\r\n");
		}
		
		/*******************end of mpu initation**************************/
		
		/*********mpl initiation*************/
		
		inv_err_code = inv_init_mpl();
		
		if (inv_err_code){
				NRF_LOG_RAW_INFO("Could not initialize MPL.\r\n");
		}
		
		/******end of mpl initiation*********/
		
		/************enable modules************/
		
		inv_enable_quaternion();
		
		inv_enable_9x_sensor_fusion();
		
		inv_enable_fast_nomot();
		
		inv_enable_vector_compass_cal();
    
		inv_enable_magnetic_disturbance();
		
		inv_err_code = inv_enable_eMPL_outputs();
		if(inv_err_code){
				NRF_LOG_RAW_INFO("enable_eMPL_outputs failed! \r\n");
		}
		
		/*********end of enable modules********/
		
		/*********mpl starting*************/
		
		inv_err_code = inv_start_mpl();
		
    if (inv_err_code == INV_ERROR_NOT_AUTHORIZED) {
        while (1) {
					 NRF_LOG_RAW_INFO("Not authorized.\r\n");
					 nrf_delay_ms(1000000);
        }
    }
    if (inv_err_code) {
				NRF_LOG_RAW_INFO("Could not start the MPL.\r\n");
				nrf_delay_ms(1000000);
    }
		if (inv_err_code == INV_SUCCESS){
				NRF_LOG_RAW_INFO("MPL starts! \r\n");
		}
		
		NRF_LOG_FLUSH();
		
		/******end of mpl starting*********/
		
		
		/***********set up hardware************/
		
		inv_err_code = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
		
		inv_err_code = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL); //magn data doesn't go to fifo
		
		inv_err_code = mpu_set_sample_rate(DEFAULT_MPU_HZ);
		
		inv_err_code = mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
		
		/********end of set up hardware********/
		
		/***Read back configuration in case it was set improperly. ***/
		
    mpu_get_sample_rate(&gyro_rate);
		
    mpu_get_gyro_fsr(&gyro_fsr);
		
    mpu_get_accel_fsr(&accel_fsr);
		
		mpu_get_compass_fsr(&compass_fsr);

		
		/**********************end of read back***********************/
		
		/************* Sync driver configuration with MPL*************/
    /* Sample rate expected in microseconds. */
		
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
    /* The compass rate is independent of the gyro and accel rates. As long as
     * inv_set_compass_sample_rate is called with the correct value, the 9-axis
     * fusion algorithm's compass correction gain will work properly.
     */
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
		
		/**********end of Sync driver configuration with MPL**********/
    
		/************ Set chip-to-body orientation matrix*************/
    //Set hardware units to dps/g's/degrees scaling factor.
     
    inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)gyro_fsr<<15);
						
    inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)accel_fsr<<15);	
						
    inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(compass_pdata.orientation),
            (long)compass_fsr<<15);						

		/*********end of set chip-to-body orientation matrix**********/
		
		/****************Initialize HAL state variables***************/

    hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;

    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

		/*************end of initialize HAL state variables************/
		
		millis(&timestamp);

		/*************initialize the DMP**************/
		
		inv_err_code = dmp_load_motion_driver_firmware();
		while(inv_err_code != 0){
			NRF_LOG_RAW_INFO("Could not load DMP Image, Error Code: %d, retry in 1 sec \r\n", inv_err_code);
			NRF_LOG_FLUSH();
			nrf_delay_ms(1000);
			inv_err_code = dmp_load_motion_driver_firmware();
		}
		
		if (inv_err_code != 0) {
			NRF_LOG_RAW_INFO("Could not load DMP Image!!! Error Code: %d \r\n", inv_err_code);
			NRF_LOG_FLUSH();
    }else{
			NRF_LOG_RAW_INFO("DMP downloaded! \r\n");
			NRF_LOG_FLUSH();
		}
		
		dmp_set_orientation(
				inv_orientation_matrix_to_scalar(gyro_pdata.orientation));	
		dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);
		/*
		* Known Bug -
		* DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
		* specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
		* a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
		* there will be a 25Hz interrupt from the MPU device.
		*
		* There is a known issue in which if you do not enable DMP_FEATURE_TAP
		* then the interrupts will be at 200Hz even if fifo rate
		* is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
		*
		* DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
		*/

    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
	  dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
		
		/*
		//100Hz
		if (hal.dmp_on) {
				dmp_set_fifo_rate(100);
				inv_set_quat_sample_rate(10000L);
		} else
				mpu_set_sample_rate(100);
		inv_set_gyro_sample_rate(10000L);
		inv_set_accel_sample_rate(10000L);
		*/
		
		/*
		//40Hz
		if (hal.dmp_on) {
				dmp_set_fifo_rate(40);
				inv_set_quat_sample_rate(25000L);
		} else
				mpu_set_sample_rate(40);
		inv_set_gyro_sample_rate(25000L);
		inv_set_accel_sample_rate(25000L);
		*/
		
		//50Hz
		if (hal.dmp_on) {
				dmp_set_fifo_rate(50);
				inv_set_quat_sample_rate(20000L);
		} else
				mpu_set_sample_rate(50);
		inv_set_gyro_sample_rate(20000L);
		inv_set_accel_sample_rate(20000L);

		/*
		//10Hz
		if (hal.dmp_on) {
				dmp_set_fifo_rate(10);
				inv_set_quat_sample_rate(100000L);
		} else
				mpu_set_sample_rate(10);
		inv_set_gyro_sample_rate(100000L);
		inv_set_accel_sample_rate(100000L);
		*/

		__enable_irq();
		
		run_self_test();

		NRF_LOG_FLUSH();
		
		/**********end of initialize the DMP**********/
				
		
		while(1){
								
				unsigned long sensor_timestamp;
				int new_data = 0;

				millis(&timestamp);

				new_compass = hal.new_gyro;
				
				if (!hal.sensors || !hal.new_gyro || !new_compass) {
						continue;
				}    

				if (hal.new_gyro && hal.lp_accel_mode) {
						//lp_accel_mode is not used in this project
				} else if (hal.new_gyro && hal.dmp_on) {
						short gyro[3], accel_short[3], sensors;
						unsigned char more;
						long accel[3], quat[4];
						/* This function gets new data from the FIFO when the DMP is in
						 * use. The FIFO can contain any combination of gyro, accel,
						 * quaternion, and gesture data. The sensors parameter tells the
						 * caller which data fields were actually populated with new data.
						 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
						 * the FIFO isn't being filled with accel data.
						 * The driver parses the gesture data to determine if a gesture
						 * event has occurred; on an event, the application will be notified
						 * via a callback (assuming that a callback function was properly
						 * registered). The more parameter is non-zero if there are
						 * leftover packets in the FIFO.
						 */
						inv_err_code = dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
						
						if(inv_err_code){
								//NRF_LOG_RAW_INFO("read fifo failed, err code: %d \r\n",inv_err_code);
								//NRF_LOG_FLUSH();
								continue;
						}
						if (!more){
								hal.new_gyro = 0;
						}
						if (sensors & INV_XYZ_GYRO) {
								inv_build_gyro(gyro, sensor_timestamp);
								new_data = 1;
						}
						if (sensors & INV_XYZ_ACCEL) {
								accel[0] = (long)accel_short[0];
								accel[1] = (long)accel_short[1];
								accel[2] = (long)accel_short[2];
								inv_build_accel(accel, 0, sensor_timestamp);
								new_data = 1;
						}
						if (sensors & INV_WXYZ_QUAT) {
								inv_build_quat(quat, 0, sensor_timestamp);
								new_data = 1;
						}
				}

				if (new_compass) {
						short compass_short[3];
						long compass[3];
						new_compass = 0;
						/* For any MPU device with an AKM on the auxiliary I2C bus, the raw
						 * magnetometer registers are copied to special gyro registers.
						 */
						inv_err_code = mpu_get_compass_reg(compass_short, &sensor_timestamp);
						if (!inv_err_code) {
								compass[0] = (long)compass_short[0];
								compass[1] = (long)compass_short[1];
								compass[2] = (long)compass_short[2];
								/* NOTE: If using a third-party compass calibration library,
								 * pass in the compass data in uT * 2^16 and set the second
								 * parameter to INV_CALIBRATED | acc, where acc is the
								 * accuracy from 0 to 3.
								 */
								inv_build_compass(compass, 0, sensor_timestamp);
						}
						new_data = 1;
				}

				if (new_data) {
						inv_err_code=inv_execute_on_data();
						
						/* This function reads bias-compensated sensor data and sensor
						 * fusion outputs from the MPL. The outputs are formatted as seen
						 * in eMPL_outputs.c. This function only needs to be called at the
						 * rate requested by the host.
						 */
						read_from_mpl();

				}
				
    }

}


/** @} */
