#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "app_uart.h"
#include "app_error.h"
#include "app_util.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "app_mpu.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "boards.h"
//#include "nrf_drv_mpu.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"


#include "nrf_drv_inv_dmp.h"
#include "app_timer.h"

#define MPU_TWI_BUFFER_SIZE     	16 // 14 byte buffers will suffice to read acceleromter, gyroscope and temperature data in one transmission.
#define MPU_TWI_TIMEOUT 			10000 
#define MPU_ADDRESS     			0x68 
#define MPU_AK89XX_MAGN_ADDRESS     0x0C

void millis(long unsigned int *timestamp)
{
	
		uint32_t ticks = app_timer_cnt_get();
		//printf("Ticks: %d ;  \n ", ticks);
		*timestamp = (long unsigned int)ROUNDED_DIV((1000*(APP_TIMER_CONFIG_RTC_FREQUENCY+1)*ticks), APP_TIMER_CLOCK_FREQ);

}
