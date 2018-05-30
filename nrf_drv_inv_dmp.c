#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "app_uart.h"
#include "app_error.h"
#include "app_util.h"
#include "nrf_delay.h"
#include "nrf.h"
//#include "bsp.h"
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

#include <stdarg.h> 
#include "log.h"
#include "nrf_log_internal.h"

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(NRF_LOG)
#include "nrf_strerror.h"
#define NRF_LOG_ERROR_STRING_GET(code) nrf_strerror_get(code)
#else
#define NRF_LOG_ERROR_STRING_GET(code) ""
#endif

#define BUF_SIZE        (256)
#define PACKET_LENGTH   (23)
#define PACKET_DEBUG    (1)
#define PACKET_QUAT     (2)
#define PACKET_DATA     (3)

void millis(long unsigned int *timestamp)
{
	
		uint32_t ticks = app_timer_cnt_get();
		//printf("Ticks: %d ;  \n ", ticks);
		*timestamp = (long unsigned int)ROUNDED_DIV((1000*(APP_TIMER_CONFIG_RTC_FREQUENCY+1)*ticks), APP_TIMER_CLOCK_FREQ);

}

int _MLPrintLog (int priority, const char* tag, const char* fmt, ...)
{
	  va_list args;
    int length, ii;
    char buf[BUF_SIZE], out[PACKET_LENGTH], this_length;

    /* This can be modified to exit for unsupported priorities. */
    switch (priority) {
    case MPL_LOG_UNKNOWN:
    case MPL_LOG_DEFAULT:
    case MPL_LOG_VERBOSE:
    case MPL_LOG_DEBUG:
    case MPL_LOG_INFO:
    case MPL_LOG_WARN:
    case MPL_LOG_ERROR:
    case MPL_LOG_SILENT:
        break;
    default:
        return 0;
    }
		
		va_start(args, fmt);

    length = vsprintf(buf, fmt, args);
    if (length <= 0) {
        va_end(args);
        return length;
    }
		 memset(out, 0, PACKET_LENGTH);
    out[0] = '$';
    out[1] = PACKET_DEBUG;
    out[2] = priority;
    out[21] = '\r';
    out[22] = '\n';
    for (ii = 0; ii < length; ii += (PACKET_LENGTH-5)) {
#define min(a,b) ((a < b) ? a : b)
        this_length = min(length-ii, PACKET_LENGTH-5);
        memset(out+3, 0, 18);
        memcpy(out+3, buf+ii, this_length);
				NRF_LOG_INFO("%s",nrf_log_push(out));
    }
    va_end(args);

    return 0;
}
