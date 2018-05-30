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
#include "nordic_common.h"
#include "app_uart.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "app_mpu.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_common.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "boards.h"

//#include "math.h"

//files needed for mpu
#include "nrf_drv_mpu.h"
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

//file needed for ble
#include "ble_db_discovery.h"
#include "app_util.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "softdevice_handler.h"
#include "nrf_ble_gatt.h"
#include "ble_advdata.h"
#include "ble_nus_c.h"

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

/************Parameters for BLE********************/

#define CENTRAL_LINK_COUNT      1                                       /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT   0                                       /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#define CONN_CFG_TAG            1                                       /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define SCAN_INTERVAL           0x00A0                                  /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0050                                  /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT            0x0000                                  /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS)         /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS)         /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY           0                                       /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Determines supervision time-out in units of 10 millisecond. */

#define UUID16_SIZE             2                                       /**< Size of 16 bit UUID */
#define UUID32_SIZE             4                                       /**< Size of 32 bit UUID */
#define UUID128_SIZE            16                                      /**< Size of 128 bit UUID */

#define ECHOBACK_BLE_UART_DATA  1                                       /**< Echo the UART data that is received over the Nordic UART Service back to the sender. */

static ble_nus_c_t              m_ble_nus_c;                            /**< Instance of NUS service. Must be passed to all NUS_C API calls. */
static nrf_ble_gatt_t           m_gatt;                                 /**< GATT module instance. */
static ble_db_discovery_t       m_ble_db_discovery;                     /**< Instance of database discovery module. Must be passed to all db_discovert API calls */
static uint16_t                 m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
  {
    (uint16_t)MIN_CONNECTION_INTERVAL,  // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL,  // Maximum connection
    (uint16_t)SLAVE_LATENCY,            // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT       // Supervision time-out
  };

/** @brief Parameters used when scanning. */
static ble_gap_scan_params_t const m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION <= 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION >= 3)
        .use_whitelist = 0,
    #endif
};

/**@brief NUS uuid. */
static ble_uuid_t const m_nus_uuid =
{
    .uuid = BLE_UUID_NUS_SERVICE,
    .type = NUS_SERVICE_UUID_TYPE
};

/******************parameters for mpu*******************/
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
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
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

static void uart_error_handle(app_uart_evt_t * p_event)
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

static void uart_config(void)
{
	  uint32_t err_code;

    //bsp_board_leds_init();

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

/******************funtions for ble******************/

/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    ret = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}

/**@brief Function for handling characters received by the Nordic UART Service.
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    NRF_LOG_DEBUG("Receiving data.\r\n");
    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);

    for (uint32_t i = 0; i < data_len; i++)
    {
        do
        {
            ret_val = app_uart_put(p_data[i]);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.\r\n", i);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
    if (p_data[data_len-1] == '\r')
    {
        while (app_uart_put('\n') == NRF_ERROR_BUSY);
    }
    if (ECHOBACK_BLE_UART_DATA)
    {
        // Send data back to peripheral.
        do
        {
            ret_val = ble_nus_c_string_send(&m_ble_nus_c, p_data, data_len);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. \r\n", ret_val);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
}

/**@brief Callback handling NUS Client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS Client Handle. This identifies the NUS client
 * @param[in]   p_ble_nus_evt Pointer to the NUS Client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, const ble_nus_c_evt_t * p_ble_nus_evt)
{
    ret_code_t err_code;

    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete.\r\n");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            printf("Connected to device with Nordic UART Service.\r\n");
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.\r\n");
            scan_start();
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief Reads an advertising report and checks if a UUID is present in the service list.
 *
 * @details The function is able to search for 16-bit, 32-bit and 128-bit service UUIDs.
 *          To see the format of a advertisement packet, see
 *          https://www.bluetooth.org/Technical/AssignedNumbers/generic_access_profile.htm
 *
 * @param[in]   p_target_uuid The UUID to search for.
 * @param[in]   p_adv_report  Pointer to the advertisement report.
 *
 * @retval      true if the UUID is present in the advertisement report. Otherwise false
 */
static bool is_uuid_present(ble_uuid_t               const * p_target_uuid,
                            ble_gap_evt_adv_report_t const * p_adv_report)
{
    ret_code_t   err_code;
    ble_uuid_t   extracted_uuid;
    uint16_t     index  = 0;
    uint8_t    * p_data = (uint8_t *)p_adv_report->data;

    while (index < p_adv_report->dlen)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (   (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE)
            || (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE))
        {
            for (uint32_t i = 0; i < (field_length / UUID16_SIZE); i++)
            {
                err_code = sd_ble_uuid_decode(UUID16_SIZE,
                                              &p_data[i * UUID16_SIZE + index + 2],
                                              &extracted_uuid);

                if (err_code == NRF_SUCCESS)
                {
                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
                        return true;
                    }
                }
            }
        }
        else if (   (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_MORE_AVAILABLE)
                 || (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_COMPLETE))
        {
            for (uint32_t i = 0; i < (field_length / UUID32_SIZE); i++)
            {
                err_code = sd_ble_uuid_decode(UUID32_SIZE,
                                              &p_data[i * UUID32_SIZE + index + 2],
                                              &extracted_uuid);

                if (err_code == NRF_SUCCESS)
                {
                    if (   (extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
                        return true;
                    }
                }
            }
        }

        else if (   (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE)
                 || (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE))
        {
            err_code = sd_ble_uuid_decode(UUID128_SIZE, &p_data[index + 2], &extracted_uuid);
            if (err_code == NRF_SUCCESS)
            {
                if (   (extracted_uuid.uuid == p_target_uuid->uuid)
                    && (extracted_uuid.type == p_target_uuid->type))
                {
                    return true;
                }
            }
        }
        index += field_length + 1;
    }
    return false;
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    ret_code_t            err_code;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            const ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;

            if (is_uuid_present(&m_nus_uuid, p_adv_report))
            {

                err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                              &m_scan_params,
                                              &m_connection_param,
                                              CONN_CFG_TAG);

                if (err_code == NRF_SUCCESS)
                {
                    // scan is automatically stopped by the connect
                    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
                    APP_ERROR_CHECK(err_code);
                    NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x\r\n",
                             p_adv_report->peer_addr.addr[0],
                             p_adv_report->peer_addr.addr[1],
                             p_adv_report->peer_addr.addr[2],
                             p_adv_report->peer_addr.addr[3],
                             p_adv_report->peer_addr.addr[4],
                             p_adv_report->peer_addr.addr[5]
                             );
                }
            }
        }break; // BLE_GAP_EVT_ADV_REPORT

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected to target\r\n");
            err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            // start discovery of services. The NUS Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_ble_db_discovery, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                NRF_LOG_INFO("Scan timed out.\r\n");
                scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.\r\n");
            }
            break; // BLE_GAP_EVT_TIMEOUT

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        default:
            break;
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *          been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    nrf_ble_gatt_on_ble_evt(&m_gatt, p_ble_evt);
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    ble_nus_c_on_ble_evt(&m_ble_nus_c,p_ble_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = softdevice_app_ram_start_get(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Overwrite some of the default configurations for the BLE stack.
    ble_cfg_t ble_cfg;

    // Configure the maximum number of connections.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = PERIPHERAL_LINK_COUNT;
    ble_cfg.gap_cfg.role_count_cfg.central_role_count = CENTRAL_LINK_COUNT;
    ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = 1;
    err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the maximum ATT MTU.
    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag                 = CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = NRF_BLE_GATT_MAX_MTU_SIZE;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the maximum event length.
    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag                     = CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = 320;
    ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count   = BLE_GAP_CONN_COUNT_DEFAULT;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the ATT table size.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gatts_cfg.attr_tab_size.attr_tab_size = BLE_GATTS_ATTR_TAB_SIZE_MIN;
    err_code = sd_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = softdevice_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.\r\n");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)\r\n", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}

/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the NUS Client. */
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler = ble_nus_c_evt_handler;

    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
}

static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LED, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the Database Discovery Module.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}

static void ble_app_uart_c_setup(){
	
		//initializing buttons and leds
		buttons_leds_init();
		
		//initializing the Database Discovery Module
		db_discovery_init();
		
		//initializing the BLE stack
		ble_stack_init();
		
		//initializing the GATT library
		gatt_init();
		
		//initializing the NUS Client
		nus_c_init();
	
}

static void pin_in_read(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
		hal.new_gyro = 1;
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

/******************functions for mpu******************/

static void mpu_twi_setup(void)
{
		//NRF_LOG_RAW_INFO("\r\nMPU setup start... \r\n");
		
    ret_code_t ret_code;
    // Initiate MPU driver
    ret_code = mpu_init();
    APP_ERROR_CHECK(ret_code); // Check for errors in return value
    	
		NRF_LOG_RAW_INFO("\r\nTWI starts! \r\n");
	
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
		
		if (!inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)){
				return;
    }
		
		if (!inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp)){
				return;
		}
		
		quat_print[0]= data[0] * 1.0 / (1<<30);
		quat_print[1]= data[1] * 1.0 / (1<<30);			
		quat_print[2]= data[2] * 1.0 / (1<<30);
		quat_print[3]= data[3] * 1.0 / (1<<30);
		
		printf("%7.5f,%7.5f,%7.5f,%7.5f,%7.5f,%7.5f,%7.5f \r\n",quat_print[0],quat_print[1],quat_print[2],quat_print[3]	\
																													,float_data[0],float_data[1],float_data[2]);
		//printf("%7.5f,%7.5f,%7.5f \r\n",float_data[0],float_data[1],float_data[2]);
		
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
	
		// Request LF clock.
    lfclk_request();
		// Initialize the application timer module.
   	err_code = app_timer_init();
		APP_ERROR_CHECK(err_code);	
		create_timers();
		err_code = app_timer_start(timer1, APP_TIMER_TICKS(360000), NULL);
		APP_ERROR_CHECK(err_code);
	
		//set up uart
		uart_config();
	
		//set up NRF logger module
		APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
		
		/**************Set up ble**************/
		ble_app_uart_c_setup();
		
		//set up GPIOTE
		GPIO_setup();
		
		//set up mpu acc & gyro
		mpu_twi_setup();
		
		//i2c r/w function test
		/*
		uint8_t test_data[3]={0,0,0};
		err_code=i2c_write_porting(0x68,0x13, 3, test_data);
		err_code=mpu_twi_read_test(0x68, 0x13, 3, TempReading);
		APP_ERROR_CHECK(err_code);
		NRF_LOG_RAW_INFO("Test result: %d ; %d ; %d \r\n", TempReading[0],TempReading[1],TempReading[2]);
		*/
			
		/************************* DMP MPL setting ***********************/
		inv_error_t inv_err_code;
		
		/*********************mpu initiation******************************/
		
    inv_err_code = mpu_init_inv(&int_param);
    if (inv_err_code) {
				NRF_LOG_RAW_INFO("Could not initialize the mpu.\r\n");
		}else{
				NRF_LOG_RAW_INFO("MPU starts! \r\n");
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
		//inv_enable_in_use_auto_calibration();
		
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
			nrf_delay_ms(1000);
			inv_err_code = dmp_load_motion_driver_firmware();
		}
		
		if (inv_err_code != 0) {
			NRF_LOG_RAW_INFO("Could not load DMP Image!!! Error Code: %d \r\n", inv_err_code);
    }else{
			NRF_LOG_RAW_INFO("DMP downloaded! \r\n");
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
		
		
		//100Hz
		if (hal.dmp_on) {
				dmp_set_fifo_rate(100);
				inv_set_quat_sample_rate(10000L);
		} else
				mpu_set_sample_rate(100);
		inv_set_gyro_sample_rate(10000L);
		inv_set_accel_sample_rate(10000L);
		
		/*
		//50Hz
		if (hal.dmp_on) {
				dmp_set_fifo_rate(50);
				inv_set_quat_sample_rate(20000L);
		} else
				mpu_set_sample_rate(50);
		inv_set_gyro_sample_rate(20000L);
		inv_set_accel_sample_rate(20000L);
		*/
		
		__enable_irq();
		
		run_self_test();
		
		/**********end of initialize the DMP**********/
				
		
		while(1){
				
				/*
				uint8_t cr;
				app_uart_get(&cr);
				app_uart_put(cr);
				if(cr=='t'){
						run_self_test();
					  inv_accel_was_turned_off();
						inv_gyro_was_turned_off();
						inv_compass_was_turned_off();
				}
				*/
			
				unsigned long sensor_timestamp;
				int new_data = 0;

				millis(&timestamp);

				//new_compass = hal.new_gyro;
				new_compass=0;
			
				/*
        if ((timestamp > hal.next_compass_ms) && !hal.lp_accel_mode &&
            hal.new_gyro && (hal.sensors & COMPASS_ON)) {
            hal.next_compass_ms = timestamp + COMPASS_READ_MS;
            new_compass = 1;
        }
				*/
						
				if (!hal.sensors || !hal.new_gyro) {
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
						
						/*
						//Debug for DMP quat output
						double quat_print[4] = {0};
						quat_print[0]= quat[0] * 1.0 / (1<<30);
						quat_print[1]= quat[1] * 1.0 / (1<<30);			
						quat_print[2]= quat[2] * 1.0 / (1<<30);
						quat_print[3]= quat[3] * 1.0 / (1<<30);
		
						printf("%7.5f,%7.5f,%7.5f,%7.5f,%7.5f,%7.5f,%7.5f \r\n",quat_print[0],quat_print[1],quat_print[2],quat_print[3]	\
																													, 0.0, 0.0, 0.0);
						*/
						
						
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
