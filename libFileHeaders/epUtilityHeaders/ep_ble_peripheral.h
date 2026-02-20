/****************************************************************************
 * Copyright (c) 2022 Embedded Planet, Inc.                                 *
 * SPDX-License-Identifier: Apache-2.0                                      *
 *                                                                          *
 * Licensed under the Apache License, Version 2.0 (the "License");          *
 * you may not use this file except in compliance with the License.         *
 * You may obtain a copy of the License at                                  *
 *                                                                          *
 *     http://www.apache.org/licenses/LICENSE-2.0                           *
 *                                                                          *
 * Unless required by applicable law or agreed to in writing, software      *
 * distributed under the License is distributed on an "AS IS" BASIS,        *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 * See the License for the specific language governing permissions and      *
 * limitations under the License.                                           *
 ****************************************************************************/

/**
 * @file    ep_ble_peripheral.h
 * @version 1.0.0
 * @author  Embedded Planet, Inc.
 * @author  Dan Maher
 * @date    13 Oct 2022
 *
 * @brief Contains functions for the configuration and control of the Embedded Planet BLE Peripheral utility
 *
 * @note Based on the nRF5 SDK BLE Peripheral Template example
 *
 * Built for use with the nRF5 SDK 17.1 and FreeRTOS.
 *
 * @see Error codes:    https://infocenter.nordicsemi.com/index.jsp?topic=%2Fsdk_nrf5_v17.1.0%2Fgroup__nrfx__error__codes.html
 */

#ifndef EP_BLE_PERIPHERAL_H
#define EP_BLE_PERIPHERAL_H

#include "main.h"
#include "ble_advertising.h"
#include "nrf_sdh_ble.h"
#include "fds.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"

#ifndef DEVICE_NAME
    #define DEVICE_NAME                     "EP_DEV"                                /**< Name of device. Will be included in the advertising data. */
#endif

#ifndef MANUFACTURER_NAME
    #define MANUFACTURER_NAME               "Embedded_Planet_Inc"                   /**< Manufacturer. Will be passed to Device Information Service. */
#endif

#ifndef MANUFACTURER_ID
    #define MANUFACTURER_ID                 0xFFFF                                  /** < Manufacturer ID. 0xFFFF only to be used during development */
#endif

#ifndef DEVICE_APPEARANCE
    #define DEVICE_APPEARANCE               0x0540                                  /**< Device appearance to add to adv data. 0x0540 is generic sensor, see BLE spec for full list */
#endif

#ifndef APP_ADV_INTERVAL
    #define APP_ADV_INTERVAL                10000                                   /**< The advertising interval (in units of 0.625 ms. This value corresponds to 6250ms). */
#endif

#ifndef APP_ADV_DURATION
    #define APP_ADV_DURATION                3000                                    /**< The advertising duration in units of 10 milliseconds. */
#endif

#ifndef APP_ADV_WAIT
    #define APP_ADV_WAIT                    pdMS_TO_TICKS(300000)                   /**< The wait time between advertising sessions, argument in milliseconds. */
#endif

#ifndef APP_CONT_ADV
    #define APP_CONT_ADV                    0                                       /**< Perform continuous advertising (auto-restart immediately after timeout) */
#endif

#ifndef MIN_CONN_INTERVAL
    #define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#endif

#ifndef MAX_CONN_INTERVAL
    #define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#endif

#ifndef SLAVE_LATENCY
    #define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#endif

#ifndef CONN_SUP_TIMEOUT
    #define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */
#endif

#ifndef FIRST_CONN_PARAMS_UPDATE_DELAY
    #define FIRST_CONN_PARAMS_UPDATE_DELAY  5000                                    /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#endif

#ifndef NEXT_CONN_PARAMS_UPDATE_DELAY
    #define NEXT_CONN_PARAMS_UPDATE_DELAY   30000                                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#endif

#ifndef MAX_CONN_PARAMS_UPDATE_COUND
    #define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */
#endif

#ifndef BLE_TX_POWER_BOOST
    #define BLE_TX_POWER_BOOST              8                                       /** < BLE TX power boost. For details, see sd_ble_gap_tx_power_set() in ble_gap.h */
#endif

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define BLE_SERVICE_UUID_MASK  0x00FF /** < 16-bit service UUID mask */

#if BLE_SERVICE_SYSTEM
    #include "ble_service_system.h"
    extern ble_serv_system system_service;
#endif

#if BLE_SERVICE_CELL
    #include "ble_service_cell.h"
    extern ble_serv_cell cell_service;
#endif

#if BLE_SERVICE_BME680
    #include "bme680.h"
    #include "ble_service_bme680.h"
    extern ble_serv_bme680 bme680_service;
#endif

#if BLE_SERVICE_ICM20602
    #include "icm20602.h"
    #include "ble_service_icm20602.h"
    extern ble_serv_icm20602 icm20602_service;
#endif

#if BLE_SERVICE_ISOSMART
    #include "ble_service_isosmart.h"
    extern ble_serv_isosmart isosmart_service;
#endif
//NEW SENSOR INCLUDE INFORMATION HERE

typedef enum{
    EP_BLE_NEVER_CONNECTED,
    EP_BLE_DISCONNECTED,
    EP_BLE_TIMEOUT,
    EP_BLE_CONNECTED
} EP_BLE_PERIPH_STATUS_ENUM;

NRF_BLE_GATT_DEF(m_gatt);           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);             /**< Context for the Queued Write module.*/

/**
 * @brief Needs to be called by the application to initialize BLE.
 *
 * @param  devName      Pointer to name for peripheral device
 * @param  devNameSize  Size of dev name
 * @param  epSerial     Pointer to serial number string
 */
void ep_ble_peripheral_init(char *devName, uint8_t devNameSize, char *epSerial);

/**
 * @brief Needs to be called by the application to uninitialize BLE.
 *
 */
void ep_ble_peripheral_uninit(void);

/**
 * @brief Used to check the connection status of the peripheral
 *
 * @return EP_BLE_PERIPH_STATUS_ENUM indicates whether a central
 * is connected, disconnected, or if the advertising period has timed out
 */
EP_BLE_PERIPH_STATUS_ENUM ep_ble_peripheral_status(void);

/**
 * @brief Used to update the advertising packet
 *
 * @param data          6 bytes of user defined data
 * @param ep_serial     Serial number
 */
void ep_ble_beacon_update(uint8_t data[6], uint8_t ep_serial[7]);


/**
 * @}
 */

#endif