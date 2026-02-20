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
 * @file    ep_ble_central.h
 * @version 1.0.0
 * @author  Embedded Planet, Inc.
 * @author  Dan Maher
 * @date    27 Oct 2022
 * 
 * @brief Public function declarations for initialization of the EP BLE Central utility
 * 
 * Built for use with the nRF5 SDK 17.1 and FreeRTOS.
 * 
 * @see Error codes:    https://infocenter.nordicsemi.com/index.jsp?topic=%2Fsdk_nrf5_v17.1.0%2Fgroup__nrfx__error__codes.html
 */

#ifndef EP_BLE_CENTRAL_H
#define EP_BLE_CENTRAL_H

#include <stdint.h>
#include <stdint.h>
#include "main.h"

#include "ble_gap.h"

#include "FreeRTOS.h"
#include "queue.h"

#if BLE_SERVICE_AGORA
#include "agora_ble.h"
#endif

#define APP_BLE_CONN_CFG_TAG      1     /**< Tag that refers to the BLE stack configuration that is set with @ref sd_ble_cfg_set. The default tag is @ref APP_BLE_CONN_CFG_TAG. */
#define APP_BLE_OBSERVER_PRIO     3     /**< BLE observer priority of the application. There is no need to modify this value. */

#ifndef BLE_TX_POWER_BOOST
    #define BLE_TX_POWER_BOOST      8     /** < BLE TX power boost. For details, see sd_ble_gap_tx_power_set() in ble_gap.h */
#endif

#ifndef TARGET_PERIPH_NAME_COUNT
    #define TARGET_PERIPH_NAME_COUNT     1   /** < Number of possible device types that the central can connect to */
#endif

#ifndef TARGET_PERIPH_NAMES
    #define TARGET_PERIPH_NAMES (const char*[TARGET_PERIPH_NAME_COUNT]){"EP_DEV"}   /** < BLE peripheral advertising names that the central will try to connect to */
#endif

#define BLE_ADV_QUEUE_SIZE      10                      /** < Maximum number of items that the queue can hold */

typedef struct{
    bool beacon_active;     /** < Indicates whether or not the beacon slot is active */
    bool ignore_beacon;     /** < Indicates whether or not the beacon is currently being ignored by the central */
    uint32_t last_active;   /** < Time in seconds that this beacon was last active. */
    uint32_t ignore_time;   /** < Time in seconds that this beacon shall remain ignored by the central. */
    uint8_t addr[BLE_GAP_ADDR_LEN];     /**< 48-bit address, LSB format */
} ble_beacons;

extern ble_beacons beacons[TARGET_PERIPH_ADDR_COUNT];

/**
 * @brief Advertising packet that is passed to xBleAdvQueue
 * 
 */
typedef struct {
    bool            scan_response;      /** < True if the data is a scan response packet. Otherwise false. */
    uint8_t         data[31];           /** < The raw advertising packet data */
    uint8_t         size;               /** < The size of the advertising packet in bytes */
    uint8_t         beacon_index;       /** < The beacon index number of the originating device from the beacons struct. */
    uint32_t        timestamp;          /** < System timestamp in seconds that the beacon data was received */
    int8_t          rssi;               /** < Received Signal Strength Indication in dBm of the packet */
} ble_adv_data;

extern QueueHandle_t xBleAdvQueue;   /** < Used to pass advertising packet data to external modules */

/**
 * @brief Needs to be called by the application to initialize BLE central.
 * 
 * @param connect_if_match  True - connect to peripheral, use GATT UUID data, False - don't connec to peripheral, only use beacon data
 * 
 */
void ep_ble_central_init(bool connect_if_match);

void ble_scan_restart(void);

#endif