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
 * @file    ble_service_system.h
 * @version 1.0.0
 * @author  Embedded Planet, Inc.
 * @author  Dan Maher
 * @date    13 Oct 2022
 * 
 * @brief Contains functions for the configuration and control of the System service. For use with the ep_ble_peripheral utility.
 * 
 * @note Based on the nRF5 SDK BLE Peripheral Template example
 * 
 * Built for use with the nRF5 SDK 17.1 and FreeRTOS.
 * 
 * @see Error codes:    https://infocenter.nordicsemi.com/index.jsp?topic=%2Fsdk_nrf5_v17.1.0%2Fgroup__nrfx__error__codes.html
 */

#ifndef BLE_SERVICE_SYSTEM_H
#define BLE_SERVICE_SYSTEM_H

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_UUID_EP_BASE    {0xA7, 0x04, 0x62, 0x56, 0xD5, 0x10, 0x87, 0xB8, 0x91, 0x49, 0xAB, 0x99, 0x00, 0x00, 0x00, 0x00} // 128-bit base UUID
#define BLE_UUID_SYSTEM_SERVICE 0x0100 // System Service UUID

// System characteristic UUIDs
typedef enum{
    BLE_UUID_SYSTEM_SERIAL  = 0x0101,
    BLE_UUID_SYSTEM_GATT    = 0x0102,
    BLE_UUID_SYSTEM_FW      = 0x0103,
    BLE_UUID_SYSTEM_TIME    = 0x0104,
    BLE_UUID_SYSTEM_BATT    = 0x0105,
    BLE_UUID_SYSTEM_SR      = 0x0106,
    BLE_UUID_SYSTEM_TP      = 0x0107
} system_char_uuid_enum;

/**
 * @brief This structure contains the handles for a System service
 */
typedef struct
{
    uint16_t conn_handle;       /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t service_handle;    /**< Handle of Our Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t serial_char_handle;
    ble_gatts_char_handles_t gatt_char_handle;
    ble_gatts_char_handles_t fw_char_handle;
    ble_gatts_char_handles_t time_char_handle;
    ble_gatts_char_handles_t batt_char_handle;
    ble_gatts_char_handles_t sr_char_handle;
    ble_gatts_char_handles_t tp_char_handle;
}ble_serv_system;

typedef struct{
    uint8_t *serial;
    uint8_t gatt_v[6];
    uint8_t firmware_v[6];
    uint64_t run_time;
    float batt;
    uint32_t sample_rate;
    uint32_t trans_period;
} system_service_data;

/**
 * @brief Function for initiating an System service. Should only be accessed by the ble utility, not the user application.
 * 
 * @param system_service system BLE service struct
 */
void ble_serv_system_init(ble_serv_system *system_service);

/**
 * @brief Used externally in the application code to update all System characteristics
 * 
 * @param system_service    system BLE service struct
 * @param new_data          struct of new system data
 */
void ble_characteristic_update_system(ble_serv_system *system_service, system_service_data *new_data);

/**
 * @brief Used to deactivate sensor data characteristics. Must be called prior to calling x_ble_init.
 * 
 */
void ble_system_data_char_off();

/**
 * @brief Used to deactivate sensor configuration characteristics. Must be called prior to calling x_ble_init.
 * 
 */
void ble_system_config_char_off();


#endif  /* BLE_SERVICE_SYSTEM_H */