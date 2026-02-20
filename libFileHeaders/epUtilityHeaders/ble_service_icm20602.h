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
 * @file    ble_service_icm20602.h
 * @version 1.0.0
 * @author  Embedded Planet, Inc.
 * @author  Dan Maher
 * @date    13 Oct 2022
 * 
 * @brief Contains functions for the configuration and control of the ICM20602 BLE service. For use with the ep_ble_peripheral utility.
 * 
 * @note Based on the nRF5 SDK BLE Peripheral Template example
 * 
 * Built for use with the nRF5 SDK 17.1 and FreeRTOS.
 * 
 * @see Error codes:    https://infocenter.nordicsemi.com/index.jsp?topic=%2Fsdk_nrf5_v17.1.0%2Fgroup__nrfx__error__codes.html
 */

#ifndef BLE_SERVICE_ICM20602_H
#define BLE_SERVICE_ICM20602_H

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "icm20602.h"

// ICM20602 service UUID
#define BLE_UUID_EP_BASE    {0xA7, 0x04, 0x62, 0x56, 0xD5, 0x10, 0x87, 0xB8, 0x91, 0x49, 0xAB, 0x99, 0x00, 0x00, 0x00, 0x00} // 128-bit base UUID
#define BLE_UUID_ICM20602_SERVICE 0x0800 // ICM20602 Service UUID

// ICM20602 characteristic UUIDs
typedef enum{
    BLE_UUID_ICM20602_TEMP      = 0x0801,
    BLE_UUID_ICM20602_ACCELX    = 0x0802,
    BLE_UUID_ICM20602_ACCELY    = 0x0803,
    BLE_UUID_ICM20602_ACCELZ    = 0x0804,
    BLE_UUID_ICM20602_GYROX     = 0x0805,
    BLE_UUID_ICM20602_GYROY     = 0x0806,
    BLE_UUID_ICM20602_GYROZ     = 0x0807,
} icm20602_char_uuid_enum;

/**
 * @brief This structure contains the handles for a ICM20602 service
 */
typedef struct
{
    uint16_t conn_handle;       /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t service_handle;    /**< Handle of Our Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t temp_char_handle;
    ble_gatts_char_handles_t accelx_char_handle;
    ble_gatts_char_handles_t accely_char_handle;
    ble_gatts_char_handles_t accelz_char_handle;
    ble_gatts_char_handles_t gyrox_char_handle;
    ble_gatts_char_handles_t gyroy_char_handle;
    ble_gatts_char_handles_t gyroz_char_handle;
}ble_serv_icm20602;

/**
 * @brief Function for initiating a icm20602 service. Should only be accessed by the ble utility, not the user application.
 * 
 * @param icm20602_service icm20602 BLE service struct
 */
void ble_serv_icm20602_init(ble_serv_icm20602 *icm20602_service);

/**
 * @brief Used externally in the application code to update all icm20602 characteristics
 * 
 * @param icm20602_service    icm20602 BLE service struct
 * @param new_data          struct of new icm20602 data
 */
void ble_characteristic_update_icm20602(ble_serv_icm20602 *icm20602_service, icm20602_sensor_data *new_data);

/**
 * @brief Used to deactivate sensor data characteristics. Must be called prior to calling x_ble_init.
 * 
 */
void ble_icm20602_data_char_off();

/**
 * @brief Used to deactivate sensor configuration characteristics. Must be called prior to calling x_ble_init.
 * 
 */
void ble_icm20602_config_char_off();


#endif  /* BLE_SERVICE_ICM20602_H */