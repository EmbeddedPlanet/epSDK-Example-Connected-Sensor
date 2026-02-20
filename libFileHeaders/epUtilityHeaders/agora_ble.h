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
 * @file    agora_ble.h
 * @version 1.0.0
 * @author  Embedded Planet, Inc.
 * @author  Dan Maher
 * @date    27 Oct 2022
 * 
 * @brief Interface for communication with Agora peripheral services & characteristics. For use with the EP BLE Central utility
 * 
 * Built for use with the nRF5 SDK 17.1 and FreeRTOS.
 * 
 * @see Error codes:    https://infocenter.nordicsemi.com/index.jsp?topic=%2Fsdk_nrf5_v17.1.0%2Fgroup__nrfx__error__codes.html
 */

#ifndef AGORA_BLE_H
#define AGORA_BLE_H

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "main.h"
#include "queue.h"
#include "ble_db_discovery.h"
#include "agora_ble_service_defs.h"
#include "ble_types.h"

#ifndef AGORA_CONNECTIONS_MAX
    #define AGORA_CONNECTIONS_MAX   6       /** < Number of Agora connections that agora_ble allows */
#endif

#ifndef AGORA_CHARS_MAX
    #define AGORA_CHARS_MAX         160     /** < Number of available characteristics on Agora */
#endif

#define BLE_TO_JSON                 0       /** < Thread that converts received BLE data to JSON packets and makes them available to
                                                the application via a queue. */
#define BLE_TO_EPCP                 1       /** < Thread that converts received BLE data to EPCP packets and makes them available to
                                                the application via a queue. */
#ifndef BLE_DATA_AGGREGATOR
    #warning BLE Data Aggregator not defined!
#endif

extern ble_uuid128_t agora_base_uuid;   /** < Holds the base UUID of the Agora */

#if BLE_DATA_AGGREGATOR == BLE_TO_JSON

    #ifndef BLE_JSON_QUEUE_SIZE
        #define BLE_JSON_QUEUE_SIZE     10                      /** < Maximum number of items that the queue can hold */
    #endif

    #ifndef BLE_JSON_QUEUE_TIMEOUT
        #define BLE_JSON_QUEUE_TIMEOUT          pdMS_TO_TICKS(3000)     /** Time to wait on queue before moving on to the next sample */
    #endif

    #ifndef BLE_JSON_AGGREGATION_INTERVAL
        #define BLE_JSON_AGGREGATION_INTERVAL   pdMS_TO_TICKS(540000)   /** Time between aggregating data and converting to JSON */
    #endif

    extern QueueHandle_t xBleJsonQueue;                         /** < Used to pass peripheral data to external modules in JSON format */

#endif

#if BLE_DATA_AGGREGATOR == BLE_TO_EPCP

    #ifndef BLE_EPCP_QUEUE_SIZE
        #define BLE_EPCP_QUEUE_SIZE     10                      /** < Maximum number of items that the queue can hold */
    #endif

    #ifndef BLE_EPCP_QUEUE_TIMEOUT
        #define BLE_EPCP_QUEUE_TIMEOUT          pdMS_TO_TICKS(3000)     /** Time to wait on queue before moving on to the next sample */
    #endif

    #ifndef BLE_EPCP_AGGREGATION_INTERVAL
        #define BLE_EPCP_AGGREGATION_INTERVAL   pdMS_TO_TICKS(60000)   /** Time between aggregating data and converting to EPCP */
    #endif

#endif

/**
 * @brief 
 * 
 */
typedef struct{
    char        *data;  /** < Pointer to the JSON string */
    uint16_t    size;   /** < Size of the JSON string */
} ble_json;

/**
 * @brief Used to hold callback function for processing a fetched characteristic's data.
 * 
 * @param data          A pointer to the new data
 * @param destination   A pointer to the destination
 * @param size          Size of array to be copied. Not always used.
 * 
 */
typedef void (*notif_process_func)(uint8_t * data, void * destination, uint8_t size);

/**
 * @brief Used for tracking which notif_process_func is to be used for which characteristic update, and where
 * the final value destination is. This information is registered when the char notify is registered in the notif_en function
 * 
 */
typedef struct{
    void *value;                        /** < Pointer to the characteristic value destination. Should be in a sensor data struct, such as bme680_ble_service_data */
    notif_process_func process_func;    /** < Pointer to the function that is used to process the retrieved characteristic data and store it */
    uint8_t key;                        /** < The CCCD/notification handle that is used to recognize the characteristic being updated */
    uint8_t size;                       /** < Size reference for arrays passed over BLE */
} agora_ble_char_map;

/**
 * @brief Struct that holds the connection and sensor data for a single connected Agora
 * 
 */
typedef struct{
    //Connection information
    bool conn_active;                               /** < True when Agora is connected, false when it is not */
    uint16_t conn_handle;                            /** < BLE connection handle assigned to this Agora */
    uint8_t char_map_add_index;                     /** < Current add location for new characteristic notification information */
    agora_ble_char_map char_map[AGORA_CHARS_MAX];   /** < Each characteristic that is configured to notify will have an entry in this array */

    //Sensor information
    system_ble_service_data system;                  /** < Holds the most recent system data */
    bme680_ble_service_data bme680;                 /** < Holds the most recent sensor data from the BME680 */
    icm20602_ble_service_data icm20602;             /** < Holds the most recent sensor data from the ICM20602 */
} agora_ble;

/**
 * @brief Array of connected Agoras to keep track of connection and sensor data
 * 
 */
extern agora_ble connected_agoras[AGORA_CONNECTIONS_MAX];

/**
 * @brief Sets Agora structs to defaults. 
 * To be called by the BLE Central utility prior to any other functions in this utility.
 * 
 */
void agora_ble_init();

/**
 * @brief Adds BLE connection information to an available Agora struct
 * 
 * @details To be called after the BLE Central utility detects that an Agora has just connected.
 * 
 * @param conn_handle The BLE connection handle of the connected Agora
 */
void agora_ble_add_connection(uint16_t conn_handle);

/**
 * @brief Removes BLE connection information from the disconnected Agora's struct
 * 
 * @details To be called after the BLE Central utility detects that an Agora has just disconnected
 * 
 * @param conn_handle The BLE connection handle of the disconnected Agora
 */
void agora_ble_rem_connection(uint16_t conn_handle);

/**
 * @brief Activates cccd notifications on the connected Agora for all data points.
 * 
 * @details To be called byt the BLE Central utility after database discovery on an Agora has been completed.
 * 
 * @param p_evt         The BLE discovery event that contains the discovered database data information
 * @param gatt_queue    The gatt_queue that the BLE Central utility is using to queue up gatt commands
 */
void agora_ble_notif_en(ble_db_discovery_evt_t * p_evt, nrf_ble_gq_t * gatt_queue);

/**
 * @brief Triggers the saved callback function for processing a specific Agora sensor data point.
 * 
 * @details To be called by the BLE Central utility after a notify / HVX event has occurred.
 * 
 * @param p_ble_evt The BLE event that is to be processed. Contains the hvx handle that is used to
 * find the proper callback function.
 */
void agora_ble_process_notif(ble_evt_t * p_ble_evt);

#endif