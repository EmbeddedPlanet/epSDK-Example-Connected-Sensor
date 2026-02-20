/****************************************************************************                                                                     *
 * Copyright (c) 2026 Embedded Planet, Inc.                                 *
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
 * Created on: Sept 1, 2022
 * Created by: maherdan
 * 
 * Copyright (c) Embedded Planet, Inc - All rights reserved
 *
 * This source file is private and confidential.
 * Unauthorized copying of this file is strictly prohibited.
 * 
 * Version 1.0 - 01SEPT22  Initial
 */
#ifndef __MAIN_H__
#define __MAIN_H__

/* *INDENT-OFF* */
#ifdef __cplusplus
    extern "C" {
#endif
/* *INDENT-ON* */

#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"

/* General defines */

/* Version needs to be defined as string and ints */
#define VERSION_NUM                         "00.00.03"                                   /**< Version. Will be passed to Device Information Service. */
#define VERSION_MAJOR                       0
#define VERSION_MINOR                       0
#define VERSION_BUILD                       3

/* Length of EP serial number is 6 characters */
#define SERIAL_LENGTH   6

/* Struct to hold system information */
typedef struct
{
    char updateVerStr[9];
    uint8_t updateVerMaj;
    uint8_t updateVerMin;
    uint16_t updateVerBui;
    uint8_t ep_serial[SERIAL_LENGTH + 1];
    char partNoStr[8];
    char hwIDStr[8];
} system_info_struct;
extern system_info_struct system_info;

typedef struct
{
    uint8_t  serialNumber[6];                    /* 8 byte serial number */
    uint32_t ulHardwareID;                       /* 32 bit ID that can be generated unique for a particular platform. */
    uint8_t  partNumber[6];                      /* 6 byte part number, ie. 800263 for P800000000263. */
    uint8_t  loraSubband;                        /* 8 bit lora sub-band */
    uint8_t  loraDevEUI[8];                      /* 8 byte app key for lora */
    uint8_t  loraJoinEUI[8];                     /* 8 byte app key for lora */
    uint8_t  loraAppKey[16];                     /* 16 byte app key for lora */
} ProductionTable_t;

/* OTA Defines */
#define otapal_FLASH_START               ( 0x1000 )
#define otapal_DESCRIPTOR_START          ( 0x0000 )
#define otapal_BANK_SIZE                 ( 0x90000 )         /* Flash size available */
#define otapal_BLE_TBL_START             ( 0x90000 )
#define otapal_PROD_TBL_START            ( 0xA0000 )

//////////////////////////////////////////////////
/****                                        ****/
/****               WATCHDOG                 ****/
/****                                        ****/
//////////////////////////////////////////////////

/* Watchdog reload value in s */
#define WATCHDOG_RELOAD         60

/* How often the watchdog should be fed in percent of WATCHDOG_RELOAD */
#define WATCHDOG_RELOAD_RATE    50

//////////////////////////////////////////////////
/****                                        ****/
/****               BLUETOOTH                ****/
/****                                        ****/
//////////////////////////////////////////////////

/**< Used to activate/deactivate BLE in the main application
 * 
 *   0 = Deactivated
 *   1 = Activated
 */
#define BLE_ACTIVE     1

/** < BLE TX power boost in dBm. For details, see sd_ble_gap_tx_power_set() in ble_gap.h */
#define BLE_TX_POWER_BOOST      8


///////////////////////////////////////
/**          BLE PERIPHERAL         **/
///////////////////////////////////////

/**< Name of device. Will be included in the advertising data. */
#define DEVICE_NAME_LOCAL "EP_DEV"

/**< Manufacturer. Will be passed to Device Information Service. */
#define MANUFACTURER_NAME   "Embedded_Planet_Inc"

/**< When activated, the peripheral will generate and serve up the BLE System Service via its GATT server
 * 
 *   0 = Deactivated
 *   1 = Activated
 */
#define BLE_SERVICE_SYSTEM      1

/**< When activated, the peripheral will generate and serve up the BLE CELL Service via its GATT server
 * 
 *   0 = Deactivated
 *   1 = Activated
 */
#define BLE_SERVICE_CELL        0

/**< When activated, the peripheral will generate and serve up the BLE BME680 Service via its GATT server
 * 
 *   0 = Deactivated
 *   1 = Activated
 */
#define BLE_SERVICE_BME680      0

/**< When activated, the peripheral will generate and serve up the BLE ICM20602 Service via its GATT server
 * 
 *   0 = Deactivated
 *   1 = Activated
 */
#define BLE_SERVICE_ICM20602    1

/** < Manufacturer ID. 0xFFFF only to be used during development */
#define MANUFACTURER_ID     0xFFFF

 /**< Device appearance to add to adv data. 0x0540 is generic sensor, see BLE spec for full list */
#define DEVICE_APPEARANCE   0x0540

/**< The advertising interval in units of 0.625 ms */
#define APP_ADV_INTERVAL    10000

/**< The advertising duration in units of 10 milliseconds. */
#define APP_ADV_DURATION    3000

/**< The wait time between advertising sessions, argument in milliseconds. */
#define APP_ADV_WAIT    pdMS_TO_TICKS(300000)

/**< When activated, the peripheral will advertise continiously when not connected to a central device.
 * 
 *   0 = Deactivated (perform intermittent advertising by following APP_ADV_DURATION and APP_ADV_WAIT)
 *   1 = Activated (perform continuous advertising by auto-restarting immediately after timeout)  
 */
#define APP_CONT_ADV    0

/**< Minimum acceptable connection interval */
#define MIN_CONN_INTERVAL   MSEC_TO_UNITS(100, UNIT_1_25_MS)

 /**< Maximum acceptable connection interval */
#define MAX_CONN_INTERVAL   MSEC_TO_UNITS(200, UNIT_1_25_MS)

/**< Slave latency. */
#define SLAVE_LATENCY   0

/**< Connection supervisory timeout (4 seconds). */
#define CONN_SUP_TIMEOUT    MSEC_TO_UNITS(4000, UNIT_10_MS)

/**< Time in ms from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  5000

/**< Time in ms between each call to sd_ble_gap_conn_param_update after the first call */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   30000

/**< Number of attempts before giving up the connection parameter negotiation. */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

//////////////////////////////////////////////////
/****                                        ****/
/****                  LED                   ****/
/****                                        ****/
//////////////////////////////////////////////////

/** < Amount of time, in milliseconds, that the LED remains on during slow blinks*/
#define LED_SLOW_BLINK_LENGTH               pdMS_TO_TICKS(1000)

/** < Amount of time, in milliseconds, between slow blinks */
#define LED_SLOW_BLINK_INTERVAL             pdMS_TO_TICKS(1000)

/** < Amount of time, in milliseconds, that the LED remains on during fast blinks */
#define LED_FAST_BLINK_LENGTH               pdMS_TO_TICKS(200)

/** < Amount of time, in milliseconds, between fast blinks */
#define LED_FAST_BLINK_INTERVAL             pdMS_TO_TICKS(200)

/** < Amount of time, in milliseconds, that the LED remains on during extra fast blinks */
#define LED_EXTRA_FAST_BLINK_LENGTH         pdMS_TO_TICKS(75)

/** < Amount of time, in milliseconds, between extra fast blinks */
#define LED_EXTRA_FAST_BLINK_INTERVAL       pdMS_TO_TICKS(75)

/** < Amount of time, in milliseconds, that the LED remains on during alive blinks */
#define LED_ALIVE_BLINK_LENGTH              pdMS_TO_TICKS(100)

/** < Amount of time, in milliseconds, between alive blinks */
#define LED_ALIVE_BLINK_INTERVAL            pdMS_TO_TICKS(15000)

/** < Amount of time, in milliseconds, that the LED remains on during multi-blink patterns */
#define LED_MULTI_BLINK_LENGTH              pdMS_TO_TICKS(135)

/** < Amount of time, in milliseconds, between slow multi-blinks */
#define LED_MULTI_BLINK_INTERVAL            pdMS_TO_TICKS(1000)

///////////////////////////////////////
/**         GLOBAL FUNCTIONS        **/
///////////////////////////////////////
/* Function for handling sensor power enable operation */
extern void sensorPwrEnConfig( bool status );

/* *INDENT-OFF* */
#ifdef __cplusplus
    }
#endif
/* *INDENT-ON* */

#endif /* __MAIN_H__ */
