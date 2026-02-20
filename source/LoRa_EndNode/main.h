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
 * Created by: golobmichael
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

#include "led_helper.h"
#include "LoRaWAN.h"

/* General defines */

/* Version needs to be defined as string and ints */
#define VERSION_NUM                         "00.00.04"                                   /**< Version. Will be passed to Device Information Service. */
#define VERSION_MAJOR                       0
#define VERSION_MINOR                       0
#define VERSION_BUILD                       4

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
/****            COMMUNICATION               ****/
/****                                        ****/
//////////////////////////////////////////////////

/* Enum used to determine the transmission communication paths. */
typedef enum e_comm_conf{
    COMM_NONE,
    COMM_CELL_ONLY,
    COMM_LORA_ONLY,
    COMM_CELL_PRIMARY_LORA_BACKUP,
    COMM_LORA_PRIMARY_CELL_BACKUP,
    COMM_CELL_PRIMARY_LORA_PRIMARY  
 } enum_comm_conf;

 /* Set in application to one of the above communciation configurations */
extern uint8_t comm_conf;

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
#define otapal_BANK_SIZE                 ( 0xA0000 )         /* Flash size available */
#define otapal_BLE_TBL_START             ( 0x90000 )
#define otapal_PROD_TBL_START            ( 0xA0000 )

//////////////////////////////////////////////////
/****                                        ****/
/****               LoRa                     ****/
/****                                        ****/
//////////////////////////////////////////////////

/**< Used to activate/deactivate LoRa in the main application, LoRa is always active on connected equipment
 * 
 *   0 = Deactivated
 *   1 = Activated
 */
#define LORA_ACTIVE     1

/* LoRa transmission interval in seconds */
#define MIN_TRANS_INTERVAL    60U

#define LORA_QUEUE_TIMEOUT                  pdMS_TO_TICKS(3000)                     /** Time to wait on queue before moving on to the next sample */

/* LoRa queue size for transmissions */
#define LORA_QUEUE_SIZE                     6

/**
 * @brief Default region is set to US915. Application can choose to configure a different region
 * by setting the appropirate compiler flag for the region and setting this config to the corresponding
 * region.
 */
#define LORAWAN_REGION    LORAMAC_REGION_US915


/**
 * @brief LoRa MAC layer port used by the application.
 * Downlink unicast messages should be send to this port number.
 */
#define LORAWAN_APP_PORT                       ( 15 )

/**
 * @brief Should send confirmed messages (with an acknowledgment) or not.
 */
#define LORAWAN_CONFIRMED_SEND                 ( 1 )

/**
 * @brief Defines a random jitter bound in milliseconds for application data transmission duty cycle.
 *
 * This allows devices to space their transmissions slighltly between each other in cases like all devices reboots and tries to
 * join server at same time.
 */
#define LORAWAN_APPLICATION_JITTER_MS          ( 500 )


/**
 * @brief Maximum time to wait to receive a downlink packet or event after sending an uplink packet.
 *
 * As per LoRaWAN spec, class A end device uses two receive windows slots after sending an uplink packet. For US915the max window duration is
 * 3000 ms and the second RX window max delay is 2 seconds. So setting the receive timeout to higher than the receive window slots.
 */
#define CLASSA_RECEIVE_WINDOW_DURATION_MS    ( 6000 )

/* LoRa transmission interval in seconds */
//#define MIN_TRANS_INTERVAL    60U

extern QueueHandle_t xLoraQueue;

/**
 * @brief Max join attempts
 * 
 */
#define lorawanConfigMAX_JOIN_ATTEMPTS  20

extern SemaphoreHandle_t xSPISemaphore;

extern void vLorawanClassATask( void * params );

/**
 * @brief Default subband for LoRa.
 */
static const uint8_t subbandDefault = 1;

/**
 * @brief Default device EUI needed for both OTAA and ABP activation.
 */
static const uint8_t devEUIDefault[ 8 ] = { 0x20, 0x21, 0x11, 0x02, 0x17, 0x95, 0x90, 0x02 };

/**
 * @brief Default join EUI needed for both OTAA and ABP activation.
 */
static const uint8_t joinEUIDefault[ 8 ] = { 0x99, 0xF1, 0x18, 0x7D, 0xC0, 0x07, 0x32, 0x87 };

/**
 * @brief Default app key required for OTAA activation.
 */
static const uint8_t appKeyDefault[ 16 ] = { 0x89, 0xF1, 0x18, 0x7D, 0xC0, 0x07, 0x32, 0x87, 0x71, 0xE5, 0x74, 0xFD, 0xF7, 0xE7, 0x69, 0x99 };

/**
 * @brief App session key required for ABP activation.
 */
//static const uint8_t appSessionKey[ 16 ] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/**
 * @brief Network Session key required for ABP activation.
 */
//static const uint8_t nwkSessionKey[ 16 ] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

//////////////////////////////////////////////////
/****                                        ****/
/****                  LED                   ****/
/****                                        ****/
//////////////////////////////////////////////////

#define LED_CELL_OK             LED_SINGLE_BLINK
#define LED_FAIL_TO_SEND        LED_DOUBLE_BLINK
#define LED_FAIL_TO_REG         LED_TRIPLE_BLINK
#define ENABLE_LED_OPERATION    true

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