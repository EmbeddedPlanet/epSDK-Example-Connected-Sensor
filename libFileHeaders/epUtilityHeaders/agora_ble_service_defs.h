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
 * @file    agora_ble_service_defs.h
 * @version 1.0.0
 * @author  Embedded Planet, Inc.
 * @author  Dan Maher
 * @date    27 Oct 2022
 * 
 * @brief Contains UUIDs of services/characteristics as well as structs to hold retrieved sensor data. For use with the EP BLE Central utility
 * 
 * Built for use with the nRF5 SDK 17.1 and FreeRTOS.
 * 
 * @see Error codes:    https://infocenter.nordicsemi.com/index.jsp?topic=%2Fsdk_nrf5_v17.1.0%2Fgroup__nrfx__error__codes.html
 */

#ifndef AGORA_BLE_DEFS_H
#define AGORA_BLE_DEFS_H

#include <stdint.h>

#define SYSTEM_SERVICE_SERIAL_SIZE  7   /** < Size of the serial number string */
#define SYSTEM_SERVICE_GATT_SIZE    7   /** < Size of the gatt version number string */
#define SYSTEM_SERVICE_FW_SIZE      7   /** < Size of the firmware version number string */

/**
 * @brief System service characteristic UUIDs
 * 
 */
typedef enum{
    SYSTEM_BASE_UUID        = 0x0100,
    SYSTEM_BASE_UUID_MASK   = 0x0F00,
    SYSTEM_SERIAL_UUID      = 0x0101,
    SYSTEM_GATT_UUID        = 0x0102,
    SYSTEM_FW_UUID          = 0x0103,
    SYSTEM_TIME_UUID        = 0x0104,
    SYSTEM_BATT_UUID        = 0x0105,
    SYSTEM_SR_UUID          = 0x0106,
    SYSTEM_TP_UUID          = 0x0107
} system_ble_char_uuids;

/**
 * @brief Received data from the System service
 */
typedef struct
{
    char serial[SYSTEM_SERVICE_SERIAL_SIZE];    /** < Agora serial number (6 characters, no termination character) */
    char gatt_v[SYSTEM_SERVICE_GATT_SIZE];      /** < Gatt version that connected agora is using
                                                    gatt_v[0-1] = MAJOR
                                                    gatt_v[2-3] = MINOR
                                                    gatt_v[4-5] = PATCH  */
    char firmware_v[SYSTEM_SERVICE_FW_SIZE];    /** < Firmware version that connected agora is running
                                                    firmware_v[0-1] = MAJOR
                                                    firmware_v[2-3] = MINOR
                                                    firmware_v[4-5] = PATCH  */
    uint64_t run_time;                          /** < Real time or system run time */
    float batt;                                 /** < Battery voltage in volts */
    uint32_t sample_rate;                       /** < Sample rate */
    uint32_t trans_period;                      /** < Transmission period */
}system_ble_service_data;

/**
 * @brief BME680 service characteristic UUIDs
 * 
 */
typedef enum{
    BME680_BASE_UUID            = 0x0700,
    BME680_BASE_UUID_MASK       = 0x0F00,
    BME680_TEMP_UUID            = 0x0701,
    BME680_PRES_UUID            = 0x0702,
    BME680_HUM_UUID             = 0x0703,
    BME680_GAS_UUID             = 0x0704,
    BME680_CO2_UUID             = 0x0705,
    BME680_BREATH_UUID          = 0x0706,
    BME680_IAQ_SCORE_UUID       = 0x0707,
    BME680_IAQ_ACCURACY_UUID    = 0x0708
} bme680_ble_char_uuids;

/**
 * @brief Received data from the BME680 sensor service
 * 
 */
typedef struct{
    float temperature;                  /** < Temperature in degrees C, adjusted to compensate for the onboard heater element  */
    float pressure;                     /** < Unadjusted pressure in Pa */
    float humidity;                     /** < Humidity in %, adjusted to compensate for the onboard heater element*/
    float gas;                          /** < Percentage of min and max filtered gas */
    float co2;                          /** < co2 equivalent estimate in ppm */   
    float breath;                       /** < Breath VOC concentration estimate in ppm */
    float iaq_score;                    /** < Indoor air quality score (optimized for mobile applications). Range is 0-500  */
    uint8_t iaq_accuracy;               /** < Accuracy of the iaq score. Range is 0(unreliable) - 3(high accuracy)  */
}bme680_ble_service_data;

/**
 * @brief ICM20602 service characteristic UUIDs
 * 
 */
typedef enum{
    ICM20602_BASE_UUID      = 0x0800,
    ICM20602_BASE_UUID_MASK = 0x0F00,
    ICM20602_TEMP_UUID      = 0x0801,
    ICM20602_ACCELX_UUID    = 0x0802,
    ICM20602_ACCELY_UUID    = 0x0803,
    ICM20602_ACCELZ_UUID    = 0x0804,
    ICM20602_GYROX_UUID     = 0x0805,
    ICM20602_GYROY_UUID     = 0x0806,
    ICM20602_GYROZ_UUID     = 0x0807,
} icm20602_char_uuid_enum;

/**
 * @brief Received data from the ICM20602 sensor service
 * 
 */
typedef struct{
    float temperature;  /** < Temperature in degrees C. */
    float accel_x;      /** < Acceleration on x-axis in gs. */
    float accel_y;      /** < Acceleration on y-axis in gs. */
    float accel_z;      /** < Acceleration on z-axis in gs. */
    float gyro_x;       /** < Angular velocity on x-axis in dps. */
    float gyro_y;       /** < Angular velocity on y-axis in dps. */
    float gyro_z;       /** < Angular velocity on z-axis in dps. */
} icm20602_ble_service_data;

#endif