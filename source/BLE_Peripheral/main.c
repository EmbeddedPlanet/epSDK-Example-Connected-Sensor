/****************************************************************************     
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
 * @file    example.c
 * @version 1.1.0
 * @author  Embedded Planet, Inc.
 * @author  Dan Maher
 * @date    13 Oct 2022
 * 
 * @brief Example BLE peripheral program that uses the ep_ble_peripheral api
 * 
 * Built for use with the nRF5 SDK 17.1 and FreeRTOS.
 * 
 * @see Error codes:    https://infocenter.nordicsemi.com/index.jsp?topic=%2Fsdk_nrf5_v17.1.0%2Fgroup__nrfx__error__codes.html
 */

#include "main.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"
#include "app_error.h"
#include <stdio.h>
#include <stdlib.h>
#include "nrf.h"
#include "boards.h"
#include "nrfx_twi.h"
#include "nrfx_clock.h"
#include "nrf_log.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "ep_bsp.h"
#include "uart_helper.h"
#include "led_helper.h"
#include "ep_ble_peripheral.h"
#include "qspi_helper.h"
#include "cell_helper.h"
#include "bme680.h"
#include "icm20602.h"

#include "compact_payload_config.h"
#include "epcp_builder_agora.h"

/* Firmware version information */
#define VERSION_NUM "00.00.03"
#define MAJOR_FW    "00"
#define MINOR_FW    "00"
#define PATCH_FW    "03"

/* GATT Spec version information */
#define MAJOR_GATT  "00"
#define MINOR_GATT  "01"
#define PATCH_GATT  "01"

/* Timer for sampling sensors and publishing over BLE in ms*/
TimerHandle_t xSampleTimer;
#define SENSOR_SAMPLE_INTERVAL pdMS_TO_TICKS(60000)
#define SEMAPHORE_WAIT_LIMIT pdMS_TO_TICKS(65000)

/* Semaphore to indicate that the sample timer has fired */
SemaphoreHandle_t xSampleTimerTriggeredSemaphore;

/* Empty ICM struct */
ICM20602 icm;

system_info_struct system_info;

/* TWI instance ID. */
#define TWI_INSTANCE_ID 0

/* TWI instance. */
static const nrfx_twi_t twi = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID);

system_info_struct system_info;

/* TWI initialization */
void twi_init (void)
{
    nrfx_err_t err_code;

    const nrfx_twi_config_t twi_lm75b_config = {
       .scl                = PIN_NAME_SCL,
       .sda                = PIN_NAME_SDA,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .hold_bus_uninit     = false
    };

    err_code = nrfx_twi_init(&twi, &twi_lm75b_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrfx_twi_enable(&twi);
}

/* Fires every SENSOR_SAMPLE_INTERVAL to give a semaphore to the app_loop to continue */
void sensor_sample_callback(){
    xSemaphoreGive(xSampleTimerTriggeredSemaphore);
}

/* Waits for the semaphore that is given by sensor_sample_callback. Retrieves latest sensor data and posts via BLE */
void app_loop(void *pvParameters){
    init_uart(TASK_1);
    DBGI("******************************************************");
    DBGI("*       Embedded Planet: BLE PERIPHERAL v%s    *", VERSION_NUM);
    DBGI("******************************************************");
    uninit_uart(TASK_1);
    while(1){
        uint8_t timer_triggered = xSemaphoreTake(xSampleTimerTriggeredSemaphore, SEMAPHORE_WAIT_LIMIT);
        init_uart(TASK_1);
        if(timer_triggered){
            nrfx_err_t err;

            DBGI("Device %s updating BLE characteristics...\r\n", system_info.ep_serial);

            #if BLE_SERVICE_SYSTEM
            //Update system information
            system_service_data system_data;
            system_data.serial = system_info.ep_serial;
            memcpy(&system_data.gatt_v[0], MAJOR_GATT, sizeof(MAJOR_GATT));
            memcpy(&system_data.gatt_v[2], MINOR_GATT, sizeof(MINOR_GATT));
            memcpy(&system_data.gatt_v[4], PATCH_GATT, sizeof(PATCH_GATT));
            memcpy(&system_data.firmware_v[0], MAJOR_FW, sizeof(MAJOR_FW));
            memcpy(&system_data.firmware_v[2], MINOR_FW, sizeof(MINOR_FW));
            memcpy(&system_data.firmware_v[4], PATCH_FW, sizeof(PATCH_FW));
            system_data.batt = ep_bsp_read_battery_voltage();
            ble_characteristic_update_system(&system_service, &system_data);
            #endif

            #if BLE_SERVICE_BME680
            //Get new BME680 data if available
            if(xSemaphoreTake(xBme680DataReadySemaphore, 0)){
                bme680_sensor_data bme_data = bme680_get_latest_data();
                ble_characteristic_update_bme680(&bme680_service, &bme_data); //Update BLE characteristics for sensor
            }else{
                printf("BME680 data not available\r\n");
            }
            #endif

            #if BLE_SERVICE_ICM20602
            //Get new ICM20602 data if available
            bool icm_data_ready = icm20602_data_ready(&icm, &err);
            if(icm_data_ready){
                icm20602_sensor_data icm_data = icm20602_get_data(&icm, &err); 
                ble_characteristic_update_icm20602(&icm20602_service, &icm_data); //Update BLE characteristics for sensor
            }else{
                DBGE("icm20602 err: %x\r\n", err);
            }
            #endif

            //Used to copy signed values to the uint8_t array without changing the value
            union epcp_convert_type ct;

            const uint8_t adv_data_accelx_indx = 0;
            const uint8_t adv_data_accely_indx = 2;
            const uint8_t adv_data_accelz_indx = 4;
            uint8_t adv_data[6] = {0xFF};

            //Get accel/gyro data and add to adv_data
            icm20602_sensor_data icm_data;
            icm_data_ready = icm20602_data_ready(&icm, &err);
            if(icm_data_ready){
                icm_data = icm20602_get_data(&icm, &err);
                //Convert and add accel data 
                ct.i32 = icm_data.accel_x * 100;
                adv_data[adv_data_accelx_indx] = ct.i32 >> 8;
                adv_data[adv_data_accelx_indx + 1] = ct.i32;
                ct.i32 = icm_data.accel_y * 100;
                adv_data[adv_data_accely_indx] = ct.i32 >> 8;
                adv_data[adv_data_accely_indx + 1] = ct.i32;
                ct.i32 = icm_data.accel_z * 100;
                adv_data[adv_data_accelz_indx] = ct.i32 >> 8;
                adv_data[adv_data_accelz_indx + 1] = ct.i32;
            }else{
                DBGE("icm20602 err: %x\r\n", err);
            }

            DBGI("ax: %f, ay: %f, az: %f\r\n", icm_data.accel_x, icm_data.accel_y, icm_data.accel_z);
            
            ep_ble_beacon_update(adv_data, system_info.ep_serial);
           
        }else{
            DBGE("Semaphore timeout\r\n");
        }
        uninit_uart(TASK_1);
    }
}

/* Function for application main entry */
int main(void){
    nrfx_err_t err_code = NRF_SUCCESS;

    // Initialize bsp
    ep_bsp_init(WATCHDOG_RELOAD, WATCHDOG_RELOAD_RATE);
    init_uart(MAIN_LOOP);
    //Init uart for program output
    uart_helper.dbgi = true;

    //Init LED to follow BLE advertising and button long push to restart the system
    led_init();

    /* Check for version number in qpsi, otherwise use default */
    ImageDescriptor_t xDescriptor;
    ProductionTable_t xProdTable;

     /* Initialize qspi driver.*/
    err_code = qspi_init();

    if( err_code != NRFX_SUCCESS )
    {
        DBGE( "Unable to initialize QSPI driver" );
    }

    nrf_delay_ms(pdMS_TO_TICKS(2500));

    err_code = qspi_read( (uint8_t * ) &xDescriptor, sizeof(xDescriptor), 0 );
    if( err_code != NRFX_SUCCESS )
    {
        DBGE( "Read failed with with error code %d", err_code );
    }

    if( xDescriptor.usImageFlags == otapalIMAGE_FLAG_VALID )
    {
        DBGI("Image Flag:%d",xDescriptor.usImageFlags);
        DBGI("Image Checksum:%ld", xDescriptor.checksum);

        system_info.updateVerMaj = ( ( xDescriptor.updateVersion >> 24 ) & 0xFF );
        system_info.updateVerMin = ( ( xDescriptor.updateVersion >> 16 ) & 0xFF );
        system_info.updateVerBui = ( xDescriptor.updateVersion & 0xFFFF );
        sprintf( system_info.updateVerStr,"%02d.%02d.%02ld", system_info.updateVerMaj, system_info.updateVerMin, system_info.updateVerBui );
    }
    else
    {
        /* Set integer version */
        system_info.updateVerMaj = VERSION_MAJOR;
        system_info.updateVerMin = VERSION_MINOR;
        system_info.updateVerBui = VERSION_BUILD;
        /* Set string version */
        sprintf( system_info.updateVerStr,"%02d.%02d.%02ld", system_info.updateVerMaj, system_info.updateVerMin, system_info.updateVerBui );
    }

    /* Read production table in external flash */
    err_code = qspi_read( (uint8_t * ) &xProdTable, sizeof(xProdTable), otapal_PROD_TBL_START );
    if( err_code != NRFX_SUCCESS )
    {
        DBGE( "Prod Table read failed with with error code %d", err_code );
    }
    else
    {
        /* Use production table values in external flash */
        if( strncmp( xProdTable.serialNumber, "\xFF\xFF\xFF\xFF\xFF\xFF", sizeof( xProdTable.serialNumber ) ) != 0 )
        {
            memcpy( system_info.ep_serial, xProdTable.serialNumber, sizeof( xProdTable.serialNumber ) );
            DBGI("Device SN: %s", system_info.ep_serial);
        }
        /* Use default production table values, if needed */
        else
        {
            DBGE("Invalid Production Table, Programming Defaults");
        }
    }

    /* Prior to BLE initialization, you can deactivate data or config characteristics for a specific service. For example,
    we want to deactivate system service configuration characteristics: */
    ble_system_config_char_off();
    /* All sensor characteristics default to active, so these do not need to be called unless you want to leave something
    out of the BLE service. */

    // BLE Initialization. 
    ep_ble_peripheral_init(DEVICE_NAME_LOCAL, strlen(DEVICE_NAME_LOCAL), system_info.ep_serial);

    //Enable sensor power
    nrf_gpio_cfg_output(PIN_NAME_SENSOR_POWER_ENABLE);
    nrf_gpio_pin_set(PIN_NAME_SENSOR_POWER_ENABLE);

    //Init nRF TWI interface
    twi_init();
    //Small delay needed to allow time for twi to set up
    nrf_delay_ms(20);

    // Sensor Initialize
    bme680_init(twi, BSEC_SAMPLE_RATE_LP, 0.0f);
    icm20602_init(&icm, ICM20602_ADDR_LOW, twi);

    /* Set up timer for reading sensors and posting via BLE */
    xSampleTimer = xTimerCreate("sample_timer", SENSOR_SAMPLE_INTERVAL, pdTRUE, 0, sensor_sample_callback);
    if(xSampleTimer){
        xTimerStart(xSampleTimer, 0);
    }else{
        printf("timer creation failed\r\n");
        return 0;
    }

    /* Set up semaphore to indicate if sample timer has fired */
    xSampleTimerTriggeredSemaphore = xSemaphoreCreateBinary();
    if(!xSampleTimerTriggeredSemaphore){
        printf("semaphore creation failed\r\n");
        return 0;
    }

    /* The app_loop task handles sensor data processing and posting over BLE */
    xTaskCreate(app_loop, "app_loop", 2000, NULL, 1, NULL);

    //uninit_uart(MAIN_LOOP);
    vTaskStartScheduler();

    /* Should never reach here */
    while(1);
}