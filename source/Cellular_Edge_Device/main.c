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
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
// FreeRTOS Includes
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "queue.h"
// Nordic Includes
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_freertos.h"
#include "nrf_delay.h"
// Project Includes
#include "main.h"
#include "ep_bsp.h"
#include "uart_helper.h"
#include "cell_helper.h"
//#include "ep_ble_central.h"
#include "htu21d.h"
#include "vl53l0x.h"
#include "bme680.h"
#include "icm20602.h"
#include "qspi_helper.h"
#include "led_helper.h"
#include "time_helper.h"
#include "compact_payload.h"
#if defined(THINGSBOARD_HTTPS_INTEGRATION) || defined(AZURE_MQTTS_X509) || defined(AWS_MQTTS_X509) || defined(AWS_HTTPS_X509) || defined(AZURE_MQTTS_SAS) || defined(AZURE_HTTPS_SAS)
#include "nrf_crypto.h"
#endif

//Activate / deactivate onboard sensors
#define SI7021_ACTIVE   1
#define VL53L0X_ACTIVE  0
#define BME680_ACTIVE   0
#define ICM20602_ACTIVE 1

#define mainLED_TASK_STACK_SIZE             128
#define mainSAMPLE_TASK_STACK_SIZE          1024
#define mainCell_TASK_STACK_SIZE            8196
#define CELL_QUEUE_TIMEOUT                  pdMS_TO_TICKS(3000)                     /** Time to wait on queue before moving on to the next sample */

// Declare FreeRTOS tasks, queues, and semaphore
SemaphoreHandle_t cellularSemaphore;
SemaphoreHandle_t dataReadySemaphore;
SemaphoreHandle_t xSensPwrEnSemaphore;
QueueHandle_t xCellQueue;
TaskHandle_t ledTaskHandle;
TaskHandle_t sensorSampleTaskHandle;
TaskHandle_t cellularTaskHandle;

system_info_struct system_info;

// Frame counter for payload
static uint8_t frameCounter = 0;

//Sets the comm path configuration, Cell enabled by default
uint8_t comm_conf = COMM_CELL_ONLY;

uint32_t cellTransInt = MIN_TRANS_INTERVAL;

#if ICM20602_ACTIVE
/* Empty ICM struct */
ICM20602 icm;
#endif

/* TWI instance ID. */
#define TWI_INSTANCE_ID 0

/* TWI instance. */
static const nrfx_twi_t twi = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Initial PDP config */
CellularBLEConfig_t pdnConfig = { '\0' };

//Save EPCP version locally
uint8_t epcp_ver = EPCP_VERSION;

/* Task for reading local sensors and adding to cell data queue */
void sensorSampleTask(void *pvParameters);

/* Miscellaneous initialization including preparing the logging and cell. */
static void prvMiscInitialization( void );

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

/*-----------------------------------------------------------*/
static void LEDTask( void * pvParameters )
{
    led_init();

    for(;;)
    {
        if( cellStatus == CellSuccess )
        {
            led_mode(LED_CELL_OK, true, 1);
        }
        else if( cellStatus == CellFailToSend || cellStatus == CellFailToRec )
        {
            led_mode(LED_FAIL_TO_SEND, true, 1);
        }
        else
        {
            led_mode(LED_FAIL_TO_REG, true, 1);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    vTaskDelete( NULL );
}

/*-----------------------------------------------------------*/
void sensorPwrEnConfig( bool status )
{
    static uint8_t counter = 0;
    static bool init = true;

    //If first pass then configure to output and create semaphore
    if(init)
    {
        //Create semaphore
        xSensPwrEnSemaphore = xSemaphoreCreateMutex();
        if(xSensPwrEnSemaphore == NULL){
            DBGE("PwrEn semaphore creation failure!");
        }

        //Give semaphore
        xSemaphoreGive(xSensPwrEnSemaphore);

        //Configure output
        nrf_gpio_cfg_output(PIN_NAME_SENSOR_POWER_ENABLE);

        //Clear init flag
        init = false;
    }

    //Try to take semaphore
    if(xSemaphoreTake(xSensPwrEnSemaphore, pdMS_TO_TICKS(500)) == pdTRUE){
        //If status is false, and all tasks want this pin disabled, then clear pin
        if(status == false)
        {
            //Decrement counter if above zero
            if(counter > 0)
            {
                counter--;
            }
            //If counter is now 0, then disable
            if(counter == 0)
            {
                nrf_gpio_pin_clear(PIN_NAME_SENSOR_POWER_ENABLE);
            }
        }

        //If status is true then set pin and increase counter
        if(status == true)
        {
            nrf_gpio_pin_set(PIN_NAME_SENSOR_POWER_ENABLE);
            if(counter < 0xFF)
            {
                counter++;
            }
        }

        //Give semaphore
        xSemaphoreGive(xSensPwrEnSemaphore);
    }
}

/*-----------------------------------------------------------*/

/**@brief Function for application main entry.
 */
int main(void)
{
    /* Miscellaneous initialization including preparing the logging and cell */
    prvMiscInitialization();

    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    for (;;)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}

/*-----------------------------------------------------------*/
// Taken from FreeRTOS demo: https://github.com/FreeRTOS/FreeRTOS/blob/main/FreeRTOS-Plus/Demo/FreeRTOS_Cellular_Interface_Windows_Simulator/Common/main.c
static void prvMiscInitialization( void )
{
    nrfx_err_t err_code = NRFX_SUCCESS;

    // Initialize bsp
    ep_bsp_init( WATCHDOG_RELOAD, WATCHDOG_RELOAD_RATE );

    // Init UART
    init_uart(MAIN_LOOP);
    uart_helper.dbgi = true;

    set_time(0);
    get_time_s();

    // Initialize LEDs, default to alive blink normal operation
    led_init();
    led_mode(LED_ALIVE_BLINK, 1, true);    

    // Check for version number in qpsi, otherwise use default
    ImageDescriptor_t xDescriptor;
    ProductionTable_t xProdTable;

    // Initialize qspi driver
    err_code = qspi_init();

    if( err_code != NRFX_SUCCESS )
    {
        DBGE( "Unable to initialize QSPI driver" );
    }

    nrf_delay_ms(2500);

    err_code = qspi_read( (uint8_t * ) &xDescriptor, sizeof(xDescriptor), 0 );
    if( err_code != NRFX_SUCCESS )
    {
        DBGE( "Read failed with with error code %d", err_code );
    }

    if( xDescriptor.usImageFlags == otapalIMAGE_FLAG_VALID )
    {
        DBGI("Image Flag: 0x%X",xDescriptor.usImageFlags);
        DBGI("Image Checksum: 0x%X", xDescriptor.checksum);
        DBGI("Image Version: 0x%X", xDescriptor.updateVersion);

        system_info.updateVerMaj = ( ( xDescriptor.updateVersion >> 24 ) & 0xFF );
        system_info.updateVerMin = ( ( xDescriptor.updateVersion >> 16 ) & 0xFF );
        system_info.updateVerBui = ( xDescriptor.updateVersion & 0xFFFF );
        sprintf( system_info.updateVerStr,"%02x.%02x.%02x", system_info.updateVerMaj, system_info.updateVerMin, system_info.updateVerBui );        
    }
    else
    {
        // Set integer version
        system_info.updateVerMaj = VERSION_MAJOR;
        system_info.updateVerMin = VERSION_MINOR;
        system_info.updateVerBui = VERSION_BUILD;
        // Set string version
        sprintf( system_info.updateVerStr,"%02d.%02d.%02d", system_info.updateVerMaj, system_info.updateVerMin, system_info.updateVerBui );
    }

    // Read production table in external flash
    err_code = qspi_read( (uint8_t * ) &xProdTable, sizeof(xProdTable), otapal_PROD_TBL_START );
    if( err_code != NRFX_SUCCESS )
    {
        DBGE( "Prod Table read failed with with error code %d", err_code );
    }
    else
    {
        // Use production table values in external flash
        if( strncmp( xProdTable.serialNumber, "\xFF\xFF\xFF\xFF\xFF\xFF", sizeof( xProdTable.serialNumber ) ) != 0 )
        {
            memcpy( system_info.ep_serial, xProdTable.serialNumber, sizeof( xProdTable.serialNumber ) );
            system_info.ep_serial[SERIAL_LENGTH] = '\0';
            DBGI("Device SN: %s", system_info.ep_serial);
        }
        // Use default production table values, if needed
        else
        {
            DBGE("Invalid Production Table, Programming Defaults");
        }
    }

    qspi_uninit();

    DBGI("******************************************************************");
    DBGI("* Embedded Planet CONNECTED SENSOR FreeRTOS Example v%s *", system_info.updateVerStr);
    DBGI("******************************************************************");   

    // Task to read local sensor data and forward it to the cell queue
    xTaskCreate(sensorSampleTask, 
                "SensorTask", 
                mainSAMPLE_TASK_STACK_SIZE, 
                NULL, 
                tskIDLE_PRIORITY+2,
                &sensorSampleTaskHandle);

#if ENABLE_LED_OPERATION
    // LED control task
    xTaskCreate( LEDTask,
                "LEDTask",
                mainLED_TASK_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY+1,
                &ledTaskHandle );
#endif

    /* Cell needs a couple of seconds before starting */
    nrf_delay_ms(2000);

    DBGI( ( "Cell Task Initializing...\r\n" ) );

    cellularSemaphore = xSemaphoreCreateBinary();
    if(!cellularSemaphore){
        DBGE("cellularSemaphore creation failed!");
    }

    //Setup cellular data queue
    xCellQueue = xQueueCreate(CELL_QUEUE_SIZE, sizeof(cell_queue_msg));
    if(xCellQueue == NULL){
        DBGE("xCellQueue creation failed!");
    }

    #if defined(THINGSBOARD_HTTPS_INTEGRATION) || defined(AZURE_MQTTS_X509) || defined(AWS_MQTTS_X509) || defined(AWS_HTTPS_X509) || defined(AZURE_MQTTS_SAS) || defined(AZURE_HTTPS_SAS)
    //If protocol is MQTT, then initialize crypto here
    if( nrf_crypto_init() != NRF_SUCCESS)
    {
        DBGE(("Failed to initialize nrf crypto"));
    }
    #endif

    /* Create the task to run tests. */
    xTaskCreate( CellularTask,
                "CellularTask",
                mainCell_TASK_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY+2,
                NULL );  

    uninit_uart(MAIN_LOOP); //comment out to prevent sleep
}

/*-----------------------------------------------------------*/
void parse_downlink(void){
    // String for get app attributes
    char sharedAppAttr[MAX_ATTR_VALUE_LENGTH] = {'\0'};
    for( uint8_t attribute = 0; attribute < num_HTTP_ATTRIBUTES; attribute++ )
    {
        getAttributes(attribute,sharedAppAttr);
        // Check if attribute value is valid
        if(sharedAppAttr[0] != '\0')
        {
            //DBGE("%d, %s", attribute, sharedAppAttr);
            vTaskDelay(pdMS_TO_TICKS(10));
            switch(attribute){

                // Handle transmission interval update
                case 0:
                {
                    cellTransInt = ( uint32_t ) strtoul( sharedAppAttr, NULL, 10 );
                    DBGI("New sample interval: %ds", cellTransInt);
                }break;

                default:
                {
                    //DBGW("Undefined attribute!");
                }
            }
        }
    }
}

/*-----------------------------------------------------------*/
/* Waits for the semaphore that is given by sensor_sample_callback. Retrieves latest local sensor data and adds to cell queue */
void sensorSampleTask(void *pvParameters)
{
     nrfx_err_t err;
    
    while(1){
        init_uart(TASK_1);                

        //Init nRF TWI interface
        twi_init();

        //Small delay needed after twi/spi init before communication can begin
        vTaskDelay(pdMS_TO_TICKS(20));
        
        //Enable sensor power        
        DBGI("Sensor power on");
        sensorPwrEnConfig(true);
        vTaskDelay(pdMS_TO_TICKS(250));

        DBGI("FreeRTOS Heap Space: %ld",xPortGetFreeHeapSize());

        //Get latest cell diagnostic values
        cellDiag parameters = {'\0'};
        if( queryCellularDiag(&parameters) != CellSuccess )
        {
            /* Keep trying if unable to attain values */
            continue;
        }

        //Loop and give time for IMEI during power up
        uint8_t loopCnt = 0;
        DBGI("Waiting for IMEI to be received");
        while( parameters.imei[0] == 0 && parameters.imei[15] == 0 )
        {
            vTaskDelay(pdMS_TO_TICKS(500));
            queryCellularDiag(&parameters);
            loopCnt++;
            if(loopCnt > 240)
            {
                break;
            }
        }
        //Cannot send data on this loop if IMEI is unavailable
        if(loopCnt > 240)
        {
            DBGE("No IMEI to receive");
            // Clear pin configurations to prevent back-powering (High-Impedance / No Pull)
            nrfx_twi_uninit(&twi);
            nrf_gpio_cfg_default(PIN_NAME_SCL);
            nrf_gpio_cfg_default(PIN_NAME_SDA);
            uninit_uart(TASK_1);
            sensorPwrEnConfig(false);        
            vTaskDelay(pdMS_TO_TICKS(cellTransInt * 1000));
            // Force new cell imei attempt
            newDataAdded = true;
            continue;
        }
        DBGI("IMEI received");

        #if ICM20602_ACTIVE == 1
        icm20602_init(&icm, ICM20602_ADDR_LOW, twi);
        #endif
        
        //Get current time. In seconds if we have already synced time via cellular or GNSS. If not, this represents the run time of the system.
        uint32_t ts = get_time_s();

        //Increment frame counter or roll over if needed
        frameCounter = (uint8_t) (frameCounter + 1);

        //Get SI7021 data if active and available
        float si_temp, si_hum;
        #if SI7021_ACTIVE == 1
        htu21_init(twi);
        err = htu21_is_connected();
        if(err != NRFX_SUCCESS){
            DBGW("SI7021 data not available!");
        }
        htu21_read_temperature_and_relative_humidity(&si_temp, &si_hum);
        #endif

        //Get BME680 data if active and available
        bme680_sensor_data bme_data; 
        #if BME680_ACTIVE == 1
        bme680_init(twi, BSEC_SAMPLE_RATE_LP, 0.0f);
        //Get new BME680 data if available
        if(xSemaphoreTake(xBme680DataReadySemaphore, pdMS_TO_TICKS(5000))){
            bme_data = bme680_get_latest_data();
        }else{
            DBGI("BME680 data not available!");
        }
        vTaskDelete(bme_thread_data);
        #endif

        //Get icm20602 data if active and available
        icm20602_sensor_data icm_data;
        int errFlag;
        #if ICM20602_ACTIVE == 1
        //Get new ICM20602 data if available
        bool icm_data_ready = icm20602_data_ready(&icm, &errFlag);
        if(icm_data_ready){
            icm_data = icm20602_get_data(&icm, &errFlag); 
        }else{
            DBGW("ICM20602 data not available!");
        }
        #endif

        //Get VLXL0X data if active and available
        VL53L0X tof;
        #if VL53L0X_ACTIVE == 1
        errFlag = vl53l0x_init(&tof, twi);
        if(errFlag != NRFX_SUCCESS){
            DBGE("VL53L0X init error!");
        }
        //Set the tof timing budget to 200ms
        setMeasurementTimingBudget(&tof, 200000);
        #endif

        //Set up data converter
        union epcp_convert_type ct;

        //Set up compact payload
        compact_payload_init();

        //Add System data to subpacket
        //Convert battery from float to int per spec
        ct.ui32 = ep_bsp_read_battery_voltage() * 100;
        if(frameCounter == 1){
            compact_payload_add_data(EPCP_SYSTEM_FULL, EPCP_VER, &epcp_ver);
            compact_payload_add_data(EPCP_SYSTEM_FULL, EPCP_FW_MAJOR, &system_info.updateVerMaj);
            compact_payload_add_data(EPCP_SYSTEM_FULL, EPCP_FW_MINOR, &system_info.updateVerMin);
            compact_payload_add_data(EPCP_SYSTEM_FULL, EPCP_FW_PATCH, &system_info.updateVerBui);
            compact_payload_add_data(EPCP_SYSTEM_FULL, EPCP_MSG_CNT, &frameCounter);
            compact_payload_add_data(EPCP_SYSTEM_FULL, EPCP_SN, system_info.ep_serial);
            compact_payload_add_data(EPCP_SYSTEM_FULL, EPCP_TIME, &ts);
            compact_payload_add_data(EPCP_SYSTEM_FULL, EPCP_BATT, &(ct.ui32));
        }else{
            compact_payload_add_data(EPCP_SYSTEM_CONDENSED, EPCP_VER, &epcp_ver);
            compact_payload_add_data(EPCP_SYSTEM_CONDENSED, EPCP_MSG_CNT, &frameCounter);
            compact_payload_add_data(EPCP_SYSTEM_CONDENSED, EPCP_TIME, &ts);
            compact_payload_add_data(EPCP_SYSTEM_CONDENSED, EPCP_BATT, &(ct.ui32));
        }

        //Add Cell data to subpacket
        char* endptr;
        uint64_t hexIMEI = strtoull(parameters.imei,&endptr,10); //Convert IMEI from string to number
        compact_payload_add_data(EPCP_CELL, EPCP_IMEI, &(hexIMEI));
        compact_payload_add_data(EPCP_CELL, EPCP_RSSI, &(parameters.rssi));
        compact_payload_add_data(EPCP_CELL, EPCP_RSRQ, &(parameters.rsrq));

        //Add HTU21D / SI7021 data to subpacket
        #if SI7021_ACTIVE == 1
        //Convert TEMP to int per spec
        ct.i32 = si_temp * 100;
        compact_payload_add_data(EPCP_SI7021, EPCP_TEMP, &(ct.ui32));
        //Convert HUM to int per spec
        ct.i32 = si_hum * 100;
        compact_payload_add_data(EPCP_SI7021, EPCP_HUM, &(ct.ui32));
        #endif

        //Add BME680 data to subpacket
        #if BME680_ACTIVE == 1
        ct.i32 = bme_data.temp * 100;
        compact_payload_add_data(EPCP_BME680, EPCP_TEMP, &(ct.ui32));
        ct.i32 = bme_data.raw_pressure * 100;
        compact_payload_add_data(EPCP_BME680, EPCP_PRES, &(ct.ui32));
        ct.ui32 = bme_data.humidity * 100;
        compact_payload_add_data(EPCP_BME680, EPCP_HUM, &(ct.ui32));
        ct.ui32 = bme_data.raw_gas * 100;
        compact_payload_add_data(EPCP_BME680, EPCP_GAS, &(ct.ui32));
        ct.ui32 = bme_data.co2_equivalent * 100;
        compact_payload_add_data(EPCP_BME680, EPCP_CO2, &(ct.ui32));
        ct.ui32 = bme_data.breath_voc_equivalent * 100;
        compact_payload_add_data(EPCP_BME680, EPCP_BREATH, &(ct.ui32));
        ct.ui32 = bme_data.iaq;
        compact_payload_add_data(EPCP_BME680, EPCP_IAQ_SCORE, &(ct.ui32));
        ct.ui32 = bme_data.iaq_accuracy;
        compact_payload_add_data(EPCP_BME680, EPCP_IAQ_ACCURACY, &(ct.ui32));
        #endif

        //Add ICM20602 data to subpacket
        #if ICM20602_ACTIVE == 1
        ct.i32 = icm_data.accel_x * 100;
        compact_payload_add_data(EPCP_ICM20602, EPCP_ACCEL_X, &(ct.ui32));
        ct.i32 = icm_data.accel_y * 100;
        compact_payload_add_data(EPCP_ICM20602, EPCP_ACCEL_Y, &(ct.ui32));
        ct.i32 = icm_data.accel_z * 100;
        compact_payload_add_data(EPCP_ICM20602, EPCP_ACCEL_Z, &(ct.ui32));
        ct.i32 = icm_data.gyro_x * 100;
        compact_payload_add_data(EPCP_ICM20602, EPCP_GYRO_X, &(ct.ui32));
        ct.i32 = icm_data.gyro_y * 100;
        compact_payload_add_data(EPCP_ICM20602, EPCP_GYRO_Y, &(ct.ui32));
        ct.i32 = icm_data.gyro_z * 100;
        compact_payload_add_data(EPCP_ICM20602, EPCP_GYRO_Z, &(ct.ui32));
        #endif

        //Add VL53L0X data to subpacket
        #if VL53L0X_ACTIVE == 1
        ct.ui32 = vl53l0x_get_data(&tof, &errFlag);
        if(errFlag != NRFX_SUCCESS){
            DBGE("VL53L0X init error!");
        }
        compact_payload_add_data(EPCP_VL53L0X, EPCP_DIST, &(ct.ui32));
        #endif

        //Generate completed packet
        cell_queue_msg local_sensor_data_msg;
        local_sensor_data_msg.size  = compact_payload_get_packet(local_sensor_data_msg.data); 

        #if CELLULAR_ACTIVE == 1
        //Forward data to cell queue if there is space
        if(xQueueSend(xCellQueue, &local_sensor_data_msg, CELL_QUEUE_TIMEOUT) != pdTRUE){
            DBGE("Queue Timeout!");
        }

        // Set flag
        newDataAdded = true;
        #endif

        parse_downlink();

        DBGI("Sleep for %d seconds", cellTransInt);
        DBGI("Sensor power off");
        // Clear pin configurations to prevent back-powering (High-Impedance / No Pull)
        nrfx_twi_uninit(&twi);
        nrf_gpio_cfg_default(PIN_NAME_SCL);
        nrf_gpio_cfg_default(PIN_NAME_SDA);
        uninit_uart(TASK_1);
        sensorPwrEnConfig(false);        
        vTaskDelay(pdMS_TO_TICKS(cellTransInt * 1000));
    }
}

void addGnssQueueMsg( bool status )
{
    //Create message to send to queue
    static uint32_t prevFixTime = 0;

    cellDiag parameters = {'\0'};
    //Get latest cell values
    if( queryCellularDiag(&parameters) != CellSuccess )
    {
        /* retry once */
        if( queryCellularDiag(&parameters) != CellSuccess )
        {
            DBGE("Failed to query cell diagnostic data");
            return;
        }
    }

    /* Ensure new fix by change in timestamp */
    if(parameters.fixTime == prevFixTime && parameters.gtpLatFloat == 0 || status == false){
        DBGE("No new fix detected");
        return;
    }

    //Set up compact payload
    compact_payload_init();

    //Set up data converter
    union epcp_convert_type ct;

    // Increment frame counter
    frameCounter = (uint8_t) (frameCounter + 1);

    // Get time for data packet
    uint64_t ts = get_time_s();

    compact_payload_add_data(EPCP_SYSTEM_CONDENSED, EPCP_VER, &epcp_ver);
    compact_payload_add_data(EPCP_SYSTEM_CONDENSED, EPCP_MSG_CNT, &frameCounter);
    compact_payload_add_data(EPCP_SYSTEM_CONDENSED, EPCP_TIME, &ts);
    ct.ui32 = ep_bsp_read_battery_voltage() * 100;
    compact_payload_add_data(EPCP_SYSTEM_CONDENSED, EPCP_BATT, &(ct.ui32));

    //Add Cell data to subpacket
    char* endptr;
    uint64_t hexIMEI = strtoull(parameters.imei,&endptr,10); //Convert IMEI from string to number
    compact_payload_add_data(EPCP_CELL, EPCP_IMEI, &(hexIMEI));
    compact_payload_add_data(EPCP_CELL, EPCP_RSSI, &(parameters.rssi));
    compact_payload_add_data(EPCP_CELL, EPCP_RSRQ, &(parameters.rsrq));

    if(parameters.fixTime != prevFixTime || status == false){
        //Add satellites
        compact_payload_add_data(EPCP_GNSS, EPCP_SAT, &(parameters.satellites));
        //If failed fix then set values to 0
        if(parameters.fix < 2){
            parameters.lat_float = 0;
            parameters.long_float = 0;
            compact_payload_set_error(EPCP_GNSS,true);
        }
        //Add GNSS data to subpacket    
        prevFixTime = parameters.fixTime;
        //Convert LAT from float to int per spec
        ct.i32 = parameters.lat_float * 100000;
        compact_payload_add_data(EPCP_GNSS, EPCP_LAT, &(ct.ui32));
        //Convert LON from float to int per spec
        ct.i32 = parameters.long_float * 100000;
        compact_payload_add_data(EPCP_GNSS, EPCP_LON, &(ct.ui32));
    }

    //Add GTP data to subpacket
    if(parameters.gtpLatFloat != 0){
        //Convert LAT from float to int per spec
        ct.i32 = parameters.gtpLatFloat * 100000;
        compact_payload_add_data(EPCP_GTP, EPCP_LAT, &(ct.ui32));
        //Convert LON from float to int per spec
        ct.i32 = parameters.gtpLongFloat * 100000;
        compact_payload_add_data(EPCP_GTP, EPCP_LON, &(ct.ui32));
        //Add accuracy
        ct.i32 = parameters.gtpAccuracy * 100;
        compact_payload_add_data(EPCP_GTP, EPCP_ACC, &(ct.ui32));
        //Clear data
        parameters.gtpLatFloat = 0;
    }

    //Generate completed packet
    cell_queue_msg gnss_data_msg;
    gnss_data_msg.size = compact_payload_get_packet(gnss_data_msg.data);   

    // Add new queue entry
    if(xQueueSend(xCellQueue, &gnss_data_msg, CELL_QUEUE_TIMEOUT) != pdTRUE){
        DBGE("Queue Timeout!");
    }
}