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
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "queue.h"

#include "ep_bsp.h"

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_freertos.h"
#include "nrf_delay.h"
#include "fds.h"
#include "nrf_drv_clock.h"
#include "uart_helper.h"
#include "cell_helper.h"
#include "ep_ble_central.h"
#include "epcp_builder_agora.h"
#include "htu21d.h"
#include "icm20602.h"
#include "qspi_helper.h"
#include "led_helper.h"
#include "time_helper.h"

SemaphoreHandle_t cellularSemaphore;
QueueHandle_t xCellQueue;
/* Semaphore to lock changing of sensor power enable */
SemaphoreHandle_t xSensPwrEnSemaphore;

TaskHandle_t ledTaskHandle;
TaskHandle_t sensorSampleTaskHandle;

//Activate / deactivate onboard sensors
#define SI7021_ACTIVE   1
#define BME680_ACTIVE   0
#define ICM20602_ACTIVE 0

#define mainLED_TASK_STACK_SIZE             128
#define mainCell_TASK_STACK_SIZE            4500
#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define OSTIMER_WAIT_FOR_QUEUE              2                                       /**< Number of ticks to wait for the timer queue to be ready */
#define CELL_QUEUE_TIMEOUT                  pdMS_TO_TICKS(3000)                     /** Time to wait on queue before moving on to the next sample */

#if ICM20602_ACTIVE
/* Empty ICM struct */
ICM20602 icm;
#endif

CellStatus_t cell_status;

system_info_struct system_info;

/* Cellular Transmission Interval */
static uint32_t cellTransInt = MIN_TRANS_INTERVAL;

/* TWI instance ID. */
#define TWI_INSTANCE_ID 0

/* TWI instance. */
static const nrfx_twi_t twi = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Initial PDP config */
CellularBLEConfig_t pdnConfig = { ATT_PDN, CELLULAR_PDN_CONTEXT_IPV4, "" };

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

/* Task for reading local sensors and adding to cell data queue */
void sensorSampleTask(void *pvParameters);

/* The task function to setup cellular with thread ready environment. */
static void CellularTask( void * pvParameters );

/* Miscellaneous initialization including preparing the logging and cell. */
static void prvMiscInitialization( void );

/* Helper function for making received Agora version numbers more readable */
static void parse_version(char* version, char* conv_fw_ver);


/*-----------------------------------------------------------*/
// Taken from FreeRTOS demo: https://github.com/FreeRTOS/FreeRTOS/blob/main/FreeRTOS-Plus/Demo/FreeRTOS_Cellular_Interface_Windows_Simulator/Common/main.c
//Cellular communication task
static void CellularTask( void * pvParameters )
{
    bool first_run = true;
    bool retCellular = false;
    cellParam cellParams = {0};
    uint32_t regTimeout = 900; // 900s registration timeout

    // Set cell parameters, must be called prior to scheduler starting if using SSL/TLS due to NRF SDK incompatibilities
    strncpy(cellParams.appVersion, system_info.updateVerStr, strlen(system_info.updateVerStr));
    strncpy(cellParams.brokerAddrPost, IOT_BROKER_ADDRESS_POST, strlen(IOT_BROKER_ADDRESS_POST));
    strncpy(cellParams.brokerAddrGet, IOT_BROKER_ADDRESS_GET, strlen(IOT_BROKER_ADDRESS_GET));
    cellParams.carrier = CELLULAR_CARRIER;
    strncpy(cellParams.cert, CERTIFICATE, sizeof(CERTIFICATE));
    cellParams.gnssStatus = TELIT_GNSS_STATUS;
    cellParams.gnssInterval = GNSS_ATTEMPT_INTERVAL;
    cellParams.gnssTimeout = GNSS_TIMEOUT;
    strncpy(cellParams.serverAddrPrefixPost, SERVER_ADDR_PREFIX_POST, strlen(SERVER_ADDR_PREFIX_POST));
    strncpy(cellParams.serverAddrPrefixGet, SERVER_ADDR_PREFIX_GET, strlen(SERVER_ADDR_PREFIX_GET));
    strncpy(cellParams.serverAttribSuffix, SERVER_ADDR_SUFFIX_ATTR, strlen(SERVER_ADDR_SUFFIX_ATTR));
    strncpy(cellParams.serverDownloadSuffix, SERVER_ADDR_SUFFIX_FOTA, strlen(SERVER_ADDR_SUFFIX_FOTA));
    strncpy(cellParams.serverTelemSuffix, SERVER_ADDR_SUFFIX_TELE, strlen(SERVER_ADDR_SUFFIX_TELE));
    cellParams.accessTokenInPathPost = SERVER_PATH_ACCESS_TOKEN_POST_ENABLED;
    cellParams.accessTokenInPathGet = SERVER_PATH_ACCESS_TOKEN_GET_ENABLED;
    strncpy(cellParams.pvtKey, PVT_KEY, sizeof(PVT_KEY));
    strncpy(cellParams.rootCA1, ROOT_CA1, sizeof(ROOT_CA1));
    strncpy(cellParams.mqttTopicString, MQTT_TOPIC, sizeof(MQTT_TOPIC));
    cellParams.technology = CELLULAR_TECH;
    cellParams.checksumMethod = otapal_CHECKSUM_METHOD;
    cellParams.fotaFlashStart = otapal_FLASH_START;
    cellParams.fotaDescrTblStart = otapal_DESCRIPTOR_START;
    cellParams.fotaBankSize = otapal_BANK_SIZE;
    cellParams.cellOnOffPin = CELL_ON_OFF;
    cellParams.cellPwrEnPin = CELL_PWR_EN;
    cellParams.cellRTSPin = CELL_RTS_PIN_NUMBER;
    cellParams.payloadFormat = CELL_PAYLOAD_FORMAT;

    // Initialize cell
    retCellular = cellInit( &cellParams, &pdnConfig );    

    if( retCellular != true )
    {
        DBGI("Cellular failed to initialize.\r\n");
    }
    else
    {
        DBGI( ( "Cellular successfully initialized.\r\n" ) );  
    }
    //runCellular once prior to loop because first transmission is configured to never try for a GNSS fix
    cellStatus = runCellular(cellTransInt, regTimeout);
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(15000));
        while(xQueuePeek(xCellQueue, NULL, 0) == pdTRUE){
            if(!first_run){
                // Reenable LED task
                led_resume();
                vTaskResume( ledTaskHandle );
            }else{
                first_run = false;
            }

            //This call of function to get unit registered to network and connected to desired endpoint address
            cell_status = runCellular(cellTransInt, regTimeout);

            // Suspend LED task
            vTaskSuspend( ledTaskHandle );
            // Enable debug out for LED
            led_pause();
        }        
    }
    vTaskDelete( NULL );
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
        xSensPwrEnSemaphore = xSemaphoreCreateBinary();
        nrf_gpio_cfg_output(PIN_NAME_SENSOR_POWER_ENABLE);
        //Clear flag
        init = false;
    }
    else
    {
        //Take semaphore
        xSemaphoreTake(xSensPwrEnSemaphore, pdMS_TO_TICKS(500));
    }

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
    // Initialize bsp
    ep_bsp_init( WATCHDOG_RELOAD, WATCHDOG_RELOAD_RATE );

    init_uart(MAIN_LOOP);
    uart_helper.dbgi = true;

    // Initialize LEDs, default to alive blink normal operation
    led_init();
    led_mode(LED_ALIVE_BLINK,1,true);

    //Enable sensor power
    sensorPwrEnConfig(true);

    nrf_delay_ms(1500);

    /* Check for version number in qpsi, otherwise use default */
    ImageDescriptor_t xDescriptor;
    ProductionTable_t xProdTable;

    /* Initialize qspi driver.*/
    nrfx_err_t err_code = qspi_init();

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

    DBGI("***************************************************");
    DBGI("*       Embedded Planet: BLE CENTRAL v%s    *", system_info.updateVerStr);
    DBGI("***************************************************");
                
#if BLE_ACTIVE
    /* Task to read local sensor data and forward it to the cell queue */
    xTaskCreate(sensorSampleTask, 
                "SensorTask", 
                1024, 
                NULL, 
                tskIDLE_PRIORITY+2,
                NULL);
#endif
    /* Create the task to run tests. */
    xTaskCreate( LEDTask,
                "LEDTask",
                mainLED_TASK_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY+1,
                &ledTaskHandle );

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

    /* Create the task to run tests. */
    xTaskCreate( CellularTask,
                "CellularTask",
                mainCell_TASK_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY+2,
                NULL ); 

#if BLE_ACTIVE
    DBGI( ( "BLE Task Initializing .\r\n" ) );
    /* Initialize BLE stack and create the SoftDevice BLE task */
    ep_ble_central_init(CONNECT_IF_MATCH);
#endif    

    //uninit_uart(MAIN_LOOP); //comment out to prevent sleep
}

/* Waits for the semaphore that is given by sensor_sample_callback. Retrieves latest local sensor data and adds to cell queue */
void sensorSampleTask(void *pvParameters)
{
    nrfx_err_t err;
    
    // Frame counter for payload
    static uint8_t frameCounter = 0;

    init_uart(TASK_1);

    //Enable sensor power        
    DBGI("Sensor power on");
    sensorPwrEnConfig(true);
    vTaskDelay(pdMS_TO_TICKS(20));

    //Init nRF TWI interface
    twi_init();

    //Small delay needed after twi/spi init before communication can begin
    vTaskDelay(pdMS_TO_TICKS(20));

    //Sensor initializations
    #if BME680_ACTIVE == 1
    bme680_init(twi, BSEC_SAMPLE_RATE_LP, 0.0f);
    #endif
    #if ICM20602_ACTIVE == 1
    icm20602_init(&icm, ICM20602_ADDR_LOW, twi);
    #endif

    while(1){
        // Display heap usage
        DBGI("FreeRTOS Heap Space: %ld",xPortGetFreeHeapSize());

        //Get latest cell diagnostic values
        cellDiag parameters = {'\0'};
        if( queryCellularDiag(&parameters) != CELLULAR_SUCCESS )
        {
            /* Do not send if unable to attain values */
            continue;
        }
        
        //Get current time. In seconds if we have already synced time via cellular or GNSS. If not, this represents the run time of the system.
        uint32_t ts = get_time_s();

        //Set and adjust transmission frame counter
        if(frameCounter >= 0xFF){
            frameCounter = 0;
        }
        frameCounter++;

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

        #if BME680_ACTIVE == 1
            //Get BME680 data if active and available
            bme680_sensor_data bme_data; 
            DBGW("BME get semaphore");
            if(xSemaphoreTake(xBme680DataReadySemaphore, pdMS_TO_TICKS(5000))){
                bme_data = bme680_get_latest_data();\
            }else{
                DBGW("BME680 data not available!");
            }
        #endif

        //Get icm20602 data if active and available
        icm20602_sensor_data icm_data;
        #if ICM20602_ACTIVE == 1
            //Get new ICM20602 data if available
            bool icm_data_ready = icm20602_data_ready(&icm, &err);
            if(icm_data_ready){
                icm_data = icm20602_get_data(&icm, &err); 
            }else{
                DBGW("ICM20602 data not available!");
            }
        #endif

        //Set up compact payload
        epcp_builder_agora agora_compact_payload;
        epcp_builder_agora_init(&agora_compact_payload);

        //Add System data to subpacket
        //Convert battery from float to int per spec
        uint32_t batVolt = ep_bsp_read_battery_voltage() * 100;
        uint8_t epcpVer = EPCP_VERSION;
        if(frameCounter == 1){
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM, EPCP_VER, &epcpVer);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM, EPCP_FW_MAJOR, &system_info.updateVerMaj);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM, EPCP_FW_MINOR, &system_info.updateVerMin);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM, EPCP_FW_PATCH, &system_info.updateVerBui);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM, EPCP_MSG_CNT, &frameCounter);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM, EPCP_SN, system_info.ep_serial);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM, EPCP_TIME, &ts);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM, EPCP_BATT, &batVolt);
        }else{
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM_V2, EPCP_VER, &epcpVer);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM_V2, EPCP_MSG_CNT, &frameCounter);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM_V2, EPCP_TIME, &ts);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM_V2, EPCP_BATT, &batVolt);
        }

        //Add Cell data to subpacket
        char* endptr;
        uint64_t hexIMEI = strtoull(parameters.imei,&endptr,10); //Convert IMEI from string to number
        agora_add_data(&agora_compact_payload, EPCP_CELL, EPCP_IMEI, &(hexIMEI));
        agora_add_data(&agora_compact_payload, EPCP_CELL, EPCP_RSSI, &(parameters.rssi));
        agora_add_data(&agora_compact_payload, EPCP_CELL, EPCP_RSRQ, &(parameters.rsrq));

        //Add HTU21D / SI7021 data to subpacket
        #if SI7021_ACTIVE == 1
            //Convert TEMP to int per spec
            int32_t temperature = si_temp * 100;
            agora_add_data(&agora_compact_payload, EPCP_SI7021, EPCP_TEMP, &temperature);
            //Convert HUM to int per spec
            int32_t humidity = si_hum * 100;
            agora_add_data(&agora_compact_payload, EPCP_SI7021, EPCP_HUM, &humidity);
        #endif

        //Add BME680 data to subpacket
        #if BME680_ACTIVE == 1
            int32_t bmeTemp = bme_data.temp * 100;
            agora_add_data(&agora_compact_payload, EPCP_BME680, EPCP_TEMP, &bmeTemp);
            int32_t bmePress = bme_data.raw_pressure * 100;
            agora_add_data(&agora_compact_payload, EPCP_BME680, EPCP_PRES, &bmePress);
            uint32_t bmeHumid = bme_data.humidity * 100;
            agora_add_data(&agora_compact_payload, EPCP_BME680, EPCP_HUM, &bmeHumid);
            uint32_t bmeGas = bme_data.raw_gas * 100;
            agora_add_data(&agora_compact_payload, EPCP_BME680, EPCP_GAS, &bmeGas);
            uint32_t bmeCO2 = bme_data.co2_equivalent * 100;
            agora_add_data(&agora_compact_payload, EPCP_BME680, EPCP_CO2, &bmeCO2);
            uint32_t bmeVOC = bme_data.breath_voc_equivalent * 100;
            agora_add_data(&agora_compact_payload, EPCP_BME680, EPCP_BREATH, &bmeVOC);
            uint32_t bmeIAQ = bme_data.iaq;
            agora_add_data(&agora_compact_payload, EPCP_BME680, EPCP_IAQ_SCORE, &bmeIAQ);
            uint32_t bmeIAQAcc = bme_data.iaq_accuracy;
            agora_add_data(&agora_compact_payload, EPCP_BME680, EPCP_IAQ_ACCURACY, &bmeIAQAcc);
        #endif

        //Add ICM20602 data to subpacket
        #if ICM20602_ACTIVE == 1
            int32_t accelX = icm_data.accel_x * 100;
            agora_add_data(&agora_compact_payload, EPCP_ICM20602, EPCP_ACCEL_X, &accelX);
            int32_t accelY = icm_data.accel_y * 100;
            agora_add_data(&agora_compact_payload, EPCP_ICM20602, EPCP_ACCEL_Y, &accelY);
            int32_t accelZ = icm_data.accel_z * 100;
            agora_add_data(&agora_compact_payload, EPCP_ICM20602, EPCP_ACCEL_Z, &accelZ);
            int32_t gyroX = icm_data.gyro_x * 100;
            agora_add_data(&agora_compact_payload, EPCP_ICM20602, EPCP_GYRO_X, &gyroX);
            int32_t gyroY = icm_data.gyro_y * 100;
            agora_add_data(&agora_compact_payload, EPCP_ICM20602, EPCP_GYRO_Y, &gyroY);
            int32_t gyroZ = icm_data.gyro_z * 100;
            agora_add_data(&agora_compact_payload, EPCP_ICM20602, EPCP_GYRO_Z, &gyroZ);
        #endif

        //Generate completed packet
        cell_queue_msg local_sensor_data_msg;
        local_sensor_data_msg.size  = agora_get_packet(&agora_compact_payload, local_sensor_data_msg.data); 

        //Forward data to cell queue if there is space
        if(xQueueSend(xCellQueue, &local_sensor_data_msg, CELL_QUEUE_TIMEOUT) != pdTRUE){
            DBGE("Cell queue Timeout!");
        }

        //Deinit the compact payload and put the task to sleep
        epcp_builder_agora_deinit(&agora_compact_payload);
        vTaskDelay(pdMS_TO_TICKS(cellTransInt * 1000));
    }
}

void appendMsgWithGNSS( void )
{
    //Create message to send to queue
    cell_queue_msg cell_msg;
    static uint32_t prevFixTime = 0;

    if( xQueuePeek( xCellQueue,
                        &( cell_msg ),
                        ( TickType_t ) 10 ) == pdTRUE )
    {
        if(cell_msg.has_gnss){
            //This message already contains GNSS information
            return;
        }

        //Read message to append
        if( xQueueReceive( xCellQueue, &( cell_msg ), ( TickType_t ) 100 ) == pdTRUE )
        {
            cellDiag parameters = {'\0'};
            //Get latest cell values
            if( queryCellularDiag(&parameters) != CELLULAR_SUCCESS )
            {
                /* retry once */
                if( queryCellularDiag(&parameters) != CELLULAR_SUCCESS )
                {
                    DBGE("Failed to query cell diagnostic data");
                    return;
                }
            }


            //Set up compact payload
            epcp_builder_agora agora_compact_payload;
            epcp_builder_agora_init(&agora_compact_payload);

            //Print GNSS data to debug log
            DBGI("Lat:%f,Long:%f",parameters.lat_float,parameters.long_float);
           
            //Set up data converter
            union epcp_convert_type ct;

            //Add GNSS data to subpacket
            if(parameters.fixTime != prevFixTime){           
                prevFixTime = parameters.fixTime;
                //Convert LAT from float to int per spec
                ct.i32 = parameters.lat_float * 10000;
                agora_add_data(&agora_compact_payload, EPCP_GNSS, EPCP_LAT, &(ct.ui32));

                //Convert LON from float to int per spec
                ct.i32 = parameters.long_float * 10000;
                agora_add_data(&agora_compact_payload, EPCP_GNSS, EPCP_LON, &(ct.ui32));
            }

            //Generate completed packet
            cell_queue_msg gnss_sensor_data_msg;
            gnss_sensor_data_msg.size = agora_get_packet(&agora_compact_payload, gnss_sensor_data_msg.data);

            // Generate combined packet */
            cell_queue_msg combined_sensor_data_msg;

            memcpy(&combined_sensor_data_msg.data[0],&cell_msg.data[0],cell_msg.size);
            memcpy(&combined_sensor_data_msg.data[cell_msg.size],&gnss_sensor_data_msg.data[0],gnss_sensor_data_msg.size);

            combined_sensor_data_msg.size = ((cell_msg.size)+(gnss_sensor_data_msg.size));

            //Its possible to miss gnss fix if cell service is unable for an extended period and will retry on next gnss interval
            if(xQueueSendToFront(xCellQueue, &combined_sensor_data_msg, CELL_QUEUE_TIMEOUT) != pdTRUE){
                DBGE("Queue Timeout!");
            }

            //Free message
            epcp_builder_agora_deinit(&agora_compact_payload);            
        }
    }
}