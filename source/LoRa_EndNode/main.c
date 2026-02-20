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
#include "nrf_crypto.h"
// Project Includes
#include "main.h"
#include "ep_bsp.h"
#include "uart_helper.h"
#include "htu21d.h"
#include "vl53l0x.h"
#include "bme680.h"
#include "icm20602.h"
#include "qspi_helper.h"
#include "led_helper.h"
#include "time_helper.h"
#include "cell_helper.h"
#include "compact_payload_config.h"
#include "epcp_builder_agora.h"
#include "LoRaWAN.h"

//Activate / deactivate onboard sensors
#define SI7021_ACTIVE   1
#define VL53L0X_ACTIVE  0
#define BME680_ACTIVE   0
#define ICM20602_ACTIVE 1

#define mainLED_TASK_STACK_SIZE             128
#define mainSAMPLE_TASK_STACK_SIZE          1024
#define LORAWAN_CLASSA_TASK_STACK_SIZE      2048
#define LORAWAN_CLASSA_TASK_PRIORITY        ( tskIDLE_PRIORITY + 1 )

// Declare FreeRTOS tasks, queues, and semaphore
SemaphoreHandle_t dataReadySemaphore;
SemaphoreHandle_t loraSemaphore;
SemaphoreHandle_t xSPISemaphore;
SemaphoreHandle_t xSensPwrEnSemaphore;
QueueHandle_t xLoraQueue;
TaskHandle_t ledTaskHandle;
TaskHandle_t sensorSampleTaskHandle;
TaskHandle_t loraTaskHandle;

system_info_struct system_info;

//Sets the comm path configuration, Cell enabled by default, can also enable LoRa
uint8_t comm_conf = COMM_LORA_ONLY;

/* LoRa data transmit interval */
uint32_t loraTransInt = MIN_TRANS_INTERVAL;

//Activate / deactivate onboard sensors
#define SI7021_ACTIVE   1
#define VL53L0X_ACTIVE  0
#define BME680_ACTIVE   0
#define ICM20602_ACTIVE 1

//FreeRTOS task and queue settigns
#define mainLED_TASK_STACK_SIZE             128
#define mainCell_TASK_STACK_SIZE            5000
#define LORAWAN_CLASSA_TASK_STACK_SIZE      2048
#define LORAWAN_CLASSA_TASK_PRIORITY        ( tskIDLE_PRIORITY + 1 )
#define CELL_QUEUE_TIMEOUT                  pdMS_TO_TICKS(3000)                     /** Time to wait on queue before moving on to the next sample */

#if ICM20602_ACTIVE
/* Empty ICM struct */
ICM20602 icm;
#endif

/* TWI instance ID. */
#define TWI_INSTANCE_ID 0

/* TWI instance. */
static const nrfx_twi_t twi = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Task for reading local sensors and adding to LoRa data queue */
void sensorSampleTask(void *pvParameters);

/* Miscellaneous initialization including preparing the logging and lora. */
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
        led_mode(LED_CELL_OK, true, 1);

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
        sprintf( system_info.updateVerStr,"%02d.%02d.%02ld", system_info.updateVerMaj, system_info.updateVerMin, system_info.updateVerBui );        
    }
    else
    {
        // Set integer version
        system_info.updateVerMaj = VERSION_MAJOR;
        system_info.updateVerMin = VERSION_MINOR;
        system_info.updateVerBui = VERSION_BUILD;
        // Set string version
        sprintf( system_info.updateVerStr,"%02d.%02d.%02ld", system_info.updateVerMaj, system_info.updateVerMin, system_info.updateVerBui );
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

    //Setup LoRa data queue
    xLoraQueue = xQueueCreate(LORA_QUEUE_SIZE, sizeof(lora_queue_msg));
    if(xLoraQueue == NULL){
        DBGE("xLoraQueue creation failed!");
    }

    /* Initialize LoRa parameters */
    loraParam loraParams = {0};
    // Set lora parameters, must be called prior to scheduler starting if using SSL/TLS due to NRF SDK incompatibilities
    loraParams.region = LORAWAN_REGION;
    loraParams.appPort = LORAWAN_APP_PORT;
    loraParams.confSend = LORAWAN_CONFIRMED_SEND;
    loraParams.jitterMS = LORAWAN_APPLICATION_JITTER_MS;
    loraParams.rxWindowMS = CLASSA_RECEIVE_WINDOW_DURATION_MS;
    loraParams.txIntervalSec = loraTransInt;
    loraParams.mosi = SER_CON_SPIS_MOSI_PIN;
    loraParams.miso = SER_CON_SPIS_MISO_PIN;
    loraParams.sck = SER_CON_SPIS_SCK_PIN;
    loraParams.subBand = subbandDefault;
    loraParams.maxJoinAttempts = lorawanConfigMAX_JOIN_ATTEMPTS;
    loraParams.commInterface = comm_conf;
    memcpy(loraParams.devEUI, devEUIDefault, sizeof(devEUI));
    memcpy(loraParams.joinEUI, joinEUIDefault, sizeof(joinEUI));
    memcpy(loraParams.appKey, appKeyDefault, sizeof(appKey));

    // Initialize LoRa parameters
    if( loraConfig( loraParams ) == false )
    {
        DBGI( "LoRa parameters failed to initialize" );
    }
    else
    {
        DBGI( "LoRa parameters successfully initialized" );  
    }

    /* Add user tasks */
    xTaskCreate( vLorawanClassATask, "LoRaWanClassA", LORAWAN_CLASSA_TASK_STACK_SIZE, NULL, LORAWAN_CLASSA_TASK_PRIORITY, &loraTaskHandle );
    
    //Create semaphore
    xSPISemaphore = xSemaphoreCreateBinary();
    if(xSPISemaphore == NULL){
        DBGE("xSPISemaphore semaphore creation failure!");
    }

    //Give semaphore
    xSemaphoreGive(xSPISemaphore);

    uninit_uart(MAIN_LOOP); //comment out to prevent sleep
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

        //Get BME680 data if active and available
        bme680_sensor_data bme_data; 
        #if BME680_ACTIVE == 1
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

        //Get VLXL0X data if active and available
        VL53L0X tof;
        #if VL53L0X_ACTIVE == 1
            err = vl53l0x_init(&tof, twi);
            if(err != NRFX_SUCCESS){
                DBGE("VL53L0X init error!");
            }
            //Set the tof timing budget to 200ms
            setMeasurementTimingBudget(&tof, 200000);
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

        //Add VL53L0X data to subpacket
        #if VL53L0X_ACTIVE == 1
            uint32_t vl5310_data = vl53l0x_get_data(&tof, &err);
            if(err != NRFX_SUCCESS){
                DBGE("VL53L0X init error!");
            }
            agora_add_data(&agora_compact_payload, EPCP_VL53L0X, EPCP_DIST, &vl5310_data);
        #endif

        //Generate completed packet
        lora_queue_msg local_sensor_data_msg;
        local_sensor_data_msg.size  = agora_get_packet(&agora_compact_payload, local_sensor_data_msg.data); 

        //Forward data to cell queue if there is space
        if(xQueueSend(xLoraQueue, &local_sensor_data_msg, LORA_QUEUE_TIMEOUT) != pdTRUE){
            DBGE("LoRa queue Timeout!");
        }

        //Deinit the compact payload and put the task to sleep
        epcp_builder_agora_deinit(&agora_compact_payload);
        vTaskDelay(pdMS_TO_TICKS(loraTransInt * 1000));
    }
}