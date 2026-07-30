/**
 * Created on: Sept 1, 2022
 * Created by: golobmichael
 * 
 * Copyright (c) Embedded Planet, Inc - All rights reserved
 *
 * This source file is private and confidential.
 * Unauthorized copying of this file is strictly prohibited.
 * 
 * Version 1.0 - 01SEPT22  Initial, ported from: https://github.com/EmbeddedPlanet/nrf_sdk_17_1/tree/master/examples/ble_peripheral/ble_app_hrs_freertos, golobmichael
 */

//Standard library includes
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

//nRF includes
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_freertos.h"
#include "nrf_delay.h"
#include "nrf_crypto.h"
#include "fds.h"
#include "nrf_drv_clock.h"

//General FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "queue.h"

//FreeRTOS library includes
#include "cellular_common_api.h"
#include "cellular_common.h"
#include "cellular_comm_interface.h"

//Embedded Planet includes
#include "ep_bsp.h"
#include "uart_helper.h"
#include "epcp_builder_agora.h"
#include "ep_ble_peripheral.h"
#include "qspi_helper.h"
#include "led_helper.h"
#include "time_helper.h"
#include "htu21d.h"
#include "vl53l0x.h"
#include "bme680.h"
#include "icm20602.h"
#include "ble_config.h"
#if defined(THINGSBOARD_HTTPS_INTEGRATION) || defined(AZURE_MQTTS_X509) || defined(AWS_MQTTS_X509) || defined(AWS_HTTPS_X509) || defined(AZURE_MQTTS_SAS) || defined(AZURE_HTTPS_SAS)
#include "nrf_crypto.h"
#endif

/* LoRa includes */
//#include "LoRaWAN.h"
#include "LoRaMac.h"
#include "spi.h"
#include "board.h"
//#include "LoRaWANConfig.h"
#include "gpio.h"
#include "sx1276-board.h"

//Activate / deactivate onboard sensors
#define SI7021_ACTIVE   1
#define VL53L0X_ACTIVE  1
#define BME680_ACTIVE   0
#define ICM20602_ACTIVE 0

//FreeRTOS task and queue settigns
#define mainLED_TASK_STACK_SIZE             128
#define mainCell_TASK_STACK_SIZE            5000
#define LORAWAN_CLASSA_TASK_STACK_SIZE      2048
#define LORAWAN_CLASSA_TASK_PRIORITY        ( tskIDLE_PRIORITY + 1 )
#define CELL_QUEUE_TIMEOUT                  pdMS_TO_TICKS(3000)                     /** Time to wait on queue before moving on to the next sample */

//TWI instance ID
#define TWI_INSTANCE_ID 0

//Set the active sensors. Can be NO_SENSORS to deactivate all sensors or any other combination of enum_sensor_conf
uint8_t sensor_conf = BME680_ACTIVE | VL53L0X_ACTIVE | ICM20602_ACTIVE;

//Sets the comm path configuration
uint8_t comm_conf = COMM_CELL_ONLY;

//Task handles
TaskHandle_t ledTaskHandle;
TaskHandle_t sensorSampleTaskHandle;
TaskHandle_t cellularTaskHandle;
TaskHandle_t loraTaskHandle;

//Semaphores
SemaphoreHandle_t cellularSemaphore;
SemaphoreHandle_t loraSemaphore;
SemaphoreHandle_t xSensPwrEnSemaphore;
SemaphoreHandle_t xSPISemaphore;

//Queues
QueueHandle_t xCellQueue;
QueueHandle_t xLoraQueue;

// Frame counter for payload
static uint8_t frameCounter = 0;

// Empty ICM struct
ICM20602 icm;

system_info_struct system_info;

//Cellular transmission interval
static uint32_t cellTransInt = MIN_TRANS_INTERVAL;

/* Initial PDP config */
CellularBLEConfig_t pdnConfig = { '\0' };
CellularBLEConfig_t pdnConfigDefault = { '\0' };

//TWI instance
static const nrfx_twi_t twi = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID);

// Parse cell downlink attributes
void parse_downlink(void);

//TWI initialization
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

//Task for reading sensor values and adding to communication data queues
void sensorSampleTask(void *pvParameters)
{   
    nrfx_err_t err;

    //Init nRF TWI interface
    twi_init();

    //Small delay needed after twi/spi init before communication can begin
    vTaskDelay(pdMS_TO_TICKS(20));

    //One-time sensor initializations
    bme680_init(twi, BSEC_SAMPLE_RATE_LP, 0.0f);
    icm20602_init(&icm, ICM20602_ADDR_LOW, twi);
    
    while(1){
        DBGI("FreeRTOS Heap Space: %ld",xPortGetFreeHeapSize());

        init_uart(TASK_1);

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
            continue;
        }
        DBGI("IMEI received");
        
        //Get current time. In seconds if we have already synced time via cellular or GNSS. If not, this represents the run time of the system.
        uint32_t ts = get_time_s();

        //Increment frame counter or roll over if needed
        frameCounter = (uint8_t) (frameCounter + 1);

        //Get SI7021 data if active and available
        float si_temp, si_hum;
        if((sensor_conf & SI7021_ACTIVE) == SI7021_ACTIVE){
            htu21_init(twi);
            err = htu21_is_connected();
            if(err != NRFX_SUCCESS){
                DBGW("SI7021 data not available!");
            }
            htu21_read_temperature_and_relative_humidity(&si_temp, &si_hum);
        }

        //Get BME680 data if active and available
        bme680_sensor_data bme_data; 
        if((sensor_conf & BME680_ACTIVE) == BME680_ACTIVE){
            DBGW("BME get semaphore");
            if(xSemaphoreTake(xBme680DataReadySemaphore, pdMS_TO_TICKS(5000))){
                bme_data = bme680_get_latest_data();\
            }else{
                DBGW("BME680 data not available!");
            }
        }

        //Get icm20602 data if active and available
        icm20602_sensor_data icm_data;
        int errFlag;
        if((sensor_conf & ICM20602_ACTIVE) == ICM20602_ACTIVE){
            //Get new ICM20602 data if available
            bool icm_data_ready = icm20602_data_ready(&icm, &errFlag);
            if(icm_data_ready){
                icm_data = icm20602_get_data(&icm, &errFlag); 
            }else{
                DBGW("ICM20602 data not available!");
            }
        }

        //Get VLXL0X data if active and available
        VL53L0X tof;
        if((sensor_conf & VL53L0X_ACTIVE) == VL53L0X_ACTIVE){
            errFlag = vl53l0x_init(&tof, twi);
            if(errFlag != NRFX_SUCCESS){
                DBGE("VL53L0X init error!");
            }
            //Set the tof timing budget to 200ms
            setMeasurementTimingBudget(&tof, 200000);
        }

        //Set up data converter
        union epcp_convert_type ct;

        //Set up compact payload
        epcp_builder_agora agora_compact_payload;
        epcp_builder_agora_init(&agora_compact_payload);

        //Add System data to subpacket
        //Convert battery from float to int per spec
        ct.ui32 = ep_bsp_read_battery_voltage() * 100;
        if(frameCounter == 1){
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM, EPCP_VER, &epcp_ver);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM, EPCP_FW_MAJOR, &system_info.updateVerMaj);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM, EPCP_FW_MINOR, &system_info.updateVerMin);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM, EPCP_FW_PATCH, &system_info.updateVerBui);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM, EPCP_MSG_CNT, &frameCounter);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM, EPCP_SN, system_info.ep_serial);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM, EPCP_TIME, &ts);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM, EPCP_BATT, &(ct.ui32));
        }else{
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM_V2, EPCP_VER, &epcp_ver);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM_V2, EPCP_MSG_CNT, &frameCounter);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM_V2, EPCP_TIME, &ts);
            agora_add_data(&agora_compact_payload, EPCP_SYSTEM_V2, EPCP_BATT, &(ct.ui32));
        }

        //Add Cell data to subpacket
        char* endptr;
        uint64_t hexIMEI = strtoull(parameters.imei,&endptr,10); //Convert IMEI from string to number
        agora_add_data(&agora_compact_payload, EPCP_CELL, EPCP_IMEI, &(hexIMEI));
        agora_add_data(&agora_compact_payload, EPCP_CELL, EPCP_RSSI, &(parameters.rssi));
        agora_add_data(&agora_compact_payload, EPCP_CELL, EPCP_RSRQ, &(parameters.rsrq));

        //Add HTU21D / SI7021 data to subpacket
        if((sensor_conf & SI7021_ACTIVE) == SI7021_ACTIVE){
            //Convert TEMP to int per spec
            ct.i32 = si_temp * 100;
            agora_add_data(&agora_compact_payload, EPCP_SI7021, EPCP_TEMP, &(ct.ui32));
            //Convert HUM to int per spec
            ct.i32 = si_hum * 100;
            agora_add_data(&agora_compact_payload, EPCP_SI7021, EPCP_HUM, &(ct.ui32));
        }

        //Add BME680 data to subpacket
        if((sensor_conf & BME680_ACTIVE) == BME680_ACTIVE){
            ct.i32 = bme_data.temp * 100;
            agora_add_data(&agora_compact_payload, EPCP_BME680, EPCP_TEMP, &(ct.ui32));
            ct.i32 = bme_data.raw_pressure * 100;
            agora_add_data(&agora_compact_payload, EPCP_BME680, EPCP_PRES, &(ct.ui32));
            ct.ui32 = bme_data.humidity * 100;
            agora_add_data(&agora_compact_payload, EPCP_BME680, EPCP_HUM, &(ct.ui32));
            ct.ui32 = bme_data.raw_gas * 100;
            agora_add_data(&agora_compact_payload, EPCP_BME680, EPCP_GAS, &(ct.ui32));
            ct.ui32 = bme_data.co2_equivalent * 100;
            agora_add_data(&agora_compact_payload, EPCP_BME680, EPCP_CO2, &(ct.ui32));
            ct.ui32 = bme_data.breath_voc_equivalent * 100;
            agora_add_data(&agora_compact_payload, EPCP_BME680, EPCP_BREATH, &(ct.ui32));
            ct.ui32 = bme_data.iaq;
            agora_add_data(&agora_compact_payload, EPCP_BME680, EPCP_IAQ_SCORE, &(ct.ui32));
            ct.ui32 = bme_data.iaq_accuracy;
            agora_add_data(&agora_compact_payload, EPCP_BME680, EPCP_IAQ_ACCURACY, &(ct.ui32));
        }

        //Add ICM20602 data to subpacket
        if((sensor_conf & ICM20602_ACTIVE) == ICM20602_ACTIVE){
            ct.i32 = icm_data.accel_x * 100;
            agora_add_data(&agora_compact_payload, EPCP_ICM20602, EPCP_ACCEL_X, &(ct.ui32));
            ct.i32 = icm_data.accel_y * 100;
            agora_add_data(&agora_compact_payload, EPCP_ICM20602, EPCP_ACCEL_Y, &(ct.ui32));
            ct.i32 = icm_data.accel_z * 100;
            agora_add_data(&agora_compact_payload, EPCP_ICM20602, EPCP_ACCEL_Z, &(ct.ui32));
            ct.i32 = icm_data.gyro_x * 100;
            agora_add_data(&agora_compact_payload, EPCP_ICM20602, EPCP_GYRO_X, &(ct.ui32));
            ct.i32 = icm_data.gyro_y * 100;
            agora_add_data(&agora_compact_payload, EPCP_ICM20602, EPCP_GYRO_Y, &(ct.ui32));
            ct.i32 = icm_data.gyro_z * 100;
            agora_add_data(&agora_compact_payload, EPCP_ICM20602, EPCP_GYRO_Z, &(ct.ui32));
        }

        //Add VL53L0X data to subpacket
        if((sensor_conf & VL53L0X_ACTIVE) == VL53L0X_ACTIVE){
            ct.ui32 = vl53l0x_get_data(&tof, &errFlag);
            if(errFlag != NRFX_SUCCESS){
                DBGE("VL53L0X init error!");
            }
            agora_add_data(&agora_compact_payload, EPCP_VL53L0X, EPCP_DIST, &(ct.ui32));
        }

        //Generate completed packet
        cell_queue_msg local_sensor_data_msg;
        local_sensor_data_msg.size  = agora_get_packet(&agora_compact_payload, local_sensor_data_msg.data); 

        #if CELLULAR_ACTIVE
        //Forward data to cell queue if there is space
        if(xQueueSend(xCellQueue, &local_sensor_data_msg, CELL_QUEUE_TIMEOUT) != pdTRUE){
            DBGE("Queue Timeout!");
        }

        // Set flag
        newDataAdded = true;
        #endif

        parse_downlink();

        epcp_builder_agora_deinit(&agora_compact_payload);
        
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
    epcp_builder_agora agora_compact_payload;
    epcp_builder_agora_init(&agora_compact_payload);

    //Set up data converter
    union epcp_convert_type ct;

    // Increment frame counter
    frameCounter = (uint8_t) (frameCounter + 1);

    // Get time for data packet
    uint64_t ts = get_time_s();

    agora_add_data(&agora_compact_payload, EPCP_SYSTEM_V2, EPCP_VER, &epcp_ver);
    agora_add_data(&agora_compact_payload, EPCP_SYSTEM_V2, EPCP_MSG_CNT, &frameCounter);
    agora_add_data(&agora_compact_payload, EPCP_SYSTEM_V2, EPCP_TIME, &ts);
    ct.ui32 = ep_bsp_read_battery_voltage() * 100;
    agora_add_data(&agora_compact_payload, EPCP_SYSTEM_V2, EPCP_BATT, &(ct.ui32));

    //Add Cell data to subpacket
    char* endptr;
    uint64_t hexIMEI = strtoull(parameters.imei,&endptr,10); //Convert IMEI from string to number
    agora_add_data(&agora_compact_payload, EPCP_CELL, EPCP_IMEI, &(hexIMEI));
    agora_add_data(&agora_compact_payload, EPCP_CELL, EPCP_RSSI, &(parameters.rssi));
    agora_add_data(&agora_compact_payload, EPCP_CELL, EPCP_RSRQ, &(parameters.rsrq));

    if(parameters.fixTime != prevFixTime || status == false){
        //Add satellites
        agora_add_data(&agora_compact_payload, EPCP_GNSS, EPCP_SAT, &(parameters.satellites));
        //If failed fix then set values to 0
        if(parameters.fix < 2){
            parameters.lat_float = 0;
            parameters.long_float = 0;
            agora_set_error(&agora_compact_payload,EPCP_GNSS,true);
        }
        //Add GNSS data to subpacket    
        prevFixTime = parameters.fixTime;
        //Convert LAT from float to int per spec
        ct.i32 = parameters.lat_float * 100000;
        agora_add_data(&agora_compact_payload, EPCP_GNSS, EPCP_LAT, &(ct.ui32));
        //Convert LON from float to int per spec
        ct.i32 = parameters.long_float * 100000;
        agora_add_data(&agora_compact_payload, EPCP_GNSS, EPCP_LON, &(ct.ui32));
    }

    //Add GTP data to subpacket
    if(parameters.gtpLatFloat != 0){
        //Convert LAT from float to int per spec
        ct.i32 = parameters.gtpLatFloat * 100000;
        agora_add_data(&agora_compact_payload, EPCP_GTP, EPCP_GTP_LAT, &(ct.ui32));
        //Convert LON from float to int per spec
        ct.i32 = parameters.gtpLongFloat * 100000;
        agora_add_data(&agora_compact_payload, EPCP_GTP, EPCP_GTP_LON, &(ct.ui32));
        //Add accuracy
        ct.i32 = parameters.gtpAccuracy * 100;
        agora_add_data(&agora_compact_payload, EPCP_GTP, EPCP_GTP_ACC, &(ct.ui32));
        //Clear data
        parameters.gtpLatFloat = 0;
    }

    //Generate completed packet
    cell_queue_msg gnss_data_msg;
    gnss_data_msg.size = agora_get_packet(&agora_compact_payload, gnss_data_msg.data);   

    // Add new queue entry
    if(xQueueSend(xCellQueue, &gnss_data_msg, CELL_QUEUE_TIMEOUT) != pdTRUE){
        DBGE("Queue Timeout!");
    }

    //Free message
    epcp_builder_agora_deinit(&agora_compact_payload);            
}

//Miscellaneous application initialization
static void prvMiscInitialization( void )
{
    // Initialize bsp
    ep_bsp_init( WATCHDOG_RELOAD, WATCHDOG_RELOAD_RATE );

    init_uart(MAIN_LOOP);
    uart_helper.dbgi = true;

    // Initialize crypto library for SSL over cell, needs to be prior to freertos scheduler starting
    if( nrf_crypto_init() != NRF_SUCCESS)
    {
        DBGE("Failed to initialize nrf crypto");
    }

    // Initialize LEDs, default to alive blink normal operation
    led_init();
    led_mode(LED_ALIVE_BLINK,1,true);

    //Enable sensor power
    sensorPwrEnConfig(true);

    nrf_delay_ms(1500);

    /* Check for version number in qpsi, otherwise use default */
    ImageDescriptor_t xDescriptor;
    ProductionTable_t xProdTable;
    /* Initialize LoRa parameters */
    loraParam loraParams = {0};

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
        sprintf( system_info.updateVerStr,"%02x.%02x.%02x", system_info.updateVerMaj, system_info.updateVerMin, system_info.updateVerBui );
    }
    else
    {
        /* Set integer version */
        system_info.updateVerMaj = VERSION_MAJOR;
        system_info.updateVerMin = VERSION_MINOR;
        system_info.updateVerBui = VERSION_BUILD;
        /* Set string version */
        sprintf( system_info.updateVerStr,"%02x.%02x.%02x", system_info.updateVerMaj, system_info.updateVerMin, system_info.updateVerBui );
    }

    /* Read production table in external flash */
    err_code = qspi_read( (uint8_t * ) &xProdTable, sizeof(xProdTable), otapal_PROD_TBL_START );
    if( err_code != NRFX_SUCCESS )
    {
        DBGE( "Prod Table read failed with with error code %d", err_code );

         /* Use defaults */
        memcpy( devEUI, devEUIDefault, sizeof( devEUIDefault ) );
        memcpy( joinEUI, joinEUIDefault, sizeof( joinEUIDefault ) );
        memcpy( appKey, appKeyDefault, sizeof( appKeyDefault ) );
        subBand = subbandDefault;
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

        if( strncmp( xProdTable.loraDevEUI, "\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF", sizeof( xProdTable.loraDevEUI ) ) != 0 )
        {            
            loraParams.subBand = xProdTable.loraSubband;
            memcpy(loraParams.devEUI, xProdTable.loraDevEUI, sizeof(xProdTable.loraDevEUI));
            memcpy(loraParams.joinEUI, xProdTable.loraJoinEUI, sizeof(xProdTable.loraJoinEUI));
            memcpy(loraParams.appKey, xProdTable.loraAppKey, sizeof(xProdTable.loraAppKey));
        }
        /* Use default production table values, if needed */
        else
        {
            DBGE("Invalid LoRa Credentials, Programming Defaults");
            loraParams.subBand = subbandDefault;
            memcpy(loraParams.devEUI, devEUIDefault, sizeof(devEUIDefault));
            memcpy(loraParams.joinEUI, joinEUIDefault, sizeof(joinEUIDefault));
            memcpy(loraParams.appKey, appKeyDefault, sizeof(appKeyDefault));
        }
    }

    /* Read ble settings table in external flash */
    err_code = qspi_read( (uint8_t * ) &pdnConfig, sizeof(pdnConfig), otapal_BLE_TBL_START );
    /* If error or invalid then use defaults */
    //if( err_code != NRFX_SUCCESS )
    //{
    //    DBGE( "BLE Settings Table read failed with with error code %d", err_code );

        strncpy(pdnConfig.apnName, pdnConfigDefault.apnName, sizeof(pdnConfigDefault.apnName));
    //    DBGI("Resetting to defaults: APN: %s", pdnConfig.apnName);
    //}
    //else
    //{
        /* Display values read */
        DBGI("Cell APN: %s, %d", pdnConfig.apnName, strlen(pdnConfig.apnName));
    //}

    DBGI("*********************************************");
    DBGI("*       Embedded Planet: AGORA v%s    *", system_info.updateVerStr);
    DBGI("*********************************************");

    // Check if LoRa is active
#if LORA_ACTIVE
    /* Display to debug port */
    // Set lora parameters, must be called prior to scheduler starting if using SSL/TLS due to NRF SDK incompatibilities
    loraParams.region = LORAWAN_REGION;
    loraParams.appPort = LORAWAN_APP_PORT;
    loraParams.confSend = LORAWAN_CONFIRMED_SEND;
    loraParams.jitterMS = LORAWAN_APPLICATION_JITTER_MS;
    loraParams.rxWindowMS = CLASSA_RECEIVE_WINDOW_DURATION_MS;
    loraParams.txIntervalSec = cellTransInt;
    loraParams.mosi = SER_CON_SPIS_MOSI_PIN;
    loraParams.miso = SER_CON_SPIS_MISO_PIN;
    loraParams.sck = SER_CON_SPIS_SCK_PIN;
    loraParams.subBand = subbandDefault;
    loraParams.maxJoinAttempts = lorawanConfigMAX_JOIN_ATTEMPTS;
    loraParams.commInterface = comm_conf;

    // Initialize LoRa parameters
    if( loraConfig( loraParams ) == false )
    {
        DBGI( "LoRa parameters failed to initialize" );
    }
    else
    {
        DBGI( "LoRa parameters successfully initialized" );  
    }

    if(comm_conf != COMM_CELL_ONLY){
        //Setup LoRa data queue
        xLoraQueue = xQueueCreate(LORA_QUEUE_SIZE, sizeof(lora_queue_msg));
        if(xLoraQueue == NULL){
            DBGE("xLoraQueue creation failed!");
        }
    }

    /* Add user tasks */
    xTaskCreate( vLorawanClassATask, "LoRaWanClassA", LORAWAN_CLASSA_TASK_STACK_SIZE, NULL, LORAWAN_CLASSA_TASK_PRIORITY, &loraTaskHandle );
#endif

    //Create semaphore
    xSPISemaphore = xSemaphoreCreateBinary();
    if(xSPISemaphore == NULL){
        DBGE("xSPISemaphore semaphore creation failure!");
    }

    //Give semaphore
    xSemaphoreGive(xSPISemaphore);

    /* Task to read local sensor data and forward it to the cell queue */
    xTaskCreate(sensorSampleTask, 
                "SensorTask", 
                2048, 
                NULL, 
                tskIDLE_PRIORITY+2,
                &sensorSampleTaskHandle);
    //We want this task to start in a suspended state so that we can get GNSS info in on the first transmission if available.                
    vTaskSuspend(sensorSampleTaskHandle);

    /* Create the task to run tests. */
    xTaskCreate( LEDTask,
                "LEDTask",
                mainLED_TASK_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY+1,
                &ledTaskHandle );

    /* The ble_config task handles BLE configuration of the device after boot */
    xTaskCreate(ble_config, 
                "ble_conf", 
                500, 
                NULL, 
                tskIDLE_PRIORITY+2, 
                NULL);

    // BLE Initialization. 
    ep_ble_peripheral_init(DEVICE_NAME_LOCAL, strlen(DEVICE_NAME_LOCAL), system_info.ep_serial);

#if CELLULAR_ACTIVE
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
#endif

    set_time(0);

    /* Disable QSPI */
    qspi_uninit();

    //uninit_uart(MAIN_LOOP); //comment out to prevent sleep
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