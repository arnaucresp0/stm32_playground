/**
*****************************************************************************
* @file         hw_rev_module.c
* @brief        This module is for hardware revision function manage.
* @author       Arnau Crespo
* @version      1.0.0
******************************************************************************
*/

#define HW_REV_APP_C

/******************************************************************************
* MODULES USED
*****************************************************************************/
// Add modules from more generic to most specific
#include <stdint.h>
#include <math.h>
#include <stdlib.h>

#include "hw_rev_module.h"
#include "uart_control.h"
#include "flash.h"
#include "version.h"



/******************************************************************************
* DEFINITIONS AND MACROS
*****************************************************************************/
//Those defines are extracted from hw_epwm.h, and are shared between DSP models.
#define MAX_VOLTAGE_UL              1023        //MAX VOLTAGE UPPER LIMIT 3.3V (dec. 1023)
#define MAX_VOLTAGE_LL              1000        // MAX VOLTAGE LOWER LIMIT 3.29V (dec. 1020)
#define MIN_VOLTAGE_UL              256         //MAX VOLTAGE UPPER LIMIT 0.8V (dec. 256)
#define MIN_VOLTAGE_LL              0           //MIN VOLTAGE LOWER LIMIT 0 V (dec. 0)

#define HW_VER_002                  2           //PREVIOUS POWERHUB HARDWARE VERSION
#define HW_VER_003                  3           //ACTUAL HARDWARE VERSION
#define HW_SUB_VER                  1           //SUBVERSION THAT IS SET MANUALLY

#define MCU_STATUS_TIMEOUT          10   		//In [s]
#define RETRY_TIMEOUT               570         //Timeout to retry to send again the  messages
#define SERIAL_LENGTH               5           //THIS IS THE NUMBER OF DIGITS THAT HAS THE SERIAL NUMBER.
#define MESSAGE_LENGTH              19          //MSP STATUS MESSAGE LENGTH
/******************************************************************************
* TYPEDEFS AND STRUCTURES
******************************************************************************/

/******************************************************************************
* PROTOTYPES OF LOCAL FUNCTIONS
*****************************************************************************/
static bool send_status_data(void);

/******************************************************************************
* LOCAL VARIABLES
*****************************************************************************/
static uint16_t status_counter = 0;
static uint8_t packet[MESSAGE_LENGTH];
static uint32_t serialNumber = 0;
/*****************************************************************************
* EXPORTED FUNCTIONS
*****************************************************************************/

/**
***********************************************************************
* @brief - This function needs to be called every 1ms
***********************************************************************
*/
void send_MCU_status(void){
    status_counter++;
    if (status_counter > MCU_STATUS_TIMEOUT){
        //Send the info packet;
        if (send_status_data() == true){
            status_counter = 0;
        }else{
            status_counter = MCU_STATUS_TIMEOUT - RETRY_TIMEOUT;
        }
    }
}

/**
***********************************************************************
* @brief - Function to generate a 5 random digit as serial number of the board
* @param - [param] ...
* @param - [param2] ...
* \b note:  More info if needed
***********************************************************************
*/
void generate_Serial_Num(void){
    uint32_t serial = 0;
    const uint16_t deviceID = *((volatile uint32_t*)0x1FFFF7AC); // Get the unique ID from the STM32 Chip
    srand(deviceID); // Generate a 5-digit random number based on the unique ID
    for (uint8_t i = 0; i < SERIAL_LENGTH; i++){
        serial = serial * 10 + (rand() % 10);
    }
    if(serialNumber == 0){
        serialNumber = serial;
        Save_To_Flash(FLASH_SERIAL_ADDR, &serialNumber, sizeof(serialNumber));
    }
}

/**
***********************************************************************
* @brief - Function to return the resial number of the board
***********************************************************************
*/
static uint32_t getSerialNumber(void){
    return serialNumber;
}

/**
***********************************************************************
* @brief - Function to build and send the status data packet
***********************************************************************
*/
static bool send_status_data(void){
    const uint8_t length = MESSAGE_LENGTH;
    //uint8_t packet[MESSAGE_LENGTH];
    //FIRMWARE VERSION
    packet[0] = FIRMWARE_VERSION;                              // Firmware version
    packet[1] = FIRMWARE_VERSION2;                             // Firmware version2

    //HARDWARE VERSION
    packet[2] = HARDWARE_VERSION;
    packet[3] = HARDWARE_VERSION2;

    //SERIAL NUMBER
    const uint32_t serialNum = getSerialNumber();
    packet[4] = (serialNum >> 24) & 0xFF;
    packet[5] = (serialNum >> 16) & 0xFF;
    packet[6] = (serialNum >> 8) & 0xFF;
    packet[7] = serialNum & 0xFF;

    //EVALVE OPEN TIME:
    const uint16_t OpenTime = eValveControl_getEvalveOpenTime();
    const uint8_t ot1 = (OpenTime >> 8) & 0xFF;
    const uint8_t ot2 = OpenTime & 0xFF;
    packet[8] = ot1;                                          // eValve open time
    packet[9] = ot2;

    //EVALVE TOTAL CYCLES:
    const uint32_t eV_CyclesCounter = eValveControl_return_OpenCloseCounter();
    const uint8_t cc4 = (eV_CyclesCounter & 0x000000FF);
    const uint8_t cc3 = (eV_CyclesCounter & 0x0000FF00) >> 8;
    const uint8_t cc2 = (eV_CyclesCounter & 0x00FF0000) >> 16;
    const uint8_t cc1 = (eV_CyclesCounter & 0xFF000000) >> 24;
    packet[10] = cc1;
    packet[11] = cc2;
    packet[12] = cc3;
    packet[13] = cc4;

    //EVALVE TOTAL FLUSHTIME:
    const uint32_t eV_FlushTimeSum = eValveControl_return_OpenCloseTime();
    const uint8_t ft4 = (eV_FlushTimeSum & 0x000000FF);
    const uint8_t ft3 = (eV_FlushTimeSum & 0x0000FF00) >> 8;
    const uint8_t ft2 = (eV_FlushTimeSum & 0x00FF0000) >> 16;
    const uint8_t ft1 = (eV_FlushTimeSum & 0xFF000000) >> 24;
    packet[14] = ft1;
    packet[15] = ft2;
    packet[16] = ft3;
    packet[17] = ft4;

    //GENERAL STATUS
    const uint8_t general_status = 0;
    packet[18] = general_status;

    return uartControl_message_creator(MAILBOX_INFO, length, packet);
}

/******************************************************************************
* EOF
*****************************************************************************/
