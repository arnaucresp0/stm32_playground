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

#include "stm32f0xx_hal.h"
#include "hw_rev_module.h"
#include "uart_control.h"
#include "flash.h"
#include "usart.h"
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

#define MCU_STATUS_TIMEOUT          100   		//In [s]
#define RETRY_TIMEOUT               570         //Timeout to retry to send again the  messages
#define SERIAL_LENGTH               5           //THIS IS THE NUMBER OF DIGITS THAT HAS THE SERIAL NUMBER.
#define MESSAGE_LENGTH              20          //MSP STATUS MESSAGE LENGTH

#define MAILBOX_INFO				243
/******************************************************************************
* TYPEDEFS AND STRUCTURES
******************************************************************************/

/******************************************************************************
* PROTOTYPES OF LOCAL FUNCTIONS
*****************************************************************************/
static HAL_StatusTypeDef send_status_data(void);
static uint32_t xorshift32(uint32_t *state);
/******************************************************************************
* LOCAL VARIABLES
*****************************************************************************/
static uint16_t status_counter = 0;
static uint8_t packet[MESSAGE_LENGTH];
static uint32_t serialNumber = 0;
DMA_HandleTypeDef hdma_usart2_tx;
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
    	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        if (send_status_data() == HAL_OK){
            status_counter = 0;
        }else{
            status_counter = MCU_STATUS_TIMEOUT - RETRY_TIMEOUT;
        }
    }
}

static uint32_t xorshift32(uint32_t *state) {
    uint32_t x = *state;
    x ^= x << 13;   // Left shift + XOR
    x ^= x >> 17;   // Right shift + XOR
    x ^= x << 5;    // Left shift + XOR
    *state = x;     // Store new state
    return x;
}

/**
***********************************************************************
* @brief - Function to generate a 5 random digit as serial number of the board
* @param - [param] ...
* @param - [param2] ...
* \b note:  More info if needed
***********************************************************************
*/
void generate_Serial_Num(void) {
    uint32_t serial = 0;
    static uint32_t rng_state = 0;

    // Use Unique Device ID as seed
    rng_state = *((volatile uint32_t*)0x1FFFF7AC);

    // Generate a 5-digit number
    for (uint8_t i = 0; i < SERIAL_LENGTH; i++) {
        serial = serial * 10 + (xorshift32(&rng_state) % 10);
    }

    if (serialNumber == 0) {
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
static HAL_StatusTypeDef send_status_data(void){
    const uint8_t length = MESSAGE_LENGTH;
    //uint8_t packet[MESSAGE_LENGTH];
    //FIRMWARE VERSION
    packet[0] = MAILBOX_INFO;                              // Firmware version
    packet[1] = MESSAGE_LENGTH;                             // Firmware version2

    packet[2] = FIRMWARE_VERSION;
    packet[3] = FIRMWARE_VERSION2;

    //HARDWARE VERSION
    packet[4] = HARDWARE_VERSION;
    packet[5] = HARDWARE_VERSION2;

    //SERIAL NUMBER
    const uint32_t serialNum = getSerialNumber();
    packet[6] = (serialNum >> 24) & 0xFF;
    packet[7] = (serialNum >> 16) & 0xFF;
    packet[8] = (serialNum >> 8) & 0xFF;
    packet[9] = serialNum & 0xFF;

    //EVALVE OPEN TIME:
    const uint16_t OpenTime = eValveControl_getEvalveOpenTime();
    const uint8_t ot1 = (OpenTime >> 8) & 0xFF;
    const uint8_t ot2 = OpenTime & 0xFF;
    packet[10] = ot1;                                          // eValve open time
    packet[11] = ot2;

    //EVALVE TOTAL CYCLES:
    const uint32_t eV_CyclesCounter = eValveControl_return_OpenCloseCounter();
    const uint8_t cc4 = (eV_CyclesCounter & 0x000000FF);
    const uint8_t cc3 = (eV_CyclesCounter & 0x0000FF00) >> 8;
    const uint8_t cc2 = (eV_CyclesCounter & 0x00FF0000) >> 16;
    const uint8_t cc1 = (eV_CyclesCounter & 0xFF000000) >> 24;
    packet[12] = cc1;
    packet[13] = cc2;
    packet[14] = cc3;
    packet[15] = cc4;

    //EVALVE TOTAL FLUSHTIME:
    const uint32_t eV_FlushTimeSum = eValveControl_return_OpenCloseTime();
    const uint8_t ft4 = (eV_FlushTimeSum & 0x000000FF);
    const uint8_t ft3 = (eV_FlushTimeSum & 0x0000FF00) >> 8;
    const uint8_t ft2 = (eV_FlushTimeSum & 0x00FF0000) >> 16;
    const uint8_t ft1 = (eV_FlushTimeSum & 0xFF000000) >> 24;
    packet[16] = ft1;
    packet[17] = ft2;
    packet[18] = ft3;
    packet[19] = ft4;

    //GENERAL STATUS
    const uint8_t general_status = 0;
    packet[20] = general_status;

    //uartControl_message_creator(MAILBOX_INFO, length, packet)

    return HAL_UART_Transmit_DMA(&huart2, packet, length);
}

/******************************************************************************
* EOF
*****************************************************************************/
