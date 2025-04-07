/**
*****************************************************************************
* @file         warning_module.c
* @brief        This module is for triggering different possible warnings that may occur during the firmware execution
* @author       Arnau Crespo
* @version      1.0.0
******************************************************************************
*/

#define WARNING_MOD_C

/******************************************************************************
* MODULES USED
*****************************************************************************/
// Add modules from more generic to most specific
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "warning_module.h"
#include "stm32f0xx_hal.h"
#include "uart_control.h"
#include "usart.h"

/******************************************************************************
* DEFINITIONS AND MACROS
*****************************************************************************/
#define WARNING_TIMEOUT     7000    //Timeout to send the warning message periodically (1 min)
#define RETRY_TIMEOUT       300     //Timeout to retry to send again the  messages
/******************************************************************************
* TYPEDEFS AND STRUCTURES
******************************************************************************/
//Union for warnings_t
typedef struct{
    uint8_t evalve_overpower    : 1;      //If the evalve consumes slightly more than what it should.
    uint8_t evalve_underpower   : 1;      //If the evalve consumes slightly less than what it should.
    uint8_t kus_overpower       : 1;      //If the kus consumes slightly more than what it should.
    uint8_t kus_underpower      : 1;      //If the kus consumes slightly less thant what it should.
}warning_bits_u;

typedef union{
    warning_bits_u warningBits;
    uint8_t warningsActive;
}warnings_u;

/******************************************************************************
* PROTOTYPES OF LOCAL FUNCTIONS
*****************************************************************************/
static HAL_StatusTypeDef send_warning_message(void);

/******************************************************************************
* LOCAL VARIABLES
*****************************************************************************/
static uint16_t     warning_tick_counter = 0;
static warnings_u   warningsStruct = {0};
/******************************************************************************
* EXPORTED FUNCTIONS
*****************************************************************************/

/**
***********************************************************************
* @brief - Increase the timeout tick counter for periodically send the
* warning message.
***********************************************************************
*/
 void warning_module_tick_counter(void){
    warning_tick_counter++;
}

/**
***********************************************************************
* @brief - Explain here what this function does
***********************************************************************
*/
void warning_module_main(void){
    //Check if there is any warning active
    if (warningsStruct.warningsActive > 0){
        //If the message was not sent, try to send it again
        if (warning_tick_counter > WARNING_TIMEOUT){
            if (send_warning_message() == true){    //Send the info packet and clear the warnings
                warning_tick_counter = 0;
                warningsStruct.warningsActive = 0;
            }else{
                warning_tick_counter = WARNING_TIMEOUT - RETRY_TIMEOUT;
            }
        }
    }
}

static void set_warning(warning_type_e warning, bool value){
    switch(warning){
        default:
            return;
        case(EVALVE_OVERPOWER_WARN):
            warningsStruct.warningBits.evalve_overpower = value;
            break;
        case(EVALVE_UNDERPOWER_WARN):
            warningsStruct.warningBits.evalve_underpower = value;
            break;
        case(KUS_OVERPOWER_WARN):
            warningsStruct.warningBits.kus_overpower = value;
            break;
        case(KUS_UNDERPOWER_WARN):
            warningsStruct.warningBits.kus_underpower = value;
            break;
    }
    if (value == true)  warning_tick_counter = WARNING_TIMEOUT; //Force send uart message
}

/**
***********************************************************************
* @brief - This function is called when a warning is triggered
***********************************************************************
*/
void trigger_warning(warning_type_e warning){
    set_warning(warning, true);
}

/**
***********************************************************************
* @brief - Clear the warning
***********************************************************************
*/
void clear_warning(warning_type_e warning){
    set_warning(warning, false);
}


/******************************************************************************
* LOCAL FUNCTIONS
*****************************************************************************/

/**
***********************************************************************
* @brief send the UART warning message
***********************************************************************
*/
static HAL_StatusTypeDef send_warning_message(void){
    uint8_t packet[MAX_WARNINGS + 2]; // 1 mailbox + 1 length + 4 warning bytes
    uint8_t packetCounter = 0;
    packet[packetCounter++] = MAILBOX_WARNING; // Mailbox identifier
    packet[packetCounter++] = 0x00; // Placeholder for length
    // Append warning bits
    packet[packetCounter++] = warningsStruct.warningBits.evalve_overpower;
    packet[packetCounter++] = warningsStruct.warningBits.evalve_underpower;
    packet[packetCounter++] = warningsStruct.warningBits.kus_overpower;
    packet[packetCounter++] = warningsStruct.warningBits.kus_underpower;
    // Fill in the length field (total size including mailbox + length)
    packet[1] = packetCounter;
    return HAL_UART_Transmit_DMA(&huart2, packet, packetCounter);
}


/******************************************************************************
* EOF
*****************************************************************************/
