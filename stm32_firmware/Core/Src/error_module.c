/**
*****************************************************************************
* @file         error_module.c
* @brief        This module is for handling the errors in the PowerHub.
* @author       Arnau Crespo
* @version      1.0.0
******************************************************************************
*/

/******************************************************************************
* MODULES USED
*****************************************************************************/
// Add modules from more generic to most specific
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "error_module.h"
#include "flash.h"
#include "stm32f0xx_hal.h"
#include "uart_control.h"
#include "usart.h"

/******************************************************************************
* DEFINITIONS AND MACROS
*****************************************************************************/
#define MSP_ERROR_TIMEOUT       3000    //Time accepted between errors (30 sec)

/******************************************************************************
* TYPEDEFS AND STRUCTURES
******************************************************************************/

typedef struct{
    uint8_t errors_counters[MAX_ERRORS];
    uint8_t reset_errors;           	//This variable holds the value of the reset type (reset_errors_e);
    uint8_t errorsActive            :1; //This bit is set to 1 if there is any error active.
    uint8_t errorsPendingToBeSent   :1; //This bit is set to 0 if the errors were already sent.
}errors_struct_t;

/******************************************************************************
* PROTOTYPES OF LOCAL FUNCTIONS
*****************************************************************************/
static HAL_StatusTypeDef send_error_on_uart(void);
static void read_reset_register(void);
inline bool tick_timeout_check(const uint16_t timeout, const uint16_t tickCounter);
/******************************************************************************
* LOCAL VARIABLES
*****************************************************************************/
static errors_struct_t  errorList = {0};
static uint16_t error_tick_counter = 0;
/******************************************************************************
* EXPORTED FUNCTIONS
*****************************************************************************/
/**
***********************************************************************
* @brief - This function increment the error_tick to send the error message.
***********************************************************************
*/
void error_pending_tick_counter(void){
    error_tick_counter++;
}

/**
***********************************************************************
* @brief - Initialize the error module
***********************************************************************
*/
void error_module_init(void){
    //This function will read the SYSRSTIV to detect the reset cause at startup.
    read_reset_register();
}

/**
***********************************************************************
* @brief - Marks the flag to send the errors as soon as possible.
***********************************************************************
*/
void error_module_send_errors(void){
    errorList.errorsPendingToBeSent = 1;
}

/**
***********************************************************************
* @brief - This is the main function to manage the powerHub errors
***********************************************************************
*/
void error_module_main(void){
    if (errorList.errorsActive == 1){
        //If errorsPendingToBeSent is false, send the errors periodically until they are cleared.
        // Or if the errors were already sent, resend them every 30 sec.
        if ((error_tick_counter > MSP_ERROR_TIMEOUT) || (errorList.errorsPendingToBeSent == true)){
            //Send the info packet;
            if (send_error_on_uart() == HAL_OK){
                errorList.errorsPendingToBeSent = 0;
            }
            //Retry again later...
            error_tick_counter = 0;
        }
    }
}

/**
***********************************************************************
* @brief - This function is called to trigger the error from each module and set the error type.
* @param - [error_type_e] The error that needs to be triggered.
***********************************************************************
*/
void trigger_error(error_type_e error){
	//Increase the error counter of the corresponding error.
	errorList.errors_counters[error]++;
	//Check if there were any errors active before, if not set the flag to 1.
	if (errorList.errorsActive == 0){
		errorList.errorsActive = 1;
		//Activate the pending flag to send the errors as soon as possible.
		errorList.errorsPendingToBeSent = 1;
	}
	Save_To_Flash(FLASH_ERRORS_ADDR, errorList.errors_counters, sizeof(errorList.errors_counters));
}

/**
***********************************************************************
* @brief - Function to build and send the errors data packet.
***********************************************************************
*/
static HAL_StatusTypeDef send_error_on_uart(void){
	uint8_t packet[MAX_ERRORS + 3]; // 1 mailbox + 1 length + MAX_ERRORS + 1 reset_error
	uint8_t packetCounter = 0;
	// Set mailbox
	packet[packetCounter++] = MAILBOX_ERROR; //Save the mailbox in [0] position and then increment the packet counter.
	packet[packetCounter++] = 0x00; // Reserve space for length
	// Copy error counters
	memcpy(&packet[packetCounter], (const void*)&errorList.errors_counters, MAX_ERRORS);
	packetCounter += MAX_ERRORS;
	// Append reset_error
	packet[packetCounter++] = errorList.reset_errors;
	// Set the total packet length (including mailbox and length byte)
	packet[1] = packetCounter;
	// Transmit packet
	return HAL_UART_Transmit_DMA(&huart2, packet, packetCounter);
}

/**
***********************************************************************
* @brief - This function return the selected error.
* @return - [bool] Returns true if there is any error pending to be read.
***********************************************************************
*/
bool any_error_active(void){
    return errorList.errorsActive == 1;
}

/**
***********************************************************************
* @brief - This function reset all the errors setting all the bits to 0.
***********************************************************************
*/
void reset_all_errors(void){
    memset((void*)&errorList, 0, sizeof(errorList));
}
/**
***********************************************************************
* @brief - This function is called to read the last reset type.
***********************************************************************
*/
static void read_reset_register(void){
    const uint16_t readResetError = *(uint16_t *)(0x015E);
    if (readResetError != 0){
        //Add the error to the error byte...
        errorList.reset_errors |= readResetError;
        trigger_error(INTERN_RESET_ERROR);
    }
}

/******************************************************************************
* LOCAL FUNCTIONS
*****************************************************************************/

/***********************************************************************
* @brief - Returns true if the timeout has been reached.
***********************************************************************
*/
inline bool tick_timeout_check(const uint16_t timeout, const uint16_t tickCounter){
    return ((uint16_t)tickCounter > (uint16_t)timeout);
}

/******************************************************************************
* EOF
*****************************************************************************/
