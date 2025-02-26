/**
*****************************************************************************
* @file         evalve_control.c
* @brief        This is the evalve control module. Controls the evalve status
*               and actions.
* @author       Daniel Vea
* @version      1.0.0
******************************************************************************
*/

#define POWER_MAN_C

/******************************************************************************
* MODULES USED
*****************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "stm32f0xx_hal.h"
#include "evalve_control.h"
#include "adc.h"
#include "flash.h"

/******************************************************************************
* DEFINITIONS AND MACROS
*****************************************************************************/
#define EV_OVERPOWER_LIMIT          1000      //Overpower limit
#define EV_OVERPOWER_WARNING        900      //Overpower warning

#define EV_UNDERPOWER_LIMIT         100      //Underpower limit
#define EV_UNDERPOWER_WARNING       400      //Underpower warning

#define FLASH_ADDR_COUNTER     (FLASH_USER_START_ADDR)       // Address for OpenCloseCounter
#define FLASH_ADDR_TOTAL_TIME  (FLASH_USER_START_ADDR + 4)   // Address for TotalCycleTimes

/******************************************************************************
* TYPEDEFS AND STRUCTURES
******************************************************************************/

typedef enum {
    EVALVE_CONTROL_NO_FLUSH_STATE               = 0,    //NOP
    EVALVE_CONTROL_RISING_START_FLUSHING_STATE  = 1,    //RISING START
    EVALVE_CONTROL_PULSE_START_FLUSHING_STATE   = 2,    //20ms pulse
    EVALVE_CONTROL_ENDING_START_FLUSHING_STATE  = 3,    //ENDING START
    EVALVE_CONTROL_FLUSHING_STATE               = 4,    //leave the valve open
    EVALVE_CONTROL_RISING_STOP_FLUSHING_STATE   = 5,    //RISING STOP
    EVALVE_CONTROL_PULSE_STOP_FLUSHING_STATE    = 6,    //20ms pulse
    EVALVE_CONTROL_ENDING_STOP_FLUSHING_STATE   = 7,    //ENDING STOP
}eValve_control_state_machine_e;

typedef enum{
    EVALVE_STATUS_NOP    = 0,
    EVALVE_STATUS_CLOSE  = 1,
    EVALVE_STATUS_OPEN   = 2,
}eValve_status_e;


/******************************************************************************
* PROTOTYPES OF LOCAL FUNCTIONS
*****************************************************************************/

static void evalveControl_changeStatus(eValve_status_e state);
static void eValveControl_OpenCloseCycles(void);
inline bool uart_tick_timeout_check(const uint16_t timeout, const uint16_t tickCounter);
/******************************************************************************
* LOCAL VARIABLES
*****************************************************************************/

//State machine variable
static eValve_control_state_machine_e evalveControlState = EVALVE_CONTROL_NO_FLUSH_STATE;
//This variable counts how many milliseconds has passed.
volatile static uint16_t millisecCounter = 0;
//This variable indicates how long the valve needs to be open in [ms].
static uint16_t eValveOpenTime = 2000;
//This variable that will count the open-close cycles
static uint32_t OpenCloseCounter = 0;
//This varaiable that will sum the eValve open-close cycle time
static uint32_t TotalCycleTimes = 0;
static uint32_t totalAdcSum = 0;
static uint16_t sampleCounter = 0;

/******************************************************************************
* EXPORTED FUNCTIONS
*****************************************************************************/
/**
***********************************************************************
* @brief - Initialize eValve related GPIOs
***********************************************************************
*/

void evalveControl_reset(void){
    TotalCycleTimes = 0;
    OpenCloseCounter = 0;
}


/**
***********************************************************************
* @brief - Initialize eValve related GPIOs
***********************************************************************
*/
void evalveControl_init(void){
    //State machine initial state
    evalveControlState = EVALVE_CONTROL_NO_FLUSH_STATE;
    millisecCounter = 0;
    //Start the device with the watervalve closed.
    eValveControl_closeValve();
    Load_From_Flash(FLASH_OPENCLOSE_ADDR, &OpenCloseCounter, sizeof(OpenCloseCounter));
    Load_From_Flash(FLASH_CYCLETIME_ADDR, &TotalCycleTimes, sizeof(TotalCycleTimes));
}

/*
***********************************************************************
* @brief - Returns true if the timeout has been reached.
***********************************************************************
*/
inline bool uart_tick_timeout_check(const uint16_t timeout, const uint16_t tickCounter){
    return ((uint16_t)tickCounter > (uint16_t)timeout);
}

static void read_valve_signal_pulse(void){
    const uint16_t eValvePwr = adc_manager_read(0);
    totalAdcSum += eValvePwr;
    ++sampleCounter;
}

/**
***********************************************************************
* @brief - Main state machine for the electrovalve control.
* \b note:  This function needs to be called every 1ms.
***********************************************************************
*/

void eValveControl_main(void){
    eValve_status_e evalveStatus = EVALVE_STATUS_NOP;   //Set evalve control to 0-0

    switch(evalveControlState){
        default:
        case(EVALVE_CONTROL_ENDING_STOP_FLUSHING_STATE):
        case(EVALVE_CONTROL_NO_FLUSH_STATE):    //Do nothing on the eValve. It will default to "NOP"
            break;
        case(EVALVE_CONTROL_RISING_START_FLUSHING_STATE):   //Open eValve
            evalveStatus = EVALVE_STATUS_OPEN; //Set eValve control lines to 1-0
            evalveControlState = EVALVE_CONTROL_PULSE_START_FLUSHING_STATE;
            break;      //Reset milliseconds timer counter
        case(EVALVE_CONTROL_PULSE_STOP_FLUSHING_STATE):
        case(EVALVE_CONTROL_PULSE_START_FLUSHING_STATE):
            //This state is to create a pulse of EVALVE_SIGNAL_PULSE duration
            read_valve_signal_pulse();
            if (millisecCounter >= EVALVE_SIGNAL_PULSE)return;
            //check_valve_signal_pulse();
            //EVALVE_CONTROL_ENDING_START_FLUSHING_STATE
            evalveControlState = (eValve_control_state_machine_e)((uint8_t)evalveControlState + 1);
            return;
        case(EVALVE_CONTROL_ENDING_START_FLUSHING_STATE):   //Stop opening eValve after
            //Set evalve control to 0-0
            //Set the delay counter to how long the eValve needs to be open...
            evalveControlState = EVALVE_CONTROL_FLUSHING_STATE;
            break;      //Reset milliseconds timer counter
        case(EVALVE_CONTROL_FLUSHING_STATE):   //Wait for a while with the eValve open.
			if (millisecCounter >= eValveOpenTime)return;
            evalveControlState = EVALVE_CONTROL_RISING_STOP_FLUSHING_STATE;
            return;
        case(EVALVE_CONTROL_RISING_STOP_FLUSHING_STATE):    //Start closing the evalve.
            evalveStatus = EVALVE_STATUS_CLOSE;
            evalveControlState = EVALVE_CONTROL_PULSE_STOP_FLUSHING_STATE;
            eValveControl_OpenCloseCycles();
            break;          //Reset milliseconds timer counter
    }
    evalveControl_changeStatus(evalveStatus); //Set eValve control lines
    millisecCounter = 0;        //Reset milliseconds timer counter
}

uint16_t eValveControl_getEvalveOpenTime(void){
    return eValveOpenTime;
}

void eValveControl_millisCounter(void){
    //This counter can run forever because we are resetting it before checking it.
    millisecCounter++;
}

/**
***********************************************************************
* @brief - Sets for how long the eValve needs to be open.
* @param - [openTime] Evalve open time in [ms]
* \b note:  This function needs to be called every 1ms.
***********************************************************************
*/
void eValveControl_setOpenTime(uint16_t openTime){
    if (openTime > 0){
        eValveOpenTime = openTime;
    }
}

/**
***********************************************************************
* @brief - Closes the watervalve.
***********************************************************************
*/
void eValveControl_closeValve(void){
    evalveControlState = EVALVE_CONTROL_RISING_STOP_FLUSHING_STATE;
}

/**
***********************************************************************
* @brief - Triggers the sequence to open the valve and release a flush
***********************************************************************
*/
void eValveControl_triggerWaterflush(void){
    evalveControlState = EVALVE_CONTROL_RISING_START_FLUSHING_STATE;
}

uint32_t eValveControl_return_OpenCloseCounter(void){
    return OpenCloseCounter;
}

uint32_t eValveControl_return_OpenCloseTime(void){
    return TotalCycleTimes;
}

/******************************************************************************
* LOCAL FUNCTIONS
*****************************************************************************/

/**
***********************************************************************
* @brief - Opens the electrovalve
* @param - [powerState] If the power sensor is ON (true) or OFF (false)
***********************************************************************
*/
static void evalveControl_changeStatus(eValve_status_e state){
    //Change eValve state
    switch(state){
        default:
        case(EVALVE_STATUS_NOP):    //0-0
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            break;
        case(EVALVE_STATUS_OPEN):   //0-1
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            break;
        case(EVALVE_STATUS_CLOSE):   //1-0
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            break;
    }
}
/**
***********************************************************************
* @brief - Counts the open/close cycles and save it to FLASH
* @param - [param] ...
* @param - [param2] ...
* \b note:  More info if needed
***********************************************************************
*/
static void eValveControl_OpenCloseCycles(void){
    //If the eValve was opened then count the open/close cycle
    OpenCloseCounter ++;
    //Sum the openTime
    TotalCycleTimes += eValveOpenTime; //Divide to get the value in seconds
    Save_To_Flash(FLASH_OPENCLOSE_ADDR, &OpenCloseCounter, sizeof(OpenCloseCounter));
    Save_To_Flash(FLASH_CYCLETIME_ADDR, &TotalCycleTimes, sizeof(TotalCycleTimes));

}



/******************************************************************************
* EOF
*****************************************************************************/
