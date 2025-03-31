/**
*****************************************************************************
* @file         uart_control.c
* @brief        This is the module that handles UART comms
* @author       Daniel Vea
* @version      1.0.0
******************************************************************************
*/

#define UART_MAN_C

/******************************************************************************
* MODULES USED
*****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "uart_control.h"
#include "usart.h"
#include "evalve_control.h"
#include "utils/circular_buffer.h"
#include "version.h"

/******************************************************************************
* DEFINITIONS AND MACROS
*****************************************************************************/
#define SERIAL_MAGIC_PACKET_SIZE        3
#define ECC_DATA_LENGTH                 8
#define SERIAL_MAX_TX_DATA_PACKET_SIZE  240
#define SERIAL_MAILBOX_SIZE             1
#define SERIAL_DATA_LENGTH_SIZE         1
#define SERIAL_PACKET_LENGTH            SERIAL_MAX_TX_DATA_PACKET_SIZE + ECC_DATA_LENGTH + SERIAL_MAILBOX_SIZE + SERIAL_DATA_LENGTH_SIZE
#define SERIAL_PACKET_LENGTH_FULL       SERIAL_PACKET_LENGTH + SERIAL_MAGIC_PACKET_SIZE    //Includes magic packet
#define POWEROFTWO_MEMORY_SIZE          256
#define SERIAL_PI_TIMEOUT               2           //In [0.1ms]
#define SERIAL_KUS_TIMEOUT              200         //In [0.1ms]. This timeout determines when the channel is free

/******************************************************************************
* TYPEDEFS AND STRUCTURES
******************************************************************************/

typedef enum{
    UART_STATE_CLEAN_STATE              = 0,
    UART_STATE_WAITING_DATA             = 1,
    UART_STATE_WAITING_MAILBOX          = 2,
    UART_STATE_MCU_COMMAND_DETECTED     = 3,
}uart_control_states_e;

typedef enum{
    UART_CONFIG_NOT_RECOGNIZED         = 0,
    UART_WATERFLUSH_CONFIG             = 1,
    UART_SCREEN_DC_CONFIG              = 2,
}uart_config_e;

typedef struct{
    uint8_t     mailbox;
    uint8_t     length;
    uint8_t     data[SERIAL_PACKET_LENGTH - 8];  //Includes 8 byte for ECC
}SendSerialMessage_t;

typedef struct{
    uint16_t                        tickCounter;
    uint8_t                         circularArray[POWEROFTWO_MEMORY_SIZE * 2];
    uint8_t                         auxMessage[SERIAL_PACKET_LENGTH_FULL];
    uart_control_states_e state;
    ring_buffer_t                   circularBuffer;
    SendSerialMessage_t 			serialMessagePendingToBeSent;
}data_buffer_t;


/******************************************************************************
* PROTOTYPES OF LOCAL FUNCTIONS
*****************************************************************************/

static bool uart_mailbox_checker(uint8_t mailbox);
static void uart_execute_commands(uint8_t *data);
bool uart_tick_timeout_check(const uint16_t timeout, const uint16_t tickCounter);
void uart_tick_counter_reset(volatile uint16_t *counter);
static void uart_main_state_machine(void);
static volatile uart_manager_error_t errorsPi;

/******************************************************************************
* LOCAL VARIABLES
*****************************************************************************/

static const  uint8_t CARTRIDGE_SERIAL_MAGIC_NUMBER[SERIAL_MAGIC_PACKET_SIZE] = {0x0F, 0x01, 0xFA};
static volatile data_buffer_t DataBuffer;
static uint8_t byteCounter = 0;
static uint8_t receivedByte = 0;
/******************************************************************************
* EXPORTED FUNCTIONS
*****************************************************************************/

/**
***********************************************************************
* @brief - This function initialize the UART
***********************************************************************
*/
void uartControl_init(void){
    //Initialize UART manager
	MX_USART2_UART_Init();
    ring_buffer_init((ring_buffer_t*)&(DataBuffer.circularBuffer), (char*)DataBuffer.circularArray, sizeof(DataBuffer.circularArray));
    DataBuffer.state = UART_STATE_CLEAN_STATE;
    HAL_UART_Receive_IT(&huart2, &receivedByte, 1);
}

/**
***********************************************************************
* @brief - This function needs to be called every 1ms
***********************************************************************
*/
void uartControl_tick_counter(void){
    ++(DataBuffer.tickCounter);
}

/**
***********************************************************************
* @brief - This function is called every time the UART receives a Byte
***********************************************************************
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    // Store the received byte
    ring_buffer_queue((ring_buffer_t*)&(DataBuffer.circularBuffer), receivedByte);

    // Reset the timeout counter
    uart_tick_counter_reset(&(DataBuffer.tickCounter));

    // Restart UART reception for the next byte
    HAL_UART_Receive_IT(&huart2, &receivedByte, 1);
}


/**
***********************************************************************
* @brief - This is the main function of the UART app.
***********************************************************************
*/
void uartControl_main(void){
	uart_main_state_machine();
    //const error_type_e errors = uart_manager_read_uart_errors();
    //if (errors != NO_ERROR){
        //trigger_error(errors);
    //}
}

/******************************************************************************
* LOCAL FUNCTIONS
*******************************************************************************/

/**
***********************************************************************
* @brief - This is the main state machine for receiving data from PI. It will keep
sending the data asynchronously to the KUS UART until there is a match with the
magic bytes and a recognized command. Once the command is recognized, it will
wait for the whole packet to be received and then it will process it. Afterwards
the command will be executed.
***********************************************************************
*/
static void uart_main_state_machine(void){
    uint8_t localAuxMessage[SERIAL_PACKET_LENGTH];
    uint8_t counter = 0;

    switch(DataBuffer.state){
        default:
        case(UART_STATE_CLEAN_STATE):   //Clean variables and reset some stuff
			__HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);
			__HAL_UART_CLEAR_IT(&huart2, UART_IT_TXE);
			DataBuffer.state = UART_STATE_WAITING_DATA;
			byteCounter = 0;
			DataBuffer.serialMessagePendingToBeSent.mailbox = 0;
			DataBuffer.tickCounter = 0;
			// Fall through
            //Jump straight to waiting data...
			break;
        case UART_STATE_WAITING_DATA: //Wait for new data to read or send...
			// CHECK FOR INCOMING DATA
			while ((counter < SERIAL_MAGIC_PACKET_SIZE) &&
				   ring_buffer_peek((ring_buffer_t*)&(DataBuffer.circularBuffer), (char*)localAuxMessage, counter)) {
				if (localAuxMessage[0] != CARTRIDGE_SERIAL_MAGIC_NUMBER[counter]) {
					ring_buffer_dequeue((ring_buffer_t*)&(DataBuffer.circularBuffer), (char*)localAuxMessage);
					counter = 0;
				}
				else counter++;
			}
			if (counter == SERIAL_MAGIC_PACKET_SIZE) {
				DataBuffer.state = UART_STATE_WAITING_MAILBOX;
			}
			return; //Do not check the timer...

        case (UART_STATE_WAITING_MAILBOX):
            //If the magic packet is found, then check for the 4th byte
            if (ring_buffer_peek((ring_buffer_t*)&(DataBuffer.circularBuffer), (char*)localAuxMessage, SERIAL_MAGIC_PACKET_SIZE) == true){
                DataBuffer.state = UART_STATE_MCU_COMMAND_DETECTED;                   //This is a command for the MCU
                //if the mailbox is not valid, then this data will be sent to the kus
                if (uart_mailbox_checker(localAuxMessage[0]) == false){
                    break; //Do nothing because this UART do not talk to KUS
				}//otherwise this data is an MSP command
			}
            break;  //Keep checking the timer.

        case(UART_STATE_MCU_COMMAND_DETECTED):
            while (ring_buffer_dequeue((ring_buffer_t*)&(DataBuffer.circularBuffer), (char*)localAuxMessage) == true){
                //keep popping the data until the full packet is received
                DataBuffer.auxMessage[byteCounter] = localAuxMessage[0];
                byteCounter++;
                if (byteCounter >= SERIAL_PACKET_LENGTH_FULL){
                    //Exclude the magic packet from the message
                    if (DataBuffer.auxMessage[SERIAL_MAGIC_PACKET_SIZE] == 0){
                        uart_execute_commands(localAuxMessage);       //Execute the received command
                    }
                    else{
                    	//Wrong packet
                    }
					DataBuffer.state = UART_STATE_CLEAN_STATE;      //Clean variables
					break;
                }
            }
            break;  //If not enough data, keep checking the timer.
    }

    if (uart_tick_timeout_check(SERIAL_PI_TIMEOUT, DataBuffer.tickCounter) == true){   //If in 0.4ms there is no more data, timeout occurs
        DataBuffer.state = UART_STATE_CLEAN_STATE;       //Clean variables
    }
}

/*
***********************************************************************
* @brief - Goes back to the first state of the state machine and resets
* some variables.
***********************************************************************
*/
void uart_tick_counter_reset(volatile uint16_t *counter){
    *counter = 0;
}

/*
***********************************************************************
* @brief - Returns true if the timeout has been reached.
***********************************************************************
*/
bool uart_tick_timeout_check(const uint16_t timeout, const uint16_t tickCounter){
    return ((uint16_t)tickCounter > (uint16_t)timeout);
}

/*
***********************************************************************
* @brief - This function checks if the received mailbox is any of the
* mailboxes that can trigger actions on this MCU.
* @param - [mailbox] Mailbox to be checked
* \b note:  This function returns true if there is a match, false otherwise
***********************************************************************
*/
static void uart_execute_commands(uint8_t *data){
    const uint8_t mailbox = (uint8_t)data[0];
    //const uint8_t length = (uint8_t)data[1];
    if (mailbox == (uint8_t)MAILBOX_COMMANDS){
        const uart_commands_e command = (uart_commands_e)data[2];
        switch(command){
            default:
            case(UART_COMMAND_NOT_RECOGNIZED):
                //trigger_error(UART_COMMAND_NOT_VALID);
                break;
            case(UART_COMMAND_TRIGGER_WATERFLUSH):
                eValveControl_triggerWaterflush();
                break;
            case(UART_COMMAND_POWER_CYCLE_CARTRIDGE):
                //powerMan_sensor_power_set(SENSOR_POWER_CYCLE);
                break;
            case(UART_COMMAND_POWER_OFF_CARTRIDGE):
                //powerMan_sensor_power_set(SENSOR_POWER_OFF);
                break;
            case(UART_COMMAND_RESET_COUNTERS):
                //evalveControl_reset();           //Reset the eValve statistics counters
                break;
            case(UART_COMMAND_RESET_ERRORS):
                //reset_all_errors();
                break;
            case(UART_COMMAND_SEND_ERRORS):
                //error_module_send_errors();
                break;
            case(UART_COMMAND_DISABLE_PI_WATCHDOG):
                //powerMan_disable_pi_watchdog();
                break;
        }
    }else if (mailbox == (uint8_t)MAILBOX_CONFIG){
        const uart_config_e config = (uart_config_e)data[2];
        const uint16_t openTime = ((uint8_t)data[3] << 8) | (uint8_t)data[4];
        switch(config){
            default:
            case(UART_CONFIG_NOT_RECOGNIZED):
                //trigger_error(UART_CONFIG_NOT_VALID);
                break;
            case(UART_WATERFLUSH_CONFIG):
                //eValveControl_setOpenTime(openTime);
                break;
        }
    }
}

/*
***********************************************************************
* @brief - This function checks if the received mailbox is any of the
* mailboxes that can trigger actions on this MCU.
* @param - [mailbox] Mailbox to be checked
* \b note:  This function returns true if there is a match, false otherwise
***********************************************************************
*/
static bool uart_mailbox_checker(uint8_t mailbox){
    switch((uart_mailboxes_e) mailbox){
        default:
            return false;
        case(MAILBOX_COMMANDS):
        case(MAILBOX_CONFIG):
            return true;
    }
}

bool uartControl_message_creator(uart_mailboxes_e mailbox, const uint8_t length, uint8_t* data){
    //Check if there is any pending message in memory. If mailbox equals 0 it means that its empty.
	DataBuffer.serialMessagePendingToBeSent.mailbox = mailbox;                                    		// Mailbox
	DataBuffer.serialMessagePendingToBeSent.length = length;                                      		// Length of data
	memcpy((void*)DataBuffer.serialMessagePendingToBeSent.data, data, sizeof(uint8_t) * length);         //Copy the message data
	return true;
}


/******************************************************************************
* EOF
*****************************************************************************/
