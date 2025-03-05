/**
*****************************************************************************
* @file         uart_control.h
* @brief        This is the UART control module that handles both UART channels
* @author       Daniel Vea
* @version      1.0.0
******************************************************************************
*/

#ifndef UART_CONTROL_H
#define UART_CONTROL_H

/******************************************************************************
 * MODULES USED
 *****************************************************************************/

/******************************************************************************
 * DEFINITIONS AND MACROS
 *****************************************************************************/

/******************************************************************************
 * TYPEDEFS AND STRUCTURES
 *****************************************************************************/

//Those are the available mailboxes for powerhub communications
typedef enum{
    MAILBOX_MIN             = 240,  //Reserved
    MAILBOX_COMMANDS        = 241,  //Mailbox to receive commands from scrapp
    MAILBOX_CONFIG          = 242,
    MAILBOX_INFO            = 243,  //hw version, fw version...
    MAILBOX_ERROR           = 244,  //to report the errors...
    MAILBOX_WARNING         = 245,   //to report the warnings...
    MAILBOX_MAX             = 250,  //Reserved
}uart_mailboxes_e;

typedef enum{
    UART_COMMAND_NOT_RECOGNIZED         = 0,
    UART_COMMAND_TRIGGER_WATERFLUSH     = 1,
    UART_COMMAND_POWER_CYCLE_CARTRIDGE  = 2,
    UART_COMMAND_POWER_OFF_CARTRIDGE    = 3,
    UART_COMMAND_RESET_ERRORS           = 4,
    UART_COMMAND_RESET_COUNTERS         = 5,
    UART_COMMAND_SEND_ERRORS            = 6,
    UART_COMMAND_DISABLE_PI_WATCHDOG    = 7,
}uart_commands_e;

typedef struct{
    uint8_t     framingError    : 1;
    uint8_t     parityError     : 1;
    uint8_t     receiveOverrun  : 1;
}uart_manager_error_t;

/******************************************************************************
 * EXPORTED VARIABLES
 *****************************************************************************/

/******************************************************************************
 * EXPORTED FUNCTIONS
 *****************************************************************************/

void uartControl_main(void);
void uartControl_init(void);
void uartControl_tick_counter(void);
bool uartControl_message_creator(uart_mailboxes_e mailbox, const uint8_t length, uint8_t* data);
void rxCallbackUART(uint8_t receivedByte);
void txCallbackUART(void);

/******************************************************************************
 * INLINED FUNCTIONS
 *****************************************************************************/

#endif /* UART_CONTROL_H */

/******************************************************************************
 * EOF
 *****************************************************************************/
