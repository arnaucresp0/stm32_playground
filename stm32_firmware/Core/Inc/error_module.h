/**
 ******************************************************************************
 * @file    error_module.h
 * @brief   Header for error_module.c
 * @author  Arnau Crespo
 * @version 1.0.0
 ******************************************************************************
 */

#ifndef ERROR_MODULE_H
#define ERROR_MODULE_H

/******************************************************************************
 * MODULES USED
 *****************************************************************************/
#include <stdbool.h>

/******************************************************************************
 * DEFINITIONS AND MACROS
 *****************************************************************************/

/******************************************************************************
 * TYPEDEFS AND STRUCTURES
 *****************************************************************************/
typedef enum{
    HUB_POWER_DOWN_ERROR        = 0,    //When the screen has lost power
    EVALVE_OVERPOWER_ERROR      = 1,    //When the evalve is consuming more power than expected
    EVALVE_UNDERPOWER_ERROR     = 2,    //When the evalve is consuming less power than expected
    KUS_OVERPOWER_ERROR         = 3,    //When the KUS is consuming more power than expected
    KUS_UNDERPOWER_ERROR        = 4,    //When the KUS is consuming less power than expected
    UART_COMMAND_NOT_VALID      = 5,    //When the command received is not valid
    UART_WRONG_PACKET           = 6,    //When the packet received is not valid
    UART_NO_DATA                = 7,    //When the uart has timedout
    UART_CONFIG_NOT_VALID       = 8,   //When the uart configuration command is not valid
    PI_WTDG_TRIGGERED           = 9,   //When the Pi watchdog is triggered
    PI_UART_FRAMING_ERROR       = 10,   //When the Pi uart has a framing error
    PI_UART_PARITY_ERROR        = 11,   //When the Pi uart has a parity error
    PI_UART_OVERRUN_ERROR       = 12,   //When the Pi uart has an overrun error
    KUS_UART_FRAMING_ERROR      = 13,   //When the KUS uart has a framing error
    KUS_UART_PARITY_ERROR       = 14,   //When the KUS uart has a parity error
    KUS_UART_OVERRUN_ERROR      = 15,   //When the KUS uart has an overrun error
    INTERN_RESET_ERROR          = 16,   //The reason for the MSP430 reset
	NO_ERROR                    = 0xFF, //This one indicates that there is no error active
	MAX_ERRORS                  ,       //This one indicates the maximum size of errors possible

}error_type_e;

typedef enum{
    NO_RESET_ERROR              = 0x0,
    BROWNOUT                    = 0x2,
    RSTIFG_BOR                  = 0x4,
    PMMSW_BOR                   = 0x6,
    SVSHIFG_BOR                 = 0xE,
    PMMSW_POR                   = 0x14,
    WTDG_TIMEOUT_PUC            = 0x16,
    FRAM_BIT_PUC                = 0x1C,
    PERIPHERAL_AREA_FETCH_PUC   = 0x1E,
    FLL_UNLOCK_PUC              = 0x24,
}reset_errors_e;


/******************************************************************************
 * EXPORTED VARIABLES
 *****************************************************************************/


/******************************************************************************
 * EXPORTED FUNCTIONS
 *****************************************************************************/
void error_pending_tick_counter(void);
void error_module_main(void);
void trigger_error(error_type_e error);
bool any_error_active(void);
void reset_all_errors(void);
void error_module_init(void);
void error_module_send_errors(void);

/******************************************************************************
 * INLINED FUNCTIONS
 *****************************************************************************/

#endif /* ERROR_MODULE */

/******************************************************************************
 * EOF
 *****************************************************************************/
