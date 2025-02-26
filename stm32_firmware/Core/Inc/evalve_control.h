/**
*****************************************************************************
* @file         evalve_control.h
* @brief        This is the header of the evalve control module.
* @author       Daniel Vea
* @version      1.0.0
******************************************************************************
*/

#ifndef EVALVE_CONTROL_H
#define EVALVE_CONTROL_H

/******************************************************************************
 * MODULES USED
 *****************************************************************************/
#include <stdbool.h>

/******************************************************************************
 * DEFINITIONS AND MACROS
 *****************************************************************************/

#define EVALVE_SIGNAL_PULSE     2000U     //This is the pulse duration in [ms]

/******************************************************************************
 * TYPEDEFS AND STRUCTURES
 *****************************************************************************/

/******************************************************************************
 * EXPORTED VARIABLES
 *****************************************************************************/

/******************************************************************************
 * EXPORTED FUNCTIONS
 *****************************************************************************/
void evalveControl_reset(void);
void evalveControl_init(void);
void eValveControl_main(void);
void eValveControl_millisCounter(void);
void eValveControl_setOpenTime(uint16_t openTime);
void eValveControl_closeValve(void);
void eValveControl_triggerWaterflush(void);
uint16_t eValveControl_getEvalveOpenTime(void);
uint32_t eValveControl_return_OpenCloseCounter(void);
uint32_t eValveControl_return_OpenCloseTime(void);
/******************************************************************************
 * INLINED FUNCTIONS
 *****************************************************************************/

#endif /* EVALVE_CONTROL_H */

/******************************************************************************
 * EOF
 *****************************************************************************/
