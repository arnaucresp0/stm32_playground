/**
*****************************************************************************
* @file         power_management.h
* @brief        This is the header of the power management module.
* @author       Daniel Vea
* @version      1.0.0
******************************************************************************
*/

#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

/******************************************************************************
 * MODULES USED
 *****************************************************************************/
#include <stdbool.h>

/******************************************************************************
 * DEFINITIONS AND MACROS
 *****************************************************************************/
#define POWER_OFF           false
#define POWER_ON            true
//bool to define the status of the supercapacitors.
#define CAP_DISCHARGED      false
#define CAP_CHARGED         true

/******************************************************************************
 * TYPEDEFS AND STRUCTURES
 *****************************************************************************/

typedef enum{
    SENSOR_POWER_OFF        = 0,    //Off state
    SENSOR_POWER_POWERON    = 2,    //This state is to turn the sensor on
    SENSOR_POWER_RUNNING    = 3,    //This state is to check if the sensor is running
    SENSOR_POWER_CYCLE      = 4,    //This state is to power cycle the sensor
    SENSOR_POWER_ERROR      = 5,    //This state is to check if the sensor is still connected
}sensor_power_e;

/******************************************************************************
 * EXPORTED VARIABLES
 *****************************************************************************/

/******************************************************************************
 * EXPORTED FUNCTIONS
 *****************************************************************************/

void powerMan_init(void);
void powerMan_loop(void);
void powerMan_tick_counter(void);
void powerMan_sensor_power_set(sensor_power_e powerState);
void powerMan_disable_pi_watchdog(void);

/******************************************************************************
 * INLINED FUNCTIONS
 *****************************************************************************/

#endif /* POWER_MANAGEMENT_H */

/******************************************************************************
 * EOF
 *****************************************************************************/
