/**
*****************************************************************************
* @file         power_man.c
* @brief        This is the powerManager module, it handles the power control
*               of the board like energy feedback, system power disable and
*               enable, and sensor power enable and disable.
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

#include "adc.h"
#include "power_management.h"
#include "evalve_control.h"

/******************************************************************************
* DEFINITIONS AND MACROS
*****************************************************************************/
#define KUS_OVERPOWER_LIMIT         815U    //Overpower limit (350mA aprox.)
#define KUS_OVERPOWER_WARNING       580U    //Overpower warning (250mA aprox.)

#define KUS_UNDERPOWER_WARNING      80U     //Underpower warning (80mA aprox.)
#define KUS_UNDERPOWER_LIMIT        50U     //Underpower limit (20mA aprox.)

#define RPI_WTDG_GRACE_TIME_1       5000    //This is the grace time for the raspberry to miss the first kick. (30 sec.)
#define RPI_WTDG_TIMEOUT            1000    //This is the timeout time for the raspberry to miss the second kick. (10 sec.)
#define REBOOT_DELAY_LENGTH         100     //This is the reboot timer value. (5 sec.)

#define SENSOR_POWERCYCLE_TIME      150U    //in [10ms] (1.5sec)
#define SENSOR_POWERON_DELAY        10U     //in [10ms] (100ms)
#define SENSOR_ERROR_DELAY          3U      //in [10ms] (30ms)

//Macro to turn off the sensor power.
//#define KUS_OFF                     gpio_manager_gpioClear(CTL_CARTRIDGE_EN_GPIO_Port, CTL_CARTRIDGE_EN_Pin);sensorPowerStatus = SENSOR_POWER_OFF;
//Macro to turn on the sensor power.
//#define KUS_ON                      gpio_manager_gpioSet(CTL_CARTRIDGE_EN_GPIO_Port, CTL_CARTRIDGE_EN_Pin);sensorPowerStatus = SENSOR_POWER_POWERON;
//Macro to turn off the screen power.
//#define SOM_OFF 					gpio_manager_gpioClear(CTL_SOM_EN_GPIO_Port, CTL_SOM_EN_Pin);sensorPowerStatus = SENSOR_POWER_OFF;
//Macro to turn off the screen power.
//#define SOM_ON						gpio_manager_gpioSet(CTL_CARTRIDGE_EN_GPIO_Port, CTL_CARTRIDGE_EN_Pin);sensorPowerStatus = POWERMAN_SYSTEM_POWERUP_WAIT_FIRST_KICK;

#define SCREEN_INIT_DELAY           500    //Initial screen delay value.

#define SOM_INIT_BOOT_DELAY   15000U  //Initial time to wait for the raspberry to boot up. (150sec)

/******************************************************************************
* TYPEDEFS AND STRUCTURES
******************************************************************************/

typedef enum {
    POWERMAN_WAIT_SUPERCAPS                     = 0,
    POWERMAN_SYSTEM_POWERUP_INIT_SCREEN         = 1,
    POWERMAN_SYSTEM_POWERUP_WAIT_FIRST_KICK     = 2,
    POWERMAN_SYSTEM_POWERED                     = 3,
    POWERMAN_SYSTEM_GRACE_TIME                  = 4,
    POWERMAN_SYSTEM_OFF                         = 5,
}powerMan_states_e;

/******************************************************************************
* PROTOTYPES OF LOCAL FUNCTIONS
*****************************************************************************/
static void powerMan_sensorPower(void);
static void powerMan_sensorPower_check(void);
static bool powerMan_superCap_is_charged(void);
static bool powerMan_backup_check(void);
static bool power_tick_timeout_check(const uint16_t timeout, const uint16_t tickCounter);

/******************************************************************************
* LOCAL VARIABLES
*****************************************************************************/
static sensor_power_e sensorPowerStatus = SENSOR_POWER_OFF;
static volatile uint16_t powerScreen_tick, powerSensor_tick;
static volatile powerMan_states_e powerMan_state = POWERMAN_WAIT_SUPERCAPS;

/**
***********************************************************************
* @brief - Initialize power related GPIOs
***********************************************************************
*/
void powerMan_init(void){
	powerScreen_tick = 0;
    powerSensor_tick = 0;
}

/**
***********************************************************************
* @brief - This is the main state machine function for the power
*          management module.
***********************************************************************
*/
void powerMan_loop(void){
    /*if (powerMan_backup_check() == POWER_OFF){      //Check the backup pin to close the EValve and shutdown the system.
        if (powerMan_state != POWERMAN_SYSTEM_OFF){     //Execute just once
            eValveControl_closeValve();                     //Close the eValve.
            //trigger_error(HUB_POWER_DOWN_ERROR); //Trigger the error.
        }
        powerMan_state = POWERMAN_SYSTEM_OFF;
    }*/

    switch(powerMan_state){
        default:
        case(POWERMAN_SYSTEM_OFF):     //Turn off the system safely
            //Turn off the screen and raspberry. And the sensor.
            //KUS_OFF;                            //Turn off the sensor power enable.
        	//SOM_OFF;							//Turn off the screen power enable.
            //This way the reset is holded for 1 second.
            if (power_tick_timeout_check(REBOOT_DELAY_LENGTH, powerScreen_tick) == false) return;  //Wait 1 sec before changing states...
            powerMan_state = POWERMAN_WAIT_SUPERCAPS;
            return;
        case(POWERMAN_WAIT_SUPERCAPS):  //init state, waits for the supercaps...
            if (powerMan_superCap_is_charged() == CAP_CHARGED){
                //Turn on raspberry
                powerMan_state = POWERMAN_SYSTEM_POWERUP_INIT_SCREEN;
            }   //Otherwise it will never leave this state...
            return;
        case(POWERMAN_SYSTEM_POWERUP_INIT_SCREEN):     //Initialize the screen after 5 seconds
            if (power_tick_timeout_check(SCREEN_INIT_DELAY, powerScreen_tick) == false) return;  //If no timeout, stop here
            //SOM_ON;
        	powerMan_state = POWERMAN_SYSTEM_POWERUP_WAIT_FIRST_KICK;
            break;  //Reset the tick counter
        case(POWERMAN_SYSTEM_POWERUP_WAIT_FIRST_KICK):  //Delay state...
            if (power_tick_timeout_check(SOM_INIT_BOOT_DELAY, powerScreen_tick) == false) return;  //If no timeout, stop here
            //If the first kick is not received, turn off the raspberry
            powerMan_state = POWERMAN_SYSTEM_POWERED;
            break;  //Reset the watchdog tick counter
        case(POWERMAN_SYSTEM_POWERED):  //System is powered. Keep checking the power and watchdog.
            //powerMan_sensorPower();
            //If no timeout, stop here. Ignore the watchdog if the flag is set.
            if ((power_tick_timeout_check(RPI_WTDG_GRACE_TIME_1, powerScreen_tick) == false)) return;
            //If the first kick jump to the grace time
            powerMan_state = POWERMAN_SYSTEM_GRACE_TIME;
            break;  //Reset the watchdog tick counter
        case(POWERMAN_SYSTEM_GRACE_TIME):   //Grace time to receive the missed kick
            if (power_tick_timeout_check(RPI_WTDG_TIMEOUT, powerScreen_tick) == false) return;  //If no timeout, stop here
            //If the first kick is not received, turn off the SOM
            powerMan_state = POWERMAN_SYSTEM_OFF;
            //trigger_error(PI_WTDG_TRIGGERED); //Trigger the error.
            break;  //Reset the watchdog tick counter
    }
    powerScreen_tick = 0; //Reset the tick counter
}

/**
***********************************************************************
* @brief - Function that increases the tick counter every 1ms
***********************************************************************
*/
void powerMan_tick_counter(void){
	powerScreen_tick++;
	powerSensor_tick++;
}

/**
***********************************************************************
* @brief - Changes the sensor power to ON or OFF
* @param - [powerState] If the power sensor is ON (true) or OFF (false)
***********************************************************************
*/
void powerMan_sensor_power_set(sensor_power_e powerState){
    sensorPowerStatus = powerState;
    powerSensor_tick = 0;
}

/******************************************************************************
* LOCAL FUNCTIONS
*****************************************************************************/

/***********************************************************************
* @brief - Returns true if the timeout has been reached.
***********************************************************************
*/
static bool power_tick_timeout_check(const uint16_t timeout, const uint16_t tickCounter){
    return ((uint16_t)tickCounter > (uint16_t)timeout);
}

/**
***********************************************************************
* @brief - Changes the sensor power to ON or OFF
***********************************************************************
*/
static void powerMan_sensorPower(void){
    switch(sensorPowerStatus){
        default:
        case(SENSOR_POWER_OFF):
            //KUS_OFF;
            break;
        case(SENSOR_POWER_CYCLE):
            //Turn off
            //gpio_manager_gpioClear(CTL_CARTRIDGE_EN_GPIO_Port,CTL_CARTRIDGE_EN_Pin);
            if (power_tick_timeout_check(SENSOR_POWERCYCLE_TIME, powerSensor_tick) == false) return;  //If no timeout, stop here
            powerSensor_tick = 0;
            //turn on after the timeout
        case(SENSOR_POWER_POWERON):
            //KUS_ON;
            if (power_tick_timeout_check(SENSOR_POWERON_DELAY, powerSensor_tick) == false) return;  //If no timeout, stop here
            sensorPowerStatus = SENSOR_POWER_RUNNING;
            break;
        case(SENSOR_POWER_ERROR):   //This state is to check if the sensor is still connected
            //Wait before powering down the sensor, and check again
            if (power_tick_timeout_check(SENSOR_ERROR_DELAY, powerSensor_tick) == false) return;  //If no timeout, stop here
            //If the sensor is still not connected, turn off the sensor.
        case(SENSOR_POWER_RUNNING):
            powerMan_sensorPower_check();   //keep checking the sensor power.
            break;
    }
}

/**
***********************************************************************
* @brief - Check the sensor power status
***********************************************************************
*/
static void powerMan_sensorPower_check(void){
    //If the previous state was an error stop powering the sensor
    const sensor_power_e prevSensorStatusPower = sensorPowerStatus;
    const uint16_t pwrSensor = adc_manager_read(ADC_CHANNEL_A1); //Read the sensor power

    if (pwrSensor > KUS_OVERPOWER_LIMIT){               //OVERPOWER ERROR (815)
        if (prevSensorStatusPower == SENSOR_POWER_ERROR){
            //trigger_error(KUS_OVERPOWER_ERROR);
            sensorPowerStatus = SENSOR_POWER_OFF;
            return;
        }
        sensorPowerStatus = SENSOR_POWER_ERROR;
        powerSensor_tick = 0; //Reset the tick counter
     }else if(pwrSensor > KUS_OVERPOWER_WARNING){       //OVERPOWER WARNING (580)
        //trigger_warning(KUS_OVERPOWER_WARN);
     }else if (pwrSensor  < KUS_UNDERPOWER_LIMIT){      //UNDERPOWER ERROR (50)
        if (prevSensorStatusPower == SENSOR_POWER_ERROR){
            //trigger_error(KUS_UNDERPOWER_ERROR);
            sensorPowerStatus = SENSOR_POWER_OFF;
            return;
        }
        sensorPowerStatus = SENSOR_POWER_ERROR;
        powerSensor_tick = 0; //Reset the tick counter
     }else if (pwrSensor  < KUS_UNDERPOWER_WARNING){    //UNDERPOWER WARNING (160)
        //trigger_warning(KUS_UNDERPOWER_WARN);
     }else{                                             //POWER NORMAL
        //Clear both warning cases
        //clear_warning(KUS_UNDERPOWER_WARN);
        //clear_warning(KUS_OVERPOWER_WARN);
        sensorPowerStatus = SENSOR_POWER_RUNNING;
     }
}

/**
***********************************************************************
* @brief - Check the state of the backup pin to close the EValve
* \b note:  The backupState comes from reading the BCK_UP pin that is set to HIGH in normal mode
*           and when it is LOW it means that the main power supply is off and the system needs to
*           close the EValve urgently.
***********************************************************************
*/
static bool powerMan_backup_check(void){
    const uint8_t backup_pin_Val = GPIO_PIN_SET; //gpio_manager_gpioReadInput(CTL_BAT_PG_GPIO_Port, CTL_BAT_PG_Pin);
    if (GPIO_PIN_SET ==  backup_pin_Val){
        return POWER_OFF;
    }else{
        return POWER_ON;
    }
}

/**
***********************************************************************
* @brief - Check the state of the backup ready pin to check that the capacitors are charged
* @param - Returns true if the capacitors are charged and false if they are not.
***********************************************************************
*/
static bool powerMan_superCap_is_charged(void){
    if (GPIO_PIN_RESET == 0) { //gpio_manager_gpioReadInput(CTL_BAT_STAT_GPIO_Port, CTL_BAT_STAT_Pin))
        return CAP_CHARGED;
        //Status Led is turned on to indicate the SuperCap is charged.
    }else{
        return CAP_DISCHARGED;
    }
	return CAP_CHARGED;
}

/******************************************************************************
* EOF
*****************************************************************************/
