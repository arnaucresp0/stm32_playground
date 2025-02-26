/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

typedef enum{
    ADC_CHANNEL_A0      = 0, //CTL_EVALVE_FEEDBACK CHANNEL
    ADC_CHANNEL_A1      = 1, //**not used**
    ADC_CHANNEL_A2      = 2, //**not used**
    ADC_CHANNEL_A3      = 3, //**not used**
    ADC_CHANNEL_A4      = 4, //**debug**
    ADC_CHANNEL_A5      = 5, //CTL_CARTRIDGE_FEEDBACK CHANNEL
    ADC_CHANNEL_A6      = 6, //**not used**
    ADC_CHANNEL_A7      = 7, // **debug**
	ADC_CHANNEL_A8	  = 8, //**not used**
	ADC_CHANNEL_A9		= 9, //CTL_BATTERY FEEDBACK CHANNEL
    ADC_CHANNEL_MAX,
}adc_channel_e;

void MX_ADC_Init(void);
void adc_manager_start(void);
uint32_t adc_manager_read(adc_channel_e channel);


#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
