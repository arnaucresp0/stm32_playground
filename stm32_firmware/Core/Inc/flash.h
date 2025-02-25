/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#ifdef __cplusplus
extern "C" {
#endif

#define FLASH_USER_START_ADDR  0x0800FC00  // Last page of Flash (1 KB page size)
#define FLASH_VAR1_ADDR        (FLASH_USER_START_ADDR)         // OpenCloseCounter
#define FLASH_VAR2_ADDR        (FLASH_VAR1_ADDR + 4)           // TotalCycleTimes
#define FLASH_ERRORS_ADDR      (FLASH_VAR2_ADDR + 4)           // errors_counters[]
#define FLASH_VAR3_ADDR        (FLASH_ERRORS_ADDR + 18)        // Last uint32_t

void Save_To_Flash(uint32_t address, void *data, uint16_t size);
void Load_From_Flash(uint32_t address, void *data, uint16_t size);
uint32_t Read_From_Flash(uint32_t address);

#ifdef __cplusplus
}
#endif

#endif /* INC_FLASH_H_ */
