/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "flash.h"

/**
***********************************************************************
* @brief - This function saves uint32_t data in a specific address
* @param -
***********************************************************************
*/
void Save_To_Flash(uint32_t address, void *data, uint16_t size)
{
    HAL_FLASH_Unlock();  // Unlock Flash for writing

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;

    // Erase Flash page before writing (only when writing first variable)
    if (address == FLASH_USER_START_ADDR)
    {
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
        EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
        EraseInitStruct.NbPages = 1;

        if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return;  // Handle error
        }
    }

    // Write 16-bit values
    uint16_t *data16 = (uint16_t *)data;
    for (uint16_t i = 0; i < size / 2; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + (i * 2), data16[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return;  // Handle error
        }
    }

    HAL_FLASH_Lock();  // Lock Flash after writing
}


/**
***********************************************************************
* @brief - This function returns the variable stored
* @param -
***********************************************************************
*/
void Load_From_Flash(uint32_t address, void *data, uint16_t size)
{
    uint16_t *data16 = (uint16_t *)data;
    for (uint16_t i = 0; i < size / 2; i++)
    {
        data16[i] = *(volatile uint16_t *)(address + (i * 2));
    }
}

/**
***********************************************************************
* @brief - This function returns the variable stored
* @param -
***********************************************************************
*/
uint32_t Read_From_Flash(uint32_t address)
{
    return *(volatile uint32_t*)address;  // Read 32-bit data from Flash
}

