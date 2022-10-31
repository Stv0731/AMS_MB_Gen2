/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define LED_RED_ON()          HAL_GPIO_WritePin(GPIOE, LED_RED_Pin,    GPIO_PIN_RESET)
#define LED_RED_OFF()         HAL_GPIO_WritePin(GPIOE, LED_RED_Pin,    GPIO_PIN_SET)
#define LED_RED_BLINK()       HAL_GPIO_TogglePin(GPIOE, LED_RED_Pin)
#define LED_GREEN_ON()        HAL_GPIO_WritePin(GPIOE, LED_GREEN_Pin,  GPIO_PIN_RESET)
#define LED_GREEN_OFF()       HAL_GPIO_WritePin(GPIOE, LED_GREEN_Pin,  GPIO_PIN_SET)
#define LED_YELLOW_ON()       HAL_GPIO_WritePin(GPIOE, LED_YELLOW_Pin, GPIO_PIN_RESET)
#define LED_YELLOW_OFF()      HAL_GPIO_WritePin(GPIOE, LED_YELLOW_Pin, GPIO_PIN_SET)

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
void SDA_Pin_Config(uint8_t mode);
void SCL_Pin_Config(uint8_t mode);
void SDA_PIN_SET(uint8_t x);
void SCL_PIN_SET(uint8_t x);
uint8_t SDA_PIN_GET(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

