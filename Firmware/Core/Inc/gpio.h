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
#define LED_STUS_ON()          HAL_GPIO_WritePin(LED_STUS_GPIO_Port, LED_STUS_Pin,    GPIO_PIN_RESET)
#define LED_STUS_OFF()         HAL_GPIO_WritePin(LED_STUS_GPIO_Port, LED_STUS_Pin,    GPIO_PIN_SET)
#define LED_STUS_BLINK()       HAL_GPIO_TogglePin(LED_STUS_GPIO_Port, LED_STUS_Pin)

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
void SDA_Pin_Config(GPIO_TypeDef *Port, uint32_t Pin, uint8_t mode);
void SCL_Pin_Config(GPIO_TypeDef *Port, uint32_t Pin, uint8_t mode);
void SDA_PIN_SET(GPIO_TypeDef *Port, uint32_t Pin, uint8_t x);
void SCL_PIN_SET(GPIO_TypeDef *Port, uint32_t Pin, uint8_t x);
uint8_t SDA_PIN_GET(GPIO_TypeDef *Port, uint32_t Pin);
void DEV_PIN_SET(GPIO_TypeDef *Port, uint32_t Pin, uint8_t x);
uint8_t DEV_PIN_GET(GPIO_TypeDef *Port, uint32_t Pin);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

