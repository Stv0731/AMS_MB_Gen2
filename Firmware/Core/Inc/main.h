/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* Kernel includes */
#include "FreeRTOS.h"
#include "croutine.h"
#include "event_groups.h"
#include "list.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* Task priority defines */
#define TASK_START_PRI              (tskIDLE_PRIORITY + 2UL)
#define TASK_ADC_PRI                (tskIDLE_PRIORITY + 2UL)

/* Task stack size defines */
#define TASK_START_STK_SIZE         configMINIMAL_STACK_SIZE
#define TASK_ADC_STK_SIZE           configMINIMAL_STACK_SIZE

extern TaskHandle_t StartTaskHandle;
//TaskHandle_t pxStartTask;
extern TaskHandle_t pxADCTask;
extern TaskHandle_t pxUSBTask;
extern TaskHandle_t pxSensorTask;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_7
#define LED_RED_GPIO_Port GPIOE
#define LED_YELLOW_Pin GPIO_PIN_8
#define LED_YELLOW_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_9
#define LED_GREEN_GPIO_Port GPIOE
#define SPI4_CS_Pin GPIO_PIN_11
#define SPI4_CS_GPIO_Port GPIOE
#define BUTTON2_Pin GPIO_PIN_10
#define BUTTON2_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define USB_Disc_Pin GPIO_PIN_9
#define USB_Disc_GPIO_Port GPIOA
#define BUTTON1_Pin GPIO_PIN_10
#define BUTTON1_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define INT_LED_YELLOW_Pin GPIO_PIN_5
#define INT_LED_YELLOW_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_6
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB
#define INT_LED_GREEN_Pin GPIO_PIN_8
#define INT_LED_GREEN_GPIO_Port GPIOB
#define CTR_EN_I2C_Pin GPIO_PIN_2
#define CTR_EN_I2C_GPIO_Port GPIOD

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
