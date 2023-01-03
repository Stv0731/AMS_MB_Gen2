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
#define TASK_AMS_PRI                (tskIDLE_PRIORITY + 3UL)
#define TASK_USB_PRI                (tskIDLE_PRIORITY + 4UL)
#define TASK_SCI_PRI                (tskIDLE_PRIORITY + 4UL)

/* Task stack size defines */
#define TASK_START_STK_SIZE         configMINIMAL_STACK_SIZE
#define TASK_ADC_STK_SIZE           configMINIMAL_STACK_SIZE

extern TaskHandle_t StartTaskHandle;
//TaskHandle_t pxStartTask;
extern TaskHandle_t pxADCTask;
extern TaskHandle_t pxUSBTask;
extern TaskHandle_t pxSensorTask;
extern TaskHandle_t pxSCITask;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TMP117_ALERT_Pin GPIO_PIN_13
#define TMP117_ALERT_GPIO_Port GPIOC
#define PC14_Pin GPIO_PIN_14
#define PC14_GPIO_Port GPIOC
#define PC15_Pin GPIO_PIN_15
#define PC15_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_1
#define KEY2_GPIO_Port GPIOC
#define KEY3_Pin GPIO_PIN_2
#define KEY3_GPIO_Port GPIOC
#define LED_STUS_Pin GPIO_PIN_3
#define LED_STUS_GPIO_Port GPIOC
#define PA4_Pin GPIO_PIN_4
#define PA4_GPIO_Port GPIOA
#define PC4_Pin GPIO_PIN_4
#define PC4_GPIO_Port GPIOC
#define PC5_Pin GPIO_PIN_5
#define PC5_GPIO_Port GPIOC
#define PB0_Pin GPIO_PIN_0
#define PB0_GPIO_Port GPIOB
#define PB1_Pin GPIO_PIN_1
#define PB1_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define TMP117_SCL_Pin GPIO_PIN_10
#define TMP117_SCL_GPIO_Port GPIOB
#define PB12_Pin GPIO_PIN_12
#define PB12_GPIO_Port GPIOB
#define PB13_Pin GPIO_PIN_13
#define PB13_GPIO_Port GPIOB
#define PB14_Pin GPIO_PIN_14
#define PB14_GPIO_Port GPIOB
#define PB15_Pin GPIO_PIN_15
#define PB15_GPIO_Port GPIOB
#define PC8_Pin GPIO_PIN_8
#define PC8_GPIO_Port GPIOC
#define I2C_SEN2_SDA_Pin GPIO_PIN_9
#define I2C_SEN2_SDA_GPIO_Port GPIOC
#define I2C_SEN2_SCL_Pin GPIO_PIN_8
#define I2C_SEN2_SCL_GPIO_Port GPIOA
#define PA15_Pin GPIO_PIN_15
#define PA15_GPIO_Port GPIOA
#define PD2_Pin GPIO_PIN_2
#define PD2_GPIO_Port GPIOD
#define TMP117_SDA_Pin GPIO_PIN_3
#define TMP117_SDA_GPIO_Port GPIOB
#define ADS_SCL_Pin GPIO_PIN_4
#define ADS_SCL_GPIO_Port GPIOB
#define ADS_SDA_Pin GPIO_PIN_5
#define ADS_SDA_GPIO_Port GPIOB
#define I2C_SEN_SCL_Pin GPIO_PIN_6
#define I2C_SEN_SCL_GPIO_Port GPIOB
#define I2C_SEN_SDA_Pin GPIO_PIN_7
#define I2C_SEN_SDA_GPIO_Port GPIOB
#define LCD_SCL_Pin GPIO_PIN_8
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_9
#define LCD_SDA_GPIO_Port GPIOB
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
