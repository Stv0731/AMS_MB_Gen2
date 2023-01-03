/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TMP117_ALERT_Pin|PC14_Pin|PC15_Pin|LED_STUS_Pin
                          |PC4_Pin|PC5_Pin|PC8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PA4_Pin|PA15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PB0_Pin|PB1_Pin|TMP117_SCL_Pin|PB12_Pin
                          |PB13_Pin|PB14_Pin|PB15_Pin|TMP117_SDA_Pin
                          |ADS_SCL_Pin|ADS_SDA_Pin|LCD_SCL_Pin|LCD_SDA_Pin|I2C_SEN_SCL_Pin|I2C_SEN_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PD2_GPIO_Port, PD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin
                           PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = TMP117_ALERT_Pin|PC14_Pin|PC15_Pin|LED_STUS_Pin
                          |PC4_Pin|PC5_Pin|PC8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin|KEY3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = PA4_Pin|PA15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin PBPin PBPin PBPin
                           PBPin PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = PB0_Pin|PB1_Pin|TMP117_SCL_Pin|PB12_Pin
                          |PB13_Pin|PB14_Pin|PB15_Pin|TMP117_SDA_Pin
                          |ADS_SCL_Pin|ADS_SDA_Pin|LCD_SCL_Pin|LCD_SDA_Pin|I2C_SEN_SCL_Pin|I2C_SEN_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = PD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
void SDA_Pin_Config(GPIO_TypeDef *Port, uint32_t Pin, uint8_t mode){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    if (mode == 0){
        /*Configure : Output */
        GPIO_InitStruct.Pin   = Pin;//I2C_SDA_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(Port, &GPIO_InitStruct);
    }
    else{
        /*Configure : input */
        GPIO_InitStruct.Pin   = Pin;//I2C_SDA_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(Port, &GPIO_InitStruct);
    }
}

void SCL_Pin_Config(GPIO_TypeDef *Port, uint32_t Pin, uint8_t mode){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    if (mode == 0){
        /*Configure : Output */
        GPIO_InitStruct.Pin   = Pin;//I2C_SCL_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(Port, &GPIO_InitStruct);
    }
    else{
        /*Configure : input */
        GPIO_InitStruct.Pin   = Pin;//I2C_SCL_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(Port, &GPIO_InitStruct);
    }
}

void SDA_PIN_SET(GPIO_TypeDef *Port, uint32_t Pin, uint8_t x){
    if (x){
        HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);
    }
    else{
        HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
    }
}

void SCL_PIN_SET(GPIO_TypeDef *Port, uint32_t Pin, uint8_t x){
    if (x){
        HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);
    }
    else{
        HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
    }
}

uint8_t SDA_PIN_GET(GPIO_TypeDef *Port, uint32_t Pin){
    uint8_t tmp;
    
    tmp = HAL_GPIO_ReadPin(Port, Pin);
    
    return tmp;
}

void DEV_PIN_SET(GPIO_TypeDef *Port, uint32_t Pin, uint8_t x){
    if (x){
        HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);
    }
    else{
        HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
    }
}

uint8_t DEV_PIN_GET(GPIO_TypeDef *Port, uint32_t Pin){
    uint8_t tmp;
    
    tmp = HAL_GPIO_ReadPin(Port, Pin);
    
    return tmp;
}

/* USER CODE END 2 */
