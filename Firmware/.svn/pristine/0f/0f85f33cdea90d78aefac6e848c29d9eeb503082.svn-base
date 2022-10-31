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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin|USB_Disc_Pin|BUTTON1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_RED_Pin|SPI4_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_YELLOW_Pin|LED_GREEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUTTON2_Pin|SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin|USB_Disc_Pin|BUTTON1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PEPin PEPin PEPin PEPin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_YELLOW_Pin|LED_GREEN_Pin|SPI4_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = BUTTON2_Pin|SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
void SDA_Pin_Config(uint8_t mode){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    if (mode == 0){
        /*Configure : Output */
        GPIO_InitStruct.Pin   = I2C_SDA_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else{
        /*Configure : input */
        GPIO_InitStruct.Pin   = I2C_SDA_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void SCL_Pin_Config(uint8_t mode){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    if (mode == 0){
        /*Configure : Output */
        GPIO_InitStruct.Pin   = I2C_SCL_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else{
        /*Configure : input */
        GPIO_InitStruct.Pin   = I2C_SCL_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void SDA_PIN_SET(uint8_t x){
    if (x){
        HAL_GPIO_WritePin(GPIOB, I2C_SDA_Pin, GPIO_PIN_SET);
    }
    else{
        HAL_GPIO_WritePin(GPIOB, I2C_SDA_Pin, GPIO_PIN_RESET);
    }
}

void SCL_PIN_SET(uint8_t x){
    if (x){
        HAL_GPIO_WritePin(GPIOB, I2C_SCL_Pin, GPIO_PIN_SET);
    }
    else{
        HAL_GPIO_WritePin(GPIOB, I2C_SCL_Pin, GPIO_PIN_RESET);
    }
}

uint8_t SDA_PIN_GET(void){
    uint8_t tmp = HAL_GPIO_ReadPin(GPIOB, I2C_SDA_Pin);
    
    return tmp;
}

/* USER CODE END 2 */
