/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */
static uint8_t Samplechannel;
static uint8_t Sampleindex;
#define MAX_SAMPLE_SIZE     7
uint16_t AnalogRawData[ADC_CHANNEL_NUMBER];

void ADC_Task_Init(void);
static void vADCTask(void* argument);
static void SelectADCChannel(uint8_t ch);
static uint16_t Rawfilter(uint16_t* Dat, uint8_t len);
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  Samplechannel = AD0_CHANNEL;
  Sampleindex = 0;
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  ADC1_COMMON->CCR |= 0x00800000;

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  //ADC_Task_Init();
  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


void ADC_Task_Init(void)
{
          /* Create first task */
    xTaskCreate(vADCTask,
                "ADCTask",
                TASK_ADC_STK_SIZE,
                NULL,
                TASK_ADC_PRI,
                &pxADCTask);
}
uint16_t resultbuffer[MAX_SAMPLE_SIZE];
static void vADCTask(void* argument)
{
    uint32_t millisec = 10;
    TickType_t ticks = pdMS_TO_TICKS(millisec);
    
    //uint16_t resultbuffer[MAX_SAMPLE_SIZE];
    
    (void) argument;
    
    for(;;)
    {
        //     *** Polling mode IO operation ***
        //     =================================
        //     [..]    
        //       (+) Start the ADC peripheral using HAL_ADC_Start() 
        //       (+) Wait for end of conversion using HAL_ADC_PollForConversion(), at this stage
        //           user can specify the value of timeout according to his end application      
        //       (+) To read the ADC converted values, use the HAL_ADC_GetValue() function.
        //       (+) Stop the ADC peripheral using HAL_ADC_Stop()
        //       
        //     *** Interrupt mode IO operation ***    
        //     ===================================
        for (Samplechannel=0; Samplechannel < ADC_CHANNEL_NUMBER; Samplechannel++ ){
            SelectADCChannel(Samplechannel);
            for (Sampleindex = 0; Sampleindex < MAX_SAMPLE_SIZE; Sampleindex++){
                HAL_ADC_Start(&hadc1);
                
                if (HAL_OK == HAL_ADC_PollForConversion(&hadc1, 10000)){
                    resultbuffer[Sampleindex] = HAL_ADC_GetValue(&hadc1);
                }
                HAL_ADC_Stop(&hadc1);
                //vTaskDelay(ticks);
            }
            AnalogRawData[Samplechannel] = Rawfilter(&resultbuffer[1], (Sampleindex-1));
            vTaskDelay(ticks);
        }
        
        vTaskDelay(ticks);
    }
}


static void SelectADCChannel(uint8_t ch)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    
    switch(ch){
    case AD0_CHANNEL:
        sConfig.Channel = ADC_CHANNEL_0;
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
            Error_Handler();
        }
        break;
    case AD1_CHANNEL:
        sConfig.Channel = ADC_CHANNEL_2;
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
            Error_Handler();
        }
        break;
    case AD2_CHANNEL:
        sConfig.Channel = ADC_CHANNEL_16;
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
            Error_Handler();
        }
        break;
    default:
        break;
    }
}

static uint16_t Rawfilter(uint16_t* Dat, uint8_t len)
{
    uint32_t sum = 0;
    uint16_t max = 0x0000;
    uint16_t min = 0xFFFF;
    uint8_t i;
    for (i=0; i < len; i++){
        sum += Dat[i];
        if (Dat[i] > max){
            max = Dat[i];
        }
        if (Dat[i] < min){
            min = Dat[i];
        }
    }
    if (len == 0)
        return 0xFFFF;
    else if (len <= 2)
        return (uint16_t)(sum/len);
    else
        return (uint16_t)((sum-max-min)/(len-2));
}

uint16_t ADC_GetValue(uint8_t channel)
{
    if (channel < ADC_CHANNEL_NUMBER){
        return AnalogRawData[channel];
    }
    else{
        return 0xFFFF;
    }
}

/* USER CODE END 1 */
