/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "string.h"
#include "stdlib.h"
#include "stdio.h"

<<<<<<< .mine
#include "main.h"
#include "ampComm.h"
#include "ampProcess.h"
#include "amp6100br.h"
#include "bmp3_common.h"
      
extern uint8_t AutoRespFlag;       // bit0: USB, bit1: SCI

QueueHandle_t xQueue_SCI;
TimerHandle_t xTimer_SCI_ReceiveDone;

static _SCI_DEVICE_LIST sciEntity[NUM_OF_SCI];

#define SCI_RCV_PACK_SIZE    128
#define SCI_RXV_PACK_NUM     10
static uint8_t sci_rcv_pack_ndx = 0;
static uint8_t sci_trs_pack_ndx = 0;
static uint8_t sci_rcv_pack[SCI_RXV_PACK_NUM][SCI_RCV_PACK_SIZE];
static uint8_t sci_resp_pack[SCI_RXV_PACK_NUM][SCI_RCV_PACK_SIZE];

static void vTask_SCI_Comm(void* argument);
static void vTimerReceiveDone_PCcomCallback(TimerHandle_t xTimer);
static void vTimerReceiveDone_RSVDcomCallback(TimerHandle_t xTimer);

static void sci_rcvd_pack_process(_SCI_DEVICE dev, uint8_t *buf);
static void sci_send_pack_process(_SCI_DEVICE dev, uint8_t *buf);
uint8_t SCI_Transmit_Start(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);
static uint8_t SCI_Start_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData);
||||||| .r32
=======
#include "main.h"
#include "ampComm.h"
#include "ampProcess.h"
#include "amp6100br.h"
#include "bmp3_common.h"
      
extern uint8_t AutoRespFlag;       // bit0: USB, bit1: SCI

QueueHandle_t xQueue_SCI;
TimerHandle_t xTimer_SCI_ReceiveDone;

#define SCI_RCV_PACK_SIZE    128
#define SCI_RXV_PACK_NUM     10
static uint8_t sci_rcv_pack_ndx = 0;
static uint8_t sci_trs_pack_ndx = 0;
static uint8_t sci_rcv_pack[SCI_RXV_PACK_NUM][SCI_RCV_PACK_SIZE];
static uint8_t sci_resp_pack[SCI_RXV_PACK_NUM][SCI_RCV_PACK_SIZE];

static void vTask_SCI_Comm(void* argument);
static void vTimerReceiveDone_Callback(TimerHandle_t xTimer);
static void sci_rcvd_pack_process(uint8_t *buf);
static void sci_send_pack_process(uint8_t *buf);
uint8_t SCI_Transmit_Start(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);
static uint8_t SCI_Start_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData);
>>>>>>> .r44
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  SCI_Start_Receive_IT(&huart2, sci_rcv_pack[sci_rcv_pack_ndx]);
  
  /* USER CODE END USART1_Init 2 */

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

<<<<<<< .mine
    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */
||||||| .r32
  /* USER CODE BEGIN USART2_MspInit 1 */
=======
    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */
>>>>>>> .r44

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

<<<<<<< .mine
    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */
||||||| .r32
  /* USER CODE BEGIN USART2_MspDeInit 1 */
=======
    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */
>>>>>>> .r44

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

<<<<<<< .mine
/*
 ** PUBLIC FUNCTION: SCI_Interface_Init()
 *
 *  DESCRIPTION:
 *      Initialize the sci interface.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
void SCI_Interface_Init(void) {
    
	xTaskCreate(vTask_SCI_Comm,
		"SCI Comm",
		2048/4,  // In words, not bytes
		NULL,
		TASK_SCI_PRI,
		&pxSCITask);
}

/*
 ** PUBLIC FUNCTION: vTask_SCI_Comm()
 *
 *  DESCRIPTION:
 *      The task for all SCI driver.
 *
 *  PARAMETERS:
 *      argument    :   the pointer to the input parameters.
 *
 *  RETURNS:
 *       None.
 */
static void vTask_SCI_Comm(void* argument)
{
    _SCI_QueueType msg;
	uint32_t millisec = 100, i;
	TickType_t ticks = pdMS_TO_TICKS(millisec);
    
    for (i = 0; i < NUM_OF_SCI; i++){
        sciEntity[i].dev = (_SCI_DEVICE)i;
        if (i == DEV_SCI_PCCOM){
            sciEntity[i].huart = &huart6;
        }
        else if (i == DEV_SCI_RESERVED){
            sciEntity[i].huart = &huart1;
        }
        if (i == DEV_SCI_PCCOM){
            sciEntity[i].xTimer_SCI_ReceiveDone = xTimerCreate("Timer's sci rcv", 2/portTICK_PERIOD_MS, pdFALSE, ( void * )0, vTimerReceiveDone_PCcomCallback);
        }
        else if (i == DEV_SCI_RESERVED){
            sciEntity[i].xTimer_SCI_ReceiveDone = xTimerCreate("Timer's sci rcv", 2/portTICK_PERIOD_MS, pdFALSE, ( void * )0, vTimerReceiveDone_RSVDcomCallback);
        }
        if( xTimer_SCI_ReceiveDone != NULL ) {
            //xTimerStart( xTimer_SCI_ReceiveDone, 0 );
        }
    }
    xQueue_SCI	= xQueueCreate( 256, ( UBaseType_t ) sizeof( _SCI_QueueType ) );
    
    
	for (;;) {
        if( xQueueReceive( xQueue_SCI, (_SCI_QueueType*)&msg, ticks ) == pdTRUE )
        {
            switch(msg.event)
            {
            case SCI_EV_RCV_DATA:
                sci_rcvd_pack_process((_SCI_DEVICE)msg.dev, msg.buf);
                break;
                
            case SCI_EV_SND_DATA:
                sci_send_pack_process((_SCI_DEVICE)msg.dev, msg.buf);
                break;
                
            case SCI_EV_ERROR:

                break; 
                
            default:
                break;
            }
        }
        else
        {
            //TimeOut_Check( xTaskGetTickCount() );
//            char *st = "This is my first usb cdc port!";
//            SCI_Send_MsgProduce(st, 30);
        }
	}
}

static void sci_rcvd_pack_process(_SCI_DEVICE dev, uint8_t *buf)
{
    
    if (dev == DEV_SCI_PCCOM){
        
        SCITransDataType *pd = (SCITransDataType*)buf;
        uint16_t len = pd->length;
        char* pString = (char*)pd->p_dat;
        char* presp = (char*)sci_resp_pack[/*sci_trs_pack_ndx*/0];
        uint8_t len_resp;
        
        if( memcmp((char*)pString, "AT", len) == 0 ){
            sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
            presp = "OK";
            len_resp = strlen(presp);
            SCI_Send_MsgProduce(dev, (uint8_t*)presp, len_resp);
        }
        else if ( memcmp((char*)pString, "AT+REG+", strlen("AT+REG+")) == 0 ){      // None used
            uint32_t reg;
            sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
            pString += strlen("AT+REG+");
            reg	= atoi(pString);
            
            pString	= strchr(pString,'?');
            if (pString != NULL){
                sprintf ( presp, "+REG+%d:%d", reg, amp61xx_entity.pRegister->array[reg]);
                len_resp = strlen(presp);
                SCI_Send_MsgProduce(dev, (uint8_t*)presp, len_resp);
            }
        }
        else if ( memcmp((char*)pString, "AT+CO_T+", strlen("AT+CO_T+")) == 0 ){    // None used
            uint8_t ndx;
            sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
            pString += strlen("AT+CO_T+");
            ndx	= atoi(pString);

            pString	= strchr(pString,'?');
            if (pString != NULL){
                sprintf ( presp, "+CO_T+%d:%d", ndx, amp61xx_entity.co_t_array[ndx-1]);
                len_resp = strlen(presp);
                SCI_Send_MsgProduce(dev, (uint8_t*)presp, len_resp);
            }
        }
        else if ( memcmp((char*)pString, "AT+CO_P+", strlen("AT+CO_P+")) == 0 ){    // None used
            uint8_t ndx;
            sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1);
            pString += strlen("AT+CO_P+");
            ndx	= atoi(pString);

            pString	= strchr(pString,'?');
            if (pString != NULL){
                sprintf ( presp, "+CO_P+%d:%d", ndx, amp61xx_entity.co_p_array[ndx-1]);
                len_resp = strlen(presp);
                SCI_Send_MsgProduce(dev, (uint8_t*)presp, len_resp);
            }
        }
        else if ( memcmp((char*)pString, "AT+PRES?", strlen("AT+PRES?")) == 0 ){    // None used
            sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
            sprintf ( presp, "+PRES: %d, %d", amp_entity.pres_raw, amp_entity.pres_rt);
            len_resp = strlen(presp);
            SCI_Send_MsgProduce(dev, (uint8_t*)presp, len_resp);
            
        }
        else if ( memcmp((char*)pString, "AT+TEMP?", strlen("AT+TEMP?")) == 0 ){    // None used
            sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
            sprintf ( presp, "+TEMP: %d, %d", amp_entity.temp_raw, amp_entity.temp_rt);
            len_resp = strlen(presp);
            SCI_Send_MsgProduce(dev, (uint8_t*)presp, len_resp);
            
        }
        else if ( memcmp((char*)pString, "AT+SAMPLES?", strlen("AT+SAMPLES?")) == 0 ){
            sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
            sprintf ( presp, "+SAMPLES: %d, %d, %d, %d", amp_entity.pres_raw, amp_entity.pres_rt, amp_entity.temp_raw, amp_entity.temp_rt);
            len_resp = strlen(presp);
            SCI_Send_MsgProduce(dev, (uint8_t*)presp, len_resp);
            
        }
        else if ( memcmp((char*)pString, "AT+CMD=", strlen("AT+CMD=")) == 0 ){      // AMP61XX
            uint8_t tcmd = 0;
            sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
            pString += strlen("AT+CMD=");
            tcmd = atoi(pString);

            if (amp_entity.SensorType == AMP6120 || amp_entity.SensorType == AMP6127){
                Amp_Control_Req(tcmd, 0, 0);
                
                presp = "OK";
                len_resp = strlen(presp);
                SCI_Send_MsgProduce(dev, (uint8_t*)presp, len_resp);
            }
            else if (amp_entity.SensorType == BMP3XX){
                bmp3_osrp_config(tcmd&0x07);
            }
            else{
                presp = "Failed";
                len_resp = strlen(presp);
                SCI_Send_MsgProduce(dev, (uint8_t*)presp, len_resp);
            }
        }
        else if ( memcmp((char*)pString, "AT+ADC=", strlen("AT+ADC=")) == 0 ){
            uint8_t en;
            sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
            pString += strlen("AT+ADC=");
            en = atoi(pString);
            Amp_Control_Req(en, 0, 0);
            
            presp = "OK";
            len_resp = strlen(presp);
            SCI_Send_MsgProduce(dev, (uint8_t*)presp, len_resp);
        }
        else if ( memcmp((char*)pString, "AT+START=", strlen("AT+START=")) == 0 ){
            uint8_t en;
            sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
            pString += strlen("AT+START=");
            en = atoi(pString);
            Amp_ADC_Start(en);
            
            if (en)
                AutoRespFlag |= 0x02;
            else
                AutoRespFlag &= ~0x02;
            
            presp = "OK";
            len_resp = strlen(presp);
            SCI_Send_MsgProduce(dev, (uint8_t*)presp, len_resp);
            
        }
        else if ( memcmp((char*)pString, "AT+IIR=", strlen("AT+IIR=")) == 0 ){
            uint8_t en;
            sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
            pString += strlen("AT+IIR=");
            en = atoi(pString);
            
            if (amp_entity.SensorType == AMP6120 || amp_entity.SensorType == AMP6127){
                Amp_IIR_Enable(en);
            }
            else if (amp_entity.SensorType == BMP3XX){
                bmp_iir_config(en);
            }
                
            presp = "OK";
            len_resp = strlen(presp);
            SCI_Send_MsgProduce(dev, (uint8_t*)presp, len_resp);
            
        }
        else if ( memcmp((char*)pString, "AT+ODR=", strlen("AT+ODR=")) == 0 ){
            uint16_t freq;
            sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
            pString += strlen("AT+ODR=");
            freq = atoi(pString);
            Amp_ODR_Set(freq);
            if (amp_entity.SensorType == BMP3XX){
                bmp_odr_config((float)freq);
            }
                
            presp = "OK";
            len_resp = strlen(presp);
            SCI_Send_MsgProduce(dev, (uint8_t*)presp, len_resp);
            
        }
        else if ( memcmp((char*)pString, "AT+SENSORTYPE=", strlen("AT+SENSORTYPE=")) == 0 ){
            uint16_t type;
            sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
            pString += strlen("AT+SENSORTYPE=");
            type = atoi(pString);
            Amp_SensorType_Set(type);
            
            presp = "OK";
            len_resp = strlen(presp);
            SCI_Send_MsgProduce(dev, (uint8_t*)presp, len_resp);
            
        }
        else if ( memcmp((char*)pString, "AT+SENSORCHANNEL=", strlen("AT+SENSORCHANNEL=")) == 0 ){
            uint16_t ch;
            sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
            pString += strlen("AT+SENSORCHANNEL=");
            ch = atoi(pString);
            Amp_SampleChannel_Set(ch);
            
            presp = "OK";
            len_resp = strlen(presp);
            SCI_Send_MsgProduce(dev, (uint8_t*)presp, len_resp);
            
        }
        memset(pd->p_dat, 0, len);
    }
    else if (dev == DEV_SCI_RESERVED){
        
    }
    
}

void SendSampleDataFromSCI(void)
{
    char* presp = (char*)sci_resp_pack[/*usb_trs_pack_ndx*/0];
    uint8_t len_resp;
    
    sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
    sprintf ( presp, "+SAMPLES: %d, %d, %d, %d", amp_entity.pres_raw, amp_entity.pres_rt, amp_entity.temp_raw, amp_entity.temp_rt);
    len_resp = strlen(presp);
    SCI_Send_MsgProduce(DEV_SCI_PCCOM, (uint8_t*)presp, len_resp);
}

uint8_t GetSCIDevicePort(_SCI_DEVICE dev, UART_HandleTypeDef *huart){
    uint8_t i;
    
    for (i = 0; i < NUM_OF_SCI; i++){
        if (sciEntity[i].dev == dev){
            huart = sciEntity[i].huart;
            return 1;
        }
    }
    return 0;
}

uint8_t GetSCIDeviceTimer(_SCI_DEVICE dev, TimerHandle_t xTimer){
    uint8_t i;
    
    for (i = 0; i < NUM_OF_SCI; i++){
        if (sciEntity[i].dev == dev){
            xTimer = sciEntity[i].xTimer_SCI_ReceiveDone;
            return 1;
        }
    }
    return 0;
}

static void sci_send_pack_process(_SCI_DEVICE dev, uint8_t *buf)
{
    SCITransDataType *pd = (SCITransDataType*)buf;
    UART_HandleTypeDef *huart = NULL;
    
    if (!GetSCIDevicePort(dev, huart) && huart == NULL){
        return;
    }
    
    if (SCI_HDL_NOERR != SCI_Transmit_Start(huart, pd->p_dat, pd->length)){
        
    }
}

/*
 ** PUBLIC FUNCTION: SCI_MsgSend()
 */
static void SCI_MsgSend(uint8_t dev, uint8_t event, SCITransDataType* p_dat )
{
    _SCI_QueueType msg;
    uint8_t *p = (uint8_t*)p_dat;
    uint8_t dat_len = sizeof(SCITransDataType);
    if(NULL == xQueue_SCI)
        return;
    
    msg.dev     = dev;
    msg.event   = event;
    if( p != NULL && dat_len < sizeof(_SCI_QueueType) ) 
        memcpy( msg.buf, (uint8_t*)p_dat, dat_len );
    
    xQueueSend( xQueue_SCI, ( _SCI_QueueType * ) &msg, ( TickType_t ) 100 );
}

static void SCI_MsgSend_FromISR(uint8_t dev, uint8_t event, SCITransDataType* p_dat )
{
    _SCI_QueueType msg;
    uint8_t *p = (uint8_t*)p_dat;
    uint8_t dat_len = sizeof(SCITransDataType);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;  
    
    if(NULL == xQueue_SCI)
        return;
    
    msg.dev     = dev;
    msg.event   = event;
    if( p != NULL && dat_len < sizeof(_SCI_QueueType) ) 
        memcpy( msg.buf, (uint8_t*)p_dat, dat_len );
    
    xQueueSendFromISR( xQueue_SCI, &msg, &xHigherPriorityTaskWoken );
    
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

void SCI_Send_MsgProduce(_SCI_DEVICE dev, uint8_t* Buf, uint16_t Len)
{
    SCITransDataType dat;
    
    dat.length = Len;
    dat.p_dat = Buf;
    
    SCI_MsgSend(dev, SCI_EV_SND_DATA, &dat);
}

void SCI_Recv_MsgProduce(_SCI_DEVICE dev, uint8_t isr)
{
    SCITransDataType dat;
    UART_HandleTypeDef *huart;
    
    if (!GetSCIDevicePort(dev, huart)){
        return;
    }
    
    dat.length = huart->RxXferCount;
    dat.p_dat = sci_rcv_pack[sci_rcv_pack_ndx];
    sci_rcv_pack_ndx++;
    if (sci_rcv_pack_ndx == SCI_RXV_PACK_NUM)
        sci_rcv_pack_ndx = 0;
    
    SCI_Start_Receive_IT(huart, sci_rcv_pack[sci_rcv_pack_ndx]);
    if (isr)
        SCI_MsgSend_FromISR(dev, SCI_EV_RCV_DATA, &dat);
    else
        SCI_MsgSend(dev, SCI_EV_RCV_DATA, &dat);
}

// refer to UART_Receive_IT()
static uint8_t SCI_Receive_IT(uint8_t dev, UART_HandleTypeDef *huart)
{
    uint8_t ret = SCI_HDL_NOERR;
    
    uint8_t  *pdata8bits;
    
    TimerHandle_t xTimer = NULL;
    GetSCIDeviceTimer(dev, xTimer);

    /* Check that a Rx process is ongoing */
    if (huart->RxState == HAL_UART_STATE_BUSY_RX && xTimer != NULL)
    {
        pdata8bits = (uint8_t *) huart->pRxBuffPtr;

        *pdata8bits = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
        huart->pRxBuffPtr += 1U;
        huart->RxXferCount++;

        if (xTimerResetFromISR( xTimer, 0 ) != pdPASS){
        }
    }
    else
    {
        ret = SCI_HDL_ERR;
    }
    
    return ret;
}

void vTimerReceiveDone_PCcomCallback(TimerHandle_t xTimer)
{
    /* Rx process is completed, restore huart->RxState to Ready */
    sciEntity[DEV_SCI_PCCOM].huart->RxState = HAL_UART_STATE_READY;
    
    SCI_Recv_MsgProduce(DEV_SCI_PCCOM, 0);
}
void vTimerReceiveDone_RSVDcomCallback(TimerHandle_t xTimer)
{
    sciEntity[DEV_SCI_RESERVED].huart->RxState = HAL_UART_STATE_READY;
    
    SCI_Recv_MsgProduce(DEV_SCI_RESERVED, 0);
}

// refer to UART_Transmit_IT()
static uint8_t SCI_Transmit_IT(UART_HandleTypeDef *huart)
{
    uint8_t ret = SCI_HDL_NOERR;
    //const uint16_t *tmp;

    /* Check that a Tx process is ongoing */
    if (huart->gState == HAL_UART_STATE_BUSY_TX)
    {
        huart->Instance->DR = (uint8_t)(*huart->pTxBuffPtr++ & (uint8_t)0x00FF);

        if (--huart->TxXferCount == 0U)
        {
            /* Disable the UART Transmit Data Register Empty Interrupt */
            __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);

            /* Enable the UART Transmit Complete Interrupt */
            __HAL_UART_ENABLE_IT(huart, UART_IT_TC);
        }
        ret =  SCI_HDL_NOERR;
    }
    else
    {
        ret =  SCI_HDL_ERR;
    }
    return ret;
}

static uint8_t SCI_Start_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData)
{
  huart->pRxBuffPtr = pData;
  huart->RxXferSize = SCI_RCV_PACK_SIZE;
  huart->RxXferCount = 0;

  huart->ErrorCode = HAL_UART_ERROR_NONE;
  huart->RxState = HAL_UART_STATE_BUSY_RX;

  /* Process Unlocked */
  __HAL_UNLOCK(huart);

  if (huart->Init.Parity != UART_PARITY_NONE)
  {
    /* Enable the UART Parity Error Interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_PE);
  }

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(huart, UART_IT_ERR);

  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);

  return HAL_OK;
}


// refer to UART_EndTransmit_IT()
static uint8_t SCI_EndTransmit_IT(UART_HandleTypeDef *huart)
{
    uint8_t ret = SCI_HDL_NOERR;
    
    /* Disable the UART Transmit Complete Interrupt */
    __HAL_UART_DISABLE_IT(huart, UART_IT_TC);

    /* Tx process is ended, restore huart->gState to Ready */
    huart->gState = HAL_UART_STATE_READY;

    // Tx complete action...
    
    return ret;
}

// refer to HAL_UART_Transmit_IT()
uint8_t SCI_Transmit_Start(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
    /* Check that a Tx process is not already ongoing */
    if (huart->gState == HAL_UART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(huart);

        huart->pTxBuffPtr = pData;
        huart->TxXferSize = Size;
        huart->TxXferCount = Size;

        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->gState = HAL_UART_STATE_BUSY_TX;

        /* Process Unlocked */
        __HAL_UNLOCK(huart);

        /* Enable the UART Transmit data register empty Interrupt */
        __HAL_UART_ENABLE_IT(huart, UART_IT_TXE);

        return SCI_HDL_NOERR;
    }
    else
    {
        return SCI_HDL_ERR;
    }
}


/* SCI irq handler, called by USART2_IRQHandler() */
void SCI_IRQHandler(_SCI_DEVICE dev)
{
  UART_HandleTypeDef *huart = NULL;
  if (!GetSCIDevicePort(dev, huart) && huart == NULL){
    return;
  }
  uint32_t isrflags   = READ_REG(huart->Instance->SR);
  uint32_t cr1its     = READ_REG(huart->Instance->CR1);
  uint32_t cr3its     = READ_REG(huart->Instance->CR3);
  uint32_t errorflags = 0x00U;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
  if (errorflags == RESET)
  {
    /* UART in mode Receiver -------------------------------------------------*/
    if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
      SCI_Receive_IT(dev, huart);
      return;
    }
  }

  /* If some errors occur */
  if ((errorflags != RESET) && (((cr3its & USART_CR3_EIE) != RESET)
                                || ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET)))
  {
    /* UART parity error interrupt occurred ----------------------------------*/
    if (((isrflags & USART_SR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
    {
      huart->ErrorCode |= HAL_UART_ERROR_PE;
    }

    /* UART noise error interrupt occurred -----------------------------------*/
    if (((isrflags & USART_SR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      huart->ErrorCode |= HAL_UART_ERROR_NE;
    }

    /* UART frame error interrupt occurred -----------------------------------*/
    if (((isrflags & USART_SR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      huart->ErrorCode |= HAL_UART_ERROR_FE;
    }

    /* UART Over-Run interrupt occurred --------------------------------------*/
    if (((isrflags & USART_SR_ORE) != RESET) && (((cr1its & USART_CR1_RXNEIE) != RESET)
                                                 || ((cr3its & USART_CR3_EIE) != RESET)))
    {
      huart->ErrorCode |= HAL_UART_ERROR_ORE;
    }
    
    return;
  } /* End if some error occurs */

  /* UART in mode Transmitter ------------------------------------------------*/
  if (((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
  {
    SCI_Transmit_IT(huart);
    return;
  }

  /* UART in mode Transmitter end --------------------------------------------*/
  if (((isrflags & USART_SR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
  {
    SCI_EndTransmit_IT(huart);
    return;
  }
}



||||||| .r32
=======
/*
 ** PUBLIC FUNCTION: SCI_Interface_Init()
 *
 *  DESCRIPTION:
 *      Initialize the sci interface.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
void SCI_Interface_Init(void) {
    
	xTaskCreate(vTask_SCI_Comm,
		"SCI Comm",
		2048/4,  // In words, not bytes
		NULL,
		TASK_SCI_PRI,
		&pxSCITask);
}

/*
 ** PUBLIC FUNCTION: vTask_SCI_Comm()
 *
 *  DESCRIPTION:
 *      The task for all SCI driver.
 *
 *  PARAMETERS:
 *      argument    :   the pointer to the input parameters.
 *
 *  RETURNS:
 *       None.
 */
static void vTask_SCI_Comm(void* argument)
{
    _SCI_QueueType msg;
	uint32_t millisec = 100;
	TickType_t ticks = pdMS_TO_TICKS(millisec);

    xQueue_SCI	= xQueueCreate( 256, ( UBaseType_t ) sizeof( _SCI_QueueType ) );
    
    xTimer_SCI_ReceiveDone = xTimerCreate("Timer's sci rcv", 2/portTICK_PERIOD_MS, pdFALSE, ( void * )0, vTimerReceiveDone_Callback);
	if( xTimer_SCI_ReceiveDone != NULL ) 
	{
		//xTimerStart( xTimer_SCI_ReceiveDone, 0 );
	}
    
	for (;;) {
        if( xQueueReceive( xQueue_SCI, (_SCI_QueueType*)&msg, ticks ) == pdTRUE )
        {
            switch(msg.event)
            {
            case SCI_EV_RCV_DATA:
                sci_rcvd_pack_process(msg.buf);
                break;
                
            case SCI_EV_SND_DATA:
                sci_send_pack_process(msg.buf);
                break;
                
            case SCI_EV_ERROR:

                break; 
                
            default:
                break;            
            }
        }
        else
        {
            //TimeOut_Check( xTaskGetTickCount() );
//            char *st = "This is my first usb cdc port!";
//            SCI_Send_MsgProduce(st, 30);
        }
	}
}

static void sci_rcvd_pack_process(uint8_t *buf)
{
    SCITransDataType *pd = (SCITransDataType*)buf;
    uint16_t len = pd->length;
    char* pString = (char*)pd->p_dat;
    char* presp = (char*)sci_resp_pack[/*sci_trs_pack_ndx*/0];
    uint8_t len_resp;
    
    if( memcmp((char*)pString, "AT", len) == 0 ){
        sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
        presp = "OK";
        len_resp = strlen(presp);
        SCI_Send_MsgProduce((uint8_t*)presp, len_resp);
    }
    else if ( memcmp((char*)pString, "AT+REG+", strlen("AT+REG+")) == 0 ){      // None used
        uint32_t reg;
        sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
        pString += strlen("AT+REG+");
        reg	= atoi(pString);
        
        pString	= strchr(pString,'?');
        if (pString != NULL){
            sprintf ( presp, "+REG+%d:%d", reg, amp61xx_entity.pRegister->array[reg]);
            len_resp = strlen(presp);
            SCI_Send_MsgProduce((uint8_t*)presp, len_resp);
        }
    }
    else if ( memcmp((char*)pString, "AT+CO_T+", strlen("AT+CO_T+")) == 0 ){    // None used
        uint8_t ndx;
        sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
        pString += strlen("AT+CO_T+");
        ndx	= atoi(pString);

        pString	= strchr(pString,'?');
        if (pString != NULL){
            sprintf ( presp, "+CO_T+%d:%d", ndx, amp61xx_entity.co_t_array[ndx-1]);
            len_resp = strlen(presp);
            SCI_Send_MsgProduce((uint8_t*)presp, len_resp);
        }
    }
    else if ( memcmp((char*)pString, "AT+CO_P+", strlen("AT+CO_P+")) == 0 ){    // None used
        uint8_t ndx;
        sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1);
        pString += strlen("AT+CO_P+");
        ndx	= atoi(pString);

        pString	= strchr(pString,'?');
        if (pString != NULL){
            sprintf ( presp, "+CO_P+%d:%d", ndx, amp61xx_entity.co_p_array[ndx-1]);
            len_resp = strlen(presp);
            SCI_Send_MsgProduce((uint8_t*)presp, len_resp);
        }
    }
    else if ( memcmp((char*)pString, "AT+PRES?", strlen("AT+PRES?")) == 0 ){    // None used
        sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
        sprintf ( presp, "+PRES: %d, %d", amp_entity.pres_raw, amp_entity.pres_rt);
        len_resp = strlen(presp);
        SCI_Send_MsgProduce((uint8_t*)presp, len_resp);
        
    }
    else if ( memcmp((char*)pString, "AT+TEMP?", strlen("AT+TEMP?")) == 0 ){    // None used
        sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
        sprintf ( presp, "+TEMP: %d, %d", amp_entity.temp_raw, amp_entity.temp_rt);
        len_resp = strlen(presp);
        SCI_Send_MsgProduce((uint8_t*)presp, len_resp);
        
    }
    else if ( memcmp((char*)pString, "AT+SAMPLES?", strlen("AT+SAMPLES?")) == 0 ){
        sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
        sprintf ( presp, "+SAMPLES: %d, %d, %d, %d", amp_entity.pres_raw, amp_entity.pres_rt, amp_entity.temp_raw, amp_entity.temp_rt);
        len_resp = strlen(presp);
        SCI_Send_MsgProduce((uint8_t*)presp, len_resp);
        
    }
    else if ( memcmp((char*)pString, "AT+CMD=", strlen("AT+CMD=")) == 0 ){      // AMP61XX
        uint8_t tcmd = 0;
        sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
        pString += strlen("AT+CMD=");
        tcmd = atoi(pString);

        if (amp_entity.SensorType == AMP6120 || amp_entity.SensorType == AMP6127){
            Amp_Control_Req(tcmd, 0, 0);
            
            presp = "OK";
            len_resp = strlen(presp);
            SCI_Send_MsgProduce((uint8_t*)presp, len_resp);
        }
        else if (amp_entity.SensorType == BMP3XX){
            bmp3_osrp_config(tcmd&0x07);
        }
        else{
            presp = "Failed";
            len_resp = strlen(presp);
            SCI_Send_MsgProduce((uint8_t*)presp, len_resp);
        }
    }
    else if ( memcmp((char*)pString, "AT+ADC=", strlen("AT+ADC=")) == 0 ){
        uint8_t en;
        sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
        pString += strlen("AT+ADC=");
        en = atoi(pString);
        Amp_Control_Req(en, 0, 0);
        
        presp = "OK";
        len_resp = strlen(presp);
        SCI_Send_MsgProduce((uint8_t*)presp, len_resp);
    }
    else if ( memcmp((char*)pString, "AT+START=", strlen("AT+START=")) == 0 ){
        uint8_t en;
        sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
        pString += strlen("AT+START=");
        en = atoi(pString);
        Amp_ADC_Start(en);
        
        if (en)
            AutoRespFlag |= 0x02;
        else
            AutoRespFlag &= ~0x02;
        
        presp = "OK";
        len_resp = strlen(presp);
        SCI_Send_MsgProduce((uint8_t*)presp, len_resp);
        
    }
    else if ( memcmp((char*)pString, "AT+IIR=", strlen("AT+IIR=")) == 0 ){
        uint8_t en;
        sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
        pString += strlen("AT+IIR=");
        en = atoi(pString);
        
        if (amp_entity.SensorType == AMP6120 || amp_entity.SensorType == AMP6127){
            Amp_IIR_Enable(en);
        }
        else if (amp_entity.SensorType == BMP3XX){
            bmp_iir_config(en);
        }
            
        presp = "OK";
        len_resp = strlen(presp);
        SCI_Send_MsgProduce((uint8_t*)presp, len_resp);
        
    }
    else if ( memcmp((char*)pString, "AT+ODR=", strlen("AT+ODR=")) == 0 ){
        uint16_t freq;
        sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
        pString += strlen("AT+ODR=");
        freq = atoi(pString);
        Amp_ODR_Set(freq);
        if (amp_entity.SensorType == BMP3XX){
            bmp_odr_config((float)freq);
        }
            
        presp = "OK";
        len_resp = strlen(presp);
        SCI_Send_MsgProduce((uint8_t*)presp, len_resp);
        
    }
    else if ( memcmp((char*)pString, "AT+SENSORTYPE=", strlen("AT+SENSORTYPE=")) == 0 ){
        uint16_t type;
        sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
        pString += strlen("AT+SENSORTYPE=");
        type = atoi(pString);
        Amp_SensorType_Set(type);
        
        presp = "OK";
        len_resp = strlen(presp);
        SCI_Send_MsgProduce((uint8_t*)presp, len_resp);
        
    }
    else if ( memcmp((char*)pString, "AT+SENSORCHANNEL=", strlen("AT+SENSORCHANNEL=")) == 0 ){
        uint16_t ch;
        sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
        pString += strlen("AT+SENSORCHANNEL=");
        ch = atoi(pString);
        Amp_SampleChannel_Set(ch);
        
        presp = "OK";
        len_resp = strlen(presp);
        SCI_Send_MsgProduce((uint8_t*)presp, len_resp);
        
    }
    
//    if (len == 5 && pd->p_dat[0] == 0xAA && pd->p_dat[1] == 0xAB){
//        SCI_Send_MsgProduce(pd->p_dat, len);
//    }
    memset(pd->p_dat, 0, len);
}

void SendSampleDataFromSCI(void)
{
    char* presp = (char*)sci_resp_pack[/*usb_trs_pack_ndx*/0];
    uint8_t len_resp;
    
    sci_trs_pack_ndx = sci_trs_pack_ndx == (SCI_RXV_PACK_NUM-1) ? 0 : (sci_trs_pack_ndx+1); 
    sprintf ( presp, "+SAMPLES: %d, %d, %d, %d", amp_entity.pres_raw, amp_entity.pres_rt, amp_entity.temp_raw, amp_entity.temp_rt);
    len_resp = strlen(presp);
    SCI_Send_MsgProduce((uint8_t*)presp, len_resp);
}
static void sci_send_pack_process(uint8_t *buf)
{
    SCITransDataType *pd = (SCITransDataType*)buf;
    
    if (SCI_HDL_NOERR != SCI_Transmit_Start(&huart2, pd->p_dat, pd->length)){
        
    }
}

/*
 ** PUBLIC FUNCTION: SCI_MsgSend()
 */
static void SCI_MsgSend(uint8_t event, SCITransDataType* p_dat )
{
    _SCI_QueueType msg;
    uint8_t *p = (uint8_t*)p_dat;
    uint8_t dat_len = sizeof(SCITransDataType);
    if(NULL == xQueue_SCI)
        return;
    
    msg.event   = event;
    
    if( p != NULL && dat_len < sizeof(_SCI_QueueType) ) 
        memcpy( msg.buf, (uint8_t*)p_dat, dat_len );
    
    xQueueSend( xQueue_SCI, ( _SCI_QueueType * ) &msg, ( TickType_t ) 100 );
}

static void SCI_MsgSend_FromISR(uint8_t event, SCITransDataType* p_dat )
{
    _SCI_QueueType msg;
    uint8_t *p = (uint8_t*)p_dat;
    uint8_t dat_len = sizeof(SCITransDataType);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;  
    
    if(NULL == xQueue_SCI)
        return;
    
    msg.event   = event;
    if( p != NULL && dat_len < sizeof(_SCI_QueueType) ) 
        memcpy( msg.buf, (uint8_t*)p_dat, dat_len );
    
    xQueueSendFromISR( xQueue_SCI, &msg, &xHigherPriorityTaskWoken );
    
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

void SCI_Send_MsgProduce(uint8_t* Buf, uint16_t Len)
{
    SCITransDataType dat;
    
    dat.length = Len;
    dat.p_dat = Buf;
    
    SCI_MsgSend(SCI_EV_SND_DATA, &dat);
}

void SCI_Recv_MsgProduce(uint8_t isr)
{
    SCITransDataType dat;
    
//    for (uint8_t i = 0; i < Len; i++){
//        sci_rcv_pack[sci_rcv_pack_ndx][i] = Buf[i];
//    }
//    
    dat.length = huart2.RxXferCount;
    dat.p_dat = sci_rcv_pack[sci_rcv_pack_ndx];
    sci_rcv_pack_ndx++;
    if (sci_rcv_pack_ndx == SCI_RXV_PACK_NUM)
        sci_rcv_pack_ndx = 0;
    
    SCI_Start_Receive_IT(&huart2, sci_rcv_pack[sci_rcv_pack_ndx]);
    if (isr)
        SCI_MsgSend_FromISR(SCI_EV_RCV_DATA, &dat);
    else
        SCI_MsgSend(SCI_EV_RCV_DATA, &dat);
}

// refer to UART_Receive_IT()
static uint8_t SCI_Receive_IT(UART_HandleTypeDef *huart)
{
    uint8_t ret = SCI_HDL_NOERR;
    
    uint8_t  *pdata8bits;

    /* Check that a Rx process is ongoing */
    if (huart->RxState == HAL_UART_STATE_BUSY_RX)
    {
        pdata8bits = (uint8_t *) huart->pRxBuffPtr;

        *pdata8bits = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
        huart->pRxBuffPtr += 1U;
        huart->RxXferCount++;

        if (xTimerResetFromISR( xTimer_SCI_ReceiveDone, 0 ) != pdPASS){
        }
    }
    else
    {
        ret = SCI_HDL_ERR;
    }
    
    return ret;
}

void vTimerReceiveDone_Callback(TimerHandle_t xTimer)
{
    /* Rx process is completed, restore huart->RxState to Ready */
    huart2.RxState = HAL_UART_STATE_READY;
    
    SCI_Recv_MsgProduce(0);
}

// refer to UART_Transmit_IT()
static uint8_t SCI_Transmit_IT(UART_HandleTypeDef *huart)
{
    uint8_t ret = SCI_HDL_NOERR;
    //const uint16_t *tmp;

    /* Check that a Tx process is ongoing */
    if (huart->gState == HAL_UART_STATE_BUSY_TX)
    {
        huart->Instance->DR = (uint8_t)(*huart->pTxBuffPtr++ & (uint8_t)0x00FF);

        if (--huart->TxXferCount == 0U)
        {
            /* Disable the UART Transmit Data Register Empty Interrupt */
            __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);

            /* Enable the UART Transmit Complete Interrupt */
            __HAL_UART_ENABLE_IT(huart, UART_IT_TC);
        }
        ret =  SCI_HDL_NOERR;
    }
    else
    {
        ret =  SCI_HDL_ERR;
    }
    return ret;
}

static uint8_t SCI_Start_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData)
{
  huart->pRxBuffPtr = pData;
  huart->RxXferSize = SCI_RCV_PACK_SIZE;
  huart->RxXferCount = 0;

  huart->ErrorCode = HAL_UART_ERROR_NONE;
  huart->RxState = HAL_UART_STATE_BUSY_RX;

  /* Process Unlocked */
  __HAL_UNLOCK(huart);

  if (huart->Init.Parity != UART_PARITY_NONE)
  {
    /* Enable the UART Parity Error Interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_PE);
  }

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(huart, UART_IT_ERR);

  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);

  return HAL_OK;
}


// refer to UART_EndTransmit_IT()
static uint8_t SCI_EndTransmit_IT(UART_HandleTypeDef *huart)
{
    uint8_t ret = SCI_HDL_NOERR;
    
    /* Disable the UART Transmit Complete Interrupt */
    __HAL_UART_DISABLE_IT(huart, UART_IT_TC);

    /* Tx process is ended, restore huart->gState to Ready */
    huart->gState = HAL_UART_STATE_READY;

    // Tx complete action...
    
    return ret;
}

// refer to HAL_UART_Transmit_IT()
uint8_t SCI_Transmit_Start(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
    /* Check that a Tx process is not already ongoing */
    if (huart->gState == HAL_UART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(huart);

        huart->pTxBuffPtr = pData;
        huart->TxXferSize = Size;
        huart->TxXferCount = Size;

        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->gState = HAL_UART_STATE_BUSY_TX;

        /* Process Unlocked */
        __HAL_UNLOCK(huart);

        /* Enable the UART Transmit data register empty Interrupt */
        __HAL_UART_ENABLE_IT(huart, UART_IT_TXE);

        return SCI_HDL_NOERR;
    }
    else
    {
        return SCI_HDL_ERR;
    }
}


/* SCI irq handler, called by USART2_IRQHandler() */
void SCI_IRQHandler(UART_HandleTypeDef *huart)
{
  uint32_t isrflags   = READ_REG(huart->Instance->SR);
  uint32_t cr1its     = READ_REG(huart->Instance->CR1);
  uint32_t cr3its     = READ_REG(huart->Instance->CR3);
  uint32_t errorflags = 0x00U;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
  if (errorflags == RESET)
  {
    /* UART in mode Receiver -------------------------------------------------*/
    if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
      SCI_Receive_IT(huart);
      return;
    }
  }

  /* If some errors occur */
  if ((errorflags != RESET) && (((cr3its & USART_CR3_EIE) != RESET)
                                || ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET)))
  {
    /* UART parity error interrupt occurred ----------------------------------*/
    if (((isrflags & USART_SR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
    {
      huart->ErrorCode |= HAL_UART_ERROR_PE;
    }

    /* UART noise error interrupt occurred -----------------------------------*/
    if (((isrflags & USART_SR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      huart->ErrorCode |= HAL_UART_ERROR_NE;
    }

    /* UART frame error interrupt occurred -----------------------------------*/
    if (((isrflags & USART_SR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      huart->ErrorCode |= HAL_UART_ERROR_FE;
    }

    /* UART Over-Run interrupt occurred --------------------------------------*/
    if (((isrflags & USART_SR_ORE) != RESET) && (((cr1its & USART_CR1_RXNEIE) != RESET)
                                                 || ((cr3its & USART_CR3_EIE) != RESET)))
    {
      huart->ErrorCode |= HAL_UART_ERROR_ORE;
    }
    
    return;
  } /* End if some error occurs */

  /* UART in mode Transmitter ------------------------------------------------*/
  if (((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
  {
    SCI_Transmit_IT(huart);
    return;
  }

  /* UART in mode Transmitter end --------------------------------------------*/
  if (((isrflags & USART_SR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
  {
    SCI_EndTransmit_IT(huart);
    return;
  }
}



>>>>>>> .r44
/* USER CODE END 1 */
