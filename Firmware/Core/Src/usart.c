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
/* USER CODE END 0 */

UART_HandleTypeDef huart2;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  SCI_Start_Receive_IT(&huart2, sci_rcv_pack[sci_rcv_pack_ndx]);
  
  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

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



/* USER CODE END 1 */
