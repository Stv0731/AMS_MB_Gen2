/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

typedef struct
{
  uint8_t       buf[200];
  uint8_t       wPtr;
  uint32_t      tick;
}
_DEV_SCI_PACKET;

typedef enum
{
    SCI_EV_RCV_DATA = 1,
    SCI_EV_SND_DATA,
    SCI_EV_SND_COMPLETET,
    SCI_EV_ERROR
}
_SCI_Event;

typedef struct
{
  uint32_t      event;
  uint8_t       buf[12];
}
_SCI_QueueType;

typedef struct
{
	uint8_t*    p_dat;
    uint16_t    length;
}SCITransDataType;

/* Parameters  */
typedef struct {
    uint32_t baud;                      /* 波特率 */
    uint32_t hal_idel_timeout;              /* 数据包结束超时时间,单位ms */
    uint32_t tr_timeout;                /* 发送转接受间隔时间,单位ms */
}_SCI_PARAM;

/* function defined for DevSci_Ioctl() */
#define DEV_SCI_FC_CONFIG_SET           0
#define DEV_SCI_FC_CONFIG_GET           1
#define DEV_SCI_FC_STATUS_GET           2
#define DEV_SCI_FC_DATA_READY_GET       3

enum SCI_HANLDE_RETURN_DEF
{
    SCI_HDL_NOERR = 0,
    SCI_HDL_ERR,
    SCI_HDL_ERR2
};

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void SCI_Interface_Init(void);
void SCI_Send_MsgProduce(uint8_t* Buf, uint16_t Len);
void SCI_Recv_MsgProduce(uint8_t isr);
void SCI_IRQHandler(UART_HandleTypeDef *huart);
void SendSampleDataFromSCI(void);
//void SendSampleData(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

