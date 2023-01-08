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

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */

<<<<<<< .mine
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
  uint8_t       dev;
  uint8_t       event;
  uint8_t       buf[14];
}
_SCI_QueueType;

typedef struct
{
	uint8_t*    p_dat;
    uint16_t    length;
}SCITransDataType;

/* Parameters  */
typedef struct {
    uint32_t baud;                      /* ²¨ÌØÂÊ */
    uint32_t hal_idel_timeout;              /* Êý¾Ý°ü½áÊø³¬Ê±Ê±¼ä,µ¥Î»ms */
    uint32_t tr_timeout;                /* ·¢ËÍ×ª½ÓÊÜ¼ä¸ôÊ±¼ä,µ¥Î»ms */
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

typedef enum {
    DEV_SCI_1 = 0,
    DEV_SCI_2,
    NUM_OF_SCI
}_SCI_DEVICE;
#define DEV_SCI_PCCOM       DEV_SCI_1
#define DEV_SCI_RESERVED    DEV_SCI_2

typedef struct{
    _SCI_DEVICE dev;
    UART_HandleTypeDef *huart; 
    TimerHandle_t xTimer_SCI_ReceiveDone;
}_SCI_DEVICE_LIST;

||||||| .r32
=======
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
    uint32_t baud;                      /* æ³¢ç‰¹çŽ‡ */
    uint32_t hal_idel_timeout;              /* æ•°æ®åŒ…ç»“æŸè¶…æ—¶æ—¶é—´,å•ä½ms */
    uint32_t tr_timeout;                /* å‘é€è½¬æŽ¥å—é—´éš”æ—¶é—´,å•ä½ms */
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

>>>>>>> .r44
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
<<<<<<< .mine
void SCI_Interface_Init(void);
void SCI_Send_MsgProduce(_SCI_DEVICE dev, uint8_t* Buf, uint16_t Len);
void SCI_Recv_MsgProduce(_SCI_DEVICE dev, uint8_t isr);
void SCI_IRQHandler(_SCI_DEVICE dev);
void SendSampleDataFromSCI(void);
||||||| .r32
=======
void SCI_Interface_Init(void);
void SCI_Send_MsgProduce(uint8_t* Buf, uint16_t Len);
void SCI_Recv_MsgProduce(uint8_t isr);
void SCI_IRQHandler(UART_HandleTypeDef *huart);
void SendSampleDataFromSCI(void);
//void SendSampleData(void);
>>>>>>> .r44

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

