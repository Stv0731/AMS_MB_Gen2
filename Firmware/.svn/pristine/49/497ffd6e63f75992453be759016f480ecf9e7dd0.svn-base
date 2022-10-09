/*******************************************************************************
 * File Name  : usb_interface.h
 *
 * Abstract   : 
 *
 * History    : 1.00  (2021-12-22)
 *
 * Author     : Steven
 *
 * Copyright (C) 2021~2022 YXSENS Corporation. All rights reserved.
 *
*******************************************************************************/

#ifndef _DEVICE_SCI_H_
#define _DEVICE_SCI_H_

#ifdef  __cplusplus
extern "C" {
#endif


typedef struct
{
  uint8_t       buf[200];
  uint8_t       wPtr;
  uint32_t      tick;
}
_DEV_SCI_PACKET;

typedef enum
{
    USB_EV_RCV_DATA = 1,
    USB_EV_SND_DATA,
    USB_EV_SND_COMPLETET,
    USB_EV_ERROR
}
_USB_Event;

typedef struct
{
  uint32_t      event;
  uint8_t       buf[12];
}
_USB_QueueType;

typedef struct
{
	uint8_t*    p_dat;
    uint16_t    length;
}USBTransDataType;

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


void Usb_Interface_Init(void);
void Usb_Send_MsgProduce(uint8_t* Buf, uint16_t Len);
void Usb_Recv_MsgProduce(uint8_t* Buf, uint16_t Len);
void SendSampleData(void);

#ifdef  __cplusplus
}
#endif

#endif
/* _DEVICE_SCI_H_ */

