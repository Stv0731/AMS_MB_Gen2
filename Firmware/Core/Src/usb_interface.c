/*******************************************************************************
 * File Name  : usb_interface.c
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

#include "main.h"
     
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "usb_interface.h"

#include "ampComm.h"
#include "ampProcess.h"
#include "amp6100br.h"
#include "bmp3_common.h"
#include "tmp117.h"
     
extern uint8_t AutoRespFlag;       // bit0: USB, bit1: SCI

QueueHandle_t xQueue_Usb;

#define USB_RCV_PACK_SIZE    256
#define USB_RXV_PACK_NUM     10
static uint8_t usb_rcv_pack_ndx = 0;
static uint8_t usb_trs_pack_ndx = 0;
static uint8_t usb_rcv_pack[USB_RXV_PACK_NUM][USB_RCV_PACK_SIZE];
static uint8_t usb_resp_pack[USB_RXV_PACK_NUM][USB_RCV_PACK_SIZE];

static uint8_t auto_resp_delay = 0;

void UsbCdc_MsgSend(uint8_t event, uint8_t fd, uint8_t data );
static void vTask_USB_Comm(void* argument);

/*
 ** PUBLIC FUNCTION: Usb_Interface_Init()
 *
 *  DESCRIPTION:
 *      Initialize the usb interface.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
void Usb_Interface_Init(void) {
    
	xTaskCreate(vTask_USB_Comm,
		"USB CDC Comm",
		2048/4,  // In words, not bytes
		NULL,
		TASK_USB_PRI,
		&pxUSBTask);
}

void usb_rcvd_pack_process(uint8_t *buf)
{
    USBTransDataType *pd = (USBTransDataType*)buf;
    uint16_t len = pd->length;
    char* pString = (char*)pd->p_dat;
    char* presp = (char*)usb_resp_pack[/*usb_trs_pack_ndx*/0];
    uint8_t len_resp;
    
    if( memcmp((char*)pString, "AT", len) == 0 ){
        usb_trs_pack_ndx = usb_trs_pack_ndx == (USB_RXV_PACK_NUM-1) ? 0 : (usb_trs_pack_ndx+1); 
        presp = "OK";
        len_resp = strlen(presp);
        Usb_Send_MsgProduce((uint8_t*)presp, len_resp);
    }
    else if ( memcmp((char*)pString, "AT+REG+", strlen("AT+REG+")) == 0 ){      // None used
        uint32_t reg;
        usb_trs_pack_ndx = usb_trs_pack_ndx == (USB_RXV_PACK_NUM-1) ? 0 : (usb_trs_pack_ndx+1); 
        pString += strlen("AT+REG+");
        reg	= atoi(pString);
        
        pString	= strchr(pString,'?');
        if (pString != NULL){
            sprintf ( presp, "+REG+%d:%d", reg, amp61xx_entity.pRegister->array[reg]);
            len_resp = strlen(presp);
            Usb_Send_MsgProduce((uint8_t*)presp, len_resp);
        }
    }
    else if ( memcmp((char*)pString, "AT+CO_T+", strlen("AT+CO_T+")) == 0 ){    // None used
        uint8_t ndx;
        usb_trs_pack_ndx = usb_trs_pack_ndx == (USB_RXV_PACK_NUM-1) ? 0 : (usb_trs_pack_ndx+1); 
        pString += strlen("AT+CO_T+");
        ndx	= atoi(pString);

        pString	= strchr(pString,'?');
        if (pString != NULL){
            sprintf ( presp, "+CO_T+%d:%d", ndx, amp61xx_entity.co_t_array[ndx-1]);
            len_resp = strlen(presp);
            Usb_Send_MsgProduce((uint8_t*)presp, len_resp);
        }
    }
    else if ( memcmp((char*)pString, "AT+CO_P+", strlen("AT+CO_P+")) == 0 ){    // None used
        uint8_t ndx;
        usb_trs_pack_ndx = usb_trs_pack_ndx == (USB_RXV_PACK_NUM-1) ? 0 : (usb_trs_pack_ndx+1);
        pString += strlen("AT+CO_P+");
        ndx	= atoi(pString);

        pString	= strchr(pString,'?');
        if (pString != NULL){
            sprintf ( presp, "+CO_P+%d:%d", ndx, amp61xx_entity.co_p_array[ndx-1]);
            len_resp = strlen(presp);
            Usb_Send_MsgProduce((uint8_t*)presp, len_resp);
        }
    }
    else if ( memcmp((char*)pString, "AT+PRES?", strlen("AT+PRES?")) == 0 ){    // None used
        usb_trs_pack_ndx = usb_trs_pack_ndx == (USB_RXV_PACK_NUM-1) ? 0 : (usb_trs_pack_ndx+1); 
        sprintf ( presp, "+PRES: %d, %d", amp_entity.pres_raw, amp_entity.pres_rt);
        len_resp = strlen(presp);
        Usb_Send_MsgProduce((uint8_t*)presp, len_resp);
        
    }
    else if ( memcmp((char*)pString, "AT+TEMP?", strlen("AT+TEMP?")) == 0 ){    // None used
        usb_trs_pack_ndx = usb_trs_pack_ndx == (USB_RXV_PACK_NUM-1) ? 0 : (usb_trs_pack_ndx+1); 
        sprintf ( presp, "+TEMP: %d, %d", amp_entity.temp_raw, amp_entity.temp_rt);
        len_resp = strlen(presp);
        Usb_Send_MsgProduce((uint8_t*)presp, len_resp);
        
    }
    else if ( memcmp((char*)pString, "AT+SAMPLES?", strlen("AT+SAMPLES?")) == 0 ){
        usb_trs_pack_ndx = usb_trs_pack_ndx == (USB_RXV_PACK_NUM-1) ? 0 : (usb_trs_pack_ndx+1); 
        sprintf ( presp, "+SAMPLES: %d, %d, %d, %d", amp_entity.pres_raw, amp_entity.pres_rt, amp_entity.temp_raw, amp_entity.temp_rt);
        len_resp = strlen(presp);
        Usb_Send_MsgProduce((uint8_t*)presp, len_resp);
        
    }
    else if ( memcmp((char*)pString, "AT+CMD=", strlen("AT+CMD=")) == 0 ){      // AMP61XX
        uint8_t tcmd = 0;
        usb_trs_pack_ndx = usb_trs_pack_ndx == (USB_RXV_PACK_NUM-1) ? 0 : (usb_trs_pack_ndx+1); 
        pString += strlen("AT+CMD=");
        tcmd = atoi(pString);

        if (amp_entity.SensorType == AMP6120 || amp_entity.SensorType == AMP6127){
            Amp_Control_Req(tcmd, 0, 0);
            
            presp = "OK";
            len_resp = strlen(presp);
            Usb_Send_MsgProduce((uint8_t*)presp, len_resp);
        }
        else if (amp_entity.SensorType == BMP3XX){
            bmp3_osrp_config(tcmd&0x07);
        }
        else{
            presp = "Failed";
            len_resp = strlen(presp);
            Usb_Send_MsgProduce((uint8_t*)presp, len_resp);
        }
    }
    else if ( memcmp((char*)pString, "AT+ADC=", strlen("AT+ADC=")) == 0 ){
        uint8_t en;
        usb_trs_pack_ndx = usb_trs_pack_ndx == (USB_RXV_PACK_NUM-1) ? 0 : (usb_trs_pack_ndx+1); 
        pString += strlen("AT+ADC=");
        en = atoi(pString);
        Amp_Control_Req(en, 0, 0);
        
        presp = "OK";
        len_resp = strlen(presp);
        Usb_Send_MsgProduce((uint8_t*)presp, len_resp);
    }
    else if ( memcmp((char*)pString, "AT+START=", strlen("AT+START=")) == 0 ){
        uint8_t en;
        usb_trs_pack_ndx = usb_trs_pack_ndx == (USB_RXV_PACK_NUM-1) ? 0 : (usb_trs_pack_ndx+1); 
        pString += strlen("AT+START=");
        en = atoi(pString);
        Amp_ADC_Start(en);
        if (en)
            AutoRespFlag |= 0x01;
        else
            AutoRespFlag &= ~0x01;
        presp = "OK";
        len_resp = strlen(presp);
        Usb_Send_MsgProduce((uint8_t*)presp, len_resp);
        
    }
    else if ( memcmp((char*)pString, "AT+IIR=", strlen("AT+IIR=")) == 0 ){
        uint8_t en;
        usb_trs_pack_ndx = usb_trs_pack_ndx == (USB_RXV_PACK_NUM-1) ? 0 : (usb_trs_pack_ndx+1); 
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
        Usb_Send_MsgProduce((uint8_t*)presp, len_resp);
        
    }
    else if ( memcmp((char*)pString, "AT+ODR=", strlen("AT+ODR=")) == 0 ){
        uint16_t freq;
        usb_trs_pack_ndx = usb_trs_pack_ndx == (USB_RXV_PACK_NUM-1) ? 0 : (usb_trs_pack_ndx+1); 
        pString += strlen("AT+ODR=");
        freq = atoi(pString);
        Amp_ODR_Set(freq);
        if (amp_entity.SensorType == BMP3XX){
            bmp_odr_config((float)freq);
        }
            
        presp = "OK";
        len_resp = strlen(presp);
        Usb_Send_MsgProduce((uint8_t*)presp, len_resp);
        
    }
    else if ( memcmp((char*)pString, "AT+SENSORTYPE=", strlen("AT+SENSORTYPE=")) == 0 ){
        uint16_t type;
        usb_trs_pack_ndx = usb_trs_pack_ndx == (USB_RXV_PACK_NUM-1) ? 0 : (usb_trs_pack_ndx+1); 
        pString += strlen("AT+SENSORTYPE=");
        type = atoi(pString);
        Amp_SensorType_Set(type);
        
        presp = "OK";
        len_resp = strlen(presp);
        Usb_Send_MsgProduce((uint8_t*)presp, len_resp);
        
    }
    else if ( memcmp((char*)pString, "AT+SENSORCHANNEL=", strlen("AT+SENSORCHANNEL=")) == 0 ){
        uint16_t ch;
        usb_trs_pack_ndx = usb_trs_pack_ndx == (USB_RXV_PACK_NUM-1) ? 0 : (usb_trs_pack_ndx+1); 
        pString += strlen("AT+SENSORCHANNEL=");
        ch = atoi(pString);
        Amp_SampleChannel_Set(ch);
        
        presp = "OK";
        len_resp = strlen(presp);
        Usb_Send_MsgProduce((uint8_t*)presp, len_resp);
        
    }
    
//    if (len == 5 && pd->p_dat[0] == 0xAA && pd->p_dat[1] == 0xAB){
//        Usb_Send_MsgProduce(pd->p_dat, len);
//    }
    auto_resp_delay = 2;
    memset(pd->p_dat, 0, len);
}

void SendSampleData(void)
{
    char* presp = (char*)usb_resp_pack[/*usb_trs_pack_ndx*/0];
    int16_t tmp117 = GetTMP117RtdTemperature();
    uint8_t len_resp;
    
    if (auto_resp_delay != 0)
        return;
    
    usb_trs_pack_ndx = usb_trs_pack_ndx == (USB_RXV_PACK_NUM-1) ? 0 : (usb_trs_pack_ndx+1); 
    sprintf ( presp, "+SAMPLES: %d, %d, %d, %d, %d", amp_entity.pres_raw, amp_entity.pres_rt, amp_entity.temp_raw, amp_entity.temp_rt, tmp117);
    len_resp = strlen(presp);
    Usb_Send_MsgProduce((uint8_t*)presp, len_resp);
}

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
void usb_send_pack_process(uint8_t *buf)
{
    USBTransDataType *pd = (USBTransDataType*)buf;
    
    CDC_Transmit_FS(pd->p_dat, pd->length);
}


/*
 ** PUBLIC FUNCTION: vTask_DevSCI_Driver()
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

static void vTask_USB_Comm(void* argument)
{
    _USB_QueueType msg;
	uint32_t millisec = 100;//1000;
	TickType_t ticks = pdMS_TO_TICKS(millisec);

    xQueue_Usb	= xQueueCreate( 256, ( UBaseType_t ) sizeof( _USB_QueueType ) );
    
	for (;;) {
        if( xQueueReceive( xQueue_Usb, (_USB_QueueType*)&msg, ticks ) == pdTRUE )
        {
            switch(msg.event)
            {
            case USB_EV_RCV_DATA:
                usb_rcvd_pack_process(msg.buf);
                break;
                
            case USB_EV_SND_DATA:
                usb_send_pack_process(msg.buf);
                break;
                
            case USB_EV_ERROR:

                break; 
                
            default:
                break;            
            }
        }
        else
        {
            //TimeOut_Check( xTaskGetTickCount() );
//            char *st = "This is my first usb cdc port!";
//            Usb_Send_MsgProduce(st, 30);
            if (auto_resp_delay) auto_resp_delay--;
        }
        
	}
}
/*
 ** PUBLIC FUNCTION: DevSCI_SciMsgSend()
 *
 *  DESCRIPTION:
 *      The task for all SCI driver.
 *
 *  PARAMETERS:
 *      fd          :   the device ID
 *      event       :   sci event
 *      data        :   receive data
 *
 *  RETURNS:
 *       None.
 */
void Usb_MsgSend(uint8_t event, USBTransDataType* p_dat )
{
    _USB_QueueType msg;
    uint8_t *p = (uint8_t*)p_dat;
    uint8_t dat_len = sizeof(USBTransDataType);
    if(NULL == xQueue_Usb)
        return;
    
    msg.event   = event;
    
    if( p != NULL && dat_len < sizeof(_USB_QueueType) ) 
        memcpy( msg.buf, (uint8_t*)p_dat, dat_len );
    
    xQueueSend( xQueue_Usb, ( _USB_QueueType * ) &msg, ( TickType_t ) 100 );
}

void usb_MsgSend_FromISR(uint8_t event, USBTransDataType* p_dat )
{
    _USB_QueueType msg;
    uint8_t *p = (uint8_t*)p_dat;
    uint8_t dat_len = sizeof(USBTransDataType);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;  
    
    if(NULL == xQueue_Usb)
        return;
    
    msg.event   = event;
    if( p != NULL && dat_len < sizeof(_USB_QueueType) ) 
        memcpy( msg.buf, (uint8_t*)p_dat, dat_len );
    
    xQueueSendFromISR( xQueue_Usb, &msg, &xHigherPriorityTaskWoken );
    
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

void Usb_Send_MsgProduce(uint8_t* Buf, uint16_t Len)
{
    USBTransDataType dat;
    
    dat.length = Len;
    dat.p_dat = Buf;
    
    Usb_MsgSend(USB_EV_SND_DATA, &dat);
}

void Usb_Recv_MsgProduce(uint8_t* Buf, uint16_t Len)
{
    USBTransDataType dat;
    
    for (uint8_t i = 0; i < Len; i++){
        usb_rcv_pack[usb_rcv_pack_ndx][i] = Buf[i];
    }
    
    dat.length = Len;
    dat.p_dat = usb_rcv_pack[usb_rcv_pack_ndx];
    usb_rcv_pack_ndx++;
    if (usb_rcv_pack_ndx == USB_RXV_PACK_NUM)
        usb_rcv_pack_ndx = 0;
    
    usb_MsgSend_FromISR(USB_EV_RCV_DATA, &dat);
}






/*------------------------ End of source file --------------------------------*/
/*------------------------ Nothing Below This Line ---------------------------*/
