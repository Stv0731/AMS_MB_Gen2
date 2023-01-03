/*
********************************************************************************
*
* File : amp6100br.c
*
* Date : 2022/05/04
*
* Revision : 1.0.0
*
* Usage: The sensor AMP6100 configuration and operation interface
*
*******************************************************************************/
/******** Includes ************************************************************/
#include "main.h"
#include "string.h"

#include "ampComm.h"
#include "ampProcess.h"
#include "amp6100br.h"
#include "bmp3_common.h"
#include "amp8220.h"

extern void SendSampleData(void);

/******** Private definitions *************************************************/

/******** Private function declarations ***************************************/

/******** Variables ***********************************************************/
amp_object_t amp_entity;
static uint32_t raw_hisotry = 0xFFFFFFFF;

uint8_t AutoRespFlag = 0;       // bit0: USB, bit1: SCI

/******** Function Prototype **************************************************/
void vTaskAmpProcess(void* argument);

/*
 ** PRIVATE FUNCTION: amp_write_cmd()
 *
 *  DESCRIPTION:
 *      write the request command to sensor thru iic bus.
 *
 *  PARAMETERS:
 *      *pdat: the pointer to the command.
 *      len: the length of command in bytes.
 *             
 *  RETURNS:
 *      None.
 */  

/*
 ** PRIVATE FUNCTION: iir_prs_filter()
 *
 */ 
uint32_t iir_prs_filter(uint32_t raw, uint8_t iir)
{
	if (iir !=0 && raw_hisotry != 0xFFFFFFFF)
	{
		raw = ((uint64_t)raw + (uint64_t)(iir - 1) * raw_hisotry) / iir;
	}
	raw_hisotry = raw;
    return raw;
}

/*
 ** PUBLIC FUNCTION: Amp_ADC_Start()
 *
 */ 
void Amp_ADC_Start(uint8_t en)
{
    if (en)
        amp_entity.Started = TRUE;
    else
        amp_entity.Started = FALSE;
}

/*
 ** PUBLIC FUNCTION: Amp_IIR_Enable()
 *
 *  DESCRIPTION:
 *      config the I2C pin.
 *
 *  PARAMETERS:
 *      en: 0~16, 0: disable, !0: filterfactor
 *             
 *  RETURNS:
 *      None.
 */ 
void Amp_IIR_Enable(uint8_t en)
{
    amp_entity.iirCoef = en;
}

/*
 ** PUBLIC FUNCTION: Amp_ODR_Set()
 *
 */ 
void Amp_ODR_Set(uint16_t freq)
{
    if (freq >=1 && freq <= 1000){  // 1~1000Hz
        amp_entity.odr = freq; 
    }
}

/*
 ** PUBLIC FUNCTION: Amp_SensorType_Set()
 *
 */ 
void Amp_SensorType_Set(uint16_t type)
{
    if (type < NBROFAMP){
        amp_entity.SensorType = (_SensorType_t)type;
    }
}

/*
 ** PUBLIC FUNCTION: Amp_SampleChannel_Set()
 *
 */ 
void Amp_SampleChannel_Set(uint16_t ch)
{
    amp_entity.SampleChannel = (_SampleChannel_t)ch;
}


/*
 ** PRIVATE FUNCTION: AmpSensorInit()
 *
 */ 
void AmpProcessInit(void)
{
    amp_object_t *ampPtr;
    
    memset(&amp_entity, 0, sizeof(amp_object_t));
    
    ampPtr = &amp_entity;
    ampPtr->SensorType      = TYPE_NONE;
    ampPtr->SampleChannel   = CH_NONE;
    ampPtr->odr             = 10;       /* 1Hz */
    ampPtr->iirCoef         = 4;
    ampPtr->offset          = 0;
    ampPtr->Started         = FALSE;
    
    /* Task Init */
	xTaskCreate(vTaskAmpProcess,
		"AmpProcess",
		2048/4,  // In words, not bytes
		NULL,
		TASK_AMS_PRI,
		&pxSensorTask);
}

/*
 ** PUBLIC FUNCTION: vTaskAmpProcess()
 *
 *  DESCRIPTION:
 *      the amp process task.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
extern void SendSampleDataFromSCI(void);
uint8_t bmp3_start = 0;
void vTaskAmpProcess(void* argument) 
{
	uint32_t millisec = 2;
	TickType_t ticks = pdMS_TO_TICKS(millisec);
    
    Amp61XXInit();
    Amp8220Init();
    //bmp3_ss_init();

	for (;;) {
        if (bmp3_start == 0  && amp_entity.SensorType == BMP3XX){
            bmp3_start = 1;
            //portENTER_CRITICAL();
            //vTaskSuspend(StartTaskHandle);
            //vTaskSuspend(pxADCTask);
            //vTaskSuspend(pxUSBTask);
            //portEXIT_CRITICAL();
            bmp3_ss_init();
            //portENTER_CRITICAL();
            //vTaskResume(StartTaskHandle);
            //vTaskResume(pxADCTask);
            //vTaskResume(pxUSBTask);
            //portEXIT_CRITICAL();
        }
        
        if (amp_entity.Started)
        {
            uint8_t ret = FALSE;
            
            switch(amp_entity.SensorType)
            {
                case AMP6120:
                case AMP6127:
                    ret = AMP61XXSampleProcess(millisec);
                    //SendSampleData();
                    break;
                    
                case AMP8220:
                    ret = AMP8220SampleProcess(millisec);
                    //if(ret) SendSampleData();
                    break;
                    
                case BMP3XX:
                    if (bmp3_start != 0)
                        BMP3XXSampleProcess(millisec);
                    break;
            }
            if (ret != FALSE){
                if (AutoRespFlag & 0x01)
                    SendSampleData();
                if (AutoRespFlag & 0x02)
                    SendSampleDataFromSCI();
            }
        }
        
        vTaskDelay(ticks);
    }
}


/*******************END OF FILE************************************************/