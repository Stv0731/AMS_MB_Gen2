/*
********************************************************************************
*
* File : amp8220.h
*
* Date : 2022/10/30
*
* Revision : 1.0.0
*
* Usage: The header file for the sensor AMP8220 configuration and operation interface.
*
*******************************************************************************/
#ifndef __AMP8220_H
#define __AMP8220_H

#include "ampComm.h"

#define AMP6100BR_DEV_ID        0x78


#define AMP8220_IIR_filter_default     8       /* IIR filter: default=8, range: 1~32                */
#define AMP8220_odr_default            1       /* Output data rate: default=1Hz, range: 1~100Hz     */

#define AMP8220_PRS_MIN	      10             /*  10,K Pa    */
#define AMP8220_PRS_MAX	    1150             /* 115,K Pa    */

#define AMP8220_PRS_SCALE       10


typedef struct{
    amp_object_t*   parent;
    _SensorType_t   metype;
	uint16_t		ADC_raw;
	uint16_t        Vout;
	uint32_t		pres_real;
	
	float           a;
	float           b;
    float           VDD;
}
amp_8220_t;
extern amp_8220_t amp8220_entity;

/** Global function Declarations */

UINT8 AMP8220SampleProcess(UINT32 timebyms);
void Amp8220Init(void);

#endif /* End of amp8220 include */
/*******************END OF FILE******************************************************************************/