/*
********************************************************************************
*
* File : amp8220.c
*
* Date : 2022/10/30
*
* Revision : 1.0.0
*
* Usage: The sensor AMP8220 configuration and operation interface.
*
*******************************************************************************/
/******** Includes ************************************************************/
#include "main.h"
#include "string.h"

#include "amp8220.h"
#include "adc.h"

/******** Private definitions *************************************************/

/******** Private function declarations ***************************************/

/******** Variables ***********************************************************/
amp_8220_t amp8220_entity;

/******** Function Prototype **************************************************/
// VOUT = VDD * (a * P + b)
// P = (Vout/Vdd - b) / a (Kpa)
// Vdd = VDD
// Vout = 
// a = 0.0081
// b = -0.00095
uint32_t PressCalculate(float Vout)
{
	float tmp;
	tmp = ((float)Vout / amp8220_entity.VDD - amp8220_entity.b)/amp8220_entity.a * AMP8220_PRS_SCALE;
	
	return (uint32_t)(tmp);
}


/*
 ** PUBLIC FUNCTION: PresDat_Convert()
 *
 */ 
static uint32_t PresDat_Convert(uint32_t vout, uint8_t iir, int32_t pres_max, int32_t pres_min)
{
    uint32_t tmp = 0;
    
    if (iir){
        uint64_t tmp64;

        tmp64 = iir_prs_filter(vout, iir);
        tmp = PressCalculate((float)tmp64/1000.0);
    }
    else{
        tmp = PressCalculate((float)vout/1000.0);
    }
    
    tmp  = (tmp > pres_max) ? pres_max : ((tmp < pres_min) ? pres_min : tmp);
    
    return (uint32_t)tmp;
}

/*
 ** PUBLIC FUNCTION: SensorSampleOutput()
 *
 */ 
static uint16_t cyclebyms;
extern uint16_t AnalogRawData[];
UINT8 AMP8220SampleProcess(UINT32 timebyms)
{
    static uint16_t timer;
    uint16_t raw = 0xFFFF;
    uint8_t ret = FALSE;
    amp_8220_t *entity = &amp8220_entity; 
	
    if (entity->parent == NULL || 
        !(entity->parent->SensorType==AMP8220) ||
        !( entity->parent->SampleChannel == ANALOG1 
		|| entity->parent->SampleChannel == ANALOG2
		|| entity->parent->SampleChannel == ANALOG3
		|| entity->parent->SampleChannel == ANALOG4
		)){
        return ret;
    }
    if (entity->parent->Started != TRUE){
        return ret;
    }
	
    cyclebyms = (uint16_t)((float)1000.0/entity->parent->odr) + 1;
    
    timer++;
    if (timer*timebyms >= cyclebyms){
        timer = 0;
        
        /* 1. Read ADC value */
		switch (entity->parent->SampleChannel){
        case ANALOG1:
            raw = ADC_GetValue(AD1_CHANNEL);
            break;
        default:
            break;
		}

        /* 3. Convert AD result */
        
        if (raw != 0xFFFF){
            entity->ADC_raw = raw;
            entity->Vout = (uint32_t)raw * 3300 * 2 / 4095;
            
            entity->pres_real = PresDat_Convert(entity->Vout, entity->parent->iirCoef, AMP8220_PRS_MAX*AMP8220_PRS_SCALE, AMP8220_PRS_MIN * AMP8220_PRS_SCALE);
        }

        entity->parent->pres_raw = entity->Vout;
        entity->parent->pres_rt  = entity->pres_real;
        entity->parent->temp_raw = 0;//entity->temp_raw;
        entity->parent->temp_rt  = 0;
        ret = TRUE;
    }
    return ret;
}

/*
 ** PUBLIC FUNCTION: AmpSensorInit()
 *
 */ 
void Amp8220Init(void)
{
    amp_8220_t *entity;
    
    entity = &amp8220_entity;
    entity->parent = &amp_entity;
    entity->metype = AMP8220;
    
    entity->a   = 0.0078;//0.0081;
    entity->b   = 0.0019;//-0.00095;
    entity->VDD = 5.0;
}
/*******************END OF FILE************************************************/