/*
********************************************************************************
*
* File : amp6100br.c
*
* Date : 2022/05/04
*
* Revision : 1.0.0
*
* Usage: The sensor AMP61xx configuration and operation interface.
*        Including AMP6120, AMP6127,
*
*******************************************************************************/
/******** Includes ************************************************************/
#include "main.h"
#include "string.h"

#include "amp6100br.h"

/******** Private definitions *************************************************/

/******** Private function declarations ***************************************/

/******** Variables ***********************************************************/
amp_register_t  amp_Register;
amp_61xx_t    amp61xx_entity;

/******** Function Prototype **************************************************/
void vTask_amp6100br(void* argument);

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
u8 amp_write_cmd(u8 *pdat, u8 len)
{
    u8 dev = amp61xx_entity.portID;
    u8 ret = FALSE;
    
    if (amp61xx_entity.parent != NULL){
        if (amp61xx_entity.parent->SampleChannel == I2C_CH1){
            ret = bsp_iic_writeBytes(dev, len, pdat);
        }
        else if (amp61xx_entity.parent->SampleChannel == I2C_CH2){
            //ret = bsp_iic_writeBytes(dev, len, pdat);
        }
    }

    return ret;
}

/*
 ** PRIVATE FUNCTION: amp_read_stus()
 *
 */  
u8 amp_read_stus(void)
{  
    u8 dev = amp61xx_entity.portID;//AMP6100BR_DEV_ID;
    u8 stus;
    
    if (amp61xx_entity.parent != NULL){
        if (amp61xx_entity.parent->SampleChannel == I2C_CH1){
            bsp_iic_readBytes(dev, 1, (u8*)&stus);
        }
        else if (amp61xx_entity.parent->SampleChannel == I2C_CH2){
            //bsp_iic_readBytes(dev, 1, (u8*)&stus);
        }
    }

    return stus;
}
/*
 ** PRIVATE FUNCTION: amp_read_bytes()
 *
 */  
u8 amp_read_bytes(u8 *dst, u8 len)
{
    u8 dev = amp61xx_entity.portID;//AMP6100BR_DEV_ID;
    u8 ret = FALSE;
    
    if (amp61xx_entity.parent != NULL){
        if (amp61xx_entity.parent->SampleChannel == I2C_CH1){
            ret = bsp_iic_readBytes(dev, len, dst);
        }
        else if (amp61xx_entity.parent->SampleChannel == I2C_CH2){
            //ret = bsp_iic_readBytes(dev, len, dst);
        }
    }
    return ret;
}

/*
 ** PRIVATE FUNCTION: amp_read_ADResult()
 *
 */  
u8 amp_read_ADResult(u8 *pDat, u8 size)
{
    u8 dev = amp61xx_entity.portID;
    
    if (amp61xx_entity.parent != NULL){
        if (amp61xx_entity.parent->SampleChannel == I2C_CH1){
            if (bsp_iic_readBytes(dev, size, pDat)){
                return TRUE;
            }
        }
        else if (amp61xx_entity.parent->SampleChannel == I2C_CH2){
//            if (bsp_iic_readBytes(dev, size, pDat)){
//                return TRUE;
//            }
        }
    }

    return FALSE;
}

/*
 ** PRIVATE FUNCTION: TempDat_Convert()
 *
 */ 
s16 TempDat_Convert(u16 raw, u8 iir, s16 temp_max, s16 temp_min)
{
    s16 tmp = 0;
    
    if (iir){
        tmp = ((s32)raw * (temp_max - temp_min) >> 16) + temp_min;
    }
    else{
        tmp = ((s32)raw * (temp_max - temp_min) >> 16) + temp_min;
    }
    
    tmp  = (tmp > temp_max) ? temp_max : ((tmp < temp_min) ? temp_min : tmp);
    
    return tmp;
}

/*
 ** PRIVATE FUNCTION: PresDat_Convert()
 *
 */ 
u32 PresDat_Convert(u32 raw, u8 iir, s32 pres_max, s32 pres_min)
{
    u32 tmp = 0;
    
    if (iir){
        u64 tmp64;

        tmp64 = iir_prs_filter(raw, iir);
        tmp = ((u64)tmp64 * (pres_max - pres_min) >> 24) + pres_min;
    }
    else{
        tmp = ((u64)raw * (pres_max - pres_min) >> 24) + pres_min;
    }
    
    tmp  = (tmp > pres_max) ? pres_max : ((tmp < pres_min) ? pres_min : tmp);
    
    return (u32)tmp;
}
/*
 ** PUBLIC FUNCTION: Amp_Control_Req()
 *
 */ 
void Amp_Control_Req(u8 cmd, u8 osr_p, u8 osr_t)
{
    u8 t_cmd = cmd;
    switch(cmd){
    case 0xAC:
        amp61xx_entity.cmd_pres = t_cmd;
        break;
    default:
        if ((cmd&0xB0) == 0xB0){
            amp61xx_entity.cmd_pres = t_cmd;
            switch((cmd>>3)&0x01){
            case 0:
                amp61xx_entity.osr_t = 4;
                break;
            default:
                amp61xx_entity.osr_t = 8;
            }
            switch(cmd&0x07){
            case 0: amp61xx_entity.osr_p = 128; break;
            case 1: amp61xx_entity.osr_p = 64; break;
            case 2: amp61xx_entity.osr_p = 32; break;
            case 3: amp61xx_entity.osr_p = 16; break;
            case 4: amp61xx_entity.osr_p = 8; break;
            case 5: amp61xx_entity.osr_p = 4; break;
            case 6: amp61xx_entity.osr_p = 2; break;
            case 7: amp61xx_entity.osr_p = 1; break;                
            default: break;
            }
        }
        break;
    }
}

/*
 ** PRIVATE FUNCTION: osr_config_set()
 *
 */ 
u8 Amp61XX_Range_Set(amp_61xx_t *entity)
{
    u8 ret = FALSE;
    if (entity->metype != entity->parent->SensorType){
        entity->metype = entity->parent->SensorType;
        if (entity->metype == AMP6120){
            entity->tempDat_max = AMP6120_TMP_MAX;
            entity->tempDat_min = AMP6120_TMP_MIN;
            entity->presDat_max = AMP6120_PRS_MAX * AMP61XX_PRS_SCALE;  // 120000pa
            entity->presDat_min = AMP6120_PRS_MIN * AMP61XX_PRS_SCALE;   //  20000pa
            ret = TRUE;
        }
        else if (entity->metype == AMP6127){
            entity->tempDat_max = AMP6127_TMP_MAX;
            entity->tempDat_min = AMP6127_TMP_MIN;
            entity->presDat_max = AMP6127_PRS_MAX * AMP61XX_PRS_SCALE;  // 120000pa
            entity->presDat_min = AMP6127_PRS_MIN * AMP61XX_PRS_SCALE;   //  20000pa
            ret = TRUE;
        }
    }
    return ret;
}


/*
 ** PUBLIC FUNCTION: SensorSampleOutput()
 *
 */ 
u16 timeDelay = 0;
u8 tmpbuf[10];
u32 sampleholdingtime = 0;
u32 sampletimer = 0;
u32 sampletimer2 = 0;
u16 cyclebyms;
extern uint16_t AnalogRawData[];
UINT8 AMP61XXSampleProcess(UINT32 timebyms)
{
    static u16 timer;
    //u16 cyclebyms;
    u16 raw_temp=0xFFFF;//, timeDelay;
    u32 raw_pres=0x00FFFFFF;
    u8 cmd;
    uint8_t ret = FALSE;
    amp_61xx_t *entity = &amp61xx_entity; 
    if (entity->parent == NULL || 
        !(entity->parent->SensorType==AMP6120 || entity->parent->SensorType==AMP6127) ||
        !(entity->parent->SampleChannel == I2C_CH1 || entity->parent->SampleChannel == I2C_CH2)){
        return ret;
    }
    if (entity->parent->Started != TRUE){
        return ret;
    }
    cyclebyms = (u16)((float)1000.0/entity->parent->odr) + 1;
    
    Amp61XX_Range_Set(entity);
    
    timer++;
    if (timer*timebyms >= cyclebyms){
        timer = 0;
        cmd = entity->cmd_pres;
        
        /* 1. Start ADC */
        tmpbuf[0] = cmd;
        amp_write_cmd(tmpbuf, 1);
        sampletimer = HAL_GetTick();
        /* 2. Waiting */
        vTaskDelay(2);//delay_us(5000);
        for(timeDelay = 0; timeDelay < 0x02FF; timeDelay++){
            amp_stus_t stus;
            stus.all = amp_read_stus();
            
            entity->stus.all = stus.all;
            if (stus.bit.busy == 0){
                break;
            }
        }
        sampletimer2 = HAL_GetTick();
        sampleholdingtime = sampletimer2 - sampletimer;
        
        /* 3. Read AD result */
        if (amp_read_ADResult(tmpbuf, 6)){
            entity->stus.all = tmpbuf[0];
            raw_pres = ((u32)tmpbuf[1]<<16) + ((u32)tmpbuf[2]<<8) + ((u32)tmpbuf[3]<<0);
            raw_temp = ((u16)tmpbuf[4]<< 8) + ((u16)tmpbuf[5]<<0);
        }
        
        /* 4. Convert raw to real value */
        if (raw_pres!=0x00FFFFFF){
            entity->pres_raw = raw_pres;
            entity->pres_rt = PresDat_Convert(raw_pres, entity->parent->iirCoef, entity->presDat_max, entity->presDat_min);
            //entity->pres_raw = raw_hisotry;
        }
        if (raw_temp!=0xFFFF){
            entity->temp_raw = raw_temp;
            entity->temp_rt = TempDat_Convert(raw_temp, entity->parent->iirCoef, entity->tempDat_max, entity->tempDat_min);
        }
            
        sampleholdingtime = HAL_GetTick() - sampletimer;

        entity->parent->pres_raw = entity->pres_raw;
        entity->parent->pres_rt  = entity->pres_rt;
        entity->parent->temp_raw = AnalogRawData[1];//entity->temp_raw;
        entity->parent->temp_rt  = entity->temp_rt;
        ret = TRUE;
    }
    return ret;
}

/*
 ** PRIVATE FUNCTION: osr_config_set()
 *
 */ 
u8 osr_config_set(u8 osr_p, u8 osr_t)
{
	u8 cmd = 0xB0;
	
	if (osr_t == 8){
		cmd |= 0x08;
	}
	switch(osr_p){
		case 1:  cmd |= 0x07; break;
		case 2:  cmd |= 0x06; break;
		case 4:  cmd |= 0x05; break;
		case 8:  cmd |= 0x04; break;
		case 16: cmd |= 0x03; break;
		case 32: cmd |= 0x02; break;
		case 64: cmd |= 0x01; break;
		default: break;
	}
	return cmd;
}

/*
 ** PRIVATE FUNCTION: AmpSensorInit()
 *
 */ 
void Amp61XXInit(void)
{
    amp_61xx_t *entity;
    
    memset(&amp_Register, 0, sizeof(amp_register_t));
    memset(&amp61xx_entity, 0, sizeof(amp_61xx_t));
    
    entity = &amp61xx_entity;
    entity->parent = &amp_entity;
    entity->metype = AMP6120;
    
    entity->portID = AMP6100BR_DEV_ID;
    entity->pRegister = &amp_Register;
    
    entity->osr_p = AMP612X_osr_p_default;
    entity->osr_t = AMP612X_osr_t_default;
    
    entity->tempDat_max = AMP6120_TMP_MAX;
    entity->tempDat_min = AMP6120_TMP_MIN;
    entity->presDat_max = AMP6120_PRS_MAX * AMP61XX_PRS_SCALE;  // 120000pa
    entity->presDat_min = AMP6120_PRS_MIN * AMP61XX_PRS_SCALE;   //  20000pa
    
    entity->cmd_pres = osr_config_set(entity->osr_p, entity->osr_t);
}
/*******************END OF FILE************************************************/