/*
********************************************************************************
*
* File : tmp117.c
*
* Date : 2023/01/05
*
* Revision : 1.0.0
*
* Usage: The sensor tmp117 configuration and operation interface.
*
*******************************************************************************/
/******** Includes ************************************************************/
#include "main.h"
#include "string.h"

#include "tmp117.h"

/******** Private definitions *************************************************/

/******** Private function declarations ***************************************/

/******** Variables ***********************************************************/
tmp117_t tmp117_entity;

/******** Function Prototype **************************************************/

/*
 ** PRIVATE FUNCTION: tmp117_write()
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
static u8 tmp117_write(u8 reg, u16 data)
{
    u8 dev = TMP117_DEV_ID;
    u8 ret = FALSE;
    u8 pdat[4];
    pdat[0] = reg;
    pdat[1] = (u8)(data>>8);
    pdat[1] = (u8)(data&0x00FF);
    
    ret = bsp_iic_writeBytes(DEV_IIC_TM117, dev, 3, pdat);

    return ret;
}

/*
 ** PRIVATE FUNCTION: amp_read_bytes()
 *
 */  
static u8 tmp117_read(u8 reg, u16 *value)
{
    u8 dev = TMP117_DEV_ID;
    u8 ret = FALSE;
    u8 pdat[4];
    pdat[0] = reg;
    
    ret = bsp_iic_writeBytes(DEV_IIC_TM117, dev, 1, pdat);
    ret = bsp_iic_readBytes(DEV_IIC_TM117, dev, 2, pdat);
    
    *value = ((u16)pdat[0]<<8) + pdat[1];
    
    return ret;
}

/*
 ** PRIVATE FUNCTION: TempDat_Convert()
 *
 */ 
static float TempDat_Convert(s16 raw, float temp_max, float temp_min)
{
    float tmp = 0.0;
    
    tmp = (float)raw;
    tmp = tmp * TMP117_CONVT_COEF;
    
    tmp  = (tmp > temp_max) ? temp_max : ((tmp < temp_min) ? temp_min : tmp);
    
    return tmp;
}

/*
 ** PUBLIC FUNCTION: SensorSampleOutput()
 *
 */ 
u8 Tmp117SampleProcess(u32 timebyms)
{
    static u16 timer;
    u16 cyclebyms = 1000;
    u8 ret = 1;
    
    timer++;
    if (timer*timebyms >= cyclebyms){
        timer = 0;
        
        /* 1. read */
        if (tmp117_read(adr_Temp_Result, &tmp117_entity.reg_Temp_Result)){
            tmp117_entity.temp_raw = tmp117_entity.reg_Temp_Result;
            tmp117_entity.tempDat_rt = TempDat_Convert(tmp117_entity.temp_raw, tmp117_entity.tempDat_max, tmp117_entity.tempDat_min);
        }
        
    }
    return ret;
}

/*
 ** PRIVATE FUNCTION: Tmp117Init()
 *
 */ 
void Tmp117Init(void)
{
    tmp117_entity.temp_raw_min = 0;
    tmp117_entity.temp_raw_max = 0;
    tmp117_entity.temp_raw = 0;
    tmp117_entity.tempDat_max = TMP117_TMP_MAX;
    tmp117_entity.tempDat_min = TMP117_TMP_MIN;
    tmp117_entity.tempDat_rt = 0;

    tmp117_read(adr_Temp_Result, &tmp117_entity.reg_Temp_Result);
    tmp117_read(adr_Configuration, &tmp117_entity.reg_Configuration);
    tmp117_read(adr_THigh_Limit, &tmp117_entity.reg_THigh_Limit);
    tmp117_read(adr_TLow_Limit, &tmp117_entity.reg_TLow_Limit);
    tmp117_read(adr_EEPROM_UL, &tmp117_entity.reg_EEPROM_UL);
    tmp117_read(adr_EEPROM1, &tmp117_entity.reg_EEPROM1);
    tmp117_read(adr_EEPROM2, &tmp117_entity.reg_EEPROM2);
    tmp117_read(adr_Temp_Offset, &tmp117_entity.reg_Temp_Offset);
    tmp117_read(adr_EEPROM3, &tmp117_entity.reg_EEPROM3);
    tmp117_read(adr_Device_ID, &tmp117_entity.reg_Device_ID);
}

/*
 ** PRIVATE FUNCTION: Tmp117Init()
 *
 */ 
s16 GetTMP117RtdTemperature(void)
{
    s16 tmp = (s16)((tmp117_entity.tempDat_rt + 0.05)*10);
    return tmp;
}

/*******************END OF FILE************************************************/