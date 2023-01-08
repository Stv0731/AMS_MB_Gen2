/*
********************************************************************************
*
* File : tmp117.h
*
* Date : 2023/01/05
*
* Revision : 1.0.0
*
* Usage: The header file for the sensor tmp117 configuration and operation interface.
*
*******************************************************************************/
#ifndef __TMP117_H
#define __TMP117_H

#include "i2c_interface.h"

#define TMP117_DEV_ID        0x48


#define TMP117_TMP_MIN	       -50.0               /* -50.0 Celsius */
#define TMP117_TMP_MAX	       150.0               /* 150.0 Celsius */

#define TMP117_RAW_MIN	      0x8000                /* -256.0 Celsius */
#define TMP117_RAW_MAX	      0x7FFF                /*  256.0 Celsius */

#define TMP117_CONVT_COEF     0.0078125             /* 0.0078125 °C RESOLUTION */

/* Sensor operating status information */
#define adr_Temp_Result     0x00  //00h R 8000h Temp_Result Temperature result register Go
#define adr_Configuration   0x01  //01h R/W 0220h(1) Configuration Configuration register Go
#define adr_THigh_Limit     0x02  //02h R/W 6000h(1) THigh_Limit Temperature high limit register Go
#define adr_TLow_Limit      0x03  //03h R/W 8000h(1) TLow_Limit Temperature low limit register Go
#define adr_EEPROM_UL       0x04  //04h R/W 0000h EEPROM_UL EEPROM unlock register Go
#define adr_EEPROM1         0x05  //05h R/W xxxxh(1) EEPROM1 EEPROM1 register Go
#define adr_EEPROM2         0x06  //06h R/W xxxxh(1) EEPROM2 EEPROM2 register Go
#define adr_Temp_Offset     0x07  //07h R/W 0000h(1) Temp_Offset Temperature offset register Go
#define adr_EEPROM3         0x08  //08h R/W xxxxh(1) EEPROM3 EEPROM3 register Go
#define adr_Device_ID       0x0F  //0Fh R 0117h Device_ID Device ID register G

typedef struct{
    
    s16             temp_raw_min;
    s16             temp_raw_max;
    s16             temp_raw;
    float           tempDat_max;    /* real temperature max data, unit: 0.1 cnetigrade */
    float           tempDat_min;
    float           tempDat_rt;

    //ADDRESS TYPE RESET ACRONYM REGISTER NAME SECTION
    u16 reg_Temp_Result;    //00h R 8000h Temp_Result Temperature result register Go
    u16 reg_Configuration;  //01h R/W 0220h(1) Configuration Configuration register Go
    u16 reg_THigh_Limit;    //02h R/W 6000h(1) THigh_Limit Temperature high limit register Go
    u16 reg_TLow_Limit;     //03h R/W 8000h(1) TLow_Limit Temperature low limit register Go
    u16 reg_EEPROM_UL;      //04h R/W 0000h EEPROM_UL EEPROM unlock register Go
    u16 reg_EEPROM1;        //05h R/W xxxxh(1) EEPROM1 EEPROM1 register Go
    u16 reg_EEPROM2;        //06h R/W xxxxh(1) EEPROM2 EEPROM2 register Go
    u16 reg_Temp_Offset;    //07h R/W 0000h(1) Temp_Offset Temperature offset register Go
    u16 reg_EEPROM3;        //08h R/W xxxxh(1) EEPROM3 EEPROM3 register Go
    u16 reg_Device_ID;      //0Fh R 0117h Device_ID Device ID register G
}
tmp117_t;
extern tmp117_t tmp117_entity;

/** Global function Declarations */

u8 Tmp117SampleProcess(u32 timebyms);
void Tmp117Init(void);
s16 GetTMP117RtdTemperature(void);

#endif /* End of __TMP117_H include */
/*******************END OF FILE******************************************************************************/