/*
********************************************************************************
*
* File : amp6100br.h
*
* Date : 2022/05/04
*
* Revision : 1.0.0
*
* Usage: The header file for the sensor AMP6100 configuration and operation interface.
*
*******************************************************************************/
#ifndef __AMP6100BR_H
#define __AMP6100BR_H

#include "i2c_interface.h"
#include "ampComm.h"

#define AMP6100BR_DEV_ID        0x78

#define AMP612X_osr_p_default         16       /* osr_p: default=16, range: 1,2,4,8,16,32,64,128    */
#define AMP612X_osr_t_default          4       /* osr_t: default=4,  range: 4,8                     */
#define AMP612X_IIR_filter_default     8       /* IIR filter: default=8, range: 1~32                */
#define AMP612X_odr_default            1       /* Output data rate: default=1Hz, range: 1~100Hz     */

#define AMP6120_TMP_MIN	      -400             /* -40.0 Celsius */
#define AMP6120_TMP_MAX	      1500             /* 150.0 Celsius */
#define AMP6127_TMP_MIN	      -400             /* -40.0 Celsius */
#define AMP6127_TMP_MAX	      1500             /* 150.0 Celsius */

#define AMP6120_PRS_MIN	     20000             /*  20,000 Pa    */
#define AMP6120_PRS_MAX	    120000             /* 120,000 Pa    */
#define AMP6127_PRS_MIN	        20             /*   20 kPa      */
#define AMP6127_PRS_MAX	      1020             /* 1020 kPa      */

#define AMP61XX_PRS_SCALE       10

/* Sensor operating status information */
typedef union{
    u8 all;
    struct{
        u8 bit0:1;
        u8 bit1:1;
        u8 nvmErr:1;
        u8 mode:1;
        u8 bit4:1;
        u8 busy:1;
        u8 adcPwr:1;
        u8 bit7:1;
    }bit;
}
amp_stus_t;

/* AEF format define */
typedef union{
    u16 all;
    struct{
        u16 gain_stage1:2;
        u16 gain_stage2:3;
        u16 gain_polarity:1;
        u16 clk_divider:2;
        u16 A2D_offset:3;
        u16 osr_p:3;
        u16 osr_t:2;
    }bit;
}
amp_afe_t;

/* NVM register define */
#define MAX_REG_NUMBER      32
typedef union{
    u16 array[MAX_REG_NUMBER];
}
amp_register_t;

#define MAX_COT_NUMBER      3
#define MAX_COP_NUMBER      8

typedef struct{
    amp_object_t*   parent;
    _SensorType_t   metype;
    u8              portID;
    amp_stus_t      stus;
    amp_afe_t       afe_default;
    amp_register_t* pRegister;
    u32             co_t_array[MAX_COT_NUMBER];
    u32             co_p_array[MAX_COP_NUMBER];
    
    u16             osr_p;
    u16             osr_t;
    
    s16             tempDat_max;    /* real temperature max data, unit: 0.1 cnetigrade */
    s16             tempDat_min;
    u32             presDat_max;    /* real pressure max data, unit: 1hPa */
    u32             presDat_min;
    
    s16             temp_rt;
    u32             pres_rt;
    
    u16             temp_raw;
    u32             pres_raw;
    
    u8              cmd_pres;
}
amp_61xx_t;
extern amp_61xx_t amp61xx_entity;

/** Global function Declarations */
u8 amp_write_cmd(u8 *pdat, u8 len);
u8 amp_read_stus(void);
u8 amp_read_bytes(u8 *dst, u8 len);
u8 amp_read_reg(u8 addr, u16 *reg);
u8 amp_read_PresDat(u32 *pres);
u16 amp_read_TempDat(u16 *temp);

void Amp_Control_Req(u8 cmd, u8 osr_p, u8 osr_t);

UINT8 AMP61XXSampleProcess(UINT32 timebyms);
void Amp61XXInit(void);

#endif /* End of amp6100br include */
/*******************END OF FILE******************************************************************************/