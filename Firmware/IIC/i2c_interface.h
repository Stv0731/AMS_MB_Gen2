/*
********************************************************************************
*
* File : i2c_interface.h
*
* Date : 2022/05/04
*
* Revision : 1.0.0
*
* Usage: IIC read and write interface
*
*******************************************************************************/
#ifndef __I2C_INTERFACE_H
#define __I2C_INTERFACE_H

#include "main.h"

/** unsigned types */
typedef unsigned char 			u8;
typedef unsigned short 			u16;
typedef unsigned int 			u32;
typedef unsigned long long 		u64;
typedef signed char 			s8;
typedef signed short 			s16;
typedef signed int 				s32;
typedef signed long long 		s64;

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

/** Device ID define */
typedef enum{
    DEV_IIC_SENSOR = 0,
    DEV_IIC_SENSOR2,
    DEV_IIC_TM117,
    DEV_IIC_DISPLAY,
    DEV_IIC_ADS1113,
    NUM_OF_DEV_IIC
}_IIC_DEVICE_ID;

typedef struct{
    _IIC_DEVICE_ID devID;
    
    GPIO_TypeDef  *SCL_Port;
    uint32_t       SCL_Pin;
    GPIO_TypeDef  *SDA_Port;
    uint32_t       SDA_Pin;
    
}_IIC_DEVICE;
extern _IIC_DEVICE IIC_Device_List[NUM_OF_DEV_IIC];

/** Global function Declarations */
void delay_us(u32 micros);
u8 bsp_iic_writeBytes(_IIC_DEVICE_ID devID, u8 dev, u8 length, u8* data);
u8 bsp_iic_readBytes(_IIC_DEVICE_ID devID, u8 dev, u8 length, u8 *data);
void IIC_Device_Init(void);

u8 GetDevicebyID(_IIC_DEVICE_ID id, _IIC_DEVICE *dev);
#endif /* End of i2c_interface include */
/*******************END OF FILE******************************************************************************/