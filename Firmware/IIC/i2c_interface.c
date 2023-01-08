/*
********************************************************************************
*
* File : i2c_interface.c
*
* Date : 2022/05/04
*
* Revision : 1.0.0
*
* Usage: IIC read and write interface
*
*******************************************************************************/
/******** Includes ************************************************************/
#include "i2c_interface.h"
#include "gpio.h"

/******** Private definitions *************************************************/

/******** Private function declarations ***************************************/
static void bsp_iic_start(_IIC_DEVICE_ID id);
static void bsp_iic_stop(_IIC_DEVICE_ID id);
static void bsp_iic_SandAck(_IIC_DEVICE_ID id);
static void bsp_iic_SandNack(_IIC_DEVICE_ID id);
static u8 bsp_iic_Wait_Ack(_IIC_DEVICE_ID id);
static u8 bsp_iic_read_byte(_IIC_DEVICE_ID id, u8 ack);
static void bsp_iic_send_byte(_IIC_DEVICE_ID id, u8 data);
static void config_sda_out(_IIC_DEVICE_ID id);
static void config_scl_out(_IIC_DEVICE_ID id);

/******** Variables ***********************************************************/
_IIC_DEVICE IIC_Device_List[NUM_OF_DEV_IIC] =
{   // ID               SCL_Port                SCL_Pin             SDA_Port                SDA_pin
    {DEV_IIC_SENSOR,    I2C_SEN_SCL_GPIO_Port,  I2C_SEN_SCL_Pin,    I2C_SEN_SDA_GPIO_Port,  I2C_SEN_SDA_Pin},
    {DEV_IIC_SENSOR2,   I2C_SEN2_SCL_GPIO_Port, I2C_SEN2_SCL_Pin,   I2C_SEN2_SDA_GPIO_Port, I2C_SEN2_SDA_Pin},
    {DEV_IIC_TM117,     TMP117_SCL_GPIO_Port,   TMP117_SCL_Pin,     TMP117_SDA_GPIO_Port,   TMP117_SDA_Pin},
    {DEV_IIC_DISPLAY,   LCD_SCL_GPIO_Port,      LCD_SCL_Pin,        LCD_SDA_GPIO_Port,      LCD_SDA_Pin},
    {DEV_IIC_ADS1113,   ADS_SCL_GPIO_Port,      ADS_SCL_Pin,        ADS_SDA_GPIO_Port,      ADS_SDA_Pin}
};

/******** Function Prototype **************************************************/

u8 GetDevicebyID(_IIC_DEVICE_ID id, _IIC_DEVICE *dev)
{
    u8 i = 0;
    for (i=0; i < NUM_OF_DEV_IIC; i++){
        if (IIC_Device_List[i].devID == id){
            dev = &IIC_Device_List[i];
            return 1;
        }
    }
    return 0;
}

/** SCL&SDA hardware operation  */
void IIC_SCL_Set(_IIC_DEVICE_ID devID, u8 x)
{
    _IIC_DEVICE *entity = &IIC_Device_List[devID];
    
//    if (!GetDevicebyID(devID, &entity)){
//        return;
//    }
    
    SCL_PIN_SET(entity->SCL_Port, entity->SCL_Pin, x);
}
void IIC_SDA_Set(_IIC_DEVICE_ID devID, u8 x)
{
    _IIC_DEVICE *entity = &IIC_Device_List[devID];
    
//    if (!GetDevicebyID(devID, &entity)){
//        return;
//    }
    
    SDA_PIN_SET(entity->SDA_Port, entity->SDA_Pin, x);
}
u8 IIC_SDA_Read(_IIC_DEVICE_ID devID)
{
    _IIC_DEVICE *entity = &IIC_Device_List[devID];
    
//    if (!GetDevicebyID(devID, &entity)){
//        return;
//    }
    
    return SDA_PIN_GET(entity->SDA_Port, entity->SDA_Pin);
};

/*
 ** PUBLIC FUNCTION: delay_us()
 *
 *  DESCRIPTION:
 *      delay micros us.
 *
 *  PARAMETERS:
 *      micros: delay value by 1us
 *             
 *  RETURNS:
 *      None.
 */  
void delay_us(u32 micros)
{  
    volatile u32 tm=micros *10;//<< 4;
    while(tm--){;}
    tm++;
}

/*
 ** PRIVATE FUNCTION: config_sda_in()
 *
 *  DESCRIPTION:
 *      config SDA pin for input.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
static void config_sda_in(_IIC_DEVICE_ID devID)
{   
    _IIC_DEVICE *entity = &IIC_Device_List[devID];
    
//    if (!GetDevicebyID(devID, &entity)){
//        return;
//    }
    
    SDA_Pin_Config(entity->SDA_Port, entity->SDA_Pin, 1);
}

/*
 ** PRIVATE FUNCTION: config_sda_out()
 *
 *  DESCRIPTION:
 *      config SDA pin for output.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
void config_sda_out(_IIC_DEVICE_ID devID)
{
    _IIC_DEVICE *entity = &IIC_Device_List[devID];
    
//    if (!GetDevicebyID(devID, &entity)){
//        return;
//    }
    
    SDA_Pin_Config(entity->SDA_Port, entity->SDA_Pin, 0);
}

/*
 ** PRIVATE FUNCTION: config_scl_out()
 *
 *  DESCRIPTION:
 *      config scl pin for output.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
void config_scl_out(_IIC_DEVICE_ID devID)
{
    _IIC_DEVICE *entity = &IIC_Device_List[devID];
    
//    if (!GetDevicebyID(devID, &entity)){
//        return;
//    }
    
    SCL_Pin_Config(entity->SDA_Port, entity->SDA_Pin, 0);
}

/*
 ** PRIVATE FUNCTION: bsp_iic_start()
 *
 *  DESCRIPTION:
 *      generate a i2c start or restart.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
void bsp_iic_start(_IIC_DEVICE_ID devID)
{
    config_sda_out(devID);
    IIC_SDA_Set(devID, 1);
    IIC_SCL_Set(devID, 1);
    delay_us(50);
    IIC_SDA_Set(devID, 0);
    delay_us(20);
    IIC_SCL_Set(devID, 0);
}

/*
 ** PRIVATE FUNCTION: bsp_iic_stop()
 *
 *  DESCRIPTION:
 *      generate a i2c stop.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
void bsp_iic_stop(_IIC_DEVICE_ID devID)
{
    config_sda_out(devID);
    IIC_SCL_Set(devID, 0);
    IIC_SDA_Set(devID, 0);
    delay_us(20);
    IIC_SCL_Set(devID, 1);
    delay_us(15);
    IIC_SDA_Set(devID, 1);
    delay_us(20);	
}

/*
 ** PRIVATE FUNCTION: bsp_iic_SandAck()
 *
 *  DESCRIPTION:
 *      generate a i2c ack to slave.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
void bsp_iic_SandAck(_IIC_DEVICE_ID devID)
{
    IIC_SCL_Set(devID, 0);
    config_sda_out(devID);
    IIC_SDA_Set(devID, 0);
    delay_us(15);
    IIC_SCL_Set(devID, 1);
    delay_us(15);
    IIC_SCL_Set(devID, 0);
}

/*
 ** PRIVATE FUNCTION: bsp_iic_SandNack()
 *
 *  DESCRIPTION:
 *      generate a i2c noack to slave.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
void bsp_iic_SandNack(_IIC_DEVICE_ID devID)
{
    IIC_SCL_Set(devID, 0);
    config_sda_out(devID); 	
    IIC_SDA_Set(devID, 1);
    delay_us(15);
    IIC_SCL_Set(devID, 1);
    delay_us(15);
    IIC_SCL_Set(devID, 0);
}

/*
 ** PRIVATE FUNCTION: bsp_iic_Wait_Ack()
 *
 *  DESCRIPTION:
 *      wait a i2c ack from slave.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
u8 bsp_iic_Wait_Ack(_IIC_DEVICE_ID devID)
{
    u8 ucErrTime=0;

    config_sda_in(devID);
    IIC_SCL_Set(devID, 0);
    delay_us(40);
    IIC_SCL_Set(devID, 1);

    while(IIC_SDA_Read(devID)){
        ucErrTime++;
        if(ucErrTime>150){
            bsp_iic_stop(devID);	
            return 1;
        }
        delay_us(10); 
    }
    
    delay_us(40);
    IIC_SCL_Set(devID, 0);
    return 0;  
}

/*
 ** PRIVATE FUNCTION: bsp_iic_read_byte()
 *
 *  DESCRIPTION:
 *      read a byte from i2c slave.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
u8 bsp_iic_read_byte(_IIC_DEVICE_ID devID, u8 ack)
{
    unsigned char i,receive=0;
    config_sda_in(devID); 

    for(i=0;i<8;i++){
        IIC_SCL_Set(devID, 0);
        delay_us(15);
        IIC_SCL_Set(devID, 1);
        receive<<=1;
        if(IIC_SDA_Read(devID))
        receive++;   
        delay_us(15);	
    }
    if(!ack)
        bsp_iic_SandNack(devID);   
    else
        bsp_iic_SandAck(devID);  
    
    return receive;
}

/*
 ** PRIVATE FUNCTION: bsp_iic_send_byte()
 *
 *  DESCRIPTION:
 *      write a byte to i2c slave.
 *
 *  PARAMETERS:
 *      data: data to write
 *             
 *  RETURNS:
 *      None.
 */
void bsp_iic_send_byte(_IIC_DEVICE_ID devID, u8 data)
{
   u8 t;   
   config_sda_out(devID);
   IIC_SCL_Set(devID, 0);  
    
	for(t=0;t<8;t++){              
        if(data&0x80)
          IIC_SDA_Set(devID, 1);
        else
          IIC_SDA_Set(devID, 0);
        data<<=1;
        delay_us(10);
        IIC_SCL_Set(devID, 1);
        delay_us(15);
        IIC_SCL_Set(devID, 0);
        delay_us(5);
    }
}

/*
 ** PUBLIC FUNCTION: bsp_iic_readBytes()
 *
 *  DESCRIPTION:
 *      Reads the length value of the specified register for the specified device.
 *
 *  PARAMETERS:
 *      dev: Device address, reg: slave memmory address, 
 *      length: the request data length, *data: pointer to target data buffer.
 *             
 *  RETURNS:
 *      the read data.
 */
u8 bsp_iic_readBytes(_IIC_DEVICE_ID devID, u8 dev, u8 length, u8 *data)
{
    u8 count = 0;
    
    bsp_iic_start(devID);
    bsp_iic_send_byte(devID, (dev<<1)+1);
    bsp_iic_Wait_Ack(devID);
    delay_us(40); 
    for(count=0;count<length;count++)
    {  
        if(count!=(length-1)) 
        { 
            data[count] = bsp_iic_read_byte(devID, 1);
            delay_us(40); 
        }
        else 
        {
            data[count] = bsp_iic_read_byte(devID, 0);
        }
    }
    bsp_iic_stop(devID);

    return count;
}

/*
 ** PUBLIC FUNCTION: bsp_iic_writeBytes()
 *
 *  DESCRIPTION:
 *      Write the length value to the specified register for the specified device.
 *
 *  PARAMETERS:
 *      dev: Device address, reg: slave memmory address, 
 *      length: the request data length, *data: pointer to data buffer.
 *             
 *  RETURNS:
 *      ture/false.
 */
u8 bsp_iic_writeBytes(_IIC_DEVICE_ID devID, u8 dev, u8 length, u8* data)
{
	u8 count = 0;

	bsp_iic_start(devID);
	bsp_iic_send_byte(devID, dev<<1);
	//bsp_iic_Wait_Ack();
    if (1 == bsp_iic_Wait_Ack(devID))
    {
        delay_us(10000);                // Delay 10ms
        bsp_iic_start(devID);                // Restart
        bsp_iic_send_byte(devID, dev<<1);
        bsp_iic_Wait_Ack(devID);
    }
    
	for(count=0;count<length;count++)
	{
	    delay_us(50);
		bsp_iic_send_byte(devID, data[count]);
	    bsp_iic_Wait_Ack(devID);
	}

	bsp_iic_stop(devID);
    return 1;
}

/*
 ** PUBLIC FUNCTION: IIC_Device_Init()
 *
 *  DESCRIPTION:
 *      config the I2C pin.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
void IIC_Device_Init(void)
{
    u8 devID;
    for (devID = 0; devID < NUM_OF_DEV_IIC; devID++){
        config_scl_out((_IIC_DEVICE_ID)devID);
        config_sda_out((_IIC_DEVICE_ID)devID);
        
        IIC_SCL_Set((_IIC_DEVICE_ID)devID, 1);
        IIC_SCL_Set((_IIC_DEVICE_ID)devID, 1);
    }
}

/*******************END OF FILE************************************************/