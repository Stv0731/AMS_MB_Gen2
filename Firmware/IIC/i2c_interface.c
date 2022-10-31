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

/******** Private definitions *************************************************/

/******** Private function declarations ***************************************/
static void bsp_iic_start(void);
static void bsp_iic_stop(void);
static void bsp_iic_SandAck(void);
static void bsp_iic_SandNack(void);
static u8 bsp_iic_Wait_Ack(void);
static u8 bsp_iic_read_byte(u8 ack);
static void bsp_iic_send_byte(u8 data);
static void config_sda_out(void);
static void config_scl_out(void);

/******** Variables ***********************************************************/

/******** Function Prototype **************************************************/

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
static void config_sda_in(void)
{
    /* User add related code */
    SDA_Pin_Config(1);
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
void config_sda_out(void)
{
    /* User add related code */
    SDA_Pin_Config(0);
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
void config_scl_out(void)
{
    /* User add related code */
    SCL_Pin_Config(0);
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
void bsp_iic_start(void)
{
    config_sda_out();
    IIC_SDA_Set(1);
    IIC_SCL_Set(1);
    delay_us(50);
    IIC_SDA_Set(0);
    delay_us(20);
    IIC_SCL_Set(0);
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
void bsp_iic_stop(void)
{
    config_sda_out();
    IIC_SCL_Set(0);
    IIC_SDA_Set(0);
    delay_us(20);
    IIC_SCL_Set(1);
    delay_us(15);
    IIC_SDA_Set(1);
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
void bsp_iic_SandAck(void)
{
    IIC_SCL_Set(0);
    config_sda_out();
    IIC_SDA_Set(0);
    delay_us(15);
    IIC_SCL_Set(1);
    delay_us(15);
    IIC_SCL_Set(0);
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
void bsp_iic_SandNack(void)
{
    IIC_SCL_Set(0);
    config_sda_out(); 	
    IIC_SDA_Set(1);
    delay_us(15);
    IIC_SCL_Set(1);
    delay_us(15);
    IIC_SCL_Set(0);
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
u8 bsp_iic_Wait_Ack(void)
{
    u8 ucErrTime=0;

    config_sda_in();
    IIC_SCL_Set(0);
    delay_us(40);
    IIC_SCL_Set(1);

    while(IIC_SDA_Read()){
        ucErrTime++;
        if(ucErrTime>150){
            bsp_iic_stop();	
            return 1;
        }
        delay_us(10); 
    }
    
    delay_us(40);
    IIC_SCL_Set(0);
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
u8 bsp_iic_read_byte(u8 ack)
{
    unsigned char i,receive=0;
    config_sda_in(); 

    for(i=0;i<8;i++){
        IIC_SCL_Set(0);
        delay_us(15);
        IIC_SCL_Set(1);
        receive<<=1;
        if(IIC_SDA_Read())
        receive++;   
        delay_us(15);	
    }
    if(!ack)
        bsp_iic_SandNack();   
    else
        bsp_iic_SandAck();  
    
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
void bsp_iic_send_byte(u8 data)
{
   u8 t;   
   config_sda_out();
   IIC_SCL_Set(0);  
    
	for(t=0;t<8;t++){              
        if(data&0x80)
          IIC_SDA_Set(1);
        else
          IIC_SDA_Set(0);
        data<<=1;
        delay_us(10);
        IIC_SCL_Set(1);
        delay_us(15);
        IIC_SCL_Set(0);
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
u8 bsp_iic_readBytes(u8 dev, u8 length, u8 *data)
{
    u8 count = 0;
    
    bsp_iic_start();
    bsp_iic_send_byte((dev<<1)+1);
    bsp_iic_Wait_Ack();
    delay_us(40); 
    for(count=0;count<length;count++)
    {  
        if(count!=(length-1)) 
        { 
            data[count] = bsp_iic_read_byte(1);
            delay_us(40); 
        }
        else 
        {
            data[count] = bsp_iic_read_byte(0);
        }
    }
    bsp_iic_stop();

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
u8 bsp_iic_writeBytes(u8 dev, u8 length, u8* data)
{
	u8 count = 0;

	bsp_iic_start();
	bsp_iic_send_byte(dev<<1);
	//bsp_iic_Wait_Ack();
    if (1 == bsp_iic_Wait_Ack())
    {
        delay_us(10000);                // Delay 10ms
        bsp_iic_start();                // Restart
        bsp_iic_send_byte(dev<<1);
        bsp_iic_Wait_Ack();
    }
    
	for(count=0;count<length;count++)
	{
	    delay_us(50);
		bsp_iic_send_byte(data[count]);
	    bsp_iic_Wait_Ack();
	}

	bsp_iic_stop();
    return 1;
}

/*
 ** PUBLIC FUNCTION: bsp_iic_Config()
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
void bsp_iic_Config(void)
{
    config_scl_out();
    config_sda_out();
    
    IIC_SCL_Set(1);
    IIC_SCL_Set(1);
}

/*******************END OF FILE************************************************/