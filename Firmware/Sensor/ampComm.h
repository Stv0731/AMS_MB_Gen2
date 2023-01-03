/*
********************************************************************************
*
* File : ampComm.h
*
* Date : 2022/05/04
*
* Revision : 1.0.0
*
* Usage: The header file for the sensor AMP series common configuration and operation interface.
*
*******************************************************************************/
#ifndef __AMPCOMM_H
#define __AMPCOMM_H

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

/** unsigned types */
typedef unsigned char 			UINT8;
typedef unsigned short 			UINT16;
typedef unsigned int 			UINT32;
typedef unsigned long long 		UINT64;
typedef signed char 			INT8;
typedef signed short 			INT16;
typedef signed int 				INT32;
typedef signed long long 		INT64;
typedef float                   F32;
typedef double                  F64;

typedef enum{
    TYPE_NONE = 0,
    AMP6120 = 1,
    AMP6127 = 2,
    AMP8220 = 3,
    BMP3XX  = 100,
    NBROFAMP
}_SensorType_t;

typedef enum{
    CH_NONE = 0,
    I2C_CH1 = 1,
    I2C_CH2,
    SPI_CH1 = 11,
    SPI_CH2,
    ANALOG1 = 21,
    ANALOG2,
    ANALOG3,
    ANALOG4
}_SampleChannel_t;

typedef struct{
    _SensorType_t       SensorType;     /* Sensor type:  */
    _SampleChannel_t    SampleChannel;  /* Sample channel */
    F32                 odr;            /* Output Data Rate, unit:1Hz, range: 1~1000 */
    UINT16              iirCoef;        /* IIR filter cofficient, 1~64 */
    UINT16              offset;         /* the output data offset */
    UINT8               Started;        /* Startup the sample, 0=stoped, 1=started */

    INT16               temp_rt;
    UINT32              pres_rt;
    UINT16              temp_raw;
    UINT32              pres_raw;
}
amp_object_t;
extern amp_object_t amp_entity;

UINT32 iir_prs_filter(UINT32 raw, UINT8 iir);
void Amp_ADC_Start(uint8_t en);
void Amp_IIR_Enable(uint8_t en);
void Amp_ODR_Set(uint16_t freq);
void Amp_SensorType_Set(uint16_t type);
void Amp_SampleChannel_Set(uint16_t ch);

#endif /* End of ampComm include */
/*******************END OF FILE******************************************************************************/