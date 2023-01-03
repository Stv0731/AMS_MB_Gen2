/**\
 * Copyright (c) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "bmp3.h"
#include "bmp3_common.h"
#include "main.h"
#include "i2c_interface.h"
#include "ampComm.h"

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

/*! BMP3 shuttle board ID */
#define BMP3_SHUTTLE_ID 0xD3

/* Variable to store the device address */
static uint8_t dev_addr;
static uint8_t tmp_iic_buf[128];

/*!
 * I2C read function map to COINES platform
 */
BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t ret = 0;
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    //portENTER_CRITICAL();
    ret = bsp_iic_writeBytes(DEV_IIC_SENSOR, dev_addr, 1, &reg_addr);
    ret = bsp_iic_readBytes(DEV_IIC_SENSOR, dev_addr, len, reg_data);
    //portEXIT_CRITICAL();
    //return HAL_I2C_Mem_Read(&hi2c1, dev_addr << 0x01, reg_addr, 1, reg_data, (uint16_t)len, 100);
    ret = ret;
    return BMP3_INTF_RET_SUCCESS;
}

/*!
 * I2C write function map to COINES platform
 */
BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    uint8_t *ptr = &tmp_iic_buf[0];
    uint8_t i, ret = 0;
    ret = ret;
    *ptr++ = reg_addr;
    for (i=0; i < len; i++){
        *ptr++ = *reg_data++;
    }
    //portENTER_CRITICAL();
    ret = bsp_iic_writeBytes(DEV_IIC_SENSOR, dev_addr, (len+1), tmp_iic_buf);
    //portEXIT_CRITICAL();
    //return HAL_I2C_Mem_Write(&hi2c1, dev_addr << 0x01, reg_addr, 1, (uint8_t *)reg_data, (uint16_t)len, 100);
    return BMP3_INTF_RET_SUCCESS;
}

/*!
 * SPI read function map to COINES platform
 */
BMP3_INTF_RET_TYPE bmp3_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t rslt = 0;
    uint8_t reg_spi[1] = {reg_addr | 0x80};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, reg_spi, 1, 100);
    rslt = HAL_SPI_Receive(&hspi1, reg_data, (uint16_t)len, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    return rslt;
}

/*!
 * SPI write function map to COINES platform
 */
BMP3_INTF_RET_TYPE bmp3_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t reg_spi[1] = {reg_addr & 0x7f};
    uint8_t rslt = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX) ;
    HAL_SPI_Transmit(&hspi1, reg_spi, 1, 100);
    rslt = HAL_SPI_Transmit(&hspi1, (uint8_t *)reg_data, (uint16_t)len, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    return rslt;
}

/*!
 * Delay function map to COINES platform
 */
void bmp3_delay_us(uint32_t period, void *intf_ptr)
{
    //HAL_Delay(1);
    delay_us(500);
}

void bmp3_check_rslt(const char api_name[], int8_t rslt)
{
//    switch (rslt)
//    {
//    case BMP3_OK:
//
//        /* Do nothing */
//        break;
//    case BMP3_E_NULL_PTR:
//        printf("API [%s] Error [%d] : Null pointer\r\n", api_name, rslt);
//        break;
//    case BMP3_E_COMM_FAIL:
//        printf("API [%s] Error [%d] : Communication failure\r\n", api_name, rslt);
//        break;
//    case BMP3_E_INVALID_LEN:
//        printf("API [%s] Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
//        break;
//    case BMP3_E_DEV_NOT_FOUND:
//        printf("API [%s] Error [%d] : Device not found\r\n", api_name, rslt);
//        break;
//    case BMP3_E_CONFIGURATION_ERR:
//        printf("API [%s] Error [%d] : Configuration Error\r\n", api_name, rslt);
//        break;
//    case BMP3_W_SENSOR_NOT_ENABLED:
//        printf("API [%s] Error [%d] : Warning when Sensor not enabled\r\n", api_name, rslt);
//        break;
//    case BMP3_W_INVALID_FIFO_REQ_FRAME_CNT:
//        printf("API [%s] Error [%d] : Warning when Fifo watermark level is not in limit\r\n", api_name, rslt);
//        break;
//    default:
//        printf("API [%s] Error [%d] : Unknown error code\r\n", api_name, rslt);
//        break;
//    }
}

BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf)
{
    int8_t rslt = BMP3_OK;

    /* Bus configuration : I2C */
    if (intf == BMP3_I2C_INTF)
    {
        //printf("I2C Interface\n");
        dev_addr = BMP3_ADDR_I2C_SEC;
        bmp3->read = bmp3_i2c_read;
        bmp3->write = bmp3_i2c_write;
        bmp3->intf = BMP3_I2C_INTF;
    }
    /* Bus configuration : SPI */
    else if (intf == BMP3_SPI_INTF)
    {
        //printf("SPI Interface\n");
        // dev_addr = COINES_SHUTTLE_PIN_7;
        bmp3->read = bmp3_spi_read;
        bmp3->write = bmp3_spi_write;
        bmp3->intf = BMP3_SPI_INTF;
    }

    delay_us(5000);//HAL_Delay(100);
    bmp3->delay_us = bmp3_delay_us;
    bmp3->intf_ptr = &dev_addr;

    return rslt;
}

uint8_t bmp3_osrp_config_flg = 0xFF;
uint8_t bmp3_osrp_config(uint8_t osrp)
{
  uint8_t value;

  // 1C: OSR,  osr_p(bit2.0): 2  , osr_t(bit6.4):  1
      switch(osrp){
      case 6:   // x2
          value = 1;
          break;
      case 5:   // x4
          value = 2;
          break;
      case 4:   // x8
          value = 3;
          break;
      case 3:   // x16
          value = 4;
          break;
      case 2:   // x32
          value = 5;
          break;
      default:   // x1
          value = 0;
          break;
      }
  //value += 0x10;
  bmp3_osrp_config_flg = value;
  return 1;
}
uint8_t bmp_odr_config_flg = 0xFF;
uint8_t bmp_odr_config(float odr)
{
  uint8_t value;
  float tmp;
  tmp = 1000.0/odr;
      if (tmp < 5.0)        value = 0;
      else if (tmp < 10.0)  value = 1;
      else if (tmp < 20.0)  value = 2;
      else if (tmp < 40.0)  value = 3;
      else if (tmp < 80.0)  value = 4;
      else if (tmp < 160.0) value = 5;
      else if (tmp < 320.0) value = 6;
      else                  value = 7;
          
  bmp_odr_config_flg = value;

  return 1;
}
uint8_t bmp_iir_config_flg = 0xFF;
uint8_t bmp_iir_config(uint8_t coef)
{
  uint8_t value;
  // 1F: ODR: 3(coef_3)
  switch(coef){
    case 1:   value = 0;  break;
    case 2:   value = 1;  break;
    case 4:   value = 2;  break;
    case 8:   value = 3;  break;
    case 16:   value = 4;  break;
    case 32:   value = 5;  break;
    case 64:   value = 6;  break;
    default:   value = 0;  break;
  }
  bmp_iir_config_flg = value;
  
  return 1;
}

  uint8_t settings_sel;
  struct bmp3_dev dev;
  struct bmp3_data data = {0};
  struct bmp3_settings settings = {0};
  struct bmp3_status status = {{0}};
uint8_t bmp_config_action(void)
{
    //uint8_t reg, value;
    //uint8_t id = 0x77;
    int8_t rslt = 0;

    if (bmp3_osrp_config_flg != 0xFF){
        settings.odr_filter.press_os = bmp3_osrp_config_flg;
        rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
        bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

        bmp3_osrp_config_flg = 0xFF;
        return 1;
    }
//    if (bmp_odr_config_flg != 0xFF){
//        settings.odr_filter.odr = bmp_odr_config_flg;
//        rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
//        bmp3_check_rslt("bmp3_set_sensor_settings", rslt);
//         
//        bmp_odr_config_flg = 0xFF;
//        return 1;
//    }
    if (bmp_iir_config_flg != 0xFF){
        settings.odr_filter.iir_filter = bmp_iir_config_flg;
        rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
        bmp3_check_rslt("bmp3_set_sensor_settings", rslt);
        
        bmp_iir_config_flg = 0xFF;
        return 1;
    }
    return 0;
}

/* Tempalate */
//  uint8_t settings_sel;
//  struct bmp3_dev dev;
//  struct bmp3_data data = {0};
//  struct bmp3_settings settings = {0};
//  struct bmp3_status status = {{0}};
      
  uint8_t reg, value, id;
      extern uint8_t bmp3_start;
void bmp3_ss_init(void)
{
  int8_t rslt = 0;

  // I2C
  rslt = bmp3_interface_init(&dev, BMP3_I2C_INTF);
  bmp3_check_rslt("bmp3_interface_init", rslt);

  rslt = bmp3_init(&dev);
  bmp3_check_rslt("bmp3_init", rslt);

  settings.int_settings.drdy_en = BMP3_ENABLE;
  settings.press_en = BMP3_ENABLE;
  settings.temp_en = BMP3_ENABLE;

  settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
  settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
  settings.odr_filter.odr = BMP3_ODR_50_HZ;//BMP3_ODR_100_HZ;
  settings.odr_filter.iir_filter = BMP3_IIR_FILTER_DISABLE;

  settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR | BMP3_SEL_DRDY_EN;

  rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
  bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

  settings.op_mode = BMP3_MODE_NORMAL;
  rslt = bmp3_set_op_mode(&settings, &dev);
  bmp3_check_rslt("bmp3_set_op_mode", rslt);
    bmp3_start = 4;
}

uint8_t regs[200];
uint32_t bmp3xx_prs, bmp3xx_tmp, bmp3xx_period;
void BMP3XXSampleProcess(uint32_t timebyms)
{
    int8_t rslt = 0;
    uint8_t loop = 0;
    static uint16_t timer = 0;
    //static uint8_t switcher = 0;
    if (amp_entity.odr != 0)
        bmp3xx_period = (uint32_t)((1000.0 / amp_entity.odr) + 10.0);
    bmp3xx_period = (bmp3xx_period < 10) ? 10 : (bmp3xx_period > 1000 ? 1000 : bmp3xx_period);
    
    if (bmp_config_action()){
        timer = 0;
    }
    if (++timer * timebyms < bmp3xx_period){
        return;
    }

    //switcher = 0;
    rslt = bmp3_get_status(&status, &dev);
    bmp3_check_rslt("bmp3_get_status", rslt);

    /* Read temperature and pressure data iteratively based on data ready interrupt */
    if ((rslt == BMP3_OK) && (status.intr.drdy == BMP3_ENABLE))
    {
      /*
       * First parameter indicates the type of data to be read
       * BMP3_PRESS_TEMP : To read pressure and temperature data
       * BMP3_TEMP       : To read only temperature data
       * BMP3_PRESS      : To read only pressure data
       */
      rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
      bmp3_check_rslt("bmp3_get_sensor_data", rslt);
      amp_entity.pres_rt = bmp3xx_prs = (uint32_t)(data.pressure*10.0);
      amp_entity.temp_rt = bmp3xx_tmp = (uint32_t)(data.temperature*10.0);
      
      /* NOTE : Read status register again to clear data ready interrupt status */
      rslt = bmp3_get_status(&status, &dev);
      bmp3_check_rslt("bmp3_get_status", rslt);

      loop = loop + 1;
    }
}


