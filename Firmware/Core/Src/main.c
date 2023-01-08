/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "usb_interface.h"
#include "ampProcess.h"
#include "i2c_interface.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Task Handle defines */
TaskHandle_t StartTaskHandle;
//TaskHandle_t pxStartTask;
TaskHandle_t pxADCTask;
TaskHandle_t pxUSBTask;
TaskHandle_t pxSensorTask;
TaskHandle_t pxSCITask;

uint8_t usbRxData[2000];
uint16_t usbRxLen;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void vStartTask(void* argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
<<<<<<< .mine
//  MX_I2C1_Init();
//  MX_I2C3_Init();
//  MX_SPI1_Init();
//  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
||||||| .r36
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
=======
//  MX_I2C3_Init();
//  MX_SPI1_Init();
//  MX_SPI2_Init();
//  MX_SPI3_Init();
//  MX_SPI4_Init();
  MX_USART2_UART_Init();
  //MX_I2C1_Init();
>>>>>>> .r44
  /* USER CODE BEGIN 2 */
  IIC_Device_Init();

  /* USER CODE END 2 */

  /* Init scheduler */
  //osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  //MX_FREERTOS_Init();

  /* Start scheduler */
  //osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
      /* Create first task */
    xTaskCreate(vStartTask,
                "StartTask",
                TASK_START_STK_SIZE,
                NULL,
                TASK_START_PRI,
                (TaskHandle_t*) (StartTaskHandle));
    /* Start scheduler */
    vTaskStartScheduler();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*
 ** PUBLIC FUNCTION: delay_us()
 *
 *  DESCRIPTION:
 *      delay micros us.
 *
 *  PARAMETERS:
 *      micros: delay value by 3us//1us
 *             
 *  RETURNS:
 *      None.
 */
uint8_t tmpdb = 0;
extern void IIC_SDA_Set(_IIC_DEVICE_ID devID, u8 x);
void vStartTask(void* argument)
{
    uint32_t millisec = 10;
    TickType_t ticks = pdMS_TO_TICKS(millisec);
    
    uint16_t i = 0;
    //uint8_t sci_send_pack[10] = {'1','2','3','4','5','6','7','8','9','10'};
    
    (void) argument;
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();
    
    uint32_t timestart = HAL_GetTick();
        
    /* ADC task init */
    //ADC_Task_Init();
    
    /* USB interface initialization */
    Usb_Interface_Init();
    
    /* SCI_Interface_Init */
    SCI_Interface_Init();
    
    /* Sensor process task create */
    AmpProcessInit();
    
    while(1)
    {
        if(++i == 100){
            LED_STUS_BLINK();
            //SCI_Send_MsgProduce(sci_send_pack, 6);
        }
        else if (i==200){
            i = 0;
            //LED_GREEN_OFF();
        }
        
//        if((HAL_GetTick() - timestart) > 1000){
//            HAL_Delay(1);
//            timestart = HAL_GetTick();
//            //CDC_Transmit_FS(usbRxData, 5);//
//        }
//        
//        if (usbRxLen){
//            if (usbRxData[0] == 0xAA && usbRxData[1] == 0xAB && usbRxData[2] == 0xAC && usbRxData[3] == 0xAD){
//                CDC_Transmit_FS(usbRxData, 5);
//            }
//            usbRxLen = 0;
//        }
        if (tmpdb == 10){
            //DEV_PIN_SET(IIC_Device_List[DEV_IIC_SENSOR].SDA_Port, IIC_Device_List[DEV_IIC_SENSOR].SDA_Pin, 0);
            _IIC_DEVICE *entity = NULL;
            entity = &IIC_Device_List[0];
            //IIC_SDA_Set(DEV_IIC_SENSOR, 1);
            SDA_PIN_SET(entity->SDA_Port, entity->SDA_Pin, 0);
        }
        else if(tmpdb == 11){
            //DEV_PIN_SET(IIC_Device_List[DEV_IIC_SENSOR].SDA_Port, IIC_Device_List[DEV_IIC_SENSOR].SDA_Pin, 1);
            _IIC_DEVICE *entity = NULL;
            entity = &IIC_Device_List[0];
            //IIC_SDA_Set(DEV_IIC_SENSOR, 1);
            SDA_PIN_SET(entity->SDA_Port, entity->SDA_Pin, 1);
        }
        if (tmpdb == 20){
            IIC_SDA_Set(DEV_IIC_SENSOR, 0);
            //SDA_PIN_SET(IIC_Device_List[DEV_IIC_SENSOR].SDA_Port, IIC_Device_List[DEV_IIC_SENSOR].SDA_Pin, 0);
        }
        else if(tmpdb == 21){
            IIC_SDA_Set(DEV_IIC_SENSOR, 1);
            //SDA_PIN_SET(IIC_Device_List[DEV_IIC_SENSOR].SDA_Port, IIC_Device_List[DEV_IIC_SENSOR].SDA_Pin, 1);
        }
        //vTaskSuspend(StartTaskHandle);
        vTaskDelay(ticks);
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
