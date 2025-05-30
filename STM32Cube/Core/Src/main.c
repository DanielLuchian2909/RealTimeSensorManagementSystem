/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "kernel.h"
#include "scheduler.h"
#include "EnvironmentalSensor.h"

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
typedef struct Bme280DataAlias
{
// Compensated pressure
UINT pressure;

// Compensated temperature
INT temperature;

// Compensated humidity
UINT humidity;
} Bme280DataAlias;

//Current flag used for syncrhronization (will be replaced by proper mutexes and semaphores later)
volatile BOOLE bSensorBusy = FALSE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* I/O Utilities */
INT __io_putchar(INT ch); //Transmit a character over UART

/* Thread Test Functions */
void ReadWriteSensorData(void* pvParameters_);
void TestThread2(void* pvParameters_);
void TestThread3(void* pvParameters_);

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  //Start the timer
  HAL_TIM_Base_Start(&htim1);

  //Intialize all kernel related information
  rtos_KernelInit();

  //Create an environmental sensor class and an alias to the class
  EnvSensorHandle* psSensorHandle = cInterfaceCreateEnvironmentalSensor();

  //Create the thread to collect and display sensor data
  rtos_CreateThread(ReadWriteSensorData, (void*)psSensorHandle);

  //Create a test thread to confirm scheduling
  rtos_CreateThread(TestThread2, NULL);

  //Start the kernel
  rtos_KernelStart();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
//-----------------------------------------------------------------------
INT //Printed character
__io_putchar( //Transmits a character ofer UART
		INT ch) //Character to print
{
	//Transmit characters
	if (HAL_UART_Transmit(&huart2, (UCHAR*)&ch, 1, HAL_MAX_DELAY) != HAL_OK)
		return -1; //If transmission fail, return error code
	return ch; //Otherwise reuturn transmitted character
}

//-----------------------------------------------------------------------
void
ReadWriteSensorData( //Function that runs reading and writing the sensor
		void* pvParameters_) //Thread function parameter
 {
	//Cast argument
	EnvSensorHandle* psInputs = *(EnvSensorHandle*)pvParameters_;

	//Initialize a pointer to sensor data
	Bme280DataAlias* psSensorData = NULL;

	//Thread execution code
	while (1)
	{
		//Check if the sensor is busy
		if (!bSensorBusy)
		{
            //Set the flag to indicate the sensor is in use
			bSensorBusy = TRUE;

            //Read the sensor data
            cInterfaceReadSensorData(psInputs, READ_MODE_SINGLE, ENVSENSOR_SELECT_ALL);

            //Get the sensor data
            psSensorData = cInterfaceGetSensorData(psInputs);

            // Print the sensor data
            if (psSensorData != NULL)
            {
                printf("Temperature: %d°C, Humidity: %d%%, Pressure: %d hPa\n",
                		psSensorData->temperature, psSensorData->humidity, psSensorData->pressure);
            }

            //Set sensor to available
            bSensorBusy = FALSE;
		}
	}
 }

//-----------------------------------------------------------------------
void
TestThread2( //Function that serial prints "Thread2 Running\n"
		void* pvParameters_)
 {

	//Cast argument
	UINT inputs = *(UINT*)pvParameters_;

	while (1)
	{
		printf("Thread2 Running\n");
		for(int i = 0; i < 20002; i++){} //make sure the max iterations are different
		HAL_Delay(500);
		rtos_Yield();
	}
 }

//-----------------------------------------------------------------------
void
TestThread3( //Function that serial prints "Thread3 Running\n"
		void* pvParameters_)
 {

	//Cast argument
	UINT inputs = *(UINT*)pvParameters_;

	while (1)
	{
		printf("Thread3 Running\n");
		for(int i = 0; i < 20002; i++){} //make sure the max iterations are different
		HAL_Delay(500);
		//rtos_Yield();
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
