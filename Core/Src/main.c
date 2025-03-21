/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "BMI088.h"
#include "Fusion.h"
#include <stdbool.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NUM_SAMPLES_CALI	400 	// Num of samples to compute the calibration
#define SAMPLE_TIME_MS_USB  5		// sampling period in the while loop

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */
BMI088 imu;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#define SAMPLE_TIME_MS_USB  2		// SAMPLE_TIME_MS_USB/1000 s is the period
//#define SAMPLE_PERIOD (0.001f) 		// replace this with actual sample period
//float real_Ts_USB = 0.001f;	// mseconds
//uint32_t real_Fs_USB = 1;		// mseconds

char logBuf[128];
float gyr[3] = {0,0,0};
float acc[3] = {0,0,0};

////////////////////////////////////////////////////////////////////////////////////////////
/*NOTE: Register addres are written in the datasheet and in the .h file*/
////////////////////////////////////////////////////////////////////////////////////////////
// Register BMI_ACC_CONF 		0x40:  LPF and ODR, page 23 datasheet --> See section 4.4.1 for details
//#define VAL_BMI_ACC_BWP		0x0A		// bit [7:4]	--> 0b1001xxxx
//#define VAL_BMI_ACC_ODR		0X07		// bit [3:0]	--> 0bxxxx1000
#define VAL_BMI_ACC_CONF		0x47	// bit [7:0]    --> 0b10011000 ==  is the mask of the two above

// Register BMI_ACC_RANGE 		0x41:  Range accelerometer, +-3g to +- 24g
#define VAL_BMI_ACC_RANGE		0x01	// 0x01 is +-6g

// Register BMI_ACC_PWR_CONF 	0x7C:  Active or suspend mode
#define VAL_BMI_ACC_PWC_CONF	0x00	// 0X00 is for the active mode

// Register BMI_ACC_PWR_CTRL 	0x7D:  Accelerometer ON or OFF
#define VAL_BMI_ACC_PWC_CTRL	0x04	// 0X04 is for accelerometer ON

// Register BMI_GYR_RANGE 		0x0F:  Range gyroscope, +-125°/s to +-2000°/s
#define VAL_BMI_GYR_RANGE		0x02	// 0x02 is +-500°/s

// Register BMI_GYR_BANDWIDTH  	0x10:  Bandwidth gyroscope. See table at page 29 of the datasheet
#define VAL_BMI_GYR_BANDWIDTH  	0x07	// 0x03 is for ODR = 400[Hz], filter bandwidth = 47[Hz]

// Register BMI_GYR_LPM1 		0x11:  Gyroscope into normal, suspend, deep suspend mode
//#define VAL_BMI_GYR_LPM1		0x00	// 0X00 is for normal mode

//Register BMI_GYR_SELF_TEST    0x3C: Register for the self test (GYRO_SELF_TEST)
//#define VAL_BMI_GYR_SELF_TEST	0x??	//these are 4 flag to set separately

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
void Init_AccGyro_Reg(BMI088 *imu)
{
	BMI088_WriteAccRegister(imu, BMI_ACC_CONF, VAL_BMI_ACC_CONF);
	BMI088_WriteAccRegister(imu, BMI_ACC_RANGE, VAL_BMI_ACC_RANGE);
	BMI088_WriteAccRegister(imu, BMI_ACC_PWR_CONF, VAL_BMI_ACC_PWC_CONF);
	BMI088_WriteGyrRegister(imu, BMI_GYR_RANGE, VAL_BMI_ACC_PWC_CTRL);
	BMI088_WriteGyrRegister(imu, BMI_GYR_RANGE, VAL_BMI_GYR_RANGE);
	BMI088_WriteGyrRegister(imu, BMI_GYR_BANDWIDTH, VAL_BMI_GYR_BANDWIDTH);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{   // we have an interrupt
	if(GPIO_Pin == INT_ACC_Pin)
	{
		// we check if the interrupt pin is the accelerometer one
		BMI088_ReadAccelerometerDMA(&imu);	// if yes read from the DMA memory
	}
	else if(GPIO_Pin == INT_GYR_Pin)
	{
		// we check if the interrupt pin is the gyroscope one
		BMI088_ReadGyroscopeDMA(&imu);
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)		// It tells us that the transfer has been completed
{
	if(hspi->Instance == SPI1)		// Check if it is the correct SPI (we want SPI1)
	{
		if (imu.readingAcc)
		{
			BMI088_ReadAccelerometerDMA_Complete(&imu);
		}

		if (imu.readingGyr)
		{
			BMI088_ReadGyroscopeDMA_Complete(&imu);
		}
	}
}

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /*--------------------------------------------*/
	  // Sample rate
	  float SAMPLE_PERIOD = 1 / (1000 * SAMPLE_TIME_MS_USB);
	  BMI088_Init(&imu, &hspi1, GPIOA, GPIO_PIN_4, GPIOC, GPIO_PIN_4);

	  HAL_Delay(500);

	  /* This is for the offset calculation of the sensor */
	  //BMI088_InitCalibration(&imu, imu.offset_gyr, imu.offset_acc, FALSE, NUM_SAMPLES_CALI);
	  /*sprintf(logBuf, "\tgX_off=%.3f,\tgY_off=%.3f,\tgZ_off=%.3f\r\n", imu.offset_gyr[0], imu.offset_gyr[1], imu.offset_gyr[2]);
	  while (CDC_Transmit_FS((uint8_t *)logBuf, strlen(logBuf)) == USBD_BUSY) {}*/

	  // Fusion algorithms & object definition
	  FusionAhrs ahrs;
	  FusionAhrsInitialise(&ahrs);
  /*--------------------------------------------*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Timers:
  uint32_t timerUSB = 0;

  // Registers setup
  //Init_AccGyro_Reg(&imu);


  while (1)
  {

  /* Log data via USB */
	  if( (HAL_GetTick() - timerUSB) >= SAMPLE_TIME_MS_USB)
	  {
		  /*gyr[0] = imu.gyr_rps[0]; //- imu.offset_gyr[0];
		  gyr[1] = imu.gyr_rps[1]; //- imu.offset_gyr[2];
		  gyr[2] = imu.gyr_rps[2]; //- imu.offset_gyr[2];

		  acc[0] = imu.acc_mps2[0]; //- imu.offset_acc[0];
		  acc[1] = imu.acc_mps2[1]; //- imu.offset_acc[2];
		  acc[2] = imu.acc_mps2[2]; //- imu.offset_acc[2];*/

					  /*sprintf(logBuf, "aX=%.3f,\taY=%.3f,\taZ=%.3f,\tgX=%.3f,\tgY=%.3f,\tgZ=%.3f\r\n",
							  acc[0], acc[1], acc[2], gyr[0],  gyr[1],  gyr[2]);
					  while (CDC_Transmit_FS((uint8_t *)logBuf, strlen(logBuf)) == USBD_BUSY) {}

					  sprintf(logBuf, "________,\t________,\t________,\t__=%.3f,\t__=%.3f,\t__=%.3f\r\n",
							  (gyr[0] - imu.offset_gyr[0]),  (gyr[1] - imu.offset_gyr[1]),  (gyr[2] - imu.offset_gyr[2]) );
							  while (CDC_Transmit_FS((uint8_t *)logBuf, strlen(logBuf)) == USBD_BUSY) {}*/

		  // Passing measured data to structs gyroscope and accelerometer. This is done to compute the angles
		  FusionVector gyroscope = { {imu.gyr_rps[0], imu.gyr_rps[1], imu.gyr_rps[2]} };
		  FusionVector accelerometer = { {imu.acc_mps2[0], imu.acc_mps2[1], imu.acc_mps2[2]} };

		  FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_TIME_MS_USB);
		  FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

					  sprintf(logBuf, "Roll=%.3f\t Pitch=%.3f\t Yaw=%.3f\n\r", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
					  while (CDC_Transmit_FS((uint8_t *)logBuf, strlen(logBuf)) == USBD_BUSY) {}

		  timerUSB = HAL_GetTick();
	  }





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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACC_NCS_GPIO_Port, ACC_NCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GYR_NCS_GPIO_Port, GYR_NCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : INT_ACC_Pin INT_GYR_Pin */
  GPIO_InitStruct.Pin = INT_ACC_Pin|INT_GYR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ACC_NCS_Pin */
  GPIO_InitStruct.Pin = ACC_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACC_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GYR_NCS_Pin */
  GPIO_InitStruct.Pin = GYR_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GYR_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
