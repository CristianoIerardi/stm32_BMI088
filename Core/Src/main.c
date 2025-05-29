/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @autor			: CriIera
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
#include "config.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "BMI088.h"
#include <stdio.h>
//#include "EKF.h"
#include "ComputeOrientation.h"
#include "Filters.h"
#include "MadgwickAHRS.h"
#include <math.h>

//#ifdef USE_SERIAL
	//#include "Serial_Comm.h"
//#endif //USE_SERIAL
//#ifdef USE_API
	//#include "API_Comm.h"
//#endif //USE_API



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define SERIAL_OR_API  		0 	// 0 if we use API
								// 1 if we use SERIAL

//#define SAMPLE_TIME_MS_USB  	10
//#define SAMPLE_TIME_MS_TOGGLE  	500
uint8_t SAMPLE_TIME_MS_TOGGLE = 500;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
BMI088 imu;
LPF_FILTER filt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////// GLOBAL VARIABLES ////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*------------------------------------------------------------------------*/
/* Clock and period, frequency, params of timers */
const float f_CK = 84000000; 				// [Hz] clock frequency. See .ioc files
float T_TIM2 = 0;							// seconds. This is the call period of the timer TIM2
float F_TIM2 = 0;

uint32_t timestamp_TIM3 = 0;				// Number of times that TIM3 is called
uint32_t timestamp_TIM2 = 0;				// Number of times that TIM2 is called
uint32_t measureTick = 0;					// Tick corresponding to the timestamp of the current measurement

/*------------------------------------------------------------------------*/
/* Variables for algorithms */
Quaternion q = {1, 0, 0, 0}; 				// Initial state
/*------------------------------------------------------------------------*/
/* Shared variables and string to be sent format */
#define PACKET_HEADER 0xAABBCCDD
#define PACKET_FOOTER 0XEE8899FF


BinaryPacket pkt;

/*------------------------------------------------------------------------*/
/* Timers */
uint32_t timerUSB = 0;
uint32_t timerToggle = 0;

/*------------------------------------------------------------------------*/
/* Filter params LPF and HPF (Currently the HPF is not used and to correct in "Filters.h")*/
float f_LP_gyr = 5.0f; // LP freq cut frequency in Hz of gyro
float f_LP_acc = 5.0f; // LP freq cut frequency in Hz of acc
float f_LP_angles = 5.0f;  // LP freq cut frequency in Hz of angles values calculation

float f_HP_gyr = 0.0001f; // HP freq cut frequency in Hz of gyro
float f_HP_acc = 0.0001f; // HP freq cut frequency in Hz of acc


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////// FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// Function that toggles the led of the board to show if the device is working
void Toggle(uint32_t waitingTime)
{
	// Toggle to show if the code is running
	if ((HAL_GetTick() - timerToggle) >= waitingTime)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
		timerToggle = HAL_GetTick();
	}
	timerUSB = HAL_GetTick();
}

/// Function to show some SPI and DMA parameter
void Debug_SPI_DMA()
{

  	HAL_SPI_StateTypeDef spiState = imu.spiHandle->State;
  	HAL_DMA_StateTypeDef dmaRxState = imu.spiHandle->hdmarx->State;

  	char txBuff[32];
  	sprintf(txBuff, "%u\t%u\n\r", spiState, dmaRxState);
  	while(CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff)) == HAL_BUSY);

  	if (__HAL_DMA_GET_IT_SOURCE(imu.spiHandle->hdmarx, DMA_IT_TC) == RESET)
  	{
  		sprintf(txBuff, "#\n\r");
  		while(CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff)) == HAL_BUSY);
  	}
}


/// DMA Reading
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{   // we have an interrupt
	if(GPIO_Pin == INT_ACC_Pin)
	{
		// we check if the interrupt pin is the accelerometer one
		if (!imu.readingAcc)
			BMI088_ReadAccelerometerDMA(&imu);	// if yes read from the DMA memory
	}
	else if(GPIO_Pin == INT_GYR_Pin)
	{
		// we check if the interrupt pin is the gyroscope one
		if (!imu.readingGyr)
			BMI088_ReadGyroscopeDMA(&imu);
	}

}

/// DMA CALLBACK
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

/**
 * @brief Prints the contents of a BinaryPacket in hexadecimal format over USB CDC.
 *
 * This function takes a pointer to a BinaryPacket structure, converts its content
 * to hexadecimal representation, and sends it over USB (CDC_Transmit_FS). The output
 * is split into lines of 48 bytes for readability. Each byte is printed as two hex digits
 * followed by a space. A carriage return and newline ("\r\n") is inserted at the end of each line.
 *
 * @param[in] p Pointer to the BinaryPacket structure to be printed.
 */
void print_packet_hex(BinaryPacket* p) {
    uint8_t* bytePtr = (uint8_t*)p;       ///< Pointer to the raw bytes of the packet
    char buffer[248];                     ///< Temporary buffer to hold the formatted string
    int len = 0;                          ///< Current length of the buffer

    for (int i = 0; i < sizeof(BinaryPacket); i++) {
        // Format one byte as two hexadecimal characters followed by a space
        len += snprintf((char*)&buffer[len], sizeof(buffer) - len, "%02X ", bytePtr[i]);

        // Send a line every 48 bytes or at the end of the packet
        if ((i + 1) % 48 == 0 || i == sizeof(BinaryPacket) - 1) {
            buffer[len++] = '\r';
            buffer[len++] = '\n';

            // Transmit the formatted buffer over USB CDC
            CDC_Transmit_FS((uint8_t*)buffer, len);

            // Reset buffer length for the next line
            len = 0;

            // Optional small delay to prevent USB buffer overflow
            // HAL_Delay(1);
        }
    }
}



/// Callback of the timers
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Calculate angles with quaternions
    if(htim->Instance == TIM2)
    {
    	timestamp_TIM2++;	// how many times TIM2 is called (not used yet)
        // Code to execute at constant sample rate
        Take_IMU_Measurements(&imu, &pkt);

        /// Filtering Gyro and Acc measurements
        filt = LPF_GyrAcc_Update_All(&filt, pkt.gyr, pkt.acc);

		/// Algorithm application to find angles
        MadgwickAHRSupdateIMU(pkt.gyr[0], pkt.gyr[1], pkt.gyr[2], pkt.acc[0], pkt.acc[1], pkt.acc[2], F_TIM2);
        q.w = q0; q.x = q1; q.y = q2; q.z = q3;
        QuaternionToEuler(q, pkt.ang);

        /* LPF Filtering angles */
        //filt = LPF_Angles_Update_All(&filt, angles);

        /* module of the acceleration vector (not used right now) */
        //pkt.abs_acc = sqrt(pow(pkt.acc[0],2)+pow(pkt.acc[1],2) + pow(pkt.acc[2],2));

    }

    // Send data with CDC_Transfer_FS
    if(htim->Instance == TIM3)
	{
    	timestamp_TIM3++;	// how many times TIM3 is called (not used yet)

    	// Send every data using just one string and one TX
		static char txBuff[256];
		sprintf(txBuff, "A,%lu,%.4f,%.4f,%.4f\r\nI,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
				measureTick, pkt.ang[0], pkt.ang[1], pkt.ang[2],
				measureTick, pkt.gyr[0], pkt.gyr[1], pkt.gyr[2], pkt.acc[0], pkt.acc[1], pkt.acc[2]);
				//measureTick, pkt.abs_acc); // I send the abs_acc instead the temperature just to plot it in the API graph
		CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff));
	}

    if (htim->Instance == TIM4)
	{

    	/*------- SEND STRING --------------------------*/
		//static char uartBuff[256];
		/* If you want to send the string (too many bytes...) */
		/* sprintf(uartBuff, "A,%lu,%.4f,%.4f,%.4f\r\nI,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\nT,%lu,%.4f\r\n",
					measureTick, pkt.ang[0], pkt.ang[1], pkt.ang[2],
					measureTick, pkt.gyr[0], pkt.gyr[1], pkt.gyr[2], pkt.acc[0], pkt.acc[1], pkt.acc[2],
					measureTick, pkt.abs_acc);

		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)uartBuff, strlen(uartBuff));
		//CDC_Transmit_FS((uint8_t *) uartBuff, strlen(uartBuff));*/

    	/*------- SEND NUMBER --------------------------*/
    	pkt.header = PACKET_HEADER;
    	pkt.footer = PACKET_FOOTER;

    	/*// HEX data debug
    	pkt.timestamp = HAL_GetTick();		// 2846468521 --> A9A9A9A9
		pkt.ang[0] = 0;
		pkt.ang[1] = 0;
		pkt.ang[2] = 0;
		pkt.gyr[0] = 0;
		pkt.gyr[1] = 0;
		pkt.gyr[2] = 0;
		pkt.acc[0] = 0;  //-71.954;
		pkt.acc[1] = 0;  //-152.49;
		pkt.acc[2] = 0;  //-21.6;
		*/

    	//print_packet_hex(&pkt);		// Function to debug the sent HEX string
    	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&pkt, sizeof(pkt));

		Toggle(SAMPLE_TIME_MS_TOGGLE);	// Function that toggle the led
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
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /*.... Priorities management .................................*/
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 1);
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 1);
  HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /*............................................................*/

  HAL_Delay(1000);

  BMI088_Init(&imu, &hspi1, GPIOA, GPIO_PIN_4, GPIOC, GPIO_PIN_4);
  SetQuaternionFromEuler(&q, 0, 0, 0);				// Angles on the starting position: roll=0, pitch=0, yaw=0
  Filter_Init(&filt, f_LP_gyr, f_LP_acc, f_HP_gyr, f_HP_acc, f_LP_angles, T_TIM2);

  HAL_Delay(1000);

  /* ----- START TIMERS ------------------------------------------------------- */
  HAL_TIM_Base_Start_IT(&htim2);   // Start timer: calculation of the algorithm
  Init_BMI088_Bias(&imu, 1000000);
  //HAL_TIM_Base_Start_IT(&htim3);   // Start timer: send data with CDC_Transmit_FS serial interface !!!!!!!!!!!!!!!!!!!!!!!!!
  HAL_TIM_Base_Start_IT(&htim4);   // Start the UART transmission to ESP32
  /* -------------------------------------------------------------------------- */



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
	  //Debug_SPI_DMA();
	  Toggle(SAMPLE_TIME_MS_TOGGLE);



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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 42-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* These following 2 lines calculate the Frequency and the Period of the Timer TIM2*/
	T_TIM2 = 1.0f / (f_CK / (float)((htim2.Init.Period +1 ) * htim2.Init.Prescaler + 1));
	F_TIM2 = 1 / T_TIM2;
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 42-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 42-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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

	char txBuff[128];
	sprintf(txBuff, "SPI Error!");
	while(CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff)) == HAL_BUSY);

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
