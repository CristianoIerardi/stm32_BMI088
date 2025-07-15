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
#include "string.h"
#include "stdlib.h"
#include "mcp3564.h"

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
uint32_t SAMPLE_TIME_MS_TOGGLE = 1000;	// For blinking led (to debug the code with the function Toggle(__))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
BMI088 imu;
LPF_FILTER filt;	// Filter object of IMU sensor

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
static void MX_SPI2_Init(void);
static void MX_TIM9_Init(void);
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

uint32_t timestamp_TIM3 = 0;				// Number of times that TIM3 is called (for debug)
uint32_t timestamp_TIM2 = 0;				// Number of times that TIM2 is called (for debug)
uint32_t measureTick = 0;					// Tick corresponding to the timestamp of the current measurement

/*------------------------------------------------------------------------*/
/* Variables for algorithms */
Quaternion q = {1, 0, 0, 0}; 				// Variable for madgwick algorithm. {1,0,0,0} is the Initial state --> not moving

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

/*------------------------------------------------------------------------*/
/* UART rx communication */
/*#define RX_BUFFER_SIZE 32		// dimension of the buffer. the string from ESP32 must have a length
char rx_uart_buff[64];
*/
#define UART_RX_BUFFER_SIZE 64
uint8_t rx_uart_buff[UART_RX_BUFFER_SIZE];
uint8_t rx_byte;
uint16_t rx_index = 0;

/*------------------------------------------------------------------------*/
/* MCP3564R Sensor params */
float adc[4];			// Voltage value of the mesurement of different channels
uint8_t allDiffCh = 0;		// if 1 it means that we are ready for sending data

/*------------------------------------------------------------------------*/


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////// FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++ PRIORITIES SETTINGS +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void SetPriorities(void)
{
	 HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);

	  // DMA: max priority
	  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 2);
	  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 1);   // RX SPI2
	  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 3);   // TX SPI2

	  // USB CDC (OTG_FS) - high but under DMA
	  HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 4);

	  // EXTI (GPIO sensors)
	  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
	  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 1);
	  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 2);

	  // UART (commands and debug)
	  HAL_NVIC_SetPriority(USART1_IRQn, 1, 3);

	  // Timer - low priority
	  HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
	  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 1);  // If we use TIM2

	  // Enable
	  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
	  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	  HAL_NVIC_EnableIRQ(USART1_IRQn);
	  HAL_NVIC_EnableIRQ(TIM4_IRQn);
	  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++ DEBUG FUNCTIONS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++ CALLBACK FUNCTIONS +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/// UART CALLBACK FUNCTION
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rx_byte == '\n' || rx_byte == '\r')
        {
            rx_uart_buff[rx_index] = '\0';  // termina stringa
            HandleReceivedString((char*)rx_uart_buff);
            rx_index = 0;
        }
        else
        {
            if (rx_index < UART_RX_BUFFER_SIZE - 1)
            {
                rx_uart_buff[rx_index++] = rx_byte;
            }
            else
            {
                rx_index = 0;  // overflow protection
            }
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);  // restart interrupt
    }
}



/// DMA Start Reading
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{   // we have an interrupt
	if(GPIO_Pin == INT_ACC_Pin)				/// DMA2
	{
		// we check if the interrupt pin is the accelerometer one
		if (!imu.readingAcc)
			BMI088_ReadAccelerometerDMA(&imu);	// if yes read from the DMA memory
	}
	else if(GPIO_Pin == INT_GYR_Pin)	/// DMA2
	{
	// we check if the interrupt pin is the gyroscope one
	if (!imu.readingGyr)
		BMI088_ReadGyroscopeDMA(&imu);
	}
	else if (GPIO_Pin == MCP3564_IRQ_Pin) {	/// DMA1
		MCP3561_StartReadADCData_DMA(&hspi2);			// Start reading with DMA1
		//allDiffCh = MCP3561_ReadADCData(&hspi2, pkt.adc);	// It read the value from the sensor MCP3564R and it writes into the variable adc[4] the measurements
	}
}


/// DMA CALLBACK
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)		// It tells us that the transfer has been completed
{
	if(hspi->Instance == SPI1)	// SPI1 used for Acc and Gyro
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
	if (hspi->Instance == SPI2)	// SPI2 used for MCP3564R sensor
	{
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
		allDiffCh = MCP3561_ReadADCData_DMA(&hspi2, pkt.adc);	// It change the global variable adc[4] with the update value
//		pkt.adc[0] = adc[0];
//		pkt.adc[1] = adc[1];
//		pkt.adc[2] = adc[2];
//		pkt.adc[3] = adc[3];
	}
}


/// TIMERS CALLBACK
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

    // Send data with CDC_Transfer_FS if enabled!!!
    if(htim->Instance == TIM3)
	{
    	timestamp_TIM3++;	// how many times TIM3 is called (not used yet)

    	// Send every data using just one string and one TX
		static char txBuff[256];
		sprintf(txBuff, "A,%lu,%.4f,%.4f,%.4f\r\nI,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\nS,%lu,%.4f,%.4f,%.4f,%.4f\r\n",
					measureTick, pkt.ang[0], pkt.ang[1], pkt.ang[2],
					measureTick, pkt.gyr[0], pkt.gyr[1], pkt.gyr[2], pkt.acc[0], pkt.acc[1], pkt.acc[2],
					measureTick, pkt.adc[0], pkt.adc[1], pkt.adc[2], pkt.adc[3]);
				//measureTick, pkt.abs_acc); // I send the abs_acc instead the temperature just to plot it in the API graph
		CDC_Transmit_FS((uint8_t *) txBuff, strlen(txBuff));
	}

    if (htim->Instance == TIM4)
	{

    	/*------- SEND STRING --------------------------*/
		/* If you want to send the string (too many bytes...) */
    /*	static char uartBuff[256];
		sprintf(uartBuff, "A,%lu,%.4f,%.4f,%.4f\r\nI,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\nS,%lu,%.4f,%.4f,%.4f,%.4f\r\n",
					measureTick, pkt.ang[0], pkt.ang[1], pkt.ang[2],
					measureTick, pkt.gyr[0], pkt.gyr[1], pkt.gyr[2], pkt.acc[0], pkt.acc[1], pkt.acc[2],
					measureTick, pkt.adc[0], pkt.adc[1], pkt.adc[2], pkt.adc[3]);

		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)uartBuff, strlen(uartBuff));*/
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


	}
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
  MX_SPI2_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  SetPriorities();	// function to set priorities

  HAL_Delay(500);

  /* ----- BMI088 and MADGWICK SETUP ------------------------------------------*/
  BMI088_Init(&imu, &hspi1, GPIOA, GPIO_PIN_4, GPIOC, GPIO_PIN_4);
  SetQuaternionFromEuler(&q, 0, 0, 0);				// Angles on the starting position: roll=0, pitch=0, yaw=0
  Filter_Init(&filt, f_LP_gyr, f_LP_acc, f_HP_gyr, f_HP_acc, f_LP_angles, T_TIM2);

  HAL_Delay(500);

  HAL_TIM_Base_Start_IT(&htim2);     // Start timer: calculation of the algorithm
  Init_BMI088_Bias(&imu, 1000000);	 // the second passed variable is the number of iterations to find the offset
  //HAL_TIM_Base_Start_IT(&htim3);   // Start timer: send data with CDC_Transmit_FS serial interface !!!!!!!! --> Not needed


  /* ----- MCP3564R SETUP ----------------------------------------------------- */
  HAL_TIM_Base_Start(&htim9);
  HAL_TIM_OC_Start(&htim9, TIM_CHANNEL_1);
  HAL_Delay(20);

  MCP3561_Reset(&hspi2);
  HAL_Delay(20);
  //MCP3561_PrintRegisters(&hspi2);
  //printf("\n");

  MCP3561_Init(&hspi2);
  HAL_Delay(20);
  //MCP3561_PrintRegisters(&hspi2);
  //printf("\n");
  HAL_Delay(2000);


  /* ----- START ESP32 TRANSMISSION --------------------------------------------*/
  HAL_TIM_Base_Start_IT(&htim4);     // Start the UART transmission to ESP32
  HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_uart_buff, 1);

  /* -------------------------------------------------------------------------- */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//MCP3561_PrintRegisters(&hspi2);
	  Toggle(SAMPLE_TIME_MS_TOGGLE);
	if(allDiffCh)
	{
		printf("%.3f\t%.3f\t%.3f\t%.3f\n", pkt.adc[0], pkt.adc[1], pkt.adc[2], pkt.adc[3]);
		allDiffCh = 0;
	}
	//Debug_SPI_DMA();



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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 17-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
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
  HAL_GPIO_WritePin(GPIOB, SPI2_CS_Pin|GPIO_PIN_4, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : MCP3564_IRQ_Pin */
  GPIO_InitStruct.Pin = MCP3564_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MCP3564_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_CS_Pin PB4 */
  GPIO_InitStruct.Pin = SPI2_CS_Pin|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// --- Static buffer for printf output redirection ---
// Define a suitable buffer size. A larger buffer can be more efficient for
// long printf strings, but consumes more RAM. 256 bytes is a common choice.
#define PRINTF_BUFFER_SIZE 256
static char s_printf_buffer[PRINTF_BUFFER_SIZE];
static int s_printf_buffer_idx = 0; // Current index in the buffer

/**
 * @brief  Retargets the C library printf function to the USB CDC.
 * This function is called by the printf family of functions
 * to output a single character. Characters are buffered and
 * transmitted when a newline is encountered or the buffer is full.
 * @param  ch: The character to be output.
 * @retval The character output.
 */
PUTCHAR_PROTOTYPE {
    // Optional: Add carriage return before newline if only newline is received.
    // This ensures proper line ending (\r\n) for terminals expecting it
    // when printf only outputs '\n'.
    if (ch == '\n') {
        if (s_printf_buffer_idx == 0 || s_printf_buffer[s_printf_buffer_idx - 1] != '\r') {
            // Ensure there's space for '\r' before adding it
            if (s_printf_buffer_idx < PRINTF_BUFFER_SIZE) {
                s_printf_buffer[s_printf_buffer_idx++] = '\r';
            }
        }
    }

    // Store the current character in the buffer
    // Ensure there's space for the character before adding it
    if (s_printf_buffer_idx < PRINTF_BUFFER_SIZE) {
        s_printf_buffer[s_printf_buffer_idx++] = (char)ch;
    }

    // Check if the buffer is full or if a newline character was received.
    // If either condition is true, transmit the buffered data.
    if (s_printf_buffer_idx >= PRINTF_BUFFER_SIZE || ch == '\n') {
        // Transmit the buffered data via USB CDC
        // The CDC_Transmit_FS function will handle the actual USB transfer.
        CDC_Transmit_FS((uint8_t*)s_printf_buffer, s_printf_buffer_idx);

        // Reset the buffer index after transmission
        s_printf_buffer_idx = 0;
    }

    return ch; // Return the character that was put
}

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
