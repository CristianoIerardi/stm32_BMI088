/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/*------ Communication Libraries -----------------------*/
//#undef USE_SERIAL
//#define USE_SERIAL
//#undef USE_API
//#define USE_API
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern char txBuff[128];

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define INT_ACC_Pin GPIO_PIN_2
#define INT_ACC_GPIO_Port GPIOC
#define INT_ACC_EXTI_IRQn EXTI2_IRQn
#define INT_GYR_Pin GPIO_PIN_3
#define INT_GYR_GPIO_Port GPIOC
#define INT_GYR_EXTI_IRQn EXTI3_IRQn
#define ACC_NCS_Pin GPIO_PIN_4
#define ACC_NCS_GPIO_Port GPIOA
#define GYR_NCS_Pin GPIO_PIN_4
#define GYR_NCS_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
