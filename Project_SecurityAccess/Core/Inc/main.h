/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MID_Pin GPIO_PIN_13
#define MID_GPIO_Port GPIOC
#define MID_EXTI_IRQn EXTI15_10_IRQn
#define WakeUp_Pin GPIO_PIN_0
#define WakeUp_GPIO_Port GPIOA
#define WakeUp_EXTI_IRQn EXTI0_IRQn
#define USER_Pin GPIO_PIN_1
#define USER_GPIO_Port GPIOA
#define USER_EXTI_IRQn EXTI1_IRQn
#define ADC_Pin GPIO_PIN_6
#define ADC_GPIO_Port GPIOA
#define LEFT_Pin GPIO_PIN_4
#define LEFT_GPIO_Port GPIOC
#define LEFT_EXTI_IRQn EXTI4_IRQn
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOB
#define RIGHT_Pin GPIO_PIN_7
#define RIGHT_GPIO_Port GPIOC
#define RIGHT_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
