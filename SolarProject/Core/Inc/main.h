/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define LDR_TOP_Pin GPIO_PIN_0
#define LDR_TOP_GPIO_Port GPIOC
#define LDR_BOTTOM_Pin GPIO_PIN_1
#define LDR_BOTTOM_GPIO_Port GPIOC
#define LDR_LEFT_Pin GPIO_PIN_2
#define LDR_LEFT_GPIO_Port GPIOC
#define LDR_RIGHT_Pin GPIO_PIN_3
#define LDR_RIGHT_GPIO_Port GPIOC
#define STEP_HORIZONTAL_Pin GPIO_PIN_0
#define STEP_HORIZONTAL_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define DIR_VERTICAL_Pin GPIO_PIN_1
#define DIR_VERTICAL_GPIO_Port GPIOB
#define DIR_HORIZONTAL_Pin GPIO_PIN_10
#define DIR_HORIZONTAL_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_13
#define LD2_GPIO_Port GPIOB
#define LIMIT_VERTICAL_Pin GPIO_PIN_7
#define LIMIT_VERTICAL_GPIO_Port GPIOC
#define LIMIT_VERTICAL_EXTI_IRQn EXTI9_5_IRQn
#define LIMIT_HORIZONTAL_90DEG_Pin GPIO_PIN_8
#define LIMIT_HORIZONTAL_90DEG_GPIO_Port GPIOC
#define LIMIT_HORIZONTAL_90DEG_EXTI_IRQn EXTI9_5_IRQn
#define LIMIT_HORIZONTAL_30DEG_Pin GPIO_PIN_9
#define LIMIT_HORIZONTAL_30DEG_GPIO_Port GPIOC
#define LIMIT_HORIZONTAL_30DEG_EXTI_IRQn EXTI9_5_IRQn
#define HORIZONTAL_ROUTES_COUNTER_Pin GPIO_PIN_12
#define HORIZONTAL_ROUTES_COUNTER_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define RESET_HORIZONTAL_Pin GPIO_PIN_4
#define RESET_HORIZONTAL_GPIO_Port GPIOB
#define RESET_VERTICAL_Pin GPIO_PIN_5
#define RESET_VERTICAL_GPIO_Port GPIOB
#define STEP_VERTICAL_Pin GPIO_PIN_8
#define STEP_VERTICAL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
