/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define BUTTON_1_Pin GPIO_PIN_7
#define BUTTON_1_GPIO_Port GPIOA
#define BUTTON_1_EXTI_IRQn EXTI9_5_IRQn
#define STAT_LED_Pin GPIO_PIN_5
#define STAT_LED_GPIO_Port GPIOC
#define SHIFT_OE_Pin GPIO_PIN_0
#define SHIFT_OE_GPIO_Port GPIOB
#define SHIFT_DATA_Pin GPIO_PIN_1
#define SHIFT_DATA_GPIO_Port GPIOB
#define SHIFT_DATA_CLK_Pin GPIO_PIN_2
#define SHIFT_DATA_CLK_GPIO_Port GPIOB
#define SHIFT_STORE_CLK_Pin GPIO_PIN_10
#define SHIFT_STORE_CLK_GPIO_Port GPIOB
#define SHIFT_MCLR_Pin GPIO_PIN_11
#define SHIFT_MCLR_GPIO_Port GPIOB
#define BUTTON_5_Pin GPIO_PIN_10
#define BUTTON_5_GPIO_Port GPIOA
#define BUTTON_5_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_10_Pin GPIO_PIN_3
#define BUTTON_10_GPIO_Port GPIOB
#define BUTTON_10_EXTI_IRQn EXTI3_IRQn
#define BUTTON_9_Pin GPIO_PIN_4
#define BUTTON_9_GPIO_Port GPIOB
#define BUTTON_9_EXTI_IRQn EXTI4_IRQn
#define BUTTON_8_Pin GPIO_PIN_5
#define BUTTON_8_GPIO_Port GPIOB
#define BUTTON_8_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_7_Pin GPIO_PIN_8
#define BUTTON_7_GPIO_Port GPIOB
#define BUTTON_7_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_6_Pin GPIO_PIN_9
#define BUTTON_6_GPIO_Port GPIOB
#define BUTTON_6_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
