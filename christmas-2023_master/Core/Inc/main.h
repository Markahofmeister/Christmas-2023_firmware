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
#define GAIN_3DB_NOT_Pin GPIO_PIN_0
#define GAIN_3DB_NOT_GPIO_Port GPIOC
#define GAIN_6DB_NOT_Pin GPIO_PIN_1
#define GAIN_6DB_NOT_GPIO_Port GPIOC
#define GAIN_12DB_Pin GPIO_PIN_2
#define GAIN_12DB_GPIO_Port GPIOC
#define GAIN_15DB_Pin GPIO_PIN_3
#define GAIN_15DB_GPIO_Port GPIOC
#define I2S_AMP_SD_Pin GPIO_PIN_2
#define I2S_AMP_SD_GPIO_Port GPIOA
#define BUTTON_1_Pin GPIO_PIN_7
#define BUTTON_1_GPIO_Port GPIOA
#define BUTTON_1_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_2_Pin GPIO_PIN_4
#define BUTTON_2_GPIO_Port GPIOC
#define BUTTON_2_EXTI_IRQn EXTI4_IRQn
#define STAT_LED_Pin GPIO_PIN_5
#define STAT_LED_GPIO_Port GPIOC
#define SHIFT_OE_Pin GPIO_PIN_0
#define SHIFT_OE_GPIO_Port GPIOB
#define SHIFT_DATA_OUT_Pin GPIO_PIN_1
#define SHIFT_DATA_OUT_GPIO_Port GPIOB
#define SHIFT_DATA_CLK_Pin GPIO_PIN_2
#define SHIFT_DATA_CLK_GPIO_Port GPIOB
#define SHIFT_STORE_CLK_Pin GPIO_PIN_10
#define SHIFT_STORE_CLK_GPIO_Port GPIOB
#define SHIFT_MCLR_Pin GPIO_PIN_11
#define SHIFT_MCLR_GPIO_Port GPIOB
#define SDIO_CARD_DETECT_Pin GPIO_PIN_7
#define SDIO_CARD_DETECT_GPIO_Port GPIOC
#define BUTTON_3_Pin GPIO_PIN_8
#define BUTTON_3_GPIO_Port GPIOA
#define BUTTON_3_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_4_Pin GPIO_PIN_9
#define BUTTON_4_GPIO_Port GPIOA
#define BUTTON_4_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_5_Pin GPIO_PIN_10
#define BUTTON_5_GPIO_Port GPIOA
#define BUTTON_5_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_10_Pin GPIO_PIN_3
#define BUTTON_10_GPIO_Port GPIOB
#define BUTTON_10_EXTI_IRQn EXTI3_IRQn
#define BUTTON_9_IN_Pin GPIO_PIN_4
#define BUTTON_9_IN_GPIO_Port GPIOB
#define BUTTON_8_Pin GPIO_PIN_5
#define BUTTON_8_GPIO_Port GPIOB
#define BUTTON_8_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_7_IN_Pin GPIO_PIN_8
#define BUTTON_7_IN_GPIO_Port GPIOB
#define BUTTON_6_IN_Pin GPIO_PIN_9
#define BUTTON_6_IN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
