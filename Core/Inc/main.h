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
#include "stm32g4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define dip4_Pin GPIO_PIN_14
#define dip4_GPIO_Port GPIOC
#define AENABLE2_Pin GPIO_PIN_15
#define AENABLE2_GPIO_Port GPIOC
#define AC1_Pin GPIO_PIN_0
#define AC1_GPIO_Port GPIOA
#define AC2_Pin GPIO_PIN_1
#define AC2_GPIO_Port GPIOA
#define BC2_Pin GPIO_PIN_4
#define BC2_GPIO_Port GPIOA
#define AENEABLE_Pin GPIO_PIN_5
#define AENEABLE_GPIO_Port GPIOA
#define BC1_Pin GPIO_PIN_6
#define BC1_GPIO_Port GPIOA
#define BENABLE2_Pin GPIO_PIN_7
#define BENABLE2_GPIO_Port GPIOA
#define BENABLE_Pin GPIO_PIN_4
#define BENABLE_GPIO_Port GPIOC
#define SENSOR14_Pin GPIO_PIN_0
#define SENSOR14_GPIO_Port GPIOB
#define SENSOR1_Pin GPIO_PIN_1
#define SENSOR1_GPIO_Port GPIOB
#define SENSOR2_Pin GPIO_PIN_2
#define SENSOR2_GPIO_Port GPIOB
#define SENSOR3_Pin GPIO_PIN_10
#define SENSOR3_GPIO_Port GPIOB
#define SENSOR4_Pin GPIO_PIN_11
#define SENSOR4_GPIO_Port GPIOB
#define SENSOR5_Pin GPIO_PIN_12
#define SENSOR5_GPIO_Port GPIOB
#define SENSOR6_Pin GPIO_PIN_13
#define SENSOR6_GPIO_Port GPIOB
#define SENSOR7_Pin GPIO_PIN_14
#define SENSOR7_GPIO_Port GPIOB
#define SENSOR8_Pin GPIO_PIN_15
#define SENSOR8_GPIO_Port GPIOB
#define SENSOR9_Pin GPIO_PIN_6
#define SENSOR9_GPIO_Port GPIOC
#define SENSOR10_Pin GPIO_PIN_8
#define SENSOR10_GPIO_Port GPIOA
#define SENSOR11_Pin GPIO_PIN_9
#define SENSOR11_GPIO_Port GPIOA
#define SENSOR12_Pin GPIO_PIN_10
#define SENSOR12_GPIO_Port GPIOA
#define SENSOR13_Pin GPIO_PIN_11
#define SENSOR13_GPIO_Port GPIOA
#define BPWM_Pin GPIO_PIN_12
#define BPWM_GPIO_Port GPIOA
#define SENSOR15_Pin GPIO_PIN_10
#define SENSOR15_GPIO_Port GPIOC
#define SENSOR16_Pin GPIO_PIN_11
#define SENSOR16_GPIO_Port GPIOC
#define DIP1_Pin GPIO_PIN_4
#define DIP1_GPIO_Port GPIOB
#define DIP2_Pin GPIO_PIN_5
#define DIP2_GPIO_Port GPIOB
#define APWM_Pin GPIO_PIN_6
#define APWM_GPIO_Port GPIOB
#define DIP4_Pin GPIO_PIN_9
#define DIP4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
