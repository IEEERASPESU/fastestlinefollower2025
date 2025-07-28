/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  * of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  // Set initial state for all motor and LED pins to LOW
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|BENABLE_Pin|AENABLE2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, AENEABLE_Pin|BENABLE2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 (On-board LED) */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins for Motor Driver Control */
  // All four motor control pins are now configured as outputs.
  GPIO_InitStruct.Pin = BENABLE_Pin|AENABLE2_Pin;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = AENEABLE_Pin|BENABLE2_Pin;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins for Sensor Array */
  // All sensor pins are configured as inputs with internal pull-up resistors.
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;

  GPIO_InitStruct.Pin = SENSOR9_Pin|SENSOR15_Pin|SENSOR16_Pin;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SENSOR10_Pin|SENSOR11_Pin|SENSOR12_Pin|SENSOR13_Pin;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SENSOR14_Pin|SENSOR1_Pin|SENSOR2_Pin|SENSOR3_Pin
                          |SENSOR4_Pin|SENSOR5_Pin|SENSOR6_Pin|SENSOR7_Pin
                          |SENSOR8_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins for DIP Switches */
  // All DIP switch pins are configured as inputs with internal pull-up resistors.
  // Note: The original file had a "DIP3_Pin" which is not in the new boilerplate.
  // Using pins available in the new project.
  GPIO_InitStruct.Pin = DIP1_Pin|DIP2_Pin|DIP4_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
