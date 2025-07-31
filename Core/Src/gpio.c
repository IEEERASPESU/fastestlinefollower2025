static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|BENEABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AENEABLE_Pin|BENEABLE2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AENEABLE2_GPIO_Port, AENEABLE2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 BENEABLE_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|BENEABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : AENEABLE_Pin BENEABLE2_Pin */
  GPIO_InitStruct.Pin = AENEABLE_Pin|BENEABLE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR14_Pin SENSOR1_Pin SENSOR2_Pin SENSOR3_Pin
                           SENSOR4_Pin SENSOR5_Pin SENSOR6_Pin SENSOR7_Pin
                           SENSOR8_Pin DIP1_Pin DIP2_Pin */
  GPIO_InitStruct.Pin = SENSOR14_Pin|SENSOR1_Pin|SENSOR2_Pin|SENSOR3_Pin
                          |SENSOR4_Pin|SENSOR5_Pin|SENSOR6_Pin|SENSOR7_Pin
                          |SENSOR8_Pin|DIP1_Pin|DIP2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR9_Pin SENSOR15_Pin SENSOR16_Pin */
  GPIO_InitStruct.Pin = SENSOR9_Pin|SENSOR15_Pin|SENSOR16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR10_Pin SENSOR11_Pin SENSOR12_Pin SENSOR13_Pin */
  GPIO_InitStruct.Pin = SENSOR10_Pin|SENSOR11_Pin|SENSOR12_Pin|SENSOR13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : AENEABLE2_Pin */
  GPIO_InitStruct.Pin = AENEABLE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AENEABLE2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}
