/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "MPU6050.h"
#include "mpu6050_my.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */
extern TaskHandle_t MpuTaskHandle;
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GREEN_Pin ORANGE_Pin RED_Pin BLUE_Pin */
  GPIO_InitStruct.Pin = GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 7, 7);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 2 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_3) {
    // Because i2c us blocking
    //MPU6050_IRQHandler(MPU6050_GetHandleTypeDef());

    // Переключаємо пріорітетт миттєво
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (MpuTaskHandle != NULL) {
      // Будимо задачу
      vTaskNotifyGiveFromISR(MpuTaskHandle, &xHigherPriorityTaskWoken);

      // Обробляємо вищий пріоритет
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}
/* USER CODE END 2 */
