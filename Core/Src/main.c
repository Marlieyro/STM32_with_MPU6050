/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

#include <math.h>

#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "kalman_filter.h"
#include "MPU6050.h"
#include "mpu6050_my.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
TIM_HandleTypeDef htim6;
TaskHandle_t MpuTaskHandle = 0;
TaskHandle_t LedPwmTaskHandle = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint32_t g_Test_One;
extern int32_t g_Test_Two;
float_t temp_roll_angle_val = 0;
float_t kalman_val = 0;
KalmanFilter kalman_filter = {0};
uint32_t dt = 0;
uint32_t prev_tick = 0;
uint32_t pwm_pulse = 0;
uint8_t channel = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
void vMpuTask(void *pvParameters);
void vLedPwmTask(void *pvParameters);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(TIM_getHandelTypeDef(), TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(TIM_getHandelTypeDef(), TIM_CHANNEL_3);
  //PU6050_Read_Accel_Data(MPU6050_GetHandleTypeDef(), MPU6050_GetAccelDataStruct());
  MPU6050_Init(MPU6050_GetHandleTypeDef());
  MPU6050_Accel_Calibration(MPU6050_GetHandleTypeDef());
  MPU6050_Interrupt_Conf(MPU6050_GetHandleTypeDef(), MPU6050_GetInterruptConfigStruct(),MPU6050_DATA_RDY_EN_MASK);
  MPU6050_MotionInterrupt_Config(MPU6050_GetHandleTypeDef());
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  xTaskCreate(vMpuTask, "MPU_Task", 256, NULL, 3, &MpuTaskHandle);
  xTaskCreate(vLedPwmTask, "LED_Task", 128, NULL, 1, &LedPwmTaskHandle);
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  long result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (result < out_min) result = out_min;
  if (result > out_max) result = out_max;
  return result;
}

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{

  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock = 0;
  uint32_t              uwPrescalerValue = 0;
  uint32_t              pFLatency;

  __HAL_RCC_TIM6_CLK_ENABLE();

  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  uwTimclock = SystemCoreClock;

  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);
  htim6.Instance = TIM6;
  htim6.Init.Period = 1000U - 1U; // 1 МГц / 1000 = 1 кГц (1 мілісекунда)
  htim6.Init.Prescaler = uwPrescalerValue;
  htim6.Init.ClockDivision = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;

  if(HAL_TIM_Base_Init(&htim6) == HAL_OK)
  {
    // Вмикаємо таймер у режимі переривань
    HAL_TIM_Base_Start_IT(&htim6);

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TickPriority, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);


    return HAL_OK;
  }

  return HAL_OK;
}

void vMpuTask(void *pvParameters) {

  while(1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    MPU6050_IRQHandler(MPU6050_GetHandleTypeDef());

    MPU6050_Read_Accel_Data(MPU6050_GetHandleTypeDef(), MPU6050_GetAccelDataStruct());
    temp_roll_angle_val = atan2f(MPU6050_GetAccelDataStruct()->acc_y, MPU6050_GetAccelDataStruct()->acc_z) * (180 / M_PI);
    // NO KALMAN FILTER!
    channel = (temp_roll_angle_val < 0) ? TIM_CHANNEL_1 : TIM_CHANNEL_3;
  }
}

void vLedPwmTask(void *pvParameters) {
  uint16_t pwm_duty = 0;

  while(1) {

    pwm_duty = map((long)fabs(temp_roll_angle_val), (long)0, (long)90, (long)0, (long)200);


    if (temp_roll_angle_val < 0) {
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_duty);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    } else {
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_duty);
    }
    /*__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 100);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 100);*/

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
