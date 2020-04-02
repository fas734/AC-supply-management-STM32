/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author         : fas734
  * @date           : 29.03.2020
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 fas734.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by fas734 under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_SECONDS_GPIO_Port LD6_GPIO_Port
#define LED_SECONDS_Pin LD6_Pin
#define LED_MINUTES_GPIO_Port LD4_GPIO_Port
#define LED_MINUTES_Pin LD4_Pin
#define LED_FRIDGE_WORKING_GPIO_Port LD3_GPIO_Port
#define LED_FRIDGE_WORKING_Pin LD3_Pin
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint16_t seconds = 0;
volatile uint16_t minutes = 0;
volatile const uint16_t intervals_24_hour[] = {
  /* ATTENTION! total sum of all intervals must be 1440min (==24h*60min) */
  /* work, wait //    visual equal                 index_of_interval */
      15, 225,  //  +++ ------------------3h_45min------------------- 0
      15,  30,  //  +++ ------                                        1
      15,  30,  //  +++ ------                                        2
      15,  30,  //  +++ ------                                        3
      15,  30,  //  +++ ------                                        4
      15,  30,  //  +++ ------                                        5
      15,  30,  //  +++ ------                                        6
      15,  30,  //  +++ ------                                        7
      15,  30,  //  +++ ------                                        8
      15,  30,  //  +++ ------                                        9
      15,  30,  //  +++ ------                                       10
      15,  30,  //  +++ ------                                       11
      15,  30,  //  +++ ------                                       12
      15,  30,  //  +++ ------                                       13
      15,  30,  //  +++ ------                                       14
      15,  30,  //  +++ ------                                       15
      15,  30,  //  +++ ------                                       16
      15,  30,  //  +++ ------                                       17
      15,  30,  //  +++ ------                                       18
      15,  30,  //  +++ ------                                       19
      15,  30,  //  +++ ------                                       20
      15,  30,  //  +++ ------                                       21
      15,  30,  //  +++ ------                                       22
      15,  30,  //  +++ ------                                       23
      15,  30,  //  +++ ------                                       24
      15,  30,  //  +++ ------                                       25
      15,  30,  //  +++ ------                                       26
      15,  15   //  +++ ---                                          27
};
volatile const uint16_t intervals_primitive[] = {
  /* work, wait //    visual equal                 index_of_interval */
      15,  30   //  +++ ------                                        0
};
volatile const uint16_t * intervals_pointer = intervals_24_hour;
volatile uint16_t intervals_size = (sizeof(intervals_24_hour) / \
                                    sizeof(intervals_24_hour[0]));
volatile uint16_t index_of_interval = 0;
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // blink on start
  HAL_GPIO_WritePin(LED_FRIDGE_WORKING_GPIO_Port, LED_FRIDGE_WORKING_Pin, \
                    GPIO_PIN_SET);
  // Increased delay to make sure the motor stopped.
  HAL_Delay(5900);
  HAL_GPIO_WritePin(LED_FRIDGE_WORKING_GPIO_Port, LED_FRIDGE_WORKING_Pin, \
                    GPIO_PIN_RESET);
  HAL_Delay(1000);
  HAL_TIM_Base_Start_IT(&htim3);  // seconds timer
  HAL_GPIO_WritePin(LED_SECONDS_GPIO_Port, LED_SECONDS_Pin, GPIO_PIN_RESET);

  minutes = 0;
  #warning Set interval number.
  // Multiply 2 because each interval contains 'work' and 'wait' intervals \
      (it is two intervals inside one interval)
  index_of_interval = 27*2;
  // RESET turns ON a fridge (opens the OPTOCOUPLER) on start
  HAL_GPIO_WritePin(OPTOCOUPLER_GPIO_Port, OPTOCOUPLER_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_FRIDGE_WORKING_GPIO_Port, LED_FRIDGE_WORKING_Pin, \
                    GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // One minute has gone
    if(seconds >= 60)
    {
      seconds = 0;
      minutes++;
      HAL_GPIO_WritePin(LED_MINUTES_GPIO_Port, LED_MINUTES_Pin, GPIO_PIN_SET);
    }
    // cycle intervals BEGIN
    if(minutes >= intervals_pointer[index_of_interval])
    {
      minutes = 0;
      if(index_of_interval < (intervals_size - 1))
        index_of_interval++;
      else
        index_of_interval = 0;
      HAL_GPIO_TogglePin(LED_FRIDGE_WORKING_GPIO_Port, LED_FRIDGE_WORKING_Pin);
      HAL_GPIO_TogglePin(OPTOCOUPLER_GPIO_Port, OPTOCOUPLER_Pin);
    }
    // cycle intervals END
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin 
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin 
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OPTOCOUPLER_GPIO_Port, OPTOCOUPLER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin 
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin 
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin 
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin 
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT4_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT4_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OPTOCOUPLER_Pin */
  GPIO_InitStruct.Pin = OPTOCOUPLER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OPTOCOUPLER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MISOA7_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MISOA7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DM_Pin DP_Pin */
  GPIO_InitStruct.Pin = DM_Pin|DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF14_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_SCL_Pin I2C1_SDA_Pin */
  GPIO_InitStruct.Pin = I2C1_SCL_Pin|I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  
  #warning Press user button to change type of intervals.
  if (GPIO_Pin == GPIO_PIN_0)
  {
    // changes type of intervals
    if(intervals_pointer == intervals_24_hour)
    {
      intervals_pointer = intervals_primitive;
      intervals_size = (sizeof(intervals_primitive) / \
                        sizeof(intervals_primitive[0]));
    }
    else
    {
      intervals_pointer = intervals_24_hour;
      intervals_size = (sizeof(intervals_24_hour) / \
                        sizeof(intervals_24_hour[0]));
    }
    index_of_interval = 0;
    seconds = 0;
    minutes = 0;
    HAL_GPIO_WritePin(LED_FRIDGE_WORKING_GPIO_Port, LED_FRIDGE_WORKING_Pin, \
                    GPIO_PIN_SET);
    HAL_GPIO_WritePin(OPTOCOUPLER_GPIO_Port, OPTOCOUPLER_Pin, GPIO_PIN_RESET);
  }
  
}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  if (htim->Instance == TIM3)  // 1 second
  {
    if(HAL_GPIO_ReadPin(LED_MINUTES_GPIO_Port, LED_MINUTES_Pin) \
        == GPIO_PIN_SET)
    {
      // turn off LED_MINUTES after 1 second
      HAL_GPIO_WritePin(LED_MINUTES_GPIO_Port, LED_MINUTES_Pin, \
        GPIO_PIN_RESET);
    }
    HAL_GPIO_TogglePin(LED_SECONDS_GPIO_Port, LED_SECONDS_Pin);
    seconds++;
  }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT fas734 *****END OF FILE****/
