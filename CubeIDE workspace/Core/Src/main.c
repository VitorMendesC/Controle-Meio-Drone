/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct r_data{
    float slope_rad_ms;
    float start_pos;
    float set_point;

    uint32_t t0, t1, t2, t3, t4, t5;
};
typedef struct r_data route_data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_RESET_CONDITION 0
#define MOTOR_IDLE_CONDITION 50
#define MOTOR_MA_CONDITION 150
#define MOTOR_HOVER_CONDITION 500
#define MOTOR_MAX_CONTROL 900

////////////////////////////////////////////////////////////////////////////////////////////   ADDED   ////
#define INTEGRAL_MAXIMUM_VALUE 5
#define MOTOR_MIN_CONTROL 60
#define TS 20E-3
#define MAX_SLOPE_RAD_MS 0.0001
#define START_ROUTE_DELAY_MS 3000
#define M 0.250
#define G 9.8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t flag_user = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == B1_Pin)
	{
		flag_user = 1;
	}
}

void blink_led(int repetitions)
{
	for(int i=0; i< repetitions; i++)
	{
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(20);
	}
}
void wait_user()
{
	while(!flag_user)
	{
		HAL_Delay(10);
	}
	flag_user = 0;
}

uint8_t dataready = 0;
uint32_t angle_in_bits = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Read & Update The ADC Result
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	angle_in_bits = HAL_ADC_GetValue(hadc);
	dataready=1;
	HAL_ADC_Start_IT(&hadc1);
}

uint32_t clip(int16_t action, int16_t min, int16_t max)
{
	uint32_t normalizedU = 0;

	if (action < min)
	{
		action = min;
	}
	if (action > max)
	{
		action = max;
	}
	normalizedU = (uint32_t)(action);
	return normalizedU;
}

void PWM_setCombinedValue(TIM_HandleTypeDef* htim, int16_t left_actionx10, int16_t right_actionx10)
{
	//PPM protocol,
	// 1 ms => 0%
	// 2 ms => 100%
	// ARR = 20000
	//PSC = 83
	//Clk = 84 Mhz
	uint32_t norm_U_dir = clip(right_actionx10, 0, 1000);
	uint32_t norm_U_esq = clip(left_actionx10, 0, 1000);
	norm_U_dir = 1000+norm_U_dir;
	norm_U_esq = 1000+norm_U_esq;
	htim->Instance->CCR1 = norm_U_esq;
	htim->Instance->CCR2 = norm_U_dir;
}

///////////////////////////////////////////////////////////////////////////////    ADDED FUNCTIONS    ////
float ADC_to_rad(uint32_t ADC_value){
	return 0.0005*ADC_value - 1.1874;
}

float trapezoid_area(float current_value, float anterior_value){
	return TS*((anterior_value+current_value)/2);
}

float f_to_pwm_rigth(float f){
    return -7.5039*pow(f,2) + 51.102*f + 7.8998;
}

float f_to_pwm_left(float f){
    return -6.7909*pow(f,2) + 48.729*f + 7.6053;
}

void set_route_struct(route_data *d, float start_pos, float set_point, uint32_t time_constant_rad_ms){

    float slope_rad_ms;

    if(set_point-start_pos >= 0.){
        slope_rad_ms = MAX_SLOPE_RAD_MS;
    }
    else{
        slope_rad_ms = -MAX_SLOPE_RAD_MS;
    }

    d->slope_rad_ms = slope_rad_ms;
    d->start_pos = start_pos;
    d->set_point = set_point;

    d->t0 = HAL_GetTick();
    d->t1 = (uint32_t) ((set_point-start_pos)/slope_rad_ms);
    d->t2 = d->t1 + time_constant_rad_ms;
    d->t3 = d->t2 + 2*d->t1;
    d->t4 = d->t3 + time_constant_rad_ms;
    d->t5 = d->t4 + d->t1;
    return;
}

float route_planner(route_data *d){
    uint32_t t = HAL_GetTick() - d->t0;
    float relative_set_point = 0;

    if(t >= 0 && t < d->t1){
        relative_set_point = d->start_pos + d->slope_rad_ms*t;
    }
    else if(t >= d->t1 && t <d->t2){
        relative_set_point = d->set_point;
    }
    else if(t >= d->t2 && t < d->t3){
        relative_set_point = d->set_point - d->slope_rad_ms*(t - d->t2);
    }
    else if (t >= d->t3 && t < d->t4){
        relative_set_point = 2*d->start_pos - d->set_point;
    }
    else if (t >= d->t4 && t < d->t5){
        relative_set_point = 2*d->start_pos - d->set_point + d->slope_rad_ms*(t - d->t4);
    }
    else{
        relative_set_point = d->start_pos;
    }
    
    return relative_set_point;
}

/////////////////////////////////////////////////////////////////////////   END OF ADDED FUNCTIONS    ////

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  blink_led(50);
  wait_user();
  HAL_UART_Transmit(&huart2, "Programa com controle!\r\n", 24, 10000);
  HAL_TIM_PWM_Init(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(2000);
  //Motor1 é esquerdo
  PWM_setCombinedValue(&htim3,0,0);
  HAL_Delay(5000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_Delay(100);
  HAL_ADC_Start_IT(&hadc1);
  /* DEFINA AQUI AS PARTES DO CONTROLADOR - COMEÇO 1*/

  float angle_rad = 0;
  float angle_rad_setpoint = 0;
  float angle_rad_error = 0;
  float anterior_angle_rad_error = 0;
  uint8_t pData[16];

  float Kp = 0.112731*7;
  float Ti = 16.862/2;
  float Td = 2.16241/30;

  float integral = 0;		// Do I need to limit the integral sum? by how much? (CAPPED)
  float derivate = 0;
  float u = 0;


  float pwm_l = 0;
  float pwm_r = 0;

  route_data route_d;
  float true_set_point = 0.5;

  int first_execution = 1;
  uint32_t loop_counter = 0;

  /* DEFINA AQUI AS PARTES DO CONTROLADOR - FIM 1 */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(dataready)
	{
		dataready=0;
		angle_rad = ADC_to_rad(angle_in_bits);

		/* DEFINA AQUI AS PARTES DO CONTROLADOR - COMEÇO 2*/
		angle_rad_error = angle_rad_setpoint - angle_rad;

		integral += trapezoid_area(angle_rad_error, anterior_angle_rad_error);	// Updates integral value
		derivate = (angle_rad_error - anterior_angle_rad_error)/TS;

		u = Kp * (angle_rad_error + 1/Ti * integral + Td * derivate);				// saida do controlador em NEWTON

		pwm_r = f_to_pwm_rigth(M*G/(4*cosf(angle_rad)) + u/2);
		pwm_l = f_to_pwm_left(M*G/(4*cosf(angle_rad)) - u/2);

		PWM_setCombinedValue(&htim3, clip((int16_t) (10*pwm_l), 50, 950)
				, clip((int16_t) (10*pwm_r), 50, 950));


////////////////////////////////////////////////////////////////////////////    DEBBUGING	////
//		sprintf(pData, "SP = %.4f \n\r", angle_rad_setpoint);
//		HAL_UART_Transmit(&huart2, pData, 16, 50);
//
//		sprintf(pData, "E[%%] = %.4f \n\r", pwm_l);
//		HAL_UART_Transmit(&huart2, pData, 16, 50);
//
//		sprintf(pData, "D[%%] = %.4f \n\r\n\r", pwm_r);
//		HAL_UART_Transmit(&huart2, pData, 16, 50);

		sprintf(pData, "%1.4f \n\r", angle_rad);
		HAL_UART_Transmit(&huart2, pData, 16, 50);

/////////////////////////////////////////////////////////////////////    END OF DEBUGING    ////

		anterior_angle_rad_error = angle_rad_error;

		loop_counter += 1;

		if(loop_counter*20 > START_ROUTE_DELAY_MS){
			if(first_execution){
				set_route_struct(&route_d, angle_rad, true_set_point, 2000);
				first_execution = 0;
			}
			angle_rad_setpoint = route_planner(&route_d);	// Relative set point
		}

		/* DEFINA AQUI AS PARTES DO CONTROLADOR - FIM 2 */
	}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
