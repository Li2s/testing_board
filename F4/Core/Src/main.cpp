/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
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
#include "stdio.h"
#include "string.h"
#include "PWMCapturer\PWMCapturer.h"
#include "Servo\Servo.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint16_t min_value_ms = 989;
const uint16_t mid_value_ms = 1500;
const uint16_t max_value_ms = 2013;
const uint8_t measurement_error = 4;

PWMCapturer aileron_servo_command = PWMCapturer(
		&htim3,
		1,
		min_value_ms,
		mid_value_ms,
		max_value_ms,
		measurement_error);

void IcHandlerTim3(TIM_HandleTypeDef *htim)
{
	switch ((uint8_t)htim->Channel)
	{
		case HAL_TIM_ACTIVE_CHANNEL_1:
			aileron_servo_command.calculatePulseWidth();
			break;
	}
}
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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_RegisterCallback(&htim3, HAL_TIM_IC_CAPTURE_CB_ID, IcHandlerTim3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  Servo switch_PWM_gen(htim3.Instance, 3), aileron_PWM_gen(htim3.Instance, 4), arm_PWM_gen(htim3.Instance, 2);
  char str[100] = "\0";
  char start_str[10] = "\0";
  uint32_t eps = 4; //delta between PWM measurements
  uint8_t check1_flag = 0,
		  check2_flag = 0,
		  counter = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_UART_Receive (&huart1, (uint8_t*) start_str, sizeof(start_str), 1000);

	  arm_PWM_gen.setPositionMicroSeconds(989);//set disARM mode
	  switch_PWM_gen.setPositionMicroSeconds(989);
	  aileron_PWM_gen.setPositionMicroSeconds(1500);

	  if (counter < 1 && start_str[0] == '1')
	  {
		  counter++;

		  //---------------------------------------------------------------------
		  //-------------test1 - direct mode aileron command check---------------
		  //---------------------------------------------------------------------
		  check1_flag = 0;
		  check2_flag = 0;

		  sprintf(str, "Test 1 - Direct mode aileron command check is in progress...\n");
		  HAL_UART_Transmit(&huart1, (uint8_t*) str, sizeof(str), 1000);
		  memset(str, '\0', sizeof(str));

		  switch_PWM_gen.setPositionMicroSeconds(989); // set the direct mode
		  HAL_Delay(1000);

		  aileron_PWM_gen.setPositionMicroSeconds(989); // set the stick fully left
		  HAL_Delay(1000);


		  if(abs((int)(aileron_servo_command.getPulseWidthDif() - 989)) < eps) // check the command is equal to the stick
			  check1_flag = 1;
		  else
			  check1_flag = 0;

		  sprintf(str, "Command from the stick = %d, command to the servo = %d\n", (int)989, (int)aileron_servo_command.getPulseWidthDif());
		  HAL_UART_Transmit(&huart1, (uint8_t*) str, sizeof(str), 1000);
		  memset(str, '\0', sizeof(str));
		  HAL_Delay(1000);

		  aileron_PWM_gen.setPositionMicroSeconds(2013); // set the stick fully right
		  HAL_Delay(1000);

		  if(abs((int)(aileron_servo_command.getPulseWidthDif() - 2013)) < eps) // check the command is equal to the stick
			  check2_flag = 1;
		  else
			  check2_flag = 0;

		  sprintf(str, "Command from the stick = %d, command to the servo = %d\n", (int)2013, (int)aileron_servo_command.getPulseWidthDif());
		  HAL_UART_Transmit(&huart1, (uint8_t*) str, sizeof(str), 1000);
		  memset(str, '\0', sizeof(str));

		  if (check1_flag*check2_flag == 1)
		  {
			  sprintf(str, "Test 1 has passed\n");
			  HAL_UART_Transmit(&huart1, (uint8_t*) str, sizeof(str), 1000);
			  memset(str, '\0', sizeof(str));
		  }
		  else
		  {
			  sprintf(str, "Test 1 has failed\n");
			  HAL_UART_Transmit(&huart1, (uint8_t*) str, sizeof(str), 1000);
			  memset(str, '\0', sizeof(str));
		  }

		  //---------------------------Reset-------------------------------------

		  aileron_PWM_gen.setPositionMicroSeconds(1500);
		  HAL_Delay(3000);


		  //---------------------------------------------------------------------
		  //------test2 - stab mode integral calc and limitation check ----------
		  //---------------------------------------------------------------------
		  check1_flag = 0;
		  check2_flag = 0;

		  sprintf(str, "Test 2 - Stab mode integral calc and limitation check is in progress...\n");
		  HAL_UART_Transmit(&huart1, (uint8_t*) str, sizeof(str), 1000);
		  memset(str, '\0', sizeof(str));

		  arm_PWM_gen.setPositionMicroSeconds(2013);//set ARM mode
		  HAL_Delay(1000);
		  switch_PWM_gen.setPositionMicroSeconds(1500); // set the stab mode
		  HAL_Delay(1000);

		  aileron_PWM_gen.setPositionMicroSeconds(989); // set the stick fully left
		  HAL_Delay(995); // 1 second

		  //omega_zad_x = (0.234375*rc_input[AIL2] - 351.5625) = -120 deg/s;
		  //output[AIL1] = (int)(1500+0.4*omega_x_PI_reg.getOutput()) = 996;

		  if(abs((int)(aileron_servo_command.getPulseWidthDif() - 1115)) < 10*eps) // check the command is equal to the stick
			  check1_flag = 1;
		  else
			  check1_flag = 0;


		  sprintf(str, "Omega X command from the stick = %d deg/s, command to the servo = %d\n", (int)(-120),aileron_servo_command.getPulseWidthDif());
		  HAL_UART_Transmit(&huart1, (uint8_t*) str, sizeof(str), 1000);
		  memset(str, '\0', sizeof(str));

		  HAL_Delay(3000); // 3 more seconds

		  if(abs((int)(aileron_servo_command.getPulseWidthDif() - 836)) < 10*eps) // check the integral is limited
			  check2_flag = 1;
		  else
			  check2_flag = 0;

		  sprintf(str, "Omega X command from the stick = %d deg/s, command to the servo = %d\n", (int)(-120),aileron_servo_command.getPulseWidthDif());
		  HAL_UART_Transmit(&huart1, (uint8_t*) str, sizeof(str), 1000);
		  memset(str, '\0', sizeof(str));


		  if (check1_flag*check2_flag == 1)
		  {
			  sprintf(str, "Test 2 has passed\n");
			  HAL_UART_Transmit(&huart1, (uint8_t*) str, sizeof(str), 1000);
			  memset(str, '\0', sizeof(str));
		  }
		  else
		  {
			  sprintf(str, "Test 2 has failed\n");
			  HAL_UART_Transmit(&huart1, (uint8_t*) str, sizeof(str), 1000);
			  memset(str, '\0', sizeof(str));
		  }

		  //---------------------------Reset-------------------------------------

		  aileron_PWM_gen.setPositionMicroSeconds(1500);
		  HAL_Delay(3000);


	  }
	  else
	  {
		  if(counter == 1)
		  {
			  HAL_UART_Transmit(&huart1, (uint8_t*)"Tests finished!\n", 100, 1000);
			  counter++;
			  switch_PWM_gen.setPositionMicroSeconds(989);
			  aileron_PWM_gen.setPositionMicroSeconds(1500);
		  }
	  }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 22000;
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
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
