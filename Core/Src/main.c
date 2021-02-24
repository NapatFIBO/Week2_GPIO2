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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint16_t ButtonMatrixState = 0;
uint32_t ButtonMatrixTimeStamp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void ButtonMatrixUpdate(); //Scan and Update data of Button Matrix

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
  /* USER CODE BEGIN 2 */
  //--------------------------------------------------------------------------------
enum _StateNumber
{
	StateNumber_0 = 0b1000000000000,
	StateNumber_1 = 0b100000000,
	StateNumber_2 = 0b1000000000,
	StateNumber_3 = 0b10000000000,
	StateNumber_4 = 0b10000,
	StateNumber_5 = 0b100000,
	StateNumber_6 = 0b1000000,
	StateNumber_7 = 0b1,
	StateNumber_8 = 0b10,
	StateNumber_9 = 0b100,
	StateNumber_OK = 0b1000000000000000,
	StateNumber_Clear = 0b1000,
	StateNumber_NoInput = 0b0
};
enum _StateDisplay
{
	StateDisplay_Start = 0,
	StateDisplay_SN1 = 10,
	StateDisplay_SN2 = 20,
	StateDisplay_SN3 = 30,
	StateDisplay_SN4 = 40,
	StateDisplay_SN5 = 50,
	StateDisplay_SN6 = 60,
	StateDisplay_SN7 = 70,
	StateDisplay_SN8 = 80,
	StateDisplay_SN9 = 90,
	StateDisplay_SN10 = 100,
	StateDisplay_SN11 = 110,
	StateDisplay_Error = 120,
	StateDisplay_OK = 130,
	StateDisplay_LED2 = 140
};

uint16_t STATE_Display = 0;
uint16_t DataBase = 0;
uint16_t ButtonState[2];
  //--------------------------------------------------------------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //--------------------------------------------------------------------------------
  while (1)
  {
	  ButtonMatrixUpdate();
	  ButtonState[0] = ButtonMatrixState;
	  switch(STATE_Display)
	  {
	  	case StateDisplay_Start:
	  		DataBase = 0b0;
	  		STATE_Display = StateDisplay_SN1;
	  		break;
	  	case StateDisplay_SN1:
	  		if(ButtonMatrixState == StateNumber_NoInput)
	  		{
	  			STATE_Display = StateDisplay_SN1;
	  		}
	  		else if(ButtonMatrixState == StateNumber_6)
	  		{
	  			STATE_Display = StateDisplay_SN2;
	  		}
	  		else if(ButtonMatrixState == StateNumber_Clear)
	  		{
	  			STATE_Display = StateDisplay_Start;
	  		}
	  		else if(ButtonMatrixState == StateNumber_OK)
	  		{
	  			STATE_Display = StateDisplay_SN1;
	  		}
	  		else
	  		{
	  			STATE_Display = StateDisplay_Error;
	  		}
	  		break;
	  	case StateDisplay_SN2:
	  		if(ButtonMatrixState == StateNumber_NoInput)
	  		{
	  			STATE_Display = StateDisplay_SN2;
	  		}
	  		else if(ButtonMatrixState == StateNumber_2)
	  		{
	  			STATE_Display = StateDisplay_SN3;
	  		}
	  		else if(ButtonMatrixState == StateNumber_Clear)
	  		{
	  			STATE_Display = StateDisplay_Start;
	  		}
	  		else if(ButtonMatrixState == StateNumber_OK)
	  		{
	  			STATE_Display = StateDisplay_SN2;
	  		}
	  		else
	  		{
	  			STATE_Display = StateDisplay_Error;
	  		}
	  		break;
	  	case StateDisplay_SN3:
	  		if(ButtonMatrixState == StateNumber_NoInput)
	  		{
	  			STATE_Display = StateDisplay_SN3;
	  		}
	  		else if(ButtonMatrixState == StateNumber_3)
	  		{
	  			STATE_Display = StateDisplay_SN4;
	  		}
	  		else if(ButtonMatrixState == StateNumber_Clear)
	  		{
	  			STATE_Display = StateDisplay_Start;
	  		}
	  		else if(ButtonMatrixState == StateNumber_OK)
	  		{
	  			STATE_Display = StateDisplay_SN3;
	  		}
	  		else
	  		{
	  			STATE_Display = StateDisplay_Error;
	  		}
	  		break;
	  	case StateDisplay_SN4:
	  		if(ButtonMatrixState == StateNumber_NoInput)
	  		{
	  			STATE_Display = StateDisplay_SN4;
	  		}
	  		else if(ButtonMatrixState == StateNumber_4)
	  		{
	  			STATE_Display = StateDisplay_SN5;
	  		}
	  		else if(ButtonMatrixState == StateNumber_Clear)
	  		{
	  			STATE_Display = StateDisplay_Start;
	  		}
	  		else if(ButtonMatrixState == StateNumber_OK)
	  		{
	  			STATE_Display = StateDisplay_SN4;
	  		}
	  		else
	  		{
	  			STATE_Display = StateDisplay_Error;
	  		}
	  		break;
	  	case StateDisplay_SN5:
	  		if(ButtonMatrixState == StateNumber_NoInput)
	  		{
	  			STATE_Display = StateDisplay_SN5;
	  		}
	  		else if(ButtonMatrixState == StateNumber_0)
	  		{
	  			STATE_Display = StateDisplay_SN6;
	  		}
	  		else if(ButtonMatrixState == StateNumber_Clear)
	  		{
	  			STATE_Display = StateDisplay_Start;
	  		}
	  		else if(ButtonMatrixState == StateNumber_OK)
	  		{
	  			STATE_Display = StateDisplay_SN5;
	  		}
	  		else
	  		{
	  			STATE_Display = StateDisplay_Error;
	  		}
	  		break;
	  	case StateDisplay_SN6:
	  		if(ButtonMatrixState == StateNumber_NoInput)
	  		{
	  			STATE_Display = StateDisplay_SN6;
	  		}
	  		else if(ButtonMatrixState == StateNumber_5)
	  		{
	  			STATE_Display = StateDisplay_SN7;
	  		}
	  		else if(ButtonMatrixState == StateNumber_Clear)
	  		{
	  			STATE_Display = StateDisplay_Start;
	  		}
	  		else if(ButtonMatrixState == StateNumber_OK)
	  		{
	  			STATE_Display = StateDisplay_SN6;
	  		}
	  		else
	  		{
	  			STATE_Display = StateDisplay_Error;
	  		}
	  		break;
	  	case StateDisplay_SN7:
	  		if(ButtonMatrixState == StateNumber_NoInput)
	  		{
	  			STATE_Display = StateDisplay_SN7;
	  		}
	  		else if(ButtonMatrixState == StateNumber_0)
	  		{
	  			STATE_Display = StateDisplay_SN8;
	  		}
	  		else if(ButtonMatrixState == StateNumber_Clear)
	  		{
	  			STATE_Display = StateDisplay_Start;
	  		}
	  		else if(ButtonMatrixState == StateNumber_OK)
	  		{
	  			STATE_Display = StateDisplay_SN8;
	  		}
	  		else
	  		{
	  			STATE_Display = StateDisplay_Error;
	  		}
	  		break;
	  	case StateDisplay_SN8:
	  		if(ButtonMatrixState == StateNumber_NoInput)
	  		{
	  			STATE_Display = StateDisplay_SN8;
	  		}
	  		else if(ButtonMatrixState == StateNumber_0)
	  		{
	  			STATE_Display = StateDisplay_SN9;
	  		}
	  		else if(ButtonMatrixState == StateNumber_Clear)
	  		{
	  			STATE_Display = StateDisplay_Start;
	  		}
	  		else if(ButtonMatrixState == StateNumber_OK)
	  		{
	  			STATE_Display = StateDisplay_SN8;
	  		}
	  		else
	  		{
	  			STATE_Display = StateDisplay_Error;
	  		}
	  		break;
	  	case StateDisplay_SN9:
	  		if(ButtonMatrixState == StateNumber_NoInput)
	  		{
	  			STATE_Display = StateDisplay_SN9;
	  		}
	  		else if(ButtonMatrixState == StateNumber_0)
	  		{
	  			STATE_Display = StateDisplay_SN10;
	  		}
	  		else if(ButtonMatrixState == StateNumber_Clear)
	  		{
	  			STATE_Display = StateDisplay_Start;
	  		}
	  		else if(ButtonMatrixState == StateNumber_OK)
	  		{
	  			STATE_Display = StateDisplay_SN9;
	  		}
	  		else
	  		{
	  			STATE_Display = StateDisplay_Error;
	  		}
	  		break;
	  	case StateDisplay_SN10:
	  		if(ButtonMatrixState == StateNumber_NoInput)
	  		{
	  			STATE_Display = StateDisplay_SN10;
	  		}
	  		else if(ButtonMatrixState == StateNumber_1)
	  		{
	  			STATE_Display = StateDisplay_SN11;
	  		}
	  		else if(ButtonMatrixState == StateNumber_Clear)
	  		{
	  			STATE_Display = StateDisplay_Start;
	  		}
	  		else if(ButtonMatrixState == StateNumber_OK)
	  		{
	  			STATE_Display = StateDisplay_SN10;
	  		}
	  		else
	  		{
	  			STATE_Display = StateDisplay_Error;
	  		}
	  		break;
	  	case StateDisplay_SN11:
	  		if(ButtonMatrixState == StateNumber_NoInput)
	  		{
	  			STATE_Display = StateDisplay_SN11;
	  		}
	  		else if(ButtonMatrixState == StateNumber_6)
	  		{
	  			STATE_Display = StateDisplay_OK;
	  		}
	  		else if(ButtonMatrixState == StateNumber_Clear)
	  		{
	  			STATE_Display = StateDisplay_Start;
	  		}
	  		else if(ButtonMatrixState == StateNumber_OK)
	  		{
	  			STATE_Display = StateDisplay_SN1;
	  		}
	  		else
	  		{
	  			STATE_Display = StateDisplay_Error;
	  		}
	  		break;
	  	case StateDisplay_OK:
	  		if(ButtonMatrixState == StateNumber_NoInput)
	  		{
	  			STATE_Display = StateDisplay_OK;
	  		}
	  		else if(ButtonMatrixState == StateNumber_OK)
	  		{
	  			STATE_Display = StateDisplay_LED2;
	  		}
	  		else
	  		{
	  			STATE_Display = StateDisplay_Error;
	  		}
	  		break;
	  	case StateDisplay_LED2:
	  		if(ButtonMatrixState == StateNumber_NoInput)
	  		{
	  			STATE_Display = StateDisplay_LED2;
	  			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	  		}
	  		else
	  		{
	  			STATE_Display = StateDisplay_Error;
	  			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	  		}
	  	case StateDisplay_Error:
	  		if(ButtonMatrixState == StateNumber_Clear)
	  		{
	  			STATE_Display = StateDisplay_Start;
	  		}
	  		else
	  		{
		  		STATE_Display = StateDisplay_Error;
	  		}
	  		break;
	  }
	  ButtonState[1] = ButtonState[0];





  //--------------------------------------------------------------------------------
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

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

  /*Configure GPIO pins : PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//Function Implementation
GPIO_TypeDef* ButtonMatrixPort[8] = {GPIOA,       GPIOB,      GPIOB,      GPIOB,             GPIOA,      GPIOC,      GPIOB,      GPIOA     };
uint16_t      ButtonMatrixPin [8] = {GPIO_PIN_10, GPIO_PIN_3, GPIO_PIN_5, GPIO_PIN_4,        GPIO_PIN_9, GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_7};
uint8_t       ButtonMatrixLine    = 0;
void ButtonMatrixUpdate()
{
	if(HAL_GetTick() - ButtonMatrixTimeStamp >= 100)
	{
		ButtonMatrixTimeStamp = HAL_GetTick();
		for(int i = 0; i < 4; i++)
		{
			GPIO_PinState PinState = HAL_GPIO_ReadPin(ButtonMatrixPort[i], ButtonMatrixPin[i]);
			if(PinState == GPIO_PIN_RESET) // Button is Pressed
			{
				ButtonMatrixState |= (uint16_t)1 << (i + ButtonMatrixLine * 4);

			}
			else
			{
				ButtonMatrixState &= ~((uint16_t)1 << (i + ButtonMatrixLine * 4));

			}
		}
		uint8_t NowOutputPin = ButtonMatrixLine + 4;
		HAL_GPIO_WritePin(ButtonMatrixPort[NowOutputPin], ButtonMatrixPin[NowOutputPin], GPIO_PIN_SET);
		ButtonMatrixLine = (ButtonMatrixLine + 1) % 4;

		uint8_t NextOutputPin = ButtonMatrixLine + 4;
		HAL_GPIO_WritePin(ButtonMatrixPort[NextOutputPin], ButtonMatrixPin[NextOutputPin], GPIO_PIN_RESET);
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
