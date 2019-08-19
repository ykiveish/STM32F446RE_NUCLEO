/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NRB_OF_CONVERSION 		16
#define WINDOW_SIZE 			6
#define SAMPLE_NOISE_THRESHOLD 	50
#define DATA_THRESHOLD 			180
#define NO_DATA_THRESHOLD		70

typedef struct {
	int		 	SignalAVG;
	int		 	SignalSum;
	uint32_t	SamplesCount;
} DSPContext;

typedef struct {
	int 		Sum;
	uint32_t	LeftIndex;
	uint32_t	RightIndex;
	int			LeftItem;
	int			RightItem;
} SampleWindow;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint8_t BufferReadyToProcess = 0x0;

uint8_t adc_buffer[NRB_OF_CONVERSION];
uint8_t data_to_process[2][2048];
uint8_t dsp_buffers[2][2048];
short diffSignalBuffer[2048];
short temp_buffer_3[2048];

uint16_t data_to_process_index = 0;
uint8_t buffer_index = 0;
uint8_t irq_data_from_dac = 0x0;
uint8_t packets_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void ProcessDMADone(void);
void ProcessWindowData(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ProcessDMADone(void) {
	// memcpy(&data_to_process[buffer_index][data_to_process_index], adc_buffer, NRB_OF_CONVERSION);
	// data_to_process_index += NRB_OF_CONVERSION;
	data_to_process[buffer_index][data_to_process_index] = adc_buffer[8];
	data_to_process_index++;

	if (data_to_process_index >= 2048) {
		data_to_process_index 	= 0x0;
		buffer_index			= 0x1 - buffer_index;
		BufferReadyToProcess 	= 0x1;
	}
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,0);
}

void MoveWindowRight(uint8_t* buffer, SampleWindow * window, uint16_t sample) {
	if (window->RightIndex - window->LeftIndex > WINDOW_SIZE) {
		window->LeftIndex++;
		window->LeftItem = buffer[window->LeftIndex];
	}

	if (window->RightIndex > WINDOW_SIZE) {
		window->Sum -= window->LeftItem;
	}

	window->Sum += sample;
	window->RightIndex++;
}

void AdvancedDSP(void) { /*
	SampleWindow RightWindow = {};
	SampleWindow LeftWindow  = {};
	DSPContext	 Context	 = {};

	Context.SamplesCount = 0;
	Context.SignalAVG = 0;
	Context.SignalSum = 0;

	RightWindow.LeftIndex = 0;
	RightWindow.LeftItem = 0;
	RightWindow.RightIndex = 0;
	RightWindow.RightItem = 0;
	RightWindow.Sum = 0;

	LeftWindow.LeftIndex = 0;
	LeftWindow.LeftItem = 0;
	LeftWindow.RightIndex = 0;
	LeftWindow.RightItem = 0;
	LeftWindow.Sum = 0;

	int MaxDiffPositive = 0;
	int MaxDiffNegative = 0;
	int MinDiffPositive = 0;
	int MinDiffNegative = 0;

	// Window summary difference signal
	for (uint16_t idx = 0; idx < 2048; idx++) {
		MoveWindowRight(ptrRawBuffer, &RightWindow, ptrRawBuffer[idx]);

		if (RightWindow.LeftIndex - LeftWindow.RightIndex == 1) {
			MoveWindowRight(ptrRawBuffer, &LeftWindow, ptrRawBuffer[idx - WINDOW_SIZE]);
		}

		if (ptrRawBuffer[idx] > SAMPLE_NOISE_THRESHOLD) {
			Context.SignalSum += ptrRawBuffer[idx];
			Context.SamplesCount++;
		}

		diffSignalBuffer[idx] = RightWindow.Sum - LeftWindow.Sum;
		if (diffSignalBuffer[idx] > 0) {
			if (diffSignalBuffer[idx] > MaxDiffPositive) {
				MaxDiffPositive = diffSignalBuffer[idx];
			}

			if (diffSignalBuffer[idx] < MinDiffPositive) {

			}
		} else {
			if (diffSignalBuffer[idx] < MaxDiffNegative) {
				MaxDiffNegative = diffSignalBuffer[idx];
			}
		}
	}

	Context.SignalAVG = Context.SignalSum / Context.SamplesCount;

	ptrRawBuffer = &dsp_buffers[0][0];
	for (uint16_t idx = 0; idx < 2048; idx++) {
		ptrRawBuffer[idx] = ((float)((float)(diffSignalBuffer[idx]) / (float)1000.0) * 100.0) + 100;
	}

	for (uint16_t idx = 0; idx < 2048; idx++) {
		if (diffSignalBuffer[idx] > 0) {
			if (diffSignalBuffer[idx] > MaxDiffPositive) {
				MaxDiffPositive = diffSignalBuffer[idx];
			}
		} else {
			if (diffSignalBuffer[idx] < MaxDiffNegative) {
				MaxDiffNegative = diffSignalBuffer[idx];
			}
		}


		if (diffSignalBuffer[idx] > Context.SignalAVG) {
			temp_buffer_3[idx] = Context.SignalAVG;
		} else if (diffSignalBuffer[idx] < -Context.SignalAVG) {
			temp_buffer_3[idx] = 0 - Context.SignalAVG;
		} else {
			temp_buffer_3[idx] = 0;
		}
	}

	for (uint16_t idx = 0; idx < 2048; idx++) {
		temp_buffer[idx] = ((float)((float)(temp_buffer_3[idx]) / (float)1000.0) * 100.0) + 100;
	}

	uint8_t isPulse 			= 0x0;
	uint16_t pulseSampleCount 	= 0;
	uint16_t pulseLeftIndex 	= 0;
	int prevSample 				= temp_buffer_3[0];

	for (uint16_t idx = 1; idx < 2048; idx++) {
		if (temp_buffer_3[idx] != prevSample) {
			if (!prevSample) {
				if (temp_buffer_3[idx] > 0) {
					temp_buffer[idx - 1] = 50;
					if (isPulse == 0x0) {
						pulseLeftIndex = idx -1;
					}
					isPulse = 0x1;
					pulseSampleCount++;
				} else {
					temp_buffer[idx - 1] = 50;
					isPulse = 0x1;
					pulseSampleCount++;
				}
			} else if (prevSample > 0) {
				if (temp_buffer_3[idx] == 0) {
					temp_buffer[idx - 1] = 50;
					isPulse = 0x1;
					pulseSampleCount++;
				}
			} else if (prevSample < 0) {
				if (temp_buffer_3[idx] == 0) {
					temp_buffer[idx - 1] = 0;
					isPulse = 0x0;
					pulseLeftIndex 		= 0;
					pulseSampleCount 	= 0;
				}
			}
		} else {
			if (!prevSample) {
				if (isPulse == 0x1) {
					temp_buffer[idx - 1] = 50;
					pulseSampleCount++;
				} else {
					temp_buffer[idx - 1] = 0;
				}
			} else {
				temp_buffer[idx - 1] = 50;
			}
		}
		prevSample = temp_buffer_3[idx];
	}
*/ }

void ProcessWindowData(void) {
	memcpy(&dsp_buffers[0][0], &data_to_process[buffer_index][0], 2048);
	uint8_t* ptrRawBuffer = &dsp_buffers[0][0];

	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,0);

	uint8_t state = 0x0;
	uint8_t* ptrOutBuffer = &dsp_buffers[1][0];
	for (uint16_t idx = 0; idx < 2048; idx++) {
		if (0x1 == state || 0x0 == state) {
			if (DATA_THRESHOLD < ptrRawBuffer[idx]) {
				ptrOutBuffer[idx] = 25;
				state = 0x2;
			} else {
				ptrOutBuffer[idx] = 0;
			}
		} else if (0x2 == state) {
			if (NO_DATA_THRESHOLD > ptrRawBuffer[idx]) {
				ptrOutBuffer[idx] = 0;
				state = 0x1;
			} else {
				ptrOutBuffer[idx] = 25;
			}
		}
	}

	HAL_UART_Transmit_DMA(&huart1, ptrOutBuffer, 2048);
	// Data processing section
	for (uint16_t idx = 0; idx < 3; idx++){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,0);
	}

	if (packets_count > 10) {
		HAL_ADC_Stop_DMA(&hadc1);
		packets_count = 0;
	}

	packets_count++;
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart) {
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,0);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, NRB_OF_CONVERSION);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (BufferReadyToProcess) {
		  ProcessWindowData();
		  BufferReadyToProcess = 0x0;
	  }

	  if (0x1 == irq_data_from_dac) {
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, NRB_OF_CONVERSION);
		  irq_data_from_dac = 0x0;
	  }

	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,1);
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,0);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 16;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 15;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 16;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
