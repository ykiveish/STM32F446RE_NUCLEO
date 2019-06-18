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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SPI_TIMEOUT_MAX				0x1000
#define CMD_LENGTH					((uint16_t)0x0004)

#define COMM_SPI_READ				0x03
#define COMM_SPI_WRITE				0x02

#define ESC_WRITE 		   			0x80
#define ESC_READ 		   			0xC0
#define ECAT_CSR_BUSY     			0x80

#define AL_CONTROL              	0x0120      			// AL control
#define AL_STATUS               	0x0130      			// AL status
#define AL_STATUS_CODE          	0x0134      			// AL status code
#define AL_EVENT                	0x0220      			// AL event request
#define AL_EVENT_MASK           	0x0204      			// AL event interrupt mask

#define TOT_BYTE_NUM_OUT			1
#define TOT_BYTE_NUM_IN				8
#define TOT_BYTE_NUM_ROUND_OUT		4
#define TOT_BYTE_NUM_ROUND_IN		8

#define PRAM_ABORT        			0x40000000
#define PRAM_BUSY         			0x80
#define PRAM_AVAIL        			0x01
#define READY             			0x08
#define DUMMY_BYTE					0xFF

#define BYTE_TEST               	0x0064      			// byte order test register
#define HW_CFG                  	0x0074      			// hardware configuration register
#define RESET_CTL               	0x01F8      			// reset register
#define ECAT_CSR_DATA           	0x0300      			// EtherCAT CSR Interface Data Register
#define ECAT_CSR_CMD            	0x0304      			// EtherCAT CSR Interface Command Register
#define ECAT_PRAM_RD_ADDR_LEN   	0x0308      			// EtherCAT Process RAM Read Address and Length Register
#define ECAT_PRAM_RD_CMD        	0x030C      			// EtherCAT Process RAM Read Command Register
#define ECAT_PRAM_WR_ADDR_LEN   	0x0310      			// EtherCAT Process RAM Write Address and Length Register
#define ECAT_PRAM_WR_CMD        	0x0314      			// EtherCAT Process RAM Write Command Register
#define WDOG_STATUS             	0x0440      			// watch dog status

#define DIGITAL_RST       			0x00000001

#define ESM_INIT                	0x01          			// state machine control
#define ESM_PREOP               	0x02          			// (state request)
#define ESM_BOOT                	0x03          			//
#define ESM_SAFEOP              	0x04          			// safe-operational
#define ESM_OP                  	0x08          			// operational

#define Tout 						1000
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef union {
    unsigned short  Word;
    unsigned char   Byte[2];
} UWORD;

typedef union {
    unsigned long   Long;
    unsigned short  Word[2];
    unsigned char   Byte[4];
} ULONG;

typedef union {
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct {
		uint8_t     Leds;
	} Cust;
} PROCBUFFER_OUT;

typedef union {
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct {
		uint16_t    Analog_0;
		uint16_t    Analog_1;
		uint16_t    Bit16_RisingTestRamp;
		uint8_t     DipSwitches;
		uint8_t     Bit8_FallingTestRamp;
	} Cust;
} PROCBUFFER_IN;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
unsigned long SPIReadRegisterDirect(unsigned short Address, unsigned char Len);
void SPIWriteRegisterDirect (unsigned short Address, unsigned long DataOut);
unsigned long SPIReadRegisterIndirect (unsigned short Address, unsigned char Len);
void SPIWriteRegisterIndirect (unsigned long DataOut, unsigned short Address, unsigned char Len);
void SPIWriteProcRamFifo(void);
void SPIReadProcRamFifo(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t 		IsDeviceInitiated = 0x1;
PROCBUFFER_OUT 	BufferOut;
PROCBUFFER_IN 	BufferIn;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned long SPIReadRegisterDirect (unsigned short Address, unsigned char Len) {
	ULONG Command, Result;
	UWORD Addr;
	char uartbuffer[64] = {0};

	Addr.Word = Address;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

	Command.Byte[0] = COMM_SPI_READ;
	Command.Byte[1] = Addr.Byte[1];
	Command.Byte[2] = Addr.Byte[0];

	if (HAL_SPI_Transmit(&hspi2, Command.Byte, 3, SPI_TIMEOUT_MAX) != HAL_OK) {
		memset(uartbuffer, 0, 64);
		sprintf(uartbuffer, "ERROR# HAL_SPI_Transmit\r\n\r\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)uartbuffer, sizeof(uartbuffer), HAL_MAX_DELAY);
	}

	if (HAL_SPI_Receive(&hspi2, Result.Byte, Len, SPI_TIMEOUT_MAX) != HAL_OK) {
		memset(uartbuffer, 0, 64);
		sprintf(uartbuffer, "ERROR# HAL_SPI_Receive\r\n\r\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)uartbuffer, sizeof(uartbuffer), HAL_MAX_DELAY);
	}

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

	return Result.Long;
}

void SPIWriteRegisterDirect (unsigned short Address, unsigned long DataOut) {
  ULONG Data;
  UWORD Addr;
  uint8_t Buffer[8] = {0};
  char uartbuffer[64] = {0};

  Addr.Word = Address;
  Data.Long = DataOut;

  Buffer[0] = COMM_SPI_WRITE;
  Buffer[1] = Addr.Byte[1];
  Buffer[2] = Addr.Byte[0];
  Buffer[3] = Data.Byte[0];
  Buffer[4] = Data.Byte[1];
  Buffer[5] = Data.Byte[2];
  Buffer[6] = Data.Byte[3];

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  if (HAL_SPI_Transmit(&hspi2, Buffer, 7, SPI_TIMEOUT_MAX) != HAL_OK) {
	memset(uartbuffer, 0, 64);
	sprintf(uartbuffer, "ERROR# HAL_SPI_Transmit\r\n\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t *)uartbuffer, sizeof(uartbuffer), HAL_MAX_DELAY);
  }

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
}

unsigned long SPIReadRegisterIndirect (unsigned short Address, unsigned char Len) {
  ULONG TempLong;
  UWORD Addr;

  Addr.Word = Address;

  /*
	CSR_BUSY	|	R_nW	|	RESERVED	|	CSR_SIZE	|	CSR_ADDR
	31			|	30		|	29:19		|	18:16		|	15:0
   */
  TempLong.Byte[0] = Addr.Byte[0];                          // address of the register
  TempLong.Byte[1] = Addr.Byte[1];                          // to read, LsByte first
  TempLong.Byte[2] = Len;                                   // number of bytes to read
  TempLong.Byte[3] = ESC_READ;                              // ESC read
  SPIWriteRegisterDirect (ECAT_CSR_CMD, TempLong.Long);

  // wait for command execution
  do {
	TempLong.Long = SPIReadRegisterDirect(ECAT_CSR_CMD, 4);
  } while(TempLong.Byte[3] & ECAT_CSR_BUSY);

  TempLong.Long = SPIReadRegisterDirect(ECAT_CSR_DATA, Len); // read the requested register

  return TempLong.Long;
}

void SPIWriteRegisterIndirect (unsigned long DataOut, unsigned short Address, unsigned char Len) {
  ULONG TempLong;
  UWORD Addr;

  Addr.Word = Address;

  SPIWriteRegisterDirect(ECAT_CSR_DATA, DataOut);            	// write the data

  TempLong.Byte[0] = Addr.Byte[0];                            	// address of the register
  TempLong.Byte[1] = Addr.Byte[1];                            	// to write, LsByte first
  TempLong.Byte[2] = Len;                                     	// number of bytes to write
  TempLong.Byte[3] = ESC_WRITE;                               	// ESC write

  SPIWriteRegisterDirect(ECAT_CSR_CMD, TempLong.Long);       	// write the command

  do {
    TempLong.Long = SPIReadRegisterDirect(ECAT_CSR_CMD, 4);
  } while (TempLong.Byte[3] & ECAT_CSR_BUSY);
}

#define FST_BYTE_NUM_ROUND_IN TOT_BYTE_NUM_IN
/*
 * From master
 */
void SPIWriteProcRamFifo(void) {
  ULONG TempLong;
  unsigned char i;
  uint8_t Buffer[32] = {0};
  char uartbuffer[64] = {0};

  // abort any possible pending transfer
  SPIWriteRegisterDirect(ECAT_PRAM_WR_CMD, PRAM_ABORT);
  SPIWriteRegisterDirect(ECAT_PRAM_WR_ADDR_LEN, (0x00001200 | (((uint32_t)TOT_BYTE_NUM_IN) << 16)));
  // start command (set PRAM_WRITE_BUSY)
  SPIWriteRegisterDirect(ECAT_PRAM_WR_CMD, 0x80000000);

  do {
      TempLong.Long = SPIReadRegisterDirect (ECAT_PRAM_WR_CMD,2);
  } while (TempLong.Byte[1] < (FST_BYTE_NUM_ROUND_IN/4));

  Buffer[0] = COMM_SPI_WRITE;
  Buffer[1] = 0x00; // address of the write fifo
  Buffer[2] = 0x20; // MsByte first (ECAT_PRAM_WR_DATA 020h-03Ch [ETHERCAT PROCESS RAM WRITE DATA FIFO])
  
  // transfer the data
  for (i = 0; i < FST_BYTE_NUM_ROUND_IN; i++) {
	  Buffer[3 + i] = BufferIn.Byte[i];
  }
  
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi2, Buffer, FST_BYTE_NUM_ROUND_IN + 3, SPI_TIMEOUT_MAX) != HAL_OK) {
	memset(uartbuffer, 0, 64);
	sprintf(uartbuffer, "ERROR# HAL_SPI_Transmit\r\n\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t *)uartbuffer, sizeof(uartbuffer), HAL_MAX_DELAY);
  }
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

  memset(uartbuffer, 0, 64);
  sprintf(uartbuffer, ">TRANSMIT\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t *)uartbuffer, sizeof(uartbuffer), HAL_MAX_DELAY);
}

#define FST_BYTE_NUM_ROUND_OUT TOT_BYTE_NUM_OUT
/*
 * To master
 */
void SPIReadProcRamFifo(void) {
  ULONG TempLong;
  unsigned char i;
  uint8_t Buffer[32] = {0};

  // abort any possible pending transfer
  SPIWriteRegisterDirect (ECAT_PRAM_RD_CMD, PRAM_ABORT);
  SPIWriteRegisterDirect (ECAT_PRAM_RD_ADDR_LEN, (0x00001000 | (((uint32_t)TOT_BYTE_NUM_OUT) << 16)));
  // start command
  SPIWriteRegisterDirect (ECAT_PRAM_RD_CMD, 0x80000000);

  do {
	TempLong.Long = SPIReadRegisterDirect (ECAT_PRAM_RD_CMD,2);
  } while (TempLong.Byte[1] != (FST_BYTE_NUM_ROUND_OUT/4));

  Buffer[0] = COMM_SPI_READ;
  Buffer[1] = 0x00; // address of the read FIFO
  Buffer[2] = 0x00; // FIFO MsByte first
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  // write command
  if (HAL_SPI_Transmit(&hspi2, Buffer, 3, SPI_TIMEOUT_MAX) != HAL_OK) {
    HAL_UART_Transmit(&huart3, (uint8_t *)"ERROR# HAL_SPI_Transmit\r\n", 25, HAL_MAX_DELAY);
  }

  if (HAL_SPI_Receive(&hspi2, BufferOut.Byte, FST_BYTE_NUM_ROUND_OUT, SPI_TIMEOUT_MAX) != HAL_OK) {
  	HAL_UART_Transmit(&huart3, (uint8_t *)"ERROR# HAL_SPI_Receive\r\n", 24, HAL_MAX_DELAY);
  }

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  char buffer[64] = {0};
  ULONG TempLong;
  unsigned char WatchDog = 0;
  unsigned char Operational = 0;
  unsigned char Status;

  SPIWriteRegisterDirect (RESET_CTL, DIGITAL_RST);

  unsigned short i = 0;
  do {
    TempLong.Long = SPIReadRegisterDirect (RESET_CTL, 4);
    sprintf(buffer, "SPI# (RESET_CTL) 0x%08x \r\n", TempLong.Long);
    HAL_UART_Transmit(&huart3, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);
    i++;
  } while (((TempLong.Byte[0] & 0x01) != 0x00) && (i != Tout));

  if (i == Tout) {
	IsDeviceInitiated = 0x0;
  }

  i = 0;
  do {
	TempLong.Long = SPIReadRegisterDirect (BYTE_TEST, 4);
	sprintf(buffer, "SPI# (BYTE_TEST) 0x%08x \r\n", TempLong.Long);
	HAL_UART_Transmit(&huart3, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);
	i++;
  } while ((TempLong.Long != 0x87654321) && (i != Tout));

  if (i == Tout) {
	IsDeviceInitiated = 0x0;
  }

  i = 0;
  do {
	TempLong.Long = SPIReadRegisterDirect (HW_CFG, 4);
	sprintf(buffer, "SPI# (HW_CFG) 0x%08x \r\n", TempLong.Long);
	HAL_UART_Transmit(&huart3, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);
	i++;
  } while (((TempLong.Byte[3] & READY) == 0) && (i != Tout));

  if (i == Tout) {
    IsDeviceInitiated = 0x0;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	uint32_t current_tick = HAL_GetTick();
	if (IsDeviceInitiated) {
	  TempLong.Long = SPIReadRegisterIndirect(WDOG_STATUS, 1); // read the watchdog status
	  if ((TempLong.Byte[0] & 0x01) == 0x01) {
		  WatchDog = 0;	// set/reset the corresponding flag
	  } else {
		  WatchDog = 1;
	  }

	  TempLong.Long = SPIReadRegisterIndirect (AL_STATUS, 1);   // read the EtherCAT State Machine status
	  Status = TempLong.Byte[0] & 0x0F;                         // to see if we are in operational state
	  if (Status == ESM_OP) {
		  Operational = 1;
	  } else {
		  Operational = 0;
	  }

	  SPIWriteProcRamFifo();

	  memset(buffer, 0, 64);
	  sprintf(buffer, "WatchDog: %d, Operational: %d (STATUS:0x%08x)\r\n", WatchDog, Operational, Status);
	  HAL_UART_Transmit(&huart3, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);
	} else {
		memset(buffer, 0, 64);
		sprintf(buffer, "Something wrong, no communication with device\r\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);
	}

	while (HAL_GetTick() <= (current_tick + 500));

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
