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
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define END_ADDR 0b00100011
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t ADCData[4] = { 0 };
uint64_t _micros = 0;
float EncoderVel = 0;
uint64_t Timestamp_Encoder = 0;
uint64_t Timestamp_Encoder2 = 0;
uint64_t Timestamp_Encoder3 = 0;
uint64_t nx = 0;
float vrpm = 0;
float require = 0;
int32_t EncoderPositionDiff;
uint64_t EncoderTimeDiff;
char TxDataBuffer[32] = { 0 };
char RxDataBuffer[32] = { 0 };
int a0, a1, a2, a3;
int zerostate = 0;
int sclk[2] = { 0 };
float position = 0;
float error = 0;
char status[100];
float Kp = 2000, Ki = 200, Kd = 800, errorpid[2] = { 0 }, sumpid = 0, velocity =
		0;
float pi = 3.14159265358993238462643383;
float ptg = 0, vmax = 0.8, rotationtime = 0, a = 0, distance = 0.0, ttrajec = 0,
		calculatedp = 0, previous = 0;
int activate = 0;
int32_t pwm = 0;
int emergency = 0;
int check = 0;
int buffer[32] = { 0 };
int pointer = 0;
int bytecount = 0;
int mode = 0;
int enable = 0;
int stationnumber = 0;
int frame3check = 0;
int station[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int inputchar;
int indexpos = 0;
int laserflag = 0;
int buf = 0;
int gripperstatus = 0;
float dumpvmax = 0.8;
float queue[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float qpointer = 0;
float allq = 0;
char temp[];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint64_t micros();
float EncoderVelocity_Update();
int16_t UARTRecieveIT();
void UARTMode();
void Laser();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_ADC_Start_DMA(&hadc1, ADCData, 4);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
	{
//		char temp[] = "Hello world\r\n please type something\r\n";
		uint8_t ack[] = { 0x58, 0x75 };
		HAL_UART_Transmit(&huart2, (uint8_t*) ack, 2, 10);
	}
//	char temp[] ="\x58\x75";
//	HAL_UART_Transmit(&huart2, (uint8_t*) temp,
//			strlen(temp), 1000);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//I2C
		Laser();
		//UART Part
		HAL_UART_Receive_IT(&huart2, (uint8_t*) RxDataBuffer, 4);
		inputchar = UARTRecieveIT();
		if (inputchar != -1) {
//			HAL_Delay(3000);
			UARTMode();
//			static uint8_t data[2] = {0x58,0x75};
//			HAL_UART_Transmit(&huart2, (uint8_t*)data,2, 1000);
//			sprintf(TxDataBuffer, "\x58\x75");
//			HAL_UART_Transmit(&huart2, (uint8_t*) TxDataBuffer,
//					strlen(TxDataBuffer), 1000);
		}
		//PWM set
		if (emergency == 0) {
			if (require == 9988) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
			} else if (require >= 0) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
			} else if (require < 0) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
			}
			if (micros() - Timestamp_Encoder >= 100) {
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
				Timestamp_Encoder = micros();
				EncoderVel = ((2 * EncoderVel + EncoderVelocity_Update()) / 3);
				vrpm = EncoderVel / 524288 * 60;
			}
			sclk[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
			if (zerostate == 1) {
				require = 0.2;
				if (sclk[0] == 0 && sclk[1] == 1) {
					pwm = 1200;
					position = 0;
					require = 0;
					zerostate = 0;
					error = (TIM5->CNT);
				}
			}
			sclk[1] = sclk[0];
			position = ((TIM5->CNT) - error) / 524288 * 360;
			if (position < 0) {
				position = 360 + position;
			}
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 1) {
				indexpos = position;
			}
			if (micros() - Timestamp_Encoder2 >= 100000) {
				int st1 = position;
				int st2 = vrpm * 6;
				sprintf(status,
						"Position is %d degree Velocity is %d degree/second \r\n",
						st1, st2);
//				HAL_UART_Transmit(&huart2, (uint8_t*) status, strlen(status),
//						10);
				Timestamp_Encoder2 = micros();
			}
		}

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 99;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 10000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 99;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 524288;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin PA8 PA9 */
	GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PA7 PA10 */
	GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA11 */
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void UARTMode() {
	buffer[pointer] = inputchar;
	buf = buffer[pointer];
	if (bytecount == 0) {
		if (inputchar >= 145 && inputchar <= 158) {
			pointer = 0;
			buffer[pointer] = inputchar;
		}
		switch (inputchar) {
		case 145:
			mode = 1;
			bytecount += 1;
			break;
		case 146:
			mode = 2;
			bytecount += 1;
			break;
		case 147:
			mode = 3;
			bytecount += 1;
			break;
		case 148:
			mode = 4;
			bytecount += 1;
			break;
		case 149:
			mode = 5;
			bytecount += 1;
			break;
		case 150:
			mode = 6;
			bytecount += 1;
			break;
		case 151:
			mode = 7;
			bytecount += 1;
			break;
		case 152:
			mode = 8;
			bytecount += 1;
			break;
		case 153:
			mode = 9;
			bytecount += 1;
			break;
		case 154:
			mode = 10;
			bytecount += 1;
			break;
		case 155:
			mode = 11;
			bytecount += 1;
			break;
		case 156:
			mode = 12;
			bytecount += 1;
			break;
		case 157:
			mode = 13;
			bytecount += 1;
			break;
		case 158:
			mode = 14;
			bytecount += 1;
			break;
		default:
			break;
		}
	} else if (mode == 2) {
		if (bytecount == 1
				&& (buffer[pointer] == (buffer[pointer - 1] ^ 0b11111111))) {
			enable = 1;
			uint8_t ack[] = { 0x58, 0x75 };
			HAL_Delay(100);
			HAL_UART_Transmit(&huart2, (uint8_t*) ack, 2, 10);
			mode = 0;
			bytecount = 0;
		} else if (bytecount == 1
				&& (buffer[pointer] != (buffer[pointer - 1] ^ 0b11111111))) {
			mode = 0;
			bytecount = 0;
		}
	} else if (mode == 3) {
		if (bytecount == 1
				&& (buffer[pointer] == (buffer[pointer - 1] ^ 0b11111111))) {
			enable = 0;
			uint8_t ack[] = { 0x58, 0x75 };
			HAL_Delay(100);
			HAL_UART_Transmit(&huart2, (uint8_t*) ack, 2, 10);
			mode = 0;
			bytecount = 0;
		} else if (bytecount == 1
				&& (buffer[pointer] != (buffer[pointer - 1] ^ 0b11111111))) {
			mode = 0;
			bytecount = 0;
		}
	} else if (mode == 4) {
		if (bytecount == 1) {
			bytecount = 2;
		} else if (bytecount == 2) {
			bytecount = 3;
		} else if (bytecount == 3
				&& buffer[pointer]
						== (((buffer[pointer - 1] + buffer[pointer - 2]
								+ buffer[pointer - 3]) % 256) ^ 0b11111111)) {
			uint8_t ack[] = { 0x58, 0x75 };
			HAL_Delay(100);
			HAL_UART_Transmit(&huart2, (uint8_t*) ack, 2, 10);
			vmax = buffer[pointer - 1] / 60.0 * 2.0 * 3.141;
			dumpvmax = vmax;
			mode = 0;
			bytecount = 0;
		} else if (bytecount == 3
				&& buffer[pointer]
						!= (((buffer[pointer - 1] + buffer[pointer - 2]
								+ buffer[pointer - 3]) % 256) ^ 0b11111111)) {
			mode = 0;
			bytecount = 0;
		}
	} else if (mode == 5) {
		if (bytecount == 1) {
			bytecount = 2;
		} else if (bytecount == 2) {
			bytecount = 3;
		} else if (bytecount == 3
				&& buffer[pointer]
						== (((buffer[pointer - 1] + buffer[pointer - 2]
								+ buffer[pointer - 3]) % 256) ^ 0b11111111)) {
			uint8_t ack[] = { 0x58, 0x75 };
			HAL_Delay(100);
			HAL_UART_Transmit(&huart2, (uint8_t*) ack, 2, 10);
			ptg = ((256.0 * buffer[pointer - 2]) + buffer[pointer - 1])
					/ 10000.0;
			mode = 0;
			bytecount = 0;
		} else if (bytecount == 3
				&& buffer[pointer]
						!= (((buffer[pointer - 1] + buffer[pointer - 2]
								+ buffer[pointer - 3]) % 256) ^ 0b11111111)) {
			mode = 0;
			bytecount = 0;
		}
	} else if (mode == 6) {
		if (bytecount == 1) {
			bytecount = 2;
		} else if (bytecount == 2) {
			bytecount = 3;
		} else if (bytecount == 3
				&& buffer[pointer]
						== (((buffer[pointer - 1] + buffer[pointer - 2]
								+ buffer[pointer - 3]) % 256) ^ 0b11111111)) {
			uint8_t ack[] = { 0x58, 0x75 };
			HAL_Delay(100);
			HAL_UART_Transmit(&huart2, (uint8_t*) ack, 2, 10);
			ptg = station[buffer[pointer - 1] - 1];
			mode = 0;
			bytecount = 0;
		} else if (bytecount == 3
				&& buffer[pointer]
						!= (((buffer[pointer - 1] + buffer[pointer - 2]
								+ buffer[pointer - 3]) % 256) ^ 0b11111111)) {
			mode = 0;
			bytecount = 0;
		}
	} else if (mode == 7) {
		if (bytecount == 1) {
			stationnumber = buffer[pointer];
			frame3check += (buffer[pointer] + buffer[pointer - 1]);
			bytecount = 2;
		} else if (bytecount < 2 + stationnumber) {
			frame3check += buffer[pointer];
			bytecount += 1;
		} else if (bytecount == 2 + stationnumber
				&& buffer[pointer] == ((frame3check % 256) ^ 0b11111111)) {
			//set goal n station
			uint8_t ack[] = { 0x58, 0x75 };
			HAL_Delay(100);
			HAL_UART_Transmit(&huart2, (uint8_t*) ack, 2, 10);
			mode = 0;
			bytecount = 0;
			frame3check = 0;
		} else if (bytecount == 2 + stationnumber
				&& buffer[pointer] != ((frame3check % 256) ^ 0b11111111)) {
			mode = 0;
			bytecount = 0;
			frame3check = 0;
		}
	} else if (mode == 8) {
		if (bytecount == 1
				&& (buffer[pointer] == (buffer[pointer - 1] ^ 0b11111111))) {
			activate = 1;
			uint8_t ack[] = { 0x58, 0x75 };
			HAL_Delay(100);
			HAL_UART_Transmit(&huart2, (uint8_t*) ack, 2, 10);
			mode = 0;
			bytecount = 0;
		} else if (bytecount == 1
				&& (buffer[pointer] != (buffer[pointer - 1] ^ 0b11111111))) {
			mode = 0;
			bytecount = 0;
		}
	} else if (mode == 9) {
		if (bytecount == 1
				&& (buffer[pointer] == (buffer[pointer - 1] ^ 0b11111111))) {
			for (int i = 0; i < 10; ++i) {
				if (position - station[i] <= 0.01) {
					sprintf(temp, 'Station %d', i);
					HAL_UART_Transmit(&huart2, (uint8_t*) temp, strlen(temp),
							10);
				}
			}
			mode = 0;
			bytecount = 0;
		} else if (bytecount == 1
				&& (buffer[pointer] != (buffer[pointer - 1] ^ 0b11111111))) {
			mode = 0;
			bytecount = 0;
		}
	} else if (mode == 10) {
		if (bytecount == 1
				&& (buffer[pointer] == (buffer[pointer - 1] ^ 0b11111111))) {
			//request position
			sprintf(temp, '%d degree', position);
			HAL_UART_Transmit(&huart2, (uint8_t*) temp, strlen(temp), 10);
			mode = 0;
			bytecount = 0;
		} else if (bytecount == 1
				&& (buffer[pointer] != (buffer[pointer - 1] ^ 0b11111111))) {
			mode = 0;
			bytecount = 0;
		}
	} else if (mode == 11) {
		if (bytecount == 1
				&& (buffer[pointer] == (buffer[pointer - 1] ^ 0b11111111))) {
			//request Max velocity
			sprintf(temp, '%d rad/s', vmax);
			HAL_UART_Transmit(&huart2, (uint8_t*) temp, strlen(temp), 10);
			mode = 0;
			bytecount = 0;
		} else if (bytecount == 1
				&& (buffer[pointer] != (buffer[pointer - 1] ^ 0b11111111))) {
			mode = 0;
			bytecount = 0;
		}
	} else if (mode == 12) {
		if (bytecount == 1
				&& (buffer[pointer] == (buffer[pointer - 1] ^ 0b11111111))) {
			//enable gripper
			uint8_t ack[] = { 0x58, 0x75 };
			HAL_Delay(100);
			HAL_UART_Transmit(&huart2, (uint8_t*) ack, 2, 10);
			laserflag = 1;
			mode = 0;
			bytecount = 0;
		} else if (bytecount == 1
				&& (buffer[pointer] != (buffer[pointer - 1] ^ 0b11111111))) {
			mode = 0;
			bytecount = 0;
		}
	} else if (mode == 13) {
		if (bytecount == 1
				&& (buffer[pointer] == (buffer[pointer - 1] ^ 0b11111111))) {
			//disable gripper
			uint8_t ack[] = { 0x58, 0x75 };
			HAL_Delay(100);
			HAL_UART_Transmit(&huart2, (uint8_t*) ack, 2, 10);
			mode = 0;
			bytecount = 0;
		} else if (bytecount == 1
				&& (buffer[pointer] != (buffer[pointer - 1] ^ 0b11111111))) {
			mode = 0;
			bytecount = 0;
		}
	} else if (mode == 14) {
		if (bytecount == 1
				&& (buffer[pointer] == (buffer[pointer - 1] ^ 0b11111111))) {
			uint8_t ack[] = { 0x58, 0x75 };
			HAL_Delay(100);
			HAL_UART_Transmit(&huart2, (uint8_t*) ack, 2, 10);
			zerostate = 1;
			mode = 0;
			bytecount = 0;
		} else if (bytecount == 1
				&& (buffer[pointer] != (buffer[pointer - 1] ^ 0b11111111))) {
			mode = 0;
			bytecount = 0;
		}
	}
	pointer += 1;
	if (pointer > 32)
		pointer = 0;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_11) {
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 0) {
			emergency = 1;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
			pwm = 9000;
		} else {
			require = 0;
			ttrajec = rotationtime + 1;
			pwm = 0;
			emergency = 0;
			sumpid = 0;
			zerostate = 0;
		}

	}
}
int16_t UARTRecieveIT() {
	static uint32_t dataPos = 0;
	int16_t data = -1;
	if (huart2.RxXferSize - huart2.RxXferCount != dataPos) {
		data = RxDataBuffer[dataPos];
		dataPos = (dataPos + 1) % huart2.RxXferSize;
	}
	return data;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	sprintf(TxDataBuffer, "Received:[%d%d%d%d]\r\n", RxDataBuffer[0],
			RxDataBuffer[1], RxDataBuffer[2], RxDataBuffer[3]);
	a0 = RxDataBuffer[0];
	a1 = RxDataBuffer[1];
	a2 = RxDataBuffer[2];
	a3 = RxDataBuffer[3];
	if (a0 == 115 && a1 == 116 && a2 == 111 && a3 == 112) { //stop
		require = 0;
	} else if (a0 == 115 && a1 == 101 && a2 == 116 && a3 == 48) { //set0
		require = 3;
		pwm = 1000;
		zerostate = 1;
	} else if (a0 == 43) { //+
		require = ((RxDataBuffer[1] - 48) * 100) + ((RxDataBuffer[2] - 48) * 10)
				+ ((RxDataBuffer[3] - 48) * 1);
	} else if (a0 == 45) { //-
		require = -1
				* (((RxDataBuffer[1] - 48) * 100)
						+ ((RxDataBuffer[2] - 48) * 10)
						+ ((RxDataBuffer[3] - 48) * 1));
	}
//	HAL_UART_Transmit(&huart2, (uint8_t*) TxDataBuffer, strlen(TxDataBuffer),
//			1000);
}

#define  HTIM_ENCODER htim5
#define  MAX_SUBPOSITION_OVERFLOW 262144
#define  MAX_ENCODER_PERIOD 524288

float EncoderVelocity_Update() {
	//Save Last state
	static uint32_t EncoderLastPosition = 0;
	static uint64_t EncoderLastTimestamp = 0;

	//read data
	uint32_t EncoderNowPosition = HTIM_ENCODER.Instance->CNT;
	uint64_t EncoderNowTimestamp = micros();

	EncoderTimeDiff = EncoderNowTimestamp - EncoderLastTimestamp;
	EncoderPositionDiff = EncoderNowPosition - EncoderLastPosition;

	//compensate overflow and underflow
	if (EncoderPositionDiff >= MAX_SUBPOSITION_OVERFLOW) {
		EncoderPositionDiff -= MAX_ENCODER_PERIOD;
	} else if (-EncoderPositionDiff >= MAX_SUBPOSITION_OVERFLOW) {
		EncoderPositionDiff += MAX_ENCODER_PERIOD;
	}

	//Update Position and time
	EncoderLastPosition = EncoderNowPosition;
	EncoderLastTimestamp = EncoderNowTimestamp;

	//Calculate velocity
	//EncoderTimeDiff is in uS
	return (EncoderPositionDiff * 1000000) / (float) EncoderTimeDiff;

}
void Laser() {
	if (laserflag == 1 && hi2c1.State == HAL_I2C_STATE_READY) {
		static uint8_t data[1] = { 0x45 };
		HAL_I2C_Master_Transmit(&hi2c1, END_ADDR << 1, data, 1, 1000);
		laserflag = 0;
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		_micros += 4294967295;
	}
	if (htim == &htim4) {
		if (emergency == 0) {
			if (zerostate == 0) {
				if (activate == 1) {
					ttrajec = 0;
					if (ptg < (2 * pi / 360 * position)) {
						if (((2 * pi / 360 * position) - ptg)
								<= (2 * pi - (2 * pi / 360 * position) + ptg)) {
							distance = ptg - (2 * pi / 360 * position);
						} else {
							distance = 2 * pi - (2 * pi / 360 * position) + ptg;
						}
					} else if (ptg >= (2 * pi / 360 * position)) {
						if (ptg - (2 * pi / 360 * position)
								<= 2 * pi - ptg + (2 * pi / 360 * position)) {
							distance = ptg - (2 * pi / 360 * position);
						} else {
							distance = -(2 * pi - ptg
									+ (2 * pi / 360 * position));
						}
					}
					if (distance >= 0) {
						if (8 / 5 * vmax * vmax / distance >= 0.5) {
							vmax = sqrt(0.5 * distance * 3 / 8);
						}
						rotationtime = (3.00 * distance) / (2 * vmax);
					} else {
						if (-8 / 5 * vmax * vmax / distance >= 0.5) {
							vmax = sqrt(-0.5 * distance * 3 / 8);
						}
						rotationtime = -(3.00 * distance) / (2 * vmax);
					}
					a = 2 * sqrt(vmax) / rotationtime;
					activate = 0;
				}
				if (distance >= 0) {
					require = -(a * a * ttrajec * ttrajec)
							+ (2 * sqrt(vmax) * a * ttrajec);
				} else {
					require = (a * a * ttrajec * ttrajec)
							- (2 * sqrt(vmax) * a * ttrajec);
				}
				if (distance >= 0) {
					calculatedp = (-(a * a * ttrajec * ttrajec * ttrajec / 3)
							+ (sqrt(vmax) * a * ttrajec * ttrajec)) + previous;
				} else {
					calculatedp = ((a * a * ttrajec * ttrajec * ttrajec / 3)
							- (sqrt(vmax) * a * ttrajec * ttrajec)) + previous;
				}
				ttrajec += 0.001;
				if (ttrajec >= rotationtime) {
					a = 0;
					previous = 2 * pi / 360 * position;
					sumpid = 0;
					vmax = dumpvmax;
				}
			}
			// PID
			velocity = EncoderVel / 524288 * (2 * pi);
			if (require == 0)
				errorpid[0] = 0;
			if (require < 0) {
				errorpid[0] = velocity - require;
			}
			if (require > 0) {
				errorpid[0] = require - velocity;
			}
			sumpid = sumpid + errorpid[0];
			pwm = Kp * errorpid[0] + Ki * sumpid
					+ Kd * (errorpid[0] - errorpid[1]);
			errorpid[1] = errorpid[0];
			if (pwm < 0) {
				pwm = -pwm;
			}
			if (pwm > 9001)
				pwm = 9000;
		}
	}
}
uint64_t micros() {
	return _micros + htim2.Instance->CNT;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
