/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "mpu.h"
#include "dht.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#define BMP180_ADDR 0x77 << 1

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
uint16_t readValue;
uint16_t readValue2;
int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
uint16_t AC4, AC5, AC6;
/* USER CODE END Includes */
int32_t UT, UP, B5;
int temperature, pressure, altitude;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void BMP180_ReadCalibrationData(void);
int16_t BMP180_Read16(uint8_t reg);
uint16_t BMP180_ReadU16(uint8_t reg);
void BMP180_StartTemperature(void);
void BMP180_ReadUT(void);
void BMP180_StartPressure(void);
void BMP180_ReadUP(void);
float BMP180_CalculateAltitude(float pressure);

void HAL_Delay(uint32_t delay);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
int16_t BMP180_Read16(uint8_t reg) {
	uint8_t data[2];
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR, reg, 1, data, 2, HAL_MAX_DELAY);
	return (int16_t)((data[0] << 8) | data[1]);
}

// Read 16-bit unsigned value
uint16_t BMP180_ReadU16(uint8_t reg) {
	uint8_t data[2];
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR, reg, 1, data, 2, HAL_MAX_DELAY);
	return (uint16_t)((data[0] << 8) | data[1]);
}

// Read calibration data
void BMP180_ReadCalibrationData(void) {
	AC1 = BMP180_Read16(0xAA);
	AC2 = BMP180_Read16(0xAC);
	AC3 = BMP180_Read16(0xAE);
	AC4 = BMP180_ReadU16(0xB0);
	AC5 = BMP180_ReadU16(0xB2);
	AC6 = BMP180_ReadU16(0xB4);
	B1 = BMP180_Read16(0xB6);
	B2 = BMP180_Read16(0xB8);
	MB = BMP180_Read16(0xBA);
	MC = BMP180_Read16(0xBC);
	MD = BMP180_Read16(0xBE);
}
/* USER CODE BEGIN PFP */
char temp_c[28];
char temp_f[28];
char hum[28];
char msg[30];
char msg1[30];
char msg2[30];
char msg3[30];
char msg4[30];
char IR[30];
char co2[30];
char temp_bmp[30];
char pressure_bmp[30];
char alttitude_bmp[30];
/* USER CODE END PFP */
void BMP180_StartTemperature(void) {
	uint8_t cmd = 0x2E;
	HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDR, 0xF4, 1, &cmd, 1, HAL_MAX_DELAY);
	HAL_Delay(5); // Wait for conversion
}

// Read uncompensated temperature
void BMP180_ReadUT(void) {
	UT = BMP180_ReadU16(0xF6);
}

// Start pressure conversion
void BMP180_StartPressure(void) {
	uint8_t cmd = 0x34;
	HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDR, 0xF4, 1, &cmd, 1, HAL_MAX_DELAY);
	HAL_Delay(8); // Wait for conversion
}

// Read uncompensated pressure
void BMP180_ReadUP(void) {
	uint8_t data[3];
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR, 0xF6, 1, data, 3, HAL_MAX_DELAY);
	UP = ((data[0] << 16) | (data[1] << 8) | data[2]) >> (8);
}

// Calculate altitude
float BMP180_CalculateAltitude(float pressure) {
	return 44330.0 * (1 - pow(pressure / 101325.0, 0.1903));
}
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
void microDelay (uint16_t delay){
	__HAL_TIM_SET_COUNTER(&htim1,0);
	while(__HAL_TIM_GET_COUNTER(&htim1)<delay);

}
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
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_I2C1_Init();


	MX_TIM1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	MPU6050_Init();
	/* USER CODE END 2 */
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);
	BMP180_ReadCalibrationData();
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{


		if(DHT22_Start())
		{
			//sensor read temperature and humidity in 8-bit form and store them in variables:
			RH1 = DHT22_Read();
			RH2 = DHT22_Read();

			TC1 = DHT22_Read();
			TC2 = DHT22_Read();

			//then sensor read the sum of humidity and temperature
			SUM = DHT22_Read();

			//variable to store the sum of humidity and temperature without sensor read
			CHECK = RH1 + RH2 + TC1 + TC2;

			//check if the sum of the sensor equal to the sum created by me
			if(CHECK == SUM)
			{

				RH = (int)((RH1<<8)|RH2)/10;                       //division by 10 to give me percentage
				snprintf(hum,sizeof(hum), ":%d",RH);      //store humidity in the "msg" variable
				//display humidity on the screen

				if(TC1>127)                                        //negative temperature
				{
					tCelsius = (int)(TC2/10)*(-1);
					snprintf(temp_c,sizeof(temp_c), ",%d", tCelsius);
					//display temp in  Celsius on the screen

				}
				else                                                            //positive temperature
				{
					tCelsius = (int)((TC1<<8)|TC2)/10;
					snprintf(temp_c,sizeof(temp_c), ",%d", tCelsius);
					//display temp in  Celsius on the screen

				}
				tFahrenheit = tCelsius * 9/5 + 32;
				snprintf(temp_f,sizeof(temp_f), ",%d", tFahrenheit);                                          //display temp in  Fahrenheit  on the screen

			}

			HAL_Delay(1000);
		}

		/* USER CODE END WHILE */
		MPU6050_Read_Calculate(&ax_mps2, &ay_mps2, &az_mps2, &pitch, &roll, &gx, &gy, &gz);
		if (ax_mps2<0)
		{
			ax_mps2=ax_mps2*(-1);
			snprintf(msg,sizeof(msg),"*-%u",ax_mps2);
		}
		else{
			snprintf(msg,sizeof(msg),"*%u",ax_mps2);
		}
		if (ay_mps2<0)
		{
			ay_mps2=ay_mps2*(-1);
			snprintf(msg1,sizeof(msg1),",-%u",ay_mps2);
		}
		else{
			snprintf(msg1,sizeof(msg1),",%u",ay_mps2);
		}
		if (az_mps2<0)
		{
			az_mps2=az_mps2*(-1);
			snprintf(msg2,sizeof(msg2),",-%u",az_mps2);
		}
		else{
			snprintf(msg2,sizeof(msg2),",%u",az_mps2);
		}
		if (pitch<0)
		{
			pitch=pitch*(-1);
			snprintf(msg3,sizeof(msg3),",-%d",pitch);
		}
		else{
			snprintf(msg3,sizeof(msg3),",%d",pitch);
		}
		if (roll<0)
		{
			roll=roll*(-1);
			snprintf(msg4,sizeof(msg4),",-%d",roll);
		}
		else{
			snprintf(msg4,sizeof(msg4),",%d",roll);
		}

		/* USER CODE BEGIN 3 */



		/* USER CODE END WHILE */
		HAL_ADC_PollForConversion(&hadc1, 1000);

		// Get the ADC value from the sensor
		readValue = HAL_ADC_GetValue(&hadc1);
		snprintf(IR,sizeof(IR),"/,%d",readValue);
		HAL_Delay(1000);

		// Poll for ADC conversion result
		HAL_ADC_PollForConversion(&hadc2, 1000);

		// Get the ADC value from the sensor
		readValue2 = HAL_ADC_GetValue(&hadc2);
		snprintf(co2,sizeof(co2),"@%d",readValue2);

		HAL_Delay(1000);

		BMP180_StartTemperature();
		BMP180_ReadUT();

		BMP180_StartPressure();
		BMP180_ReadUP();

		// Calculate true temperature
		int32_t X1 = ((UT - (int32_t)AC6) * (int32_t)AC5) >> 15;
		int32_t X2 = ((int32_t)MC << 11) / (X1 + (int32_t)MD);
		B5 = X1 + X2;
		temperature = ((B5 + 8) >> 4) / 10.0f;
		snprintf(temp_bmp,sizeof(temp_bmp), "%d", temperature);

		// Calculate true pressure
		int32_t B6 = B5 - 4000;
		X1 = ((int32_t)B2 * ((B6 * B6) >> 12)) >> 11;
		X2 = ((int32_t)AC2 * B6) >> 11;
		int32_t X3 = X1 + X2;
		int32_t B3 = (((int32_t)AC1 * 4 + X3) + 2) >> 2;

		X1 = ((int32_t)AC3 * B6) >> 13;
		X2 = ((B1 * ((B6 * B6) >> 12))) >> 16;
		X3 = ((X1 + X2) + 2) >> 2;

		uint32_t B4 = ((uint32_t)AC4 * (uint32_t)(X3 + 32768)) >> 15;
		uint32_t B7 = ((uint32_t)UP - B3) * 50000;

		if (B7 < 0x80000000) {
			pressure = (B7 * 2) / B4;
		} else {
			pressure = (B7 / B4) * 2;
		}

		X1 = (pressure >> 8) * (pressure >> 8);
		X1 = (X1 * 3038) >> 16;
		X2 = ((-7357) * (int32_t)pressure) >> 16;
		pressure = (pressure + ((X1 + X2 + 3791) >> 4));
		snprintf(pressure_bmp,sizeof(pressure_bmp), "#%d",pressure);

		// Calculate altitude
		altitude = 44330.0f * (1.0f - pow((float)pressure / 101325.0f, 0.1903f));
		snprintf(alttitude_bmp,sizeof(alttitude_bmp), ",%d",altitude);


		// Debugging in Live Expressions
		// __NOP();











		HAL_UART_Transmit(&huart1, (uint8_t *)&hum,strlen(hum), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t *)&temp_c,strlen(temp_c), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t *)&temp_f,strlen(temp_f), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t *)&msg, strlen(msg), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t *)&msg1, strlen(msg1), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t *)&msg2, strlen(msg2), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t *)&msg3, strlen(msg3), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t *)&msg4, strlen(msg4), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t *)&IR, strlen(IR), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t *)&co2, strlen(co2), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t *)&pressure_bmp,strlen(pressure_bmp), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t *)&alttitude_bmp,strlen(alttitude_bmp), HAL_MAX_DELAY);
		HAL_Delay(500);

		// Debugging in Live Expressions
		//__NOP();

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */




	/* USER CODE BEGIN 3 */


	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */


/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.ContinuousConvMode = ENABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}
/* USER CODE BEGIN I2C1_Init 2 */

/* USER CODE END I2C1_Init 2 */



/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 71;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

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
	huart1.Init.BaudRate = 9600;
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
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pin : PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
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
