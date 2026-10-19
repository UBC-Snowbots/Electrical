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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//[**********************Voltage/Current ADC****************************]//
/* Raw ADC readings */
uint32_t adc_18v_voltage_raw = 0;
uint32_t adc_18v_current_raw = 0;
uint32_t adc_12v_voltage_raw = 0;
uint32_t adc_12v_current_raw = 0;

/* ADC-converted voltages */
float adc_18v_voltage = 0.0f;
float adc_18v_current = 0.0f;
float adc_12v_voltage = 0.0f;
float adc_12v_current = 0.0f;

/* Calculated actual rail measurements */
float voltage_18v = 0.0f;
float current_18v = 0.0f;
float voltage_12v = 0.0f;
float current_12v = 0.0f;


//[**********************UART TX****************************]//
uint8_t uart_tx_packet[10];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */


//[**********************Voltage/Current ADC****************************]//
uint32_t ADC_Read_Channel(uint32_t channel);

float Calculate_18V_Voltage(float adc_voltage);
float Calculate_12V_Voltage(float adc_voltage);

float Calculate_Current(float adc_voltage);

//[**********************UART TX****************************]//

void UART_Send_Measurements(void);

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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  //[**********************Voltage/Current ADC****************************]//

	adc_18v_voltage_raw = ADC_Read_Channel(ADC_CHANNEL_2);

	adc_18v_voltage =
		((float)adc_18v_voltage_raw / 4095.0f) * 3.3f;

	voltage_18v = Calculate_18V_Voltage(adc_18v_voltage);

    adc_18v_current_raw = ADC_Read_Channel(ADC_CHANNEL_4);

    adc_18v_current =
        ((float)adc_18v_current_raw / 4095.0f) * 3.3f;

    current_18v = Calculate_Current(adc_18v_current);

    adc_12v_voltage_raw = ADC_Read_Channel(ADC_CHANNEL_8);

    adc_12v_voltage =
        ((float)adc_12v_voltage_raw / 4095.0f) * 3.3f;

    voltage_12v = Calculate_12V_Voltage(adc_12v_voltage);

    adc_12v_current_raw = ADC_Read_Channel(ADC_CHANNEL_10);

    adc_12v_current =
        ((float)adc_12v_current_raw / 4095.0f) * 3.3f;

    current_12v = Calculate_Current(adc_12v_current);

    //[**********************UART TX****************************]//
    UART_Send_Measurements();

    HAL_Delay(100);

    /* USER CODE END 3 */
  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PGOOD_12V_Pin PGOOD_18V_Pin */
  GPIO_InitStruct.Pin = PGOOD_12V_Pin|PGOOD_18V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DEBUG_LED_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief Read a single ADC channel
  * @param channel ADC channel to read
  * @retval Raw 12-bit ADC value (0-4095)
  */
uint32_t ADC_Read_Channel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  uint32_t adc_value = 0;

  /* Configure the ADC channel */
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start ADC conversion */
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Wait for conversion to finish */
  if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK)
  {
    Error_Handler();
  }

  /* Get ADC result */
  adc_value = HAL_ADC_GetValue(&hadc1);

  /* Stop ADC */
  if (HAL_ADC_Stop(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  return adc_value;
}


/**
  * @brief Calculate actual 18V rail voltage from ADC voltage
  * @param adc_voltage Voltage measured at PA2
  * @retval Actual 18V rail voltage
  *
  * Voltage divider:
  *
  * Vout = Vin * R2 / (R1 + R2)
  *
  * R1 = 10k
  * R2 = 2k
  *
  * Therefore:
  *
  * Vin = Vout * (10k + 2k) / 2k
  * Vin = Vout * 6
  */
float Calculate_18V_Voltage(float adc_voltage)
{
  return adc_voltage * 6.0f;
}


/**
  * @brief Calculate actual 12V rail voltage from ADC voltage
  * @param adc_voltage Voltage measured at PB0
  * @retval Actual 12V rail voltage
  *
  * Voltage divider:
  *
  * R1 = 6.2k
  * R2 = 2k
  *
  * Vin = Vout * (6.2k + 2k) / 2k
  * Vin = Vout * 4.1
  */
float Calculate_12V_Voltage(float adc_voltage)
{
  return adc_voltage * 4.1f;
}


/**
  * @brief Calculate current from TMCS1108 output voltage
  * @param adc_voltage Hall sensor output voltage
  * @retval Measured current in Amps
  *
  * TMCS1108:
  *
  * Zero-current output = 0.5 * Vs
  * Sensitivity = 50 mV/A = 0.05 V/A
  *
  * Assuming Vs = 3.3V:
  *
  * Vzero = 1.65V
  *
  * I = (Vout - 1.65V) / 0.05V/A
  */
float Calculate_Current(float adc_voltage)
{
  const float sensor_zero_voltage = 1.65f;
  const float sensor_sensitivity = 0.05f;

  return (adc_voltage - sensor_zero_voltage) / sensor_sensitivity;
}

/**
  * @brief Send a voltage/current message over UART1
  * @param None
  * @retval None
  */
void UART_Send_Measurements(void)
{
    uint16_t v18 = (uint16_t)(voltage_18v * 100.0f);
    int16_t  i18 = (int16_t)(current_18v * 100.0f);
    uint16_t v12 = (uint16_t)(voltage_12v * 100.0f);
    int16_t  i12 = (int16_t)(current_12v * 100.0f);

    /* Packet header */
    uart_tx_packet[0] = 0xAA;
    uart_tx_packet[1] = 0x55;

    uart_tx_packet[2] = (uint8_t)(v18 >> 8);
    uart_tx_packet[3] = (uint8_t)v18;

    uart_tx_packet[4] = (uint8_t)((uint16_t)i18 >> 8);
    uart_tx_packet[5] = (uint8_t)i18;

    uart_tx_packet[6] = (uint8_t)(v12 >> 8);
    uart_tx_packet[7] = (uint8_t)v12;

    uart_tx_packet[8] = (uint8_t)((uint16_t)i12 >> 8);
    uart_tx_packet[9] = (uint8_t)i12;

    HAL_UART_Transmit(
        &huart1,
        uart_tx_packet,
        sizeof(uart_tx_packet),
        HAL_MAX_DELAY
    );
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
