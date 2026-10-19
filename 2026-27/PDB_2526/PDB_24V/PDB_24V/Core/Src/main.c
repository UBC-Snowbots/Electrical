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
#include <string.h>
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
ADC_HandleTypeDef hadc2;

FDCAN_HandleTypeDef hfdcan2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

//[**********************24V Voltage/Current ADC****************************]//

/* 24V voltage */
uint32_t adc_24v_voltage_raw = 0;
float adc_24v_voltage = 0.0f;
float voltage_24v = 0.0f;


/* Arm current */
uint32_t adc_arm_current_raw = 0;
float adc_arm_current_voltage = 0.0f;
float current_arm = 0.0f;


/* Chassis current */
uint32_t adc_chassis_current_raw = 0;
float adc_chassis_current_voltage = 0.0f;
float current_chassis = 0.0f;


/* Misc current */
uint32_t adc_misc_current_raw = 0;
float adc_misc_current_voltage = 0.0f;
float current_misc = 0.0f;

//[**********************UART4 RX****************************]//


float voltage_18v_received = 0.0f;
float current_18v_received = 0.0f;
float voltage_12v_received = 0.0f;
float current_12v_received = 0.0f;

uint8_t uart4_message_received = 0;

HAL_StatusTypeDef uart4_status = HAL_OK;
uint32_t uart4_error = 0;

uint8_t uart4_rx_byte = 0;
uint8_t uart4_rx_payload[8];

FDCAN_TxHeaderTypeDef can_tx_header;
uint8_t can_tx_data[8];

HAL_StatusTypeDef can_tx_status = HAL_OK;
uint32_t can_tx_error = 0;
uint32_t can_tx_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);


/* USER CODE BEGIN PFP */


//[**********************24V Voltage/Current ADC****************************]//

uint32_t ADC1_Read_Channel(uint32_t channel);

uint32_t ADC2_Read_Channel(uint32_t channel);

float Calculate_24V_Voltage(float adc_voltage);

float Calculate_Current(float adc_voltage);

//[**********************UART5 RX****************************]//

void UART4_Receive_Measurements(void);

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
  MX_FDCAN2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USB_PCD_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  HAL_GPIO_WritePin(
      CAN_STB_GPIO_Port,
      CAN_STB_Pin,
      GPIO_PIN_RESET
  );

  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
      Error_Handler();
  }

  can_tx_header.Identifier = 0x02000001;
  can_tx_header.IdType = FDCAN_EXTENDED_ID;
  can_tx_header.TxFrameType = FDCAN_DATA_FRAME;
  can_tx_header.DataLength = FDCAN_DLC_BYTES_8;
  can_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  can_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  can_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  can_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  can_tx_header.MessageMarker = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);

	  //[**********************UART5 RX****************************]//

	  UART4_Receive_Measurements();

	  //[**********************24V Voltage/Current ADC****************************]//


	    /*
	     * ---------------------------------------------------------
	     * 24V RAIL VOLTAGE
	     *
	     * PA1 / ADC1_IN2
	     *
	     * Voltage divider:
	     * 18k upper
	     * 2k lower
	     * ---------------------------------------------------------
	     */

	    adc_24v_voltage_raw = ADC1_Read_Channel(ADC_CHANNEL_2);

	    adc_24v_voltage =
	        ((float)adc_24v_voltage_raw / 4095.0f) * 3.35f;

	    voltage_24v = Calculate_24V_Voltage(adc_24v_voltage);

	    /*
	     * ---------------------------------------------------------
	     * ARM CURRENT
	     *
	     * PC5 / ADC2_IN11
	     *
	     * TMCS1123B1
	     * Zero current = 1.65V
	     * Sensitivity = 25mV/A
	     * ---------------------------------------------------------
	     */

	    adc_arm_current_raw = ADC2_Read_Channel(ADC_CHANNEL_11);

	    adc_arm_current_voltage =
	        ((float)adc_arm_current_raw / 4095.0f) * 3.35f;

	    current_arm = Calculate_Current(adc_arm_current_voltage);


	    /*
	     * ---------------------------------------------------------
	     * CHASSIS CURRENT
	     *
	     * PB15 / ADC2_IN15
	     *
	     * TMCS1123B1
	     * Zero current = 1.65V
	     * Sensitivity = 25mV/A
	     * ---------------------------------------------------------
	     */

	    adc_chassis_current_raw = ADC2_Read_Channel(ADC_CHANNEL_15);

	    adc_chassis_current_voltage =
	        ((float)adc_chassis_current_raw / 4095.0f) * 3.35f;

	    current_chassis = Calculate_Current(adc_chassis_current_voltage);


	    /*
	     * ---------------------------------------------------------
	     * MISC CURRENT
	     *
	     * PA5 / ADC2_IN13
	     *
	     * TMCS1123B1
	     * Zero current = 1.65V
	     * Sensitivity = 25mV/A
	     * ---------------------------------------------------------
	     */

	    adc_misc_current_raw = ADC2_Read_Channel(ADC_CHANNEL_13);

	    adc_misc_current_voltage =
	        ((float)adc_misc_current_raw / 4095.0f) * 3.35f;

	    current_misc = Calculate_Current(adc_misc_current_voltage);


	    can_tx_data[0] = 0x11;
	    can_tx_data[1] = 0x22;
	    can_tx_data[2] = 0x33;
	    can_tx_data[3] = 0x44;
	    can_tx_data[4] = 0x55;
	    can_tx_data[5] = 0x66;
	    can_tx_data[6] = 0x77;
	    can_tx_data[7] = 0x88;

	    can_tx_status = HAL_FDCAN_AddMessageToTxFifoQ(
	        &hfdcan2,
	        &can_tx_header,
	        can_tx_data
	    );

	    if (can_tx_status == HAL_OK)
	    {
	        can_tx_count++;
	    }
	    else
	    {
	        can_tx_error = HAL_FDCAN_GetError(&hfdcan2);
	    }

	    HAL_Delay(500);


	    /*
	     * Wait before starting the next measurement cycle.
	     */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 2;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 13;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CAN_STB_Pin|DEBUG_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MISC_ALERT_Pin VBUS_SENSE_Pin */
  GPIO_InitStruct.Pin = MISC_ALERT_Pin|VBUS_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARM_ALERT_Pin CHASSIS_ALERT_Pin */
  GPIO_InitStruct.Pin = ARM_ALERT_Pin|CHASSIS_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN_STB_Pin DEBUG_LED_Pin */
  GPIO_InitStruct.Pin = CAN_STB_Pin|DEBUG_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief Read a single ADC1 channel
  * @param channel ADC1 channel to read
  * @retval Raw 12-bit ADC value (0-4095)
  */
uint32_t ADC1_Read_Channel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  uint32_t adc_value = 0;

  /* Configure ADC1 channel */
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start conversion */
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Wait for conversion */
  if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK)
  {
    Error_Handler();
  }

  /* Read result */
  adc_value = HAL_ADC_GetValue(&hadc1);

  /* Stop ADC */
  if (HAL_ADC_Stop(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  return adc_value;
}

/**
  * @brief Read a single ADC2 channel
  * @param channel ADC2 channel to read
  * @retval Raw 12-bit ADC value (0-4095)
  */
uint32_t ADC2_Read_Channel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  uint32_t adc_value = 0;

  /* Configure ADC2 channel */
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start conversion */
  if (HAL_ADC_Start(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /* Wait for conversion */
  if (HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY) != HAL_OK)
  {
    Error_Handler();
  }

  /* Read result */
  adc_value = HAL_ADC_GetValue(&hadc2);

  /* Stop ADC */
  if (HAL_ADC_Stop(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  return adc_value;
}

/**
  * @brief Calculate actual 24V rail voltage from ADC voltage
  * @param adc_voltage Voltage measured at PA1
  * @retval Actual 24V rail voltage
  *
  * Voltage divider:
  *
  * R1 = 18k
  * R2 = 2k
  *
  * Vout = Vin * R2 / (R1 + R2)
  *
  * Vin = Vout * (R1 + R2) / R2
  *
  * Vin = Vout * (18k + 2k) / 2k
  *
  * Vin = Vout * 10
  */
float Calculate_24V_Voltage(float adc_voltage)
{
  return adc_voltage * 10.0f;
}

/**
  * @brief Calculate current from TMCS1123 output voltage
  * @param adc_voltage Hall sensor output voltage
  * @retval Measured current in Amps
  *
  * TMCS1123B1:
  *
  * Zero-current output = 1.65V
  * Sensitivity = 25mV/A = 0.025V/A
  *
  * I = (Vout - 1.65V) / 0.025V/A
  */
float Calculate_Current(float adc_voltage)
{
  const float sensor_zero_voltage = 1.65f;
  const float sensor_sensitivity = 0.025f;

  return (adc_voltage - sensor_zero_voltage) / sensor_sensitivity;
}


/**
  * @brief Receive a voltage/current  from the 12V 18V board over UART4
  * @param None
  * @retval None
  */
void UART4_Receive_Measurements(void)
{
    uint8_t second_header_byte;
    uint16_t v18_raw;
    int16_t i18_raw;
    uint16_t v12_raw;
    int16_t i12_raw;

    uart4_message_received = 0;

    /*
     * Search for packet header:
     * 0xAA, 0x55
     */
    uart4_status = HAL_UART_Receive(
        &huart4,
        &uart4_rx_byte,
        1,
        500
    );

    if (uart4_status != HAL_OK)
    {
        uart4_error = HAL_UART_GetError(&huart4);
        return;
    }

    if (uart4_rx_byte != 0xAA)
    {
        return;
    }

    uart4_status = HAL_UART_Receive(
        &huart4,
        &second_header_byte,
        1,
        50
    );

    if ((uart4_status != HAL_OK) || (second_header_byte != 0x55))
    {
        uart4_error = HAL_UART_GetError(&huart4);
        return;
    }

    /*
     * Receive the eight measurement bytes.
     */
    uart4_status = HAL_UART_Receive(
        &huart4,
        uart4_rx_payload,
        sizeof(uart4_rx_payload),
        50
    );

    uart4_error = HAL_UART_GetError(&huart4);

    if (uart4_status != HAL_OK)
    {
        return;
    }

    v18_raw =
        ((uint16_t)uart4_rx_payload[0] << 8) |
         uart4_rx_payload[1];

    i18_raw =
        (int16_t)(((uint16_t)uart4_rx_payload[2] << 8) |
                   uart4_rx_payload[3]);

    v12_raw =
        ((uint16_t)uart4_rx_payload[4] << 8) |
         uart4_rx_payload[5];

    i12_raw =
        (int16_t)(((uint16_t)uart4_rx_payload[6] << 8) |
                   uart4_rx_payload[7]);

    voltage_18v_received = (float)v18_raw / 100.0f;
    current_18v_received = (float)i18_raw / 100.0f;
    voltage_12v_received = (float)v12_raw / 100.0f;
    current_12v_received = (float)i12_raw / 100.0f;

    uart4_message_received = 1;
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
