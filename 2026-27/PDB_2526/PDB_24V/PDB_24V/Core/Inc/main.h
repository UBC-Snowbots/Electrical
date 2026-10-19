/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SENSE_24V_Pin GPIO_PIN_1
#define SENSE_24V_GPIO_Port GPIOA
#define MISC_ALERT_Pin GPIO_PIN_4
#define MISC_ALERT_GPIO_Port GPIOA
#define MISC_SENSE_Pin GPIO_PIN_5
#define MISC_SENSE_GPIO_Port GPIOA
#define ARM_ALERT_Pin GPIO_PIN_4
#define ARM_ALERT_GPIO_Port GPIOC
#define ARM_SENSE_Pin GPIO_PIN_5
#define ARM_SENSE_GPIO_Port GPIOC
#define CAN_STB_Pin GPIO_PIN_14
#define CAN_STB_GPIO_Port GPIOB
#define CHASSIS_SENSE_Pin GPIO_PIN_15
#define CHASSIS_SENSE_GPIO_Port GPIOB
#define CHASSIS_ALERT_Pin GPIO_PIN_7
#define CHASSIS_ALERT_GPIO_Port GPIOC
#define VBUS_SENSE_Pin GPIO_PIN_9
#define VBUS_SENSE_GPIO_Port GPIOA
#define DEBUG_LED_Pin GPIO_PIN_5
#define DEBUG_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
