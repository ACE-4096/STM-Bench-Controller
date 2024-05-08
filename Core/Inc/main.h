/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "lcd_io.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
// Channels
#define Vadj 0
#define TwelveV 1
#define FiveV 2
#define ThreeV3 3

// ADCs

	// Current
		// Vadj
		// C4
#define CurrentPort0 hadc2
#define CurrentPin0 ADC_CHANNEL_5

		// 12V
		// B2
#define CurrentPort1 hadc2
#define CurrentPin1 ADC_CHANNEL_12

		//5V
		// B0
#define CurrentPort2 hadc1
#define CurrentPin2 ADC_CHANNEL_15

		// 3V3
		// A7
#define CurrentPort3 hadc2
#define CurrentPin3 ADC_CHANNEL_4


	// Voltage
		// Vadj
		//B12
#define VoltagePort0 hadc1
#define VoltagePin0 ADC_CHANNEL_11

		// 12V
		// B1
#define VoltagePort1 hadc1
#define VoltagePin1 ADC_CHANNEL_12

		//5V
		// B14
#define VoltagePort2 hadc1
#define VoltagePin2 ADC_CHANNEL_5

		// 3V3
		// B11
#define VoltagePort3 hadc2
#define VoltagePin3 ADC_CHANNEL_14
// Buttons & Rotary Encoder


	// Control Relays
		// Vadj
#define RelayPort0 GPIOA
#define RelayPin0 9

		// 12V
#define RelayPort1 GPIOA
#define RelayPin1 11

		//5V
#define RelayPort2 GPIOC
#define RelayPin2 11

		// 3V3
#define RelayPort3 GPIOB
#define RelayPin3 4


// Button inputs
	// button 1
#define RelayPort0 GPIOA
#define RelayPin0 9

// button 2
#define RelayPort1 GPIOA
#define RelayPin1 11

// button 3
#define RelayPort2 GPIOC
#define RelayPin2 11

// button 4
#define RelayPort3 GPIOB
#define RelayPin3 4
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
