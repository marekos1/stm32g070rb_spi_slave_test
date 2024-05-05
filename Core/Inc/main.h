/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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


#include "types.h"
#include "config.h"



#define SYSTEM_HARDWARE_REVISON				1

#define SYSTEM_SOFTWARE_VERSION_MAJOR 		0
#define SYSTEM_SOFTWARE_VERSION_MINOR 		1
#define SYSTEM_SOFTWARE_VERSION_REVISION	0
#define SYSTEM_SOFTWARE_BUILD_NUMBER		0







uint32_t system_ms_time(void);

void system_no_os_waitms(uint32_t delay_ms);

void systick_tick(void);








/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

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
#define LED_Pin 			GPIO_PIN_5
#define LED_GPIO_Port 		GPIOA

#define BUTTON_Pin 			GPIO_PIN_13
#define BUTTON_GPIO_Port 	GPIOC

#define TEST1_GPIO_Port GPIOA
#define TEST1_Pin GPIO_PIN_0			//Arduino A0

#define TEST2_GPIO_Port GPIOB			//Arduino A3
#define TEST2_Pin GPIO_PIN_1

#define TEST3_GPIO_Port GPIOA			//Arduino D7
#define TEST3_Pin		GPIO_PIN_8

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
