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

#define BOARD_T200_CPU_V01					1
#define BOARD_NUCLEO_G070RB					2

//#define BOARD_TYPE 							BOARD_T200_CPU_V01
#define BOARD_TYPE 							BOARD_NUCLEO_G070RB


#define SYSTEM_SOFTWARE_VERSION_MAJOR 		0
#define SYSTEM_SOFTWARE_VERSION_MINOR 		2
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



#define TEST1_GPIO_Port 						GPIOA
#define TEST1_Pin_mask							GPIO_PIN_0			//Arduino A0

#define TEST2_GPIO_Port 						GPIOB				//Arduino A3
#define TEST2_Pin_mask							GPIO_PIN_1

#define TEST3_GPIO_Port 						GPIOA				//Arduino D7
#define TEST3_Pin_mask							GPIO_PIN_8

#define TEST1_PIN_UP() 				HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin_mask, GPIO_PIN_SET)
#define TEST1_PIN_DOWN() 			HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin_mask, GPIO_PIN_RESET)
#define TEST1_PIN_TOGGLE()  	HAL_GPIO_TogglePin(TEST1_GPIO_Port, TEST1_Pin_mask);

#define TEST2_PIN_UP() 		HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin_mask, GPIO_PIN_SET)
#define TEST2_PIN_DOWN() 		HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin_mask, GPIO_PIN_RESET)
#define TEST2_PIN_TOGGLE()	HAL_GPIO_TogglePin(TEST2_GPIO_Port, TEST2_Pin_mask);

#define TEST3_PIN_UP() 		HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin_mask, GPIO_PIN_SET)
#define TEST3_PIN_DOWN() 		HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin_mask, GPIO_PIN_RESET)
#define TEST3_PIN_TOGGLE()	HAL_GPIO_TogglePin(TEST3_GPIO_Port, TEST3_Pin_mask);



#define SPI_SLAVE_IRQ_REQUEST_GPIO_Port			GPIOB
#define SPI_SLAVE_IRQ_REQUEST_Pin_mask 			GPIO_PIN_14			//Arduino D6

#define SPI_SLAVE_CHIP_SELECT_GPIO_Port			GPIOA
#define SPI_SLAVE_CHIP_SELECT_Pin_mask			GPIO_PIN_4			//Arduino D6





#if CONFIG_TEST_PINS_MAIN

#define MAIN_TEST1_PIN_UP() 		TEST1_PIN_UP()
#define MAIN_TEST1_PIN_DOWN() 		TEST1_PIN_DOWN()
#define MAIN_TEST1_PIN_TOGGLE()  	TEST1_PIN_TOGGLE()

#define MAIN_TEST2_PIN_UP() 		TEST2_PIN_UP()
#define MAIN_TEST2_PIN_DOWN() 		TEST2_PIN_DOWN()
#define MAIN_TEST2_PIN_TOGGLE()		TEST2_PIN_TOGGLE()

#define MAIN_TEST3_PIN_UP() 		TEST3_PIN_UP()
#define MAIN_TEST3_PIN_DOWN() 		TEST3_PIN_DOWN()
#define MAIN_TEST3_PIN_TOGGLE()		TEST3_PIN_TOGGLE()

#else /* CONFIG_TEST_PINS_MAIN */

#define MAIN_TEST1_PIN_UP()
#define MAIN_TEST1_PIN_DOWN()
#define MAIN_TEST1_PIN_TOGGLE()

#define MAIN_TEST2_PIN_UP()
#define MAIN_TEST2_PIN_DOWN()
#define MAIN_TEST2_PIN_TOGGLE()

#define MAIN_TEST3_PIN_UP()
#define MAIN_TEST3_PIN_DOWN()
#define MAIN_TEST3_PIN_TOGGLE()

#endif /* CONFIG_TEST_PINS_MAIN */

#if CONFIG_TEST_PINS_SPI_SLAVE

#define SPI_SLAVE_TEST1_PIN_UP() 		TEST1_PIN_UP()
#define SPI_SLAVE_TEST1_PIN_DOWN() 		TEST1_PIN_DOWN()
#define SPI_SLAVE_TEST1_PIN_TOGGLE()  	TEST1_PIN_TOGGLE()

#define SPI_SLAVE_TEST2_PIN_UP() 		TEST2_PIN_UP()
#define SPI_SLAVE_TEST2_PIN_DOWN() 		TEST2_PIN_DOWN()
#define SPI_SLAVE_TEST2_PIN_TOGGLE()  	TEST2_PIN_TOGGLE()

#define SPI_SLAVE_TEST3_PIN_UP() 		TEST3_PIN_UP()
#define SPI_SLAVE_TEST3_PIN_DOWN() 		TEST3_PIN_DOWN()
#define SPI_SLAVE_TEST3_PIN_TOGGLE()  	TEST3_PIN_TOGGLE()

#else /* CONFIG_TEST_PINS_SPI_SLAVE */

#define SPI_SLAVE_TEST1_PIN_UP()
#define SPI_SLAVE_TEST1_PIN_DOWN()
#define SPI_SLAVE_TEST1_PIN_TOGGLE()

#define SPI_SLAVE_TEST2_PIN_UP()
#define SPI_SLAVE_TEST2_PIN_DOWN()
#define SPI_SLAVE_TEST2_PIN_TOGGLE()

#define SPI_SLAVE_TEST3_PIN_UP()
#define SPI_SLAVE_TEST3_PIN_DOWN()
#define SPI_SLAVE_TEST3_PIN_TOGGLE()

#endif /* CONFIG_TEST_PINS_SPI_SLAVE */

#if CONFIG_TEST_PINS_SPI_SLAVE_SS_IRQ

#define SPI_SLAVE_SS_IRQ_TEST1_PIN_UP() 		TEST1_PIN_UP()
#define SPI_SLAVE_SS_IRQ_TEST1_PIN_DOWN() 		TEST1_PIN_DOWN()
#define SPI_SLAVE_SS_IRQ_TEST1_PIN_TOGGLE()  	TEST1_PIN_TOGGLE()

#define SPI_SLAVE_SS_IRQ_TEST2_PIN_UP() 		TEST2_PIN_UP()
#define SPI_SLAVE_SS_IRQ_TEST2_PIN_DOWN() 		TEST2_PIN_DOWN()
#define SPI_SLAVE_SS_IRQ_TEST2_PIN_TOGGLE()		TEST2_PIN_TOGGLE()

#define SPI_SLAVE_SS_IRQ_TEST3_PIN_UP() 		TEST3_PIN_UP()
#define SPI_SLAVE_SS_IRQ_TEST3_PIN_DOWN() 		TEST3_PIN_DOWN()
#define SPI_SLAVE_SS_IRQ_TEST3_PIN_TOGGLE()		TEST3_PIN_TOGGLE()

#else /* CONFIG_TEST_PINS_SPI_SLAVE_SS_IRQ */

#define SPI_SLAVE_SS_IRQ_TEST1_PIN_UP()
#define SPI_SLAVE_SS_IRQ_TEST1_PIN_DOWN()
#define SPI_SLAVE_SS_IRQ_TEST1_PIN_TOGGLE()

#define SPI_SLAVE_SS_IRQ_TEST2_PIN_UP()
#define SPI_SLAVE_SS_IRQ_TEST2_PIN_DOWN()
#define SPI_SLAVE_SS_IRQ_TEST2_PIN_TOGGLE()

#define SPI_SLAVE_SS_IRQ_TEST3_PIN_UP()
#define SPI_SLAVE_SS_IRQ_TEST3_PIN_DOWN()
#define SPI_SLAVE_SS_IRQ_TEST3_PIN_TOGGLE()

#endif /* CONFIG_TEST_PINS_SPI_SLAVE_SS_IRQ */



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
