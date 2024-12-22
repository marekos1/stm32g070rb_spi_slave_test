/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdint.h>

#include "main.h"
#include "trace/trace.h"

#include "slave_regs/slave_regs.h"
#include "main_slave_regs.h"

#include "digital_in/digital_in_api.h"

#include "gpio/gpio.h"

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
CRC_HandleTypeDef hcrc;





/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static volatile uint32_t system_1ms_timer_tick_counter;

void systick_tick(void) {

	system_1ms_timer_tick_counter++;
}

uint32_t system_ms_time(void) {

	return system_1ms_timer_tick_counter;
}





int main(void) {
	/* USER CODE BEGIN 1 */

	uint32_t 								led_ctr, current_1ms_ctr_tick, test_ctr, irq_req_ctr;
	bool									connect = false, irq_req_state = false;
	uint32_t								_1sec_tick_ctr;


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
	MX_CRC_Init();

//	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
//	HAL_UART_Transmit(&huart2, (const uint8_t*) enter_txt, strlen(enter_txt), 1000);



	trace_cli_init();
	T_DG_MAIN("Enter");

	led_ctr = 0;
	test_ctr = 0;
	irq_req_ctr = 0;
	current_1ms_ctr_tick = 0;
	slave_registers_init();



	_1sec_tick_ctr = 1000;
	main_group_slave_status_set(true);
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		connect = slave_registers_poll(current_1ms_ctr_tick);
		current_1ms_ctr_tick++;

		if (connect && (++led_ctr > 10)) {
			led_ctr = 0;
		} else {
			led_ctr = 0;
		}

		digital_in_poll();

		if (system_1ms_timer_tick_counter > _1sec_tick_ctr) {
			_1sec_tick_ctr = system_1ms_timer_tick_counter;
			_1sec_tick_ctr += 1000;
			T_DG_MAIN("1. led_ctr: %12u", test_ctr++);
			if (++irq_req_ctr >= 10) {
				irq_req_ctr = 0;
				if (irq_req_state) {
					irq_req_state = false;
				} else {
					irq_req_state = true;
				}
			}
		}
	}
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
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
	hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
	hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
	hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
	hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}



/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {

	gpio_pin_mask_set_output_state(TEST1_GPIO_Port, TEST1_Pin_mask, false);
	gpio_pin_mask_set_output_state(TEST2_GPIO_Port, TEST2_Pin_mask, false);
	gpio_pin_mask_set_output_state(TEST3_GPIO_Port, TEST3_Pin_mask, false);

	gpio_init_pin_mask_as_output(TEST1_GPIO_Port, TEST1_Pin_mask, false, GPIO_SPEED_VERY_HIGH, GPIO_NO_PULL_UP_NO_PULL_DOWN);
	gpio_init_pin_mask_as_output(TEST2_GPIO_Port, TEST2_Pin_mask, false, GPIO_SPEED_VERY_HIGH, GPIO_NO_PULL_UP_NO_PULL_DOWN);
	gpio_init_pin_mask_as_output(TEST3_GPIO_Port, TEST3_Pin_mask, false, GPIO_SPEED_VERY_HIGH, GPIO_NO_PULL_UP_NO_PULL_DOWN);
}

/* USER CODE BEGIN 4 */

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

