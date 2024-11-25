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


#if 0

#define MAIN_TEST1_PIN_UP() 		HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_SET)
#define MAIN_TEST1_PIN_DOWN() 		HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_RESET)
#define MAIN_TEST1_PIN_TOGGLE()  	HAL_GPIO_TogglePin(TEST1_GPIO_Port, TEST1_Pin);

#define MAIN_TEST2_PIN_UP() 		HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, GPIO_PIN_SET)
#define MAIN_TEST2_PIN_DOWN() 		HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, GPIO_PIN_RESET)
#define MAIN_TEST2_PIN_TOGGLE()		HAL_GPIO_TogglePin(TEST2_GPIO_Port, TEST2_Pin);

#define MAIN_TEST3_PIN_UP() 		HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_SET)
#define MAIN_TEST3_PIN_DOWN() 		HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_RESET)
#define MAIN_TEST3_PIN_TOGGLE()		HAL_GPIO_TogglePin(TEST3_GPIO_Port, TEST3_Pin);

#else

#define MAIN_TEST1_PIN_UP()
#define MAIN_TEST1_PIN_DOWN()

#define MAIN_TEST2_PIN_UP()
#define MAIN_TEST2_PIN_DOWN()

#define MAIN_TEST3_PIN_UP()
#define MAIN_TEST3_PIN_DOWN()

#endif


int main(void) {
	/* USER CODE BEGIN 1 */

	uint32_t 								led_ctr, current_1ms_ctr_tick, test_ctr, irq_req_ctr;
	bool									connect = false, irq_req_state = false;
	bool									input_state = false, prev_input_state = false;

	uint32_t		digital_in_ctr, _1sec_tick_ctr;


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




	/*
	for (led_ctr = 0; led_ctr < 100; led_ctr++) {
		TEST3_PIN_UP();
		//TEST2_PIN_DOWN();
		HAL_Delay(10);
		TEST3_PIN_DOWN();
//		TEST2_PIN_UP();
		HAL_Delay(10);
	}
	*/

	led_ctr = 0;
	test_ctr = 0;
	irq_req_ctr = 0;
	current_1ms_ctr_tick = 0;
	slave_registers_init();

	digital_in_ctr = 0;
	/* USER CODE END 2 */



	MAIN_TEST1_PIN_UP();
	MAIN_TEST2_PIN_UP();
	MAIN_TEST3_PIN_UP();

	MAIN_TEST1_PIN_DOWN();
	MAIN_TEST2_PIN_DOWN();
	MAIN_TEST3_PIN_DOWN();


	_1sec_tick_ctr = 1000;
	main_group_slave_status_set(true);
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

	//	uart_proces();
#if 1

		connect = slave_registers_poll(current_1ms_ctr_tick);
		current_1ms_ctr_tick++;



#if 0
		if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, GPIO_PIN_13) == GPIO_PIN_SET) {

			main_group_slave_status_digital_in_state_set(0, 0, 0, false);
		} else {

			main_group_slave_status_digital_in_state_set(0, 0, 0, true);
		}
#else
		input_state = (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, GPIO_PIN_13) == GPIO_PIN_RESET);

		if (input_state != prev_input_state) {
			main_group_slave_status_digital_in_state_set(0, 0, 0, input_state);
			prev_input_state = input_state;
		}
#endif

#else
		spi_slave_init();
#endif
		if (connect && (++led_ctr > 10)) {
			led_ctr = 0;
		} else {
			led_ctr = 0;
		}
		/* USER CODE BEGIN 3 */

		if (++digital_in_ctr >= 10) {
			digital_in_ctr = 0;
			digital_in_poll();
		}

		if (system_1ms_timer_tick_counter > _1sec_tick_ctr) {
			_1sec_tick_ctr = system_1ms_timer_tick_counter;
			_1sec_tick_ctr += 1000;
			T_DG_MAIN("1. led_ctr: %12u", test_ctr++);
	//		T_DG_MAIN("2. led_ctr: %12u", test_ctr++);
	//		T_DG_MAIN("3. led_ctr: %12u", test_ctr++);
			if (++irq_req_ctr >= 10) {
				irq_req_ctr = 0;
				if (irq_req_state) {
					irq_req_state = false;
				} else {
					irq_req_state = true;
				}
			//	T_DG_MAIN("IRQ request state: %u", irq_req_state);
			//	HAL_GPIO_WritePin(IRQ_REQUEST_GPIO_Port, IRQ_REQUEST_Pin, irq_req_state ? GPIO_PIN_RESET : GPIO_PIN_SET);
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

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_RESET);


	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = TEST1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(TEST1_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = TEST2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(TEST2_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = TEST3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(TEST3_GPIO_Port, &GPIO_InitStruct);


	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);



	HAL_GPIO_WritePin(IRQ_REQUEST_GPIO_Port, IRQ_REQUEST_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = IRQ_REQUEST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(IRQ_REQUEST_GPIO_Port, &GPIO_InitStruct);

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

