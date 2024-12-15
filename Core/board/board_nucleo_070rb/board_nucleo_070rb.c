/*
 * board_nucleo_070rb.c
 *
 *  Created on: Jul 28, 2024
 *      Author: marek
 */

#include "main.h"
#if BOARD_TYPE == BOARD_NUCLEO_G070RB
#include "types.h"

#if defined(STM32G070xx)
#include "stm32g070xx.h"
#endif

#include "gpio/gpio.h"
#include "uart/uart.h"
#if 0
static const gpio_portpin_t board_nucleo_g070rb_digital_input_gpio[MSZ_T200_MODULES][DIGITAL_INPUTS_PER_MODULE] = {

	/* Module 0 */ {
	{GPIOC, 	2},
	{GPIOD, 	15},
	{GPIOD, 	14},
	{GPIOD, 	13},
	{GPIOB, 	6},
	{GPIOB, 	4},
	{GPIOD, 	2},
	{GPIOC, 	12},
	},

	/* Module 1 */ {
	{GPIOC, 	9},
	{GPIOB, 	11},
	{GPIOB, 	10},
	{GPIOB, 	3},
	{GPIOA, 	0},
	{GPIOD, 	12},
	{GPIOC, 	15},
	{GPIOD, 	8},
	},

	/* Module 2 */ {
	{GPIOA, 	9},
	{GPIOC, 	5},
	{GPIOA, 	12},
	{GPIOC, 	2},
	{GPIOC, 	3},
	{GPIOC, 	4},
	{GPIOC, 	5},
	{GPIOA, 	2},
	},

	/* Module 3 */ {
	{GPIOD, 	5},
	{GPIOD, 	6},
	{GPIOD, 	4},
	{GPIOB, 	2},
	{GPIOA, 	1},
	{GPIOC, 	8},
	{GPIOF, 	9},
	{GPIOA, 	3},
	}
};
#endif

msz_rc_t board_nucleo_070rb_digital_input_init(const msz_t200_module_no_t module_no, const digital_in_no_t digital_in_no, const bool enable) {

	msz_rc_t								rc = MSZ_RC_OK;
	GPIO_InitTypeDef 						GPIO_InitStruct = {0};

	if ((module_no == 0) && (digital_in_no == 0)) {
		if (enable) {
			GPIO_InitStruct.Pin = BUTTON_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);
		} else {
			HAL_GPIO_DeInit(BUTTON_GPIO_Port, BUTTON_Pin);
		}
	}

	return rc;
}

bool board_nucleo_070rb_digital_in_get_state(const msz_t200_module_no_t module_no, const digital_in_no_t digital_in_no) {

	bool									state = false;

	if ((module_no == 0) && (digital_in_no == 0)) {
		state = (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, GPIO_PIN_13) == GPIO_PIN_RESET);
	}

	return state;
}




msz_rc_t board_nucleo_070rb_digital_output_init(const msz_t200_module_no_t module_no, const digital_out_no_t digital_out_no, const bool enable) {

	msz_rc_t								rc = MSZ_RC_OK;
	GPIO_InitTypeDef 						GPIO_InitStruct = {0};

	if ((module_no == 2) && (digital_out_no == 0)) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		if (enable) {
			GPIO_InitStruct.Pin = LED_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
		} else {
			HAL_GPIO_DeInit(LED_GPIO_Port, LED_Pin);
		}
	}

	return rc;
}

msz_rc_t board_nucleo_070rb_set_digital_output_state(const msz_t200_module_no_t module_no, const digital_out_no_t digital_out_no, const bool new_state) {

	msz_rc_t								rc = MSZ_RC_OK;

	if ((module_no == 2) && (digital_out_no == 0)) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, new_state);
	}

	return rc;
}









const uart_msz_dev_t cli_trace_uart_dev = {
	.usart = USART2,
	.tx_gpio_pin = {GPIOA, 2, 1},
	.rx_gpio_pin = {GPIOA, 3, 1},
};

#endif /* BOARD_TYPE == BOARD_T200_CPU_V01 */

