/*
 * board_T200_cpu_v01.c
 *
 *  Created on: Jul 27, 2024
 *      Author: marek
 */

#include "main.h"
#if BOARD_TYPE == BOARD_T200_CPU_V01
#include "types.h"

#if defined(STM32G431xx)
#include "stm32g4xx.h"
#endif

#include "gpio/gpio.h"

/*
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
*/


static const gpio_portpin_t board_T200_cpu_v01_digital_input_gpio[MSZ_T200_MODULES][DIGITAL_INPUTS_PER_MODULE] = {

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
	{GPIOE, 	9},
	{GPIOB, 	11},
	{GPIOB, 	10},
	{GPIOB, 	3},
	{GPIOA, 	0},
	{GPIOD, 	12},
	{GPIOE, 	15},
	{GPIOD, 	8},
	},

	/* Module 2 */ {
	{GPIOA, 	9},
	{GPIOC, 	5},
	{GPIOA, 	12},
	{GPIOE, 	2},
	{GPIOE, 	3},
	{GPIOE, 	4},
	{GPIOE, 	5},
	{GPIOA, 	2},
	},

	/* Module 3 */ {
	{GPIOD, 	5},
	{GPIOD, 	6},
	{GPIOD, 	4},
	{GPIOB, 	2},
	{GPIOA, 	1},
	{GPIOE, 	8},
	{GPIOF, 	9},
	{GPIOA, 	3},
	}
};


msz_rc_t board_T200_cpu_v01_init_digital_input_state(const msz_t200_module_no_t module_no, const digital_in_no_t digital_in_no, const BOOL enable) {

	msz_rc_t								rc = MSZ_RC_OK;
	uint16_t								gpio_pin_mask;

	gpio_pin_mask = 1 << board_T200_cpu_v01_digital_input_gpio[module_no][digital_in_no].pin;
	if (enable) {
		rc = gpio_init_input(board_T200_cpu_v01_digital_input_gpio[module_no][digital_in_no].port, gpio_pin_mask, GPIO_SPEED_LOW, GPIO_PULL_UP);
	} else {
		rc = gpio_init_default(board_T200_cpu_v01_digital_input_gpio[module_no][digital_in_no].port, gpio_pin_mask);
	}

	return rc;
}

BOOL board_T200_cpu_v01_read_digital_input_state(const msz_t200_module_no_t module_no, const digital_in_no_t digital_in_no) {

	return gpio_get_input_state(board_T200_cpu_v01_digital_input_gpio[module_no][digital_in_no].port, board_T200_cpu_v01_digital_input_gpio[module_no][digital_in_no].pin);
}

#endif /* BOARD_TYPE == BOARD_T200_CPU_V01 */
