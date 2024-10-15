/*
 * gpio.h
 *
 *  Created on: 6 gru 2022
 *      Author: mszpakowski
 */

#ifndef DEV_GPIO_GPIO_H_
#define DEV_GPIO_GPIO_H_

#include "types.h"

#if defined(STM32F0)
#include "stm32f0xx.h"
#elif defined(STM32G431xx)
#include "stm32g4xx.h"
#elif defined(STM32G070xx)
#include "stm32g070xx.h"
#endif


typedef struct {
	GPIO_TypeDef 							*gpio_port;
	uint8_t 								gpio_pin;
	uint8_t 								alt_func;
} gpio_dev_t;

typedef struct {
	GPIO_TypeDef*							port;
	uint8_t									pin;
} gpio_portpin_t;

typedef enum {
	GPIO_SPEED_LOW = 0,
	GPIO_SPEED_MEDIUM,
	GPIO_SPEED_HIGH,
	GPIO_SPEED_VERY_HIGH
} gpio_speed_t;

typedef enum {
	GPIO_NO_PULL_UP_NO_PULL_DOWN = 0,
	GPIO_PULL_UP,
	GPIO_PULL_DOWN,
} gpio_pullup_t;

typedef enum {
	GPIO_TYPE_INPUT = 0,
	GPIO_TYPE_OUTPUT,
	GPIO_TYPE_ALTERNATE,
	GPIO_TYPE_DISABLE
} gpio_type_t;


void gpio_set_output_state(GPIO_TypeDef *gpio_port, uint8_t gpio_pin, const BOOL state);

void gpio_toggle_output_state(GPIO_TypeDef *gpio_port, uint8_t gpio_pin);

BOOL gpio_get_input_state(GPIO_TypeDef *gpio_port, uint8_t gpio_pin);

msz_rc_t gpio_init_output(GPIO_TypeDef *gpio_port, uint16_t gpio_pin_mask, BOOL open_drain, gpio_speed_t speed, gpio_pullup_t pullup);

msz_rc_t gpio_init_input(GPIO_TypeDef *gpio_port, uint16_t gpio_pin_mask, gpio_speed_t speed, gpio_pullup_t pullup);

msz_rc_t gpio_init_alt(GPIO_TypeDef *gpio_port, uint16_t gpio_pin_mask, BOOL open_drain, gpio_speed_t speed, gpio_pullup_t pullup, uint8_t af);

msz_rc_t gpio_init_default(GPIO_TypeDef *gpio_port, uint16_t gpio_pin_mask);

msz_rc_t gpio_exti_init(GPIO_TypeDef *gpio_port, uint16_t gpio_pin_mask, BOOL rising_edge, BOOL falling_edge);

#endif /* DEV_GPIO_GPIO_H_ */
