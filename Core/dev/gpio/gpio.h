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




msz_rc_t gpio_init_as_default(GPIO_TypeDef *gpio_port, const uint8_t gpio_pin);

msz_rc_t gpio_init_pin_mask_as_default(GPIO_TypeDef *gpio_port, const uint16_t gpio_pin_mask);

msz_rc_t gpio_init_as_input(GPIO_TypeDef *gpio_port, const uint8_t gpio_pin, const gpio_speed_t speed, const gpio_pullup_t pullup);

msz_rc_t gpio_init_pin_mask_as_input(GPIO_TypeDef *gpio_port, const uint16_t gpio_pin_mask, const gpio_speed_t speed, const gpio_pullup_t pullup);

bool gpio_get_input_state(GPIO_TypeDef *gpio_port, const uint8_t gpio_pin);

bool gpio_pin_mask_get_input_state(GPIO_TypeDef *gpio_port, const uint16_t gpio_pin_mask);

msz_rc_t gpio_init_pin_mask_as_output(GPIO_TypeDef *gpio_port, const uint16_t gpio_pin_mask, const bool open_drain, const gpio_speed_t speed, const gpio_pullup_t pullup);

msz_rc_t gpio_set_output_state(GPIO_TypeDef *gpio_port, const uint8_t gpio_pin, const bool state);

msz_rc_t gpio_pin_mask_set_output_state(GPIO_TypeDef *gpio_port, const uint16_t gpio_pin_mask, const bool state);

msz_rc_t gpio_init_as_alternate_func(GPIO_TypeDef *gpio_port, const uint8_t gpio_pin, const bool open_drain, const gpio_speed_t speed,
																					  const gpio_pullup_t pullup, const uint8_t af);

msz_rc_t gpio_init_pin_mask_as_alternate_func(GPIO_TypeDef *gpio_port, const uint16_t gpio_pin_mask, const bool open_drain, const gpio_speed_t speed,
																									 const gpio_pullup_t pullup, const uint8_t af);

#endif /* DEV_GPIO_GPIO_H_ */
