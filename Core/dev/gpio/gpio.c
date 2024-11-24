/*
 * gpio.c
 *
 *  Created on: 6 gru 2022
 *      Author: mszpakowski
 */

#include "types.h"
#include "gpio.h"

#if defined(STM32F0)
#include "stm32f0xx.h"
#elif defined(STM32G431xx)
#include "stm32g4xx.h"
#elif defined(STM32G070xx)
#include "stm32g070xx.h"
#endif


#if defined(STM32F0)

#elif defined(STM32G431xx)

static msz_rc_t gpio_init_port(GPIO_TypeDef *gpio_port) {

	msz_rc_t								rc = MSZ_RC_OK;

	if (gpio_port == GPIOA) {
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	} else if (gpio_port == GPIOB) {
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	} else if (gpio_port == GPIOC) {
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	} else if (gpio_port == GPIOD) {
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
	} else if (gpio_port == GPIOE) {
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
	} else if (gpio_port == GPIOF) {
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
	} else {
		rc = MSZ_RC_GPIO_INV_PORT;
	}

	return rc;
}

static uint8_t gpio_get_port_position_in_syscfg_exti(GPIO_TypeDef *gpio_port) {

	uint8_t									position = 0;

	if (gpio_port == GPIOA) {
		position = 0;
	} else if (gpio_port == GPIOB) {
		position = 1;
	} else if (gpio_port == GPIOC) {
		position = 2;
	} else if (gpio_port == GPIOD) {
		position = 3;
	} else if (gpio_port == GPIOE) {
		position = 4;
	} else if (gpio_port == GPIOF) {
		position = 5;
	} else {
		position = 6;
	}

	return position;
}


static void gpio_init(GPIO_TypeDef *gpio_port, uint8_t gpio_pin, gpio_type_t type,
													bool open_drain, gpio_speed_t speed, gpio_pullup_t pullup) {

	if (open_drain) {
		gpio_port->OTYPER |= (1 << gpio_pin);
	} else {
		gpio_port->OTYPER &= ~(1 << gpio_pin);
	}
	gpio_port->MODER &= ~(3 << (gpio_pin * 2));
	gpio_port->MODER |= ((uint32_t)(type) << (gpio_pin * 2));
	gpio_port->OSPEEDR &= ~(3 << (gpio_pin * 2));
	gpio_port->OSPEEDR |= ((uint32_t)(speed) << (gpio_pin * 2));
	gpio_port->PUPDR &= ~(3 << (gpio_pin * 2));
	gpio_port->PUPDR |= ((uint32_t)(pullup) << (gpio_pin * 2));
}

void gpio_set_output_state(GPIO_TypeDef *gpio_port, uint8_t gpio_pin, const bool state) {

	if (state) {
		gpio_port->BSRR |= (1 << gpio_pin);
	} else {
		gpio_port->BSRR |= (1 << (16 + gpio_pin));
	}
}

void gpio_toggle_output_state(GPIO_TypeDef *gpio_port, uint8_t gpio_pin) {

	if (gpio_port->ODR & (1 << gpio_pin)) {
		gpio_port->BSRR |= (1 << (16 + gpio_pin));
	} else {
		gpio_port->BSRR |= (1 << gpio_pin);
	}
}

msz_rc_t gpio_init_output(GPIO_TypeDef *gpio_port, uint16_t gpio_pin_mask, bool open_drain, gpio_speed_t speed, gpio_pullup_t pullup) {

	msz_rc_t								rc = MSZ_RC_OK;
	uint16_t								gpio_pin;

	rc = gpio_init_port(gpio_port);
	if (rc == MSZ_RC_OK) {
		for (gpio_pin = 0; gpio_pin < 16; gpio_pin++) {
			if (gpio_pin_mask & (1 << gpio_pin)) {
				gpio_init(gpio_port, gpio_pin, GPIO_TYPE_OUTPUT, open_drain, speed, pullup);
			}
		}
	}

	return rc;
}

msz_rc_t gpio_init_alt(GPIO_TypeDef *gpio_port, uint16_t gpio_pin_mask, bool open_drain, gpio_speed_t speed, gpio_pullup_t pullup, uint8_t af) {

	msz_rc_t								rc = MSZ_RC_OK;
	uint8_t									gpio_pin;

	if (af < 16) {
		rc = gpio_init_port(gpio_port);
		if (rc == MSZ_RC_OK) {
			for (gpio_pin = 0; gpio_pin < 16; gpio_pin++) {
				if (gpio_pin_mask & (1 << gpio_pin)) {
					if (gpio_pin < 8) {
						gpio_port->AFR[0] &= ~(0x0F << ((gpio_pin) * 4));
						gpio_port->AFR[0] |= (af << ((gpio_pin) * 4));
					} else {
						gpio_port->AFR[1] &= ~(0x0F << ((gpio_pin - 8) * 4));
						gpio_port->AFR[1] |= (af << ((gpio_pin - 8) * 4));
					}
					gpio_init(gpio_port, gpio_pin, GPIO_TYPE_ALTERNATE, open_drain, speed, pullup);
				}
			}
		}
	} else {
		rc = MSZ_RC_GPIO_INV_AF;
	}

	return rc;
}

msz_rc_t gpio_exti_init(GPIO_TypeDef *gpio_port, uint16_t gpio_pin_mask, bool rising_edge, bool falling_edge) {

	msz_rc_t								rc = MSZ_RC_OK;
	uint8_t									gpio_pin, port_pos;

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	port_pos = gpio_get_port_position_in_syscfg_exti(gpio_port);
	for (gpio_pin = 0; gpio_pin < 16; gpio_pin++) {
		if (gpio_pin_mask & (1 << gpio_pin)) {
			SYSCFG->EXTICR[gpio_pin / 4] &= ~(0x0F << (4 * (gpio_pin & 0x03)));
			SYSCFG->EXTICR[gpio_pin / 4] = port_pos << (4 * (gpio_pin & 0x03));
			EXTI->IMR1 |= 1 << gpio_pin;
			EXTI->EMR1 &= ~(1 << gpio_pin);
			EXTI->RTSR1 &= ~(1 << gpio_pin);
			EXTI->FTSR1 &= ~(1 << gpio_pin);
			if (rising_edge) {
				EXTI->RTSR1 |= 1 << gpio_pin;
			}
			if (falling_edge) {
				EXTI->FTSR1 |= 1 << gpio_pin;
			}
		}
	}

	return rc;
}

#elif defined(STM32G070xx)
#if 0

static uint8_t gpio_get_port_position_in_syscfg_exti(GPIO_TypeDef *gpio_port) {

	uint8_t									position = 0;

	if (gpio_port == GPIOA) {
		position = 0;
	} else if (gpio_port == GPIOB) {
		position = 1;
	} else if (gpio_port == GPIOC) {
		position = 2;
	} else if (gpio_port == GPIOD) {
		position = 3;
	} else if (gpio_port == GPIOE) {
		position = 4;
	} else if (gpio_port == GPIOF) {
		position = 5;
	} else {
		position = 6;
	}

	return position;
}

msz_rc_t gpio_exti_init(GPIO_TypeDef *gpio_port, uint16_t gpio_pin_mask, bool rising_edge, bool falling_edge) {

	msz_rc_t								rc = MSZ_RC_OK;
	uint8_t									gpio_pin, port_pos;

	RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
	port_pos = gpio_get_port_position_in_syscfg_exti(gpio_port);
	for (gpio_pin = 0; gpio_pin < 16; gpio_pin++) {
		if (gpio_pin_mask & (1 << gpio_pin)) {
			SYSCFG->EXTICR[gpio_pin / 4] &= ~(0x0F << (4 * (gpio_pin & 0x03)));
			SYSCFG->EXTICR[gpio_pin / 4] = port_pos << (4 * (gpio_pin & 0x03));
			EXTI->IMR1 |= 1 << gpio_pin;
			EXTI->EMR1 &= ~(1 << gpio_pin);
			EXTI->RTSR1 &= ~(1 << gpio_pin);
			EXTI->FTSR1 &= ~(1 << gpio_pin);
			if (rising_edge) {
				EXTI->RTSR1 |= 1 << gpio_pin;
			}
			if (falling_edge) {
				EXTI->FTSR1 |= 1 << gpio_pin;
			}
		}
	}

	return rc;
}

#endif

static msz_rc_t gpio_init_port(GPIO_TypeDef *gpio_port) {

	msz_rc_t								rc = MSZ_RC_OK;

	if (gpio_port == GPIOA) {
		RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	} else if (gpio_port == GPIOB) {
		RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
	} else if (gpio_port == GPIOC) {
		RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
	} else if (gpio_port == GPIOD) {
		RCC->IOPENR |= RCC_IOPENR_GPIODEN;
	} else if (gpio_port == GPIOF) {
		RCC->IOPENR |= RCC_IOPENR_GPIOFEN;
	} else {
		rc = MSZ_RC_GPIO_INV_PORT;
	}

	return rc;
}

#endif

static void gpio_init(GPIO_TypeDef *gpio_port, uint8_t gpio_pin, gpio_type_t type,
													bool open_drain, gpio_speed_t speed, gpio_pullup_t pullup) {

	if (open_drain) {
		gpio_port->OTYPER |= (1 << gpio_pin);
	} else {
		gpio_port->OTYPER &= ~(1 << gpio_pin);
	}
	gpio_port->MODER &= ~(3 << (gpio_pin * 2));
	gpio_port->MODER |= ((uint32_t)(type) << (gpio_pin * 2));
	gpio_port->OSPEEDR &= ~(3 << (gpio_pin * 2));
	gpio_port->OSPEEDR |= ((uint32_t)(speed) << (gpio_pin * 2));
	gpio_port->PUPDR &= ~(3 << (gpio_pin * 2));
	gpio_port->PUPDR |= ((uint32_t)(pullup) << (gpio_pin * 2));
}

msz_rc_t gpio_init_default(GPIO_TypeDef *gpio_port, uint16_t gpio_pin_mask) {

	msz_rc_t								rc = MSZ_RC_OK;
	uint8_t									gpio_pin;

	for (gpio_pin = 0; gpio_pin < 16; gpio_pin++) {
		if (gpio_pin_mask & (1 << gpio_pin)) {
			gpio_init(gpio_port, gpio_pin, GPIO_TYPE_DISABLE, false, GPIO_SPEED_LOW, GPIO_NO_PULL_UP_NO_PULL_DOWN);
		}
	}

	return rc;
}

msz_rc_t gpio_init_input(GPIO_TypeDef *gpio_port, uint16_t gpio_pin_mask, gpio_speed_t speed, gpio_pullup_t pullup) {

	msz_rc_t								rc = MSZ_RC_OK;
	uint8_t									gpio_pin;

	rc = gpio_init_port(gpio_port);
	if (rc == MSZ_RC_OK) {
		for (gpio_pin = 0; gpio_pin < 16; gpio_pin++) {
			if (gpio_pin_mask & (1 << gpio_pin)) {
				gpio_init(gpio_port, gpio_pin, GPIO_TYPE_INPUT, false, speed, pullup);
			}
		}
	}

	return rc;
}

bool gpio_get_input_state(GPIO_TypeDef *gpio_port, uint8_t gpio_pin) {

	bool									state;

	state = gpio_port->IDR & (1 << gpio_pin) ? true : false;

	return state;
}

msz_rc_t gpio_init_alt(GPIO_TypeDef *gpio_port, uint16_t gpio_pin_mask, bool open_drain, gpio_speed_t speed, gpio_pullup_t pullup, uint8_t af) {

	msz_rc_t								rc = MSZ_RC_OK;
	uint8_t									gpio_pin;

	if (af < 16) {
		rc = gpio_init_port(gpio_port);
		if (rc == MSZ_RC_OK) {
			for (gpio_pin = 0; gpio_pin < 16; gpio_pin++) {
				if (gpio_pin_mask & (1 << gpio_pin)) {
					if (gpio_pin < 8) {
						gpio_port->AFR[0] &= ~(0x0F << ((gpio_pin) * 4));
						gpio_port->AFR[0] |= (af << ((gpio_pin) * 4));
					} else {
						gpio_port->AFR[1] &= ~(0x0F << ((gpio_pin - 8) * 4));
						gpio_port->AFR[1] |= (af << ((gpio_pin - 8) * 4));
					}
					gpio_init(gpio_port, gpio_pin, GPIO_TYPE_ALTERNATE, open_drain, speed, pullup);
				}
			}
		}
	} else {
		rc = MSZ_RC_GPIO_INV_AF;
	}

	return rc;
}
