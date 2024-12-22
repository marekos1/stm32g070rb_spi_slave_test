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
#include "gpio/stm32g0/stm32g0_gpio.h"
#endif




static void gpio_init(GPIO_TypeDef *gpio_port, const uint8_t gpio_pin, const gpio_type_t type, const bool open_drain,
																	   const gpio_speed_t speed, const gpio_pullup_t pullup) {

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

msz_rc_t gpio_init_as_default(GPIO_TypeDef *gpio_port, const uint8_t gpio_pin) {

	msz_rc_t								rc = MSZ_RC_OK;

	gpio_init(gpio_port, gpio_pin, GPIO_TYPE_DISABLE, false, GPIO_SPEED_LOW, GPIO_NO_PULL_UP_NO_PULL_DOWN);

	return rc;
}

msz_rc_t gpio_init_pin_mask_as_default(GPIO_TypeDef *gpio_port, const uint16_t gpio_pin_mask) {

	msz_rc_t								rc = MSZ_RC_OK;
	uint8_t									gpio_pin;

	for (gpio_pin = 0; gpio_pin < 16; gpio_pin++) {
		if (gpio_pin_mask & (1 << gpio_pin)) {
			gpio_init(gpio_port, gpio_pin, GPIO_TYPE_DISABLE, false, GPIO_SPEED_LOW, GPIO_NO_PULL_UP_NO_PULL_DOWN);
		}
	}

	return rc;
}

msz_rc_t gpio_init_as_input(GPIO_TypeDef *gpio_port, const uint8_t gpio_pin, const gpio_speed_t speed, const gpio_pullup_t pullup) {

	msz_rc_t								rc = MSZ_RC_OK;

	gpio_init(gpio_port, gpio_pin, GPIO_TYPE_INPUT, false, speed, pullup);

	return rc;
}

msz_rc_t gpio_init_pin_mask_as_input(GPIO_TypeDef *gpio_port, const uint16_t gpio_pin_mask, const gpio_speed_t speed, const gpio_pullup_t pullup) {

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

msz_rc_t gpio_init_pin_mask_as_input_interrupt(GPIO_TypeDef *gpio_port, const uint16_t gpio_pin_mask, const gpio_speed_t speed, const gpio_pullup_t pullup,
											   const bool int_rising, const bool int_falling) {

	msz_rc_t								rc = MSZ_RC_OK;
	uint8_t									gpio_pin;
	uint32_t 								reg_value, gpio_single_pin_mask;

	rc = gpio_init_port(gpio_port);
	if (rc == MSZ_RC_OK) {
		for (gpio_pin = 0; gpio_pin < 16; gpio_pin++) {
			if (gpio_pin_mask & (1 << gpio_pin)) {
				gpio_init(gpio_port, gpio_pin, GPIO_TYPE_INPUT, false, speed, pullup);

				reg_value = EXTI->EXTICR[gpio_pin >> 2u];
				reg_value &= ~(0x0FuL << (8u * (gpio_pin & 0x03u)));
				reg_value |= (GPIO_GET_INDEX(gpio_port) << (8u * (gpio_pin & 0x03u)));
				EXTI->EXTICR[gpio_pin >> 2u] = reg_value;

				gpio_single_pin_mask = 1 << gpio_pin;
				/* Clear Rising Falling edge configuration */
				reg_value = EXTI->RTSR1;
				reg_value &= ~(gpio_single_pin_mask);
				if (int_rising) {
					reg_value |= gpio_single_pin_mask;
				}
				EXTI->RTSR1 = reg_value;

				reg_value = EXTI->FTSR1;
				reg_value &= ~(gpio_single_pin_mask);
				if (int_falling) {
					reg_value |= gpio_single_pin_mask;
				}
				EXTI->FTSR1 = reg_value;

				/* Clear EXTI line configuration */
				reg_value = EXTI->EMR1;
				reg_value &= ~(gpio_single_pin_mask);
		//		if ((GPIO_Init->Mode & EXTI_EVT) != 0x00u) {
		//			reg_value |= gpio_pin_mask;
		//		}
				EXTI->EMR1 = reg_value;

				reg_value = EXTI->IMR1;
				reg_value |= gpio_single_pin_mask;
				EXTI->IMR1 = reg_value;
			}
		}
	}

	return rc;
}

bool gpio_get_input_state(GPIO_TypeDef *gpio_port, const uint8_t gpio_pin) {

	bool									state;

	state = gpio_port->IDR & (1 << gpio_pin) ? true : false;

	return state;
}

bool gpio_pin_mask_get_input_state(GPIO_TypeDef *gpio_port, const uint16_t gpio_pin_mask) {

	bool									state;

	state = gpio_port->IDR & gpio_pin_mask ? true : false;

	return state;
}

msz_rc_t gpio_init_as_output(GPIO_TypeDef *gpio_port, const uint8_t gpio_pin, const bool open_drain, const gpio_speed_t speed, const gpio_pullup_t pullup) {

	msz_rc_t								rc = MSZ_RC_OK;

	gpio_init(gpio_port, gpio_pin, GPIO_TYPE_OUTPUT, open_drain, speed, pullup);

	return rc;
}

msz_rc_t gpio_init_pin_mask_as_output(GPIO_TypeDef *gpio_port, const uint16_t gpio_pin_mask, const bool open_drain, const gpio_speed_t speed, const gpio_pullup_t pullup) {

	msz_rc_t								rc = MSZ_RC_OK;
	uint8_t									gpio_pin;

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

msz_rc_t gpio_set_output_state(GPIO_TypeDef *gpio_port, const uint8_t gpio_pin, const bool state) {

	msz_rc_t								rc = MSZ_RC_OK;

	if (state) {
		gpio_port->ODR |= (1 << gpio_pin);
	} else {
		gpio_port->ODR &= ~(1 << gpio_pin);
	}

	return rc;
}

msz_rc_t gpio_pin_mask_set_output_state(GPIO_TypeDef *gpio_port, const uint16_t gpio_pin_mask, const bool state) {

	msz_rc_t								rc = MSZ_RC_OK;

	if (state) {
		gpio_port->ODR |= gpio_pin_mask;
	} else {
		gpio_port->ODR &= ~(gpio_pin_mask);
	}

	return rc;
}

msz_rc_t gpio_init_as_alternate_func(GPIO_TypeDef *gpio_port, const uint8_t gpio_pin, const bool open_drain, const gpio_speed_t speed,
																					  const gpio_pullup_t pullup, const uint8_t af) {

	msz_rc_t								rc = MSZ_RC_OK;

	if (af < 16) {
		rc = gpio_init_port(gpio_port);
		if (rc == MSZ_RC_OK) {
			if (gpio_pin < 8) {
				gpio_port->AFR[0] &= ~(0x0F << ((gpio_pin) * 4));
				gpio_port->AFR[0] |= (af << ((gpio_pin) * 4));
			} else {
				gpio_port->AFR[1] &= ~(0x0F << ((gpio_pin - 8) * 4));
				gpio_port->AFR[1] |= (af << ((gpio_pin - 8) * 4));
			}
			gpio_init(gpio_port, gpio_pin, GPIO_TYPE_ALTERNATE, open_drain, speed, pullup);
		}
	} else {
		rc = MSZ_RC_GPIO_INV_AF;
	}

	return rc;
}

msz_rc_t gpio_init_pin_mask_as_alternate_func(GPIO_TypeDef *gpio_port, const uint16_t gpio_pin_mask, const bool open_drain, const gpio_speed_t speed,
																									 const gpio_pullup_t pullup, const uint8_t af) {

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
