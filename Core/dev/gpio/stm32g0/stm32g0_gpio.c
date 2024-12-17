/*
 * stm32g0_gpio.c
 *
 *  Created on: Dec 16, 2024
 *      Author: marek
 */

#if defined(STM32G070xx)

#include "stm32g070xx.h"
#include "main.h"
#include "types.h"





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

msz_rc_t gpio_init_port(GPIO_TypeDef *gpio_port) {

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

#endif /* defined(STM32G070xx) */
