/*
 * stm32g0_gpio.h
 *
 *  Created on: Dec 16, 2024
 *      Author: marek
 */

#ifndef DEV_GPIO_STM32G0_STM32G0_GPIO_H_
#define DEV_GPIO_STM32G0_STM32G0_GPIO_H_

#if defined(STM32G070xx)

#include "stm32g070xx.h"
#include "main.h"
#include "types.h"

msz_rc_t gpio_init_port(GPIO_TypeDef *gpio_port);

#endif /* defined(STM32G070xx) */

#endif /* DEV_GPIO_STM32G0_STM32G0_GPIO_H_ */
