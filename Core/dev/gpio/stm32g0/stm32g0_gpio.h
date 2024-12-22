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

#if defined(GPIOE)
#define GPIO_GET_INDEX(__GPIOx__)    (((__GPIOx__) == (GPIOA))? 0uL :\
                                      ((__GPIOx__) == (GPIOB))? 1uL :\
                                      ((__GPIOx__) == (GPIOC))? 2uL :\
                                      ((__GPIOx__) == (GPIOD))? 3uL :\
                                      ((__GPIOx__) == (GPIOE))? 4uL : 5uL)
#else
#define GPIO_GET_INDEX(__GPIOx__)    (((__GPIOx__) == (GPIOA))? 0uL :\
                                      ((__GPIOx__) == (GPIOB))? 1uL :\
                                      ((__GPIOx__) == (GPIOC))? 2uL :\
                                      ((__GPIOx__) == (GPIOD))? 3uL : 5uL)
#endif /* GPIOE */

msz_rc_t gpio_init_port(GPIO_TypeDef *gpio_port);

#endif /* defined(STM32G070xx) */

#endif /* DEV_GPIO_STM32G0_STM32G0_GPIO_H_ */
