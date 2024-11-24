/*
 * uart.h
 *
 *  Created on: 12 gru 2022
 *      Author: mszpakowski
 */

#ifndef DEV_UART_UART_H_
#define DEV_UART_UART_H_

#include <stdint.h>

#include "main.h"
#include "types.h"

#if defined(STM32F0)
#include "stm32f0xx.h"
#elif defined(STM32G431xx)
#include "stm32g4xx.h"
#elif defined(STM32G070xx)
#include "stm32g070xx.h"
#endif

#include "gpio/gpio.h"


#define UART_USE_FIFO 		1
#define UART_USE_IRQ 		1
#define UART_USE_DMA 		0
#define UART_USE_HAL 		0



typedef struct {
	USART_TypeDef							*usart;
	gpio_dev_t								tx_gpio_pin;
	gpio_dev_t								rx_gpio_pin;
} uart_msz_dev_t;

typedef union {
#if UART_USE_HAL
	UART_HandleTypeDef 						hal;
#endif /* UART_USE_HAL */
	const uart_msz_dev_t					*msz;
} uart_dev_t;

typedef enum {
	UART_STOP_BITS_1 = 0,
	UART_STOP_BITS_1_5,
	UART_STOP_BITS_2
} uart_stop_bits;

typedef enum {
	DEV_UART_PARITY_NONE = 0,
} uart_parity;

typedef struct {
	uint32_t								baud;
	uint8_t									data_bits;
	uart_stop_bits							stop_bits;
	uart_parity								parity;
	bool									rx_enable;
	bool									tx_enable;
} uart_conf_t;

typedef struct{
	uint8_t									*data;
	uint8_t									*data_write_ptr;
	uint8_t									*data_read_ptr;
} uart_buffer_t;


typedef struct {


	uart_dev_t								dev;



	uart_conf_t								conf;
	bool									enable;

	uart_buffer_t							rx_buffer;
	uart_buffer_t							tx_buffer;
} uart_t;


int uart_read(uart_t *uart, uint8_t *data, uint16_t data_len, uint32_t timeout_us);

msz_rc_t uart_write(uart_t *uart, uint8_t *data, uint16_t data_len);

msz_rc_t uart_init(uart_t *uart, const uart_msz_dev_t *msz_dev, uart_conf_t *conf);

msz_rc_t uart_deinit(uart_t *uart);

void uart_irq(USART_TypeDef	*usart);

void uart_dma_irq(USART_TypeDef	*usart);

void uart_proces(void);

#endif /* DEV_UART_UART_H_ */
