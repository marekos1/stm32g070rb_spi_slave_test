/*
 * uart.c
 *
 *  Created on: 12 gru 2022
 *      Author: mszpakowski
 */

#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"
#include "types.h"

#include "uart.h"
#include "gpio/gpio.h"

#include "board.h" //for test

#if 0

#define UART_TEST1_PIN_UP() 		HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_SET)
#define UART_TEST1_PIN_DOWN() 		HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_RESET)
#define UART_TEST1_PIN_TOGGLE()  	HAL_GPIO_TogglePin(TEST1_GPIO_Port, TEST1_Pin);

#define UART_TEST2_PIN_UP() 		HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, GPIO_PIN_SET)
#define UART_TEST2_PIN_DOWN() 		HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, GPIO_PIN_RESET)
#define UART_TEST2_PIN_TOGGLE()		HAL_GPIO_TogglePin(TEST2_GPIO_Port, TEST2_Pin);

#define UART_TEST3_PIN_UP() 		HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_SET)
#define UART_TEST3_PIN_DOWN() 		HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_RESET)
#define UART_TEST3_PIN_TOGGLE()		HAL_GPIO_TogglePin(TEST3_GPIO_Port, TEST3_Pin);

#else

#define UART_TEST1_PIN_UP()
#define UART_TEST1_PIN_DOWN()

#define UART_TEST2_PIN_UP()
#define UART_TEST2_PIN_DOWN()

#define UART_TEST3_PIN_UP()
#define UART_TEST3_PIN_DOWN()

#endif


#if defined(STM32G431xx)

static msz_rc_t stm32g431_uart_enable(USART_TypeDef *usart) {

	msz_rc_t									rc = MSZ_RC_OK;

	if (usart == USART2) {
		RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
	} else if (usart == USART3) {
		RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
	} else if (usart == UART4) {
		RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;
	} else if (usart == LPUART1) {
		RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
	} else {
		rc = MSZ_RC_ERR_NOT_FOUND;
	}
#if USE_DMA
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
#endif /* if USE_DMA */

	return rc;
}

static msz_rc_t stm32g431_uart_disable(USART_TypeDef *usart) {

	msz_rc_t									rc = MSZ_RC_OK;

	if (usart == USART2) {
		RCC->APB1ENR1 &= ~(RCC_APB1ENR1_USART2EN);
	} else if (usart == USART3) {
		RCC->APB1ENR1 &= ~(RCC_APB1ENR1_USART3EN);
	} else if (usart == UART4) {
		RCC->APB1ENR1 &= ~(RCC_APB1ENR1_UART4EN);
	} else if (usart == LPUART1) {
		RCC->APB1ENR2 &= ~(RCC_APB1ENR2_LPUART1EN);
	} else {
		rc = MSZ_RC_ERR_NOT_FOUND;
	}

	return rc;
}

static msz_rc_t stm32g431_uart_reset(USART_TypeDef *usart) {

	msz_rc_t									rc = MSZ_RC_OK;

	if (usart == USART2) {
		RCC->APB1RSTR1 |= RCC_APB1RSTR1_USART2RST;
		RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_USART2RST);
	} else if (usart == USART3) {
		RCC->APB1RSTR1 |= RCC_APB1RSTR1_USART3RST;
		RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_USART3RST);
	} else if (usart == UART4) {
		RCC->APB1RSTR1 |= RCC_APB1RSTR1_UART4RST;
		RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_UART4RST);
	} else if (usart == LPUART1) {
		RCC->APB1RSTR2 |= RCC_APB1RSTR2_LPUART1RST;
		RCC->APB1RSTR2 &= ~(RCC_APB1RSTR2_LPUART1RST);
	} else {
		rc = MSZ_RC_ERR_NOT_FOUND;
	}

	return rc;
}

static msz_rc_t stm32g431_uart_init_gpio_pins(const uart_dev_t *dev, bool rx_enable, bool tx_enable) {

	msz_rc_t									rc = MSZ_RC_OK;

	if (tx_enable) {
		rc |= gpio_init_alt(dev->tx_gpio_pin.gpio_port, (1 << dev->tx_gpio_pin.gpio_pin), false, GPIO_SPEED_HIGH,
							GPIO_NO_PULL_UP_NO_PULL_DOWN, dev->tx_gpio_pin.alt_func);
	}
	if (rx_enable) {
		rc |= gpio_init_alt(dev->rx_gpio_pin.gpio_port, (1 << dev->rx_gpio_pin.gpio_pin), false, GPIO_SPEED_HIGH,
							GPIO_PULL_UP, dev->rx_gpio_pin.alt_func);
	}

	return rc;
}

static msz_rc_t stm32g431_uart_deinit_gpio_pins(const uart_dev_t *dev) {

	msz_rc_t									rc = MSZ_RC_OK;

	rc = gpio_init_default(dev->tx_gpio_pin.gpio_port, (1 << dev->tx_gpio_pin.gpio_pin));
	rc = gpio_init_default(dev->rx_gpio_pin.gpio_port, (1 << dev->rx_gpio_pin.gpio_pin));

	return rc;
}

static uint32_t stm32g431_uart_get_clock_frequncy_by_reg_conf(uint32_t reg_val) {

	uint32_t								frequency = 0;

	switch (reg_val) {
	case 0: /* PCLK */
		reg_val = RCC->CFGR >> RCC_CFGR_PPRE1_Pos;
		reg_val &= 0x0000001FU;
		frequency = SystemCoreClock;
		frequency = (frequency >> APBPrescTable[reg_val]);
		/* To jest z HALa, ale wg. mnie jest bład, bo nie jest uwzgledniony AHB precaler. */
		break;
	case 1: /* SYSCLK */
		frequency = SystemCoreClock;
		break;
	case 2: /* HSI16 */
		frequency = HSI_VALUE;
		break;
	case 3: /* LSE */
		frequency = LSE_VALUE;
		break;
	default:
		break;
	}

	return frequency;
}

static uint32_t stm32g431_uart_get_clock_frequncy(USART_TypeDef *usart) {

	uint32_t								frequency = 0;
	uint32_t								reg_val = 7; /* Something illegal */

	if (usart == USART2) {
		reg_val = RCC->CCIPR >> RCC_CCIPR_USART2SEL_Pos;
		reg_val &= 0x00000003;
	} else if (usart == USART3) {
		reg_val = RCC->CCIPR >> RCC_CCIPR_USART3SEL_Pos;
		reg_val &= 0x00000003;
	} else if (usart == UART4) {
		reg_val = RCC->CCIPR >> RCC_CCIPR_UART4SEL_Pos;
		reg_val &= 0x00000003;
	} else if (usart == LPUART1) {
		reg_val = RCC->CCIPR >> RCC_CCIPR_LPUART1SEL_Pos;
		reg_val &= 0x00000003;
	}
	frequency = stm32g431_uart_get_clock_frequncy_by_reg_conf(reg_val);

	return frequency;
}

static void stm32g431_uart_init_rx(USART_TypeDef *usart, uint32_t recv_timeout_bits) {

	uint32_t								prioritygroup;


	usart->RTOR &= ~(0x00FFFFFF);
	if (recv_timeout_bits) {
		if (!(usart == LPUART1)) {
			usart->RTOR |= (recv_timeout_bits & 0x00FFFFFF);
			usart->CR2 |= USART_CR2_RTOEN;
		}
	} else {
		usart->CR2 &= ~(USART_CR2_RTOEN);
	}
	usart->RQR |= USART_RQR_RXFRQ;
	usart->ICR |= USART_ICR_RTOCF | USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;

	prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(UART4_IRQn, NVIC_EncodePriority(prioritygroup, 7, 0U));
	NVIC_EnableIRQ(UART4_IRQn);

#if USE_DMA
	prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(prioritygroup, 7, 0U));
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
#else /* if USE_DMA */
	usart->CR1 |=  USART_CR1_RXNEIE | USART_CR1_RTOIE;
	usart->CR3 |= USART_CR3_EIE;
#endif /* if USE_DMA */
}

msz_rc_t stm32g431_uart_init(const uart_dev_t	*dev, uart_conf_t *conf) {

	msz_rc_t									rc = MSZ_RC_OK;
	uint32_t								frequency;
	USART_TypeDef 							*usart = dev->usart;
	uint64_t								budrate;

	rc = stm32g431_uart_enable(usart);
	if (rc == MSZ_RC_OK) {
		stm32g431_uart_init_gpio_pins(dev, conf->rx_enable, conf->tx_enable);
		frequency = stm32g431_uart_get_clock_frequncy(usart);
		if (frequency) {
			if (usart == LPUART1) {
				budrate = (uint64_t)frequency * (uint64_t)256;
				budrate = budrate / (uint64_t)conf->baud;
				usart->BRR = (uint32_t)(budrate & (uint64_t)0x00000000000FFFFF);
			} else {
				usart->BRR = frequency / conf->baud;
			}
			if (conf->tx_enable) {
				usart->CR1 |= USART_CR1_TE;
			}
			if (conf->rx_enable) {
				//usart->CR1 |= USART_CR1_RE;
				stm32g431_uart_init_rx(usart, 10 * (1 + 8 + 1));
			}
			if (conf->tx_enable || conf->rx_enable) {
				usart->CR1 |= USART_CR1_UE;
			}
		} else {
			rc = MSZ_RC_ERR_INV_STATE;
		}
	}

	return rc;
}

msz_rc_t stm32g431_uart_deinit(const uart_dev_t *dev) {

	msz_rc_t									rc = MSZ_RC_OK;
	USART_TypeDef 							*usart = dev->usart;

	stm32g431_uart_reset(usart);
	stm32g431_uart_deinit_gpio_pins(dev);
	stm32g431_uart_disable(usart);

	return rc;
}

msz_rc_t stm32g431_uart_write(const uart_dev_t *dev, uint8_t *data, uint16_t data_len) {

	msz_rc_t									rc = MSZ_RC_OK;
	uint16_t								i;
	USART_TypeDef 							*usart = dev->usart;

	for (i = 0; i < data_len; i++) {
		while ((usart->ISR & USART_ISR_TXE) == 0);
		usart->TDR = *(data + i);
	}
	while ((usart->ISR & USART_ISR_TC) == 0);

	return rc;
}

int stm32g431_uart_read(const uart_dev_t *dev, uint8_t *data, uint16_t data_len, uint32_t timeout_us) {

	int										recv_length = 0;
	uint16_t								i;
	USART_TypeDef 							*usart = dev->usart;

	stm32g431_uart_init_rx(usart, 10 * (1 + 8 + 1));
	for (i = 0; i < data_len; i++) {
		while ((usart->ISR & USART_ISR_RXNE) == 0) {
			if (usart->ISR & USART_ISR_RTOF) {
				recv_length = i;
				break;
			}
		}
		if (recv_length) {
			break;
		}
		*(data + i) = (uint8_t)usart->RDR;
	}
	recv_length = i;

	return recv_length;
}

msz_rc_t stm32g431_read_byte(const uart_dev_t *dev, uint8_t *data_byte) {

	msz_rc_t									rc = MSZ_RC_ERR_NOT_FOUND;
	USART_TypeDef 							*usart = dev->usart;

#if 0
	if (usart->ISR & USART_ISR_RXNE) {
		*data_byte = (uint8_t)usart->RDR;
		rc = MSZ_RC_OK;
	} else if (usart->ISR & USART_ISR_RTOF) {
		rc = MSZ_RC_ERR_TIMEOUT;
		stm32g431_uart_init_rx(usart, 10 * (1 + 8 + 1));
	}
#else
	rc = MSZ_RC_OK;

	uint32_t uart_isr = usart->ISR;

	if (uart_isr & USART_ISR_RXNE) {
		*data_byte = (uint8_t)usart->RDR;
	} else if (uart_isr & USART_ISR_RTOF) {
		rc = MSZ_RC_ERR_TIMEOUT;
		stm32g431_uart_init_rx(usart, 10 * (1 + 8 + 1));
	}
#endif

#if 0
	if (usart->ISR & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) {
		rc = MSZ_RC_ERR_INV_DATA;
		stm32g431_uart_init_rx(usart, 10 * (1 + 8 + 1));
	} else if (usart->ISR & USART_ISR_RTOF) {
		rc = MSZ_RC_ERR_TIMEOUT;
		stm32g431_uart_init_rx(usart, 10 * (1 + 8 + 1));
	}
#endif

	return rc;
}

msz_rc_t stm32g431_write_byte(const uart_dev_t *dev, uint8_t data_byte) {

	msz_rc_t								rc = MSZ_RC_ERR_NOT_FOUND;
	USART_TypeDef 							*usart = dev->usart;

	if (usart->ISR & USART_ISR_TXE) {
		usart->TDR = data_byte;
		rc = MSZ_RC_OK;
	} else {
		rc = MSZ_RC_NOT_READY;
	}

	return rc;
}


#elif defined(STM32G070xx)

#include "stm32g070xx.h"

static msz_rc_t stm32g070_uart_enable(USART_TypeDef *usart) {

	msz_rc_t									rc = MSZ_RC_OK;

	if (usart == USART1) {
		RCC->APBENR2 |= RCC_APBENR2_USART1EN;
	} else if (usart == USART2) {
		RCC->APBENR1 |= RCC_APBENR1_USART2EN;
	} else if (usart == USART3) {
		RCC->APBENR1 |= RCC_APBENR1_USART3EN;
	} else if (usart == USART4) {
		RCC->APBENR1 |= RCC_APBENR1_USART4EN;
	} else {
		rc = MSZ_RC_ERR_NOT_FOUND;
	}
#if USE_DMA
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
#endif /* if USE_DMA */

	return rc;
}

static msz_rc_t stm32g070_uart_disable(USART_TypeDef *usart) {

	msz_rc_t									rc = MSZ_RC_OK;

	if (usart == USART1) {
		RCC->APBENR2 &= ~(RCC_APBENR2_USART1EN);
	} else if (usart == USART2) {
		RCC->APBENR1 &= ~(RCC_APBENR1_USART2EN);
	} else if (usart == USART3) {
		RCC->APBENR1 &= ~(RCC_APBENR1_USART3EN);
	} else if (usart == USART4) {
		RCC->APBENR1 &= ~(RCC_APBENR1_USART4EN);
	} else {
		rc = MSZ_RC_ERR_NOT_FOUND;
	}

	return rc;
}



static msz_rc_t stm32g070_uart_init_gpio_pins(const uart_dev_t *dev, bool rx_enable, bool tx_enable) {

	msz_rc_t									rc = MSZ_RC_OK;

	if (tx_enable) {
		rc |= gpio_init_alt(dev->msz->tx_gpio_pin.gpio_port, (1 << dev->msz->tx_gpio_pin.gpio_pin), false, GPIO_SPEED_HIGH,
							GPIO_NO_PULL_UP_NO_PULL_DOWN, dev->msz->tx_gpio_pin.alt_func);
	}
	if (rx_enable) {
		rc |= gpio_init_alt(dev->msz->rx_gpio_pin.gpio_port, (1 << dev->msz->rx_gpio_pin.gpio_pin), false, GPIO_SPEED_HIGH,
							GPIO_PULL_UP, dev->msz->rx_gpio_pin.alt_func);
	}

	return rc;
}

static uint32_t stm32g070_uart_get_clock_frequncy_by_reg_conf(uint32_t reg_val) {

	uint32_t								frequency = 0;

	switch (reg_val) {
	case 0: /* PCLK */
		reg_val &= 0x0000001FU;
		frequency = SystemCoreClock;
		frequency = (frequency >> APBPrescTable[reg_val]);
		break;
	case 1: /* SYSCLK */
		frequency = SystemCoreClock;
		break;
	case 2: /* HSI16 */
		frequency = HSI_VALUE;
		break;
	case 3: /* LSE */
		frequency = LSE_VALUE;
		break;
	default:
		break;
	}

	return frequency;
}

static uint32_t stm32g070_uart_get_clock_frequncy(USART_TypeDef *usart) {

	uint32_t								frequency = 0;
	uint32_t								reg_val = 7;

	if (usart == USART1) {

	} else if (usart == USART2) {
		reg_val = RCC->CCIPR >> RCC_CCIPR_USART2SEL;
		reg_val &= 0x00000003;
	} else if (usart == USART3) {

	} else if (usart == USART4) {

	}

	frequency = stm32g070_uart_get_clock_frequncy_by_reg_conf(reg_val);

	return frequency;
}

static void stm32g070_uart_init_rx(USART_TypeDef *usart, uint32_t recv_timeout_bits) {

	usart->RTOR &= ~(0x00FFFFFF);
	if (recv_timeout_bits) {
		usart->RTOR |= (recv_timeout_bits & 0x00FFFFFF);
		usart->CR2 |= USART_CR2_RTOEN;
	} else {
		usart->CR2 &= ~(USART_CR2_RTOEN);
	}
	usart->RQR |= USART_RQR_RXFRQ;
	usart->ICR |= USART_ICR_RTOCF | USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;

#if UART_USE_IRQ && 0
	uint32_t								prioritygroup;

	prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(UART4_IRQn, NVIC_EncodePriority(prioritygroup, 7, 0U));
	NVIC_EnableIRQ(UART4_IRQn);
#endif /* UART_USE_IRQ */

#if UART_USE_DMA
	prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(prioritygroup, 7, 0U));
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
#else /* UART_USE_DMA */
//ms	usart->CR1 |=  USART_CR1_RXNEIE | USART_CR1_RTOIE;
//ms	usart->CR3 |= USART_CR3_EIE;
#endif /* UART_USE_DMA */
}






msz_rc_t stm32g070_uart_init(uart_dev_t *dev, uart_conf_t *conf) {

	msz_rc_t								rc = MSZ_RC_OK;
	uint32_t								frequency;
	USART_TypeDef 							*usart = dev->msz->usart;

	rc = stm32g070_uart_enable(usart);
	if (rc == MSZ_RC_OK) {
		stm32g070_uart_init_gpio_pins(dev, conf->rx_enable, conf->tx_enable);
		frequency = stm32g070_uart_get_clock_frequncy(usart);
		if (frequency) {
				usart->BRR = frequency / conf->baud;
			if (conf->tx_enable) {
#if UART_USE_FIFO
				usart->CR1 &= ~USART_CR1_TE;
				usart->CR1 |= USART_CR1_FIFOEN;
				usart->CR3 |= USART_CR3_TXFTCFG_2;
				usart->CR1 |= USART_CR1_TE;
#else /* UART_USE_FIFO */
				usart->CR1 |= USART_CR1_TE;
#endif /* UART_USE_FIFO */
			}
			if (conf->rx_enable) {
				stm32g070_uart_init_rx(usart, 10 * (1 + 8 + 1));
			}
			if (conf->tx_enable || conf->rx_enable) {
				usart->CR1 |= USART_CR1_UE;
			}
		} else {
			rc = MSZ_RC_ERR_INV_STATE;
		}
	}

	return rc;
}



static msz_rc_t stm32g070_uart_deinit_gpio_pins(const uart_dev_t *dev) {

	msz_rc_t									rc = MSZ_RC_OK;

	rc = gpio_init_default(dev->msz->tx_gpio_pin.gpio_port, (1 << dev->msz->tx_gpio_pin.gpio_pin));
	rc = gpio_init_default(dev->msz->rx_gpio_pin.gpio_port, (1 << dev->msz->rx_gpio_pin.gpio_pin));

	return rc;
}

static msz_rc_t stm32g070_uart_reset(USART_TypeDef *usart) {

	msz_rc_t									rc = MSZ_RC_OK;

	if (usart == USART1) {
		RCC->APBRSTR2 |= RCC_APBRSTR2_USART1RST;
		RCC->APBRSTR2 &= ~(RCC_APBRSTR2_USART1RST);
	} else if (usart == USART2) {
		RCC->APBRSTR1 |= RCC_APBRSTR1_USART2RST;
		RCC->APBRSTR1 &= ~(RCC_APBRSTR1_USART2RST);
	} else if (usart == USART3) {
		RCC->APBRSTR1 |= RCC_APBRSTR1_USART3RST;
		RCC->APBRSTR1 &= ~(RCC_APBRSTR1_USART3RST);
	} else if (usart == USART4) {
		RCC->APBRSTR1 |= RCC_APBRSTR1_USART4RST;
		RCC->APBRSTR1 &= ~(RCC_APBRSTR1_USART4RST);
	} else {
		rc = MSZ_RC_ERR_NOT_FOUND;
	}

	return rc;
}



msz_rc_t stm32g070_uart_deinit(const uart_dev_t *dev) {

	msz_rc_t									rc = MSZ_RC_OK;
	USART_TypeDef 							*usart = dev->msz->usart;

	stm32g070_uart_reset(usart);
	stm32g070_uart_deinit_gpio_pins(dev);
	stm32g070_uart_disable(usart);

	return rc;
}

#endif /*defined(STM32G431xx) */


#if UART_USE_HAL

//ms UART_HandleTypeDef huart2;

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_HAL_UART_Init(uart_dev_t *dev, const uart_msz_dev_t *msz_dev) {

	dev->hal.Instance = msz_dev->usart;
	dev->hal.Init.BaudRate = 115200;
	dev->hal.Init.WordLength = UART_WORDLENGTH_8B;
	dev->hal.Init.StopBits = UART_STOPBITS_1;
	dev->hal.Init.Parity = UART_PARITY_NONE;
	dev->hal.Init.Mode = UART_MODE_TX_RX;
	dev->hal.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	dev->hal.Init.OverSampling = UART_OVERSAMPLING_16;
	dev->hal.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	dev->hal.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	dev->hal.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&dev->hal) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&dev->hal, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&dev->hal, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&dev->hal) != HAL_OK) {
		Error_Handler();
	}
}
#endif /* UART_USE_HAL */


msz_rc_t stm32g070_write_byte(const uart_dev_t *dev, uint8_t data_byte) {

	msz_rc_t								rc = MSZ_RC_ERR_NOT_FOUND;

#if UART_USE_HAL
	if (dev->hal.Instance->ISR & USART_ISR_TXE_TXFNF) {
		dev->hal.Instance->TDR = data_byte;
#else /* UART_USE_HAL */
	if (dev->msz->usart->ISR & USART_ISR_TXE_TXFNF) {
		dev->msz->usart->TDR = data_byte;
#endif /* UART_USE_HAL */
		rc = MSZ_RC_OK;
	} else {
		rc = MSZ_RC_NOT_READY;
	}

	return rc;
}




#define UART_NUM							1
#define UART_RX_BUFFER_SIZE					1024
#define UART_TX_BUFFER_SIZE					1024

static uart_t *uart_hn[UART_NUM];


static msz_rc_t uart_init_buffer(uart_buffer_t *buf, const bool direction_enable) {

	msz_rc_t								rc = MSZ_RC_OK;

	memset(buf, 0, sizeof(*buf));
	if (direction_enable) {
		buf->data = (uint8_t *)malloc(UART_RX_BUFFER_SIZE);
		if (buf->data) {
			buf->data_write_ptr = buf->data;
			buf->data_read_ptr = buf->data;
			rc = MSZ_RC_OK;
		} else {
			rc = MSZ_RC_ERR_NO_MEMORY;
		}
	}

	return rc;
}

static msz_rc_t uart_rx_proces(uart_dev_t *dev, uart_buffer_t *buf) {

	msz_rc_t								rc = MSZ_RC_OK;
	USART_TypeDef 							*usart = dev->msz->usart;
	uint32_t								uart_isr;
	uint8_t									data_byte;

	uart_isr = usart->ISR;

	if (uart_isr & USART_ISR_FE) {
		usart->CR1 &= ~USART_CR1_RE;
		usart->ICR = USART_ICR_FECF;
		rc = MSZ_RC_ERR_INV_DATA;
	}

	if (uart_isr & USART_ISR_NE) {
		usart->CR1 &= ~USART_CR1_RE;
		usart->ICR = USART_ICR_NECF;
		rc = MSZ_RC_ERR_INV_DATA;
	}

	if (uart_isr & USART_ISR_ORE) {
		usart->CR1 &= ~USART_CR1_RE;
		usart->ICR = USART_ICR_ORECF;
		rc = MSZ_RC_ERR_INV_DATA;
	}

	if (uart_isr & USART_ISR_RTOF) {
		usart->CR1 &= ~USART_CR1_RE;
		usart->ICR = USART_ICR_RTOCF;
		rc = MSZ_RC_ERR_TIMEOUT;
	}

	if (rc == MSZ_RC_OK) {
		if (uart_isr & USART_ISR_RXNE_RXFNE) {
			board_system_board_set_test_pin(0, true);
			data_byte = (uint8_t)usart->RDR;
			if (buf->data_write_ptr) {
				if (buf->data_write_ptr < (buf->data + UART_RX_BUFFER_SIZE)) {
					*buf->data_write_ptr = data_byte;
					buf->data_write_ptr++;
					if (buf->data_write_ptr == (buf->data + UART_RX_BUFFER_SIZE)) {
						usart->CR1 &= ~USART_CR1_RE;
					}
				}
			}
			board_system_board_set_test_pin(0, false);
		}
	}

	return rc;
}

#if UART_USE_FIFO

static msz_rc_t uart_tx_proces_msz_irq(const uart_dev_t *dev, uart_buffer_t *buf) {

	msz_rc_t								rc = MSZ_RC_OK;


		if (dev->msz->usart->ISR & USART_ISR_TXFT) {
			do {
				if (buf->data_read_ptr == buf->data_write_ptr) {
					UART_TEST3_PIN_DOWN();
					dev->msz->usart->CR1 &= ~USART_CR1_TE;
					dev->msz->usart->CR3 &= ~USART_CR3_TXFTIE;
					buf->data_read_ptr = buf->data;
					buf->data_write_ptr = buf->data;
					break;
				} else {

					if (dev->msz->usart->ISR & USART_ISR_TXE_TXFNF) {
						UART_TEST3_PIN_UP();
						dev->msz->usart->CR1 |= USART_CR1_TE;

						dev->msz->usart->TDR = *(buf->data_read_ptr);
						buf->data_read_ptr++;
						dev->msz->usart->CR3 |= USART_CR3_TXFTIE;
					} else {
						break;
					}

				}
			} while (1);
		}

	if (dev->msz->usart->ISR & USART_ISR_TC) {
		dev->msz->usart->ICR = USART_ICR_TCCF;
	}

	return rc;
}

#else /* UART_USE_FIFO */

static msz_rc_t uart_tx_proces_msz_irq(const uart_dev_t *dev, uart_buffer_t *buf) {

	msz_rc_t								rc = MSZ_RC_OK;

	do {
		if (dev->msz->usart->ISR & USART_ISR_TXE_TXFNF) {
			dev->msz->usart->ICR = USART_ICR_TXFECF;
				if (buf->data_read_ptr == buf->data_write_ptr) {
					UART_TEST3_PIN_DOWN();
					dev->msz->usart->CR1 &= ~USART_CR1_TE;
					dev->msz->usart->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
					buf->data_read_ptr = buf->data;
					buf->data_write_ptr = buf->data;
					break;
				} else {
					UART_TEST3_PIN_UP();
					dev->msz->usart->CR1 |= USART_CR1_TE;

					dev->msz->usart->TDR = *(buf->data_read_ptr);
					buf->data_read_ptr++;

					dev->msz->usart->CR1 |= USART_CR1_TXEIE_TXFNFIE;
				}
		} else {
			break;
		}
	} while (1);


	if (dev->msz->usart->ISR & USART_ISR_TC) {
		dev->msz->usart->ICR = USART_ICR_TCCF;
	}

	return rc;
}

#endif /* UART_USE_FIFO */


static msz_rc_t uart_tx_proces(const uart_dev_t *dev, uart_buffer_t *buf) {

	msz_rc_t								rc = MSZ_RC_OK;

	if (buf->data_write_ptr != buf->data) {
#if UART_USE_HAL
#if UART_USE_IRQ
		HAL_UART_Transmit_IT(&dev->hal, (buf->data_read_ptr), buf->data_write_ptr - buf->data_read_ptr);
		buf->data_read_ptr = buf->data;
		buf->data_write_ptr = buf->data;
#else /* UART_USE_IRQ */
		HAL_UART_Transmit(&dev->hal, (buf->data_read_ptr), buf->data_write_ptr - buf->data_read_ptr, 10000);
		buf->data_read_ptr = buf->data;
		buf->data_write_ptr = buf->data;
#endif /* UART_USE_IRQ */

#else /* UART_USE_HAL */

#if UART_USE_IRQ

		uart_tx_proces_msz_irq(dev, buf);
#else /* UART_USE_IRQ */
		do {
#if defined(STM32G431xx)
			rc = stm32g431_write_byte(dev, *(buf->data_read_ptr));
#elif defined(STM32G070xx)
			rc = stm32g070_write_byte(dev, *(buf->data_read_ptr));
#endif /* defined(STM32xxx) */
			if (rc == MSZ_RC_OK) {
				buf->data_read_ptr++;
				if (buf->data_read_ptr == buf->data_write_ptr) {
					buf->data_read_ptr = buf->data;
					buf->data_write_ptr = buf->data;
					break;
				}
			} else {
				break;
			}
		} while (1);
#endif /* UART_USE_IRQ */

#endif /* UART_USE_HAL */
	} else {
		if (dev->msz->usart->ISR & USART_ISR_TXE_TXFNF) {
			dev->msz->usart->ICR = USART_ICR_TXFECF;
		}
		if (dev->msz->usart->ISR & USART_ISR_TC) {
			dev->msz->usart->ICR = USART_ICR_TCCF;
		}
	}

	return rc;
}



int uart_read(uart_t *uart, uint8_t *data, uint16_t data_len, uint32_t timeout_us) {

	int										recv_length = 0;
	uart_buffer_t 							*buf;
	uint16_t								data_in_buffer, data_to_copy;

	if (uart && data && data_len) {
		buf = &uart->rx_buffer;
		if (buf->data_write_ptr == NULL) {
			buf->data_write_ptr = buf->data;
			uart->dev.msz->usart->RQR |= USART_RQR_RXFRQ;
			uart->dev.msz->usart->CR1 |= USART_CR1_RE;
		} else {
			data_in_buffer = 0;
			if (buf->data_write_ptr > buf->data) {
				data_in_buffer = buf->data_write_ptr - buf->data;
			}
			data_to_copy = 0;
			if (data_in_buffer >= data_len) {
				data_to_copy = data_len;
				uart->dev.msz->usart->CR1 &= ~USART_CR1_RE;
			} else if ((uart->dev.msz->usart->CR1 & USART_CR1_RE) == 0) {
				data_to_copy = data_in_buffer;
			}
			if (data_to_copy) {
				memcpy(data, buf->data, data_to_copy);
				uart->rx_buffer.data_write_ptr = NULL;
				recv_length = data_to_copy;
			} else {
				uart->dev.msz->usart->RQR |= USART_RQR_RXFRQ;
				uart->dev.msz->usart->CR1 |= USART_CR1_RE;
			}
		}
	} else {
		recv_length = (int)MSZ_RC_ERR_INV_ARG;
	}

	return recv_length;
}


msz_rc_t uart_write(uart_t *uart, uint8_t *data, uint16_t data_len) {

	msz_rc_t								rc = MSZ_RC_OK;
	uart_buffer_t 							*buf;
	UART_TEST1_PIN_UP();
	if (uart && data && data_len) {
		if (data_len < UART_TX_BUFFER_SIZE) {
			buf = &uart->tx_buffer;
			if ((buf->data_write_ptr + data_len) <= (buf->data + UART_TX_BUFFER_SIZE)) {
				memcpy(buf->data_write_ptr, data, data_len);
				buf->data_write_ptr += data_len;
				uart_tx_proces(&uart->dev, &uart->tx_buffer);
			}
		} else {
			rc = MSZ_RC_ERR_NO_MEMORY;
		}
	} else {
		rc = MSZ_RC_ERR_INV_ARG;
	}
	UART_TEST1_PIN_DOWN();
	return rc;
}




msz_rc_t uart_init(uart_t *uart, const uart_msz_dev_t *msz_dev, uart_conf_t *conf) {

	msz_rc_t									rc = MSZ_RC_OK;
	uint8_t									uart_no;
	UART_TEST1_PIN_UP();
	if (uart && conf) {
		rc = MSZ_RC_ERR_NOT_FOUND;
		for (uart_no = 0; uart_no < UART_NUM; uart_no++) {
			if (uart_hn[uart_no] == NULL) {

				rc = uart_init_buffer(&uart->rx_buffer, conf->rx_enable);
				if (rc != MSZ_RC_OK) {
					break;
				}
				uart->rx_buffer.data_write_ptr = NULL;

				rc = uart_init_buffer(&uart->tx_buffer, conf->tx_enable);
				if (rc != MSZ_RC_OK) {
					break;
				}
				uart->tx_buffer.data_read_ptr = uart->tx_buffer.data;

				uart_hn[uart_no] = uart;
				break;
			}
		}
		if (rc == MSZ_RC_OK) {
#if UART_USE_HAL
			MX_HAL_UART_Init(&uart->dev, msz_dev);
#else /* UART_USE_HAL */
			uart->dev.msz = msz_dev;

#if defined(STM32G431xx)
			rc = stm32g431_uart_init(uart->dev, conf);
#elif defined(STM32G070xx)
			rc = stm32g070_uart_init(&uart->dev, conf);
#endif /*defined(STM32xxx) */
#if UART_USE_IRQ
				NVIC_EnableIRQ(USART2_IRQn);
#endif /* UART_USE_IRQ */
#endif /* UART_USE_HAL */

			if (rc == MSZ_RC_OK) {
				uart->conf = *conf;
				uart->enable = true;
			}
		}
	} else {
		rc = MSZ_RC_ERR_INV_ARG;
	}
	UART_TEST1_PIN_DOWN();
	return rc;
}

msz_rc_t uart_deinit(uart_t *uart) {

	msz_rc_t									rc = MSZ_RC_OK;
	uint8_t									uart_no;

	if (uart) {
		rc = MSZ_RC_ERR_NOT_FOUND;
#if defined(STM32G431xx)
		stm32g431_uart_deinit(uart->dev);
#elif defined(STM32G070xx)
		stm32g070_uart_deinit(&uart->dev);
#endif /*defined(STM32xxx) */
		if (uart->rx_buffer.data) {
			free(uart->rx_buffer.data);
		}
		if (uart->tx_buffer.data) {
			free(uart->tx_buffer.data);
		}
		for (uart_no = 0; uart_no < UART_NUM; uart_no++) {
			if (uart == uart_hn[uart_no]) {
				uart_hn[uart_no] = NULL;
				rc = MSZ_RC_OK;
				break;
			}
		}
		memset(uart, 0, sizeof(*uart));
	} else {
		rc = MSZ_RC_ERR_INV_ARG;
	}

	return rc;
}

void uart_irq(USART_TypeDef	*usart) {

#if UART_USE_IRQ
	uint8_t									uart_no;
	uart_t									*u_hn;
	UART_TEST2_PIN_UP();
	for (uart_no = 0; uart_no < UART_NUM; uart_no++) {
		u_hn = uart_hn[uart_no];
		if (u_hn) {
#if UART_USE_HAL
			if (usart == u_hn->dev.hal.Instance) {
				HAL_UART_IRQHandler(&u_hn->dev.hal);
				break;
			}
#else /* UART_USE_HAL */
			if (usart == u_hn->dev.msz->usart) {
				if (u_hn->conf.rx_enable && u_hn->rx_buffer.data) {
					uart_rx_proces(&u_hn->dev, &u_hn->rx_buffer);
				}
				if (u_hn->conf.tx_enable && u_hn->tx_buffer.data) {
					uart_tx_proces(&u_hn->dev, &u_hn->tx_buffer);
				} else {
					if (u_hn->dev.msz->usart->ISR & USART_ISR_TXE_TXFNF) {
						u_hn->dev.msz->usart->ICR = USART_ICR_TXFECF;
					}
					if (u_hn->dev.msz->usart->ISR & USART_ISR_TC) {
						u_hn->dev.msz->usart->ICR = USART_ICR_TCCF;
					}
					if (u_hn->dev.msz->usart->ISR & USART_ISR_TXFT) {
						u_hn->dev.msz->usart->CR3 &= ~USART_CR3_TXFTIE;
					}
				}
				break;
			}
#endif /* UART_USE_HAL */
		}
	}
	UART_TEST2_PIN_DOWN();
#endif /* UART_USE_IRQ */
}

void uart_dma_irq(USART_TypeDef	*usart) {

	uint8_t									uart_no;
	uart_t									*u_hn;

	for (uart_no = 0; uart_no < UART_NUM; uart_no++) {
		u_hn = uart_hn[uart_no];
		if (usart == u_hn->dev.msz->usart) {
			if (u_hn->conf.rx_enable && u_hn->rx_buffer.data) {
				uart_rx_proces(&u_hn->dev, &u_hn->rx_buffer);
			}
			break;
		}
	}
}

void uart_proces(void) {

	uint8_t									uart_no;
	uart_t									*u_hn;

	for (uart_no = 0; uart_no < UART_NUM; uart_no++) {
		u_hn = uart_hn[uart_no];
		if (u_hn && u_hn->enable) {
			if (u_hn->conf.rx_enable && u_hn->rx_buffer.data) {
			//	uart_rx_proces(u_hn->dev, 0, &u_hn->rx_buffer);
			}
			if (u_hn->conf.tx_enable && u_hn->tx_buffer.data) {
				uart_tx_proces(&u_hn->dev, &u_hn->tx_buffer);
			}
		}
	}
}
