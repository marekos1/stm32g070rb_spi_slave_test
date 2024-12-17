/*
 * msz_t200_spi_slave.c
 *
 *  Created on: Nov 12, 2024
 *      Author: marek
 */

#include "slave_regs/msz_t200_spi_slave/msz_t200_spi_slave_proto_defs.h"
#include "slave_regs/slave_regs.h"
#if CONFIG_SPI_SLAVE_CS_IRQ
#include "gpio/gpio.h"
#endif /* CONFIG_SPI_SLAVE_CS_IRQ */
#include "trace/trace.h"
#include "main.h"





static SPI_HandleTypeDef hspi1;


static const uint8_t 						msz_t200_const_header_recponce_data_tx[MSZ_T200_CONST_HEADER_LENGTH] = {0xA0, 0xA1};
static const uint8_t						msz_t200_var_header_recponce_data_tx[MSZ_T200_VAR_HEADER_LENGTH] = {0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5};









#if 1

#define SPI_SLAVE_TEST1_PIN_UP() 		HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_SET)
#define SPI_SLAVE_TEST1_PIN_DOWN() 		HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_RESET)
#define SPI_SLAVE_TEST1_PIN_TOGGLE()  	HAL_GPIO_TogglePin(TEST1_GPIO_Port, TEST1_Pin);

#define SPI_SLAVE_TEST2_PIN_UP() 		HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, GPIO_PIN_SET)
#define SPI_SLAVE_TEST2_PIN_DOWN() 		HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, GPIO_PIN_RESET)
#define SPI_SLAVE_TEST2_PIN_TOGGLE()	HAL_GPIO_TogglePin(TEST2_GPIO_Port, TEST2_Pin);

#define SPI_SLAVE_TEST3_PIN_UP() 		HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_SET)
#define SPI_SLAVE_TEST3_PIN_DOWN() 		HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_RESET)
#define SPI_SLAVE_TEST3_PIN_TOGGLE()	HAL_GPIO_TogglePin(TEST3_GPIO_Port, TEST3_Pin);

#else

#define SPI_SLAVE_TEST1_PIN_UP()
#define SPI_SLAVE_TEST1_PIN_DOWN()
#define SPI_SLAVE_TEST1_PIN_TOGGLE()

#define SPI_SLAVE_TEST2_PIN_UP()
#define SPI_SLAVE_TEST2_PIN_DOWN()
#define SPI_SLAVE_TEST2_PIN_TOGGLE()

#define SPI_SLAVE_TEST3_PIN_UP()
#define SPI_SLAVE_TEST3_PIN_DOWN()
#define SPI_SLAVE_TEST3_PIN_TOGGLE()

#endif

#if 0

#define SPI_SLAVE_SS_IRQ_TEST1_PIN_UP() 		HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_SET)
#define SPI_SLAVE_SS_IRQ_TEST1_PIN_DOWN() 		HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_RESET)
#define SPI_SLAVE_SS_IRQ_TEST1_PIN_TOGGLE()  	HAL_GPIO_TogglePin(TEST1_GPIO_Port, TEST1_Pin);

#define SPI_SLAVE_SS_IRQ_TEST2_PIN_UP() 		HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, GPIO_PIN_SET)
#define SPI_SLAVE_SS_IRQ_TEST2_PIN_DOWN() 		HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, GPIO_PIN_RESET)
#define SPI_SLAVE_SS_IRQ_TEST2_PIN_TOGGLE()		HAL_GPIO_TogglePin(TEST2_GPIO_Port, TEST2_Pin);

#define SPI_SLAVE_SS_IRQ_TEST3_PIN_UP() 		HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_SET)
#define SPI_SLAVE_SS_IRQ_TEST3_PIN_DOWN() 		HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_RESET)
#define SPI_SLAVE_SS_IRQ_TEST3_PIN_TOGGLE()		HAL_GPIO_TogglePin(TEST3_GPIO_Port, TEST3_Pin);

#else

#define SPI_SLAVE_SS_IRQ_TEST1_PIN_UP()
#define SPI_SLAVE_SS_IRQ_TEST1_PIN_DOWN()
#define SPI_SLAVE_SS_IRQ_TEST1_PIN_TOGGLE()

#define SPI_SLAVE_SS_IRQ_TEST2_PIN_UP()
#define SPI_SLAVE_SS_IRQ_TEST2_PIN_DOWN()
#define SPI_SLAVE_SS_IRQ_TEST2_PIN_TOGGLE()

#define SPI_SLAVE_SS_IRQ_TEST3_PIN_UP()
#define SPI_SLAVE_SS_IRQ_TEST3_PIN_DOWN()
#define SPI_SLAVE_SS_IRQ_TEST3_PIN_TOGGLE()

#endif




static uint32_t msz_t200_spi_get_32bdata_value(const uint8_t *data) {

	uint32_t								_32bdata_value = 0;

	_32bdata_value |= (uint32_t)(*(data + 0) << 24);
	_32bdata_value |= (uint32_t)(*(data + 1) << 16);
	_32bdata_value |= (uint32_t)(*(data + 2) << 8);
	_32bdata_value |= (uint32_t)(*(data + 3) << 0);

	return _32bdata_value;
}

static void msz_t200_spi_set_32bdata_value(uint8_t *data, const uint32_t value) {

	*(data + 0) = (uint8_t)(value >> 24);
	*(data + 1) = (uint8_t)(value >> 16);
	*(data + 2) = (uint8_t)(value >> 8);
	*(data + 3) = (uint8_t)(value >> 0);
}

static uint32_t msz_t200_spi_get_reg_addr(const uint8_t *data) {

	uint32_t								reg_addr = 0;

	reg_addr = msz_t200_spi_get_32bdata_value(data + 2);
	reg_addr &= 0x00FFFFFF;

	return reg_addr;
}

static bool msz_t200_spi_is_write_operation(const uint8_t *data) {

	bool									wr_operation = false;

	if (*(data + 6) == 0x80) {
		wr_operation = true;
	}

	return wr_operation;
}

static uint8_t msz_t200_spi_get_count_of_operations(const uint8_t *data) {

	uint8_t									number_of_operation = 0;

	number_of_operation = *(data + 7);

	return number_of_operation;
}

static uint32_t msz_t200_spi_get_crc32(uint8_t *data) {

	uint32_t								crc32;

	crc32 = msz_t200_spi_get_32bdata_value(data);

	return crc32;
}

static HAL_StatusTypeDef my_SPI_WaitFifoStateUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Fifo, uint32_t State, uint32_t Timeout, uint32_t Tickstart) {

	__IO uint32_t count;
	uint32_t tmp_timeout;
	uint32_t tmp_tickstart;
	__IO uint8_t *ptmpreg8;
	__IO uint8_t tmpreg8 = 0;

	/* Adjust Timeout value  in case of end of transfer */
	tmp_timeout = Timeout - (HAL_GetTick() - Tickstart);
	tmp_tickstart = HAL_GetTick();

	/* Initialize the 8bit temporary pointer */
	ptmpreg8 = (__IO uint8_t*) &hspi->Instance->DR;

	/* Calculate Timeout based on a software loop to avoid blocking issue if Systick is disabled */
	count = tmp_timeout * ((SystemCoreClock * 35U) >> 20U);

	while ((hspi->Instance->SR & Fifo) != State) {
		if ((Fifo == SPI_SR_FRLVL) && (State == SPI_FRLVL_EMPTY)) {
			/* Flush Data Register by a blank read */
			tmpreg8 = *ptmpreg8;
			/* To avoid GCC warning */
			UNUSED(tmpreg8);
		}

		if (Timeout != HAL_MAX_DELAY) {
			if (((HAL_GetTick() - tmp_tickstart) >= tmp_timeout)
					|| (tmp_timeout == 0U)) {
				/* Disable the SPI and reset the CRC: the CRC value should be cleared
				 on both master and slave sides in order to resynchronize the master
				 and slave for their respective CRC calculation */

				/* Disable TXE, RXNE and ERR interrupts for the interrupt process */
				__HAL_SPI_DISABLE_IT(hspi,
						(SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

				if ((hspi->Init.Mode == SPI_MODE_MASTER)
						&& ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
								|| (hspi->Init.Direction
										== SPI_DIRECTION_2LINES_RXONLY))) {
					/* Disable SPI peripheral */
					__HAL_SPI_DISABLE(hspi);
				}

				/* Reset CRC Calculation */
				if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE) {
					SPI_RESET_CRC(hspi);
				}

				hspi->State = HAL_SPI_STATE_READY;

				/* Process Unlocked */
				__HAL_UNLOCK(hspi);

				return HAL_TIMEOUT;
			}
			/* If Systick is disabled or not incremented, deactivate timeout to go in disable loop procedure */
			if (count == 0U) {
				tmp_timeout = 0U;
			}
			count--;
		}
	}

	return HAL_OK;
}

static HAL_StatusTypeDef my_SPI_WaitFlagStateUntilTimeout(SPI_HandleTypeDef *hspi,
		uint32_t Flag, FlagStatus State, uint32_t Timeout, uint32_t Tickstart) {
	__IO uint32_t count;
	uint32_t tmp_timeout;
	uint32_t tmp_tickstart;

	/* Adjust Timeout value  in case of end of transfer */
	tmp_timeout = Timeout - (HAL_GetTick() - Tickstart);
	tmp_tickstart = HAL_GetTick();

	/* Calculate Timeout based on a software loop to avoid blocking issue if Systick is disabled */
	count = tmp_timeout * ((SystemCoreClock * 32U) >> 20U);

	while ((__HAL_SPI_GET_FLAG(hspi, Flag) ? SET : RESET) != State) {
		if (Timeout != HAL_MAX_DELAY) {
			if (((HAL_GetTick() - tmp_tickstart) >= tmp_timeout)
					|| (tmp_timeout == 0U)) {
				/* Disable the SPI and reset the CRC: the CRC value should be cleared
				 on both master and slave sides in order to resynchronize the master
				 and slave for their respective CRC calculation */

				/* Disable TXE, RXNE and ERR interrupts for the interrupt process */
				__HAL_SPI_DISABLE_IT(hspi,
						(SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

				if ((hspi->Init.Mode == SPI_MODE_MASTER)
						&& ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
								|| (hspi->Init.Direction
										== SPI_DIRECTION_2LINES_RXONLY))) {
					/* Disable SPI peripheral */
					__HAL_SPI_DISABLE(hspi);
				}

				/* Reset CRC Calculation */
				if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE) {
					SPI_RESET_CRC(hspi);
				}

				hspi->State = HAL_SPI_STATE_READY;

				/* Process Unlocked */
				__HAL_UNLOCK(hspi);

				return HAL_TIMEOUT;
			}
			/* If Systick is disabled or not incremented, deactivate timeout to go in disable loop procedure */
			if (count == 0U) {
				tmp_timeout = 0U;
			}
			count--;
		}
	}

	return HAL_OK;
}

static HAL_StatusTypeDef msz_t200_spi_end_txrx_operation(SPI_HandleTypeDef *hspi, uint32_t Timeout, uint32_t Tickstart) {

	/* Control if the TX fifo is empty */
	if (my_SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FTLVL, SPI_FTLVL_EMPTY,	Timeout, Tickstart) != HAL_OK) {
		SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
		__HAL_SPI_DISABLE(hspi);
		return HAL_TIMEOUT;
	}

	/* Control the BSY flag */
	if (my_SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_BSY, RESET, Timeout, Tickstart) != HAL_OK) {
		SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
		__HAL_SPI_DISABLE(hspi);
		return HAL_TIMEOUT;
	}

	/* Control if the RX fifo is empty */
	if (my_SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY,	Timeout, Tickstart) != HAL_OK) {
		SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
		__HAL_SPI_DISABLE(hspi);
		return HAL_TIMEOUT;
	}
	__HAL_SPI_DISABLE(hspi);

	return HAL_OK;
}

static HAL_StatusTypeDef my_SPI_Start_RxTxTransaction(SPI_HandleTypeDef *hspi, uint32_t *tickstart) {

	HAL_StatusTypeDef 						errorcode = HAL_OK;
	HAL_SPI_StateTypeDef					tmp_state;
	uint32_t 								tmp_mode;

	assert_param(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

	/* Init tickstart for timeout management*/
	*tickstart = HAL_GetTick();

	/* Init temporary variables */
	tmp_state = hspi->State;
	tmp_mode = hspi->Init.Mode;


	if (!((tmp_state == HAL_SPI_STATE_READY) || ((tmp_mode == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES) && (tmp_state == HAL_SPI_STATE_BUSY_RX)))) {
		errorcode = HAL_BUSY;
		goto error;
	}

	/* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
	if (hspi->State != HAL_SPI_STATE_BUSY_RX) {
		hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
	}

	error: hspi->State = HAL_SPI_STATE_READY;

	/* Set the Rx Fifo threshold */
		/* Set fiforxthreshold according the reception data length: 8bit */
	SET_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);

	/* Check if the SPI is already enabled */
	if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) {
		/* Enable SPI peripheral */
		__HAL_SPI_ENABLE(hspi);
	}

	return errorcode;
}

static msz_rc_t msz_t200_spi_data_transfer(SPI_HandleTypeDef *hspi, const uint8_t *data_write, uint8_t *data_read, const uint32_t data_length, const uint32_t timeout_ms) {

	msz_rc_t								rc = MSZ_RC_OK;
	uint32_t 								tx_ctr, rx_ctr;
	uint32_t								start_time;

	if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY)) {

	}
	if (timeout_ms) {
		start_time = system_ms_time();
	}

	tx_ctr = 0;
	rx_ctr = 0;

	do {
		if (hspi->Instance->SR & SPI_SR_UDR) {

		}
		if (hspi->Instance->SR & SPI_SR_OVR) {
			SPI_SLAVE_TEST3_PIN_UP();
			rc = MSZ_RC_SPI_SLAVE_ERR_OVERRUN;
			do {
				hspi->Instance->DR;
			} while (hspi->Instance->SR & SPI_SR_RXNE);
			break;
		}
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) {	//CS up by MASTER
			rc = MSZ_RC_SPI_SLAVE_MASTER_FINISH_TRANSACTION;
			break;
		}


		if (hspi->Instance->SR & SPI_SR_TXE) {
			if (tx_ctr < data_length) {
				SPI_SLAVE_TEST1_PIN_DOWN();
				if (data_write) {
					*((__IO uint8_t*) &hspi->Instance->DR) = *(data_write + tx_ctr);
				} else {
					*((__IO uint8_t*) &hspi->Instance->DR) = 0xCF;
				}
				tx_ctr++;
			}
		}
		SPI_SLAVE_TEST1_PIN_UP();

		if (hspi->Instance->SR & SPI_SR_RXNE) {
			SPI_SLAVE_TEST2_PIN_DOWN();
			if (data_read) {
				*(data_read + rx_ctr) = *(__IO uint8_t*) &hspi->Instance->DR;
			} else {
				hspi->Instance->DR;
			}
			rx_ctr++;
		}
		SPI_SLAVE_TEST2_PIN_UP();

		if (timeout_ms) {
			if (system_ms_time() > (start_time + timeout_ms)) {
				rc = MSZ_RC_ERR_TIMEOUT;
				break;
			}
		}
		SPI_SLAVE_TEST3_PIN_DOWN();
	} while ((tx_ctr < data_length) || (rx_ctr < data_length));

	return rc;
}

static uint32_t msz_t200_crc32_calc(uint8_t *data, const uint32_t data_len) {

	uint32_t								crc32 = 0;
	uint32_t								byte_no;

	for (byte_no = 0; byte_no < data_len; byte_no++) {
		crc32 += *(data + byte_no);
	}

	return crc32;
}

static msz_rc_t msz_t200_spi_recv_header_const_value(SPI_HandleTypeDef *hspi, uint8_t *spi_data, const uint16_t spi_data_idx) {

	msz_rc_t								rc;

	rc = msz_t200_spi_data_transfer(hspi, msz_t200_const_header_recponce_data_tx, spi_data + spi_data_idx, MSZ_T200_CONST_HEADER_LENGTH, 10);
	if (rc == MSZ_RC_OK) {
		if ((*(spi_data + spi_data_idx + 0) != MSZ_T200_CONST_HEADER_FIRST_BYTE) || (*(spi_data + spi_data_idx + 1) != MSZ_T200_CONST_HEADER_SECOND_BYTE)) {
			rc = MSZ_RC_SPI_SLAVE_INV_CONST_HEADER;
		}
	}

	return rc;
}

static msz_rc_t msz_t200_spi_header_variable_validate(const uint8_t *spi_data, const uint16_t spi_data_idx) {

	msz_rc_t							rc = MSZ_RC_OK;

	return rc;
}

static msz_rc_t msz_t200_spi_recv_header_variable_value(SPI_HandleTypeDef *hspi, uint8_t *spi_data, const uint16_t spi_data_idx) {

	msz_rc_t								rc;

	rc = msz_t200_spi_data_transfer(hspi, msz_t200_var_header_recponce_data_tx, spi_data + spi_data_idx, MSZ_T200_VAR_HEADER_LENGTH, 10);
	if (rc == MSZ_RC_OK) {
		rc = msz_t200_spi_header_variable_validate(spi_data, spi_data_idx);
	}

	return rc;
}

static void msz_t200_spi_write_register(const uint32_t reg_addr, const uint32_t reg_value) {

	slave_irq_write_reg(reg_addr, reg_value);
}

static uint32_t msz_t200_spi_read_register(const uint32_t reg_addr) {

	uint32_t								reg_value;

	slave_irq_read_reg(reg_addr, &reg_value);

	return reg_value;
}

static void msz_t200_spi_do_write_operation(const uint8_t *data, const uint32_t data_length, const uint32_t reg_addr, const uint8_t operation_count) {

	uint32_t								operation_no, reg_value;

	for (operation_no = 0; operation_no < operation_count; operation_no++) {
		reg_value = msz_t200_spi_get_32bdata_value(data + (operation_no * 4));
		T_DG_SPISL("Wr %8u <- 0x%08X", reg_addr + operation_no, reg_value);
		msz_t200_spi_write_register(reg_addr + operation_no, reg_value);
	}
}

static uint32_t msz_t200_spi_do_read_operation(uint8_t *data, const uint32_t reg_addr, const uint32_t operation_count) {

	uint32_t								read_data_length = 0;
	uint32_t								operation_no, reg_value;

	for (operation_no = 0; operation_no < operation_count; operation_no++) {
		reg_value = msz_t200_spi_read_register(reg_addr + operation_no);
		msz_t200_spi_set_32bdata_value(data + (operation_no * 4), reg_value);
		read_data_length += 4;
	}

	return read_data_length;
}

static msz_rc_t msz_t200_spi_handle_write_operation(SPI_HandleTypeDef *hspi, uint8_t *data, const uint32_t data_idx, const uint8_t reg_count) {

	msz_rc_t								rc = MSZ_RC_OK;
	uint32_t								read_data_bytes = reg_count * 4 + 4, recv_crc32, calc_crc32;

	rc = msz_t200_spi_data_transfer(hspi, NULL, data + data_idx, read_data_bytes, 10);
	if (rc == MSZ_RC_OK) {
		recv_crc32 = msz_t200_spi_get_crc32(data + data_idx + read_data_bytes - 4);
		calc_crc32 = msz_t200_crc32_calc(data, data_idx + read_data_bytes - 4);
		if (recv_crc32 != calc_crc32) {
			rc = MSZ_RC_ERR_INV_CRC;
		}
	}

	return rc;
}

static msz_rc_t msz_t200_spi_handle_read_operation(SPI_HandleTypeDef *hspi, uint8_t *data, const uint32_t data_idx) {

	msz_rc_t								rc;
	uint32_t								write_idx = 0, calc_crc32;

	calc_crc32 = msz_t200_crc32_calc(data, data_idx);
	msz_t200_spi_set_32bdata_value(data + data_idx, calc_crc32);
	write_idx += 4;
	rc = msz_t200_spi_data_transfer(hspi, data + 8, NULL, data_idx - 8 + write_idx, 10);

	return rc;
}

static msz_rc_t msz_t200_spi_recv_header(SPI_HandleTypeDef *hspi, uint8_t *spi_data, const uint16_t spi_data_idx) {

	msz_rc_t								rc;
	uint16_t								recv_header_data_idx = 0;

/* TODO inicjacja SPI do odbioru i załadowanie pierwszych 2 bajtów do FIFO SPI można zrobić już w przerwaniu. */
	rc = msz_t200_spi_recv_header_const_value(hspi, spi_data, spi_data_idx + recv_header_data_idx);
	if (rc == MSZ_RC_OK) {
		SPI_SLAVE_TEST2_PIN_UP();
		recv_header_data_idx += MSZ_T200_CONST_HEADER_LENGTH;
		rc = msz_t200_spi_recv_header_variable_value(hspi, spi_data, spi_data_idx + recv_header_data_idx);
	}

	return rc;
}

static msz_rc_t msz_t200_spi_operation(SPI_HandleTypeDef *hspi, bool *is_write_operation) {

	msz_rc_t								rc;
	uint32_t								spi_data_idx = 0, tickstart, reg_addr;
	uint8_t									spi_data[2 + 6 + 255 * 4 + 4], operation_count;

	SPI_SLAVE_TEST1_PIN_UP();
	my_SPI_Start_RxTxTransaction(hspi, &tickstart);
	rc = msz_t200_spi_recv_header(hspi, spi_data, spi_data_idx);

	if (rc == MSZ_RC_OK) {
		spi_data_idx += MSZ_T200_CONST_HEADER_LENGTH + MSZ_T200_VAR_HEADER_LENGTH;
		*is_write_operation = msz_t200_spi_is_write_operation(spi_data);
		reg_addr = msz_t200_spi_get_reg_addr(spi_data);
		operation_count = msz_t200_spi_get_count_of_operations(spi_data);

		if (*is_write_operation) {
			rc = msz_t200_spi_handle_write_operation(hspi, spi_data, spi_data_idx, operation_count);
			if (rc == MSZ_RC_OK) {
				msz_t200_spi_do_write_operation(&spi_data[8], spi_data_idx, reg_addr, operation_count);
			} else {
				T_DG_SPISL("Write operation fail rc: %d", rc);
			}
		} else {
			spi_data_idx += msz_t200_spi_do_read_operation(&spi_data[8], reg_addr, operation_count);
			rc = msz_t200_spi_handle_read_operation(hspi, spi_data, spi_data_idx);
		}
//		T_DG_SPISL("Operation %7s Base addr: %8u, number %u", *is_write_operation ? "Write" : "Read", reg_addr, operation_count);
		if (rc != MSZ_RC_OK) {
			if (msz_t200_spi_end_txrx_operation(hspi, 1, tickstart) != HAL_OK) {
				hspi->ErrorCode = HAL_SPI_ERROR_FLAG;
			}
		}
	}

	return rc;
}

static void msz_t200_spi_clear_rx_fifo(SPI_HandleTypeDef *hspi) {

	do {
		hspi->Instance->DR;
	} while(__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE));
}



/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	RCC->APBRSTR2 |= RCC_APBRSTR2_SPI1RST;
	RCC->APBRSTR2 &= ~(RCC_APBRSTR2_SPI1RST);

	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_SLAVE;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
}



static bool volatile irq_state;
static bool volatile irq_input_state;

void msz_t200_spi_slave_input_irq(bool active) {

#if CONFIG_SPI_SLAVE_CS_IRQ
	irq_state = true;
	irq_input_state = active;
	if (irq_input_state) {
		SPI_SLAVE_SS_IRQ_TEST1_PIN_UP();
	}
#endif /* CONFIG_SPI_SLAVE_CS_IRQ */
}



bool msz_t200_spi_slave_poll(void) {

	msz_rc_t								rc;
	bool									wr_operation;

#if CONFIG_SPI_SLAVE_CS_IRQ

	SPI_SLAVE_SS_IRQ_TEST3_PIN_DOWN();
	if (irq_state) {
		irq_state = false;

		if (irq_input_state) {
			SPI_SLAVE_SS_IRQ_TEST2_PIN_UP();
			SPI_SLAVE_SS_IRQ_TEST1_PIN_DOWN();
			rc = msz_t200_spi_operation(&hspi1, &wr_operation);
			SPI_SLAVE_SS_IRQ_TEST2_PIN_DOWN();
			if (rc == MSZ_RC_OK) {
				if (wr_operation) {
					SPI_SLAVE_SS_IRQ_TEST3_PIN_UP();
				}
			} else {
				MX_SPI1_Init();
			}
			irq_input_state = false;
		} else {
			if (hspi1.Instance->SR & SPI_SR_OVR) {
				SPI_SLAVE_TEST3_PIN_UP();
				do {
					hspi1.Instance->DR;
				} while (hspi1.Instance->SR & SPI_SR_RXNE);
				SPI_SLAVE_TEST3_PIN_DOWN();
			}
		}
	}

#else /* CONFIG_SPI_SLAVE_CS_IRQ */

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) {
		TEST2_POOL_PIN_UP();
		msz_t200_spi_operation(&hspi1);
		TEST2_POOL_PIN_DOWN();
	}

#endif /* CONFIG_SPI_SLAVE_CS_IRQ */

	return wr_operation;
}

void msz_t200_spi_slave_generate_irq(const bool irq) {

	T_DG_SPISL("Enter irq: %u", irq);
#if CONFIG_SPI_SLAVE_REGS_GENERATE_IRQ
	HAL_GPIO_WritePin(SPI_SLAVE_IRQ_REQUEST_GPIO_Port, SPI_SLAVE_IRQ_REQUEST_Pin, irq ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif /* CONFIG_SPI_SLAVE_REGS_GENERATE_IRQ */
}

msz_rc_t msz_t200_spi_slave_init(void) {

	msz_rc_t								rc = MSZ_RC_OK;
	GPIO_InitTypeDef 						GPIO_InitStruct = { 0 };

#if CONFIG_SPI_SLAVE_REGS_GENERATE_IRQ

	HAL_GPIO_WritePin(SPI_SLAVE_IRQ_REQUEST_GPIO_Port, SPI_SLAVE_IRQ_REQUEST_Pin, GPIO_PIN_SET);
	GPIO_InitStruct.Pin = SPI_SLAVE_IRQ_REQUEST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI_SLAVE_IRQ_REQUEST_GPIO_Port, &GPIO_InitStruct);

#endif /* CONFIG_SPI_SLAVE_REGS_GENERATE_IRQ */

#if CONFIG_SPI_SLAVE_CS_IRQ

	gpio_init_as_input(SPI_SLAVE_CHIP_SELECT_GPIO_Port, SPI_SLAVE_CHIP_SELECT_Pin, GPIO_SPEED_HIGH, GPIO_PULL_UP);
	NVIC_EnableIRQ(EXTI4_15_IRQn);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = SPI_SLAVE_CHIP_SELECT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT | GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI_SLAVE_CHIP_SELECT_GPIO_Port, &GPIO_InitStruct);

#else /* CONFIG_SPI_SLAVE_CS_IRQ */

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };


	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif /* CONFIG_SPI_SLAVE_CS_IRQ */


	MX_SPI1_Init();
	msz_t200_spi_clear_rx_fifo(&hspi1);

	return rc;
}




