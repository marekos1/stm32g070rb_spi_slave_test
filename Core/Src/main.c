/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdint.h>

#include "main.h"

#include "spi_slave.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef int bool;
const bool true = 1;
const bool false = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#if 1

#define TEST1_PIN_UP() 		HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_SET)
#define TEST1_PIN_DOWN() 	HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_RESET)
#define TEST1_PIN_TOGGLE()  HAL_GPIO_TogglePin(TEST1_GPIO_Port, TEST1_Pin);

#define TEST2_PIN_UP() 		HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, GPIO_PIN_SET)
#define TEST2_PIN_DOWN() 	HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, GPIO_PIN_RESET)
#define TEST2_PIN_TOGGLE()	HAL_GPIO_TogglePin(TEST2_GPIO_Port, TEST2_Pin);

#define TEST3_PIN_UP() 		HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_SET)
#define TEST3_PIN_DOWN() 	HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_RESET)
#define TEST3_PIN_TOGGLE()	HAL_GPIO_TogglePin(TEST3_GPIO_Port, TEST3_Pin);

#else

#define TEST1_PIN_UP()
#define TEST1_PIN_DOWN()

#define TEST2_PIN_UP()
#define TEST2_PIN_DOWN()

#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const char *enter_txt = "S";
const char *spi_ok_txt = "O";
const char *spi_err_txt = "E";
const char *spi_hdr_txt = "H";
const char *spi_data_txt = "D";

const char *spi_hdr1_txt = "1";
const char *spi_hdr2_txt = "2";
/* USER CODE END 0 */

#if 0
static const uint8_t msz_t200_header_first = 0x4D;
static const uint8_t msz_t200_header_second = 0xD3;
static const uint32_t msz_t200_register_number = 0x01FFFFFF;
static const uint8_t msz_t200_single_operation_word_length_max = 128;

static const uint8_t msz_t200_header_size = 8;
static const uint8_t msz_t200_dummy_size = 4;
static const uint8_t msz_t200_crc32_size = 4;

static const uint32_t msz_t200_rx_buffer_size = msz_t200_header_size + msz_t200_dummy_size + (msz_t200_single_operation_word_length_max * 4) + msz_t200_crc32_size;
#else

#define MSZ_T200_HEADER_FIRST_BYTE 					0xD3
#define MSZ_T200_HEADER_SECOND_BYTE 				0x4D

#define MSZ_T200_REGISTER_NUMBER					0x01FFFFFF
#define MSZ_T200_SINGLE_OPERATION_WORD_LENGTH_MAX 	128

#define MSZ_T200_HEADER_SIZE						8
#define MSZ_T200_DUMMY_SIZE							4
#define MSZ_T200_CRC32_SIZE							4

#define MSZ_T200_BUFFER_SIZE						(MSZ_T200_HEADER_SIZE + MSZ_T200_DUMMY_SIZE + (MSZ_T200_SINGLE_OPERATION_WORD_LENGTH_MAX * 4 ) + MSZ_T200_CRC32_SIZE)
#endif


//static uint8_t msz_t200_recv_buf[MSZ_T200_BUFFER_SIZE];
//static uint8_t msz_t200_send_buf[MSZ_T200_BUFFER_SIZE];


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

static HAL_StatusTypeDef my_SPI_WaitFifoStateUntilTimeout(SPI_HandleTypeDef *hspi,
		uint32_t Fifo, uint32_t State, uint32_t Timeout, uint32_t Tickstart) {
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

static HAL_StatusTypeDef my_SPI_End_RxTxTransaction(SPI_HandleTypeDef *hspi, uint32_t Timeout, uint32_t Tickstart) {

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

typedef enum {
	MSZ_T200_RC_OK = 0,
	MSZ_T200_RC_TIMEOUT = -1,
	MSZ_T200_RC_INV_HEADER = -2,
	MSZ_T200_RC_ERR = -3,

} msz_t200_rc_t;

#define MSZ_T200_DUMMY_BYTE 0xFF
#define MSZ_T200_SUPPORT_TIMEOUT 0


static volatile uint32_t spi_busy_ctr;
static volatile uint32_t spi_over_ctr;
static volatile uint32_t spi_under_ctr;



#if 0

static msz_t200_rc_t msz_t200_spi_byte_transfer(SPI_HandleTypeDef *hspi, const uint8_t *data_write, uint8_t *data_read, const uint32_t timeout) {

	msz_t200_rc_t							rc = MSZ_T200_RC_OK;
	bool									tx, rx;
#if MSZ_T200_SUPPORT_TIMEOUT
	uint32_t								end_time = HAL_GetTick() + timeout;
#else
	(void)timeout;
#endif

	tx = false;
	rx = false;
	TEST1_PIN_UP();
	if (data_write) {
		if (*data_write == MSZ_T200_DUMMY_BYTE) {
			tx = true;
		}
	} else {

	}

	if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY)) {
		spi_busy_ctr++;
	}
	if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_OVR)) {
		spi_over_ctr++;
	}
	if (__HAL_SPI_GET_FLAG(hspi, SPI_SR_UDR)) {
		spi_under_ctr++;
	}

	do {
		if (tx == false) {
			if ((hspi->Instance->SR & SPI_SR_FTLVL) == 0) {
				*((__IO uint8_t*) &hspi->Instance->DR) = *data_write;
				tx = true;
			}
		}
		if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) {
			TEST2_PIN_UP();
			*data_read = *(__IO uint8_t*) &hspi->Instance->DR;
			rx = true;
			TEST2_PIN_DOWN();
		}
#if MSZ_T200_SUPPORT_TIMEOUT
		if (timeout && (end_time < HAL_GetTick()) && (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY) == 0)) {
			rc = MSZ_T200_RC_TIMEOUT;
			break;
		}
#endif
	} while ((tx == false) || (rx == false));
	TEST1_PIN_DOWN();

	return rc;
}

static msz_t200_rc_t msz_t200_spi_data_transfer(SPI_HandleTypeDef *hspi, uint8_t *data_write, uint8_t *data_read, const uint32_t data_write_length, const uint32_t timeout) {

	msz_t200_rc_t							rc = MSZ_T200_RC_OK;
	uint32_t								byte_no;
	uint8_t									dummy_data;

	if (data_write) {
		if (data_read) {
			for (byte_no = 0; byte_no < data_write_length; byte_no++) {
				rc = msz_t200_spi_byte_transfer(hspi, data_write + byte_no, data_read + byte_no, timeout);
			}
		} else {
			for (byte_no = 0; byte_no < data_write_length; byte_no++) {
				rc = msz_t200_spi_byte_transfer(hspi, data_write + byte_no, &dummy_data, timeout);
			}
		}
	} else {
		if (data_read) {
			for (byte_no = 0; byte_no < data_write_length; byte_no++) {
				rc = msz_t200_spi_byte_transfer(hspi, NULL, data_read + byte_no, timeout);
			}
		} else {
			for (byte_no = 0; byte_no < data_write_length; byte_no++) {
				rc = msz_t200_spi_byte_transfer(hspi, NULL, &dummy_data, timeout);
			}
		}
	}

	return rc;
}

#else

bool test_bsy = false;
bool test_ovr = false;
bool test_udr = false;
bool test_bsy_tx_not_empty = false;

#define TEST_BSY_ON_TEST2_PIN_UP() if (test_bsy) { TEST2_PIN_UP(); }
#define TEST_OVR_ON_TEST2_PIN_UP() if (test_ovr) { TEST2_PIN_UP(); }
#define TEST_UDR_ON_TEST2_PIN_UP() if (test_udr) { TEST2_PIN_UP(); }

#define TEST_BSY_TX_NOT_EMPTY_ON_TEST2_PIN_UP() if (test_bsy_tx_not_empty) { TEST2_PIN_UP(); }


static msz_t200_rc_t msz_t200_spi_data_transfer(SPI_HandleTypeDef *hspi, const uint8_t *data_write, uint8_t *data_read, const uint32_t data_write_length, const uint32_t timeout) {

	msz_t200_rc_t							rc = MSZ_T200_RC_OK;
//	bool									tx, rx;
//	uint32_t								end_time = HAL_GetTick() + timeout;

	uint32_t tx_ctr, rx_ctr;

	TEST3_PIN_UP();
//	tx = false;
//	rx = false;
	tx_ctr = 0;
	rx_ctr = 0;

	if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY)) {
		if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE) == 0) {
			TEST_BSY_TX_NOT_EMPTY_ON_TEST2_PIN_UP();
		} else {
			TEST_BSY_ON_TEST2_PIN_UP();
		}
		spi_busy_ctr++;
	}
	if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_OVR)) {
		TEST_OVR_ON_TEST2_PIN_UP();
		*(__IO uint8_t*) &hspi->Instance->DR;
		spi_over_ctr++;
		rc = MSZ_T200_RC_ERR;
	}
	if (__HAL_SPI_GET_FLAG(hspi, SPI_SR_UDR)) {
		TEST_UDR_ON_TEST2_PIN_UP();
		spi_under_ctr++;
	}

	if (rc == MSZ_T200_RC_OK) {
		do {
			TEST2_PIN_DOWN();
			TEST1_PIN_DOWN();

			if (tx_ctr < data_write_length) {
				if ((hspi->Instance->SR & SPI_SR_FTLVL) == 0) {
					TEST2_PIN_UP();
					if (data_write) {
						*((__IO uint8_t*) &hspi->Instance->DR) = *(data_write + tx_ctr);
					}
					tx_ctr++;
				}
			}


			if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) {
				if (rx_ctr < data_write_length) {
					TEST1_PIN_UP();
					*(data_read + rx_ctr) = *(__IO uint8_t*) &hspi->Instance->DR;
					rx_ctr++;
				} else {
					hspi->Instance->DR;
				}
			}


		} while ((tx_ctr < data_write_length) || (rx_ctr < data_write_length));
	}

	TEST1_PIN_DOWN();
	TEST2_PIN_DOWN();
	TEST3_PIN_DOWN();

	return rc;
}

#endif

static uint32_t msz_t200_crc32_calc(uint8_t *data, const uint32_t data_len) {

	uint32_t								crc32 = 0;
	uint32_t								byte_no;

	for (byte_no = 0; byte_no < data_len; byte_no++) {
		crc32 += *(data + byte_no);
	}

	return crc32;
}

static uint32_t msz_t200_spi_recv_header_const_value(SPI_HandleTypeDef *hspi, uint8_t *data_read) {

	uint32_t								read_bytes_idx;
	msz_t200_rc_t							rc;
	uint8_t									data_tx = 0xAA;

	read_bytes_idx = 0;
	while (1) {
		rc = msz_t200_spi_data_transfer(hspi, &data_tx, data_read + read_bytes_idx, 1, 0);
		if (rc == MSZ_T200_RC_OK) {
			if (read_bytes_idx == 0) {
				if (*(data_read + read_bytes_idx) == 0x4D) {
					read_bytes_idx++;
					data_tx++;
				} else {
					break;
				}
			} else if (read_bytes_idx == 1) {
				if (*(data_read + read_bytes_idx) == 0xD3) {
					read_bytes_idx++;
					break;
				} else {
					break;
				}
			} else {
				break;
			}
		} else {
			break;
		}
	}

	return read_bytes_idx;
}

static uint32_t msz_t200_spi_recv_header_variable_value(SPI_HandleTypeDef *hspi, uint8_t *data_read) {

	uint32_t								read_bytes_idx = 6;
	uint8_t									data_tx[6] = {0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6};
	msz_t200_rc_t							rc;

	rc = msz_t200_spi_data_transfer(hspi, data_tx, data_read, read_bytes_idx, 3);
	if (rc == MSZ_T200_RC_OK) {
		read_bytes_idx = 6;
	} else {
		read_bytes_idx = 0;
	}

	return read_bytes_idx;
}

static bool msz_t200_spi_validate_header(const uint8_t *header_data, const uint32_t header_read_length) {

	bool									header_ok = true;

	if (header_read_length != 8) {
		header_ok = false;
	} else if ((*(header_data + 0) != 0x4D) || (*(header_data + 1) != 0xD3)) {
		header_ok = false;
	}
	//TODO dodac inne warunki

	return header_ok;
}

uint32_t reg_00456789 = 0xAAAAAAAA;

static void msz_t200_spi_write_registers(const uint32_t reg_addr, const uint32_t reg_value, const uint8_t reg_count) {

	if (reg_addr == 0x00456789) {
		reg_00456789 = reg_value;
	}
}

uint32_t read_reg_val = 0xABCDEF01;

static void msz_t200_spi_read_registers(const uint32_t reg_addr, uint8_t *data, const uint8_t reg_count) {

	if (reg_addr == 0x00456789) {
		if (reg_count == 1) {
			msz_t200_spi_set_32bdata_value(data, reg_00456789);
		} else {
			msz_t200_spi_set_32bdata_value(data, 0xF0F0F0F0);
		}
	} else {
		msz_t200_spi_set_32bdata_value(data, 0xFEFEFEFE);
	}
}

static void msz_t200_spi_do_write_operation(const uint8_t *data, const uint32_t data_length, const uint32_t reg_addr, const uint8_t operation_count) {


	uint32_t reg_value = msz_t200_spi_get_32bdata_value(data);
	msz_t200_spi_write_registers(reg_addr, reg_value, operation_count);
}

static uint32_t msz_t200_spi_handle_write_operation(SPI_HandleTypeDef *hspi, uint8_t *data, const uint32_t read_bytes_idx, const uint32_t reg_addr, const uint8_t operation_count) {

	uint32_t								read_data_bytes = read_bytes_idx;
	uint32_t								recv_crc32, calc_crc32;
	msz_t200_rc_t							rc = MSZ_T200_RC_OK;

	rc = msz_t200_spi_data_transfer(hspi, NULL, data + read_data_bytes, 8, 3);
	if (rc == MSZ_T200_RC_OK) {
		read_data_bytes += 8;
		recv_crc32 = msz_t200_spi_get_crc32(data + read_data_bytes - 4);
		calc_crc32 = msz_t200_crc32_calc(data, read_data_bytes - 4);
		if (recv_crc32 == calc_crc32) {

		} else {
			read_data_bytes = 0;
		}
	} else {
		read_data_bytes = 0;
	}

	return read_data_bytes;
}

static uint32_t msz_t200_spi_handle_read_operation(SPI_HandleTypeDef *hspi, uint8_t *data, const uint32_t read_bytes_idx, const uint32_t reg_addr, const uint8_t operation_count) {

	uint32_t								calc_crc32, write_idx = read_bytes_idx;
	msz_t200_rc_t							rc = MSZ_T200_RC_OK;

	msz_t200_spi_read_registers(reg_addr, data + write_idx, operation_count);
	write_idx += (4 * operation_count);
	calc_crc32 = msz_t200_crc32_calc(data, write_idx);
	msz_t200_spi_set_32bdata_value(data + write_idx, calc_crc32);
	write_idx += 4;
	rc = msz_t200_spi_data_transfer(hspi, data + read_bytes_idx, NULL, write_idx - read_bytes_idx, 3);
	if (rc == MSZ_T200_RC_OK) {

	} else {
		write_idx = 0;
	}

	return write_idx;
}

static void msz_t200_spi_operation(SPI_HandleTypeDef *hspi) {

	uint32_t								read_bytes_idx, read_data_bytes;
	uint8_t									read_bytes[16 + 32 * 4];
	uint32_t								tickstart;
	bool 									ok = false;
	bool									wr_operation;
	uint32_t								reg_addr;
	uint8_t									operation_count;

	HAL_UART_Transmit(&huart2, (const uint8_t*) enter_txt, strlen(enter_txt), 1000);

	my_SPI_Start_RxTxTransaction(hspi, &tickstart);
	read_bytes_idx = msz_t200_spi_recv_header_const_value(hspi, &read_bytes[0]);
	if (read_bytes_idx == 2) {
		read_bytes_idx += msz_t200_spi_recv_header_variable_value(hspi, &read_bytes[read_bytes_idx]);
		ok = msz_t200_spi_validate_header(&read_bytes[0], read_bytes_idx);
		if (ok) {
			wr_operation = msz_t200_spi_is_write_operation(&read_bytes[0]);
			reg_addr = msz_t200_spi_get_reg_addr(&read_bytes[0]);
			operation_count = msz_t200_spi_get_count_of_operations(&read_bytes[0]);
			if (operation_count == 1) {
				if (wr_operation) {
					read_data_bytes = msz_t200_spi_handle_write_operation(hspi, &read_bytes[0], read_bytes_idx, reg_addr, operation_count);
					if (read_data_bytes) {
						if (read_data_bytes > read_bytes_idx) {
							read_bytes_idx = read_data_bytes;
							msz_t200_spi_do_write_operation(&read_bytes[8], read_bytes_idx, reg_addr, operation_count);
							ok = true;
						} else {
							ok = false;
						}
					} else {
						ok = false;
					}
				} else {
					read_data_bytes = msz_t200_spi_handle_read_operation(hspi, &read_bytes[0], read_bytes_idx, reg_addr, operation_count);
					if (read_data_bytes) {
						ok = true;
					} else {
						ok = false;
					}
				}
			} else {
				ok = false;
			}
		} else {
			ok = false;
		}
		if (ok == false) {
			if (my_SPI_End_RxTxTransaction(hspi, 1, tickstart) != HAL_OK) {
				hspi1.ErrorCode = HAL_SPI_ERROR_FLAG;
			}
		}
	}

#if MSZ_T200_SUPPORT_TIMEOUT
	else if (rc == MSZ_T200_RC_TIMEOUT) {

		SET_BIT(RCC->APBRSTR2, RCC_APBRSTR2_SPI1RST);
		CLEAR_BIT(RCC->APBRSTR2, RCC_APBRSTR2_SPI1RST);

		HAL_SPI_Init(hspi);

	}
#endif


	HAL_UART_Transmit(&huart2, (const uint8_t*) spi_hdr_txt, strlen(spi_hdr_txt), 1000);
}

static void msz_t200_spi_clear_rx_fifo(SPI_HandleTypeDef *hspi) {

	do {
		hspi->Instance->DR;
	} while(__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE));
}






int main(void) {
	/* USER CODE BEGIN 1 */

	uint32_t led_ctr;

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CRC_Init();

	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Transmit(&huart2, (const uint8_t*) enter_txt, strlen(enter_txt), 1000);

	/*
	for (led_ctr = 0; led_ctr < 100; led_ctr++) {
		TEST3_PIN_UP();
		//TEST2_PIN_DOWN();
		HAL_Delay(10);
		TEST3_PIN_DOWN();
//		TEST2_PIN_UP();
		HAL_Delay(10);
	}
	*/

	led_ctr = 0;
	/* USER CODE END 2 */

	MX_SPI1_Init();

	TEST1_PIN_UP();
	TEST2_PIN_UP();
	TEST3_PIN_UP();
	msz_t200_spi_clear_rx_fifo(&hspi1);
	TEST1_PIN_DOWN();
	TEST2_PIN_DOWN();
	TEST3_PIN_DOWN();
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
#if 1
		msz_t200_spi_operation(&hspi1);
#else
		spi_slave_init();
#endif
		if (++led_ctr > 10) {
			led_ctr = 0;

		}
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
	hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
	hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
	hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
	hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
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
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GPIO_Port, TEST1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_RESET);


	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = TEST1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = TEST2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(TEST2_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = TEST3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(TEST3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

