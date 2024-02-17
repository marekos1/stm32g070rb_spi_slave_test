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
const char *enter_txt = "Start\r\n";
const char *spi_ok_txt = "SPI ok\r\n";
const char *spi_err_txt = "SPI ERR!\r\n";
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


static uint8_t aTxBuffer[MSZ_T200_BUFFER_SIZE], aRxBuffer[MSZ_T200_BUFFER_SIZE];
static uint8_t spi_tra_ctr = 0;
static uint8_t spi_tra_err_ctr = 0;

static uint32_t msz_t200_spi_get_reg_addr_from_header(uint8_t *header_data) {

	uint32_t								reg_addr = 0;


	return reg_addr;
}

static bool msz_t200_spi_is_write_operation(uint8_t *header_data) {

	bool									wr_operation = false;


	return wr_operation;
}

static uint8_t msz_t200_spi_get_number_of_operations(uint8_t *header_data) {

	uint8_t									number_of_operation = 0;


	return number_of_operation;
}

uint32_t msz_t200_crc32_calc() {

	return 0;
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

static HAL_StatusTypeDef my_SPI_EndRxTxTransaction(SPI_HandleTypeDef *hspi,
		uint32_t Timeout, uint32_t Tickstart) {
	/* Control if the TX fifo is empty */
	if (my_SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FTLVL, SPI_FTLVL_EMPTY,
			Timeout, Tickstart) != HAL_OK) {
		SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
		return HAL_TIMEOUT;
	}

	/* Control the BSY flag */
	if (my_SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_BSY, RESET, Timeout,
			Tickstart) != HAL_OK) {
		SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
		return HAL_TIMEOUT;
	}

	/* Control if the RX fifo is empty */
	if (my_SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY,
			Timeout, Tickstart) != HAL_OK) {
		SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
		return HAL_TIMEOUT;
	}

	return HAL_OK;
}




static HAL_StatusTypeDef my_HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
		uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout) {
	uint16_t initial_TxXferCount;
	uint32_t tmp_mode;
	HAL_SPI_StateTypeDef tmp_state;
	uint32_t tickstart;
#if (USE_SPI_CRC != 0U)
  __IO uint32_t tmpreg = 0U;
  uint32_t             spi_cr1;
  uint32_t             spi_cr2;
  __IO uint8_t  *ptmpreg8;
  __IO uint8_t  tmpreg8 = 0;
#endif /* USE_SPI_CRC */

	/* Variable used to alternate Rx and Tx during transfer */
	uint32_t txallowed = 1U;
	HAL_StatusTypeDef errorcode = HAL_OK;

	/* Check Direction parameter */
	assert_param(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

	/* Process Locked */
	__HAL_LOCK(hspi);

	/* Init tickstart for timeout management*/
	tickstart = HAL_GetTick();

	/* Init temporary variables */
	tmp_state = hspi->State;
	tmp_mode = hspi->Init.Mode;
	initial_TxXferCount = Size;
#if (USE_SPI_CRC != 0U)
  spi_cr1             = READ_REG(hspi->Instance->CR1);
  spi_cr2             = READ_REG(hspi->Instance->CR2);
#endif /* USE_SPI_CRC */

	if (!((tmp_state == HAL_SPI_STATE_READY)
			|| ((tmp_mode == SPI_MODE_MASTER)
					&& (hspi->Init.Direction == SPI_DIRECTION_2LINES)
					&& (tmp_state == HAL_SPI_STATE_BUSY_RX)))) {
		errorcode = HAL_BUSY;
		goto error;
	}

	if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U)) {
		errorcode = HAL_ERROR;
		goto error;
	}

	/* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
	if (hspi->State != HAL_SPI_STATE_BUSY_RX) {
		hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
	}

	/* Set the transaction information */
	hspi->ErrorCode = HAL_SPI_ERROR_NONE;
	hspi->pRxBuffPtr = (uint8_t*) pRxData;
	hspi->RxXferCount = Size;
	hspi->RxXferSize = Size;
	hspi->pTxBuffPtr = (uint8_t*) pTxData;
	hspi->TxXferCount = Size;
	hspi->TxXferSize = Size;

	/*Init field not used in handle to zero */
	hspi->RxISR = NULL;
	hspi->TxISR = NULL;

#if (USE_SPI_CRC != 0U)
  /* Reset CRC Calculation */
  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    SPI_RESET_CRC(hspi);
  }
#endif /* USE_SPI_CRC */

	/* Set the Rx Fifo threshold */
	if (hspi->Init.DataSize > SPI_DATASIZE_8BIT) {
		/* Set fiforxthreshold according the reception data length: 16bit */
		CLEAR_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
	} else {
		/* Set fiforxthreshold according the reception data length: 8bit */
		SET_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
	}

	/* Check if the SPI is already enabled */
	if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) {
		/* Enable SPI peripheral */
		__HAL_SPI_ENABLE(hspi);
	}

	/* Transmit and Receive data in 16 Bit mode */
	if (hspi->Init.DataSize > SPI_DATASIZE_8BIT) {
		if ((hspi->Init.Mode == SPI_MODE_SLAVE)
				|| (initial_TxXferCount == 0x01U)) {
			hspi->Instance->DR = *((uint16_t*) hspi->pTxBuffPtr);
			hspi->pTxBuffPtr += sizeof(uint16_t);
			hspi->TxXferCount--;
		}
		while ((hspi->TxXferCount > 0U) || (hspi->RxXferCount > 0U)) {
			/* Check TXE flag */
			if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE))
					&& (hspi->TxXferCount > 0U) && (txallowed == 1U)) {
				hspi->Instance->DR = *((uint16_t*) hspi->pTxBuffPtr);
				hspi->pTxBuffPtr += sizeof(uint16_t);
				hspi->TxXferCount--;
				/* Next Data is a reception (Rx). Tx not allowed */
				txallowed = 0U;

#if (USE_SPI_CRC != 0U)
        /* Enable CRC Transmission */
        if ((hspi->TxXferCount == 0U) && (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE))
        {
          /* Set NSS Soft to received correctly the CRC on slave mode with NSS pulse activated */
          if ((READ_BIT(spi_cr1, SPI_CR1_MSTR) == 0U) && (READ_BIT(spi_cr2, SPI_CR2_NSSP) == SPI_CR2_NSSP))
          {
            SET_BIT(hspi->Instance->CR1, SPI_CR1_SSM);
          }
          SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
        }
#endif /* USE_SPI_CRC */
			}

			/* Check RXNE flag */
			if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))
					&& (hspi->RxXferCount > 0U)) {
				*((uint16_t*) hspi->pRxBuffPtr) = (uint16_t) hspi->Instance->DR;
				hspi->pRxBuffPtr += sizeof(uint16_t);
				hspi->RxXferCount--;
				/* Next Data is a Transmission (Tx). Tx is allowed */
				txallowed = 1U;
			}
			if (((HAL_GetTick() - tickstart) >= Timeout)
					&& (Timeout != HAL_MAX_DELAY)) {
				errorcode = HAL_TIMEOUT;
				goto error;
			}
		}
	}
	/* Transmit and Receive data in 8 Bit mode */
	else {
		if ((hspi->Init.Mode == SPI_MODE_SLAVE)
				|| (initial_TxXferCount == 0x01U)) {
			*((__IO uint8_t*) &hspi->Instance->DR) = (*hspi->pTxBuffPtr);
			hspi->pTxBuffPtr += sizeof(uint8_t);
			hspi->TxXferCount--;
		}
		while ((hspi->TxXferCount > 0U) || (hspi->RxXferCount > 0U)) {
			/* Check TXE flag */
			if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE))
					&& (hspi->TxXferCount > 0U) && (txallowed == 1U)) {
				*(__IO uint8_t*) &hspi->Instance->DR = (*hspi->pTxBuffPtr);
				hspi->pTxBuffPtr++;
				hspi->TxXferCount--;
				/* Next Data is a reception (Rx). Tx not allowed */
				txallowed = 0U;

#if (USE_SPI_CRC != 0U)
        /* Enable CRC Transmission */
        if ((hspi->TxXferCount == 0U) && (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE))
        {
          /* Set NSS Soft to received correctly the CRC on slave mode with NSS pulse activated */
          if ((READ_BIT(spi_cr1, SPI_CR1_MSTR) == 0U) && (READ_BIT(spi_cr2, SPI_CR2_NSSP) == SPI_CR2_NSSP))
          {
            SET_BIT(hspi->Instance->CR1, SPI_CR1_SSM);
          }
          SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
        }
#endif /* USE_SPI_CRC */
			}

			/* Wait until RXNE flag is reset */
			if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))
					&& (hspi->RxXferCount > 0U)) {
				(*(uint8_t*) hspi->pRxBuffPtr) =
						*(__IO uint8_t*) &hspi->Instance->DR;
				hspi->pRxBuffPtr++;
				hspi->RxXferCount--;
				/* Next Data is a Transmission (Tx). Tx is allowed */
				txallowed = 1U;
			}
			if ((((HAL_GetTick() - tickstart) >= Timeout)
					&& ((Timeout != HAL_MAX_DELAY))) || (Timeout == 0U)) {
				errorcode = HAL_TIMEOUT;
				goto error;
			}
		}
	}

#if (USE_SPI_CRC != 0U)
  /* Read CRC from DR to close CRC calculation process */
  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    /* Wait until TXE flag */
    if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, SET, Timeout, tickstart) != HAL_OK)
    {
      /* Error on the CRC reception */
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
      errorcode = HAL_TIMEOUT;
      goto error;
    }
    /* Read CRC */
    if (hspi->Init.DataSize == SPI_DATASIZE_16BIT)
    {
      /* Read 16bit CRC */
      tmpreg = READ_REG(hspi->Instance->DR);
      /* To avoid GCC warning */
      UNUSED(tmpreg);
    }
    else
    {
      /* Initialize the 8bit temporary pointer */
      ptmpreg8 = (__IO uint8_t *)&hspi->Instance->DR;
      /* Read 8bit CRC */
      tmpreg8 = *ptmpreg8;
      /* To avoid GCC warning */
      UNUSED(tmpreg8);

      if (hspi->Init.CRCLength == SPI_CRC_LENGTH_16BIT)
      {
        if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, SET, Timeout, tickstart) != HAL_OK)
        {
          /* Error on the CRC reception */
          SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
          errorcode = HAL_TIMEOUT;
          goto error;
        }
        /* Read 8bit CRC again in case of 16bit CRC in 8bit Data mode */
        tmpreg8 = *ptmpreg8;
        /* To avoid GCC warning */
        UNUSED(tmpreg8);
      }
    }
  }

  /* Check if CRC error occurred */
  if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR))
  {
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
    /* Clear CRC Flag */
    __HAL_SPI_CLEAR_CRCERRFLAG(hspi);

    errorcode = HAL_ERROR;
  }
#endif /* USE_SPI_CRC */

	/* Check the end of the transaction */
	if (my_SPI_EndRxTxTransaction(hspi, Timeout, tickstart) != HAL_OK) {
		errorcode = HAL_ERROR;
		hspi->ErrorCode = HAL_SPI_ERROR_FLAG;
	}

	error: hspi->State = HAL_SPI_STATE_READY;
	__HAL_UNLOCK(hspi);
	return errorcode;
}











void msz_t200_spi_operation(void) {

	uint32_t								i, reg_addr, byte_total;
	bool									wr_operation;
	uint8_t									number_of_operation;
	HAL_StatusTypeDef 						hal_rc;


#if 0

	byte_total = 0;

	for (i = byte_total; i < MSZ_T200_HEADER_SIZE; i++) {
		aTxBuffer[i] = 0x77;
	}
	hal_rc = HAL_SPI_Receive(&hspi1, &aRxBuffer[byte_total], MSZ_T200_HEADER_SIZE, 5000);
	byte_total += MSZ_T200_HEADER_SIZE;

	for (i = byte_total; i < byte_total + MSZ_T200_DUMMY_SIZE; i++) {
		aTxBuffer[i] = 0xAA;
	}
	hal_rc = HAL_SPI_Receive(&hspi1, &aRxBuffer[byte_total], MSZ_T200_DUMMY_SIZE, 5000);
	byte_total += MSZ_T200_DUMMY_SIZE;

	for (i = byte_total; i < byte_total + 4; i++) {
		aTxBuffer[i] = 0x55;
	}
	hal_rc = HAL_SPI_Receive(&hspi1, &aRxBuffer[byte_total], 4, 5000);
	byte_total += 4;

	for (i = byte_total; i < byte_total + MSZ_T200_CRC32_SIZE; i++) {
		aTxBuffer[i] = 0x88;
	}
	hal_rc = HAL_SPI_Receive(&hspi1, &aRxBuffer[byte_total], MSZ_T200_CRC32_SIZE, 5000);
	byte_total += MSZ_T200_CRC32_SIZE;

	for (i = 0; i < byte_total; i++) {
		aTxBuffer[i] = aRxBuffer[i];
		hal_rc = HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, aRxBuffer,byte_total, 5000);
	}

#elif 1
	byte_total = 0;

	for (i = byte_total; i < MSZ_T200_HEADER_SIZE; i++) {
		aTxBuffer[i] = 0x77;
	}
	byte_total += MSZ_T200_HEADER_SIZE;

	for (i = byte_total; i < byte_total + MSZ_T200_DUMMY_SIZE; i++) {
		aTxBuffer[i] = 0xAA;
	}
	byte_total += MSZ_T200_DUMMY_SIZE;

	for (i = byte_total; i < byte_total + 4; i++) {
		aTxBuffer[i] = 0x55;
	}
	byte_total += 4;

	for (i = byte_total; i < byte_total + MSZ_T200_CRC32_SIZE; i++) {
		aTxBuffer[i] = 0x88;
	}
	byte_total += MSZ_T200_CRC32_SIZE;

	hal_rc = my_HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, aRxBuffer, byte_total, 5000);

	if (hal_rc == HAL_OK) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_UART_Transmit(&huart2, (const uint8_t*) spi_ok_txt,
				strlen(enter_txt), 1000);
		spi_tra_ctr++;
	} else {
		HAL_UART_Transmit(&huart2, (const uint8_t*) spi_err_txt,
				strlen(enter_txt), 1000);
		spi_tra_err_ctr++;
	}
#elif 1
	hal_rc = HAL_SPI_TransmitReceive(&hspi1, aTxBuffer, aRxBuffer, byte_total, 5000);
	if (hal_rc == HAL_OK) {
		bool ok;
		ok = false;
		if ((aRxBuffer[0] == MSZ_T200_HEADER_FIRST_BYTE) && (aRxBuffer[1] == MSZ_T200_HEADER_SECOND_BYTE)) {
			reg_addr = msz_t200_spi_get_reg_addr_from_header(aRxBuffer);
			wr_operation = msz_t200_spi_is_write_operation(aRxBuffer);
			number_of_operation = msz_t200_spi_get_number_of_operations(aRxBuffer);
			if (reg_addr < MSZ_T200_REGISTER_NUMBER) {
				if (number_of_operation < MSZ_T200_SINGLE_OPERATION_WORD_LENGTH_MAX) {
					ok = true;
				}
			}
		} else {
			spi_tra_err_ctr++;
		}
		if (ok) {
			for (i = MSZ_T200_HEADER_SIZE; i < MSZ_T200_DUMMY_SIZE; i++) {
				aTxBuffer[i] = 0x00;
			}
			byte_total = MSZ_T200_HEADER_SIZE + MSZ_T200_DUMMY_SIZE + number_of_operation * 4;
			if (wr_operation) {
				for (i = MSZ_T200_HEADER_SIZE + MSZ_T200_DUMMY_SIZE; i < byte_total; i++) {
					aTxBuffer[i] = 0xFF;
				}
			} else {
				for (i = MSZ_T200_HEADER_SIZE + MSZ_T200_DUMMY_SIZE; i < byte_total; i++) {
					aTxBuffer[i] = 0x77;
				}
			}
			for (i = byte_total; i < byte_total + MSZ_T200_CRC32_SIZE; i++) {
				aTxBuffer[i] = 0x55;
			}
			hal_rc = HAL_SPI_TransmitReceive(&hspi1, &aTxBuffer[MSZ_T200_HEADER_SIZE], &aRxBuffer[MSZ_T200_HEADER_SIZE], byte_total + MSZ_T200_CRC32_SIZE, 5000);
		} else {
			aTxBuffer[MSZ_T200_HEADER_SIZE] = 0xAA;
			hal_rc = HAL_SPI_TransmitReceive(&hspi1, &aTxBuffer[MSZ_T200_HEADER_SIZE], aRxBuffer, MSZ_T200_BUFFER_SIZE - MSZ_T200_HEADER_SIZE, 5000);
		}

		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_UART_Transmit(&huart2, (const uint8_t*) spi_ok_txt,
				strlen(enter_txt), 1000);
		spi_tra_ctr++;
	} else {
		HAL_UART_Transmit(&huart2, (const uint8_t*) spi_err_txt,
				strlen(enter_txt), 1000);
		spi_tra_err_ctr++;
	}
#endif




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
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Transmit(&huart2, (const uint8_t*) enter_txt, strlen(enter_txt),
			1000);
	led_ctr = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
#if 1
		msz_t200_spi_operation();
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

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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

