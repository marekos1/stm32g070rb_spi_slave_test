/*
 * msz_t200_spi_slave.h
 *
 *  Created on: Nov 12, 2024
 *      Author: marek
 */

#ifndef SRC_SLAVE_REGS_MSZ_T200_SPI_SLAVE_MSZ_T200_SPI_SLAVE_H_
#define SRC_SLAVE_REGS_MSZ_T200_SPI_SLAVE_MSZ_T200_SPI_SLAVE_H_

#include "main.h"

bool msz_t200_spi_slave_poll(void);

msz_rc_t msz_t200_spi_slave_init(void);

#endif /* SRC_SLAVE_REGS_MSZ_T200_SPI_SLAVE_MSZ_T200_SPI_SLAVE_H_ */
