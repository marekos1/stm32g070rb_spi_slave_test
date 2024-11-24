/*
 * types.h
 *
 *  Created on: Feb 24, 2023
 *      Author: marek
 */

#ifndef INC_TYPES_H_
#define INC_TYPES_H_

#include <stdint.h>


typedef int bool;
#define true 	1
#define false 	0


typedef enum {
	MSZ_RC_OK = 0,
	MSZ_RC_ERR_INV_ARG = -1,
	MSZ_RC_ERR_INV_CRC = -2,
	MSZ_RC_ERR_INV_ADDRESS = -3,
	MSZ_RC_ERR_INV_DATA = -4,
	MSZ_RC_ERR_INV_VERSION = -5,
	MSZ_RC_ERR_INV_STATE = -6,
	MSZ_RC_ERR_INV_WRITE_PROTECT = -7,
	MSZ_RC_ERR_SEQNC_ERR = -8,
	MSZ_RC_ERR_NOT_FOUND = -9,

	MSZ_RC_ERR_NO_MEMORY = -10,
	MSZ_RC_ERR_NO_IMPLEMENTATION = -11,
	MSZ_RC_ERR_TIMEOUT = -12,
	MSZ_RC_ERR_NOT_ACK = -13,
	MSZ_RC_NOT_READY = -14,
	MSZ_RC_ERR_NOT_SUPPORTED = -15,

	MSZ_RC_SPI_IO_ERR = -200,

	MSZ_RC_HAL_FLASH_IO_ERR = -201,
	MSZ_RC_HAL_FLASH_NOT_FOUND = -202,

	MSZ_RC_FLASH_ERROR = -300,
	MSZ_RC_FLASH_PAGE_NOT_FOUND = -301,
	MSZ_RC_FLASH_BUFFER_ERROR = -302,

	MSZ_RC_FPGA_ERROR = -400,
	MSZ_RC_FPGA_LOAD_CONFIG_ERROR = -401,


	MSZ_RC_GPIO_INV_PORT = -500,
	MSZ_RC_GPIO_INV_PIN = -501,
	MSZ_RC_GPIO_INV_AF = -502,

	MSZ_RC_I2C_INV_CTRL = -600,

	MSZ_RC_UNIT_NO_OUTSIDE_THE_RANGE = -1000,
	MSZ_RC_MODULE_NO_OUTSIDE_THE_RANGE = -1001,


	MSZ_RC_DIGITAL_IN_UNSUPPORTED_MODULE_TYPE = -2000,

	MSZ_RC_SPI_SLAVE_INV_CONST_HEADER = -3000,
	MSZ_RC_SPI_SLAVE_INV_VAR_HEADER = -3001,
	MSZ_RC_SPI_SLAVE_INV_HEADER_LENGTH = -3002,
	MSZ_RC_SPI_SLAVE_ERR_OVERRUN = -3003,
	MSZ_RC_SPI_SLAVE_MASTER_FINISH_TRANSACTION = -3004,

	MSZ_RC_LAST
} msz_rc_t;

/****************
 * UNIT MODULES types
 */

#define MSZ_T200_UNITS 	1

typedef uint8_t msz_t200_unit_no_t;


#define MSZ_T200_MODULES 4

typedef uint8_t msz_t200_module_no_t;

typedef enum {
	MSZ_T200_MODULE_TYPE_NONE_OR_EMPTY = 0,
	MSZ_T200_MODULE_TYPE_DIGITAL_INPUT8,
	MSZ_T200_MODULE_TYPE_DIGITAL_OUTPUT8,
	MSZ_T200_MODULE_TYPE_DALLAS4

} msz_t200_module_type_t;

//typedef uint8_t msz_t200_unit_no_t;




/****************
 * digital in types
 */

#define DIGITAL_INPUTS_PER_MODULE	8

typedef uint8_t digital_in_no_t;



/****************
 * digital out types
 */

#define DIGITAL_OUTPUTS_PER_MODULE		8

typedef uint8_t digital_out_no_t;


#endif /* INC_TYPES_H_ */
