/*
 * board_nucleo_070rb.h
 *
 *  Created on: Jul 28, 2024
 *      Author: marek
 */

#ifndef BOARD_BOARD_NUCLEO_070RB_BOARD_NUCLEO_070RB_H_
#define BOARD_BOARD_NUCLEO_070RB_BOARD_NUCLEO_070RB_H_

#include "main.h"
#include "types.h"


msz_rc_t board_T200_cpu_v01_init_digital_input_state(const msz_t200_module_no_t module_no, const digital_in_no_t digital_in_no, const BOOL enable);

BOOL board_T200_cpu_v01_read_digital_input_state(const msz_t200_module_no_t module_no, const digital_in_no_t digital_in_no);

#endif /* BOARD_BOARD_NUCLEO_070RB_BOARD_NUCLEO_070RB_H_ */
