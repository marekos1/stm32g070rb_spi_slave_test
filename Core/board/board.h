/*
 * board.h
 *
 *  Created on: Jul 27, 2024
 *      Author: marek
 */

#ifndef BOARD_BOARD_H_
#define BOARD_BOARD_H_

#include "main.h"
#include "types.h"






msz_rc_t board_init_digital_input_state(const msz_t200_module_no_t module_no, const digital_in_no_t digital_in_no, const BOOL enable);

BOOL board_read_digital_input_state(const msz_t200_module_no_t module_no, const digital_in_no_t digital_in_no);




#endif /* BOARD_BOARD_H_ */
