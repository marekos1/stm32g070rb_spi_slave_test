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


msz_rc_t board_nucleo_070rb_digital_input_init(const msz_t200_module_no_t module_no, const digital_in_no_t digital_in_no, const bool enable);

bool board_nucleo_070rb_digital_in_get_state(const msz_t200_module_no_t module_no, const digital_in_no_t digital_in_no);



msz_rc_t board_nucleo_070rb_digital_output_init(const msz_t200_module_no_t module_no, const digital_out_no_t digital_out_no, const bool enable);

msz_rc_t board_nucleo_070rb_set_digital_output_state(const msz_t200_module_no_t module_no, const digital_out_no_t digital_out_no, const bool new_state);







#endif /* BOARD_BOARD_NUCLEO_070RB_BOARD_NUCLEO_070RB_H_ */
