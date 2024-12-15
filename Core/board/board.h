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

#include "uart/uart.h"




msz_rc_t board_init_digital_input_state(const msz_t200_module_no_t module_no, const digital_in_no_t digital_in_no, const bool enable);

bool board_read_digital_input_state(const msz_t200_module_no_t module_no, const digital_in_no_t digital_in_no);


msz_rc_t board_digital_output_init(const msz_t200_module_no_t module_no, const digital_out_no_t digital_out_no, const bool new_state);

msz_rc_t board_set_digital_output_state(const msz_t200_module_no_t module_no, const digital_out_no_t digital_out_no, const bool new_state);

void board_system_board_set_test_pin(const uint8_t pin_no, const bool state);


extern const uart_msz_dev_t cli_trace_uart_dev;

#endif /* BOARD_BOARD_H_ */
