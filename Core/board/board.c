/*
 * board.c
 *
 *  Created on: Jul 27, 2024
 *      Author: marek
 */

#include "main.h"
#include "types.h"

#if BOARD_TYPE == BOARD_T200_CPU_V01
#include "board_T200_cpu_v01/board_T200_cpu_v01.h"
#elif BOARD_TYPE == BOARD_NUCLEO_G070RB
#include "board_nucleo_070rb/board_nucleo_070rb.h"
#endif




msz_rc_t board_init_digital_input_state(const msz_t200_module_no_t module_no, const digital_in_no_t digital_in_no, const bool enable) {

#if BOARD_TYPE == BOARD_T200_CPU_V01
//ms	return board_T200_cpu_v01_init_digital_input_state(module_no, digital_in_no, enable);
#elif BOARD_TYPE == BOARD_NUCLEO_G070RB
//ms	return board_T200_cpu_v01_init_digital_input_state(module_no, digital_in_no, enable);
#endif /* BOARD_TYPE */

	return MSZ_RC_OK;
}

bool board_read_digital_input_state(const msz_t200_module_no_t module_no, const digital_in_no_t digital_in_no) {

#if BOARD_TYPE == BOARD_T200_CPU_V01
//ms	return board_T200_cpu_v01_read_digital_input_state(module_no, digital_in_no);
#elif BOARD_TYPE == BOARD_NUCLEO_G070RB
//ms	return board_T200_cpu_v01_read_digital_input_state(module_no, digital_in_no);
#endif /* BOARD_TYPE */

	return true;
}

void board_system_board_set_test_pin(const uint8_t pin_no, const bool state) {

}



