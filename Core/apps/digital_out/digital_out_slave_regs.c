/*
 * digital_out_slave_regs.c
 *
 *  Created on: Nov 20, 2024
 *      Author: marek
 */

#include "main.h"
#include "trace/trace.h"

#include "slave_regs/slave_regs.h"
#include "digital_out/digital_out_slave_regs.h"

volatile slave_reg_buf_t digital_out_group_regs[DIGITAL_OUT_SLAVE_REG_NR_MAX];


static void digital_out_group_slave_regs_init(volatile slave_reg_buf_t *regs, uint16_t regs_number) {

	slave_registers_init_value((regs + DIGITAL_OUT_SLAVE_REG_OUTPUT_STATE), 0, true, true, false);
}

static slave_reg_data_t digital_out_group_slave_regs_change_state(slave_reg_data_t new_reg_value) {

	slave_reg_data_t						reg_val = new_reg_value;
	msz_t200_module_no_t					module_no;
	uint8_t									module_output_state;
	digital_out_no_t						digital_out_no;
	bool									output_state;
	msz_rc_t								rc = MSZ_RC_OK;

	for (module_no = 0; module_no < MSZ_T200_MODULES; module_no++) {
		module_output_state = (uint8_t)((reg_val >> (8 * module_no)) & 0x000000FF);
		for (digital_out_no = 0; digital_out_no < DIGITAL_OUTPUTS_PER_MODULE; digital_out_no++) {
			output_state = (module_output_state & (1 << digital_out_no));
			rc = digital_out_set_state_by_apps(0, module_no, digital_out_no, output_state);
		}
	}

	return reg_val;
}




bool digital_out_group_slave_regs_req(slave_regs_poll_func_req_t req, uint16_t reg_addr, slave_reg_data_t data) {

	bool									ret_val = false;

	switch (req) {
	case SLAVE_REGS_POLL_FUNC_REQ_INIT:
		digital_out_group_slave_regs_init(digital_out_group_regs, DIGITAL_OUT_SLAVE_REG_NR_MAX);
		break;
	case SLAVE_REGS_POLL_FUNC_REQ_WRITE:
		T_DG_DIGI_OUT("Write reg_addr: 0x%08X <- 0x%08X", reg_addr, data);
		if (reg_addr == DIGITAL_OUT_SLAVE_REG_OUTPUT_STATE) {
			slave_registers_write(&digital_out_group_regs[DIGITAL_OUT_SLAVE_REG_OUTPUT_STATE], digital_out_group_slave_regs_change_state);
		}
		break;
	default:
		break;
	}

	return ret_val;
}
