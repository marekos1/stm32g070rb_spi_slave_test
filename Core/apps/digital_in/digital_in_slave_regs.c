/*
 * digital_in_slave_regs.c
 *
 *  Created on: Nov 24, 2024
 *      Author: marek
 */

#include "main.h"
#include "trace/trace.h"

#include "slave_regs/slave_regs.h"
#include "digital_in/digital_in_slave_regs.h"


volatile slave_reg_buf_t digital_in_group_regs[DIGITAL_IN_SLAVE_REG_NR_MAX];


static void digital_in_group_slave_regs_init(volatile slave_reg_buf_t *regs, uint16_t regs_number) {

	slave_registers_init_value((regs + DIGITAL_IN_SLAVE_REG_INPUT_STATE), 0, false, false, false, true);
}






bool digital_in_group_slave_regs_req(slave_regs_poll_func_req_t req, uint16_t reg_addr, slave_reg_data_t data) {

	bool									ret_val = false;

	switch (req) {
	case SLAVE_REGS_POLL_FUNC_REQ_INIT:
		digital_in_group_slave_regs_init(digital_in_group_regs, DIGITAL_IN_SLAVE_REG_NR_MAX);
		break;
	case SLAVE_REGS_POLL_FUNC_REQ_WRITE:
		T_DG_DIGI_IN("Write reg_addr: 0x%08X <- 0x%08X", reg_addr, data);
		break;
	default:
		break;
	}

	return ret_val;
}
