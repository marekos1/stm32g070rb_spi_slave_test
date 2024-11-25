/*
 * digital_in_slave_regs.h
 *
 *  Created on: Nov 24, 2024
 *      Author: marek
 */

#ifndef APPS_DIGITAL_IN_DIGITAL_IN_SLAVE_REGS_H_
#define APPS_DIGITAL_IN_DIGITAL_IN_SLAVE_REGS_H_


#include "slave_regs/slave_regs.h"



#define DIGITAL_IN_SLAVE_REG_BASE_ADDR	1000

typedef enum {
	/*  0  [ 0] */ DIGITAL_IN_SLAVE_REG_INPUT_STATE = 0,			/* - Unit 0 Output state */

	DIGITAL_IN_SLAVE_REG_NR_MAX
} digital_in_slave_reg_t;


extern volatile slave_reg_buf_t digital_in_group_regs[DIGITAL_IN_SLAVE_REG_NR_MAX];

bool digital_in_group_slave_regs_req(slave_regs_poll_func_req_t req, uint16_t reg_addr, slave_reg_data_t data);

#endif /* APPS_DIGITAL_IN_DIGITAL_IN_SLAVE_REGS_H_ */
