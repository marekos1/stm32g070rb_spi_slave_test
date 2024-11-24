/*
 * digital_out_slave_regs.h
 *
 *  Created on: Nov 20, 2024
 *      Author: marek
 */

#ifndef SRC_DIGITAL_OUT_DIGITAL_OUT_SLAVE_REGS_H_
#define SRC_DIGITAL_OUT_DIGITAL_OUT_SLAVE_REGS_H_


#include "slave_regs/slave_regs.h"



#define DIGITAL_OUT_SLAVE_REG_BASE_ADDR	14


typedef enum {
	/*  0  [ 0] */ DIGITAL_OUT_SLAVE_REG_OUTPUT_STATE = 0,			/* - Unit 0 Output state */

	DIGITAL_OUT_SLAVE_REG_NR_MAX
} digital_out_slave_reg_t;


extern volatile slave_reg_buf_t digital_out_group_regs[DIGITAL_OUT_SLAVE_REG_NR_MAX];


bool digital_out_group_slave_regs_req(slave_regs_poll_func_req_t req, uint16_t reg_addr, slave_reg_data_t data);

#endif /* SRC_DIGITAL_OUT_DIGITAL_OUT_SLAVE_REGS_H_ */
