/*
 * main_slave_regs.h
 *
 *  Created on: 05.05.2024
 *      Author: marek
 */

#ifndef INC_MAIN_SLAVE_REGS_H_
#define INC_MAIN_SLAVE_REGS_H_

#include "slave_regs/slave_regs.h"


#define MAIN_SLAVE_REG_BASE_ADDR	0


typedef enum {
	/*  0  [ 0] */ MAIN_SLAVE_REG_MODULE_READY = 0,			/* - Module ready */
	/*  1  [ 1] */ MAIN_SLAVE_REG_MODULE_TYPE,				/* - Module Type 0x82 - GPS */
	/*  2  [ 2] */ MAIN_SLAVE_REG_MODULE_HW_VERSION,		/* - Module Hardware revision */
	/*  3  [ 3] */ MAIN_SLAVE_REG_MODULE_SW_VERSION,		/* - Module Firmware revision */
	/*  4  [ 4] */ MAIN_SLAVE_REG_MODULE_GLOBAL_CONFIG,		/* - GLOBAL config */
	/*  5  [ 5] */ MAIN_SLAVE_REG_MODULE_GLOBAL_STATUS,		/* - GLOBAL status */

	/*  6  [ 6] */ MAIN_SLAVE_REG_MODULE_UNIT1_CONF,		/* - Unit 1 config - konfiguracja 4 modułow po 8 bitów*/
	/*  7  [ 7] */ MAIN_SLAVE_REG_MODULE_UNIT2_CONF,		/* - Unit 2 config */
	/*  8  [ 8] */ MAIN_SLAVE_REG_MODULE_UNIT3_CONF,		/* - Unit 3 config */
	/*  9  [ 9] */ MAIN_SLAVE_REG_MODULE_UNIT4_CONF,		/* - Unit 4 config */

	/* 10  [10] */ MAIN_SLAVE_REG_MODULE_UNIT1_STATUS,		/* - Unit 1 status */
	/* 11  [11] */ MAIN_SLAVE_REG_MODULE_UNIT2_STATUS,		/* - Unit 2 status */
	/* 12  [12] */ MAIN_SLAVE_REG_MODULE_UNIT3_STATUS,		/* - Unit 3 status */
	/* 13  [13] */ MAIN_SLAVE_REG_MODULE_UNIT4_STATUS,		/* - Unit 4 status */

	/* 14  [14] */ MAIN_SLAVE_REG_MODULE_UNIT1_OUTPUT_STATE,/* - Unit 4 status */


	/*  8  [ 8] */ MAIN_SLAVE_REG_RST,
	/*  9  [ 9] */ MAIN_SLAVE_REG_LED_CONFIG,

	/*  10 [10] */ MAIN_SLAVE_BUFFER_LENGTH,

	MAIN_SLAVE_REG_NR_MAX
} main_slave_reg_t;

/* MAIN_SLAVE_REG_MODULE_GLOBAL_STATUS */
// bit 0 - Not supported unit
#define MAIN_SLAVE_REG_MODULE_GLOBAL_STATUS_NOT_SUPPORT_UNIT 		0
// bit 1 : 4 - Unit 0 module 0 - 3 supported config
#define MAIN_SLAVE_REG_MODULE_GLOBAL_STATUS_U0M0_NOT_SUPPORT_CONF	1

extern volatile slave_reg_buf_t slave_main_group_regs[MAIN_SLAVE_REG_NR_MAX];

bool main_group_slave_regs_req(slave_regs_poll_func_req_t req, uint16_t reg_addr, slave_reg_data_t data);

void main_group_slave_status_set(bool system_ready);

void main_group_slave_status_digital_in_state_set(uint32_t unit, uint32_t module, uint32_t input, bool state);


#endif /* INC_MAIN_SLAVE_REGS_H_ */
