/*
 * main_slave_regs.c
 *
 *  Created on: 05.05.2024
 *      Author: marek
 */

#include <string.h>

#include "main.h"
#include "types.h"


#if defined(STM32F0)
#include "stm32f0xx.h"
#elif defined(STM32G431xx)
#include "stm32g4xx.h"
#endif

#include "slave_regs.h"
#include "main_slave_regs.h"


volatile slave_reg_buf_t slave_main_group_regs[MAIN_SLAVE_REG_NR_MAX];

static const slave_reg_data_t slave_main_group_regs_ident_value[6] = {
		/* 0 */ 0x00000001, 													/* - Module ready */
		/* 1 */ 0x00000082, 													/* - Module Type 0x0082 - GPS */
		/* 2 */ (slave_reg_data_t)SYSTEM_HARDWARE_REVISON,							/* - Module Hardware revision */
		/* 3 */ (slave_reg_data_t)SYSTEM_SOFTWARE_VERSION_MINOR, 					/* - Module Firmware revision */
		/* 4 */ 0x00000000, 													/* - Global config */
};



static slave_reg_data_t registers_global_conf(slave_reg_data_t new_reg_value) {

	slave_reg_data_t						reg_val = new_reg_value;

	if (reg_val & (1 << 0)) {
		NVIC_SystemReset();
		reg_val &= ~(1 << 0);	/* Self cleared bit */
	}
	if (reg_val & (1 << 1)) {
		reg_val &= ~(1 << 0);	/* Self cleared bit */
	}

	return reg_val;
}

static slave_reg_data_t registers_unit1_output_state_change(slave_reg_data_t new_reg_value) {

	slave_reg_data_t						reg_val = new_reg_value;

	if (reg_val) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}

	return reg_val;
}



static void main_group_slave_regs_init(volatile slave_reg_buf_t *main_group_regs, uint16_t main_group_regs_number) {

	uint16_t								reg_no;

	memset((void *)main_group_regs, 0, sizeof(slave_reg_buf_t) * main_group_regs_number);
	for (reg_no = 0; reg_no < (sizeof(slave_main_group_regs_ident_value) / sizeof(uint16_t)); reg_no++) {
		slave_registers_init_value((main_group_regs + reg_no), slave_main_group_regs_ident_value[reg_no], FALSE, FALSE, FALSE);
	}
	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_GLOBAL_CONFIG), 0, TRUE, TRUE, FALSE);
	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_GLOBAL_STATUS), 0, FALSE, TRUE, FALSE);

	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT1_CONF), 0, TRUE, TRUE, FALSE);
	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT2_CONF), 0, TRUE, TRUE, FALSE);
	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT3_CONF), 0, TRUE, TRUE, FALSE);
	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT4_CONF), 0, TRUE, TRUE, FALSE);

	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT1_STATUS), 0, FALSE, TRUE, FALSE);
	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT2_STATUS), 0, FALSE, TRUE, FALSE);
	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT3_STATUS), 0, FALSE, TRUE, FALSE);
	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT4_STATUS), 0, FALSE, TRUE, FALSE);

	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT1_OUTPUT_STATE), 0, TRUE, TRUE, FALSE);
}



void main_group_slave_status_set(BOOL system_ready) {

	slave_reg_data_t						reg_data;

	reg_data = 0;
	if (system_ready) {
		reg_data |= (1 << 0);
	}

	slave_set_regs_data(slave_main_group_regs, MAIN_SLAVE_REG_MODULE_READY, &reg_data, 1);
}

void main_group_slave_status_digital_in_state_set(uint32_t unit, uint32_t module, uint32_t input, BOOL state) {

	slave_reg_data_t						reg_data;

	reg_data = 0;
	if (state) {
		reg_data |= (1 << 0);
	}

	slave_set_regs_data(slave_main_group_regs, MAIN_SLAVE_REG_MODULE_UNIT1_STATUS, &reg_data, 1);
}

BOOL main_group_slave_regs_req(slave_regs_poll_func_req_t req, uint16_t reg_addr, slave_reg_data_t data) {

	BOOL									ret_val = FALSE;

	switch (req) {
	case SLAVE_REGS_POLL_FUNC_REQ_INIT:
		main_group_slave_regs_init(slave_main_group_regs, MAIN_SLAVE_REG_NR_MAX);
		break;
	case SLAVE_REGS_POLL_FUNC_REQ_WRITE:
		if (reg_addr == MAIN_SLAVE_REG_MODULE_GLOBAL_CONFIG) {
			slave_registers_write(&slave_main_group_regs[MAIN_SLAVE_REG_MODULE_GLOBAL_CONFIG], registers_global_conf);
		} else if (reg_addr == MAIN_SLAVE_REG_MODULE_UNIT1_OUTPUT_STATE) {
			slave_registers_write(&slave_main_group_regs[MAIN_SLAVE_REG_MODULE_UNIT1_OUTPUT_STATE], registers_unit1_output_state_change);
		} else if (reg_addr == MAIN_SLAVE_REG_LED_CONFIG) {

		}
		break;
	default:
		break;
	}

	return ret_val;
}
