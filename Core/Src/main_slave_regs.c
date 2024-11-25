/*
 * main_slave_regs.c
 *
 *  Created on: 05.05.2024
 *      Author: marek
 */

#include <string.h>

#include "main.h"
#include "types.h"
#include "trace/trace.h"

#if defined(STM32F0)
#include "stm32f0xx.h"
#elif defined(STM32G431xx)
#include "stm32g4xx.h"
#endif

#include "slave_regs/slave_regs.h"
#include "main_slave_regs.h"

#if CONFIG_APPS_DIGITAL_IN_ENABLE
#include "digital_in/digital_in_api.h"
#endif /* CONFIG_APPS_DIGITAL_IN_ENABLE */

#if CONFIG_APPS_DIGITAL_OUT_ENABLE
#include "digital_out/digital_out_api.h"
#endif /* CONFIG_APPS_DIGITAL_OUT_ENABLE */


volatile slave_reg_buf_t slave_main_group_regs[MAIN_SLAVE_REG_NR_MAX];

static const slave_reg_data_t slave_main_group_regs_ident_value[6] = {
		/* 0 */ 0x00000001, 													/* - Module ready */
		/* 1 */ 0x00000082, 													/* - Module Type 0x0082 - GPS */
		/* 2 */ (slave_reg_data_t)SYSTEM_HARDWARE_REVISON,						/* - Module Hardware revision */
		/* 3 */ (slave_reg_data_t)SYSTEM_SOFTWARE_VERSION_MINOR, 				/* - Module Firmware revision */
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





static slave_reg_data_t registers_unit_conf(const slave_reg_addr_t addr, slave_reg_data_t new_reg_value) {

	slave_reg_data_t						reg_val = new_reg_value, status_reg_data;
	msz_t200_module_no_t					module_no;
	msz_t200_module_type_t					module_type;
	msz_rc_t								rc = MSZ_RC_OK;

	slave_get_regs_data(slave_main_group_regs, MAIN_SLAVE_REG_MODULE_GLOBAL_STATUS, &status_reg_data, 1);
	if (addr == MAIN_SLAVE_REG_MODULE_UNIT1_CONF) {
		slave_reset_bit_in_reg(status_reg_data, MAIN_SLAVE_REG_MODULE_GLOBAL_STATUS_NOT_SUPPORT_UNIT);
		for (module_no = 0; module_no < MSZ_T200_MODULES; module_no++) {
			module_type = (msz_t200_module_type_t)((new_reg_value >> (8 * module_no)) & 0x000000FF);
			slave_reset_bit_in_reg(status_reg_data, (MAIN_SLAVE_REG_MODULE_GLOBAL_STATUS_U0M0_NOT_SUPPORT_CONF + module_no));
			switch (module_type) {
			case MSZ_T200_MODULE_TYPE_NONE_OR_EMPTY:
#if CONFIG_APPS_DIGITAL_IN_ENABLE
				digital_in_device_conf_set(0, module_no, false);
#endif /* CONFIG_APPS_DIGITAL_IN_ENABLE */
#if CONFIG_APPS_DIGITAL_OUT_ENABLE
				digital_out_device_conf_set(0, module_no, false);
#endif /* CONFIG_APPS_DIGITAL_OUT_ENABLE */
				break;
			case MSZ_T200_MODULE_TYPE_DIGITAL_INPUT8:
#if CONFIG_APPS_DIGITAL_IN_ENABLE
				rc = digital_in_device_conf_set(0, module_no, true);
#else /* CONFIG_APPS_DIGITAL_IN_ENABLE */
				rc = MSZ_RC_ERR_NOT_SUPPORTED;
#endif /* CONFIG_APPS_DIGITAL_IN_ENABLE */
				break;
			case MSZ_T200_MODULE_TYPE_DIGITAL_OUTPUT8:
#if CONFIG_APPS_DIGITAL_OUT_ENABLE
				digital_out_device_conf_set(0, module_no, true);
#else /* CONFIG_APPS_DIGITAL_OUT_ENABLE */
				rc = MSZ_RC_ERR_NOT_SUPPORTED;
#endif /* CONFIG_APPS_DIGITAL_OUT_ENABLE */
				break;
			default:
				rc = MSZ_RC_DIGITAL_IN_UNSUPPORTED_MODULE_TYPE;
				break;
			}
			if (rc != MSZ_RC_OK) {
				slave_set_bit_in_reg(status_reg_data, (MAIN_SLAVE_REG_MODULE_GLOBAL_STATUS_U0M0_NOT_SUPPORT_CONF + module_no));
			}
		}
	} else {
		slave_set_bit_in_reg(status_reg_data, MAIN_SLAVE_REG_MODULE_GLOBAL_STATUS_NOT_SUPPORT_UNIT);
	}
	slave_set_regs_data(MAIN_SLAVE_REG_BASE_ADDR + MAIN_SLAVE_REG_MODULE_GLOBAL_STATUS, &status_reg_data, 1);


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
	for (reg_no = 0; reg_no < (sizeof(slave_main_group_regs_ident_value) / sizeof(slave_reg_data_t)); reg_no++) {
		slave_registers_init_value((main_group_regs + reg_no), slave_main_group_regs_ident_value[reg_no], false, false, false, false);
	}
	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_GLOBAL_CONFIG), 0, true, true, false, false);
	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_GLOBAL_STATUS), 0, false, true, false, false);

	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT1_CONF), 0, true, true, false, false);
	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT2_CONF), 0, true, true, false, false);
	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT3_CONF), 0, true, true, false, false);
	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT4_CONF), 0, true, true, false, false);

	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT1_STATUS), 0, false, true, false, true);
	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT2_STATUS), 0, false, true, false, false);
	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT3_STATUS), 0, false, true, false, false);
	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT4_STATUS), 0, false, true, false, false);

	slave_registers_init_value((main_group_regs + MAIN_SLAVE_REG_MODULE_UNIT1_OUTPUT_STATE), 0, true, true, false, false);
}



void main_group_slave_status_set(bool system_ready) {

	slave_reg_data_t						reg_data;

	reg_data = 0;
	if (system_ready) {
		reg_data |= (1 << 0);
	}

	slave_set_regs_data(MAIN_SLAVE_REG_BASE_ADDR + MAIN_SLAVE_REG_MODULE_READY, &reg_data, 1);
}

void main_group_slave_status_digital_in_state_set(uint32_t unit, uint32_t module, uint32_t input, bool state) {

	slave_reg_data_t						reg_data;

	reg_data = 0;
	if (state) {
		reg_data |= (1 << 0);
	}

	slave_set_regs_data(MAIN_SLAVE_REG_BASE_ADDR + MAIN_SLAVE_REG_MODULE_UNIT1_STATUS, &reg_data, 1);
}

bool main_group_slave_regs_req(slave_regs_poll_func_req_t req, uint16_t reg_addr, slave_reg_data_t data) {

	bool									ret_val = false;

	switch (req) {
	case SLAVE_REGS_POLL_FUNC_REQ_INIT:
		main_group_slave_regs_init(slave_main_group_regs, MAIN_SLAVE_REG_NR_MAX);
		break;
	case SLAVE_REGS_POLL_FUNC_REQ_WRITE:
		T_DG_MAIN_SR("Write reg_addr: 0x%08X <- 0x%08X", reg_addr, data);
		if (reg_addr == MAIN_SLAVE_REG_MODULE_GLOBAL_CONFIG) {
			slave_registers_write(&slave_main_group_regs[MAIN_SLAVE_REG_MODULE_GLOBAL_CONFIG], registers_global_conf);
		} else if ((reg_addr >= MAIN_SLAVE_REG_MODULE_UNIT1_CONF) && (reg_addr <= MAIN_SLAVE_REG_MODULE_UNIT4_CONF)) {
			slave_registers_address_write(&slave_main_group_regs[reg_addr], reg_addr, registers_unit_conf);
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
