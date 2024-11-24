/*
 * digital_out.c
 *
 *  Created on: Nov 20, 2024
 *      Author: marek
 */


#include <string.h>

#include "main.h"
#include "types.h"
#include "board.h"
#include "digital_out/digital_out.h"
#include "trace/trace.h"


//static digital_out_base_t digital_out_base;

#define DIGITAL_OUT_CRIDT_ENTER()
#define DIGITAL_OUT_CRIDT_EXIT()



msz_rc_t digital_out_set_state_by_apps(const msz_t200_unit_no_t unit_no, const msz_t200_module_no_t module_no, const digital_out_no_t digital_out_no, const bool state) {

	msz_rc_t								rc = MSZ_RC_OK;

	T_DG_DIGI_OUT("Enter unit_no: %u module_no: %u digital_out_no: %u set state to: %u", unit_no, module_no, digital_out_no, state);

	if ((unit_no < MSZ_T200_UNITS) && (module_no < MSZ_T200_MODULES) && (digital_out_no < DIGITAL_OUTPUTS_PER_MODULE)) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, state);
	} else {
		rc = MSZ_RC_ERR_INV_ARG;
	}

	return rc;
}

msz_rc_t digital_out_device_conf_set(const msz_t200_unit_no_t unit_no, const msz_t200_module_no_t module_no, const bool enable) {

	msz_rc_t								rc = MSZ_RC_OK;

	T_DG_DIGI_OUT("Enter unit_no: %u module_no: %u enable: %u", unit_no, module_no, enable);

#if 0
	digital_in_no_t							digital_in_no, digital_in_max_in_module;
	uint32_t								digital_in_cap_idx;

	T_DG_DIGI_IN("Enter unit_no: %u module_no: %u enable: %u", unit_no, module_no, enable);
	if (unit_no < MSZ_T200_UNITS) {
		if (module_no < MSZ_T200_MODULES) {
			digital_in_max_in_module = 8;
			DIGITAL_IN_CRIDT_ENTER();
			for (digital_in_no = 0; digital_in_no < digital_in_max_in_module; digital_in_no++) {
				digital_in_cap_idx = DIGITAL_IN_INPUT_INDEX(unit_no, module_no, digital_in_no);
				digital_in_base.cap[digital_in_cap_idx].exists = enable;
				digital_in_base.cap[digital_in_cap_idx].unit_no = unit_no;
				digital_in_base.cap[digital_in_cap_idx].module_no = module_no;
			}
			digital_in_base.cap_change[unit_no] = true;
			DIGITAL_IN_CRIDT_EXIT();
		} else {
			rc = MSZ_RC_MODULE_NO_OUTSIDE_THE_RANGE;
		}
	} else {
		rc = MSZ_RC_UNIT_NO_OUTSIDE_THE_RANGE;
	}
#endif
	return rc;
}
