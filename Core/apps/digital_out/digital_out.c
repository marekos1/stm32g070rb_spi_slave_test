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
#include "main_slave_regs.h"


static digital_out_base_t digital_out_base;

#define DIGITAL_OUT_CRIDT_ENTER()
#define DIGITAL_OUT_CRIDT_EXIT()



static msz_rc_t digital_out_set_state(const uint32_t digital_out_cap_idx, const bool state) {

	msz_rc_t								rc = MSZ_RC_OK;
	digital_out_cap_t						*do_cap;

	do_cap = &digital_out_base.cap[digital_out_cap_idx];
	if (digital_out_base.status[digital_out_cap_idx].state != state) {
		if (do_cap->unit_no == 0) {
			rc = board_set_digital_output_state(do_cap->module_no, do_cap->digital_out_no, state);
			if (rc == MSZ_RC_OK) {
				digital_out_base.status[digital_out_cap_idx].state = state;
			}
		} else {
			// External T200 Device
		}
	}

	return rc;
}

msz_rc_t digital_out_set_state_by_apps(const msz_t200_unit_no_t unit_no, const msz_t200_module_no_t module_no, const digital_out_no_t digital_out_no, const bool state) {

	msz_rc_t								rc = MSZ_RC_OK;
	uint32_t								digital_out_cap_idx;

	T_DG_DIGI_OUT("Enter unit_no: %u module_no: %u digital_out_no: %u set state to: %u", unit_no, module_no, digital_out_no, state);

	if (unit_no < MSZ_T200_UNITS) {
		if (module_no < MSZ_T200_MODULES) {
			digital_out_cap_idx = DIGITAL_OUT_OUTPUT_INDEX(unit_no, module_no, digital_out_no);
			DIGITAL_OUT_CRIDT_ENTER();
			if (digital_out_base.cap[digital_out_cap_idx].exists) {
				rc = digital_out_set_state(digital_out_cap_idx, state);
			}
			DIGITAL_OUT_CRIDT_EXIT();
			if (rc == MSZ_RC_OK) {
				main_group_slave_status_instance_state_set(unit_no, module_no, digital_out_no, state);
			}
		} else {
			rc = MSZ_RC_MODULE_NO_OUTSIDE_THE_RANGE;
		}
	} else {
		rc = MSZ_RC_UNIT_NO_OUTSIDE_THE_RANGE;
	}

	return rc;
}

msz_rc_t digital_out_device_conf_set(const msz_t200_unit_no_t unit_no, const msz_t200_module_no_t module_no, const bool enable) {

	msz_rc_t								rc = MSZ_RC_OK;
	digital_out_no_t						digital_out_no;
	uint32_t								digital_out_cap_idx;

	T_DG_DIGI_OUT("Enter unit_no: %u module_no: %u enable: %u", unit_no, module_no, enable);
	if (unit_no < MSZ_T200_UNITS) {
		if (module_no < MSZ_T200_MODULES) {
			DIGITAL_OUT_CRIDT_ENTER();
			for (digital_out_no = 0; digital_out_no < DIGITAL_OUTPUTS_PER_MODULE; digital_out_no++) {
				digital_out_cap_idx = DIGITAL_OUT_OUTPUT_INDEX(unit_no, module_no, digital_out_no);
				digital_out_base.cap[digital_out_cap_idx].exists = enable;
				digital_out_base.cap[digital_out_cap_idx].unit_no = unit_no;
				digital_out_base.cap[digital_out_cap_idx].module_no = module_no;
				digital_out_base.cap[digital_out_cap_idx].digital_out_no = digital_out_no;
				if (unit_no == 0) {
					board_digital_output_init(module_no, digital_out_no, enable);
				} else {
					//external
				}
			}
			digital_out_base.cap_change[unit_no] = true;
			DIGITAL_OUT_CRIDT_EXIT();
		} else {
			rc = MSZ_RC_MODULE_NO_OUTSIDE_THE_RANGE;
		}
	} else {
		rc = MSZ_RC_UNIT_NO_OUTSIDE_THE_RANGE;
	}

	return rc;
}
