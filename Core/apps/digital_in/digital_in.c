/*
 * digital_in.c
 *
 *  Created on: May 19, 2024
 *      Author: marek
 */

#include <string.h>

#include "main.h"
#include "types.h"
#include "board.h"
#include "digital_in/digital_in.h"
#include "trace/trace.h"
#include "main_slave_regs.h"
#include "board.h"


static digital_in_base_t digital_in_base;

#define DIGITAL_IN_CRIDT_ENTER()
#define DIGITAL_IN_CRIDT_EXIT()










static msz_rc_t digital_in_internal_input_read_state(const msz_t200_module_no_t module_no, const digital_in_no_t digital_in_no, bool *state) {

	msz_rc_t								rc = MSZ_RC_OK;

	*state = board_digital_in_get_state(module_no, digital_in_no);

	return rc;
}

static msz_rc_t digital_in_external_module_poll(void) {

	msz_rc_t								rc = MSZ_RC_ERR_NOT_SUPPORTED;

	return rc;
}

static msz_rc_t digital_in_unit_poll(const msz_t200_unit_no_t unit_no) {

	msz_rc_t								rc = MSZ_RC_OK;
	msz_t200_module_no_t					module_no;
	uint32_t								digital_in_cap_idx;
	digital_in_cap_t						*di_cap;
	digital_in_no_t							digital_in_no;
	bool									state;
	digital_in_input_state_t				*status;

	if (unit_no == 0) {
		for (module_no = 0; module_no < MSZ_T200_MODULES; module_no++) {
			for (digital_in_no = 0; digital_in_no < DIGITAL_INPUTS_PER_MODULE; digital_in_no++) {
				digital_in_cap_idx = DIGITAL_IN_INPUT_INDEX(unit_no, module_no, digital_in_no);
				di_cap = &digital_in_base.cap[digital_in_cap_idx];
				if (di_cap->exists) {
					rc = digital_in_internal_input_read_state(module_no, digital_in_no, &state);
					if (rc == MSZ_RC_OK) {
						status = &digital_in_base.status[digital_in_cap_idx];
						if (state != status->state) {
							main_group_slave_status_instance_state_set(unit_no, module_no, digital_in_no, state);
							status->state = state;
						}
					}
				}
			}
		}
	} else {
		rc = digital_in_external_module_poll();
	}

	return rc;
}

static msz_rc_t digital_in_unit_module_init(const msz_t200_unit_no_t unit_no, const msz_t200_module_no_t module_no) {

	msz_rc_t								rc = MSZ_RC_OK;
	uint32_t								digital_in_cap_idx;
	digital_in_no_t							digital_in_no;
	digital_in_cap_t						*di_cap;

	if (unit_no == 0) {
		for (digital_in_no = 0; digital_in_no < DIGITAL_INPUTS_PER_MODULE; digital_in_no++) {
			digital_in_cap_idx = DIGITAL_IN_INPUT_INDEX(unit_no, module_no, digital_in_no);
			di_cap = &digital_in_base.cap[digital_in_cap_idx];
			board_digital_input_init(module_no, digital_in_no, di_cap->exists);
		}
	} else {
		// external
	}

	return rc;
}

msz_rc_t digital_in_device_conf_set(const msz_t200_unit_no_t unit_no, const msz_t200_module_no_t module_no, const bool enable) {

	msz_rc_t								rc = MSZ_RC_OK;
	digital_in_no_t							digital_in_no, digital_in_max_in_module;
	uint32_t								digital_in_cap_idx;
	digital_in_cap_t						*di_cap;

	T_DG_DIGI_IN("Enter unit_no: %u module_no: %u enable: %u", unit_no, module_no, enable);
	if (unit_no < MSZ_T200_UNITS) {
		if (module_no < MSZ_T200_MODULES) {
			digital_in_max_in_module = 8;
			DIGITAL_IN_CRIDT_ENTER();
			for (digital_in_no = 0; digital_in_no < digital_in_max_in_module; digital_in_no++) {
				digital_in_cap_idx = DIGITAL_IN_INPUT_INDEX(unit_no, module_no, digital_in_no);
				di_cap = &digital_in_base.cap[digital_in_cap_idx];
				di_cap->exists = enable;
				di_cap->unit_no = unit_no;
				di_cap->module_no = module_no;
			}
			digital_in_base.cap_change[unit_no][module_no] = true;
			DIGITAL_IN_CRIDT_EXIT();
		} else {
			rc = MSZ_RC_MODULE_NO_OUTSIDE_THE_RANGE;
		}
	} else {
		rc = MSZ_RC_UNIT_NO_OUTSIDE_THE_RANGE;
	}

	return rc;
}


void digital_in_poll(void) {

	msz_rc_t								rc = MSZ_RC_OK;
	msz_t200_unit_no_t 						unit_no;
	msz_t200_module_no_t 					module_no;

	DIGITAL_IN_CRIDT_ENTER();
	for (unit_no = 0; unit_no < MSZ_T200_UNITS; unit_no++) {
		for (module_no = 0; module_no < MSZ_T200_MODULES; module_no++) {
			if (digital_in_base.cap_change[unit_no][module_no]) {
				rc = digital_in_unit_module_init(unit_no, module_no);
				if (rc == MSZ_RC_OK) {
					digital_in_base.cap_change[unit_no][module_no] = false;
				}
			}
		}
	}
	for (unit_no = 0; unit_no < MSZ_T200_UNITS; unit_no++) {
		digital_in_unit_poll(unit_no);
	}
	DIGITAL_IN_CRIDT_EXIT();
}
