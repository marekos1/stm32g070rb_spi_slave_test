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
#include "digital_in/digirtal_in.h"


static digital_in_base_t digital_in_base;

#define DIGITAL_IN_CRIDT_ENTER()
#define DIGITAL_IN_CRIDT_EXIT()










static msz_rc_t digital_in_internal_input_read_status(const msz_t200_module_no_t module_no, const digital_in_no_t digital_in_no, digital_in_status_t *status) {

	msz_rc_t								rc = MSZ_RC_OK;
	BOOL									state;

	state = board_read_digital_input_state(module_no, digital_in_no);
	status->state = state;

	return rc;
}

static msz_rc_t digital_in_internal_module_poll(const msz_t200_module_no_t module_no, const digital_in_cap_t *cap, digital_in_status_t *status) {

	msz_rc_t								rc = MSZ_RC_OK;
	digital_in_no_t							digital_in_no;

	for (digital_in_no = 0; digital_in_no < DIGITAL_INPUTS_PER_MODULE; digital_in_no++) {
		digital_in_internal_input_read_status((cap + digital_in_no)->module_no, digital_in_no, status + digital_in_no);
	}

	return rc;
}

static msz_rc_t digital_in_external_module_poll(const digital_in_cap_t *cap) {

	msz_rc_t								rc = MSZ_RC_ERR_NOT_SUPPORTED;

	(void)cap;

	return rc;
}

static msz_rc_t digital_in_unit_poll(const msz_t200_unit_no_t unit_no, const digital_in_cap_t *cap, digital_in_status_t *status) {

	msz_rc_t								rc = MSZ_RC_OK;
	msz_t200_module_no_t					module_no;
	uint32_t								digital_in_cap_idx;

	if (unit_no == 0) {
		for (module_no = 0; module_no < MSZ_T200_MODULES; module_no++) {
			digital_in_cap_idx = DIGITAL_IN_INPUT_INDEX(unit_no, module_no, 0);
			if ((cap + digital_in_cap_idx)->exists) {
				rc = digital_in_internal_module_poll(module_no, cap + digital_in_cap_idx, status + digital_in_cap_idx);
			}
		}
	} else {
		rc = digital_in_external_module_poll(cap);
	}

	return rc;
}

static msz_rc_t digital_in_unit_init(const msz_t200_unit_no_t unit_no, const digital_in_cap_t *cap) {

	msz_rc_t								rc = MSZ_RC_OK;
	msz_t200_module_no_t					module_no;
	uint32_t								digital_in_cap_idx;
	digital_in_no_t							digital_in_no;

	if (unit_no == 0) {
		for (module_no = 0; module_no < MSZ_T200_MODULES; module_no++) {
			digital_in_cap_idx = DIGITAL_IN_INPUT_INDEX(unit_no, module_no, 0);
			for (digital_in_no = 0; digital_in_no < DIGITAL_INPUTS_PER_MODULE; digital_in_no++) {
				board_init_digital_input_state(module_no, digital_in_no, (cap + digital_in_cap_idx)->exists);
			}
		}
	} else {
		rc = MSZ_RC_ERR_NOT_SUPPORTED;
	}

	return rc;
}

msz_rc_t digital_in_device_conf_set(const msz_t200_unit_no_t unit_no, const msz_t200_module_no_t module_no, const msz_t200_module_type_t module_type, const BOOL enable) {

	msz_rc_t								rc = MSZ_RC_OK;
	digital_in_no_t							digital_in_no, digital_in_max_in_module;
	uint32_t								digital_in_cap_idx;

	if (unit_no < MSZ_T200_UNITS) {
		if (module_no < MSZ_T200_MODULES) {
			if (module_type == MSZ_T200_MODULE_TYPE_DIGITAL_INPUT8) {
				digital_in_max_in_module = 8;
				DIGITAL_IN_CRIDT_ENTER();
				for (digital_in_no = 0; digital_in_no < digital_in_max_in_module; digital_in_no++) {
					digital_in_cap_idx = DIGITAL_IN_INPUT_INDEX(unit_no, module_no, digital_in_no);
					digital_in_base.cap[digital_in_cap_idx].exists = enable;
					digital_in_base.cap[digital_in_cap_idx].unit_no = unit_no;
					digital_in_base.cap[digital_in_cap_idx].module_no = module_no;
				}
				digital_in_base.cap_change[unit_no] = TRUE;
				DIGITAL_IN_CRIDT_EXIT();
			} else {
				rc = MSZ_RC_DIGITAL_IN_UNSUPPORTED_MODULE_TYPE;
			}
		} else {
			rc = MSZ_RC_MODULE_NO_OUTSIDE_THE_RANGE;
		}
	} else {
		rc = MSZ_RC_MODULE_NO_OUTSIDE_THE_RANGE;
	}

	return rc;
}


void digital_in_poll(void) {

	msz_rc_t								rc = MSZ_RC_OK;
	msz_t200_unit_no_t 						unit_no;
	static digital_in_cap_t					unit_cap[MSZ_T200_UNITS][DIGITAL_IN_UNIT_TOTAL_INPUT];
	digital_in_status_t 					status[DIGITAL_IN_TOTAL_INPUT];

	DIGITAL_IN_CRIDT_ENTER();
	for (unit_no = 0; unit_no < MSZ_T200_UNITS; unit_no++) {
		if (digital_in_base.cap_change[unit_no]) {
			memcpy(unit_cap[unit_no], digital_in_base.cap, sizeof(digital_in_cap_t) * 32);
			digital_in_base.cap_change[unit_no] = FALSE;
			digital_in_unit_init(unit_no, unit_cap[unit_no]);
		}
	}
	DIGITAL_IN_CRIDT_EXIT();

	for (unit_no = 0; unit_no < MSZ_T200_UNITS; unit_no++) {
		rc = digital_in_unit_poll(unit_no, unit_cap[unit_no], status);
		if (rc == MSZ_RC_OK) {

		}
	}

}
