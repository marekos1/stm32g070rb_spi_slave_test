/*
 * digital_out_api.h
 *
 *  Created on: Nov 20, 2024
 *      Author: marek
 */

#ifndef SRC_DIGITAL_OUT_DIGITAL_OUT_API_H_
#define SRC_DIGITAL_OUT_DIGITAL_OUT_API_H_

#include "main.h"

msz_rc_t digital_out_set_state_by_apps(const msz_t200_unit_no_t unit_no, const msz_t200_module_no_t module_no, const digital_out_no_t digital_out_no, const bool state);

msz_rc_t digital_out_device_conf_set(const msz_t200_unit_no_t unit_no, const msz_t200_module_no_t module_no, const bool enable);

#endif /* SRC_DIGITAL_OUT_DIGITAL_OUT_API_H_ */
