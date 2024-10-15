/*
 * digital_in_api.h
 *
 *  Created on: Jul 7, 2024
 *      Author: marek
 */

#ifndef DIGITAL_IN_API_H_
#define DIGITAL_IN_API_H_


#include "main.h"
#include "types.h"

typedef struct {
	BOOL 									state;
} digital_in_status_t;




msz_rc_t digital_in_device_conf_set(const msz_t200_unit_no_t unit_no, const msz_t200_module_no_t module_no, const msz_t200_module_type_t module_type, const BOOL enable);

void digital_in_poll(void);

#endif /* DIGITAL_IN_API_H_ */
