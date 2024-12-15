/*
 * digirtal_in.h
 *
 *  Created on: Jul 7, 2024
 *      Author: marek
 */

#ifndef SRC_DIGITAL_IN_DIGIRTAL_IN_H_
#define SRC_DIGITAL_IN_DIGIRTAL_IN_H_


#include "types.h"
#include "digital_in/digital_in_api.h"


#define DIGITAL_IN_UNIT_TOTAL_INPUT (MSZ_T200_MODULES * DIGITAL_INPUTS_PER_MODULE)

#define DIGITAL_IN_TOTAL_INPUT (MSZ_T200_UNITS * DIGITAL_IN_UNIT_TOTAL_INPUT)

#define DIGITAL_IN_INPUT_INDEX(unit, module, digital_in) ((unit * MSZ_T200_UNITS) + (module * MSZ_T200_MODULES) + digital_in)


typedef struct {
	bool									exists;
	msz_t200_unit_no_t						unit_no;
	msz_t200_module_no_t					module_no;
} digital_in_cap_t;

typedef struct {
	bool									state;
} digital_in_input_state_t;

typedef struct {
	digital_in_cap_t 						cap[DIGITAL_IN_TOTAL_INPUT];
	bool									cap_change[MSZ_T200_UNITS][MSZ_T200_MODULES];

	digital_in_input_state_t				status[DIGITAL_IN_TOTAL_INPUT];
} digital_in_base_t;

#endif /* SRC_DIGITAL_IN_DIGIRTAL_IN_H_ */
