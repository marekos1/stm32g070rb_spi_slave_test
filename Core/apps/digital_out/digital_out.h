/*
 * digital_out.h
 *
 *  Created on: Nov 20, 2024
 *      Author: marek
 */

#ifndef SRC_DIGITAL_OUT_DIGITAL_OUT_H_
#define SRC_DIGITAL_OUT_DIGITAL_OUT_H_

#include "main.h"



#define DIGITAL_OUT_UNIT_TOTAL_OUTPUT 	(MSZ_T200_MODULES * DIGITAL_OUTPUTS_PER_MODULE)

#define DIGITAL_OUT_TOTAL_OUTPUT		(MSZ_T200_UNITS * DIGITAL_OUT_UNIT_TOTAL_OUTPUT)

#define DIGITAL_OUT_OUTPUT_INDEX(unit, module, digital_out) ((unit * MSZ_T200_UNITS) + (module * MSZ_T200_MODULES) + digital_out)



typedef struct {
	bool									exists;
	msz_t200_unit_no_t						unit_no;
	msz_t200_module_no_t					module_no;
	digital_out_no_t						digital_out_no;
} digital_out_cap_t;

typedef struct {
	bool									state;
} digital_out_output_statatus_t;


typedef struct {
	digital_out_cap_t 						cap[DIGITAL_OUT_TOTAL_OUTPUT];
	bool									cap_change[MSZ_T200_UNITS];
	digital_out_output_statatus_t			status[DIGITAL_OUT_TOTAL_OUTPUT];

} digital_out_base_t;

#endif /* SRC_DIGITAL_OUT_DIGITAL_OUT_H_ */
