/*
 * digital_out.h
 *
 *  Created on: Nov 20, 2024
 *      Author: marek
 */

#ifndef SRC_DIGITAL_OUT_DIGITAL_OUT_H_
#define SRC_DIGITAL_OUT_DIGITAL_OUT_H_

#include "main.h"

typedef struct {
//	digital_in_cap_t 						cap[DIGITAL_IN_TOTAL_INPUT];
	bool									cap_change[MSZ_T200_UNITS];

} digital_out_base_t;

#endif /* SRC_DIGITAL_OUT_DIGITAL_OUT_H_ */
