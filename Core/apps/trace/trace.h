/*
 * trace.h
 *
 *  Created on: 14 gru 2022
 *      Author: marek s
 */

#ifndef APPS_TRACE_H_
#define APPS_TRACE_H_

#include <stdio.h>

#include "main.h"
#include "types.h"


#define TRACE_GRP_OPTION_ENABLE					(1 << 0)
#define TRACE_GRP_OPTION_SHOW_GPR				(1 << 1)
#define TRACE_GRP_OPTION_SHOW_LOCATION			(1 << 2)
#define TRACE_GRP_OPTION_SHOW_TIMESTAMP			(1 << 3)


/* Group config */
#if CONFIG_TRACE_ENABLE

#define TRACE_GRP_MAIN_ENABLE					(TRACE_GRP_OPTION_ENABLE | TRACE_GRP_OPTION_SHOW_GPR | TRACE_GRP_OPTION_SHOW_LOCATION | TRACE_GRP_OPTION_SHOW_TIMESTAMP)
//#define TRACE_GRP_SPI_SLAVE_ENABLE				(TRACE_GRP_OPTION_ENABLE | TRACE_GRP_OPTION_SHOW_GPR | TRACE_GRP_OPTION_SHOW_LOCATION | TRACE_GRP_OPTION_SHOW_TIMESTAMP)
#define TRACE_GRP_MAIN_SLAVE_REGS_ENABLE		(TRACE_GRP_OPTION_ENABLE | TRACE_GRP_OPTION_SHOW_GPR | TRACE_GRP_OPTION_SHOW_LOCATION | TRACE_GRP_OPTION_SHOW_TIMESTAMP)
#define TRACE_GRP_APPS_DIGITAL_IN_ENABLE		(TRACE_GRP_OPTION_ENABLE | TRACE_GRP_OPTION_SHOW_GPR | TRACE_GRP_OPTION_SHOW_LOCATION | TRACE_GRP_OPTION_SHOW_TIMESTAMP)
#define TRACE_GRP_APPS_DIGITAL_OUT_ENABLE		(TRACE_GRP_OPTION_ENABLE | TRACE_GRP_OPTION_SHOW_GPR | TRACE_GRP_OPTION_SHOW_LOCATION | TRACE_GRP_OPTION_SHOW_TIMESTAMP)

//#define TRACE_GRP_GPS_ENABLE					(TRACE_GRP_OPTION_ENABLE | TRACE_GRP_OPTION_SHOW_GPR | TRACE_GRP_OPTION_SHOW_LOCATION | TRACE_GRP_OPTION_SHOW_TIMESTAMP)
//#define TRACE_GRP_GPS_UBX_ENABLE				(TRACE_GRP_OPTION_ENABLE | TRACE_GRP_OPTION_SHOW_GPR | TRACE_GRP_OPTION_SHOW_LOCATION | TRACE_GRP_OPTION_SHOW_TIMESTAMP)
//#define TRACE_GRP_GPS_TOD_ENABLE				(TRACE_GRP_OPTION_ENABLE | TRACE_GRP_OPTION_SHOW_GPR | TRACE_GRP_OPTION_SHOW_LOCATION | TRACE_GRP_OPTION_SHOW_TIMESTAMP)
//#define TRACE_GRP_DPLL_ENABLE					(TRACE_GRP_OPTION_ENABLE | TRACE_GRP_OPTION_SHOW_GPR | TRACE_GRP_OPTION_SHOW_LOCATION | TRACE_GRP_OPTION_SHOW_TIMESTAMP)
//#define TRACE_GRP_E1G703_ENABLE				(TRACE_GRP_OPTION_ENABLE | TRACE_GRP_OPTION_SHOW_GPR | TRACE_GRP_OPTION_SHOW_LOCATION | TRACE_GRP_OPTION_SHOW_TIMESTAMP)

#endif


typedef enum {
	TRACE_GRP_MAIN = 0,
	TRACE_GRP_SPI_SLAVE,
	TRACE_GRP_MAIN_SLAVE_REGS,
	TRACE_GRP_APPS_DIGI_IN,
	TRACE_GRP_APPS_DIGI_OUT,
/*	
	TRACE_GRP_GPS,
	TRACE_GRP_GPS_UBX,
	TRACE_GRP_GPS_TOD,
	TRACE_GRP_DPLL,
	TRACE_GRP_E1_G703,
*/
	TRACE_GRP_NUMBER
} trace_grp_t;

typedef enum {
	TRACE_LVL_NONE = 0,
	TRACE_LVL_ERROR,
	TRACE_LVL_WARNING,
	TRACE_LVL_INFO,
	TRACE_LVL_DEBUG,
	TRACE_LVL_NOISE,
	TRACE_LVL_RACKET,

	TRACE_LVL_NUMBER
} trace_lvl_t;





#if defined(TRACE_GRP_MAIN_ENABLE)
#define T_DG_MAIN(_fmt, ...)             	trace_grp_printf(TRACE_GRP_MAIN, TRACE_LVL_DEBUG, (uint8_t)TRACE_GRP_MAIN_ENABLE, __FUNCTION__, __LINE__, _fmt, ##__VA_ARGS__)
#else /* TRACE_GRP_MAIN_ENABLE */
#define T_DG_MAIN(_fmt, ...)
#endif /* TRACE_GRP_MAIN_ENABLE */

#if defined(TRACE_GRP_SPI_SLAVE_ENABLE)
#define T_DG_SPISL(_fmt, ...)             	trace_grp_printf(TRACE_GRP_SPI_SLAVE, TRACE_LVL_DEBUG, (uint8_t)TRACE_GRP_SPI_SLAVE_ENABLE, __FUNCTION__, __LINE__, _fmt, ##__VA_ARGS__)
#define T_DG_HEX_SPISL(data, data_length)	trace_grp_hex_print(TRACE_GRP_SPI_SLAVE, TRACE_LVL_DEBUG, (uint8_t)TRACE_GRP_SPI_SLAVE_ENABLE, __FUNCTION__, __LINE__, data, data_length)
#else /* TRACE_GRP_SPI_SLAVE_ENABLE */
#define T_DG_SPISL(_fmt, ...)
#endif /* TRACE_GRP_SPI_SLAVE_ENABLE */

#if defined(TRACE_GRP_MAIN_SLAVE_REGS_ENABLE)
#define T_DG_MAIN_SR(_fmt, ...)             	trace_grp_printf(TRACE_GRP_MAIN_SLAVE_REGS, TRACE_LVL_DEBUG, (uint8_t)TRACE_GRP_MAIN_SLAVE_REGS_ENABLE, __FUNCTION__, __LINE__, _fmt, ##__VA_ARGS__)
#else /* TRACE_GRP_MAIN_SLAVE_REGS_ENABLE */
#define T_DG_MAIN_SR(_fmt, ...)
#endif /* TRACE_GRP_MAIN_SLAVE_REGS_ENABLE */

#if defined(TRACE_GRP_APPS_DIGITAL_IN_ENABLE)
#define T_DG_DIGI_IN(_fmt, ...)             	trace_grp_printf(TRACE_GRP_APPS_DIGI_IN, TRACE_LVL_DEBUG, (uint8_t)TRACE_GRP_APPS_DIGITAL_IN_ENABLE, __FUNCTION__, __LINE__, _fmt, ##__VA_ARGS__)
#else /* TRACE_GRP_MAIN_SLAVE_REGS_ENABLE */
#define T_DG_DIGI_IN(_fmt, ...)
#endif /* TRACE_GRP_MAIN_SLAVE_REGS_ENABLE */

#if defined(TRACE_GRP_APPS_DIGITAL_OUT_ENABLE)
#define T_DG_DIGI_OUT(_fmt, ...)             	trace_grp_printf(TRACE_GRP_APPS_DIGI_OUT, TRACE_LVL_DEBUG, (uint8_t)TRACE_GRP_APPS_DIGITAL_OUT_ENABLE, __FUNCTION__, __LINE__, _fmt, ##__VA_ARGS__)
#else /* TRACE_GRP_MAIN_SLAVE_REGS_ENABLE */
#define T_DG_DIGI_OUT(_fmt, ...)
#endif /* TRACE_GRP_MAIN_SLAVE_REGS_ENABLE */



#if CONFIG_TRACE_ENABLE

msz_rc_t trace_grp_printf(const trace_grp_t grp, const trace_lvl_t lvl, const uint8_t opt, const char *function, const uint32_t line_no,
						const char *fmt, ...);

msz_rc_t trace_grp_hex_print(const trace_grp_t grp, const trace_lvl_t lvl, const uint8_t opt, const char *function, const uint32_t line_no,
							void *hex_data, uint16_t hex_data_length);

msz_rc_t trace_cli_print(const char *txt);

#endif /* CONFIG_TRACE_ENABLE */

msz_rc_t trace_cli_init(void);

#endif /* APPS_TRACE_H_ */
