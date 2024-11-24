/*
 * trace_cli.c
 *
 *  Created on: 13 gru 2022
 *      Author: mszpakowski
 */

#include "main.h"
#include "types.h"

#if CONFIG_TRACE_ENABLE

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "board.h"
#include "trace.h"
#include "uart/uart.h"


typedef struct {
	uart_t									uart;

} trace_cli_uart_t;

static trace_cli_uart_t trace_cli_uart;

static trace_lvl_t trace_lvl[TRACE_GRP_NUMBER] = {
	/* TRACE_GRP_MAIN */ 				TRACE_LVL_DEBUG,
	/* TRACE_GRP_SPI_SLAVE */ 			TRACE_LVL_DEBUG,
	/* TRACE_GRP_MAIN_SLAVE_REGS */		TRACE_LVL_DEBUG,
	/* TRACE_GRP_APPS_DIGI_IN */		TRACE_LVL_DEBUG,
	/* TRACE_GRP_APPS_DIGI_OUT */		TRACE_LVL_DEBUG,
};

static const char *trace_grp_name[TRACE_GRP_NUMBER] = {
	/* TRACE_GRP_MAIN */ 				"main",
	/* TRACE_GRP_SPI_SLAVE */			"SPIsl",
	/* TRACE_GRP_MAIN_SLAVE_REGS */		"mainSR",
	/* TRACE_GRP_APPS_DIGI_IN */		"DigiIN",
	/* TRACE_GRP_APPS_DIGI_OUT */		"DigiOUT",
};

static inline char trace_print_lvl_char(const trace_lvl_t lvl) {

	char 									lvl_c;

	switch (lvl) {
	case TRACE_LVL_NONE:
		lvl_c = ' ';
		break;
	case TRACE_LVL_ERROR:
		lvl_c = 'E';
		break;
	case TRACE_LVL_WARNING:
		lvl_c = 'W';
		break;
	case TRACE_LVL_INFO:
		lvl_c = 'I';
		break;
	case TRACE_LVL_DEBUG:
		lvl_c = 'D';
		break;
	case TRACE_LVL_NOISE:
		lvl_c = 'N';
		break;
	case TRACE_LVL_RACKET:
		lvl_c = 'R';
		break;
	default:
		lvl_c = '?';
		break;
	}

	return lvl_c;
}

static int trace_grp_print_header(char *txt_buf, int txt_length, const trace_grp_t grp, const trace_lvl_t lvl, const uint8_t opt,
									const char *function, const uint32_t line_no) {

	int										header_length;
	uint32_t								current_ms_time, sec, min, hr;

	header_length = 0;
	if (opt & TRACE_GRP_OPTION_SHOW_GPR) {
		header_length += snprintf(txt_buf + header_length, txt_length - header_length, "%c %7s ", trace_print_lvl_char(lvl), trace_grp_name[grp]);
	}
	if (opt & TRACE_GRP_OPTION_SHOW_TIMESTAMP) {
		current_ms_time = system_ms_time();
		sec = current_ms_time / 1000;
		min = sec / 60;
		hr = min / 60;
		min = min - (hr * 60);
		sec = (current_ms_time / 1000) - (min * 60) - (hr * 60 * 60);
		current_ms_time = current_ms_time - (((min * 60) + (hr * 60 * 60) + sec) * 1000);
		header_length += snprintf(txt_buf + header_length, txt_length - header_length, "%02lu:%02lu:%02lu.%03lu ", hr, min, sec, current_ms_time);
	}
	if (opt & TRACE_GRP_OPTION_SHOW_LOCATION) {
		header_length += snprintf(txt_buf + header_length, txt_length - header_length, "%40s#%4lu: ", function, line_no);
	}

	return header_length;
}


msz_rc_t trace_grp_printf(const trace_grp_t grp, const trace_lvl_t lvl, const uint8_t opt, const char *function, const uint32_t line_no,
						const char *fmt, ...) {

	msz_rc_t									rc = MSZ_RC_OK;
	va_list         						args;
	char									txt_buf[256];
	int										txt_length;

	if ((opt & TRACE_GRP_OPTION_ENABLE) && (lvl <= trace_lvl[grp])){
		txt_length = 0;
		txt_length += trace_grp_print_header(txt_buf, sizeof(txt_buf) - txt_length, grp, lvl, opt, function, line_no);
	    va_start(args, fmt);
	    txt_length += vsnprintf(txt_buf + txt_length, sizeof(txt_buf) - txt_length, fmt, args);
	    va_end(args);
	    if (txt_length > 0) {
	    	txt_length += snprintf(txt_buf + txt_length, sizeof(txt_buf) - txt_length, "\r\n");
	    	uart_write(&trace_cli_uart.uart, (uint8_t *)txt_buf, (uint16_t)txt_length);
	    }
	}

	return rc;
}

msz_rc_t trace_grp_hex_print(const trace_grp_t grp, const trace_lvl_t lvl, const uint8_t opt, const char *function, const uint32_t line_no,
							void *hex_data, uint16_t hex_data_length) {

	msz_rc_t								rc = MSZ_RC_OK;
	char									txt_buf[1024];
	int										txt_length;
	uint16_t								byte_no;

	if ((lvl <= trace_lvl[grp]) && opt){
		txt_length = 0;
		txt_length += trace_grp_print_header(txt_buf, sizeof(txt_buf) - txt_length, grp, lvl, opt, function, line_no);
		txt_length += snprintf(txt_buf + txt_length, sizeof(txt_buf) - txt_length, "Data length: %u", hex_data_length);
		for (byte_no = 0; byte_no < hex_data_length; byte_no++) {
			if ((byte_no % 16) == 0) {
				txt_length += snprintf(txt_buf + txt_length, sizeof(txt_buf) - txt_length, "\r\n%04X   ", byte_no);
			}
			if ((byte_no % 4) == 0) {
				txt_length += snprintf(txt_buf + txt_length, sizeof(txt_buf) - txt_length, " ");
			}
			txt_length += snprintf(txt_buf + txt_length, sizeof(txt_buf) - txt_length, "%02X ", *(((uint8_t *)hex_data) + byte_no));
		}
	    if (txt_length > 0) {
	    	txt_length += snprintf(txt_buf + txt_length, sizeof(txt_buf) - txt_length, "\r\n");
	    	uart_write(&trace_cli_uart.uart, (uint8_t *)txt_buf, (uint16_t)txt_length);
	    }
	}

	return rc;
}


msz_rc_t trace_cli_print(const char *txt) {

	msz_rc_t									rc = MSZ_RC_OK;

	uart_write(&trace_cli_uart.uart, (uint8_t *)txt, strlen(txt));

	return rc;
}


msz_rc_t trace_cli_init(void) {

	msz_rc_t								rc = MSZ_RC_OK;
	uart_conf_t								uart_conf;

	memset(&uart_conf, 0, sizeof(uart_conf));
	uart_conf.baud = 115200;
	uart_conf.data_bits = 8;
	uart_conf.stop_bits = UART_STOP_BITS_1;
	uart_conf.parity = UART_PARITY_NONE;
	uart_conf.rx_enable = false;
	uart_conf.tx_enable = true;
	rc = uart_init(&trace_cli_uart.uart, &cli_trace_uart_dev, &uart_conf);
#if 0
	trace_cli_print("\033[2J");
#else
	trace_cli_print("\n\r\n\r\n\r");
#endif

	return rc;
}

#else /* CONFIG_TRACE_ENABLE */

msz_rc_t trace_cli_init(void) {

	msz_rc_t									rc = MSZ_RC_OK;

	return rc;
}

#endif /* CONFIG_TRACE_ENABLE */

