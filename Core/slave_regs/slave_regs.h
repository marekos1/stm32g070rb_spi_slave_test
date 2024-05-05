/*
 * slave_regs.h
 *
 *  Created on: 05.05.2024
 *      Author: marek
 */

#ifndef __SLAVE_REGS_H_
#define __SLAVE_REGS_H_

#include "main.h"
#include "types.h"


#define SLAVE_REGS_CRIDT_ENTER() __disable_irq()
#define SLAVE_REGS_CRIDT_EXIT()  __enable_irq()


typedef enum {
	SLAVE_REGS_POLL_FUNC_REQ_INIT = 0,
	SLAVE_REGS_POLL_FUNC_REQ_POLL,
	SLAVE_REGS_POLL_FUNC_REQ_WRITE,
	SLAVE_REGS_POLL_FUNC_REQ_IRQ_WRITE,
} slave_regs_poll_func_req_t ;

typedef uint32_t slave_reg_data_t;
typedef uint32_t slave_reg_addr_t;

typedef struct {
	slave_reg_data_t						value;
	uint16_t								ctrl;
} slave_reg_buf_t;



typedef uint16_t (*slave_reg_write_func)(slave_reg_data_t new_reg_val);


void slave_registers_init_value(volatile slave_reg_buf_t *reg, const uint16_t reg_value,
									BOOL write_priv, BOOL change_not_check, BOOL write_by_irq);

void slave_registers_write(volatile slave_reg_buf_t *reg, slave_reg_write_func fn);


void slave_irq_write_reg(slave_reg_addr_t reg_addr, slave_reg_data_t data);

void slave_irq_read_reg(slave_reg_addr_t reg_addr, slave_reg_data_t *data);

void slave_set_regs_data(volatile slave_reg_buf_t *reg_buffer, uint16_t reg_addr, slave_reg_data_t *data, uint16_t data_length);

void slave_get_regs_data(volatile slave_reg_buf_t *reg_buffer, uint16_t reg_addr, slave_reg_data_t *data, uint16_t data_length);



BOOL slave_registers_poll(uint32_t _1ms_tick_ctr);

bts_rc slave_registers_init(void);

#endif /* __SLAVE_REGS_H_ */
