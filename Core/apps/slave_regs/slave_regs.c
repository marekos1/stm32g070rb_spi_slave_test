/*
 * slave_regs.c
 *
 *  Created on: 05.05.2024
 *      Author: marek
 */



#include "main_slave_regs.h"

#if CONFIG_APPS_DIGITAL_OUT_ENABLE
#include "digital_out/digital_out_slave_regs.h"
#endif /* CONFIG_APPS_DIGITAL_OUT_ENABLE */

#if (CONFIG_SLAVE_REGS_SPI_T200_SPI_SLAVE == CONFIG_SLAVE_REGS_BY_SPI_T200_SPI_SLAVE)
#include "slave_regs/msz_t200_spi_slave/msz_t200_spi_slave.h"
#endif /* (CONFIG_SLAVE_REGS_SPI_T200_SPI_SLAVE == CONFIG_SLAVE_REGS_BY_SPI_T200_SPI_SLAVE) */
#include "slave_regs.h"

#include "main.h"

#include <stddef.h>
#include <string.h>





/*
#include "main_slave_regs.h"
#include "slave_firmware_upgrade.h"
#include "gps_slave.h"
#include "dpll_slave.h"
*/



typedef struct {
	bool									write;
} slave_register_group_data_t;

typedef struct {
	uint16_t								base_addr;
	uint16_t								reg_buffer_length;
	volatile slave_reg_buf_t				*reg_buffer;
    bool (*req_function)(slave_regs_poll_func_req_t req, uint16_t reg_addr, slave_reg_data_t data);
    slave_register_group_data_t			*data;
} slave_register_group_entry_t;



typedef enum {
	I2C_SLAVE_REGISTER_GROUP_MAIN = 0,
#if CONFIG_APPS_DIGITAL_OUT_ENABLE
	I2C_SLAVE_REGISTER_GROUP_DIGITAL_OUT,
#endif /* CONFIG_APPS_DIGITAL_OUT_ENABLE */

	I2C_SLAVE_REGISTER_GROUP_NUMBER
} slave_register_group_t;

static slave_register_group_data_t slave_register_group_data[I2C_SLAVE_REGISTER_GROUP_NUMBER];


static const slave_register_group_entry_t slave_register_group[I2C_SLAVE_REGISTER_GROUP_NUMBER] = {
	[I2C_SLAVE_REGISTER_GROUP_MAIN] 			= {	.base_addr 			= MAIN_SLAVE_REG_BASE_ADDR,						/* 0x0000  [    0] */
													.reg_buffer_length 	= MAIN_SLAVE_REG_NR_MAX,						/* 0x000A       11 */
													.reg_buffer 		= slave_main_group_regs,
													.req_function 		= main_group_slave_regs_req,
													.data				= &slave_register_group_data[I2C_SLAVE_REGISTER_GROUP_MAIN]},
#if CONFIG_APPS_DIGITAL_OUT_ENABLE
	[I2C_SLAVE_REGISTER_GROUP_DIGITAL_OUT] 		= {	.base_addr 			= DIGITAL_OUT_SLAVE_REG_BASE_ADDR,				/* 0x000E  [    14] */
													.reg_buffer_length 	= DIGITAL_OUT_SLAVE_REG_NR_MAX,					/* 0x0001        1  */
													.reg_buffer 		= digital_out_group_regs,
													.req_function 		= digital_out_group_slave_regs_req,
													.data				= &slave_register_group_data[I2C_SLAVE_REGISTER_GROUP_DIGITAL_OUT]},
#endif /* CONFIG_APPS_DIGITAL_OUT_ENABLE */
};



static volatile bool slave_connect = false;
static uint32_t slave_reg_write_100ms_ctr;
static uint32_t slave_reg_poll_1000ms_ctr;

#define I2C_SLAVE_REG_BUFFER_CTRL_WR_PRIV_BIT 		0
#define I2C_SLAVE_REG_BUFFER_CTRL_WR_NOT_CHECK_BIT 	1
#define I2C_SLAVE_REG_BUFFER_CTRL_WR_BY_IRQ_BIT 	2
#define I2C_SLAVE_REG_BUFFER_CTRL_WR_EVENT_BIT 		3
#define I2C_SLAVE_REG_BUFFER_CTRL_CHANGE_EVENT_BIT 	4

static void slave_regs_conf_bit_in_ctrl(volatile uint16_t *ctrl, uint8_t bit, bool set) {

	*ctrl &= ~(1 << bit);
	if (set) {
		*ctrl |= (1 << bit);
	}
}

static void slave_regs_set_write_privileges(volatile slave_reg_buf_t *reg_buffer,
												bool write_privileges, bool change_not_check, bool write_by_irq) {

	slave_regs_conf_bit_in_ctrl(&reg_buffer->ctrl, I2C_SLAVE_REG_BUFFER_CTRL_WR_PRIV_BIT, write_privileges);
	slave_regs_conf_bit_in_ctrl(&reg_buffer->ctrl, I2C_SLAVE_REG_BUFFER_CTRL_WR_NOT_CHECK_BIT, change_not_check);
	slave_regs_conf_bit_in_ctrl(&reg_buffer->ctrl, I2C_SLAVE_REG_BUFFER_CTRL_WR_BY_IRQ_BIT, write_by_irq);
}

static bool slave_regs_get_write_privileges(volatile slave_reg_buf_t *reg_buffer) {

	bool									ret_val;

	ret_val = (bool)(reg_buffer->ctrl & (1 << I2C_SLAVE_REG_BUFFER_CTRL_WR_PRIV_BIT));

	return ret_val;
}

static bool slave_regs_get_write_by_irq_privileges(volatile slave_reg_buf_t *reg_buffer) {

	bool									ret_val;

	ret_val = (bool)(reg_buffer->ctrl & (1 << I2C_SLAVE_REG_BUFFER_CTRL_WR_BY_IRQ_BIT));

	return ret_val;
}

static bool slave_regs_get_write_not_check(volatile slave_reg_buf_t *reg_buffer) {

	bool									ret_val;

	ret_val = (bool)(reg_buffer->ctrl & (1 << I2C_SLAVE_REG_BUFFER_CTRL_WR_NOT_CHECK_BIT));

	return ret_val;
}

static void slave_regs_set_write_event(volatile slave_reg_buf_t *reg_buffer, bool write_event, bool change_value) {

	reg_buffer->ctrl &= ~((1 << I2C_SLAVE_REG_BUFFER_CTRL_WR_EVENT_BIT) | (1 << I2C_SLAVE_REG_BUFFER_CTRL_CHANGE_EVENT_BIT));
	if (write_event) {
		reg_buffer->ctrl |= (1 << I2C_SLAVE_REG_BUFFER_CTRL_WR_EVENT_BIT);
	}
	if (change_value) {
		reg_buffer->ctrl |= (1 << I2C_SLAVE_REG_BUFFER_CTRL_CHANGE_EVENT_BIT);
	}
}

static bool slave_regs_get_write_event(volatile slave_reg_buf_t *reg_buffer) {

	bool									ret_val;

	ret_val = (bool)(reg_buffer->ctrl & (1 << I2C_SLAVE_REG_BUFFER_CTRL_WR_EVENT_BIT));

	return ret_val;
}

static bool slave_regs_get_change_value_event(volatile slave_reg_buf_t *reg_buffer) {

	bool									ret_val;

	ret_val = (bool)(reg_buffer->ctrl & (1 << I2C_SLAVE_REG_BUFFER_CTRL_CHANGE_EVENT_BIT));

	return ret_val;
}

void slave_registers_init_value(volatile slave_reg_buf_t *reg, const uint16_t reg_value,
									bool write_priv, bool change_not_check, bool write_by_irq) {

	reg->value = reg_value;
	slave_regs_set_write_privileges(reg, write_priv, change_not_check, write_by_irq);
}

void slave_registers_write(volatile slave_reg_buf_t *reg, slave_reg_write_func fn) {

	slave_reg_buf_t							reg_data;

	SLAVE_REGS_CRIDT_ENTER();
	reg_data = *reg;
	SLAVE_REGS_CRIDT_EXIT();
	if (fn) {
		reg_data.value = fn(reg_data.value);
	}
	SLAVE_REGS_CRIDT_ENTER();
	*reg = reg_data;
	SLAVE_REGS_CRIDT_EXIT();
}

void slave_registers_address_write(volatile slave_reg_buf_t *reg, const slave_reg_addr_t addr, slave_reg_addr_write_func fn_addr) {

	slave_reg_buf_t							reg_data;

	SLAVE_REGS_CRIDT_ENTER();
	reg_data = *reg;
	SLAVE_REGS_CRIDT_EXIT();
	if (fn_addr) {
		reg_data.value = fn_addr(addr, reg_data.value);
	}
	SLAVE_REGS_CRIDT_ENTER();
	*reg = reg_data;
	SLAVE_REGS_CRIDT_EXIT();
}

static volatile slave_reg_buf_t* slave_get_reg_buffer(slave_reg_addr_t reg_addr) {

	volatile slave_reg_buf_t				*reg_buffer = NULL;
	uint8_t									reg_grp_no;

	for (reg_grp_no = 0; reg_grp_no < I2C_SLAVE_REGISTER_GROUP_NUMBER; reg_grp_no++) {
		if ((reg_addr >= slave_register_group[reg_grp_no].base_addr) &&
			(reg_addr < (slave_register_group[reg_grp_no].base_addr + slave_register_group[reg_grp_no].reg_buffer_length))) {
			reg_buffer = slave_register_group[reg_grp_no].reg_buffer + (reg_addr - slave_register_group[reg_grp_no].base_addr);
			break;
		}
	}

	return reg_buffer;
}

const slave_register_group_entry_t* slave_get_reg_group(slave_reg_addr_t reg_addr) {

	const slave_register_group_entry_t		*reg_group = NULL;
	uint8_t									reg_grp_no;

	for (reg_grp_no = 0; reg_grp_no < I2C_SLAVE_REGISTER_GROUP_NUMBER; reg_grp_no++) {
		if ((reg_addr >= slave_register_group[reg_grp_no].base_addr) &&
			(reg_addr < (slave_register_group[reg_grp_no].base_addr + slave_register_group[reg_grp_no].reg_buffer_length))) {
			reg_group = &slave_register_group[reg_grp_no];
			break;
		}
	}

	return reg_group;
}

static void slave_write_reg(slave_reg_addr_t reg_addr, slave_reg_data_t data) {

	volatile slave_reg_buf_t				*reg_buffer = NULL;
	const slave_register_group_entry_t		*reg_group = NULL;
	bool									write_priv, write_by_irq, ret_val, change_val;

	reg_buffer = slave_get_reg_buffer(reg_addr);
	if (reg_buffer) {
		write_priv = slave_regs_get_write_privileges(reg_buffer);
		if (write_priv) {
			reg_group = slave_get_reg_group(reg_addr);
			if (reg_group) {
				change_val = false;
				if (data != reg_buffer->value) {
					change_val = true;
					reg_buffer->value = data;
				}
				write_by_irq = slave_regs_get_write_by_irq_privileges(reg_buffer);
				if (write_by_irq) {
					ret_val = reg_group->req_function(SLAVE_REGS_POLL_FUNC_REQ_IRQ_WRITE, reg_addr - reg_group->base_addr, data);
				} else {
					ret_val = false;
				}
				if (ret_val == false) {
					slave_regs_set_write_event(reg_buffer, true, change_val);
					if (reg_group->data) {
						reg_group->data->write = true;
					}
				}
			}
		}
	}
}

static void slave_read_reg(slave_reg_addr_t reg_addr, slave_reg_data_t *data) {

	volatile slave_reg_buf_t			*reg_buffer = NULL;

	reg_buffer = slave_get_reg_buffer(reg_addr);
	if (reg_buffer) {
		*data = reg_buffer->value;
	}
}

void slave_irq_write_reg(slave_reg_addr_t reg_addr, slave_reg_data_t data) {

	slave_write_reg(reg_addr, data);
	slave_connect = true;
}

void slave_irq_read_reg(slave_reg_addr_t reg_addr, slave_reg_data_t *data) {

	slave_read_reg(reg_addr, data);
	slave_connect = true;
}

void slave_set_regs_data(volatile slave_reg_buf_t *reg_buffer, uint16_t reg_addr, slave_reg_data_t *data, uint16_t data_length) {

	uint16_t								data_no;

	for (data_no = 0; data_no < data_length; data_no++) {
		SLAVE_REGS_CRIDT_ENTER();
		(reg_buffer + reg_addr + data_no)->value = *(data + data_no);
		SLAVE_REGS_CRIDT_EXIT();
	}
}

void slave_get_regs_data(volatile slave_reg_buf_t *reg_buffer, uint16_t reg_addr, slave_reg_data_t *data, uint16_t data_length) {

	uint16_t								data_no;

	for (data_no = 0; data_no < data_length; data_no++) {
		SLAVE_REGS_CRIDT_ENTER();
		*(data + data_no) = (reg_buffer + reg_addr + data_no)->value;
		SLAVE_REGS_CRIDT_EXIT();
	}
}

static void slave_poll_write_function(void) {

	uint8_t									reg_grp_no;
	uint16_t								reg_no;
	volatile slave_reg_buf_t				*reg_buffer = NULL;
	bool									was_write;

	for (reg_grp_no = 0; reg_grp_no < I2C_SLAVE_REGISTER_GROUP_NUMBER; reg_grp_no++) {
		SLAVE_REGS_CRIDT_ENTER();
		was_write = slave_register_group[reg_grp_no].data->write;
		SLAVE_REGS_CRIDT_EXIT();
		if (was_write) {
			for (reg_no = 0; reg_no < slave_register_group[reg_grp_no].reg_buffer_length; reg_no++) {
				reg_buffer = slave_register_group[reg_grp_no].reg_buffer + reg_no;
				if (slave_regs_get_write_privileges(reg_buffer)) {
					SLAVE_REGS_CRIDT_ENTER();
					if (slave_regs_get_write_event(reg_buffer)) {
						if (slave_regs_get_write_not_check(reg_buffer) || slave_regs_get_change_value_event(reg_buffer)) {
							if (slave_register_group[reg_grp_no].req_function) {
								SLAVE_REGS_CRIDT_EXIT();
								slave_register_group[reg_grp_no].req_function(SLAVE_REGS_POLL_FUNC_REQ_WRITE, reg_no, reg_buffer->value);
								SLAVE_REGS_CRIDT_ENTER();
							}
						}
						slave_regs_set_write_event(reg_buffer, false, false);
					}
					SLAVE_REGS_CRIDT_EXIT();
				}
			}
		}
	}
}

static void slave_poll_function(slave_regs_poll_func_req_t req) {

	uint8_t									reg_grp_no;

	for (reg_grp_no = 0; reg_grp_no < I2C_SLAVE_REGISTER_GROUP_NUMBER; reg_grp_no++) {
		if (slave_register_group[reg_grp_no].req_function) {
			slave_register_group[reg_grp_no].req_function(req, 0, 0);
		}
	}
}
#if 0
#define I2C_SLAVE_REGISTER_WRITE_TIME_MS					100
#define I2C_SLAVE_REGISTER_POLL_TIME_MS						1000
#else
#define I2C_SLAVE_REGISTER_WRITE_TIME_MS					0
#define I2C_SLAVE_REGISTER_POLL_TIME_MS						0
#endif

bool slave_registers_poll(uint32_t _1ms_tick_ctr) {

	bool									wr_operation;

	wr_operation = msz_t200_spi_slave_poll();
	if (wr_operation || (_1ms_tick_ctr - I2C_SLAVE_REGISTER_WRITE_TIME_MS) >= slave_reg_write_100ms_ctr) {
		slave_reg_write_100ms_ctr = _1ms_tick_ctr;
		slave_poll_write_function();
	}

	if ((_1ms_tick_ctr - I2C_SLAVE_REGISTER_POLL_TIME_MS) >= slave_reg_poll_1000ms_ctr) {
		slave_reg_poll_1000ms_ctr = _1ms_tick_ctr;
		slave_poll_function(SLAVE_REGS_POLL_FUNC_REQ_POLL);
	}

	return slave_connect;
}

msz_rc_t slave_registers_init(void) {

	msz_rc_t								rc = MSZ_RC_OK;

	slave_poll_function(SLAVE_REGS_POLL_FUNC_REQ_INIT);

#if (CONFIG_SLAVE_REGS_SPI_T200_SPI_SLAVE == CONFIG_SLAVE_REGS_BY_SPI_T200_SPI_SLAVE)
	rc = msz_t200_spi_slave_init();
#endif /* (CONFIG_SLAVE_REGS_SPI_T200_SPI_SLAVE == CONFIG_SLAVE_REGS_BY_SPI_T200_SPI_SLAVE) */

	return rc;
}
