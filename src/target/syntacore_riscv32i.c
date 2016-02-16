#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target/target.h"
#include "target/target_type.h"
#include "target/breakpoints.h"
#include "target/register.h"
#include "helper/log.h"
#include "helper/binarybuffer.h"
#include "helper/types.h"
#include "jtag/jtag.h"

#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include <memory.h>

#define LOCAL_CONCAT(x,y) x##y
#define STATIC_ASSERT(e) typedef char[1 - 2*!(e)] LOCAL_CONCAT(static_assert,__LINE__)
#define ARRAY_LEN(arr) (sizeof (arr) / sizeof (arr)[0])
#define LOG_METHOD_ENTER() LOG_DEBUG("Enter\n")
enum TAP_IR
{
	TAP_INSTR_DBG_ID = 3,
	TAP_INSTR_BLD_ID = 4,
	TAP_INSTR_DBG_STATUS = 5,
	TAP_INSTR_DAP_CTRL = 6,
	TAP_INSTR_DAP_CTRL_RD = 7,
	TAP_INSTR_DAP_CMD = 8,
	TAP_INSTR_IDCODE = 0xE,
	TAP_INSTR_BYPASS = 0xF,
};

enum TAP_DR_LEN
{
	TAP_LEN_IDCODE = 32,
	TAP_LEN_DBG_ID = 32,
	TAP_LEN_BLD_ID = 32,
	TAP_LEN_DBG_STATUS = 32,
	TAP_LEN_DAP_CTRL = 4,
	TAP_LEN_DAP_CMD = 4 + 32,
	TAP_LEN_BYPASS = 1,
};
enum type_dbgc_unit_id_e
{
	DBGC_UNIT_ID_HART_0 = 0,
	DBGC_UNIT_ID_HART_1 = 1,
	DBGC_UNIT_ID_CORE = 3,
};

enum DBGC_FGRP
{
	DBGC_FGRP_HART_REGTRANS = 0,
	DBGC_FGRP_HART_DBGCMD = 1,
	DBGC_FGRP_CORE_REGTRANS = 0,
};
static int sg_error_code = ERROR_OK;
static int
get_error_code(void)
{
	return sg_error_code;
}

static int
update_error_code(int const a_error_code)
{
	if ( ERROR_OK == get_error_code() && ERROR_OK != a_error_code ) {
		sg_error_code = a_error_code;
	}
	return get_error_code();
}

static int
clear_error_code(void)
{
	int const result = get_error_code();
	sg_error_code = ERROR_OK;
	return result;
}

static void
jtag_select_IR(struct target* const restrict p_target, enum TAP_IR const new_instr)
{
	LOG_METHOD_ENTER();
	assert(p_target && p_target->tap);

	struct jtag_tap* restrict tap = p_target->tap;
	if ( buf_get_u32(tap->cur_instr, 0u, tap->ir_length) != new_instr ) {
		assert(tap->ir_length == 4);
		uint8_t const tmp = new_instr;
		struct scan_field field =
		{
			.num_bits = tap->ir_length,
			.out_value = &tmp,
		};
		jtag_add_ir_scan(tap, &field, TAP_IDLE);
		jtag_execute_queue_noclear();
	}
}

enum type_dbgc_core_dbg_sts_reg_bits_e
{
	DBGC_CORE_CDSR_HART0_DMODE_BIT = 0,
	DBGC_CORE_CDSR_HART0_RST_BIT = 1,
	DBGC_CORE_CDSR_HART0_ERR_BIT = 3,
	DBGC_CORE_CDSR_LOCK_BIT = 30,
	DBGC_CORE_CDSR_READY_BIT = 31,
};

static uint32_t
DBG_STATUS_get(struct target* p_target)
{
	LOG_METHOD_ENTER();
	assert(p_target);
	assert(p_target->tap);
	jtag_select_IR(p_target, TAP_INSTR_DBG_STATUS);
	uint32_t result = 0;
	struct scan_field field =
	{
		.num_bits = TAP_LEN_DBG_STATUS,
		.in_value = (uint8_t *)&result
	};
	jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);
	update_error_code(jtag_execute_queue());
	if ( get_error_code() != ERROR_OK ) {
		LOG_ERROR("JTAG error %d\n", get_error_code());
	}
	if ( (result & ((1u << DBGC_CORE_CDSR_READY_BIT) | (1u << DBGC_CORE_CDSR_LOCK_BIT))) != (1u << DBGC_CORE_CDSR_READY_BIT) ) {
		LOG_WARNING("TAP_INSTR_DBG_STATUS is %x!\n", result);
	}
	LOG_DEBUG("return %x\n", result);
	return result;
}

static void
DAP_CTRL_REG_set(struct target* const p_target, enum type_dbgc_unit_id_e const dap_unit, enum DBGC_FGRP const dap_group)
{
	LOG_METHOD_ENTER();
	assert(p_target);
	assert(
		(
		(
		(dap_unit == DBGC_UNIT_ID_HART_0 && 0 == p_target->coreid) ||
		(dap_unit == DBGC_UNIT_ID_HART_1 && 1 == p_target->coreid)
		) && (dap_group == DBGC_FGRP_HART_REGTRANS || dap_group == DBGC_FGRP_HART_DBGCMD)
		) ||
		(dap_unit == DBGC_UNIT_ID_CORE && dap_group == DBGC_FGRP_HART_REGTRANS)
		);
	jtag_select_IR(p_target, TAP_INSTR_DAP_CTRL);
	uint8_t const tmp_dap_unit = dap_unit;
	uint8_t const tmp_dap_group = dap_group;
	struct scan_field const fields[2] =
	{
		[1] =
		{
			.num_bits = 2,
			.out_value = &tmp_dap_unit
		},
		[0] = {
				.num_bits = 2,
				.out_value = &tmp_dap_group
			},
	};
	jtag_add_dr_scan(p_target->tap, ARRAY_LEN(fields), fields, TAP_IDLE);
	jtag_execute_queue_noclear();
}

static int
HART_DBG_STATUS_get(struct target* p_target)
{
	LOG_METHOD_ENTER();
	assert(p_target);
	assert(0 <= p_target->coreid && p_target->coreid < 2);
	uint8_t const status = (DBG_STATUS_get(p_target) >> p_target->coreid) & 0xFFu;
	LOG_DEBUG("status is %x\n", (uint32_t)status);
	if ( status & (1 << DBGC_CORE_CDSR_HART0_ERR_BIT) ) {
		LOG_DEBUG("HART_DBG_STATUS = %s\n", "TARGET_UNKNOWN");
		return TARGET_UNKNOWN;
	} else if ( status & (1 << DBGC_CORE_CDSR_HART0_RST_BIT) ) {
		LOG_DEBUG("HART_DBG_STATUS = %s\n", "TARGET_RESET");
		return TARGET_RESET;
	} else if ( status & (1 << DBGC_CORE_CDSR_HART0_DMODE_BIT) ) {
		LOG_DEBUG("HART_DBG_STATUS = %s\n", "TARGET_HALTED");
		return TARGET_HALTED;
	} else {
		LOG_DEBUG("HART_DBG_STATUS = %s\n", "TARGET_RUNNING");
		return TARGET_RUNNING;
	}
}

struct arch
{
	struct reg_cache *m_reg_cache;
	/// @todo reasons
	enum target_debug_reason nc_poll_requested;
};

static char const* const general_regs_names_list[] =
{
	"x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7",
	"x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15",
	"x16", "x17", "x18", "x19", "x20", "x21", "x22", "x23",
	"x24", "x25", "x26", "x27", "x28", "x29", "x30", "x31",
	"pc",
};

static int
check_core_reg(struct reg *p_reg)
{
	LOG_METHOD_ENTER();
	assert(p_reg);
	if ( p_reg->number > 32 ) {
		LOG_WARNING("Bad reg id =%d for register %s", p_reg->number, p_reg->name);
		return ERROR_FAIL;
	}
	assert(p_reg->arch_info);
	struct target* p_target = p_reg->arch_info;
	if ( p_target->state != TARGET_HALTED ) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	return ERROR_OK;
}

static uint32_t
read_core_register(unsigned reg_no)
{
	return 0;
}

static int
get_core_reg(struct reg *p_reg)
{
	LOG_METHOD_ENTER();
	int const check_status = check_core_reg(p_reg);
	if ( check_status != ERROR_OK ) {
		return check_status;
	}
	uint32_t const value = read_core_register(p_reg->number);
	buf_set_u32(p_reg->value, 0, 32, value);
	p_reg->dirty = true;
	p_reg->valid = true;

	return ERROR_OK;
}

static int
set_core_reg(struct reg *p_reg, uint8_t *buf)
{
	LOG_METHOD_ENTER();
	int const check_status = check_core_reg(p_reg);
	if ( check_status != ERROR_OK ) {
		return check_status;
	}
	uint32_t const value = buf_get_u32(buf, 0, 32);
	LOG_DEBUG("Updating cache for register %s <-- %08x", p_reg->name, value);

	buf_set_u32(p_reg->value, 0, 32, value);
	p_reg->dirty = true;
	p_reg->valid = true;

	return ERROR_OK;
}

static char const* const FP_regs_names_list[] =
{
	"f0", "f1", "f2", "f3", "f4", "f5", "f6", "f7",
	"f8", "f9", "f10", "f11", "f12", "f13", "f14", "f15",
	"f16", "f17", "f18", "f19", "f20", "f21", "f22", "f23",
	"f24", "f25", "f26", "f27", "f28", "f29", "f30", "f31",
};

/// @todo handlers for x0 and for pc
static struct reg_arch_type const general_reg_access_type =
{
	.get = get_core_reg,
	.set = set_core_reg,
};

static int
set_FPU_reg(struct reg *reg, uint8_t *buf)
{
	LOG_METHOD_ENTER();
	LOG_WARNING("get_core_reg IS NOT IMPLEMENTED");
	return ERROR_OK;
}
static int
get_FPU_reg(struct reg *reg)
{
	LOG_METHOD_ENTER();
	LOG_WARNING("get_core_reg IS NOT IMPLEMENTED");
	return ERROR_OK;
}

static struct reg_arch_type const FP_reg_access_type =
{
	.get = get_FPU_reg,
	.set = set_FPU_reg,
};

static struct reg
general_purpose_reg_construct(char const* const p_name, uint32_t const number, struct target* p_target)
{
	LOG_METHOD_ENTER();
	struct reg const the_reg = {
		.name = p_name,
		.number = number,
		.feature = NULL,
		.caller_save = false,
		.value = calloc(1, sizeof(uint32_t)),
		.dirty = false,
		.valid = false,
		.exist = true,
		.size = 32,  //< XLEN?
		.reg_data_type = NULL,
		.group = NULL,
		.arch_info = p_target,
		.type = &general_reg_access_type,
	};
	return the_reg;
}

static struct reg
FP_reg_construct(char const* const p_name, uint32_t const number, struct target* p_target)
{
	LOG_METHOD_ENTER();
	struct reg const the_reg = {
		.name = p_name,
		.number = number,
		.feature = NULL,
		.caller_save = false,
		.value = calloc(1, sizeof(uint64_t)),
		.dirty = false,
		.valid = false,
		.exist = true,
		.size = 64,  //< number of bits of double?
		.reg_data_type = NULL,
		.group = NULL,
		.arch_info = p_target,
		.type = &FP_reg_access_type,
	};
	return the_reg;
}

static struct reg_cache*
reg_cache__create(struct target * p_target)
{
	LOG_METHOD_ENTER();
	struct reg_cache* const p_obj = calloc(1, sizeof(struct reg_cache));
	assert(p_obj);
	static size_t const number_of_general_regs = ARRAY_LEN(general_regs_names_list);
	static size_t const number_of_FP_regs = ARRAY_LEN(FP_regs_names_list);
	static size_t const num_regs = number_of_general_regs + number_of_FP_regs;
	struct reg* const reg_list = calloc(num_regs, sizeof(struct reg));
	struct reg* p_dest_reg = reg_list;
	{
		/// Create general purpose registers cache
		char const* const* p_name = general_regs_names_list;
		for ( unsigned i = 0; i < number_of_general_regs; ++i ) {
			*p_dest_reg++ = general_purpose_reg_construct(*p_name++, i, p_target);
		}
	}
	{
		/// Create floating point registers cache
		char const* const* p_name = FP_regs_names_list;
		for ( unsigned i = 0; i < number_of_FP_regs; ++i ) {
			*p_dest_reg++ = FP_reg_construct(*p_name++, number_of_general_regs + i, p_target);
		}
	}
	struct reg_cache const the_reg_cache = {
		.name = "syntacore_riscv32 registers",
		.reg_list = reg_list,
		.num_regs = num_regs,
	};
	*p_obj = the_reg_cache;
	return p_obj;
}

static int
this_target_create(struct target *p_target, struct Jim_Interp *interp)
{
	LOG_METHOD_ENTER();
	assert(p_target);

	struct arch the_arch = {
		.nc_poll_requested = DBG_REASON_DBGRQ,
		.m_reg_cache = reg_cache__create(p_target)
	};
	struct arch* p_arch_info = calloc(1, sizeof(struct arch));
	*p_arch_info = the_arch;

	p_target->arch_info = p_arch_info;
	return ERROR_OK;
}

static void
save_context(struct target *p_target)
{
	LOG_METHOD_ENTER();
}

static void
restore_context(struct target *p_target)
{
	LOG_METHOD_ENTER();
}

static int
this_poll(struct target *p_target)
{
	LOG_METHOD_ENTER();
	p_target->state = HART_DBG_STATUS_get(p_target);
	if ( p_target->state == TARGET_RUNNING ) {
		return ERROR_OK;
	}

	struct arch* p_arch = (struct arch*)p_target->arch_info;
	if ( p_arch->nc_poll_requested == DBG_REASON_DBGRQ ) {
		p_target->debug_reason = DBG_REASON_DBGRQ;
		return ERROR_OK;
	}

	if ( p_arch->nc_poll_requested == DBG_REASON_SINGLESTEP ) {
		/// @todo enable interrupts
#if 0
		debug_write_register(p_target, DEBUG_CONTROL_COMMAND, 0x0);
#endif
	}
	p_target->debug_reason = p_arch->nc_poll_requested;
	target_call_event_callbacks(p_target, TARGET_EVENT_HALTED);
	p_arch->nc_poll_requested = DBG_REASON_DBGRQ;
	save_context(p_target);  // update reg cache
	return ERROR_OK;
}

static int
this_arch_state(struct target *target)
{
	LOG_METHOD_ENTER();
	return ERROR_OK;
}

static int
this_init(struct command_context *cmd_ctx, struct target *p_target)
{
	LOG_METHOD_ENTER();
#if 0
	clear_error_code();
	jtag_select_IR(p_target, TAP_INSTR_IDCODE);
	uint32_t result;
	struct scan_field field =
	{
		.num_bits = TAP_LEN_IDCODE,
		.in_value = (uint8_t *)&result
	};
	jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);
	update_error_code(jtag_execute_queue());
	if ( get_error_code() != ERROR_OK ) {
		LOG_ERROR("JTAG error %d in %s ()", get_error_code(), __PRETTY_FUNCTION__);
		return ERROR_OK;
	}
	LOG_DEBUG("IDCODE=%#0x", result);
#endif
	return ERROR_OK;
}

enum DBGC_DAP_OPCODE_DBGCMD
{
	DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL = 0,
	DBGC_DAP_OPCODE_DBGCMD_CORE_EXEC = 1,
	DBGC_DAP_OPCODE_DBGCMD_DBGDATA_WR = 2,
	DBGC_DAP_OPCODE_DBGCMD_UNLOCK = 3,
};
static int
this_halt(struct target *p_target)
{
	LOG_METHOD_ENTER();
	clear_error_code();
	if ( HART_DBG_STATUS_get(p_target) == TARGET_HALTED ) {
		LOG_ERROR("Halt request when RV is already in halted state");
		return ERROR_OK;
	}
	/// @todo issue Halt command
	assert(p_target);
	assert(p_target->tap);
	DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_DBGCMD);
	jtag_select_IR(p_target, TAP_INSTR_DAP_CMD);
	uint32_t const set_dmode_flag = 1u;
	uint8_t const dbg_cmd = DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL;
	struct scan_field const fields[2] =
	{
		[0] = {
			.num_bits = 32,
			.out_value = (uint8_t const*)&set_dmode_flag
		},
		[1] =
			{
				.num_bits = 4,
				.out_value = &dbg_cmd
			},
	};
	jtag_add_dr_scan(p_target->tap, ARRAY_LEN(fields), fields, TAP_IDLE);

	p_target->state = HART_DBG_STATUS_get(p_target);
	if ( p_target->state != TARGET_HALTED ) {
		// issue error if we are still running
		LOG_ERROR("RV is not halted after Halt command");
		return ERROR_FAIL;
	}
	struct arch* p_arch = (struct arch*)p_target->arch_info;
	if ( p_arch->nc_poll_requested ) {
		p_arch->nc_poll_requested = DBG_REASON_DBGRQ;
	}
	p_target->debug_reason = DBG_REASON_DBGRQ;
	return ERROR_OK;
}

static int
this_resume(struct target *target, int current, uint32_t address, int handle_breakpoints, int debug_execution)
{
	LOG_METHOD_ENTER();
	// upload reg values into HW
	restore_context(target);
	/// @todo issue resume command
	struct arch* p_arch = (struct arch*)target->arch_info;
	p_arch->nc_poll_requested = DBG_REASON_BREAKPOINT;
	target->state = TARGET_RUNNING;
	return ERROR_OK;
}

static int
this_step(struct target *target, int current, uint32_t address, int handle_breakpoints)
{
	LOG_METHOD_ENTER();
	// upload reg values into HW
	restore_context(target);
#if 0
	debug_write_register(target, DEBUG_CONTROL_STATUS, DEBUG_CONTROL_STATUS_IRQ_DISABLE);
	debug_write_register(target, DEBUG_CONTROL_COMMAND, DEBUG_CONTROL_COMMAND_STEP);
#endif
	struct arch* p_arch = (struct arch*)target->arch_info;
	p_arch->nc_poll_requested = DBG_REASON_SINGLESTEP;
	target->state = TARGET_RUNNING;
	return ERROR_OK;
}

static int
this_assert_reset(struct target *p_target)
{
	LOG_METHOD_ENTER();
#if 0
	int retval = 0;
	if ( (retval = debug_write_register(target, DEBUG_CONTROL_PWR_RST, DEBUG_CONTROL_PWR_RST_HRESET)) != ERROR_OK ) {
		return retval;
	}
#endif
	return ERROR_OK;
}

static int
this_deassert_reset(struct target *target)
{
	LOG_METHOD_ENTER();
#if 0
	int retval = 0;
	if ( (retval = debug_write_register(target, DEBUG_CONTROL_PWR_RST, 0)) != ERROR_OK ) {
		return retval;
	}
#endif
	return ERROR_OK;
}

static int
this_soft_reset_halt(struct target *target)
{
	LOG_METHOD_ENTER();
#if 0
	int retval;
	// assert reset
	if ( (retval = debug_write_register(target, DEBUG_CONTROL_PWR_RST, DEBUG_CONTROL_PWR_RST_HRESET)) != ERROR_OK ) {
		return retval;
	}
	// ...and deassert reset
	return debug_write_register(target, DEBUG_CONTROL_PWR_RST, 0);
#endif
	return ERROR_OK;
}

/// gdb_server expects valid reg values and will use set method for updating reg values
static int
read_mem_word(struct target * target, uint32_t const address, uint32_t* const data)
{
	LOG_METHOD_ENTER();
#if 0
	int retval = 0;
	if ( (retval = debug_write_register(target, DEBUG_MEMORY_ACCESS_ADDRESS, address)) != ERROR_OK ) {
		return retval;
}
	if ( (retval = debug_write_register(target, DEBUG_MEMORY_ACCESS_CMD, DEBUG_MEMORY_ACCESS_CMD_START)) != ERROR_OK ) {
		return retval;
	}
	if ( (retval = debug_read_register(target, DEBUG_MEMORY_ACCESS_RD_DATA, data)) != ERROR_OK ) {
		return retval;
	}
#endif
	LOG_DEBUG("MR A %08X D %08X", address, *data);
	return ERROR_OK;
}

static int
this_read_memory(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
	LOG_METHOD_ENTER();
	LOG_DEBUG("read_memory at %08X, %d bytes", address, size * count);
	unsigned i = 0;  // byte count
	uint32_t x = 0;  // buffer
	int retval = 0;
	unsigned const buffer_size = count * size;
	// Address is not aligned
	if ( address & 0x3 ) {
		if ( (retval = read_mem_word(target, address & (~0x3), &x)) != ERROR_OK ) {
			return retval;
		}
		while ( (address + i) & 0x3 && i != buffer_size ) {
			*(buffer + i) = ((uint8_t*)(&x))[(address + i) & 0x3];
			++i;
		}
	}
	for ( ; i + 4 <= buffer_size; i += 4 ) {
		if ( (retval = read_mem_word(target, address + i, (uint32_t*)(buffer + i))) != ERROR_OK ) {
			return retval;
		}
	}
	if ( buffer_size == i ) {
		return ERROR_OK;
	}
	if ( (retval = read_mem_word(target, address + i, &x)) != ERROR_OK ) {
		return retval;
	}
	unsigned const word_start_offset = i;
	for ( ; i != buffer_size; ++i ) {
		*(buffer + i) = ((uint8_t*)(&x))[(i - word_start_offset) & 0x3];
	}
	return ERROR_OK;
}

static int
this_examine(struct target *target)
{
	LOG_METHOD_ENTER();
	// initialize register values
#if 0
	debug_write_register(target, DEBUG_CONTROL_PWR_RST, DEBUG_CONTROL_PWR_RST_HRESET);
	debug_write_register(target, DEBUG_CONTROL_BREAK, DEBUG_CONTROL_RSTOFBE);
	debug_write_register(target, DEBUG_CONTROL_HALT, DEBUG_CONTROL_HALT_INT3);
	debug_write_register(target, DEBUG_CONTROL_PWR_RST, 0);
#endif
	target->state = TARGET_HALTED;
	save_context(target);
	target_set_examined(target);
	return ERROR_OK;
}

#define RV_SCALL() (0x00100073u)

static int
this_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	LOG_METHOD_ENTER();
	int retval = 0;
	if ( breakpoint->length != 4 || breakpoint->address & 0x3u || breakpoint->type == BKPT_HARD ) {
		return ERROR_FAIL;
	}
	if ( (retval = target_read_buffer(target, breakpoint->address, breakpoint->length, breakpoint->orig_instr)) != ERROR_OK ) {
		return retval;
	}
	if ( (retval = target_write_u32(target, breakpoint->address, RV_SCALL())) != ERROR_OK ) {
		return retval;
	}
	breakpoint->set = 1;

	return ERROR_OK;
}

static int
this_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	LOG_METHOD_ENTER();
	if ( breakpoint->length != 4 || breakpoint->type == BKPT_HARD ) {
		return ERROR_FAIL;
	}
	return target_write_buffer(target, breakpoint->address, breakpoint->length, breakpoint->orig_instr);
}

static int
this_write_memory(struct target *target, uint32_t address, uint32_t size, uint32_t count, const uint8_t *buffer)
{
	LOG_METHOD_ENTER();
	LOG_DEBUG("write_memory at %08X, %d bytes", address, size * count);
#if 0
	unsigned i = 0;  // byte count
	uint32_t x = 0;  // buffer
	int retval = 0;
	unsigned word_start_offset = 0;
	unsigned const buffer_size = count * size;
	// Address is not aligned
	if ( address & 0x3 ) {
		if ( (retval = read_mem_word(target, address & (~0x3), &x)) != ERROR_OK ) {
			return retval;
		}
		while ( (address + i) & 0x3 && i != buffer_size ) {
			((uint8_t*)(&x))[(address + i) & 0x3] = *(buffer + i);
			++i;
		}
		if ( (retval = write_mem_word(target, address & (~0x3), x)) != ERROR_OK ) {
			return retval;
		}
	}
	uint32_t const chunk_write_size = (buffer_size - i) & ~0x3u;
	if ( chunk_write_size < 8 ) {
		for ( ; i + 4 <= buffer_size; i += 4 ) {
			if ( (retval = write_mem_word(target, address + i, *(uint32_t*)(buffer + i))) != ERROR_OK ) {
				return retval;
			}
		}
	} else {
		write_mem_chunk(target, address + i, (uint32_t*)(buffer + i), chunk_write_size / 4);
		i += chunk_write_size;
	}

	if ( buffer_size == i ) {
		return ERROR_OK;
	}
	if ( (retval = read_mem_word(target, address + i, &x)) != ERROR_OK ) {
		return retval;
}
	word_start_offset = i;
	for ( ; i != buffer_size; ++i ) {
		((uint8_t*)&x)[(i - word_start_offset) & 0x3] = buffer[i];
	}

	return write_mem_word(target, (address + i) & ~0x3, x);
#endif
	return ERROR_OK;
}

static int
this_get_gdb_reg_list(struct target *target, struct reg **reg_list[], int *reg_list_size, enum target_register_class reg_class)
{
	LOG_METHOD_ENTER();
	struct arch *arch_info = (struct arch *)target->arch_info;

	size_t const num_regs = ARRAY_LEN(general_regs_names_list) + (reg_class == REG_CLASS_ALL ? ARRAY_LEN(FP_regs_names_list) : 0);
	struct reg** p_reg_array = calloc(num_regs, sizeof(struct reg*));
	struct reg *a_reg_list = arch_info->m_reg_cache->reg_list;
	// memccpy(p_reg_array, arch_info->core_cache->reg_list, num_regs * sizeof(struct reg*));
	for ( size_t i = 0; i < num_regs; ++i ) {
		p_reg_array[i] = &a_reg_list[i];
	}
	*reg_list_size = num_regs;
	*reg_list = p_reg_array;
	return ERROR_OK;
}

struct target_type syntacore_riscv32i_target =
{
	.name = "syntacore_riscv32i",

	.poll = this_poll,
	.arch_state = this_arch_state,

	.halt = this_halt,
	.resume = this_resume,
	.step = this_step,

	.assert_reset = this_assert_reset,
	.deassert_reset = this_deassert_reset,
	.soft_reset_halt = this_soft_reset_halt,

	.get_gdb_reg_list = this_get_gdb_reg_list,

	.read_memory = this_read_memory,
	.write_memory = this_write_memory,
	.add_breakpoint = this_add_breakpoint,
	.remove_breakpoint = this_remove_breakpoint,
	.target_create = this_target_create,
	.init_target = this_init,
	.examine = this_examine,
};
