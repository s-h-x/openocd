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
#define BIT_NUM_TO_MASK(bit_num) (1 << (bit_num))
#define LOW_BITS_MASK(n) (~(~0 << (n)))
#define RV_SCALL() (0x00100073u)

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

enum type_dbgc_core_dbg_sts_reg_bits_e
{
	DBGC_CORE_CDSR_HART0_DMODE_BIT = 0,
	DBGC_CORE_CDSR_HART0_RST_BIT = 1,
	DBGC_CORE_CDSR_HART0_ERR_BIT = 3,
	DBGC_CORE_CDSR_LOCK_BIT = 30,
	DBGC_CORE_CDSR_READY_BIT = 31,
};

enum DBGC_DAP_OPCODE_DBGCMD
{
	DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL = 0,
	DBGC_DAP_OPCODE_DBGCMD_CORE_EXEC = 1,
	DBGC_DAP_OPCODE_DBGCMD_DBGDATA_WR = 2,
	DBGC_DAP_OPCODE_DBGCMD_UNLOCK = 3,
};

enum DAP_OPSTATUS_BITS
{
	DAP_OPSTATUS_EXCEPT = 0,
	DAP_OPSTATUS_ERROR = 1,
	DAP_OPSTATUS_LOCK = 2,
	DAP_OPSTATUS_READY = 3,
};
struct This_Arch
{
	struct reg_cache *m_reg_cache;
	/// @todo reasons
	enum target_debug_reason nc_poll_requested;
};

typedef struct reg_arch_type reg_arch_type;
typedef struct target_type target_type;

static int this_poll(struct target *p_target);
static int this_arch_state(struct target *p_target);
static int this_halt(struct target *p_target);
static int this_resume(struct target *p_target, int current, uint32_t address, int handle_breakpoints, int debug_execution);
static int this_step(struct target *p_target, int current, uint32_t address, int handle_breakpoints);
static int this_assert_reset(struct target *p_target);
static int this_deassert_reset(struct target *p_target);
static int this_soft_reset_halt(struct target *p_target);
static int this_get_gdb_reg_list(struct target *p_target, struct reg **reg_list[], int *reg_list_size, enum target_register_class reg_class);
static int this_read_memory(struct target *p_target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);
static int this_write_memory(struct target *p_target, uint32_t address, uint32_t size, uint32_t count, const uint8_t *buffer);
static int this_add_breakpoint(struct target *p_target, struct breakpoint *breakpoint);
static int this_remove_breakpoint(struct target *p_target, struct breakpoint *breakpoint);
static int this_target_create(struct target *p_target, struct Jim_Interp *interp);
static int this_init(struct command_context *cmd_ctx, struct target *p_target);
static int this_examine(struct target *p_target);

static int get_core_reg(struct reg *p_reg);
static int set_core_reg(struct reg *p_reg, uint8_t *buf);
static int get_FPU_reg(struct reg *reg);
static int set_FPU_reg(struct reg *reg, uint8_t *buf);

static int sg_error_code = ERROR_OK;

static char const* const general_regs_names_list[] =
{
	"x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7",
	"x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15",
	"x16", "x17", "x18", "x19", "x20", "x21", "x22", "x23",
	"x24", "x25", "x26", "x27", "x28", "x29", "x30", "x31",
	"pc",
};

static char const* const FP_regs_names_list[] =
{
	"f0", "f1", "f2", "f3", "f4", "f5", "f6", "f7",
	"f8", "f9", "f10", "f11", "f12", "f13", "f14", "f15",
	"f16", "f17", "f18", "f19", "f20", "f21", "f22", "f23",
	"f24", "f25", "f26", "f27", "f28", "f29", "f30", "f31",
};

/// @todo handlers for x0 and for pc
static reg_arch_type const general_reg_access_type =
{
	.get = get_core_reg,
	.set = set_core_reg,
};

static reg_arch_type const FP_reg_access_type =
{
	.get = get_FPU_reg,
	.set = set_FPU_reg,
};

target_type syntacore_riscv32i_target =
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
	.examine = this_examine,

	.init_target = this_init,
};

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
	LOG_DEBUG("Enter");
	int const result = get_error_code();
	sg_error_code = ERROR_OK;
	return result;
}

static void
jtag_select_IR(struct target* const restrict p_target, enum TAP_IR const new_instr)
{
	LOG_DEBUG("Enter");
	assert(p_target && p_target->tap);

	struct jtag_tap* restrict tap = p_target->tap;
#if 0
	if ( buf_get_u32(tap->cur_instr, 0u, tap->ir_length) != new_instr ) {
#endif
		assert(tap->ir_length == 4);
		uint8_t const tmp = new_instr;
		struct scan_field field =
		{
			.num_bits = tap->ir_length,
			.out_value = &tmp,
		};
		jtag_add_ir_scan(tap, &field, TAP_IDLE);
		update_error_code(jtag_execute_queue());
		if ( get_error_code() != ERROR_OK ) {
			LOG_ERROR("IR scan error %d", get_error_code());
		}
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, tmp);
#if 0
	}
#endif
}

static uint32_t
DBG_STATUS_get(struct target* p_target)
{
	LOG_DEBUG("Enter");
	assert(p_target);
	assert(p_target->tap);
	jtag_select_IR(p_target, TAP_INSTR_DBG_STATUS);
	if ( get_error_code() != ERROR_OK ) {
		return 0u;
	}
	uint32_t result = 0;
	typedef struct scan_field scan_field;
	scan_field field =
	{
		.num_bits = TAP_LEN_DBG_STATUS,
		.in_value = (uint8_t *)&result
	};
	jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);

	update_error_code(jtag_execute_queue());
	if ( get_error_code() != ERROR_OK ) {
		LOG_ERROR("JTAG error %d", get_error_code());
	}
	LOG_DEBUG("drscan %s %d 0 --> %#0x", p_target->cmd_name, field.num_bits, result);
	if ( (result & (BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT) | BIT_NUM_TO_MASK(DBGC_CORE_CDSR_LOCK_BIT))) != (uint32_t)BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT) ) {
		LOG_WARNING("TAP_INSTR_DBG_STATUS is %x!", result);
		update_error_code(ERROR_TARGET_FAILURE);
	}
	LOG_DEBUG("return %x", result);
	return result;
}

static void
DAP_CTRL_REG_set(struct target* const p_target, enum type_dbgc_unit_id_e const dap_unit, enum DBGC_FGRP const dap_group)
{
	LOG_DEBUG("Enter");
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
	uint8_t const set_dap_unit_group = ((((uint8_t)dap_unit) << 2) | (((uint8_t)dap_group) & LOW_BITS_MASK(2))) & LOW_BITS_MASK(4);
	{
		/// set unit/group
		jtag_select_IR(p_target, TAP_INSTR_DAP_CTRL);
		if ( get_error_code() != ERROR_OK ) {
			return;
		}
		uint8_t status = 0;
		typedef struct scan_field scan_field;
		scan_field const field =
		{
			.num_bits = 4,
			.out_value = &set_dap_unit_group,
			.in_value = &status,
		};
		jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);
		update_error_code(jtag_execute_queue());
		if ( get_error_code() != ERROR_OK ) {
			LOG_ERROR("JTAG error %d", get_error_code());
		}
		LOG_DEBUG("drscan %s %d %#0x --> %#0x", p_target->cmd_name, field.num_bits, set_dap_unit_group, status);
		if ( (status & BIT_NUM_TO_MASK(DAP_OPSTATUS_READY)) != BIT_NUM_TO_MASK(DAP_OPSTATUS_READY) ) {
			LOG_ERROR("Status %0x", (uint32_t)status);
			update_error_code(ERROR_TARGET_FAILURE);
		}
	}
	{
		/// verify unit/group
		jtag_select_IR(p_target, TAP_INSTR_DAP_CTRL_RD);
		if ( get_error_code() != ERROR_OK ) {
			return;
		}
		uint8_t get_dap_unit_group = 0;
		typedef struct scan_field scan_field;
		scan_field const field =
		{
			.num_bits = 4,
			.in_value = &get_dap_unit_group,
		};
		jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);
		update_error_code(jtag_execute_queue());
		if ( get_error_code() != ERROR_OK ) {
			LOG_ERROR("JTAG error %d", get_error_code());
		}
		LOG_DEBUG("drscan %s %d %#0x --> %#0x", p_target->cmd_name, field.num_bits, 0, get_dap_unit_group);
		if ( get_dap_unit_group != set_dap_unit_group ) {
			LOG_ERROR("Unit/Group verification error: set %#0x, but get %#0x!", set_dap_unit_group, get_dap_unit_group);
		}
	}
}

static int
HART_DBG_STATUS_get(struct target* p_target)
{
	LOG_DEBUG("Enter");
	assert(p_target);
	/// Only 1 HART available
	assert(p_target->coreid == 0);
	uint8_t const status = (DBG_STATUS_get(p_target) >> p_target->coreid) & 0xFFu;
	if ( get_error_code() != ERROR_OK ) {
		return TARGET_UNKNOWN;
	}
	LOG_DEBUG("status is %x", (uint32_t)status);
	if ( status & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_HART0_ERR_BIT) ) {
		LOG_DEBUG("HART_DBG_STATUS == %s", "TARGET_UNKNOWN");
		return TARGET_UNKNOWN;
	} else if ( status & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_HART0_RST_BIT) ) {
		LOG_DEBUG("HART_DBG_STATUS == %s", "TARGET_RESET");
		return TARGET_RESET;
	} else if ( status & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_HART0_DMODE_BIT) ) {
		LOG_DEBUG("HART_DBG_STATUS == %s", "TARGET_HALTED");
		return TARGET_HALTED;
	} else {
		LOG_DEBUG("HART_DBG_STATUS == %s", "TARGET_RUNNING");
		return TARGET_RUNNING;
	}
}

static uint32_t
TAP_INSTR_DAP_CMD_IO(struct target *p_target, uint8_t DAP_OPCODE, uint32_t DAP_OPCODE_EXT)
{
	LOG_DEBUG("Enter");
	jtag_select_IR(p_target, TAP_INSTR_DAP_CMD);
	if ( get_error_code() != ERROR_OK ) {
		return 0;
	}
	uint32_t const dap_opcode_ext = DAP_OPCODE_EXT;
	uint8_t const dap_opcode = DAP_OPCODE;
	uint8_t DAP_OPSTATUS = 0;
	uint32_t DBG_DATA = 0;
	typedef struct scan_field scan_field;
	scan_field const fields[2] =
	{
		[0] = {
			.num_bits = 32,
			.out_value = (uint8_t const*)&dap_opcode_ext,
			.in_value = (uint8_t*)&DBG_DATA,
		},
		[1] =
			{
				.num_bits = 4,
				.out_value = &dap_opcode,
				.in_value = &DAP_OPSTATUS,
			},
	};
	assert(p_target->tap);
	jtag_add_dr_scan(p_target->tap, ARRAY_LEN(fields), fields, TAP_IDLE);
	update_error_code(jtag_execute_queue());
	if ( get_error_code() != ERROR_OK ) {
		LOG_ERROR("JTAG error %d", get_error_code());
	}
	LOG_DEBUG("drscan %s %d %#0x %d %#0x --> %#0x %#0x", p_target->cmd_name, fields[0].num_bits, dap_opcode_ext, fields[1].num_bits, dap_opcode, DBG_DATA, DAP_OPSTATUS);
	if ( (DAP_OPSTATUS & BIT_NUM_TO_MASK(DAP_OPSTATUS_READY)) != BIT_NUM_TO_MASK(DAP_OPSTATUS_READY) ) {
		LOG_ERROR("DAP_OPSTATUS == %#0x", (uint32_t)DAP_OPSTATUS);
		update_error_code(ERROR_TARGET_FAILURE);
	}
	return DBG_DATA;
}

static int
check_core_reg(struct reg *p_reg)
{
	LOG_DEBUG("Enter");
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
	LOG_DEBUG("Enter");
	LOG_ERROR("Unimplemented");
	return 0;
}

static int
get_core_reg(struct reg *p_reg)
{
	LOG_DEBUG("Enter");
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
	LOG_DEBUG("Enter");
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

static int
set_FPU_reg(struct reg *reg, uint8_t *buf)
{
	LOG_DEBUG("Enter");
	LOG_ERROR("NOT IMPLEMENTED");
	return ERROR_OK;
}
static int
get_FPU_reg(struct reg *reg)
{
	LOG_DEBUG("Enter");
	LOG_WARNING("NOT IMPLEMENTED");
	return ERROR_OK;
}

static struct reg
general_purpose_reg_construct(char const* const p_name, uint32_t const number, struct target* p_target)
{
	LOG_DEBUG("Enter");
	typedef struct reg reg;
	reg const the_reg = {
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
	LOG_DEBUG("Enter");
	typedef struct reg reg;
	reg const the_reg = {
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
	LOG_DEBUG("Enter");
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
	typedef struct reg_cache reg_cache;
	reg_cache const the_reg_cache = {
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
	LOG_DEBUG("Enter");
	assert(p_target);

	typedef struct This_Arch This_Arch;
	This_Arch the_arch = {
		.nc_poll_requested = DBG_REASON_DBGRQ,
		.m_reg_cache = reg_cache__create(p_target)
	};
	This_Arch* p_arch_info = calloc(1, sizeof(struct This_Arch));
	*p_arch_info = the_arch;

	p_target->arch_info = p_arch_info;
	return ERROR_OK;
}

static void
save_context(struct target *p_target)
{
	LOG_DEBUG("Enter");
	LOG_ERROR("Unimplemented");
}

static void
restore_context(struct target *p_target)
{
	LOG_DEBUG("Enter");
	LOG_ERROR("Unimplemented");
}

static int
this_poll(struct target *p_target)
{
	LOG_DEBUG("Enter");
	p_target->state = HART_DBG_STATUS_get(p_target);
	if ( p_target->state == TARGET_RUNNING ) {
		return clear_error_code();
	}

	struct This_Arch* p_arch = (struct This_Arch*)p_target->arch_info;
	if ( p_arch->nc_poll_requested == DBG_REASON_DBGRQ ) {
		p_target->debug_reason = DBG_REASON_DBGRQ;
		return clear_error_code();
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
	return clear_error_code();
}

static int
this_arch_state(struct target *p_target)
{
	LOG_DEBUG("Enter");
	LOG_ERROR("Unimplemented");
	return clear_error_code();
}

static int
this_init(struct command_context *cmd_ctx, struct target *p_target)
{
	LOG_DEBUG("Enter");
	return clear_error_code();
}

static int
this_halt(struct target *p_target)
{
	LOG_DEBUG("Enter");
	assert(p_target);
	LOG_DEBUG("May be already halted?");
	if ( HART_DBG_STATUS_get(p_target) == TARGET_HALTED ) {
		LOG_ERROR("Halt request when RV is already in halted state");
		return clear_error_code();
	}

	LOG_DEBUG("Set debug mode");
	DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_DBGCMD);
	if ( get_error_code() != ERROR_OK ) {
		return clear_error_code();
	}
	TAP_INSTR_DAP_CMD_IO(p_target, DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL, 1u);
	if ( get_error_code() != ERROR_OK ) {
		return clear_error_code();
	}
	LOG_DEBUG("Verify that in debug mode");
	p_target->state = HART_DBG_STATUS_get(p_target);
	if ( p_target->state != TARGET_HALTED ) {
		// issue error if we are still running
		LOG_ERROR("RV is not halted after Halt command");
		update_error_code(ERROR_TARGET_NOT_HALTED);
		return clear_error_code();
	}
	LOG_DEBUG("OK, halted");
	typedef struct This_Arch This_Arch;
	This_Arch* p_arch = (This_Arch*)p_target->arch_info;
	if ( p_arch->nc_poll_requested ) {
		p_arch->nc_poll_requested = DBG_REASON_DBGRQ;
	}
	p_target->debug_reason = DBG_REASON_DBGRQ;
	return clear_error_code();
}

static int
this_resume(struct target *p_target, int current, uint32_t address, int handle_breakpoints, int debug_execution)
{
	LOG_DEBUG("Enter");
	// upload reg values into HW
	restore_context(p_target);
	/// @todo issue resume command
	typedef struct This_Arch This_Arch;
	This_Arch* p_arch = (This_Arch*)p_target->arch_info;
	p_arch->nc_poll_requested = DBG_REASON_BREAKPOINT;
	p_target->state = TARGET_RUNNING;
	return clear_error_code();
}

static int
this_step(struct target *p_target, int current, uint32_t address, int handle_breakpoints)
{
	LOG_DEBUG("Enter");
	// upload reg values into HW
	restore_context(p_target);
#if 0
	debug_write_register(target, DEBUG_CONTROL_STATUS, DEBUG_CONTROL_STATUS_IRQ_DISABLE);
	debug_write_register(target, DEBUG_CONTROL_COMMAND, DEBUG_CONTROL_COMMAND_STEP);
#endif
	struct This_Arch* p_arch = (struct This_Arch*)p_target->arch_info;
	p_arch->nc_poll_requested = DBG_REASON_SINGLESTEP;
	p_target->state = TARGET_RUNNING;
	return clear_error_code();
}

enum
{
	DBGC_HART_REGS_DBG_CTRL = 0,
	DBGC_HART_REGS_DBG_STS = 1,
	DBGC_HART_REGS_DMODE_ENBL = 2,
	DBGC_HART_REGS_DMODE_CAUSE = 3,
	DBGC_HART_REGS_CORE_INSTR = 4,
	DBGC_HART_REGS_DBG_DATA = 5,
};
#define WRITE_DBGC_HART_REGS(write, index) ((uint8_t)(((index) & LOW_BITS_MASK(3)) | (!!(write) << 3)))

static int
set_reset_state(struct target *p_target, bool active)
{
	LOG_DEBUG("Enter");
	assert(p_target);
	DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_REGTRANS);
	if ( get_error_code() != ERROR_OK ) {
		return clear_error_code();
	}
	uint32_t const set_value = active ? 1u : 0u;
	TAP_INSTR_DAP_CMD_IO(p_target, WRITE_DBGC_HART_REGS(1, DBGC_HART_REGS_DBG_CTRL), set_value);
	if ( get_error_code() != ERROR_OK ) {
		return clear_error_code();
	}
	uint32_t const get_value = TAP_INSTR_DAP_CMD_IO(p_target, WRITE_DBGC_HART_REGS(0, DBGC_HART_REGS_DBG_CTRL), 0);
	if ( (get_value & 1) != (set_value & 1) ) {
		LOG_ERROR("Fail to verify reset state: set %#0x, but get %#0x", set_value, get_value);
		update_error_code(ERROR_TARGET_FAILURE);
		return clear_error_code();
	}
	p_target->state = HART_DBG_STATUS_get(p_target);
	if ( active && p_target->state != TARGET_RESET ) {
		/// issue error if we are still running
		LOG_ERROR("RV is not resetting after reset assert");
		update_error_code(ERROR_TARGET_FAILURE);
	} else if ( !active && p_target->state == TARGET_RESET ) {
		LOG_ERROR("RV is stiil in reset after reset deassert");
		update_error_code(ERROR_TARGET_FAILURE);
	}
	return clear_error_code();
}

static int
this_assert_reset(struct target *p_target)
{
	LOG_DEBUG("Enter");
	assert(p_target);
	return set_reset_state(p_target, true);
}

static int
this_deassert_reset(struct target *p_target)
{
	LOG_DEBUG("Enter");
	assert(p_target);
	return set_reset_state(p_target, false);
}

static int
this_soft_reset_halt(struct target *p_target)
{
	LOG_DEBUG("Enter");
#if 0
	int retval;
	// assert reset
	if ( (retval = debug_write_register(target, DEBUG_CONTROL_PWR_RST, DEBUG_CONTROL_PWR_RST_HRESET)) != ERROR_OK ) {
		return retval;
	}
	// ...and deassert reset
	return debug_write_register(target, DEBUG_CONTROL_PWR_RST, 0);
#endif
	return clear_error_code();
}

/// gdb_server expects valid reg values and will use set method for updating reg values
static int
read_mem_word(struct target * p_target, uint32_t const address, uint32_t* const data)
{
	LOG_DEBUG("Enter");
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
	return clear_error_code();
}

static int
this_read_memory(struct target *p_target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
	LOG_DEBUG("Enter");
	LOG_DEBUG("read_memory at %08X, %d bytes", address, size * count);
	unsigned i = 0;  // byte count
	uint32_t x = 0;  // buffer
	int retval = 0;
	unsigned const buffer_size = count * size;
	// Address is not aligned
	if ( address & 0x3 ) {
		if ( (retval = read_mem_word(p_target, address & (~0x3), &x)) != ERROR_OK ) {
			return clear_error_code();
		}
		while ( (address + i) & 0x3 && i != buffer_size ) {
			*(buffer + i) = ((uint8_t*)(&x))[(address + i) & 0x3];
			++i;
		}
	}
	for ( ; i + 4 <= buffer_size; i += 4 ) {
		if ( (retval = read_mem_word(p_target, address + i, (uint32_t*)(buffer + i))) != ERROR_OK ) {
			return clear_error_code();
		}
	}
	if ( buffer_size == i ) {
		return clear_error_code();
	}
	if ( (retval = read_mem_word(p_target, address + i, &x)) != ERROR_OK ) {
		return clear_error_code();
	}
	unsigned const word_start_offset = i;
	for ( ; i != buffer_size; ++i ) {
		*(buffer + i) = ((uint8_t*)(&x))[(i - word_start_offset) & 0x3];
	}
	return clear_error_code();
}

static int
this_examine(struct target *p_target)
{
	LOG_DEBUG("Enter");
	// initialize register values
#if 0
	debug_write_register(target, DEBUG_CONTROL_PWR_RST, DEBUG_CONTROL_PWR_RST_HRESET);
	debug_write_register(target, DEBUG_CONTROL_BREAK, DEBUG_CONTROL_RSTOFBE);
	debug_write_register(target, DEBUG_CONTROL_HALT, DEBUG_CONTROL_HALT_INT3);
	debug_write_register(target, DEBUG_CONTROL_PWR_RST, 0);
#endif
	p_target->state = TARGET_HALTED;
	save_context(p_target);
	target_set_examined(p_target);
	return clear_error_code();
}

static int
this_add_breakpoint(struct target *p_target, struct breakpoint *breakpoint)
{
	LOG_DEBUG("Enter");
	int retval = 0;
	if ( breakpoint->length != 4 || breakpoint->address & 0x3u || breakpoint->type == BKPT_HARD ) {
		return clear_error_code();
	}
	if ( (retval = target_read_buffer(p_target, breakpoint->address, breakpoint->length, breakpoint->orig_instr)) != ERROR_OK ) {
		return clear_error_code();
	}
	if ( (retval = target_write_u32(p_target, breakpoint->address, RV_SCALL())) != ERROR_OK ) {
		return clear_error_code();
	}
	breakpoint->set = 1;

	return clear_error_code();
}

static int
this_remove_breakpoint(struct target *p_target, struct breakpoint *breakpoint)
{
	LOG_DEBUG("Enter");
	if ( breakpoint->length != 4 || breakpoint->type == BKPT_HARD ) {
		return clear_error_code();
	}
	return target_write_buffer(p_target, breakpoint->address, breakpoint->length, breakpoint->orig_instr);
}

static int
this_write_memory(struct target *p_target, uint32_t address, uint32_t size, uint32_t count, const uint8_t *buffer)
{
	LOG_DEBUG("Enter");
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
this_get_gdb_reg_list(struct target *p_target, struct reg **reg_list[], int *reg_list_size, enum target_register_class reg_class)
{
	LOG_DEBUG("Enter");
	struct This_Arch *arch_info = (struct This_Arch *)p_target->arch_info;

	size_t const num_regs = ARRAY_LEN(general_regs_names_list) + (reg_class == REG_CLASS_ALL ? ARRAY_LEN(FP_regs_names_list) : 0);
	struct reg** p_reg_array = calloc(num_regs, sizeof(struct reg*));
	struct reg *a_reg_list = arch_info->m_reg_cache->reg_list;
	// memccpy(p_reg_array, arch_info->core_cache->reg_list, num_regs * sizeof(struct reg*));
	for ( size_t i = 0; i < num_regs; ++i ) {
		p_reg_array[i] = &a_reg_list[i];
	}
	*reg_list_size = num_regs;
	*reg_list = p_reg_array;
	return clear_error_code();
}
