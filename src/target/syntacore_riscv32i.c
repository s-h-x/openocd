#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdbool.h>
#include <stdint.h>
#include <helper/log.h>
#include <helper/binarybuffer.h>
#include <jtag/jtag.h>

#include "target.h"
#include "target_type.h"
#include "breakpoints.h"

#include <helper/types.h>
#include "register.h"
#include <limits.h>

// #define LOCAL_LOG_INFO LOG_INFO
#define LOCAL_LOG_INFO(expr,...) do {} while(0)
#define LOCAL_CONCAT(x,y) x##y
#define STATIC_ASSERT(e) typedef char[1 - 2*!(e)] LOCAL_CONCAT(static_assert,__LINE__)
#define ARRAY_LEN(arr) (sizeof (arr) / sizeof (arr)[0])

enum TAP_IR
{
	TAP_INSTR_IDCODE = 2,
	TAP_INSTR_DBG_ID = 3,
	TAP_INSTR_BLD_ID = 4,
	TAP_INSTR_DBG_STATUS = 5,
	TAP_INSTR_DAP_CTRL = 6,
	TAP_INSTR_DAP_CMD = 7,
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

struct arch
{
	struct reg_cache *core_cache;
	enum target_debug_reason nc_poll_requested;
};

static void
update_status(int* p_status, int status)
{
	assert(p_status);
	if ( *p_status == ERROR_OK && status != ERROR_OK ) {
		*p_status = status;
	}
}

static void
jtag_select_IR(struct target* restrict p_target, enum TAP_IR const new_instr)
{
	assert(p_target && p_target->tap);

	struct jtag_tap* restrict tap = p_target->tap;
	if ( buf_get_u32(tap->cur_instr, 0u, tap->ir_length) == new_instr ) {
		return;
	}

	assert(tap->ir_length == 4);
	uint8_t const tmp = new_instr;
	struct scan_field field = {
		.num_bits = tap->ir_length,
		.out_value = &tmp,
	};
	jtag_add_ir_scan(tap, &field, TAP_IDLE);
}

enum DBGC_CORE_CDSR_HART_bits
{
	DBGC_CORE_CDSR_HART_DMODE_BIT = 0,
	DBGC_CORE_CDSR_HART_RST_BIT = 1,
	DBGC_CORE_CDSR_HART_RST_STKY_BIT = 2,
	DBGC_CORE_CDSR_HART_ERR_BIT = 3,
	DBGC_CORE_CDSR_HART_ERR_STKY_BIT = 4,
};

static uint8_t
read_HART_DBG_STATUS(struct target* restrict p_target)
{
	assert(p_target && p_target->tap);
	assert(0 <= p_target->coreid && p_target->coreid < 2);
	jtag_select_IR(p_target, TAP_INSTR_DBG_STATUS);
	uint8_t tmp[4];
	struct scan_field field = {
		.num_bits = TAP_LEN_DBG_STATUS,
		.in_value = &tmp[0]
	};
	jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);
	return tmp[p_target->coreid];
}

static void
save_context(struct target *p_target);

static int
this_target_poll(struct target *p_target)
{
	LOCAL_LOG_INFO("poll");
	uint8_t const state = read_HART_DBG_STATUS(p_target);
	if ( !(state & DBGC_CORE_CDSR_HART_DMODE_BIT) ) {
		return ERROR_OK;
	}

	p_target->state = TARGET_HALTED;
	struct arch* p_arch = (struct arch*)p_target->arch_info;
	if ( p_arch->nc_poll_requested == DBG_REASON_DBGRQ ) {
		p_target->debug_reason = DBG_REASON_DBGRQ;
		return ERROR_OK;
	}

	if ( p_arch->nc_poll_requested == DBG_REASON_SINGLESTEP ) {
#if 0
		// enable interrupts
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
arch_state(struct target *target)
{
	LOCAL_LOG_INFO("arch_state");
	return ERROR_OK;
}

static int
init(struct command_context *cmd_ctx, struct target *target)
{
	LOCAL_LOG_INFO("init_target");
	return ERROR_OK;
}

static int
halt(struct target *target)
{
	LOCAL_LOG_INFO("halt");
	int retval;
	uint32_t nc_state = 0;
	if ( (retval = debug_read_register(target, DEBUG_CONTROL_STATUS, &nc_state)) == ERROR_OK ) {
		if ( nc_state & DEBUG_CONTROL_STATUS_HALTED ) {
			LOG_ERROR("Halt request when NC is already in halted state");
		}
		// issue Halt command
		if ( (retval = debug_write_register(target, DEBUG_CONTROL_COMMAND, DEBUG_CONTROL_COMMAND_HALT)) == ERROR_OK ) {
			// issue error if we are still running
			if ( (retval = debug_read_register(target, DEBUG_CONTROL_STATUS, &nc_state)) == ERROR_OK ) {
				if ( !(nc_state & DEBUG_CONTROL_STATUS_HALTED) ) {
					LOG_ERROR("NC is not halted after Halt command");
				}
#if 0
				if ( nc_poll_requested ) {
					nc_poll_requested = 0;
				}
#endif
				target->state = TARGET_HALTED;
				target->debug_reason = DBG_REASON_DBGRQ;
			}
		}
	}
	return retval;
}

static int
resume(struct target *target, int current, uint32_t address, int handle_breakpoints, int debug_execution)
{
	// upload reg values into HW
	restore_context(target);
	debug_write_register(target, DEBUG_CONTROL_COMMAND, DEBUG_CONTROL_COMMAND_RESUME);
	struct arch* p_arch = (struct arch*)target->arch_info;
	p_arch->nc_poll_requested = DBG_REASON_BREAKPOINT;
	target->state = TARGET_RUNNING;
	return ERROR_OK;
}

static int
step(struct target *target, int current, uint32_t address, int handle_breakpoints)
{
	// upload reg values into HW
	restore_context(target);
	debug_write_register(target, DEBUG_CONTROL_STATUS, DEBUG_CONTROL_STATUS_IRQ_DISABLE);
	debug_write_register(target, DEBUG_CONTROL_COMMAND, DEBUG_CONTROL_COMMAND_STEP);
	struct arch* p_arch = (struct arch*)target->arch_info;
	p_arch->nc_poll_requested = DBG_REASON_SINGLESTEP;
	target->state = TARGET_RUNNING;
	return ERROR_OK;
}

static int
assert_reset(struct target *target)
{
	LOCAL_LOG_INFO("assert_reset");
	int retval = 0;
	if ( (retval = debug_write_register(target, DEBUG_CONTROL_PWR_RST, DEBUG_CONTROL_PWR_RST_HRESET)) != ERROR_OK ) {
		return retval;
	}
	return ERROR_OK;
}

static int
deassert_reset(struct target *target)
{
	LOCAL_LOG_INFO("deassert_reset");
	int retval = 0;
	if ( (retval = debug_write_register(target, DEBUG_CONTROL_PWR_RST, 0)) != ERROR_OK ) {
		return retval;
	}
	return ERROR_OK;
}

static int
soft_reset_halt(struct target *target)
{
	LOCAL_LOG_INFO("soft_reset_halt");
	int retval;
	// assert reset
	if ( (retval = debug_write_register(target, DEBUG_CONTROL_PWR_RST, DEBUG_CONTROL_PWR_RST_HRESET)) == ERROR_OK ) {
		// ...and deassert reset
		retval = debug_write_register(target, DEBUG_CONTROL_PWR_RST, 0);
	}
	return retval;
}

/// gdb_server expects valid reg values and will use set method for updating reg values
static int
read_mem_word(struct target * target, uint32_t const address, uint32_t* const data)
{
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
	LOCAL_LOG_INFO("MR A %08X D %08X", address, *data);
	return ERROR_OK;
}

static int
read_memory(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
	LOCAL_LOG_INFO("read_memory at %08X, %d bytes", address, size * count);
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
examine(struct target *target)
{
	// initialize register values
	debug_write_register(target, DEBUG_CONTROL_PWR_RST, DEBUG_CONTROL_PWR_RST_HRESET);
	debug_write_register(target, DEBUG_CONTROL_BREAK, DEBUG_CONTROL_RSTOFBE);
	debug_write_register(target, DEBUG_CONTROL_HALT, DEBUG_CONTROL_HALT_INT3);
	debug_write_register(target, DEBUG_CONTROL_PWR_RST, 0);
	target->state = TARGET_HALTED;
	save_context(target);
	target_set_examined(target);
	LOCAL_LOG_INFO("examine");
	return ERROR_OK;
}

static int
this_target_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	int retval = 0;
	LOCAL_LOG_INFO("add_breakpoint");
	if ( breakpoint->length != 4 || breakpoint->address & 0x3u || breakpoint->type == BKPT_HARD) {
		return ERROR_FAIL;
	}
	if ( (retval = target_read_buffer(target, breakpoint->address, breakpoint->length, 1, breakpoint->orig_instr)) != ERROR_OK ) {
		return retval;
	}
	if ( (retval = target_write_u32(target, breakpoint->address, 0x00100073u)) != ERROR_OK ) {
		return retval;
	}
	breakpoint->set = 1;

	return ERROR_OK;
}

static int
this_target_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	int retval = 0;
	LOCAL_LOG_INFO("remove_breakpoint");
	if ( breakpoint->length != 4 || breakpoint->type == BKPT_HARD ) {
		return ERROR_FAIL;
	}
	return target_write_buffer(target, breakpoint->address, breakpoint->length, breakpoint->orig_instr);
}

static int
this_target_write_memory(struct target *target, uint32_t address, uint32_t size, uint32_t count, const uint8_t *buffer)
{
	LOCAL_LOG_INFO("write_memory at %08X, %d bytes", address, size * count);
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
}

typedef
struct register_struct
{
	uint32_t id;
	struct target* target;
} register_type;

static int
set_core_reg(struct reg *reg, uint8_t *buf)
{
	register_type* nc_reg = reg->arch_info;
	if ( nc_reg->id >= 10 ) {
		return ERROR_OK;
	}
	LOCAL_LOG_INFO("updating cache for %s register", reg->name);
	uint32_t const value = buf_get_u32(buf, 0, 32);

	if ( nc_reg->target->state != TARGET_HALTED ) {
		return ERROR_TARGET_NOT_HALTED;
	}
	buf_set_u32(reg->value, 0, 32, value);
	reg->dirty = true;
	reg->valid = true;

	return ERROR_OK;
}

static int
get_core_reg(struct reg *reg)
{
	LOG_WARNING("get_core_reg IS NOT IMPLEMENTED");
	return ERROR_OK;
}

static struct reg_arch_type const type_general_reg =
{
	.get = get_core_reg,
	.set = set_core_reg,
};

static int
set_FPU_reg(struct reg *reg, uint8_t *buf)
{
	LOG_WARNING("get_core_reg IS NOT IMPLEMENTED");
	return ERROR_OK;
}
static int
get_FPU_reg(struct reg *reg)
{
	LOG_WARNING("get_core_reg IS NOT IMPLEMENTED");
	return ERROR_OK;
}

static struct reg_arch_type const type_FPU_reg =
{
	.get = get_FPU_reg,
	.set = set_FPU_reg,
};

static char const* all_reg_list[] = {
	"x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7",
	"x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15",
	"x16", "x17", "x18", "x19", "x20", "x21", "x22", "x23",
	"x24", "x25", "x26", "x27", "x28", "x29", "x30", "x31",
	"pc",
	"f0", "f1", "f2", "f3", "f4", "f5", "f6", "f7",
	"f8", "f9", "f10", "f11", "f12", "f13", "f14", "f15",
	"f16", "f17", "f18", "f19", "f20", "f21", "f22", "f23",
	"f24", "f25", "f26", "f27", "f28", "f29", "f30", "f31",
};

static unsigned const num_regs_general = 33;
static unsigned const num_regs_all = sizeof all_reg_list / sizeof all_reg_list[0];

static int
this_target_get_gdb_reg_list(struct target *target, struct reg **reg_list[], int *reg_list_size, enum target_register_class reg_class)
{
	LOCAL_LOG_INFO("get_gdb_reg_list");
	struct arch *arch_info = (struct arch *)target->arch_info;

	unsigned const num_regs = reg_class == REG_CLASS_ALL ? num_regs_all : num_regs_general;

	*reg_list = calloc(num_regs, sizeof(struct reg*));

	for ( int i = 0; i < num_regs; ++i ) {
		(*reg_list)[i] = &arch_info->core_cache->reg_list[i];
	}
	*reg_list_size = num_regs;
	return ERROR_OK;
}

static int
this_target_create(struct target *target, struct Jim_Interp *interp)
{
	LOCAL_LOG_INFO("target_create");

	struct arch* arch_info = calloc(1, sizeof(struct arch));
	arch_info->nc_poll_requested = DBG_REASON_DBGRQ;

	// reg cache initialization
	arch_info->core_cache = malloc(sizeof(struct reg_cache));
	arch_info->core_cache->name = "syntacore_riscv32 registers";
	arch_info->core_cache->next = NULL;
	arch_info->core_cache->reg_list = calloc(num_regs_all, sizeof(struct reg));
	arch_info->core_cache->num_regs = num_regs_all;

	struct reg* p_reg = &arch_info->core_cache->reg_list[0];
	unsigned i = 0;
	for ( ; i < num_regs_general; ++i, ++p_reg ) {
		p_reg->name = all_reg_list[i];
		p_reg->size = 32;  // XLEN?
		uint32_t* value = calloc(1, sizeof(uint32_t));
		*value = 0x100 + i;
		p_reg->value = value;
		p_reg->dirty = false;
		p_reg->valid = false;
		p_reg->type = &type_general_reg;
		register_type* p_item = calloc(1, sizeof(register_type));
		p_item->id = i;
		p_item->target = target;
		p_reg->arch_info = p_item;
	}
	for ( ; i < num_regs_all; ++i, ++p_reg ) {
		p_reg->name = all_reg_list[i];
		p_reg->size = 64;  // XLEN?
		uint64_t* value = calloc(1, sizeof(uint64_t));
		*value = 0x100 + i;
		p_reg->value = value;
		p_reg->dirty = false;
		p_reg->valid = false;
		p_reg->type = &type_FPU_reg;
		register_type* p_item = calloc(1, sizeof(register_type));
		p_item->id = i;
		p_item->target = target;
		p_reg->arch_info = p_item;
	}
	target->arch_info = arch_info;
	return ERROR_OK;
}

struct target_type syntacore_riscv32i_target =
{
	.name = "syntacore_riscv32i",

	.poll = this_target_poll,
	.arch_state = arch_state,

	.target_request_data = NULL,

	.halt = halt,
	.resume = resume,
	.step = step,

	.assert_reset = assert_reset,
	.deassert_reset = deassert_reset,
	.soft_reset_halt = soft_reset_halt,

	.get_gdb_reg_list = this_target_get_gdb_reg_list,

	.read_memory = read_memory,
	.write_memory = this_target_write_memory,
	.add_breakpoint = this_target_add_breakpoint,
	.remove_breakpoint = this_target_remove_breakpoint,
	.target_create = this_target_create,
	.init_target = init,
	.examine = examine,
};
