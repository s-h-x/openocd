#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target/riscv/Syntacore_RV_DC.h"
#include "helper/log.h"
#include "jtag/jtag.h"

#include <assert.h>

#ifndef NUM_BITS_TO_SIZE
#define NUM_BITS_TO_SIZE(num_bits) ( ( (size_t)(num_bits) + (8 - 1) ) / 8 )
#endif
#ifndef ARRAY_LEN
#define ARRAY_LEN(arr) (sizeof (arr) / sizeof (arr)[0])
#endif
#define LOW_BITS_MASK(n) (~(~0 << (n)))
#define MAKE_TYPE_FIELD(TYPE, bits, first_bit, last_bit)     ((((TYPE)(bits)) & LOW_BITS_MASK((last_bit) + 1u - (first_bit))) << (first_bit))

#define STATIC_ASSERT2(COND,LINE) enum {static_assertion_at_line_##LINE= 1 / !!(COND)}
#define STATIC_ASSERT1(COND,LINE) STATIC_ASSERT2(COND,LINE)
#define STATIC_ASSERT(COND)  STATIC_ASSERT1(COND,__LINE__)

typedef struct scan_field scan_field;

static uint8_t const DAP_OPSTATUS_GOOD = DAP_OPSTATUS_OK;
static uint8_t const DAP_STATUS_MASK = DAP_OPSTATUS_MASK;

int error_code__get(struct target const* const p_target)
{
	assert(p_target);
	struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);
	return p_arch->error_code;
}

int error_code__update(struct target const* const p_target, int const a_error_code)
{
	assert(p_target);
	struct sc_rv32i__Arch* const p_arch = p_target->arch_info;
	assert(p_arch);
	if ( ERROR_OK == error_code__get(p_target) && ERROR_OK != a_error_code ) {
		LOG_DEBUG("Set new error code: %d", a_error_code);
		p_arch->error_code = a_error_code;
	}
	return error_code__get(p_target);
}

int error_code__get_and_clear(struct target const* const p_target)
{
	assert(p_target);
	struct sc_rv32i__Arch* const const p_arch = p_target->arch_info;
	assert(p_arch);
	int const result = error_code__get(p_target);
	p_arch->error_code = ERROR_OK;
	return result;
}

int error_code__prepend(struct target const* const p_target, int const old_err_code)
{
	assert(p_target);
	int const new_err_code = error_code__get_and_clear(p_target);
	error_code__update(p_target, old_err_code);
	error_code__update(p_target, new_err_code);
	return error_code__get(p_target);
}

/** @brief Always perform scan to write instruction register

Method can update error_code, but ignore previous errors
*/
static void IR_select_force(struct target const* const p_target, enum TAP_IR_e const new_instr)
{
	assert(p_target);
	assert(p_target->tap);
	assert(p_target->tap->ir_length == TAP_IR_LEN);
	uint8_t out_buffer[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {};
	buf_set_u32(out_buffer, 0, TAP_IR_LEN, new_instr);
	scan_field field = {.num_bits = p_target->tap->ir_length,.out_value = out_buffer};
	jtag_add_ir_scan(p_target->tap, &field, TAP_IDLE);
	LOG_DEBUG("irscan %s %d", p_target->cmd_name, new_instr);
}

/** @brief Cached version of instruction register

Method can update error_code, but ignore previous errors

*/
static void IR_select(struct target const* const p_target, enum TAP_IR_e const new_instr)
{
	assert(p_target);
	struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);
	if ( p_arch->use_ir_select_cache ) {
		assert(p_target->tap);
		assert(p_target->tap->ir_length == TAP_IR_LEN);
		/// Skip IR scan if IR is the same
		if ( buf_get_u32(p_target->tap->cur_instr, 0u, p_target->tap->ir_length) == new_instr ) {
			return;
		}
	}
	/// or call real IR scan
	IR_select_force(p_target, new_instr);
}

/** @brief Common method to retrieve data of read-only 32-bits TAPs

Method store and update error_code, but ignore previous errors and
allows to repair error state of debug controller.
*/
static uint32_t read_only_32_bits_regs(struct target const* const p_target, enum TAP_IR_e ir)
{
	assert(p_target);
	/// Low-level method save but ignore previous errors.
	int const old_err_code = error_code__get_and_clear(p_target);
	/// Error state can be updated in IR_select
	IR_select(p_target, ir);
	if ( error_code__get(p_target) != ERROR_OK ) {
		/// and it is the general error that does not allow further data retrieving.
		error_code__prepend(p_target, old_err_code);
		/// @todo some invalid data pattern should be returned (c++ boost::optional candidate?)
		return 0xBADC0DE0u;
	}

	STATIC_ASSERT(TAP_LEN_RO_32 == 32);
	uint8_t result_buffer[NUM_BITS_TO_SIZE(TAP_LEN_RO_32)] = {};
	scan_field const field = {
		.num_bits = TAP_LEN_RO_32,
		.in_value = result_buffer,
	};
	assert(p_target->tap);
	jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);

	// enforce jtag_execute_queue() to obtain result
	error_code__update(p_target, jtag_execute_queue());
	if ( error_code__get(p_target) != ERROR_OK ) {
		/// Error state can be updated in DR scan
		LOG_ERROR("JTAG error %d", error_code__get(p_target));
	}

	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_RO_32) <= sizeof(uint32_t));
	uint32_t const result = buf_get_u32(result_buffer, 0, TAP_LEN_DBG_STATUS);
	LOG_DEBUG("drscan %s %d 0 ; # %08X", p_target->cmd_name, field.num_bits, result);

	error_code__prepend(p_target, old_err_code);
	return result;
}

uint32_t IDCODE_get(struct target const* const p_target)
{
	assert(p_target);
	STATIC_ASSERT(TAP_LEN_IDCODE == TAP_LEN_RO_32);
	return read_only_32_bits_regs(p_target, TAP_INSTR_IDCODE);
}

uint32_t DBG_ID_get(struct target const* const p_target)
{
	assert(p_target);
	STATIC_ASSERT(TAP_LEN_DBG_ID == TAP_LEN_RO_32);
	return read_only_32_bits_regs(p_target, TAP_INSTR_DBG_ID);
}

uint32_t BLD_ID_get(struct target const* const p_target)
{
	assert(p_target);
	STATIC_ASSERT(TAP_LEN_BLD_ID == TAP_LEN_RO_32);
	return read_only_32_bits_regs(p_target, TAP_INSTR_BLD_ID);
}

uint32_t DBG_STATUS_get(struct target const* const p_target)
{
	assert(p_target);
	STATIC_ASSERT(TAP_LEN_DBG_STATUS == TAP_LEN_RO_32);
	uint32_t const result = read_only_32_bits_regs(p_target, TAP_INSTR_DBG_STATUS);
	static uint32_t const result_mask =
		BIT_NUM_TO_MASK(DBGC_CORE_CDSR_HART0_ERR_BIT) |
		BIT_NUM_TO_MASK(DBGC_CORE_CDSR_ERR_BIT) |
		BIT_NUM_TO_MASK(DBGC_CORE_CDSR_ERR_HWCORE_BIT) |
		BIT_NUM_TO_MASK(DBGC_CORE_CDSR_ERR_FSMBUSY_BIT) |
		BIT_NUM_TO_MASK(DBGC_CORE_CDSR_ERR_DAP_OPCODE_BIT) |
		BIT_NUM_TO_MASK(DBGC_CORE_CDSR_LOCK_BIT) |
		BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT);
	static uint32_t const good_result = BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT);
	if ( (result & result_mask) != (good_result & result_mask) ) {
		LOG_WARNING("DBG_STATUS == 0x%08X", result);
	}
	return result;
}

/// set unit/group
static void DAP_CTRL_REG_set_force(struct target const* const p_target, uint8_t const set_dap_unit_group)
{
	assert(p_target);
	int const old_err_code = error_code__get_and_clear(p_target);
	IR_select(p_target, TAP_INSTR_DAP_CTRL);
	if ( error_code__get(p_target) != ERROR_OK ) {
		error_code__prepend(p_target, old_err_code);
		return;
	}

	// clear status bits
	uint8_t status = 0;
	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CTRL) == sizeof status);
	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CTRL) == sizeof set_dap_unit_group);
	scan_field const field = {
		.num_bits = TAP_LEN_DAP_CTRL,
		.out_value = &set_dap_unit_group,
		.in_value = &status,
		.check_value = &DAP_OPSTATUS_GOOD,
		.check_mask = &DAP_STATUS_MASK,
	};

	struct sc_rv32i__Arch* const p_arch = p_target->arch_info;
	assert(p_arch);

	/// set invalid cache value
	p_arch->last_DAP_ctrl = DAP_CTRL_INVALID_CODE;

	jtag_add_dr_scan_check(p_target->tap, 1, &field, TAP_IDLE);

	// enforce jtag_execute_queue() to get status
	int const jtag_status = jtag_execute_queue();
	LOG_DEBUG("drscan %s %d 0x%1X ; # %1X", p_target->cmd_name, field.num_bits, set_dap_unit_group, status);
	if ( jtag_status != ERROR_OK ) {
		uint32_t const dbg_status = DBG_STATUS_get(p_target);
		static uint32_t const dbg_status_check_value = BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT);
		static uint32_t const dbg_status_mask =
			BIT_NUM_TO_MASK(DBGC_CORE_CDSR_LOCK_BIT) |
			BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT);
		if ( (dbg_status & dbg_status_mask) != (dbg_status_check_value & dbg_status_mask) ) {
			LOG_ERROR("JTAG error %d, operation_status=0x%1X, dbg_status=0x%08X", jtag_status, (uint32_t)status, dbg_status);
			error_code__update(p_target, jtag_status);
		} else {
			LOG_WARNING("JTAG error %d, operation_status=0x%1X, but dbg_status=0x%08X", jtag_status, (uint32_t)status, dbg_status);
		}
	} else {
		p_arch->last_DAP_ctrl = set_dap_unit_group;
	}
	error_code__prepend(p_target, old_err_code);
}

/// verify unit/group
static void DAP_CTRL_REG_verify(struct target const* const p_target, uint8_t const set_dap_unit_group)
{
	assert(p_target);
	struct sc_rv32i__Arch* const p_arch = p_target->arch_info;
	assert(p_arch);
	p_arch->last_DAP_ctrl = DAP_CTRL_INVALID_CODE;
	int const old_err_code = error_code__get_and_clear(p_target);
	IR_select(p_target, TAP_INSTR_DAP_CTRL_RD);
	if ( error_code__get(p_target) != ERROR_OK ) {
		error_code__prepend(p_target, old_err_code);
		return;
	}
	uint8_t get_dap_unit_group = 0;
	uint8_t set_dap_unit_group_mask = 0x0Fu;
	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CTRL) == sizeof get_dap_unit_group);
	scan_field const field =
	{
		.num_bits = TAP_LEN_DAP_CTRL,
		.in_value = &get_dap_unit_group,
		.check_value = &set_dap_unit_group,
		.check_mask = &set_dap_unit_group_mask,
	};
	// enforce jtag_execute_queue() to get get_dap_unit_group
	jtag_add_dr_scan_check(p_target->tap, 1, &field, TAP_IDLE);
	error_code__update(p_target, jtag_execute_queue());
	LOG_DEBUG("drscan %s %d 0x%1X ; # %1X", p_target->cmd_name, field.num_bits, 0, get_dap_unit_group);
	if ( error_code__get(p_target) == ERROR_OK ) {
		if ( get_dap_unit_group == set_dap_unit_group ) {
			p_arch->last_DAP_ctrl = get_dap_unit_group;
		} else {
			LOG_ERROR("Unit/Group verification error: set 0x%1X, but get 0x%1X!", set_dap_unit_group, get_dap_unit_group);
			error_code__update(p_target, ERROR_TARGET_FAILURE);
		}
	}
	error_code__prepend(p_target, old_err_code);
}

uint32_t DAP_CMD_scan(struct target const* const p_target, uint8_t const DAP_OPCODE, uint32_t const DAP_OPCODE_EXT)
{
	assert(p_target);
	int const old_err_code = error_code__get_and_clear(p_target);
	IR_select(p_target, TAP_INSTR_DAP_CMD);
	if ( error_code__get(p_target) != ERROR_OK ) {
		error_code__prepend(p_target, old_err_code);
		return 0xBADC0DE2u;
	}
	// Output fields
	uint8_t const dap_opcode = DAP_OPCODE;
	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CMD_OPCODE) == sizeof dap_opcode);

	uint32_t const dap_opcode_ext = DAP_OPCODE_EXT;
	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CMD_OPCODE_EXT) == sizeof dap_opcode_ext);

	// Input fields
	uint8_t DAP_OPSTATUS = 0;
	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CMD_OPCODE) == sizeof DAP_OPSTATUS);

	uint32_t DBG_DATA = 0;
	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CMD_OPCODE_EXT) == sizeof DBG_DATA);

	scan_field const fields[2] = {
		[0] = {
		.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT,
		.out_value = (uint8_t const*)&dap_opcode_ext,
		.in_value = (uint8_t*)&DBG_DATA,
	},
	[1] =
	{
		.num_bits = TAP_LEN_DAP_CMD_OPCODE,
		.out_value = &dap_opcode,
		.in_value = &DAP_OPSTATUS,
		.check_value = &DAP_OPSTATUS_GOOD,
		.check_mask = &DAP_STATUS_MASK,
	},
	};

	assert(p_target->tap);
	jtag_add_dr_scan_check(p_target->tap, ARRAY_LEN(fields), fields, TAP_IDLE);
	// enforse jtag_execute_queue() to get values
	error_code__update(p_target, jtag_execute_queue());
	LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X ; # %08X %1X", p_target->cmd_name, fields[0].num_bits, dap_opcode_ext, fields[1].num_bits, dap_opcode, DBG_DATA, DAP_OPSTATUS);
	if ( error_code__get(p_target) == ERROR_OK ) {
		if ( (DAP_OPSTATUS & DAP_OPSTATUS_MASK) != DAP_OPSTATUS_OK ) {
			LOG_ERROR("DAP_OPSTATUS == 0x%1X", (uint32_t)DAP_OPSTATUS);
			error_code__update(p_target, ERROR_TARGET_FAILURE);
		}
	}

	error_code__prepend(p_target, old_err_code);
	return DBG_DATA;
}

uint32_t debug_controller__unlock(struct target const* const p_target)
{
	assert(p_target);
	LOG_WARNING("========= Try to unlock ==============");

	struct sc_rv32i__Arch* const p_arch = p_target->arch_info;
	assert(p_arch);

	uint32_t lock_context = 0xBADC0DEAu;

	assert(p_target->tap);
	assert(p_target->tap->ir_length == TAP_IR_LEN);

	{
		static uint8_t const ir_out_buffer_DAP_CTRL[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {TAP_INSTR_DAP_CTRL};
		/// @todo jtag_add_ir_scan need non-const scan_field
		static scan_field ir_field_DAP_CTRL = {.num_bits = TAP_IR_LEN,.out_value = ir_out_buffer_DAP_CTRL};
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DAP_CTRL);
		jtag_add_ir_scan(p_target->tap, &ir_field_DAP_CTRL, TAP_IDLE);
	}

	{
		static uint8_t const set_dap_unit_group = MAKE_TYPE_FIELD(uint8_t, DBGC_UNIT_ID_HART_0, 2, 3) | MAKE_TYPE_FIELD(uint8_t, DBGC_FGRP_HART_DBGCMD, 0, 1);
		STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CTRL) == sizeof set_dap_unit_group);
		static scan_field const dr_field_DAP_CTRL = {.num_bits = TAP_LEN_DAP_CTRL,.out_value = &set_dap_unit_group};
		p_arch->last_DAP_ctrl = DAP_CTRL_INVALID_CODE;
		LOG_DEBUG("drscan %s %d 0x%1X", p_target->cmd_name, dr_field_DAP_CTRL.num_bits, set_dap_unit_group);
		jtag_add_dr_scan(p_target->tap, 1, &dr_field_DAP_CTRL, TAP_IDLE);
	}

	{
		static uint8_t const ir_out_buffer_DAP_CMD[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {TAP_INSTR_DAP_CMD};
		static scan_field ir_field_DAP_CMD = {.num_bits = TAP_IR_LEN,.out_value = ir_out_buffer_DAP_CMD};
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DAP_CMD);
		jtag_add_ir_scan(p_target->tap, &ir_field_DAP_CMD, TAP_IDLE);
	}

	{
		static uint32_t const dap_opcode_ext_UNLOCK = 0xF0F0A5A5u;
		static uint8_t const dap_opcode_UNLOCK = DBGC_DAP_OPCODE_DBGCMD_UNLOCK;
		scan_field const dr_fields_UNLOCK[2] = {
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT,.out_value = (uint8_t const*)(&dap_opcode_ext_UNLOCK),.in_value = (uint8_t*)(&lock_context)},
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE,.out_value = &dap_opcode_UNLOCK}
		};
		LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X", p_target->cmd_name, dr_fields_UNLOCK[0].num_bits, dap_opcode_ext_UNLOCK, dr_fields_UNLOCK[1].num_bits, dap_opcode_UNLOCK);
		jtag_add_dr_scan(p_target->tap, ARRAY_LEN(dr_fields_UNLOCK), dr_fields_UNLOCK, TAP_IDLE);
		LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X", p_target->cmd_name, dr_fields_UNLOCK[0].num_bits, dap_opcode_ext_UNLOCK, dr_fields_UNLOCK[1].num_bits, dap_opcode_UNLOCK);
		jtag_add_dr_scan(p_target->tap, ARRAY_LEN(dr_fields_UNLOCK), dr_fields_UNLOCK, TAP_IDLE);
	}

	{
		static uint8_t const ir_out_buffer_DBG_STATUS[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {TAP_INSTR_DBG_STATUS};
		static scan_field ir_field_DBG_STATUS = {.num_bits = TAP_IR_LEN,.out_value = ir_out_buffer_DBG_STATUS};
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DBG_STATUS);
		jtag_add_ir_scan(p_target->tap, &ir_field_DBG_STATUS, TAP_IDLE);
	}

	uint32_t status = 0xBADC0DEBu;
	{
		scan_field const dr_field_DBG_STATUS = {.num_bits = TAP_LEN_DBG_STATUS,.out_value = (uint8_t const*)(&status),.in_value = (uint8_t*)(&status)};
		LOG_DEBUG("drscan %s %d 0x%08X", p_target->cmd_name, dr_field_DBG_STATUS.num_bits, status);
		jtag_add_dr_scan(p_target->tap, 1, &dr_field_DBG_STATUS, TAP_IDLE);
	}

	// enforse jtag_execute_queue() to get values
	bool const ok = (error_code__update(p_target, jtag_execute_queue()) == ERROR_OK) && ((status & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_LOCK_BIT)) == 0);
	LOG_DEBUG("%s context=0x%08X, status=0x%08X", ok ? "Unlock succsessful!" : "Unlock unsuccsessful!", lock_context, status);
	return lock_context;
}

void HART0_clear_sticky(struct target* const p_target)
{
	LOG_DEBUG("========= Try to clear HART0 errors ============");
	assert(p_target);
	struct sc_rv32i__Arch* const p_arch = p_target->arch_info;
	assert(p_arch);
	assert(p_target->tap);
	assert(p_target->tap->ir_length == TAP_IR_LEN);

	{
		uint8_t ir_dap_ctrl_out_buffer[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {};
		buf_set_u32(ir_dap_ctrl_out_buffer, 0, TAP_IR_LEN, TAP_INSTR_DAP_CTRL);
		scan_field ir_dap_ctrl_field = {.num_bits = p_target->tap->ir_length,.out_value = ir_dap_ctrl_out_buffer};
		jtag_add_ir_scan(p_target->tap, &ir_dap_ctrl_field, TAP_IDLE);
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DAP_CTRL);
	}

	{
		/// set invalid cache value
		p_arch->last_DAP_ctrl = DAP_CTRL_INVALID_CODE;

		uint8_t const set_dap_unit_group = 0x1u;
		scan_field const dr_dap_ctrl_field = {.num_bits = TAP_LEN_DAP_CTRL,.out_value = &set_dap_unit_group};
		jtag_add_dr_scan(p_target->tap, 1, &dr_dap_ctrl_field, TAP_IDLE);
	}

	{
		uint8_t ir_dap_cmd_out_buffer[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {};
		buf_set_u32(ir_dap_cmd_out_buffer, 0, TAP_IR_LEN, TAP_INSTR_DAP_CMD);
		scan_field ir_dap_cmd_field = {.num_bits = p_target->tap->ir_length,.out_value = ir_dap_cmd_out_buffer};
		jtag_add_ir_scan(p_target->tap, &ir_dap_cmd_field, TAP_IDLE);
	}

	{
		uint32_t const dap_opcode_ext = BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS);
		uint8_t const dap_opcode = DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL;
		scan_field const fields[2] = {
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT,.out_value = (uint8_t const*)&dap_opcode_ext},
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE,.out_value = &dap_opcode}
		};
		jtag_add_dr_scan(p_target->tap, ARRAY_LEN(fields), fields, TAP_IDLE);
	}
}

static uint8_t REGTRANS_scan_type(bool const write, uint8_t const index)
{
	assert((index & ~LOW_BITS_MASK(3)) == 0);
	return MAKE_TYPE_FIELD(uint8_t, !!write, 3, 3) | MAKE_TYPE_FIELD(uint8_t, index, 0, 2);
}

void core_clear_errors(struct target* const p_target)
{
	LOG_DEBUG("========= Try to clear core errors ============");
	assert(p_target);
	struct sc_rv32i__Arch* const p_arch = p_target->arch_info;
	assert(p_arch);
	assert(p_target->tap);
	assert(p_target->tap->ir_length == TAP_IR_LEN);

	{
		uint8_t ir_dap_ctrl_out_buffer[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {};
		buf_set_u32(ir_dap_ctrl_out_buffer, 0, TAP_IR_LEN, TAP_INSTR_DAP_CTRL);
		scan_field ir_dap_ctrl_field = {.num_bits = p_target->tap->ir_length,.out_value = ir_dap_ctrl_out_buffer};
		jtag_add_ir_scan(p_target->tap, &ir_dap_ctrl_field, TAP_IDLE);
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DAP_CTRL);
	}

	{
		/// set invalid cache value
		p_arch->last_DAP_ctrl = DAP_CTRL_INVALID_CODE;

		uint8_t const set_dap_unit_group = (DBGC_UNIT_ID_CORE << TAP_LEN_DAP_CTRL_FGROUP) | DBGC_FGRP_CORE_REGTRANS;
		scan_field const field = {.num_bits = TAP_LEN_DAP_CTRL,.out_value = &set_dap_unit_group};
		jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);
		LOG_DEBUG("drscan %s %d 0x%1X", p_target->cmd_name, field.num_bits, set_dap_unit_group);
	}

	{
		uint8_t ir_dap_cmd_out_buffer[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {};
		buf_set_u32(ir_dap_cmd_out_buffer, 0, TAP_IR_LEN, TAP_INSTR_DAP_CMD);
		scan_field ir_dap_cmd_field = {.num_bits = p_target->tap->ir_length,.out_value = ir_dap_cmd_out_buffer};
		jtag_add_ir_scan(p_target->tap, &ir_dap_cmd_field, TAP_IDLE);
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DAP_CMD);
	}

	{
		uint32_t const dap_opcode_ext = 0xFFFFFFFF;
		uint8_t const dap_opcode = REGTRANS_scan_type(true, DBGC_CORE_REGS_DBG_STS);
		scan_field const fields[2] = {
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT,.out_value = (uint8_t const*)&dap_opcode_ext},
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE,.out_value = &dap_opcode}
		};
		jtag_add_dr_scan(p_target->tap, ARRAY_LEN(fields), fields, TAP_IDLE);
		LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X", p_target->cmd_name, fields[0].num_bits, dap_opcode_ext, fields[1].num_bits, dap_opcode);
	}
}

void DAP_CTRL_REG_set(struct target const* const p_target, enum type_dbgc_unit_id_e const dap_unit, uint8_t const dap_group)
{
	assert(p_target);
	assert(
		(
		((dap_unit == DBGC_UNIT_ID_HART_0 && 0 == p_target->coreid) || (dap_unit == DBGC_UNIT_ID_HART_1 && 1 == p_target->coreid)) &&
		(dap_group == DBGC_FGRP_HART_REGTRANS || dap_group == DBGC_FGRP_HART_DBGCMD)
	) ||
	(dap_unit == DBGC_UNIT_ID_CORE && dap_group == DBGC_FGRP_HART_REGTRANS)
	);

	uint8_t const set_dap_unit_group =
		MAKE_TYPE_FIELD(uint8_t,
		MAKE_TYPE_FIELD(uint8_t, dap_unit, TAP_LEN_DAP_CTRL_FGROUP, TAP_LEN_DAP_CTRL_FGROUP + TAP_LEN_DAP_CTRL_UNIT - 1) |
		MAKE_TYPE_FIELD(uint8_t, dap_group, 0, TAP_LEN_DAP_CTRL_FGROUP - 1),
		0,
		TAP_LEN_DAP_CTRL_FGROUP + TAP_LEN_DAP_CTRL_UNIT - 1);

	struct sc_rv32i__Arch* const p_arch = p_target->arch_info;
	assert(p_arch);
	if ( p_arch->use_dap_control_cache ) {
		if ( p_arch->last_DAP_ctrl == set_dap_unit_group ) {
			return;
		}
		LOG_DEBUG("DAP_CTRL_REG of %s reset to 0x%1X", p_target->cmd_name, set_dap_unit_group);
	}

	/// Invalidate last_DAP_ctrl
	p_arch->last_DAP_ctrl = DAP_CTRL_INVALID_CODE;

	int const old_err_code = error_code__get_and_clear(p_target);
	DAP_CTRL_REG_set_force(p_target, set_dap_unit_group);
	if ( p_arch->use_verify_dap_control ) {
		error_code__get_and_clear(p_target);
		DAP_CTRL_REG_verify(p_target, set_dap_unit_group);
		if ( error_code__get(p_target) == ERROR_OK ) {
			p_arch->last_DAP_ctrl = set_dap_unit_group;
		}
	}
	error_code__prepend(p_target, old_err_code);
}

static void REGTRANS_write(struct target const* const p_target, enum type_dbgc_unit_id_e a_unit, uint8_t const a_fgrp, uint8_t const index, uint32_t const data)
{
	assert(p_target);
	DAP_CTRL_REG_set(p_target, a_unit, a_fgrp);
	if ( error_code__get(p_target) != ERROR_OK ) {
		LOG_WARNING("DAP_CTRL_REG_set error");
		return;
	}
	(void)DAP_CMD_scan(p_target, REGTRANS_scan_type(true, index), data);
}

static uint32_t REGTRANS_read(struct target const* const p_target, enum type_dbgc_unit_id_e const a_unit, uint8_t const a_fgrp, uint8_t const index)
{
	assert(p_target);
	DAP_CTRL_REG_set(p_target, a_unit, a_fgrp);
	if ( error_code__get(p_target) != ERROR_OK ) {
		LOG_WARNING("DAP_CTRL_REG_set error");
		return 0xBADC0DE3u;
	}
	(void)DAP_CMD_scan(p_target, REGTRANS_scan_type(false, index), 0);
	return DAP_CMD_scan(p_target, REGTRANS_scan_type(false, index), 0);
}

uint32_t HART_REGTRANS_read(struct target const* const p_target, enum type_dbgc_regblock_hart_e const index)
{
	assert(p_target);
	typedef enum type_dbgc_unit_id_e type_dbgc_unit_id_e;
	type_dbgc_unit_id_e const unit = p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1;
	return REGTRANS_read(p_target, unit, DBGC_FGRP_HART_REGTRANS, index);
}

static void HART_REGTRANS_write(struct target const* const p_target, enum type_dbgc_regblock_hart_e const index, uint32_t const set_value)
{
	assert(p_target);
	typedef enum type_dbgc_unit_id_e type_dbgc_unit_id_e;
	type_dbgc_unit_id_e const unit = p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1;
	REGTRANS_write(p_target, unit, DBGC_FGRP_HART_REGTRANS, index, set_value);
}

void HART_REGTRANS_write_and_check(struct target const* const p_target, enum type_dbgc_regblock_hart_e const index, uint32_t const set_value)
{
	assert(p_target);
	HART_REGTRANS_write(p_target, index, set_value);
	if ( error_code__get(p_target) == ERROR_OK ) {
		struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
		assert(p_arch);
		if ( p_arch->use_verify_hart_regtrans_write ) {
			uint32_t const get_value = HART_REGTRANS_read(p_target, index);
			if ( get_value != set_value ) {
				LOG_ERROR("Write HART_REGTRANS #%d with value 0x%08X, but re-read value is 0x%08X", (uint32_t)index, set_value, get_value);
				error_code__update(p_target, ERROR_TARGET_FAILURE);
			}
		}
	}
}

uint32_t core_REGTRANS_read(struct target const* const p_target, enum type_dbgc_regblock_core_e const index)
{
	assert(p_target);
	return REGTRANS_read(p_target, DBGC_UNIT_ID_CORE, DBGC_FGRP_CORE_REGTRANS, index);
}

void core_REGTRANS_write(struct target const* const p_target, enum type_dbgc_regblock_core_e const index, uint32_t const data)
{
	assert(p_target);
	REGTRANS_write(p_target, DBGC_UNIT_ID_CORE, DBGC_FGRP_CORE_REGTRANS, index, data);
}

void exec__setup(struct target const* const p_target)
{
	assert(p_target);
	if ( error_code__get(p_target) == ERROR_OK ) {
		DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_DBGCMD);
	}
}

void exec__set_csr_data(struct target const* const p_target, uint32_t const csr_data)
{
	assert(p_target);
	if ( error_code__get(p_target) == ERROR_OK ) {
		DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_DBGDATA_WR, csr_data);
	}
}

uint32_t exec__step(struct target const* const p_target, uint32_t instruction)
{
	assert(p_target);
	if ( error_code__get(p_target) != ERROR_OK ) {
		return 0xBADC0DE9u;
	}
	return DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_CORE_EXEC, instruction);
}
