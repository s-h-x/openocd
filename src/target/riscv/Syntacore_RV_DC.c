/** @file

Syntacore RISC-V target

@copyright Syntacore
*/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "Syntacore_RV_DC.h"
#include "sc_rv32i__Arch.h"
#include "target/target.h"
#include "helper/log.h"
#include "jtag/jtag.h"
#include "static_assert.h"
#include "sc_macro.h"

#include <limits.h>
#include <assert.h>

STATIC_ASSERT(CHAR_BIT == 8);

/// IR id
enum TAP_IR_e
{
	TAP_INSTR_DBG_ID = 3,
	TAP_INSTR_BLD_ID = 4,
	TAP_INSTR_DBG_STATUS = 5,
	TAP_INSTR_DAP_CTRL = 6,
	TAP_INSTR_DAP_CTRL_RD = 7,
	TAP_INSTR_DAP_CMD = 8,
	TAP_INSTR_IDCODE = 0xE,  ///< recommended
	TAP_INSTR_BYPASS = 0xF,  ///< mandatory
};

static uint8_t const DAP_OPSTATUS_GOOD = DAP_OPSTATUS_OK;
static uint8_t const DAP_STATUS_MASK = DAP_OPSTATUS_MASK;

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
	struct scan_field field = {.num_bits = p_target->tap->ir_length,.out_value = out_buffer};
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
	uint32_t result = 0xBADC0DE0u;
	/// Low-level method save but ignore previous errors.
	int const old_err_code = error_code__get_and_clear(p_target);
	/// Error state can be updated in IR_select
	IR_select(p_target, ir);
	if ( ERROR_OK == error_code__get(p_target) ) {
		STATIC_ASSERT(TAP_LEN_RO_32 == 32);
		uint8_t result_buffer[NUM_BITS_TO_SIZE(TAP_LEN_RO_32)] = {};
		struct scan_field const field = {
			.num_bits = TAP_LEN_RO_32,
			.in_value = result_buffer,
		};
		assert(p_target->tap);
		jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);

		// enforce jtag_execute_queue() to obtain result
		error_code__update(p_target, jtag_execute_queue());
		LOG_DEBUG("drscan %s %d 0 ; # %08X", p_target->cmd_name, field.num_bits, buf_get_u32(result_buffer, 0, TAP_LEN_RO_32));
		if ( ERROR_OK != error_code__get(p_target) ) {
			/// Error state can be updated in DR scan
			LOG_ERROR("JTAG error %d", error_code__get(p_target));
		} else {
			STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_RO_32) <= sizeof(uint32_t));
			result = buf_get_u32(result_buffer, 0, TAP_LEN_RO_32);
		}
	}
	error_code__prepend(p_target, old_err_code);
	return result;
}
uint32_t sc_rv32_IDCODE_get(struct target const* const p_target)
{
	assert(p_target);
	STATIC_ASSERT(TAP_LEN_IDCODE == TAP_LEN_RO_32);
	return read_only_32_bits_regs(p_target, TAP_INSTR_IDCODE);
}
uint32_t sc_rv32_DBG_ID_get(struct target const* const p_target)
{
	assert(p_target);
	STATIC_ASSERT(TAP_LEN_DBG_ID == TAP_LEN_RO_32);
	return read_only_32_bits_regs(p_target, TAP_INSTR_DBG_ID);
}
uint32_t sc_rv32_BLD_ID_get(struct target const* const p_target)
{
	assert(p_target);
	STATIC_ASSERT(TAP_LEN_BLD_ID == TAP_LEN_RO_32);
	return read_only_32_bits_regs(p_target, TAP_INSTR_BLD_ID);
}
static uint32_t sc_rv32_DBG_STATUS_get(struct target const* const p_target)
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
static void update_DAP_CTRL_cache(struct target const* const p_target, uint8_t const set_dap_unit_group)
{
	assert(p_target);
	struct sc_rv32i__Arch* const p_arch = p_target->arch_info;
	assert(p_arch);
	p_arch->last_DAP_ctrl = set_dap_unit_group;
}
static void invalidate_DAP_CTR_cache(struct target const* const p_target)
{
	update_DAP_CTRL_cache(p_target, DAP_CTRL_INVALID_CODE);
}
/// set unit/group
static void DAP_CTRL_REG_set_force(struct target const* const p_target, uint8_t const set_dap_unit_group)
{
	assert(p_target);
	int const old_err_code = error_code__get_and_clear(p_target);
	IR_select(p_target, TAP_INSTR_DAP_CTRL);
	if ( ERROR_OK == error_code__get(p_target) ) {
		// clear status bits
		uint8_t status = 0;
		STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CTRL) == sizeof status);
		STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CTRL) == sizeof set_dap_unit_group);
		struct scan_field const field = {
			.num_bits = TAP_LEN_DAP_CTRL,
			.out_value = &set_dap_unit_group,
			.in_value = &status,
			.check_value = &DAP_OPSTATUS_GOOD,
			.check_mask = &DAP_STATUS_MASK,
		};

		invalidate_DAP_CTR_cache(p_target);
		jtag_add_dr_scan_check(p_target->tap, 1, &field, TAP_IDLE);

		// enforce jtag_execute_queue() to get status
		int const jtag_status = jtag_execute_queue();
		LOG_DEBUG("drscan %s %d 0x%1X ; # %1X", p_target->cmd_name, field.num_bits, set_dap_unit_group, status);
		if ( ERROR_OK != jtag_status ) {
			uint32_t const dbg_status = sc_rv32_DBG_STATUS_get(p_target);
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
			update_DAP_CTRL_cache(p_target, set_dap_unit_group);
		}
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
	if ( ERROR_OK == error_code__get(p_target) ) {
		uint8_t get_dap_unit_group = 0;
		uint8_t set_dap_unit_group_mask = 0x0Fu;
		STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CTRL) == sizeof get_dap_unit_group);
		struct scan_field const field =
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
		if ( ERROR_OK == error_code__get(p_target) ) {
			if ( get_dap_unit_group == set_dap_unit_group ) {
				p_arch->last_DAP_ctrl = get_dap_unit_group;
			} else {
				LOG_ERROR("Unit/Group verification error: set 0x%1X, but get 0x%1X!", set_dap_unit_group, get_dap_unit_group);
				error_code__update(p_target, ERROR_TARGET_FAILURE);
			}
		}
	}
	error_code__prepend(p_target, old_err_code);
}
void sc_rv32_DAP_CMD_scan(struct target const* const p_target, uint8_t const DAP_OPCODE, uint32_t const DAP_OPCODE_EXT, uint32_t* p_result)
{
	assert(p_target);
	int const old_err_code = error_code__get_and_clear(p_target);
	IR_select(p_target, TAP_INSTR_DAP_CMD);
	if ( ERROR_OK == error_code__get(p_target) ) {
		// Input fields
		uint8_t DAP_OPSTATUS = 0;
		STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CMD_OPCODE) == sizeof DAP_OPSTATUS);

		uint8_t dbg_data[NUM_BITS_TO_SIZE(TAP_LEN_DAP_CMD_OPCODE_EXT)] = {};
		uint8_t dap_opcode_ext[NUM_BITS_TO_SIZE(TAP_LEN_DAP_CMD_OPCODE_EXT)] = {};
		buf_set_u32(dap_opcode_ext, 0, TAP_LEN_DAP_CMD_OPCODE_EXT, DAP_OPCODE_EXT);
		struct scan_field const fields[2] =
		{
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT,.out_value = dap_opcode_ext,.in_value = dbg_data},
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE,.out_value = &DAP_OPCODE,.in_value = &DAP_OPSTATUS,.check_value = &DAP_OPSTATUS_GOOD,.check_mask = &DAP_STATUS_MASK, },
		};

		assert(p_target->tap);
		jtag_add_dr_scan_check(p_target->tap, ARRAY_LEN(fields), fields, TAP_IDLE);
		// enforse jtag_execute_queue() to get values
		error_code__update(p_target, jtag_execute_queue());
		LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X ; # %08X %1X", p_target->cmd_name, fields[0].num_bits, DAP_OPCODE_EXT, fields[1].num_bits, DAP_OPCODE, buf_get_u32(dbg_data, 0, TAP_LEN_DAP_CMD_OPCODE_EXT), DAP_OPSTATUS);
		if ( ERROR_OK == error_code__get(p_target) ) {
			if ( (DAP_OPSTATUS & DAP_OPSTATUS_MASK) != DAP_OPSTATUS_OK ) {
				LOG_ERROR("DAP_OPSTATUS == 0x%1X", (uint32_t)DAP_OPSTATUS);
				error_code__update(p_target, ERROR_TARGET_FAILURE);
			} else if ( p_result ) {
				*p_result = buf_get_u32(dbg_data, 0, TAP_LEN_DAP_CMD_OPCODE_EXT);
			}
		}
	}
	error_code__prepend(p_target, old_err_code);
}
/**
@brief Try to unlock debug controller

@warning Clear previous error_code and set ERROR_TARGET_FAILURE if unlock was unsuccessful
@return lock context
*/
static uint32_t sc_rv32_DC__unlock(struct target const* const p_target)
{
	assert(p_target);
	LOG_WARNING("========= Try to unlock ==============");

	struct sc_rv32i__Arch* const p_arch = p_target->arch_info;
	assert(p_arch);

	assert(p_target->tap);
	assert(p_target->tap->ir_length == TAP_IR_LEN);

	{
		static uint8_t const ir_out_buffer_DAP_CTRL[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {TAP_INSTR_DAP_CTRL};
		/// @todo jtag_add_ir_scan need non-const scan_field
		static struct scan_field ir_field_DAP_CTRL = {.num_bits = TAP_IR_LEN,.out_value = ir_out_buffer_DAP_CTRL};
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DAP_CTRL);
		jtag_add_ir_scan(p_target->tap, &ir_field_DAP_CTRL, TAP_IDLE);
	}

	{
		static uint8_t const set_dap_unit_group = MAKE_TYPE_FIELD(uint8_t, DBGC_UNIT_ID_HART_0, 2, 3) | MAKE_TYPE_FIELD(uint8_t, DBGC_FGRP_HART_DBGCMD, 0, 1);
		STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CTRL) == sizeof set_dap_unit_group);
		static struct scan_field const dr_field_DAP_CTRL = {.num_bits = TAP_LEN_DAP_CTRL,.out_value = &set_dap_unit_group};
		p_arch->last_DAP_ctrl = DAP_CTRL_INVALID_CODE;
		LOG_DEBUG("drscan %s %d 0x%1X", p_target->cmd_name, dr_field_DAP_CTRL.num_bits, set_dap_unit_group);
		jtag_add_dr_scan(p_target->tap, 1, &dr_field_DAP_CTRL, TAP_IDLE);
	}

	{
		static uint8_t const ir_out_buffer_DAP_CMD[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {TAP_INSTR_DAP_CMD};
		static struct scan_field ir_field_DAP_CMD = {.num_bits = TAP_IR_LEN,.out_value = ir_out_buffer_DAP_CMD};
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DAP_CMD);
		jtag_add_ir_scan(p_target->tap, &ir_field_DAP_CMD, TAP_IDLE);
	}

	uint8_t lock_context_buf[4] = {};
	{
		static uint8_t const dap_opcode_ext_UNLOCK[4] = {};
		static uint8_t const dap_opcode_UNLOCK = DBGC_DAP_OPCODE_DBGCMD_UNLOCK;
		struct scan_field const dr_fields_UNLOCK[2] = {
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT,.out_value = dap_opcode_ext_UNLOCK,.in_value = lock_context_buf},
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE,.out_value = &dap_opcode_UNLOCK}
		};
		for ( int i = 0; i < 2; ++i ) {
			LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X", p_target->cmd_name, dr_fields_UNLOCK[0].num_bits, buf_get_u32(dap_opcode_ext_UNLOCK, 0, TAP_LEN_DAP_CMD_OPCODE_EXT), dr_fields_UNLOCK[1].num_bits, dap_opcode_UNLOCK);
			jtag_add_dr_scan(p_target->tap, ARRAY_LEN(dr_fields_UNLOCK), dr_fields_UNLOCK, TAP_IDLE);
		}
	}

	{
		static uint8_t const ir_out_buffer_DBG_STATUS[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {TAP_INSTR_DBG_STATUS};
		static struct scan_field ir_field_DBG_STATUS = {.num_bits = TAP_IR_LEN,.out_value = ir_out_buffer_DBG_STATUS};
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DBG_STATUS);
		jtag_add_ir_scan(p_target->tap, &ir_field_DBG_STATUS, TAP_IDLE);
	}

	uint8_t status_buffer[4] = {};
	{
		struct scan_field const dr_field_DBG_STATUS = {.num_bits = TAP_LEN_DBG_STATUS,.out_value = status_buffer,.in_value = status_buffer};
		LOG_DEBUG("drscan %s %d 0x%08X", p_target->cmd_name, dr_field_DBG_STATUS.num_bits, buf_get_u32(status_buffer, 0, TAP_LEN_DAP_CMD_OPCODE_EXT));
		jtag_add_dr_scan(p_target->tap, 1, &dr_field_DBG_STATUS, TAP_IDLE);
	}

	// enforse jtag_execute_queue() to get values
	bool const ok =
		ERROR_OK == error_code__update(p_target, jtag_execute_queue()) &&
		0 == (buf_get_u32(status_buffer, 0, TAP_LEN_DAP_CMD_OPCODE_EXT) & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_LOCK_BIT));
	LOG_DEBUG("%s context=0x%08X, status=0x%08X", ok ? "Unlock succsessful!" : "Unlock unsuccsessful!", buf_get_u32(lock_context_buf, 0, TAP_LEN_DAP_CMD_OPCODE_EXT), buf_get_u32(status_buffer, 0, TAP_LEN_DAP_CMD_OPCODE_EXT));
	return buf_get_u32(lock_context_buf, 0, TAP_LEN_DAP_CMD_OPCODE_EXT);
}
static void sc_rv32_HART0_clear_sticky(struct target* const p_target)
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
		struct scan_field ir_dap_ctrl_field = {.num_bits = p_target->tap->ir_length,.out_value = ir_dap_ctrl_out_buffer};
		jtag_add_ir_scan(p_target->tap, &ir_dap_ctrl_field, TAP_IDLE);
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DAP_CTRL);
	}

	{
		/// set invalid cache value
		p_arch->last_DAP_ctrl = DAP_CTRL_INVALID_CODE;

		uint8_t const set_dap_unit_group = 0x1u;
		struct scan_field const dr_dap_ctrl_field = {.num_bits = TAP_LEN_DAP_CTRL,.out_value = &set_dap_unit_group};
		jtag_add_dr_scan(p_target->tap, 1, &dr_dap_ctrl_field, TAP_IDLE);
	}

	{
		uint8_t ir_dap_cmd_out_buffer[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {};
		buf_set_u32(ir_dap_cmd_out_buffer, 0, TAP_IR_LEN, TAP_INSTR_DAP_CMD);
		struct scan_field ir_dap_cmd_field = {.num_bits = p_target->tap->ir_length,.out_value = ir_dap_cmd_out_buffer};
		jtag_add_ir_scan(p_target->tap, &ir_dap_cmd_field, TAP_IDLE);
	}

	{
		uint8_t dap_opcode_ext[(TAP_LEN_DAP_CMD_OPCODE_EXT + CHAR_BIT - 1) / CHAR_BIT];
		buf_set_u32(dap_opcode_ext, 0, TAP_LEN_DAP_CMD_OPCODE_EXT, BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS));
		uint8_t const dap_opcode = DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL;
		struct scan_field const fields[2] = {
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT,.out_value = dap_opcode_ext},
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE,.out_value = &dap_opcode}
		};
		jtag_add_dr_scan(p_target->tap, ARRAY_LEN(fields), fields, TAP_IDLE);
	}
}
static uint8_t REGTRANS_scan_type(bool const write, uint8_t const index)
{
	assert((index & LOW_BITS_MASK(3)) == index);
	return
		MAKE_TYPE_FIELD(uint8_t, !!write, 3, 3) |
		MAKE_TYPE_FIELD(uint8_t, index, 0, 2);
}
static void sc_rv32_CORE_clear_errors(struct target* const p_target)
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
		struct scan_field ir_dap_ctrl_field = {.num_bits = p_target->tap->ir_length,.out_value = ir_dap_ctrl_out_buffer};
		jtag_add_ir_scan(p_target->tap, &ir_dap_ctrl_field, TAP_IDLE);
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DAP_CTRL);
	}

	{
		/// set invalid cache value
		p_arch->last_DAP_ctrl = DAP_CTRL_INVALID_CODE;

		uint8_t const set_dap_unit_group = (DBGC_UNIT_ID_CORE << TAP_LEN_DAP_CTRL_FGROUP) | DBGC_FGRP_CORE_REGTRANS;
		struct scan_field const field = {.num_bits = TAP_LEN_DAP_CTRL,.out_value = &set_dap_unit_group};
		jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);
		LOG_DEBUG("drscan %s %d 0x%1X", p_target->cmd_name, field.num_bits, set_dap_unit_group);
	}

	{
		uint8_t ir_dap_cmd_out_buffer[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {};
		buf_set_u32(ir_dap_cmd_out_buffer, 0, TAP_IR_LEN, TAP_INSTR_DAP_CMD);
		struct scan_field ir_dap_cmd_field = {.num_bits = p_target->tap->ir_length,.out_value = ir_dap_cmd_out_buffer};
		jtag_add_ir_scan(p_target->tap, &ir_dap_cmd_field, TAP_IDLE);
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DAP_CMD);
	}

	{
		uint8_t dap_opcode_ext[(TAP_LEN_DAP_CMD_OPCODE_EXT + CHAR_BIT - 1) / CHAR_BIT];
		buf_set_u32(dap_opcode_ext, 0, TAP_LEN_DAP_CMD_OPCODE_EXT, 0xFFFFFFFF);
		uint8_t const dap_opcode = REGTRANS_scan_type(true, DBGC_CORE_REGS_DBG_STS);
		struct scan_field const fields[2] = {
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT,.out_value = dap_opcode_ext},
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE,.out_value = &dap_opcode}
		};
		jtag_add_dr_scan(p_target->tap, ARRAY_LEN(fields), fields, TAP_IDLE);
		LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X", p_target->cmd_name, fields[0].num_bits, buf_get_u32(dap_opcode_ext, 0, 32), fields[1].num_bits, dap_opcode);
	}
}
void sc_rv32_DAP_CTRL_REG_set(struct target const* const p_target, enum type_dbgc_unit_id_e const dap_unit, uint8_t const dap_group)
{
	assert(p_target);
	bool const match_HART_0 = DBGC_UNIT_ID_HART_0 == dap_unit && 0 == p_target->coreid;
	bool const match_HART_1 = DBGC_UNIT_ID_HART_1 == dap_unit && 1 == p_target->coreid;
	bool const HART_unit = match_HART_0 || match_HART_1;
	bool const HART_group = DBGC_FGRP_HART_REGTRANS == dap_group || DBGC_FGRP_HART_DBGCMD == dap_group;
	bool const CORE_unit = DBGC_UNIT_ID_CORE == dap_unit;
	bool const CORE_group = DBGC_FGRP_CORE_REGTRANS == dap_group;
	assert((HART_unit && HART_group) || (CORE_unit && CORE_group));

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

	invalidate_DAP_CTR_cache(p_target);
	int const old_err_code = error_code__get_and_clear(p_target);
	DAP_CTRL_REG_set_force(p_target, set_dap_unit_group);
	if ( p_arch->use_verify_dap_control ) {
		error_code__get_and_clear(p_target);
		DAP_CTRL_REG_verify(p_target, set_dap_unit_group);
		if ( ERROR_OK == error_code__get(p_target) ) {
			update_DAP_CTRL_cache(p_target, set_dap_unit_group);
		}
	}
	error_code__prepend(p_target, old_err_code);
}
static void REGTRANS_write(struct target const* const p_target, enum type_dbgc_unit_id_e a_unit, uint8_t const a_fgrp, uint8_t const index, uint32_t const data)
{
	assert(p_target);
	sc_rv32_DAP_CTRL_REG_set(p_target, a_unit, a_fgrp);
	if ( ERROR_OK != error_code__get(p_target) ) {
		LOG_WARNING("DAP_CTRL_REG_set error");
		return;
	} else {
		sc_rv32_DAP_CMD_scan(p_target, REGTRANS_scan_type(true, index), data, NULL);
	}
}
static uint32_t REGTRANS_read(struct target const* const p_target, enum type_dbgc_unit_id_e const a_unit, uint8_t const a_fgrp, uint8_t const index)
{
	assert(p_target);
	sc_rv32_DAP_CTRL_REG_set(p_target, a_unit, a_fgrp);
	if ( error_code__get(p_target) != ERROR_OK ) {
		LOG_WARNING("DAP_CTRL_REG_set error");
		return 0xBADC0DE3u;
	}
	sc_rv32_DAP_CMD_scan(p_target, REGTRANS_scan_type(false, index), 0, NULL);
	uint32_t result;
	sc_rv32_DAP_CMD_scan(p_target, REGTRANS_scan_type(false, index), 0, &result);
	return result;
}
uint32_t sc_rv32_HART_REGTRANS_read(struct target const* const p_target, enum type_dbgc_regblock_hart_e const index)
{
	assert(p_target);
	enum type_dbgc_unit_id_e const unit = p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1;
	return REGTRANS_read(p_target, unit, DBGC_FGRP_HART_REGTRANS, index);
}
static void HART_REGTRANS_write(struct target const* const p_target, enum type_dbgc_regblock_hart_e const index, uint32_t const set_value)
{
	assert(p_target);
	enum type_dbgc_unit_id_e const unit = p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1;
	REGTRANS_write(p_target, unit, DBGC_FGRP_HART_REGTRANS, index, set_value);
}
void sc_rv32_HART_REGTRANS_write_and_check(struct target const* const p_target, enum type_dbgc_regblock_hart_e const index, uint32_t const set_value)
{
	assert(p_target);
	HART_REGTRANS_write(p_target, index, set_value);
	if ( error_code__get(p_target) == ERROR_OK ) {
		struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
		assert(p_arch);
		if ( p_arch->use_verify_hart_regtrans_write ) {
			uint32_t const get_value = sc_rv32_HART_REGTRANS_read(p_target, index);
			if ( get_value != set_value ) {
				LOG_ERROR("Write HART_REGTRANS #%d with value 0x%08X, but re-read value is 0x%08X", (uint32_t)index, set_value, get_value);
				error_code__update(p_target, ERROR_TARGET_FAILURE);
			}
		}
	}
}
uint32_t sc_rv32_core_REGTRANS_read(struct target const* const p_target, enum type_dbgc_regblock_core_e const index)
{
	assert(p_target);
	return REGTRANS_read(p_target, DBGC_UNIT_ID_CORE, DBGC_FGRP_CORE_REGTRANS, index);
}
void sc_rv32_CORE_REGTRANS_write(struct target const* const p_target, enum type_dbgc_regblock_core_e const index, uint32_t const data)
{
	assert(p_target);
	REGTRANS_write(p_target, DBGC_UNIT_ID_CORE, DBGC_FGRP_CORE_REGTRANS, index, data);
}
void sc_rv32_EXEC__setup(struct target const* const p_target)
{
	assert(p_target);
	if ( error_code__get(p_target) == ERROR_OK ) {
		sc_rv32_DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_DBGCMD);
	}
}
void sc_rv32_EXEC__push_data_to_CSR(struct target const* const p_target, uint32_t const csr_data)
{
	assert(p_target);
	if ( error_code__get(p_target) == ERROR_OK ) {
		sc_rv32_DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_DBGDATA_WR, csr_data, NULL);
	}
}
uint32_t sc_rv32_EXEC__step(struct target const* const p_target, uint32_t instruction)
{
	assert(p_target);
	uint32_t result = 0xBADC0DE9u;
	if ( ERROR_OK == error_code__get(p_target) ) {
		sc_rv32_DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_CORE_EXEC, instruction, &result);
	}
	return result;
}
uint32_t sc_rv32_get_PC(struct target const* const p_target)
{
	assert(p_target);
	struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);
	if ( p_arch->use_check_pc_unchanged ) {
		return sc_rv32_HART_REGTRANS_read(p_target, DBGC_HART_REGS_PC_SAMPLE);
	} else {
		return 0xFFFFFFFFu;
	}
}
static enum target_state HART_status_bits_to_target_state(uint32_t const status)
{
	static uint32_t const err_bits =
		BIT_NUM_TO_MASK(DBGC_HART_HDSR_ERR_BIT) |
		BIT_NUM_TO_MASK(DBGC_HART_HDSR_ERR_HWTHREAD_BIT) |
		BIT_NUM_TO_MASK(DBGC_HART_HDSR_ERR_DAP_OPCODE_BIT) |
		BIT_NUM_TO_MASK(DBGC_HART_HDSR_ERR_DBGCMD_NACK_BIT);

	if ( status & err_bits ) {
		LOG_WARNING("Error status: 0x%08x", status);
		return TARGET_UNKNOWN;
	} else if ( status & BIT_NUM_TO_MASK(DBGC_HART_HDSR_RST_BIT) ) {
		return TARGET_RESET;
	} else if ( status & BIT_NUM_TO_MASK(DBGC_HART_HDSR_DMODE_BIT) ) {
		return TARGET_HALTED;
	} else {
		return TARGET_RUNNING;
	}
}
static uint32_t try_to_get_ready(struct target* const p_target)
{
	uint32_t core_status = sc_rv32_DBG_STATUS_get(p_target);
	if ( (core_status & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT)) != 0 ) {
		return core_status;
	}
	static unsigned const max_retries = 10u;
	for ( unsigned i = 2; i <= max_retries; ++i ) {
		error_code__get_and_clear(p_target);
		core_status = sc_rv32_DBG_STATUS_get(p_target);
		if ( (core_status & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT)) != 0 ) {
			LOG_DEBUG("Ready: 0x%08X after %d requests", core_status, i);
			return core_status;
		}
	}
	LOG_ERROR("Not ready: 0x%08X after %d requests", core_status, max_retries);
	return core_status;
}

static enum target_debug_reason read_debug_cause(struct target* const p_target)
{
	assert(p_target);
	uint32_t const value = sc_rv32_HART_REGTRANS_read(p_target, DBGC_HART_REGS_DMODE_CAUSE);
	if ( error_code__get(p_target) != ERROR_OK ) {
		return DBG_REASON_UNDEFINED;
	}
	if ( value & BIT_NUM_TO_MASK(DBGC_HART_HDMCR_ENFORCE_BIT) ) {
		return DBG_REASON_DBGRQ;
	} else if ( value & BIT_NUM_TO_MASK(DBGC_HART_HDMCR_SINGLE_STEP_BIT) ) {
		return DBG_REASON_SINGLESTEP;
	} else if ( value & BIT_NUM_TO_MASK(DBGC_HART_HDMCR_SW_BRKPT_BIT) ) {
		return DBG_REASON_BREAKPOINT;
	} else if ( value & BIT_NUM_TO_MASK(DBGC_HART_HDMCR_RST_BREAK_BIT) ) {
		return DBG_REASON_DBGRQ;
	} else {
		return DBG_REASON_UNDEFINED;
	}
}
static void update_debug_reason(struct target* const p_target)
{
	assert(p_target);
	static char const* reasons_names[] = 
	{
		"DBG_REASON_DBGRQ",
		"DBG_REASON_BREAKPOINT",
		"DBG_REASON_WATCHPOINT",
		"DBG_REASON_WPTANDBKPT",
		"DBG_REASON_SINGLESTEP",
		"DBG_REASON_NOTHALTED",
		"DBG_REASON_EXIT",
		"DBG_REASON_UNDEFINED",
	};
	enum target_debug_reason const debug_reason = read_debug_cause(p_target);
	if ( debug_reason != p_target->debug_reason ) {
		LOG_DEBUG("New debug reason: 0x%08X (%s)", (uint32_t)debug_reason, debug_reason >= ARRAY_LEN(reasons_names) ? "unknown" : reasons_names[debug_reason]);
		p_target->debug_reason = debug_reason;
	}
}
static void update_debug_status(struct target* const p_target)
{
	enum target_state const old_state = p_target->state;
	/// Only 1 HART available now
	assert(p_target->coreid == 0);
	uint32_t const HART_status = sc_rv32_HART_REGTRANS_read(p_target, DBGC_HART_REGS_DBG_STS);
	enum target_state const new_state =
		ERROR_OK != error_code__get(p_target) ? TARGET_UNKNOWN :
		HART_status_bits_to_target_state(HART_status);
	LOG_DEBUG("debug_status: old=%d, new=%d", old_state, new_state);
#if 1
	if ( new_state == old_state ) {
		return;
	}
#endif
	p_target->state = new_state;
	switch ( new_state ) {
	case TARGET_HALTED:
		update_debug_reason(p_target);
		LOG_DEBUG("TARGET_EVENT_HALTED");
		target_call_event_callbacks(p_target, TARGET_EVENT_HALTED);
		break;

	case TARGET_RESET:
		update_debug_reason(p_target);
		LOG_DEBUG("TARGET_EVENT_RESET_ASSERT");
		target_call_event_callbacks(p_target, TARGET_EVENT_RESET_ASSERT);
		break;

	case TARGET_RUNNING:
		LOG_DEBUG("New debug reason: 0x%08X (DBG_REASON_NOTHALTED)", DBG_REASON_NOTHALTED);
		p_target->debug_reason = DBG_REASON_NOTHALTED;
		LOG_DEBUG("TARGET_EVENT_RESUMED");
		target_call_event_callbacks(p_target, TARGET_EVENT_RESUMED);
		break;

	case TARGET_UNKNOWN:
	default:
		LOG_WARNING("TARGET_UNKNOWN %d", new_state);
		break;
	}
}
static void check_and_repair_debug_controller_errors(struct target* const p_target)
{
	// protection from reset 
	jtag_add_tlr();
	invalidate_DAP_CTR_cache(p_target);
#if 0
	uint32_t const IDCODE = sc_rv32_IDCODE_get(p_target);
	if ( EXPECTED_IDCODE != IDCODE ) {
#if 0
		target_reset_examined(p_target);
#else
		p_target->examined = false;
#endif
		LOG_ERROR("Debug controller/JTAG error! Try to re-examine!");
		error_code__update(p_target, ERROR_TARGET_FAILURE);
		return;
	}
#endif
	uint32_t core_status = try_to_get_ready(p_target);
	if ( 0 != (core_status & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_LOCK_BIT)) ) {
		LOG_ERROR("Lock detected: 0x%08X", core_status);
		uint32_t const lock_context = sc_rv32_DC__unlock(p_target);
		if ( ERROR_OK != error_code__get(p_target) ) {
			/// return with error_code != ERROR_OK if unlock was unsuccsesful
			LOG_ERROR("Unlock unsucsessful with lock_context=0x%8X", lock_context);
			return;
		}
		core_status = sc_rv32_DBG_STATUS_get(p_target);
		LOG_INFO("Lock with lock_context=0x%8X repaired: 0x%08X", lock_context, core_status);
	}
	if ( BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT) != (core_status & (BIT_NUM_TO_MASK(DBGC_CORE_CDSR_LOCK_BIT) | BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT))) ) {
		LOG_ERROR("Core_status with LOCK!: 0x%08X", core_status);
		error_code__update(p_target, ERROR_TARGET_FAILURE);
		return;
	}
	static uint32_t const hart0_err_bits = BIT_NUM_TO_MASK(DBGC_CORE_CDSR_HART0_ERR_BIT) | BIT_NUM_TO_MASK(DBGC_CORE_CDSR_HART0_ERR_STKY_BIT);
	if ( 0 != (core_status & hart0_err_bits) ) {
		LOG_WARNING("Hart errors detected: 0x%08X", core_status);
		sc_rv32_HART0_clear_sticky(p_target);
		error_code__get_and_clear(p_target);
		core_status = sc_rv32_DBG_STATUS_get(p_target);
		LOG_WARNING("Hart errors %s: 0x%08X", core_status & hart0_err_bits ? "not fixed!" : "fixed", core_status);
	}
	static uint32_t const cdsr_err_bits =
		BIT_NUM_TO_MASK(DBGC_CORE_CDSR_ERR_BIT) |
		BIT_NUM_TO_MASK(DBGC_CORE_CDSR_ERR_HWCORE_BIT) |
		BIT_NUM_TO_MASK(DBGC_CORE_CDSR_ERR_FSMBUSY_BIT) |
		BIT_NUM_TO_MASK(DBGC_CORE_CDSR_ERR_DAP_OPCODE_BIT);
	if ( 0 != (core_status & cdsr_err_bits) ) {
		LOG_WARNING("Core errors detected: 0x%08X", core_status);
		error_code__get_and_clear(p_target);
		sc_rv32_CORE_clear_errors(p_target);
		error_code__get_and_clear(p_target);
		core_status = sc_rv32_DBG_STATUS_get(p_target);
		if ( 0 != (core_status & cdsr_err_bits) ) {
			LOG_ERROR("Core errors not fixed!: 0x%08X", core_status);
			error_code__update(p_target, ERROR_TARGET_FAILURE);
		} else {
			LOG_INFO("Core errors fixed: 0x%08X", core_status);
		}
	}
}

void sc_rv32_update_status(struct target* const p_target)
{
	assert(p_target);
	int const old_err_code = error_code__get_and_clear(p_target);
	check_and_repair_debug_controller_errors(p_target);
	if ( ERROR_OK == error_code__get(p_target) ) {
		update_debug_status(p_target);
	}
	error_code__prepend(p_target, old_err_code);
}

void sc_rv32_check_that_target_halted(struct target* const p_target)
{
	LOG_DEBUG("update_status");
	sc_rv32_update_status(p_target);
	if ( ERROR_OK == error_code__get(p_target) ) {
		if ( p_target->state != TARGET_HALTED ) {
			LOG_ERROR("Target not halted");
			error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
		}
	}
}
