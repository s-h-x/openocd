/** @file

Syntacore RISC-V target

@copyright Syntacore
*/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "sc_rv32i__Arch.h"
#include "helper/log.h"
#include "target/target.h"

/// Don't make irscan if IR is the same
#define USE_IR_SELECT_CACHE 0
/// Don't write DAP_CONTROL if it is the same 
#define USE_DAP_CONTROL_CACHE 0
/// Verify value of DAP_CONTROL after write
#define USE_VERIFY_DAP_CONTROL 1
/// Using PC_SAMPLE register instead AUIPC/CSRRW chain of instruction 
#define USE_PC_FROM_PC_SAMPLE 1
/// Verify values of HART REGTRANS after write
#define USE_VERIFY_HART_REGTRANS_WRITE 1
/// Verify values of CORE REGTRANS after write
#define USE_VERIFY_CORE_REGTRANS_WRITE 1
/// If first instruction in normal resume replaced by breakpoint opcode,
/// then emulate first step by execution of stored opcode with debug facilities
#define USE_RESUME_AT_SW_BREAKPOINT_EMULATES_SAVED_INSTRUCTION 0
#define USE_PC_ADVMT_DSBL_BIT 1
#define USE_QUEUING_FOR_DR_SCANS 1
#define USE_CHECK_PC_UNCHANGED USE_PC_FROM_PC_SAMPLE

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

struct sc_rv32i__Arch const sc_rv32_initial_arch = {
	.error_code = ERROR_OK,
	.last_DAP_ctrl = DAP_CTRL_INVALID_CODE,

	.use_ir_select_cache = !!(USE_IR_SELECT_CACHE),
	.use_dap_control_cache = !!(USE_DAP_CONTROL_CACHE),
	.use_verify_dap_control = !!(USE_VERIFY_DAP_CONTROL),
	.use_pc_from_pc_sample = !!(USE_PC_FROM_PC_SAMPLE),
	.use_check_pc_unchanged = !!(USE_CHECK_PC_UNCHANGED) && !!(USE_PC_FROM_PC_SAMPLE),
	.use_verify_hart_regtrans_write = !!(USE_VERIFY_HART_REGTRANS_WRITE),
	.use_verify_core_regtrans_write = !!(USE_VERIFY_CORE_REGTRANS_WRITE),
	.use_resume_at_sw_breakpoint_emulates_saved_instruction = !!(USE_RESUME_AT_SW_BREAKPOINT_EMULATES_SAVED_INSTRUCTION),
	.use_pc_advmt_dsbl_bit = !!(USE_PC_ADVMT_DSBL_BIT),
	.use_queuing_for_dr_scans = !!(USE_QUEUING_FOR_DR_SCANS),
};
