/** @file

Syntacore RISC-V target

@copyright Syntacore
*/
#ifndef SC_RV32I__ARCH_H_
#define SC_RV32I__ARCH_H_

#include <stdbool.h>
#include <stdint.h>

struct sc_rv32i__Arch
{
	int error_code;
	uint8_t last_DAP_ctrl;
	bool use_ir_select_cache;
	bool use_dap_control_cache;
	bool use_verify_dap_control;
	bool use_check_pc_unchanged;
	bool use_verify_hart_regtrans_write;
	bool use_verify_core_regtrans_write;
	bool use_pc_advmt_dsbl_bit;
	bool use_queuing_for_dr_scans;
};
enum
{
	DAP_CTRL_INVALID_CODE = 0xFFu,
};
extern struct sc_rv32i__Arch const sc_rv32_initial_arch;

struct target;
/// Error code handling
///@{
int error_code__get(struct target const* const p_target);
int error_code__update(struct target const* const p_target, int const a_error_code);
int error_code__get_and_clear(struct target const* const p_target);
int error_code__prepend(struct target const* const p_target, int const old_err_code);
///@}

#endif  // SC_RV32I__ARCH_H_
