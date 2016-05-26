/** @file

Syntacore RISC-V target

@copyright Syntacore
*/
#ifndef SYNTACORE_RV_DC_H_
#define SYNTACORE_RV_DC_H_

#include <stdint.h>

#define EXPECTED_IDCODE (0xC0DEDEB1u)
/// Lowest required DBG_ID
#define EXPECTED_DBG_ID        (0x00800001u)
/// Mask of DBG_ID version.
/// Required and provided masked values should be equal.
#define DBG_ID_VERSION_MASK    (0xFFFFFF00u)
/// Mask of DBG_ID subversion.
/// Required value should be less or equal to provided subversion.
#define DBG_ID_SUBVERSION_MASK (0x000000FFu)

#ifndef BIT_NUM_TO_MASK
#define BIT_NUM_TO_MASK(bit_num) (1u << (bit_num))
#endif  // BIT_NUM_TO_MASK

enum TAP_DR_LEN_e
{
	TAP_IR_LEN = 4,
	TAP_LEN_RO_32 = 32,
	TAP_LEN_IDCODE = TAP_LEN_RO_32,  ///< mandatory
	TAP_LEN_DBG_ID = TAP_LEN_RO_32,
	TAP_LEN_BLD_ID = TAP_LEN_RO_32,
	TAP_LEN_DBG_STATUS = TAP_LEN_RO_32,
	TAP_LEN_DAP_CTRL_UNIT = 2,
	TAP_LEN_DAP_CTRL_FGROUP = 2,
	TAP_LEN_DAP_CTRL = TAP_LEN_DAP_CTRL_UNIT + TAP_LEN_DAP_CTRL_FGROUP,
	TAP_NUM_FIELDS_DAP_CMD = 2,
	TAP_LEN_DAP_CMD_OPCODE = 4,
	TAP_LEN_DAP_CMD_OPCODE_EXT = 32,
	TAP_LEN_DAP_CMD = TAP_LEN_DAP_CMD_OPCODE + TAP_LEN_DAP_CMD_OPCODE_EXT,
	TAP_LEN_BYPASS = 1,  ///< mandatory
};

/// @see TAP_INSTR_DBG_STATUS
enum type_dbgc_core_dbg_sts_reg_bits_e
{
	DBGC_CORE_CDSR_HART0_DMODE_BIT = 0,
	DBGC_CORE_CDSR_HART0_RST_BIT = 1,
	DBGC_CORE_CDSR_HART0_RST_STKY_BIT = 2,
	DBGC_CORE_CDSR_HART0_ERR_BIT = 3,
	DBGC_CORE_CDSR_HART0_ERR_STKY_BIT = 4,
	DBGC_CORE_CDSR_ERR_BIT = 16,
	DBGC_CORE_CDSR_ERR_STKY_BIT = 17,
	DBGC_CORE_CDSR_ERR_HWCORE_BIT = 18,
	DBGC_CORE_CDSR_ERR_FSMBUSY_BIT = 19,
	DBGC_CORE_CDSR_ERR_DAP_OPCODE_BIT = 20,
	DBGC_CORE_CDSR_LOCK_BIT = 30,
	DBGC_CORE_CDSR_READY_BIT = 31,
};

/// @see TAP_INSTR_DAP_CMD
/// @see TAP_INSTR_DAP_CTRL
enum DAP_OPSTATUS_BITS_e
{
	DAP_OPSTATUS_EXCEPT = 0,
	DAP_OPSTATUS_ERROR = 1,
	DAP_OPSTATUS_LOCK = 2,
	DAP_OPSTATUS_READY = 3,
};

enum DAP_OPSTATUS_e
{
	DAP_OPSTATUS_MASK = BIT_NUM_TO_MASK(DAP_OPSTATUS_ERROR) | BIT_NUM_TO_MASK(DAP_OPSTATUS_LOCK) | BIT_NUM_TO_MASK(DAP_OPSTATUS_READY),
	DAP_OPSTATUS_OK = BIT_NUM_TO_MASK(DAP_OPSTATUS_READY),
};

/// Units IDs
enum type_dbgc_unit_id_e
{
	DBGC_UNIT_ID_HART_0 = 0,
	DBGC_UNIT_ID_HART_1 = 1,
	DBGC_UNIT_ID_CORE = 3,
};

/// Functional groups for HART units
///@{
enum type_dbgc_hart_fgroup_e
{
	/// @see type_dbgc_regblock_hart_e
	DBGC_FGRP_HART_REGTRANS = 0,

	/// @see type_dbgc_dap_cmd_opcode_dbgcmd_e
	DBGC_FGRP_HART_DBGCMD = 1,
};

/// @see DBGC_FGRP_HART_REGTRANS
enum type_dbgc_regblock_hart_e
{
	/// Hart Debug Control Register (HART_DBG_CTRL, HDCR)
	/// @see type_dbgc_hart_dbg_ctrl_reg_bits_e
	DBGC_HART_REGS_DBG_CTRL = 0,

	/// Hart Debug Status Register (HART_DBG_STS, HDSR) 
	/// @see type_dbgc_hart_dbg_sts_reg_bits_e
	DBGC_HART_REGS_DBG_STS = 1,

	/// Hart Debug Mode Enable Register (HART_DMODE_ENBL, HDMER) 
	/// @see type_dbgc_hart_dmode_enbl_reg_bits_e
	DBGC_HART_REGS_DMODE_ENBL = 2,

	/// Hart Debug Mode Cause Register (HART_DMODE_CAUSE, HDMCR) 
	/// @see type_dbgc_hart_dmode_cause_reg_bits_e
	DBGC_HART_REGS_DMODE_CAUSE = 3,

	/// Debugger emitting instruction register
	/// @see DBGC_DAP_OPCODE_DBGCMD_CORE_EXEC
	DBGC_HART_REGS_CORE_INSTR = 4,
	/// Debugger data to debug CSR register
	/// @see CSR_DBG_SCRATCH
	DBGC_HART_REGS_DBG_DATA = 5,
	/// PC value, available in any state
	DBGC_HART_REGS_PC_SAMPLE = 6,
};

/// @see DBGC_HART_REGS_DBG_CTRL
enum type_dbgc_hart_dbg_ctrl_reg_bits_e
{
	/// HART reset bit
	/// @warning not used now
	DBGC_HART_HDCR_RST_BIT = 0,
	/// Disable pc change for instructions emitting from debugger
	DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT = 6,
};

/// @see DBGC_HART_REGS_DBG_STS
enum type_dbgc_hart_dbg_sts_reg_bits_e
{
	/// Show halt state
	DBGC_HART_HDSR_DMODE_BIT = 0,
	/// Show current reset asserting state
	DBGC_HART_HDSR_RST_BIT = 1,
	/// Show that was reset
	DBGC_HART_HDSR_RST_STKY_BIT = 2,
	/// Show exception state (SW breakpoint too)
	DBGC_HART_HDSR_EXCEPT_BIT = 3,
	/// Show common error
	DBGC_HART_HDSR_ERR_BIT = 16,
	/// Show HW thread error
	DBGC_HART_HDSR_ERR_HWTHREAD_BIT = 17,
	/// Show error due to bad DAP opcode
	DBGC_HART_HDSR_ERR_DAP_OPCODE_BIT = 18,
	DBGC_HART_HDSR_ERR_DBGCMD_NACK_BIT = 19,
	DBGC_HART_HDSR_ERR_ILLEG_DBG_CONTEXT_BIT = 20,
	DBGC_HART_HDSR_ERR_UNEXP_RESET_BIT = 21,
	/// Show debug controller lock after error state
	/// Only unlock procedure available
	DBGC_HART_HDSR_LOCK_STKY_BIT = 31
};

/// Hart Debug Mode Enable Register (HART_DMODE_ENBL, HDMER)
/// @see DBGC_HART_REGS_DMODE_ENBL
enum type_dbgc_hart_dmode_enbl_reg_bits_e
{
	/// Enable HALT on SW breakpoint exception
	DBGC_HART_HDMER_SW_BRKPT_BIT = 3,
	/// Enable HALT after single step
	DBGC_HART_HDMER_SINGLE_STEP_BIT = 28,
#if 0
	// Not implemented, Reserved for future use
	DBGC_HART_HDMER_RST_ENTR_BRK_BIT = 29,
#endif
	/// Enable HALT after reset
	DBGC_HART_HDMER_RST_EXIT_BRK_BIT = 30,
};

enum
{
	NORMAL_DEBUG_ENABLE_MASK = BIT_NUM_TO_MASK(DBGC_HART_HDMER_SW_BRKPT_BIT) | BIT_NUM_TO_MASK(DBGC_HART_HDMER_RST_EXIT_BRK_BIT)
};

/// Hart Debug Mode Cause Register (HART_DMODE_CAUSE, HDMCR)
/// @see DBGC_HART_REGS_DMODE_CAUSE
enum type_dbgc_hart_dmode_cause_reg_bits_e
{
	/// Show halt due to SW breakpoint
	DBGC_HART_HDMCR_SW_BRKPT_BIT = 3,
	/// Show halt after single step
	DBGC_HART_HDMCR_SINGLE_STEP_BIT = 29,
	/// Show halt after reset
	DBGC_HART_HDMCR_RST_BREAK_BIT = 30,
	/// Show forced halt from debug controller
	DBGC_HART_HDMCR_ENFORCE_BIT = 31
};

/// @see DBGC_FGRP_HART_DBGCMD
enum type_dbgc_dap_cmd_opcode_dbgcmd_e
{
	/// @see type_dbgc_dap_cmd_opcode_dbgctrl_ext_e
	DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL = 0,
	/// Debugger emitting instruction register
	/// @see DBGC_HART_REGS_CORE_INSTR
	DBGC_DAP_OPCODE_DBGCMD_CORE_EXEC = 1,
	/// @see DBGC_HART_REGS_DBG_DATA
	DBGC_DAP_OPCODE_DBGCMD_DBGDATA_WR = 2,
	/// @see DBGC_HART_HDSR_LOCK_STKY_BIT
	/// @see DBGC_CORE_CDSR_LOCK_BIT
	/// @see DAP_OPSTATUS_LOCK
	DBGC_DAP_OPCODE_DBGCMD_UNLOCK = 3,
};

/// @see DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL
/// @see type_dbgc_dap_cmd_opcode_dbgctrl_ext_s
enum type_dbgc_dap_cmd_opcode_dbgctrl_ext_e
{
	DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_HALT = 0,
	DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_RESUME = 1,
	DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS = 2,
};

///@}

/// Functional groups for CORE units
///@{
enum type_dbgc_core_fgroup_e
{
	/// @see type_dbgc_regblock_core_e
	DBGC_FGRP_CORE_REGTRANS = 0,
};

/// @see DBGC_FGRP_CORE_REGTRANS
enum type_dbgc_regblock_core_e
{
	DBGC_CORE_REGS_DEBUG_ID = 0,

	/// @see type_dbgc_core_dbg_ctrl_reg_bits_e
	DBGC_CORE_REGS_DBG_CTRL = 1,
	DBGC_CORE_REGS_DBG_STS = 2,
#if 0
	DBGC_CORE_REGS_DBG_CMD = 3,
#endif
};

/// Core Debug Control Register (CORE_DBG_CTRL, CDCR)
/// @see DBGC_CORE_REGS_DBG_CTRL
enum type_dbgc_core_dbg_ctrl_reg_bits_e
{
	DBGC_CORE_CDCR_HART0_RST_BIT = 0,
	DBGC_CORE_CDCR_HART1_RST_BIT = 8,
	DBGC_CORE_CDCR_RST_BIT = 24,
	DBGC_CORE_CDCR_IRQ_DSBL_BIT = 25,
};

///@}

struct target;
/// TAPs methods
/// @{
uint32_t sc_rv32_IDCODE_get(struct target const* const p_target);
uint32_t sc_rv32_DBG_ID_get(struct target const* const p_target);
uint32_t sc_rv32_BLD_ID_get(struct target const* const p_target);
uint32_t sc_rv32_DAP_CMD_scan(struct target const* const p_target, uint8_t const DAP_OPCODE, uint32_t const DAP_OPCODE_EXT);
void sc_rv32_DAP_CTRL_REG_set(struct target const* const p_target, enum type_dbgc_unit_id_e const dap_unit, uint8_t const dap_group);
uint32_t sc_rv32_HART_REGTRANS_read(struct target const* const p_target, enum type_dbgc_regblock_hart_e const index);
void sc_rv32_HART_REGTRANS_write_and_check(struct target const* const p_target, enum type_dbgc_regblock_hart_e const index, uint32_t const set_value);
uint32_t sc_rv32_core_REGTRANS_read(struct target const* const p_target, enum type_dbgc_regblock_core_e const index);
void sc_rv32_CORE_REGTRANS_write(struct target const* const p_target, enum type_dbgc_regblock_core_e const index, uint32_t const data);
void sc_rv32_EXEC__setup(struct target const* const p_target);
void sc_rv32_EXEC__push_data_to_CSR(struct target const* const p_target, uint32_t const csr_data);
uint32_t sc_rv32_EXEC__step(struct target const* const p_target, uint32_t instruction);
/// @}

uint32_t sc_rv32_get_PC(struct target const* const p_target);
void sc_rv32_check_PC_value(struct target const* const p_target, uint32_t const pc_sample_1);
void sc_rv32_update_status(struct target* const p_target);
void sc_rv32_check_that_target_halted(struct target* const p_target);

#endif  // SYNTACORE_RV_DC_H_
