#ifndef TARGET_SC_RV32_COMMON_H_
#define TARGET_SC_RV32_COMMON_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target/target.h"
#include "target/target_type.h"
#include "target/breakpoints.h"
#include "target/register.h"
#include "jtag/jtag.h"
#include "helper/log.h"

#include <limits.h>
#include <memory.h>
#include <limits.h>
#include <assert.h>

#include <stdbool.h>

#define FPU_ENABLE (!!1)
#define WRITE_BUFFER_THRESHOLD (1u << 18)

/// Parameters of RISC-V core
/// @{
/// Size of RISC-V GP registers in bits
#define XLEN (32u)
/// Size of RISC-V FP registers in bits
#define FLEN (64u)
/// Size of RISC-V instruction
#define ILEN (32u)
/// @}

/// DBG_ID
/// @{
/// Mask of DBG_ID version.
#define DBG_ID_VERSION_MASK    (0xFFFFFF00u)

/// Mask of DBG_ID subversion.
/// Required value should be less or equal to provided subversion.
#define DBG_ID_SUBVERSION_MASK (0x000000FFu)
/// @}

/// Back-end static assert macro that raise compile time division to zero when static 'COND' is false
#define STATIC_ASSERT2(COND,LINE) enum {static_assertion_at_line_##LINE= 1 / !!(COND)}
/// Intermediate macro
#define STATIC_ASSERT1(COND,LINE) STATIC_ASSERT2(COND,LINE)
/// Front-end static assert macro
#define STATIC_ASSERT(COND)  STATIC_ASSERT1(COND,__LINE__)

/// Bit operation set of macros
/// @{

/// Simple mask with only single bit 'bit_num' is set
#define BIT_MASK(bit_num) (1u << (bit_num))

/// Bit mask value with low 'n' bits set
#define LOW_BITS_MASK(n) (~(~0 << (n)))

/// @return expression of TYPE with 'first_bit':'last_bit' bits set
#define MAKE_TYPE_FIELD(TYPE, bits, first_bit, last_bit)     ((((TYPE)(bits)) & LOW_BITS_MASK((last_bit) + 1u - (first_bit))) << (first_bit))

/// @return bit-field value
#define EXTRACT_FIELD(bits, first_bit, last_bit) (((bits) >> (first_bit)) & LOW_BITS_MASK((last_bit) + 1u - (first_bit)))

/** Number of octets required for 'num_bits' bits
@param [in] num_bits number of bits
@return number of bytes for 'num_bits' \f$\lceil {\frac{num\_bits}{CHAR\_BIT}} \rceil\f$.
*/
#define NUM_BYTES_FOR_BITS(num_bits) ( ( (size_t)(num_bits) + (CHAR_BIT - 1) ) / CHAR_BIT )

/// @return true if values bits except lower 'LEN' are zero
#define IS_VALID_UNSIGNED_FIELD(FLD,LEN) ((FLD & ~LOW_BITS_MASK(LEN)) == 0)
#define NORMALIZE_INT_FIELD(FLD, SIGN_BIT, ZEROS) ( ( ( ( -( ( (FLD) >> (SIGN_BIT) ) & LOW_BITS_MASK(1) ) ) << (SIGN_BIT) ) | (FLD) ) & ~LOW_BITS_MASK(ZEROS) )
#define IS_VALID_SIGNED_IMMEDIATE_FIELD(FLD, SIGN_BIT, LOW_ZEROS) ( (FLD) == NORMALIZE_INT_FIELD((FLD), (SIGN_BIT), (LOW_ZEROS)) )

/// @}

/// Number of array 'arr' elements
#define ARRAY_LEN(arr) (sizeof (arr) / sizeof (arr)[0])

/// Specialized assert check that register number is 5 bits only
#define CHECK_REG(REG) assert(IS_VALID_UNSIGNED_FIELD(REG,5))
#define CHECK_OPCODE(OPCODE) assert(IS_VALID_UNSIGNED_FIELD(OPCODE,7) && (OPCODE & LOW_BITS_MASK(2)) == LOW_BITS_MASK(2) && (OPCODE & LOW_BITS_MASK(5)) != LOW_BITS_MASK(5))

/// Specialized asserts to check format of RISC-V immediate
/// @{
#define CHECK_IMM_11_00(imm) assert(IS_VALID_SIGNED_IMMEDIATE_FIELD(imm, 11, 0));
#define CHECK_IMM_12_01(imm) assert(IS_VALID_SIGNED_IMMEDIATE_FIELD(imm, 12, 1));
#define CHECK_IMM_20_01(imm) assert(IS_VALID_SIGNED_IMMEDIATE_FIELD(imm, 20, 1));
/// @}

#define CHECK_FUNC7(F) assert(IS_VALID_UNSIGNED_FIELD(F,7))
#define CHECK_FUNC3(F) assert(IS_VALID_UNSIGNED_FIELD(F,3))

STATIC_ASSERT(CHAR_BIT == 8);

typedef struct target target;
typedef struct reg reg;
typedef struct reg_cache reg_cache;
typedef struct reg_arch_type reg_arch_type;
typedef struct reg_data_type_union_field reg_data_type_union_field;
typedef struct reg_data_type_union reg_data_type_union;
typedef struct breakpoint breakpoint;
typedef struct command_context command_context;
typedef struct Jim_Interp Jim_Interp;
typedef struct target_type target_type;
typedef struct scan_field scan_field;
typedef struct reg_data_type reg_data_type;
typedef struct reg_feature reg_feature;
typedef uint8_t reg_num_type;
typedef int32_t riscv_signed_type;
typedef int16_t riscv_short_signed_type;
typedef uint16_t csr_num_type;
typedef enum target_register_class target_register_class;
typedef enum target_state target_state;
typedef enum target_debug_reason target_debug_reason;
typedef int error_code;

/** @defgroup SC_RV32_TAPC_IR Syntacore TAP Controller Instructions
@ingroup SC_RV32
*/
/// @{

/** @brief TAP Controller Instructions ID
ir_scan selects one of instruction registers IO to TDI/TDO
*/
enum TAP_IR_e {
	/** @brief DBG_ID Register Read

	It connects DBG_ID_DR register between TDI and TDO pins.
	Contains debug facilities version implemented in the given processor DBGC subsystem.
	*/
	TAP_instruction_DBG_ID = 0b0011u,

	/** @brief BLD_ID Register Read

	Connects BLD_ID_DR between TDI and TDO pins.
	It identifies an entire processor system RTL build revision.
	*/
	TAP_instruction_BLD_ID = 0b0100u,

	/** @brief DBG_STATUS Register Read
	Connects DBG_STATUS_DR register providing general status information about debug operations and core state.
	*/
	TAP_instruction_DBG_STATUS = 0b0101u,

	/** @brief DAP_CTRL Register Write

	Connects DAP_CTRL_DR register is the control port of the upper level multiplexer
	for DAP_CMD register.
	*/
	TAP_instruction_DAP_CTRL = 0b0110u,

	/** @brief DAP_CTRL Register Read

	Connects DAP_CTRL_RD_DR register allowing to read
	current DAP Control Context (DAPCC) from the DAP_CONTEXT register.
	*/
	TAP_instruction_DAP_CTRL_RD = 0b0111u,

	/** @brief Debug Access Port Command (DAP Command)

	Provide multiplexed IO to inner DC registers.

	Multiplexing is controlled by DAP_CTRL
	Connects DAP_CMD_DR register.
	*/
	TAP_instruction_DAP_CMD = 0b1000u,

	/** @brief SYS_CTRL Register Access

	Connects SYS_CTRL_DR register,
	used to control state of the Processor Subsystem Reset net,
	and to get its current status.
	*/
	TAP_instruction_SYS_CTRL = 0b1001u,

	/** @brief MTAP_SWITCH Register Access

	Connects MTAP_SWITCH_DR register, used to control state
	of the Master TAP Switch Control output, and to
	get its current status.
	*/
	TAP_instruction_MTAP_SWITCH = 0b1101u,
	/** @brief IDCODE Register Read
	Conventional recommended Device Identification instruction compliant with IEEE 1149.1 Standard.
	It connects IDCODE_DR register between TDI and TDO pins.()
	*/
	TAP_instruction_IDCODE = 0b1110u,

	/** @brief BYPASS instruction
	IEEE 1149.1 Standard compliant mandatory instruction.
	It connects BYPASS_DR single bit shiht register between TDI and TDO pins.
	*/
	TAP_instruction_BYPASS = 0b1111,
};
typedef enum TAP_IR_e TAP_IR_e;

/// TAP registers size constants
enum {
	/**  @brief IR registers code size - 4 bits
	*/
	TAP_length_of_IR = 4u,
	TAP_length_of_RO_32 = 32u,
	/// mandatory 32 bits, if supported
	TAP_length_of_IDCODE = TAP_length_of_RO_32,
	TAP_length_of_DBG_ID = TAP_length_of_RO_32,
	TAP_length_of_BLD_ID = TAP_length_of_RO_32,
	TAP_length_of_DBG_STATUS = TAP_length_of_RO_32,
	TAP_length_of_DAP_CTRL_UNIT = 2u,
	TAP_length_of_DAP_CTRL_FGROUP = 2u,
	TAP_length_of_DAP_CTRL = TAP_length_of_DAP_CTRL_UNIT + TAP_length_of_DAP_CTRL_FGROUP,
	TAP_number_of_fields_DAP_CMD = 2u,
	TAP_length_of_DAP_CMD_OPCODE = 4u,
	TAP_length_of_DAP_CMD_OPCODE_EXT = 32u,
	TAP_length_of_DAP_CMD = TAP_length_of_DAP_CMD_OPCODE + TAP_length_of_DAP_CMD_OPCODE_EXT,
	/// mandatory 1 bit shift register
	TAP_length_of_BYPASS = 1,
};
/// @}

STATIC_ASSERT(TAP_length_of_RO_32 == 32);
STATIC_ASSERT(NUM_BYTES_FOR_BITS(TAP_length_of_RO_32) <= sizeof(uint32_t));

enum {
	/// Marker of invalid value of Debug Access Port multiplexer control
	DAP_CTRL_value_INVALID_CODE = 0xFFu,
};

/// type_dbgc_core_dbg_sts_reg_bits_e
/// @see TAP_INSTR_DBG_STATUS
enum {
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

/// e_DAP_opstatus_bits
/// @see TAP_INSTR_DAP_CMD
/// @see TAP_INSTR_DAP_CTRL
enum {
	DAP_opstatus_bit_EXCEPT = 0,
	DAP_opstatus_bit_ERROR = 1,
	DAP_opstatus_bit_LOCK = 2,
	DAP_opstatus_bit_READY = 3,
};

/// e_DAP_OPSTATUS
enum {
	DAP_status_MASK = BIT_MASK(DAP_opstatus_bit_ERROR) | BIT_MASK(DAP_opstatus_bit_LOCK) | BIT_MASK(DAP_opstatus_bit_READY),
	DAP_status_good = BIT_MASK(DAP_opstatus_bit_READY),
};

/// Units IDs
enum type_dbgc_unit_id_e {
	DBGC_unit_id_HART_0 = 0,
	DBGC_unit_id_HART_1 = 1,
	DBGC_unit_id_CORE = 3,
};
typedef enum type_dbgc_unit_id_e type_dbgc_unit_id_e;

/// Functional groups for HART units
///@{
/// type_dbgc_hart_fgroup_e
enum {
	/// @see type_dbgc_regblock_hart_e
	DBGC_functional_group_HART_REGTRANS = 0,

	/// @see type_dbgc_dap_cmd_opcode_dbgcmd_e
	DBGC_functional_group_HART_DBGCMD = 1,
};

/// @see DBGC_functional_group_HART_REGTRANS
enum type_dbgc_regblock_hart_e {
	/// Hart Debug Control Register (HART_DBG_CTRL, HDCR)
	/// @see type_dbgc_hart_dbg_ctrl_reg_bits_e
	DBGC_HART_register_DBG_CTRL = 0,

	/// Hart Debug Status Register (HART_DBG_STS, HDSR)
	/// @see type_dbgc_hart_dbg_sts_reg_bits_e
	DBGC_HART_register_DBG_STS = 1,

	/// Hart Debug Mode Enable Register (HART_DMODE_ENBL, HDMER)
	/// @see type_dbgc_hart_dmode_enbl_reg_bits_e
	DBGC_HART_register_DMODE_ENBL = 2,

	/// Hart Debug Mode Cause Register (HART_DMODE_CAUSE, HDMCR)
	/// @see type_dbgc_hart_dmode_cause_reg_bits_e
	DBGC_HART_register_DMODE_CAUSE = 3,

	/// Debugger emitting instruction register
	/// @see DBGC_DAP_OPCODE_DBGCMD_CORE_EXEC
	DBGC_HART_register_CORE_INSTR = 4,
	/// Debugger data to debug CSR register
	/// @see CSR_DBG_SCRATCH
	DBGC_HART_register_DBG_DATA = 5,
	/// PC value, available in any state
	DBGC_HART_register_PC_SAMPLE = 6,
};
typedef enum type_dbgc_regblock_hart_e type_dbgc_regblock_hart_e;

/// type_dbgc_hart_dbg_ctrl_reg_bits_e
/// @see DBGC_HART_REGS_DBG_CTRL
enum {
	/// HART reset bit
	/// @warning not used now
	DBGC_HART_HDCR_RST_BIT = 0,
	/// Disable pc change for instructions emitting from debugger
	DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT = 6,
};

/// type_dbgc_hart_dbg_sts_reg_bits_e
/// @see DBGC_HART_REGS_DBG_STS
enum {
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
/// type_dbgc_hart_dmode_enbl_reg_bits_e
/// @see DBGC_HART_register_DMODE_ENBL
enum {
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

enum {
	NORMAL_DEBUG_ENABLE_MASK = BIT_MASK(DBGC_HART_HDMER_SW_BRKPT_BIT) | BIT_MASK(DBGC_HART_HDMER_RST_EXIT_BRK_BIT)
};

/// Hart Debug Mode Cause Register (HART_DMODE_CAUSE, HDMCR)
/// type_dbgc_hart_dmode_cause_reg_bits_e
/// @see DBGC_HART_register_DMODE_CAUSE
enum {
	/// Show halt due to SW breakpoint
	DBGC_HART_HDMCR_SW_BRKPT_BIT = 3,
	/// Show halt after single step
	DBGC_HART_HDMCR_SINGLE_STEP_BIT = 28,
	/// Show halt after reset
	DBGC_HART_HDMCR_RST_BREAK_BIT = 30,
	/// Show forced halt from debug controller
	DBGC_HART_HDMCR_ENFORCE_BIT = 31
};

/// type_dbgc_dap_cmd_opcode_dbgcmd_e
/// @see DBGC_functional_group_HART_DBGCMD
enum {
	/// @see type_dbgc_dap_cmd_opcode_dbgctrl_ext_e
	DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL = 0,
	/// Debugger emitting instruction register
	/// @see DBGC_HART_register_CORE_INSTR
	DBGC_DAP_OPCODE_DBGCMD_CORE_EXEC = 1,
	/// @see DBGC_HART_register_DBG_DATA
	DBGC_DAP_OPCODE_DBGCMD_DBGDATA_WR = 2,
	/// @see DBGC_HART_HDSR_LOCK_STKY_BIT
	/// @see DBGC_CORE_CDSR_LOCK_BIT
	/// @see DAP_opstatus_bit_LOCK
	DBGC_DAP_OPCODE_DBGCMD_UNLOCK = 3,
};

/// type_dbgc_dap_cmd_opcode_dbgctrl_ext_e
/// @see DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL
/// @see type_dbgc_dap_cmd_opcode_dbgctrl_ext_s
enum {
	DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_HALT = 0,
	DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_RESUME = 1,
	DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS = 2,
};

///@}

/// Functional groups for CORE units
///@{
/// type_dbgc_core_fgroup_e
enum {
	/// @see type_dbgc_regblock_core_e
	DBGC_FGRP_CORE_REGTRANS = 0,
};

/// @see DBGC_FGRP_CORE_REGTRANS
enum type_dbgc_regblock_core_e {
	DBGC_CORE_REGS_DEBUG_ID = 0,

	/// @see type_dbgc_core_dbg_ctrl_reg_bits_e
	DBGC_CORE_REGS_DBG_CTRL = 1,
	DBGC_CORE_REGS_DBG_STS = 2,
#if 0
	DBGC_CORE_REGS_DBG_CMD = 3,
#endif
};
typedef enum type_dbgc_regblock_core_e type_dbgc_regblock_core_e;

/// Core Debug Control Register (CORE_DBG_CTRL, CDCR)
/// type_dbgc_core_dbg_ctrl_reg_bits_e
/// @see DBGC_CORE_REGS_DBG_CTRL
enum {
	DBGC_CORE_CDCR_HART0_RST_BIT = 0,
	DBGC_CORE_CDCR_HART1_RST_BIT = 8,
	DBGC_CORE_CDCR_RST_BIT = 24,
	DBGC_CORE_CDCR_IRQ_DSBL_BIT = 25,
};
///@}

/// RISC-V GP registers id
enum {
#if 0
	RISCV_regnum_ZERO = 0,
#endif
	RISCV_regnum_PC = 32,
	RISCV_regnum_FP_first = 33,
	RISCV_regnum_FP_last = 64,
	RISCV_regnum_CSR_first = 65,
	RISCV_rtegnum_CSR_last = 4160,

	number_of_regs_X = RISCV_regnum_PC,
	number_of_regs_GP = number_of_regs_X + 1u,
	number_of_regs_F = RISCV_regnum_FP_last - RISCV_regnum_FP_first + 1,
	number_of_regs_GDB = RISCV_rtegnum_CSR_last + 1,
};

typedef uint32_t rv_instruction32_type;
typedef uint16_t rv_instruction16_type;

enum
{
	CSR_mstatus = 0x300
};

struct sc_riscv32__Arch_constants
{
	/// Don't make irscan if IR is the same
	bool use_ir_select_cache;

	/// Don't write DAP_CONTROL if it is the same
	bool use_dap_control_cache;

	/// Verify value of DAP_CONTROL after write
	bool use_verify_dap_control;
	bool use_check_pc_unchanged;

	/// Verify values of HART REGTRANS after write
	bool use_verify_hart_regtrans_write;

	/// Verify values of CORE REGTRANS after write
	bool use_verify_core_regtrans_write;
	bool use_pc_advmt_dsbl_bit;
	bool use_queuing_for_dr_scans;
	/// expected TAP controller IDCODE
	uint32_t expected_idcode;
	/// expected TAP controller IDCODE mask
	uint32_t expected_idcode_mask;

	/** Lowest required DBG_ID
	Required and provided masked values should be equal.
	*/
	uint32_t expected_dbg_id;
	/// Syntacore Debug controller CSR
	csr_num_type debug_scratch_CSR;
	rv_instruction32_type(*opcode_FMV_D_2X)(reg_num_type rd_fp, reg_num_type rs_hi, reg_num_type rs_lo);
	rv_instruction32_type(*opcode_FMV_2X_D)(reg_num_type rd_hi, reg_num_type rd_lo, reg_num_type rs1_fp);
	bool(*is_RVC_enable)(target* const p_target);
	uint32_t(*get_mstatus_FS)(uint32_t const mstatus);
	uint32_t(*get_isa_CSR)(target* const p_target);
	void(*virt_to_phis)(target* p_target, uint32_t address, uint32_t* p_physical, uint32_t* p_bound, bool const instruction_space);
};
typedef struct sc_riscv32__Arch_constants sc_riscv32__Arch_constants;

struct sc_riscv32__Arch {
	/// stored sub-operations error_code
	error_code error_code;
	/// Cache DAP_CTRL
	uint8_t last_DAP_ctrl;
	sc_riscv32__Arch_constants const* constants;
};
typedef struct sc_riscv32__Arch sc_riscv32__Arch;

error_code
sc_riscv32__poll(target* const p_target);

error_code
sc_riscv32__arch_state(target* const p_target);

error_code
sc_riscv32__halt(target* const p_target);

error_code
sc_riscv32__resume(target* const p_target, int const current, uint32_t const address, int const handle_breakpoints, int const debug_execution);

error_code
sc_riscv32__step(target* const p_target, int const current, uint32_t const address, int const handle_breakpoints);

error_code
sc_riscv32__assert_reset(target* const p_target);

error_code
sc_riscv32__deassert_reset(target* const p_target);

error_code
sc_riscv32__soft_reset_halt(target* const p_target);

error_code
sc_riscv32__get_gdb_reg_list(target* const p_target, reg** reg_list[], int* const reg_list_size, target_register_class const reg_class);

error_code
sc_riscv32__read_memory(target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t* buffer);

error_code
sc_riscv32__write_memory(target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t const* buffer);

error_code
sc_riscv32__add_breakpoint(target* const p_target, breakpoint* const p_breakpoint);

error_code
sc_riscv32__remove_breakpoint(target* const p_target, breakpoint* const p_breakpoint);

error_code
sc_riscv32__target_create(target* const p_target, Jim_Interp* interp);

error_code
sc_riscv32__examine(target* const p_target);

void
sc_riscv32__deinit_target(target* const p_target);

error_code
sc_riscv32__read_phys_memory(target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t* buffer);

error_code
sc_riscv32__write_phys_memory(target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t const* buffer);

uint32_t
sc_rv32__csr_get_value(target* const p_target, uint32_t const csr_number);

error_code
error_code__get(target const* const p_target);

error_code
error_code__get_and_clear(target const* const p_target);

error_code
error_code__update(target const* const p_target, error_code const a_error_code);

void
sc_rv32_update_status(target* const p_target);

rv_instruction32_type
RISCV_opcode_INSTR_R_TYPE(uint8_t func7, reg_num_type rs2, reg_num_type rs1, uint8_t func3, reg_num_type rd, uint8_t opcode);

void
sc_rv32_init_regs_cache(target* const p_target);

error_code
sc_riscv32__virt2phys(target* p_target, uint32_t address, uint32_t* p_physical);

#endif  // TARGET_SC_RV32_COMMON_H_
