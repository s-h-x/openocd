/**	@file

	Syntacore RISC-V targets common methods

	@copyright Syntacore 2016, 2017
	@author sps (https://github.com/aka-sps)
*/
#include "sc_rv32_common.h"

#include "jtag/jtag.h"
#include "helper/log.h"

#include <assert.h>
#include <limits.h>
#include <memory.h>

/// Queuing operations before force jtag operations
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

/// Number of array 'arr' elements
#define ARRAY_LEN(arr) (sizeof (arr) / sizeof (arr)[0])

/// @return true if values bits except lower 'LEN' are zero
#define IS_VALID_UNSIGNED_FIELD(FLD,LEN) ((FLD & ~LOW_BITS_MASK(LEN)) == 0)
#define NORMALIZE_INT_FIELD(FLD, SIGN_BIT, ZEROS) ( ( ( ( -( ( (FLD) >> (SIGN_BIT) ) & LOW_BITS_MASK(1) ) ) << (SIGN_BIT) ) | (FLD) ) & ~LOW_BITS_MASK(ZEROS) )
#define IS_VALID_SIGNED_IMMEDIATE_FIELD(FLD, SIGN_BIT, LOW_ZEROS) ( (FLD) == NORMALIZE_INT_FIELD((FLD), (SIGN_BIT), (LOW_ZEROS)) )

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

/** Number of octets required for 'num_bits' bits
@param [in] num_bits number of bits
@return number of bytes for 'num_bits' \f$\lceil {\frac{num\_bits}{CHAR\_BIT}} \rceil\f$.
*/
#define NUM_BYTES_FOR_BITS(num_bits) ( ( (size_t)(num_bits) + (CHAR_BIT - 1) ) / CHAR_BIT )

/// @return expression of TYPE with 'first_bit':'last_bit' bits set
#define MAKE_TYPE_FIELD(TYPE, bits, first_bit, last_bit)     ((((TYPE)(bits)) & LOW_BITS_MASK((last_bit) + 1u - (first_bit))) << (first_bit))

/// @return bit-field value
#define EXTRACT_FIELD(bits, first_bit, last_bit) (((bits) >> (first_bit)) & LOW_BITS_MASK((last_bit) + 1u - (first_bit)))

static_assert(CHAR_BIT == 8, "Unsupported char size");

typedef uint16_t rv_instruction16_type;
typedef enum target_debug_reason target_debug_reason;
typedef enum target_state target_state;
typedef int16_t riscv_short_signed_type;
typedef int32_t riscv_signed_type;
typedef struct reg_feature reg_feature;
typedef struct reg_data_type reg_data_type;
typedef struct scan_field scan_field;
typedef struct Jim_Interp Jim_Interp;
typedef struct reg_data_type_union reg_data_type_union;
typedef struct reg_data_type_union_field reg_data_type_union_field;
typedef struct reg_arch_type reg_arch_type;
typedef struct reg_cache reg_cache;

/**	@defgroup SC_RV32_TAPC_IR Syntacore TAP Controller Instructions
@ingroup SC_RV32
*/
/// @{

/**	@brief TAP Controller Instructions ID
ir_scan selects one of instruction registers IO to TDI/TDO
*/
enum TAP_IR_e
{
	/**	@brief DBG_ID Register Read

	It connects DBG_ID_DR register between TDI and TDO pins.
	Contains debug facilities version implemented in the given processor DBGC subsystem.
	*/
	TAP_instruction_DBG_ID = 0b0011u,

	/**	@brief BLD_ID Register Read

	Connects BLD_ID_DR between TDI and TDO pins.
	It identifies an entire processor system RTL build revision.
	*/
	TAP_instruction_BLD_ID = 0b0100u,

	/**	@brief DBG_STATUS Register Read
	Connects DBG_STATUS_DR register providing general status information about debug operations and core state.
	*/
	TAP_instruction_DBG_STATUS = 0b0101u,

	/**	@brief DAP_CTRL Register Write

	Connects DAP_CTRL_DR register is the control port of the upper level multiplexer
	for DAP_CMD register.
	*/
	TAP_instruction_DAP_CTRL = 0b0110u,

	/**	@brief DAP_CTRL Register Read

	Connects DAP_CTRL_RD_DR register allowing to read
	current DAP Control Context (DAPCC) from the DAP_CONTEXT register.
	*/
	TAP_instruction_DAP_CTRL_RD = 0b0111u,

	/**	@brief Debug Access Port Command (DAP Command)

	Provide multiplexed IO to inner DC registers.

	Multiplexing is controlled by DAP_CTRL
	Connects DAP_CMD_DR register.
	*/
	TAP_instruction_DAP_CMD = 0b1000u,

	/**	@brief SYS_CTRL Register Access

	Connects SYS_CTRL_DR register,
	used to control state of the Processor Subsystem Reset net,
	and to get its current status.
	*/
	TAP_instruction_SYS_CTRL = 0b1001u,

	/**	@brief MTAP_SWITCH Register Access

	Connects MTAP_SWITCH_DR register, used to control state
	of the Master TAP Switch Control output, and to
	get its current status.
	*/
	TAP_instruction_MTAP_SWITCH = 0b1101u,

	/**	@brief IDCODE Register Read
	Conventional recommended Device Identification instruction compliant with IEEE 1149.1 Standard.
	It connects IDCODE_DR register between TDI and TDO pins.()
	*/
	TAP_instruction_IDCODE = 0b1110u,

	/**	@brief BYPASS instruction
	IEEE 1149.1 Standard compliant mandatory instruction.
	It connects BYPASS_DR single bit shiht register between TDI and TDO pins.
	*/
	TAP_instruction_BYPASS = 0b1111,
};
typedef enum TAP_IR_e TAP_IR_e;

/// TAP registers size constants
enum
{
	/**  @brief IR registers code size - 4 bits
	*/
	TAP_length_of_IR = 4u,
	/// @brief size of most IR registers
	TAP_length_of_RO_32 = 32u,
	/// @brief IDCODE, if supported, 32 bits mandatory
	TAP_length_of_IDCODE = TAP_length_of_RO_32,
	TAP_length_of_DBG_ID = TAP_length_of_RO_32,
	TAP_length_of_BLD_ID = TAP_length_of_RO_32,
	TAP_length_of_DBG_STATUS = TAP_length_of_RO_32,

	/// @brief DAP_CTRL Units selector field size
	TAP_length_of_DAP_CTRL_unit_field = 2u,
	/// @brief Functional group selector field size
	TAP_length_of_DAP_CTRL_fgroup_field = 2u,
	TAP_number_of_fields_DAP_CMD = 2u,
	/// @brief Total size of DAP_CTRL instruction
	TAP_length_of_DAP_CTRL = TAP_length_of_DAP_CTRL_unit_field + TAP_length_of_DAP_CTRL_fgroup_field,

	TAP_length_of_DAP_CMD_OPCODE = 4u,
	TAP_length_of_DAP_CMD_OPCODE_EXT = 32u,
	TAP_length_of_DAP_CMD = TAP_length_of_DAP_CMD_OPCODE + TAP_length_of_DAP_CMD_OPCODE_EXT,

	/// mandatory 1 bit shift register
	TAP_length_of_BYPASS = 1,
};
/// @}

/**	@brief DBG_STATUS bits
	
	Upper-level status register bits

	@see TAP_instruction_DBG_STATUS
*/
enum
{
	DBG_STATUS_bit_HART0_DMODE = BIT_MASK(0),
	DBG_STATUS_bit_HART0_Rst = BIT_MASK(1),
	DBG_STATUS_bit_HART0_Rst_Stky = BIT_MASK(2),
	DBG_STATUS_bit_HART0_Err = BIT_MASK(3),
	DBG_STATUS_bit_HART0_Err_Stky = BIT_MASK(4),
	DBG_STATUS_bit_Err = BIT_MASK(16),
	DBG_STATUS_bit_Err_Stky = BIT_MASK(17),
	DBG_STATUS_bit_Err_HwCore = BIT_MASK(18),
	DBG_STATUS_bit_Err_FsmBusy = BIT_MASK(19),
	DBG_STATUS_bit_Err_DAP_Opcode = BIT_MASK(20),
	DBG_STATUS_bit_Rst = BIT_MASK(28),
	DBG_STATUS_bit_Rst_Stky = BIT_MASK(29),
	DBG_STATUS_bit_Lock = BIT_MASK(30),
	DBG_STATUS_bit_Ready = BIT_MASK(31),
};


/**	@brief Status bits of previous operation
*/
enum
{
	/// @brief Exception detected
	DAP_opstatus_bit_EXCEPT = BIT_MASK(0),
	DAP_opstatus_bit_ERROR = BIT_MASK(1),
	/// @brief Debug controller locked after previous operation
	DAP_opstatus_bit_LOCK = BIT_MASK(2),
	/// @brief Ready for next operation
	DAP_opstatus_bit_READY = BIT_MASK(3),

	DAP_status_MASK = DAP_opstatus_bit_ERROR | DAP_opstatus_bit_LOCK | DAP_opstatus_bit_READY,
	DAP_status_good = DAP_opstatus_bit_READY,
};

/**	@brief DAP_CTRL fields enumerations

	DAP_CTRL transaction select unit and functional group and return previous operation status
*/
/// @{
/// Units IDs
enum type_dbgc_unit_id_e
{
	DBGC_unit_id_HART_0 = 0,
	DBGC_unit_id_HART_1 = 1,
	DBGC_unit_id_CORE = 3,
};
typedef enum type_dbgc_unit_id_e type_dbgc_unit_id_e;

/// Functional groups for HART units
enum
{
	/// @see HART_REGTRANS_indexes
	DBGC_functional_group_HART_REGTRANS = 0b00,
	DBGC_functional_group_HART_DBGCMD = 0b01,
	DBGC_functional_group_HART_CSR_CAP = 0b10,
};

/// Functional groups for CORE units
enum
{
	/// @see CORE_REGTRANS_indexes
	DBGC_functional_group_CORE_REGTRANS = 0,
};

/// @}

/**	@brief HART[0] Debug Registers indexes
	@pre HART unit, REGTRANS functional group registers 
	@see DBGC_functional_group_HART_REGTRANS
*/
enum HART_REGTRANS_indexes
{
	/// @brief Hart Debug Control Register HART_DBG_CTRL (HDCR)
	HART_DBG_CTRL_index = 0,

	/// @brief Hart Debug Status Register HART_DBG_STS (HDSR)
	HART_DBG_STS_index = 1,

	/// @brief Hart Debug Mode Enable Register HART_DMODE_ENBL (HDMER)
	HART_DMODE_ENBL_index = 2,

	/// @brief Hart Debug Mode Cause Register HART_DMODE_CAUSE (HDMCR)
	HART_DMODE_CAUSE_index = 3,

	/// @brief Hart Debug Core Instruction Register HART_CORE_INSTR (HDCIR)
	HART_CORE_INSTR_index = 4,

	/**	@brief Hart Debug Data HART_DBG_DATA (HDDR) register.

		Corresponds to the DBG_SCRATCH core’s CSR.

		@see Debug scratch CSR
	*/
	HART_DBG_DATA_index = 5,

	/**	@brief Hart Program Counter (PC) HART_PC_SAMPLE (HPCSR) register.

		Reflects current hart PC value.
	*/
	HART_PC_SAMPLE_index = 6,
};
typedef enum HART_REGTRANS_indexes HART_REGTRANS_indexes;

/// @see DBGC_functional_group_HART_DBGCMD
enum HART_DBGCMD_indexes
{
	/**	@brief DBG_CTRL (Debug Control Operation)
	
	Command for Debug Subsystem state change
	(includes an important option for transition between Run-Mode and Debug-Mode)
	*/
	DBG_CTRL_index = 0x0,

	/**	@brief CORE_EXEC (Debug Core Instruction Execution)

	Command carries out execution of a RISC-V instruction
	resided in the DBGC’s HART_CORE_INSTR register,
	on a corresponding core’s HART.

	@see HART_CORE_INSTR_index
	*/
	CORE_EXEC_index = 0x1,

	/**	@brief DBGDATA_WR (Debug Data Register Write)

		Command writes 32-bit data into the HART_DBG_DATA register.

		@see HART_DBG_DATA_index
	*/
	DBGDATA_WR_index = 0x2,

	/**	@brief UNLOCK
		
		Command unlocks DAP which has been previously locked due to error(s) during preceding operations.

		@see HART_DBG_STS_bit_Lock_Stky
		@see DBG_STATUS_bit_Lock
		@see DAP_opstatus_bit_LOCK
	*/
	UNLOCK_index = 0x3,
};
typedef enum HART_DBGCMD_indexes HART_DBGCMD_indexes;

enum HART_CSR_CAP_indexes
{
	/// @brief Hart MVENDORID (HMVENDORID) Register
	HART_MVENDORID_index = 0b000,

	/// @brief Hart MARCHID (HMARCHID) Register
	HART_MARCHID_index = 0b001,

	/// @brief Hart MIMPID (HMIMPID) Register
	HART_MIMPID_index = 0b010,

	/// @brief Hart MHARTID (HMHARTID) Register
	HART_MHARTID_index = 0b011,

	/// @brief Hart MISA Register
	HART_MISA_index = 0b100,
};
typedef enum HART_CSR_CAP_indexes HART_CSR_CAP_indexes;

/// @see DBGC_functional_group_CORE_REGTRANS
enum CORE_REGTRANS_indexes
{
	/**	@brief Core Debug ID (CDID) Register
	*/
	CORE_DEBUG_ID_index = 0,

	/**	@brief Core Debug Control (CDCR) Register
	*/
	CORE_DBG_CTRL_index = 1,

	/**	@brief Core Debug Status (CDSR) Register
	*/
	CORE_DBG_STS_index = 2,
};
typedef enum CORE_REGTRANS_indexes CORE_REGTRANS_indexes;

/**	@brief HART_DBG_CTRL bits
	@see HART_DBG_CTRL_index
*/
enum HART_DBG_CTRL_bits
{
	/// HART reset
	/// @warning not used now
	HART_DBG_CTRL_bit_Rst = BIT_MASK(0),

	/// Hart PC Advancement Disable
	HART_DBG_CTRL_bit_PC_Advmt_Dsbl = BIT_MASK(6),
};

/**	@brief HART_DBG_STS bits
	@see HART_DBG_STS_index
*/
enum HART_DBG_STS_bits
{
	/// @brief Hart Debug Mode Status
	HART_DBG_STS_bit_DMODE = BIT_MASK(0),

	/// @brief Hart Reset Status
	HART_DBG_STS_bit_Rst = BIT_MASK(1),

	/// @brief Hart Reset Sticky Status
	HART_DBG_STS_bit_Rst_Stky = BIT_MASK(2),

	/// @brief Hart Exception Status
	HART_DBG_STS_bit_Except = BIT_MASK(3),

	/// @brief Hart Error Summary Status
	HART_DBG_STS_bit_Err = BIT_MASK(16),

	/// @brief Hart HW Error Status
	HART_DBG_STS_bit_Err_HwThread = BIT_MASK(17),

	/// @brief Hart DAP OpCode Error Status
	HART_DBG_STS_bit_Err_DAP_OpCode = BIT_MASK(18),

	/// @brief Hart Debug Command NACK Error Status
	HART_DBG_STS_bit_Err_DbgCmd_NACK = BIT_MASK(19),

	/// @brief Hart Illegal Debug Context Error Status
	HART_DBG_STS_bit_Err_Illeg_Contxt = BIT_MASK(20),

	/// @brief Hart Unexpected Reset Error Status
	HART_DBG_STS_bit_Err_Unexp_Rst = BIT_MASK(21),

	/// @brief Hart Debug Operation Time-out Error Status
	HART_DBG_STS_bit_Err_Timeout = BIT_MASK(22),

	/// @brief Hart DAP Lock Sticky Status
	HART_DBG_STS_bit_Lock_Stky = BIT_MASK(31)
};

/**	@brief HART_DMODE_ENBL (HDMER) bits
	@see HART_DMODE_ENBL_index
*/
enum HART_DMODE_ENBL_bits
{
	/// @brief Hart Breakpoint Exception DMODE Redirection Enable
	HART_DMODE_ENBL_bit_Brkpt = BIT_MASK(3),

	/// @brief Hart Single Step DMODE Redirection Enable
	HART_DMODE_ENBL_bit_SStep = BIT_MASK(28),

	/// @brief Hart Reset Exit DMODE Redirection Enable
	HART_DMODE_ENBL_bit_Rst_Exit = BIT_MASK(30),

	HART_DMODE_ENBL_bits_Normal = HART_DMODE_ENBL_bit_Brkpt | HART_DMODE_ENBL_bit_Rst_Exit
};

/** HART_DMODE_CAUSE (HDMCR) bits
	@see HART_DMODE_CAUSE_index
*/
enum HART_DMODE_CAUSE_bits
{
	/// @brief Hart Breakpoint Exception
	HART_DMODE_CAUSE_bit_Brkpt = BIT_MASK(3),

	/// @brief Hart HW Breakpoint
	HART_DMODE_CAUSE_bit_Hw_Brkpt = BIT_MASK(27),

	/// @brief Hart Single Step
	HART_DMODE_CAUSE_bit_SStep = BIT_MASK(28),

	/// @brief Hart Reset Entrance Break
	HART_DMODE_CAUSE_bit_Rst_Entr = BIT_MASK(29),

	/// @brief Hart Reset Exit Break
	HART_DMODE_CAUSE_bit_Rst_Exit = BIT_MASK(30),

	/// @brief Hart Debug Mode Enforcement
	HART_DMODE_CAUSE_bit_Enforce = BIT_MASK(31)
};

/// @see DBG_CTRL_index
enum DBG_CTRL_bits
{
	/**	@brief Halt
	
	Transits a corresponding hart from Run-Mode to Debug-Mode (halts the hart)

	Write only
	*/
	DBG_CTRL_bit_Halt = BIT_MASK(0),

	/**	@brief Resume
	
	Transits a corresponding hart from DebugMode to Run-Mode (restarts the hart).
	*/
	DBG_CTRL_bit_Resume = BIT_MASK(1),

	/**	@brief Sticky Clear.
	
	Clears sticky status bits for corresponding HART.
	*/
	DBG_CTRL_bit_Sticky_Clr = BIT_MASK(2),
};

/// Core Debug Control Register (CORE_DBG_CTRL, CDCR) bits
/// @see CORE_DBG_CTRL_index
enum CORE_DBG_CTRL_bits
{
	/**	@brief Hart[0] Reset.
	
		Reserved for future use
	*/
	CORE_DBG_CTRL_bit_HART0_Rst = BIT_MASK(0),

	/**	@brief Core Reset
	*/
	CORE_DBG_CTRL_bit_Rst = BIT_MASK(24),

	/**	@brief Core IRQ Disable
	*/
	CORE_DBG_CTRL_bit_Irq_Dsbl = BIT_MASK(25),
};

/// RISC-V GP registers values
enum
{
	/// PC register number for gdb
	RISCV_regnum_PC = 32,

	/// First FPU register number for gdb
	RISCV_regnum_FP_first = 33,
	/// Last FPU register number for gdb
	RISCV_regnum_FP_last = 64,
	/// First CSR register number for gdb
	RISCV_regnum_CSR_first = 65,
	/// Last CSR register number for gdb
	RISCV_rtegnum_CSR_last = 4160,

	/// Number of X registers
	number_of_regs_X = RISCV_regnum_PC,
	/// Number of GP registers (X registers + PC)
	number_of_regs_GP = number_of_regs_X + 1u,

	/// Number of FPU registers
	number_of_regs_F = RISCV_regnum_FP_last - RISCV_regnum_FP_first + 1,

	/// Total number of registers for gdb
	number_of_regs_GDB = RISCV_rtegnum_CSR_last + 1,
};

static uint8_t const obj_DAP_opstatus_GOOD = DAP_status_good;
static uint8_t const obj_DAP_status_MASK = DAP_status_MASK;
static reg_data_type GP_reg_data_type = {.type = REG_TYPE_INT32,};
static reg_feature feature_riscv_org = {
	.name = "org.gnu.gdb.riscv.cpu",
};
static char const def_GP_regs_name[] = "general";

/**	@brief Check IDCODE for selected target
*/
static bool
sc_rv32__is_IDCODE_valid(target* const p_target, uint32_t const IDCODE)
{
	sc_riscv32__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);
	sc_riscv32__Arch_constants const* const p_const = p_arch->constants;
	assert(p_const);
	return (p_const->expected_idcode & p_const->expected_idcode_mask) == (IDCODE & p_const->expected_idcode_mask);
}

/**	@brief Check Debug controller version for compatibility
*/
static bool
sc_rv32__is_DBG_ID_valid(target* const p_target, uint32_t const DBG_ID)
{
	sc_riscv32__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);
	sc_riscv32__Arch_constants const* const p_const = p_arch->constants;
	assert(p_const);
	/// Mask of DBG_ID version.
	// Required value should be equal to provided version.
	static uint32_t const DBG_ID_major_version_mask = 0xFFFFFF00u;
	// Mask of DBG_ID subversion.
	// Required value should be greater or equal to provided subversion.
	static uint32_t const DBG_ID_subversion_mask = 0x000000FFu;
	return
		(DBG_ID & (DBG_ID_major_version_mask)) == (p_const->expected_dbg_id & (DBG_ID_major_version_mask)) &&
		(DBG_ID & (DBG_ID_subversion_mask)) >= (p_const->expected_dbg_id & (DBG_ID_subversion_mask));
}

/// Error code handling
///@{

error_code
sc_error_code__get(target const* const p_target)
{
	assert(p_target);
	sc_riscv32__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);
	return p_arch->error_code;
}

/** @brief Forced store operation code into target context.
*/
static inline error_code
sc_error_code__set(target const* const p_target, error_code const a_error_code)
{
	assert(p_target);
	{
		sc_riscv32__Arch* restrict const p_arch = p_target->arch_info;
		assert(p_arch);
		p_arch->error_code = a_error_code;
	}
	return a_error_code;
}

error_code
sc_error_code__get_and_clear(target const* const p_target)
{
	error_code const result = sc_error_code__get(p_target);
	sc_error_code__set(p_target, ERROR_OK);
	return result;
}

error_code
sc_error_code__update(target const* const p_target, error_code const a_error_code)
{
	error_code const old_code = sc_error_code__get(p_target);

	if (ERROR_OK != old_code || ERROR_OK == a_error_code) {
		return old_code;
	} else {
		LOG_DEBUG("Set new error code: %d", a_error_code);
		return sc_error_code__set(p_target, a_error_code);
	}
}

/**	@brief Update error context

	If passed code is not equal to ERROR_OK - replace it in the target context.
*/
static inline error_code
sc_error_code__prepend(target const* const p_target, error_code const older_err_code)
{
	if (ERROR_OK == older_err_code) {
		return sc_error_code__get(p_target);
	} else {
		LOG_DEBUG("Reset error code to previous state: %d", older_err_code);
		return sc_error_code__set(p_target, older_err_code);
	}
}
/// @}

/** RISC-V instruction encoding.
*/
/// @{
static rv_instruction32_type
RISCV_opcode_INSTR_R_TYPE(uint8_t func7, reg_num_type rs2, reg_num_type rs1, uint8_t func3, reg_num_type rd, uint8_t opcode)
{
	CHECK_OPCODE(opcode);
	CHECK_FUNC3(func3);
	CHECK_FUNC7(func7);
	CHECK_REG(rs2);
	CHECK_REG(rs1);
	CHECK_REG(rd);
	return
		MAKE_TYPE_FIELD(rv_instruction32_type, func7, 25, 31) |
		MAKE_TYPE_FIELD(rv_instruction32_type, rs2, 20, 24) |
		MAKE_TYPE_FIELD(rv_instruction32_type, rs1, 15, 19) |
		MAKE_TYPE_FIELD(rv_instruction32_type, func3, 12, 14) |
		MAKE_TYPE_FIELD(rv_instruction32_type, rd, 7, 11) |
		MAKE_TYPE_FIELD(rv_instruction32_type, opcode, 0, 6);
}

static rv_instruction32_type
RISCV_opcode_INSTR_I_TYPE(riscv_short_signed_type imm_11_00, reg_num_type rs1, uint8_t func3, reg_num_type rd, uint8_t opcode)
{
	CHECK_OPCODE(opcode);
	CHECK_REG(rd);
	CHECK_REG(rs1);
	CHECK_FUNC3(func3);
	CHECK_IMM_11_00(imm_11_00);
	return
		MAKE_TYPE_FIELD(rv_instruction32_type, EXTRACT_FIELD(imm_11_00, 0, 11), 20, 31) |
		MAKE_TYPE_FIELD(rv_instruction32_type, rs1, 15, 19) |
		MAKE_TYPE_FIELD(rv_instruction32_type, func3, 12, 14) |
		MAKE_TYPE_FIELD(rv_instruction32_type, rd, 7, 11) |
		MAKE_TYPE_FIELD(rv_instruction32_type, opcode, 0, 6);
}

static rv_instruction32_type
RISCV_opcode_INSTR_S_TYPE(riscv_short_signed_type imm_11_00, reg_num_type rs2, reg_num_type rs1, unsigned func3, uint8_t opcode)
{
	CHECK_OPCODE(opcode);
	CHECK_REG(rs2);
	CHECK_REG(rs1);
	CHECK_FUNC3(func3);
	CHECK_IMM_11_00(imm_11_00);
	return
		MAKE_TYPE_FIELD(rv_instruction32_type, EXTRACT_FIELD(imm_11_00, 5, 11), 25, 31) |
		MAKE_TYPE_FIELD(rv_instruction32_type, rs2, 20, 24) |
		MAKE_TYPE_FIELD(rv_instruction32_type, rs1, 15, 19) |
		MAKE_TYPE_FIELD(rv_instruction32_type, func3, 12, 14) |
		MAKE_TYPE_FIELD(rv_instruction32_type, EXTRACT_FIELD(imm_11_00, 0, 4), 7, 11) |
		MAKE_TYPE_FIELD(rv_instruction32_type, opcode, 0, 6);
}

static rv_instruction32_type
RISCV_opcode_INSTR_UJ_TYPE(riscv_signed_type imm_20_01, reg_num_type rd, uint8_t opcode)
{
	CHECK_OPCODE(opcode);
	CHECK_REG(rd);
	CHECK_IMM_20_01(imm_20_01);
	return
		MAKE_TYPE_FIELD(rv_instruction32_type, EXTRACT_FIELD(imm_20_01, 20, 20), 31, 31) |
		MAKE_TYPE_FIELD(rv_instruction32_type, EXTRACT_FIELD(imm_20_01, 1, 10), 21, 30) |
		MAKE_TYPE_FIELD(rv_instruction32_type, EXTRACT_FIELD(imm_20_01, 11, 11), 20, 20) |
		MAKE_TYPE_FIELD(rv_instruction32_type, EXTRACT_FIELD(imm_20_01, 12, 19), 12, 19) |
		MAKE_TYPE_FIELD(rv_instruction32_type, rd, 7, 11) |
		MAKE_TYPE_FIELD(rv_instruction32_type, opcode, 0, 6);
}

static rv_instruction32_type
RISCV_opcode_FMV_X_S(reg_num_type rd, reg_num_type rs1_fp)
{
	return RISCV_opcode_INSTR_R_TYPE(0x70u, 0u, rs1_fp, 0u, rd, 0x53u);
}

static rv_instruction32_type
RISCV_opcode_FMV_S_X(reg_num_type rd_fp, reg_num_type rs1)
{
	return RISCV_opcode_INSTR_R_TYPE(0x78u, 0u, rs1, 0u, rd_fp, 0x53u);
}

static rv_instruction32_type
RISCV_opcode_LB(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RISCV_opcode_INSTR_I_TYPE(imm, rs1, 0u, rd, 0x03u);
}

static rv_instruction32_type
RISCV_opcode_LH(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RISCV_opcode_INSTR_I_TYPE(imm, rs1, 1u, rd, 0x03u);
}

static rv_instruction32_type
RISCV_opcode_LW(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RISCV_opcode_INSTR_I_TYPE(imm, rs1, 2u, rd, 0x03u);
}

static rv_instruction32_type
RISCV_opcode_ADDI(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RISCV_opcode_INSTR_I_TYPE(imm, rs1, 0u, rd, 0x13u);
}

static rv_instruction32_type
RISCV_opcode_JALR(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RISCV_opcode_INSTR_I_TYPE(imm, rs1, 0u, rd, 0x67u);
}

static rv_instruction32_type
RISCV_opcode_CSRRW(reg_num_type rd, csr_num_type csr, reg_num_type rs1)
{
	return RISCV_opcode_INSTR_I_TYPE(NORMALIZE_INT_FIELD(csr, 11, 0), rs1, 1u, rd, 0x73u);
}

static rv_instruction32_type
RISCV_opcode_CSRRS(reg_num_type rd, csr_num_type csr, reg_num_type rs1)
{
	return RISCV_opcode_INSTR_I_TYPE(NORMALIZE_INT_FIELD(csr, 11, 0), rs1, 2u, rd, 0x73u);
}

static rv_instruction32_type
RISCV_opcode_EBREAK(void)
{
	return RISCV_opcode_INSTR_I_TYPE(1, 0u, 0u, 0u, 0x73u);
}

static rv_instruction32_type
RISCV_opcode_SB(reg_num_type rs_data, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RISCV_opcode_INSTR_S_TYPE(imm, rs_data, rs1, 0u, 0x23);
}

static rv_instruction32_type
RISCV_opcode_SH(reg_num_type rs, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RISCV_opcode_INSTR_S_TYPE(imm, rs, rs1, 1u, 0x23);
}

static rv_instruction32_type
RISCV_opcode_SW(reg_num_type rs, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RISCV_opcode_INSTR_S_TYPE(imm, rs, rs1, 2u, 0x23);
}

static rv_instruction32_type
RISCV_opcode_JAL(reg_num_type rd, riscv_signed_type imm_20_01)
{
	return RISCV_opcode_INSTR_UJ_TYPE(imm_20_01, rd, 0x6Fu);
}

static rv_instruction32_type
RISCV_opcode_CSRW(unsigned csr, reg_num_type rs1)
{
	return RISCV_opcode_CSRRW(0, csr, rs1);
}

static rv_instruction32_type
RISCV_opcode_CSRR(reg_num_type rd, csr_num_type csr)
{
	return RISCV_opcode_CSRRS(rd, csr, 0);
}

static rv_instruction16_type
RISCV_opcode_C_EBREAK(void)
{
	return 0x9002u;
}

/// SC custom instruction copy FPU double precision register value to two 32-bits GP registers (based on S-extension opcode)
rv_instruction32_type
sc_RISCV_opcode_S_FMV_2X_D(reg_num_type rd_hi, reg_num_type rd_lo, reg_num_type rs1_fp)
{
	return RISCV_opcode_INSTR_R_TYPE(0x70u, rd_hi, rs1_fp, 0u, rd_lo, 0x53u);
}

/// SC custom instruction to combine from two GP registers values to FPU double precision register value (based on S-extension opcode)
rv_instruction32_type
sc_RISCV_opcode_S_FMV_D_2X(reg_num_type rd_fp, reg_num_type rs_hi, reg_num_type rs_lo)
{
	return RISCV_opcode_INSTR_R_TYPE(0x78u, rs_hi, rs_lo, 0u, rd_fp, 0x53u);
}

/// SC custom instruction copy FPU double precision register value to two 32-bits GP registers (based on D-extension opcode)
rv_instruction32_type
sc_RISCV_opcode_D_FMV_2X_D(reg_num_type rd_hi, reg_num_type rd_lo, reg_num_type rs1_fp)
{
	return RISCV_opcode_INSTR_R_TYPE(0x71u, rd_hi, rs1_fp, 0u, rd_lo, 0x53u);
}

/// SC custom instruction to combine from two GP registers values to FPU double precision register value (based on D-extension opcode)
rv_instruction32_type
sc_RISCV_opcode_D_FMV_D_2X(reg_num_type rd_fp, reg_num_type rs_hi, reg_num_type rs_lo)
{
	return RISCV_opcode_INSTR_R_TYPE(0x79u, rs_hi, rs_lo, 0u, rd_fp, 0x53u);
}

#if 0
static rv_instruction32_type
RISCV_opcode_INSTR_SB_TYPE(riscv_short_signed_type imm_01_12, reg_num_type rs2, reg_num_type rs1, unsigned func3, uint8_t opcode)
{
	CHECK_OPCODE(opcode);
	CHECK_FUNC3(func3);
	CHECK_REG(rs1);
	CHECK_REG(rs2);
	CHECK_IMM_12_01(imm_01_12);
	return
		MAKE_TYPE_FIELD(rv_instruction32_type, EXTRACT_FIELD(imm_01_12, 12, 12), 31, 31) |
		MAKE_TYPE_FIELD(rv_instruction32_type, EXTRACT_FIELD(imm_01_12, 5, 10), 25, 30) |
		MAKE_TYPE_FIELD(rv_instruction32_type, rs2, 20, 24) |
		MAKE_TYPE_FIELD(rv_instruction32_type, rs1, 15, 19) |
		MAKE_TYPE_FIELD(rv_instruction32_type, func3, 12, 14) |
		MAKE_TYPE_FIELD(rv_instruction32_type, EXTRACT_FIELD(imm_01_12, 1, 4), 8, 11) |
		MAKE_TYPE_FIELD(rv_instruction32_type, EXTRACT_FIELD(imm_01_12, 11, 11), 7, 7) |
		MAKE_TYPE_FIELD(rv_instruction32_type, opcode, 0, 6);
}

static rv_instruction32_type
RISCV_opcode_INSTR_U_TYPE(riscv_signed_type imm_31_12, reg_num_type rd, uint8_t opcode)
{
	CHECK_OPCODE(opcode);
	CHECK_REG(rd);
#define CHECK_IMM_31_12(imm) assert(IS_VALID_SIGNED_IMMEDIATE_FIELD(imm, 31, 12));
	CHECK_IMM_31_12(imm_31_12);
#undef CHECK_IMM_31_12
	return
		MAKE_TYPE_FIELD(rv_instruction32_type, EXTRACT_FIELD(imm_31_12, 12, 31), 12, 31) |
		MAKE_TYPE_FIELD(rv_instruction32_type, rd, 7, 11) |
		MAKE_TYPE_FIELD(rv_instruction32_type, opcode, 0, 6);
}

static rv_instruction32_type
RISCV_opcode_ADD(reg_num_type rd, reg_num_type rs1, reg_num_type rs2)
{
	return RISCV_opcode_INSTR_R_TYPE(0x00u, rs2, rs1, 0u, rd, 0x33u);
}

static rv_instruction32_type
RISCV_opcode_LBU(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RISCV_opcode_INSTR_I_TYPE(imm, rs1, 4u, rd, 0x03u);
}

static rv_instruction32_type
RISCV_opcode_LHU(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RISCV_opcode_INSTR_I_TYPE(imm, rs1, 5u, rd, 0x03u);
}

static rv_instruction32_type
RISCV_opcode_CSRRC(reg_num_type rd, csr_num_type csr, reg_num_type rs1)
{
	return RISCV_opcode_INSTR_I_TYPE(NORMALIZE_INT_FIELD(csr, 11, 0), rs1, 3u, rd, 0x73u);
}

static rv_instruction32_type
RISCV_opcode_CSRRWI(reg_num_type rd, csr_num_type csr, uint8_t zimm)
{
	return RISCV_opcode_INSTR_I_TYPE(NORMALIZE_INT_FIELD(csr, 11, 0), zimm, 5u, rd, 0x73u);
}

static rv_instruction32_type
RISCV_opcode_CSRRSI(reg_num_type rd, csr_num_type csr, uint8_t zimm)
{
	return RISCV_opcode_INSTR_I_TYPE(NORMALIZE_INT_FIELD(csr, 11, 0), zimm, 6u, rd, 0x73u);
}

static rv_instruction32_type
RISCV_opcode_CSRRCI(reg_num_type rd, csr_num_type csr, uint8_t zimm)
{
	return RISCV_opcode_INSTR_I_TYPE(NORMALIZE_INT_FIELD(csr, 11, 0), zimm, 7u, rd, 0x73u);
}

static rv_instruction32_type
RISCV_opcode_AUIPC(reg_num_type rd, riscv_signed_type imm)
{
	return RISCV_opcode_INSTR_U_TYPE(imm, rd, 0x17u);
}

static rv_instruction32_type
RISCV_opcode_NOP(void)
{
	return RISCV_opcode_ADDI(0, 0, 0u);
}

#endif

/// @}

/**	@brief Always perform scan to write instruction register
*/
static inline void
IR_select_force(target const* const p_target, TAP_IR_e const new_instr)
{
	assert(p_target);
	assert(p_target->tap);
	assert(p_target->tap->ir_length == TAP_length_of_IR);
	uint8_t out_buffer[NUM_BYTES_FOR_BITS(TAP_length_of_IR)] = {};
	buf_set_u32(out_buffer, 0, TAP_length_of_IR, new_instr);
	scan_field field = {.num_bits = p_target->tap->ir_length,.out_value = out_buffer};
	jtag_add_ir_scan(p_target->tap, &field, TAP_IDLE);
	LOG_DEBUG("irscan %s %d", p_target->cmd_name, new_instr);
}

/**	@brief Cached version of instruction register selection
*/
static void
IR_select(target const* const p_target, TAP_IR_e const new_instr)
{
	assert(p_target);
	sc_riscv32__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);

	if (p_arch->constants->use_ir_select_cache) {
		assert(p_target->tap);
		assert(p_target->tap->ir_length == TAP_length_of_IR);

		/// Skip IR scan if IR is the same
		if (buf_get_u32(p_target->tap->cur_instr, 0u, p_target->tap->ir_length) == new_instr) {
			return;
		}
	}

	/// or call real IR scan
	IR_select_force(p_target, new_instr);
}

/**	@brief Common method to retrieve data of read-only 32-bits TAP IR

Method store and update error_code, but ignore previous errors and
allows to repair error state of debug controller.
*/
static uint32_t
read_only_32_bits_regs(target const* const p_target, TAP_IR_e ir)
{
	assert(p_target);
	uint32_t result = 0xBADC0DE0u;

	/// Low-level method save but ignore previous errors.
	error_code const old_err_code = sc_error_code__get_and_clear(p_target);
	/// Error state can be updated in IR_select
	IR_select(p_target, ir);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		uint8_t result_buffer[NUM_BYTES_FOR_BITS(TAP_length_of_RO_32)] = {};
		scan_field const field = {
			.num_bits = TAP_length_of_RO_32,
			.in_value = result_buffer,
		};
		assert(p_target->tap);
		jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);

		// enforce jtag_execute_queue() to obtain result
		sc_error_code__update(p_target, jtag_execute_queue());
		LOG_DEBUG("drscan %s %d 0 ; # %08X", p_target->cmd_name, field.num_bits, buf_get_u32(result_buffer, 0, TAP_length_of_RO_32));

		if (ERROR_OK != sc_error_code__get(p_target)) {
			/// Error state can be updated in DR scan
			LOG_ERROR("JTAG error %d", sc_error_code__get(p_target));
		} else {
			result = buf_get_u32(result_buffer, 0, TAP_length_of_RO_32);
		}
	}

	sc_error_code__prepend(p_target, old_err_code);
	return result;
}

/// @brief IDCODE get accessors
static inline uint32_t
sc_rv32_IDCODE_get(target const* const p_target)
{
	return read_only_32_bits_regs(p_target, TAP_instruction_IDCODE);
}

/// @brief DBG_ID get accessors
static inline uint32_t
sc_rv32_DBG_ID_get(target const* const p_target)
{
	return read_only_32_bits_regs(p_target, TAP_instruction_DBG_ID);
}

/// @brief BLD_ID get accessors
static inline uint32_t
sc_rv32_BLD_ID_get(target const* const p_target)
{
	return read_only_32_bits_regs(p_target, TAP_instruction_BLD_ID);
}

/// @brief DBG_STATUS get accessors
static uint32_t
sc_rv32_DBG_STATUS_get(target const* const p_target)
{
	uint32_t const result = read_only_32_bits_regs(p_target, TAP_instruction_DBG_STATUS);

	/** Sensitivity mask is union of bits
	- HART0_ERR
	- ERR
	- ERR_HWCORE
	- ERR_FSMBUSY
	- LOCK
	- READY
	*/
	static uint32_t const result_mask =
		DBG_STATUS_bit_HART0_Err |
		DBG_STATUS_bit_Err |
		DBG_STATUS_bit_Err_HwCore |
		DBG_STATUS_bit_Err_FsmBusy |
		DBG_STATUS_bit_Err_DAP_Opcode |
		DBG_STATUS_bit_Lock |
		DBG_STATUS_bit_Ready;

	/** In normal state only READY bit is allowed */
	static uint32_t const good_result = DBG_STATUS_bit_Ready;

	if ((result & result_mask) != (good_result & result_mask)) {
		LOG_WARNING("DBG_STATUS == 0x%08X", result);
	}

	return result;
}

/**	@brief Upper level DAP_CTRL multiplexer control port cache update method.
*/
static inline void
update_DAP_CTRL_cache(target const* const p_target, uint8_t const set_dap_unit_group)
{
	assert(p_target);
	sc_riscv32__Arch* restrict const p_arch = p_target->arch_info;
	assert(p_arch);
	p_arch->last_DAP_ctrl = set_dap_unit_group;
}

/**	@brief Upper level DAP_CTRL multiplexer control port cache invalidation.
*/
static inline void
invalidate_DAP_CTR_cache(target const* const p_target)
{
	update_DAP_CTRL_cache(p_target, DAP_CTRL_value_INVALID_CODE);
}

/**	@brief Forced variant of upper level multiplexer control port DAP_CTRL set method (don't use cache state)
*/
static inline void
DAP_CTRL_REG_set_force(target const* const p_target, uint8_t const set_dap_unit_group)
{
	assert(p_target);
	/// Save but ignore previous error state.
	error_code const older_err_code = sc_error_code__get_and_clear(p_target);
	IR_select(p_target, TAP_instruction_DAP_CTRL);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		/// Invalidate cache of DAP_CTRL
		invalidate_DAP_CTR_cache(p_target);

		/// Prepare clear status bits
		uint8_t status = 0;
		static_assert(NUM_BYTES_FOR_BITS(TAP_length_of_DAP_CTRL) == sizeof status, "Bad size");
		static_assert(NUM_BYTES_FOR_BITS(TAP_length_of_DAP_CTRL) == sizeof set_dap_unit_group, "Bad size");
		/// Prepare DR scan
		scan_field const field = {
			/// for 4 bits
			.num_bits = TAP_length_of_DAP_CTRL,
			/// send DAP unit/group
			.out_value = &set_dap_unit_group,
			/// and receive old status
			.in_value = &status,
			/// with status check that only READY bit is active
			.check_value = &obj_DAP_opstatus_GOOD,
			/// sensitivity to mask ERROR | LOCK | READY bits
			.check_mask = &obj_DAP_status_MASK,
		};

		jtag_add_dr_scan_check(p_target->tap, 1, &field, TAP_IDLE);

		/// Enforce jtag_execute_queue() to get status.
		error_code const jtag_status = jtag_execute_queue();
		LOG_DEBUG("drscan %s %d 0x%1X ; # %1X", p_target->cmd_name, field.num_bits, set_dap_unit_group, status);

		if (ERROR_OK == jtag_status) {
			/// Update DAP_CTRL cache if no errors
			update_DAP_CTRL_cache(p_target, set_dap_unit_group);
		} else {
			/// or report current error.
			uint32_t const dbg_status = sc_rv32_DBG_STATUS_get(p_target);
			static uint32_t const dbg_status_check_value = DBG_STATUS_bit_Ready;
			static uint32_t const dbg_status_mask =
				DBG_STATUS_bit_Lock |
				DBG_STATUS_bit_Ready;

			if ((dbg_status & dbg_status_mask) != (dbg_status_check_value & dbg_status_mask)) {
				LOG_ERROR("JTAG error %d, operation_status=0x%1X, dbg_status=0x%08X", jtag_status, (uint32_t)status, dbg_status);
				sc_error_code__update(p_target, jtag_status);
			} else {
				LOG_WARNING("JTAG error %d, operation_status=0x%1X, but dbg_status=0x%08X", jtag_status, (uint32_t)status, dbg_status);
			}
		}
	}

	/// Restore previous error (if it was)
	sc_error_code__prepend(p_target, older_err_code);
}

/// @brief Verify unit/group selection.
static inline void
DAP_CTRL_REG_verify(target const* const p_target, uint8_t const set_dap_unit_group)
{
	/// First of all invalidate DAP_CTR cache.
	invalidate_DAP_CTR_cache(p_target);
	/// Ignore bu save previous error state.
	error_code const old_err_code = sc_error_code__get_and_clear(p_target);
	/// Select read function of DAP_CTRL.
	IR_select(p_target, TAP_instruction_DAP_CTRL_RD);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		/// Prepare DR scan to read actual value of DAP_CTR.
		uint8_t get_dap_unit_group = 0;
		uint8_t set_dap_unit_group_mask = 0x0Fu;
		static_assert(NUM_BYTES_FOR_BITS(TAP_length_of_DAP_CTRL) == sizeof get_dap_unit_group, "Bad size");
		scan_field const field = {
			.num_bits = TAP_length_of_DAP_CTRL,
			.in_value = &get_dap_unit_group,
			/// with checking for expected value.
			.check_value = &set_dap_unit_group,
			.check_mask = &set_dap_unit_group_mask,
		};
		jtag_add_dr_scan_check(p_target->tap, 1, &field, TAP_IDLE);

		/// Enforce jtag_execute_queue() to get get_dap_unit_group.
		sc_error_code__update(p_target, jtag_execute_queue());
		LOG_DEBUG("drscan %s %d 0x%1X ; # %1X", p_target->cmd_name, field.num_bits, 0, get_dap_unit_group);

		if (ERROR_OK == sc_error_code__get(p_target)) {
			/// If no errors
			if (get_dap_unit_group == set_dap_unit_group) {
				/// and read value is equal to expected then update DAP_CTRL cache,
				update_DAP_CTRL_cache(p_target, get_dap_unit_group);
			} else {
				/// else report error.
				LOG_ERROR("Unit/Group verification error: set 0x%1X, but get 0x%1X!", set_dap_unit_group, get_dap_unit_group);
				sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
			}
		}
	}

	/// Restore previous error (if it was)
	sc_error_code__prepend(p_target, old_err_code);
}

/**	@brief Common DR scan of DAP_CMD multiplexed port.

@par[in]  p_target current target pointer
@par[in]  DAP_OPCODE 4-bits: R/W bit and 3-bits next level multiplexer selector
@par[in]  DAP_OPCODE_EXT 32-bits payload
@par[out] p_result pointer to place for input payload
*/
static void
sc_rv32_DAP_CMD_scan(target const* const p_target, uint8_t const DAP_OPCODE, uint32_t const DAP_OPCODE_EXT, uint32_t* p_result)
{
	/// Ignore but save previous error state.
	error_code const old_err_code = sc_error_code__get_and_clear(p_target);
	/// Select DAP_CMD IR.
	IR_select(p_target, TAP_instruction_DAP_CMD);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		/// Prepare DR scan for two fields.

		/// Copy output payload to buffer.
		uint8_t dap_opcode_ext[NUM_BYTES_FOR_BITS(TAP_length_of_DAP_CMD_OPCODE_EXT)] = {};
		buf_set_u32(dap_opcode_ext, 0, TAP_length_of_DAP_CMD_OPCODE_EXT, DAP_OPCODE_EXT);

		/// Reserve and init buffer for payload input.
		uint8_t dbg_data[NUM_BYTES_FOR_BITS(TAP_length_of_DAP_CMD_OPCODE_EXT)] = {};

		/// Prepare operation status buffer.
		uint8_t DAP_OPSTATUS = 0;
		static_assert(NUM_BYTES_FOR_BITS(TAP_length_of_DAP_CMD_OPCODE) == sizeof DAP_OPSTATUS, "Bad size");
		scan_field const fields[2] = {
			{.num_bits = TAP_length_of_DAP_CMD_OPCODE_EXT,.out_value = dap_opcode_ext,.in_value = dbg_data},
			/// Pass DAP_OPCODE bits. Check receiving DAP_OPSTATUS good/error bits.
			{.num_bits = TAP_length_of_DAP_CMD_OPCODE,.out_value = &DAP_OPCODE,.in_value = &DAP_OPSTATUS,.check_value = &obj_DAP_opstatus_GOOD,.check_mask = &obj_DAP_status_MASK,},
		};

		/// Add DR scan to queue.
		assert(p_target->tap);
		jtag_add_dr_scan_check(p_target->tap, ARRAY_LEN(fields), fields, TAP_IDLE);

		/// Enforse jtag_execute_queue() to get values
		sc_error_code__update(p_target, jtag_execute_queue());
		/// Log DR scan debug information.
		LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X ; # %08X %1X", p_target->cmd_name,
				  fields[0].num_bits, DAP_OPCODE_EXT,
				  fields[1].num_bits, DAP_OPCODE,
				  buf_get_u32(dbg_data, 0, TAP_length_of_DAP_CMD_OPCODE_EXT), DAP_OPSTATUS);

		if (ERROR_OK == sc_error_code__get(p_target)) {
			if ((DAP_OPSTATUS & DAP_status_MASK) != DAP_status_good) {
				/// Check and report if error was detected.
				LOG_ERROR("DAP_OPSTATUS == 0x%1X", (uint32_t)DAP_OPSTATUS);
				sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
			} else if (p_result) {
				/// or copy result bits to output if 'p_result' pointer is not NULL.
				*p_result = buf_get_u32(dbg_data, 0, TAP_length_of_DAP_CMD_OPCODE_EXT);
			}
		}
	}

	/// Restore previous error (if it was)
	sc_error_code__prepend(p_target, old_err_code);
}

/**	@brief Try to unlock debug controller.

@warning Clear previous error_code and set ERROR_TARGET_FAILURE if unlock was unsuccessful
@return lock context
*/
static inline uint32_t
sc_rv32_DC__unlock(target const* const p_target)
{
	LOG_WARNING("========= Try to unlock ==============");

	assert(p_target);
	assert(p_target->tap);
	assert(p_target->tap->ir_length == TAP_length_of_IR);

	{
		/// Enqueue selection of DAP_CTRL IR.
		static uint8_t const ir_out_buffer_DAP_CTRL[NUM_BYTES_FOR_BITS(TAP_length_of_IR)] = {TAP_instruction_DAP_CTRL};
		/// @todo jtag_add_ir_scan need non-const scan_field
		static scan_field ir_field_DAP_CTRL = {.num_bits = TAP_length_of_IR,.out_value = ir_out_buffer_DAP_CTRL};
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_instruction_DAP_CTRL);
		jtag_add_ir_scan(p_target->tap, &ir_field_DAP_CTRL, TAP_IDLE);
	}

	{
		/// Enqueue write DAP_CTRL to select HART_DBGCMD group and HART_0 unit.
		static uint8_t const set_dap_unit_group = MAKE_TYPE_FIELD(uint8_t, DBGC_unit_id_HART_0, 2, 3) | MAKE_TYPE_FIELD(uint8_t, DBGC_functional_group_HART_DBGCMD, 0, 1);
		static_assert(NUM_BYTES_FOR_BITS(TAP_length_of_DAP_CTRL) == sizeof set_dap_unit_group, "Bad size");
		static scan_field const dr_field_DAP_CTRL = {.num_bits = TAP_length_of_DAP_CTRL,.out_value = &set_dap_unit_group};
		invalidate_DAP_CTR_cache(p_target);
		LOG_DEBUG("drscan %s %d 0x%1X", p_target->cmd_name,
				  dr_field_DAP_CTRL.num_bits, set_dap_unit_group);
		jtag_add_dr_scan(p_target->tap, 1, &dr_field_DAP_CTRL, TAP_IDLE);
	}

	{
		/// Enqueue selection of DAP_CMD IR.
		static uint8_t const ir_out_buffer_DAP_CMD[NUM_BYTES_FOR_BITS(TAP_length_of_IR)] = {TAP_instruction_DAP_CMD};
		static scan_field ir_field_DAP_CMD = {.num_bits = TAP_length_of_IR,.out_value = ir_out_buffer_DAP_CMD};
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_instruction_DAP_CMD);
		jtag_add_ir_scan(p_target->tap, &ir_field_DAP_CMD, TAP_IDLE);
	}

	uint8_t lock_context_buf[4] = {};
	{
		/// Enqueue DR scan transaction to UNLOCK register, write zeros, receive lock context.
		static uint8_t const dap_opcode_ext_UNLOCK[4] = {0, 0, 0, 0};
		static uint8_t const dap_opcode_UNLOCK = UNLOCK_index;
		scan_field const dr_fields_UNLOCK[2] = {
			{.num_bits = TAP_length_of_DAP_CMD_OPCODE_EXT,.out_value = dap_opcode_ext_UNLOCK,.in_value = lock_context_buf},
			{.num_bits = TAP_length_of_DAP_CMD_OPCODE,.out_value = &dap_opcode_UNLOCK}
		};

		for (int i = 0; i < 2; ++i) {
			LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X", p_target->cmd_name,
					  dr_fields_UNLOCK[0].num_bits, buf_get_u32(dap_opcode_ext_UNLOCK, 0, TAP_length_of_DAP_CMD_OPCODE_EXT),
					  dr_fields_UNLOCK[1].num_bits, dap_opcode_UNLOCK);
			jtag_add_dr_scan(p_target->tap, ARRAY_LEN(dr_fields_UNLOCK), dr_fields_UNLOCK, TAP_IDLE);
		}
	}

	{
		/// Enqueue DBG_STATUS IR selection.
		static uint8_t const ir_out_buffer_DBG_STATUS[NUM_BYTES_FOR_BITS(TAP_length_of_IR)] = {TAP_instruction_DBG_STATUS};
		static scan_field ir_field_DBG_STATUS = {.num_bits = TAP_length_of_IR,.out_value = ir_out_buffer_DBG_STATUS};
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_instruction_DBG_STATUS);
		jtag_add_ir_scan(p_target->tap, &ir_field_DBG_STATUS, TAP_IDLE);
	}

	{
		/// Enqueue get DBG_STATUS.
		uint8_t status_buffer[4] = {};
		scan_field const dr_field_DBG_STATUS = {.num_bits = TAP_length_of_DBG_STATUS,.out_value = status_buffer,.in_value = status_buffer};
		jtag_add_dr_scan(p_target->tap, 1, &dr_field_DBG_STATUS, TAP_IDLE);

		/// Enforse jtag_execute_queue() to get values.
		error_code const status = sc_error_code__update(p_target, jtag_execute_queue());
		uint32_t const scan_status = buf_get_u32(status_buffer, 0, TAP_length_of_DAP_CMD_OPCODE_EXT);
		bool const ok =
			ERROR_OK == status &&
			/// and if status is OK, check that LOCK bit is zero now.
			0 == (scan_status & DBG_STATUS_bit_Lock);
		LOG_DEBUG("drscan %s %d 0x%08X ; # 0x%08X", p_target->cmd_name,
				  dr_field_DBG_STATUS.num_bits, 0,
				  scan_status);
		uint32_t const lock_context = buf_get_u32(lock_context_buf, 0, TAP_length_of_DAP_CMD_OPCODE_EXT);
		LOG_DEBUG("%s context=0x%08X, status=0x%08X", ok ? "Unlock succsessful!" : "Unlock unsuccsessful!",
				  lock_context,
				  scan_status);
		return lock_context;
	}
}

/**	@brief Try to clear HART0 errors.
*/
static void
sc_rv32_HART0_clear_sticky(target* const p_target)
{
	LOG_DEBUG("========= Try to clear HART0 errors ============");
	assert(p_target);
	sc_riscv32__Arch* const p_arch = p_target->arch_info;
	assert(p_arch);
	assert(p_target->tap);
	assert(p_target->tap->ir_length == TAP_length_of_IR);

	{
		/// Enqueue irscan to select DAP_CTRL IR.
		uint8_t ir_dap_ctrl_out_buffer[NUM_BYTES_FOR_BITS(TAP_length_of_IR)] = {};
		buf_set_u32(ir_dap_ctrl_out_buffer, 0, TAP_length_of_IR, TAP_instruction_DAP_CTRL);
		scan_field ir_dap_ctrl_field = {.num_bits = p_target->tap->ir_length,.out_value = ir_dap_ctrl_out_buffer};
		jtag_add_ir_scan(p_target->tap, &ir_dap_ctrl_field, TAP_IDLE);
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_instruction_DAP_CTRL);
	}

	{
		/// Invalidate DAP_CTRL cache value.
		invalidate_DAP_CTR_cache(p_target);
		/// Enqueue DR scan to set DAP_CTRL HART_DBGCMD group and HART_0 unit (0x1u)
		uint8_t const set_dap_unit_group = MAKE_TYPE_FIELD(uint8_t, DBGC_unit_id_HART_0, 2, 3) | MAKE_TYPE_FIELD(uint8_t, DBGC_functional_group_HART_DBGCMD, 0, 1);
		scan_field const dr_dap_ctrl_field = {.num_bits = TAP_length_of_DAP_CTRL,.out_value = &set_dap_unit_group};
		LOG_DEBUG("drscan %s 0x%1X 0x%08X ; ", p_target->cmd_name, dr_dap_ctrl_field.num_bits, set_dap_unit_group);
		jtag_add_dr_scan(p_target->tap, 1, &dr_dap_ctrl_field, TAP_IDLE);
	}

	{
		/// Enqueue irscan to select DAP_CMD IR.
		uint8_t ir_dap_cmd_out_buffer[NUM_BYTES_FOR_BITS(TAP_length_of_IR)] = {};
		buf_set_u32(ir_dap_cmd_out_buffer, 0, TAP_length_of_IR, TAP_instruction_DAP_CMD);
		scan_field ir_dap_cmd_field = {.num_bits = p_target->tap->ir_length,.out_value = ir_dap_cmd_out_buffer};
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_instruction_DAP_CMD);
		jtag_add_ir_scan(p_target->tap, &ir_dap_cmd_field, TAP_IDLE);
	}

	{
		/// @todo describe
		uint8_t dap_opcode_ext[NUM_BYTES_FOR_BITS(TAP_length_of_DAP_CMD_OPCODE_EXT)];
		uint32_t const opcode_ext = DBG_CTRL_bit_Sticky_Clr;
		buf_set_u32(dap_opcode_ext, 0, TAP_length_of_DAP_CMD_OPCODE_EXT, opcode_ext);
		uint8_t const dap_opcode = DBG_CTRL_index;
		scan_field const fields[2] = {
			{.num_bits = TAP_length_of_DAP_CMD_OPCODE_EXT,.out_value = dap_opcode_ext},
			{.num_bits = TAP_length_of_DAP_CMD_OPCODE,.out_value = &dap_opcode}
		};
		LOG_DEBUG("drscan %s 0x%1X 0x%08X 0x%1X 0x%08X ; ", p_target->cmd_name,
				  fields[0].num_bits, opcode_ext,
				  fields[1].num_bits, DBG_CTRL_index);
		jtag_add_dr_scan(p_target->tap, ARRAY_LEN(fields), fields, TAP_IDLE);
	}
}

/**	@brief Make code for REGTRANS transaction

To prepare type of access to 2-nd level multiplexed REGTRANS register

@par[in] write read (false) or write (true) transaction type code
@par[in] index multiplexed register index
*/
static inline uint8_t
REGTRANS_scan_type(bool const write, uint8_t const index)
{
	assert((index & LOW_BITS_MASK(3)) == index);
	return
		MAKE_TYPE_FIELD(uint8_t, !!write, 3, 3) |
		MAKE_TYPE_FIELD(uint8_t, index, 0, 2);
}

/**	@brief Try to clear errors bit of core
@todo Describe details
*/
static void
sc_rv32_CORE_clear_errors(target* const p_target)
{
	LOG_DEBUG("========= Try to clear core errors ============");
	assert(p_target);
	sc_riscv32__Arch* const p_arch = p_target->arch_info;
	assert(p_arch);
	assert(p_target->tap);
	assert(p_target->tap->ir_length == TAP_length_of_IR);

	{
		uint8_t ir_dap_ctrl_out_buffer[NUM_BYTES_FOR_BITS(TAP_length_of_IR)] = {};
		buf_set_u32(ir_dap_ctrl_out_buffer, 0, TAP_length_of_IR, TAP_instruction_DAP_CTRL);
		scan_field ir_dap_ctrl_field = {.num_bits = p_target->tap->ir_length,.out_value = ir_dap_ctrl_out_buffer};
		jtag_add_ir_scan(p_target->tap, &ir_dap_ctrl_field, TAP_IDLE);
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_instruction_DAP_CTRL);
	}

	{
		/// set invalid cache value
		invalidate_DAP_CTR_cache(p_target);

		uint8_t const set_dap_unit_group = (DBGC_unit_id_CORE << TAP_length_of_DAP_CTRL_fgroup_field) | DBGC_functional_group_CORE_REGTRANS;
		scan_field const field = {.num_bits = TAP_length_of_DAP_CTRL,.out_value = &set_dap_unit_group};
		LOG_DEBUG("drscan %s %d 0x%1X", p_target->cmd_name, field.num_bits, set_dap_unit_group);
		jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);
	}

	{
		uint8_t ir_dap_cmd_out_buffer[NUM_BYTES_FOR_BITS(TAP_length_of_IR)] = {};
		buf_set_u32(ir_dap_cmd_out_buffer, 0, TAP_length_of_IR, TAP_instruction_DAP_CMD);
		scan_field ir_dap_cmd_field = {.num_bits = p_target->tap->ir_length,.out_value = ir_dap_cmd_out_buffer};
		jtag_add_ir_scan(p_target->tap, &ir_dap_cmd_field, TAP_IDLE);
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_instruction_DAP_CMD);
	}

	{
		uint8_t dap_opcode_ext[(TAP_length_of_DAP_CMD_OPCODE_EXT + CHAR_BIT - 1) / CHAR_BIT];
		buf_set_u32(dap_opcode_ext, 0, TAP_length_of_DAP_CMD_OPCODE_EXT, 0xFFFFFFFF);
		uint8_t const dap_opcode = REGTRANS_scan_type(true, CORE_DBG_STS_index);
		scan_field const fields[2] = {
			{.num_bits = TAP_length_of_DAP_CMD_OPCODE_EXT,.out_value = dap_opcode_ext},
			{.num_bits = TAP_length_of_DAP_CMD_OPCODE,.out_value = &dap_opcode},
		};
		LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X", p_target->cmd_name,
				  fields[0].num_bits,
				  buf_get_u32(dap_opcode_ext, 0, 32),
				  fields[1].num_bits, dap_opcode);
		jtag_add_dr_scan(p_target->tap, ARRAY_LEN(fields), fields, TAP_IDLE);
	}
}

/**	@brief Common method to set DAP_CTRL upper level multiplexer control register

@par[in] p_target pointer to this target
@par[in] dap_unit multiplexed unit of multiplexed group
@par[in] dap_group multiplexed group

@todo Describe details
*/
static void
sc_rv32_DAP_CTRL_REG_set(target const* const p_target, type_dbgc_unit_id_e const dap_unit, uint8_t const dap_group)
{
	assert(p_target);
	bool const match_HART_0 = DBGC_unit_id_HART_0 == dap_unit && 0 == p_target->coreid;
	bool const match_HART_1 = DBGC_unit_id_HART_1 == dap_unit && 1 == p_target->coreid;
	bool const HART_unit = match_HART_0 || match_HART_1;
	bool const HART_group =
		DBGC_functional_group_HART_REGTRANS == dap_group ||
		DBGC_functional_group_HART_DBGCMD == dap_group ||
		DBGC_functional_group_HART_CSR_CAP == dap_group;
	bool const CORE_unit = DBGC_unit_id_CORE == dap_unit;
	bool const CORE_group = DBGC_functional_group_CORE_REGTRANS == dap_group;
	assert((HART_unit && HART_group) ^ (CORE_unit && CORE_group));

	uint8_t const set_dap_unit_group =
		MAKE_TYPE_FIELD(uint8_t,
						MAKE_TYPE_FIELD(uint8_t, dap_unit, TAP_length_of_DAP_CTRL_fgroup_field, TAP_length_of_DAP_CTRL_fgroup_field + TAP_length_of_DAP_CTRL_unit_field - 1) |
						MAKE_TYPE_FIELD(uint8_t, dap_group, 0, TAP_length_of_DAP_CTRL_fgroup_field - 1),
						0,
						TAP_length_of_DAP_CTRL_fgroup_field + TAP_length_of_DAP_CTRL_unit_field - 1);

	sc_riscv32__Arch* const p_arch = p_target->arch_info;
	assert(p_arch);

	if (p_arch->constants->use_dap_control_cache) {
		/// If use_dap_control_cache enabled and last unit/group is the same, then return without actions.
		if (p_arch->last_DAP_ctrl == set_dap_unit_group) {
			return;
		}

		LOG_DEBUG("DAP_CTRL_REG of %s reset to 0x%1X", p_target->cmd_name, set_dap_unit_group);
	}

	invalidate_DAP_CTR_cache(p_target);
	error_code const old_err_code = sc_error_code__get_and_clear(p_target);
	DAP_CTRL_REG_set_force(p_target, set_dap_unit_group);

	if (p_arch->constants->use_verify_dap_control) {
		sc_error_code__get_and_clear(p_target);
		DAP_CTRL_REG_verify(p_target, set_dap_unit_group);

		if (ERROR_OK == sc_error_code__get(p_target)) {
			update_DAP_CTRL_cache(p_target, set_dap_unit_group);
		}
	}

	sc_error_code__prepend(p_target, old_err_code);
}

/**	@brief Common REGTRANS write operation

@par[inout] p_target pointer to this target
@par[in] func_unit functional unit
@par[in] func_group functional group
@par[in] index REGTRANS register index in func_unit/func_group
*/
static void
REGTRANS_write(target const* const p_target, type_dbgc_unit_id_e func_unit, uint8_t const func_group, uint8_t const index, uint32_t const data)
{
	/// Set upper level multiplexer to access unit/group.
	sc_rv32_DAP_CTRL_REG_set(p_target, func_unit, func_group);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		/// And (if no errors) perform single DR scan with 4-bits field write bit/index bits and 32-bits data.
		sc_rv32_DAP_CMD_scan(p_target, REGTRANS_scan_type(true, index), data, NULL);
	} else {
		/// Or else, report error and do not scan.
		/// @todo LOG_ERROR?
		LOG_WARNING("DAP_CTRL_REG_set error");
		return;
	}
}

/**	@brief Common REGTRANS read operation

@par[inout] p_target pointer to this target
@par[in] func_unit functional unit
@par[in] func_group functional group
@par[in] index REGTRANS register index in func_unit/func_group
*/
static uint32_t
REGTRANS_read(target const* const p_target, type_dbgc_unit_id_e const func_unit, uint8_t const func_group, uint8_t const index)
{
	/// Set upper level multiplexer to access unit/group.
	sc_rv32_DAP_CTRL_REG_set(p_target, func_unit, func_group);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		/// If no errors then perform first DR scan with 4-bits field register index bits (set to bit 'write' to zero)
		/// and dummy (zero) 32-bits data.
		sc_rv32_DAP_CMD_scan(p_target, REGTRANS_scan_type(false, index), 0, NULL);

		/** Input data captured before TDI/TDO shifting and TAP register will update only after shifting,
		so first transaction can read only old register data, but not requested. Only second DR scan can get requested data.

		@bug Bad DC design. Context-dependent read transaction, in common case, transmit at least 36-bits of waste.
		*/
		if (ERROR_OK == sc_error_code__get(p_target)) {
			/** If errors not detected, perform second same DR scan to really get requested data */
			uint32_t result;
			sc_rv32_DAP_CMD_scan(p_target, REGTRANS_scan_type(false, index), 0, &result);
			return result;
		}
	}

	/// Else (if any error detected) log error and return fake data.
	LOG_ERROR("REGTRANS read error");
	return 0xBADC0DE3u;
}

/**	@brief HART REGTRANS read operation

@par[inout] p_target pointer to this target
@par[in] index REGTRANS register index in DBGC_unit_id_HART_0/HART_REGTRANS
*/
static inline uint32_t
sc_rv32_HART_REGTRANS_read(target const* const p_target, HART_REGTRANS_indexes const index)
{
	/// @todo remove unused DBGC_unit_id_HART_1
	type_dbgc_unit_id_e const unit = p_target->coreid == 0 ? DBGC_unit_id_HART_0 : DBGC_unit_id_HART_1;
	return REGTRANS_read(p_target, unit, DBGC_functional_group_HART_REGTRANS, index);
}

/**	@brief HART HART_CSR_CAP read operation

@par[inout] p_target pointer to this target
@par[in] index REGTRANS register index in DBGC_unit_id_HART_0/HART_REGTRANS
*/
static inline uint32_t
sc_rv32_HART_CSR_CAP_read(target const* const p_target, HART_CSR_CAP_indexes const index)
{
	/// @todo remove unused DBGC_unit_id_HART_1
	type_dbgc_unit_id_e const unit = p_target->coreid == 0 ? DBGC_unit_id_HART_0 : DBGC_unit_id_HART_1;
	return REGTRANS_read(p_target, unit, DBGC_functional_group_HART_CSR_CAP, index);
}

static uint32_t
get_ISA(target* const p_target)
{
	return sc_rv32_HART_CSR_CAP_read(p_target, HART_MISA_index);
}

/**	@brief HART REGTRANS write operation

@par[inout] p_target pointer to this target
@par[in] index REGTRANS register index in DBGC_unit_id_HART_0/HART_REGTRANS
@par[in] set_value 32-bits data
*/
static inline void
HART_REGTRANS_write(target const* const p_target, HART_REGTRANS_indexes const index, uint32_t const set_value)
{
	type_dbgc_unit_id_e const unit = p_target->coreid == 0 ? DBGC_unit_id_HART_0 : DBGC_unit_id_HART_1;
	REGTRANS_write(p_target, unit, DBGC_functional_group_HART_REGTRANS, index, set_value);
}

/**	@brief HART REGTRANS write operation with re-read writing value.

@par[inout] p_target pointer to this target
@par[in] index REGTRANS register index in DBGC_unit_id_HART_0/HART_REGTRANS
@par[in] set_value 32-bits data
*/
static void
sc_rv32_HART_REGTRANS_write_and_check(target const* const p_target, HART_REGTRANS_indexes const index, uint32_t const set_value)
{
	assert(p_target);
	HART_REGTRANS_write(p_target, index, set_value);

	if (sc_error_code__get(p_target) == ERROR_OK) {
		sc_riscv32__Arch const* const p_arch = p_target->arch_info;
		assert(p_arch);

		if (p_arch->constants->use_verify_hart_regtrans_write) {
			uint32_t const get_value = sc_rv32_HART_REGTRANS_read(p_target, index);

			if (get_value != set_value) {
				LOG_ERROR("Write HART_REGTRANS #%d with value 0x%08X, but re-read value is 0x%08X", (uint32_t)index, set_value, get_value);
				sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
			}
		}
	}
}

/**	@brief CORE REGTRANS read operation

@par[inout] p_target pointer to this target
@par[in] index REGTRANS register index in CORE/CORE_REGTRANS
*/
static inline uint32_t
sc_rv32_core_REGTRANS_read(target const* const p_target, CORE_REGTRANS_indexes const index)
{
	return REGTRANS_read(p_target, DBGC_unit_id_CORE, DBGC_functional_group_CORE_REGTRANS, index);
}

/**	@brief Core REGTRANS write operation

@par[inout] p_target pointer to this target
@par[in] index REGTRANS register index in CORE/CORE_REGTRANS
@par[in] set_value 32-bits data
*/
static inline void
sc_rv32_CORE_REGTRANS_write(target const* const p_target, CORE_REGTRANS_indexes const index, uint32_t const data)
{
	REGTRANS_write(p_target, DBGC_unit_id_CORE, DBGC_functional_group_CORE_REGTRANS, index, data);
}

/**	@brief Setup HART before group HART_DBGCMD transactions.

@par[inout] p_target pointer to this target
*/
static inline void
sc_rv32_EXEC__setup(target const* const p_target)
{
	if (sc_error_code__get(p_target) == ERROR_OK) {
		/// @note Skipped if error detected
		/// @todo remove references to DBGC_unit_id_HART_1
		sc_rv32_DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_unit_id_HART_0 : DBGC_unit_id_HART_1, DBGC_functional_group_HART_DBGCMD);
	}
}

/**	@brief Push 32-bits data through keyhole from debug controller to core special SC Debug CSR.
*/
static inline void
sc_rv32_EXEC__push_data_to_CSR(target const* const p_target, uint32_t const csr_data)
{
	if (sc_error_code__get(p_target) == ERROR_OK) {
		/// @note Skipped if error detected
		sc_rv32_DAP_CMD_scan(p_target, DBGDATA_WR_index, csr_data, NULL);
	}
}

/**	@brief Push instruction (up to 32-bits) through keyhole from debug controller immediately to core.
@return SC Debug CSR data
*/
static inline uint32_t
sc_rv32_EXEC__step(target const* const p_target, uint32_t instruction)
{
	uint32_t result = 0xBADC0DE9u;

	if (ERROR_OK == sc_error_code__get(p_target)) {
		sc_rv32_DAP_CMD_scan(p_target, CORE_EXEC_index, instruction, &result);
	}

	return result;
}

/**	@brief Return last sampled PC value
*/
static uint32_t
sc_rv32_get_PC(target const* const p_target)
{
	assert(p_target);
	sc_riscv32__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);

	/// @todo Verify use_check_pc_unchanged
	if (p_arch->constants->use_check_pc_unchanged) {
		return sc_rv32_HART_REGTRANS_read(p_target, HART_PC_SAMPLE_index);
	} else {
		return 0xFFFFFFFFu;
	}
}

/**	@brief Convert HART status bits to target state enum values
*/
static target_state
HART_status_bits_to_target_state(uint32_t const status)
{
	static uint32_t const err_bits =
		HART_DBG_STS_bit_Err |
		HART_DBG_STS_bit_Err_HwThread |
		HART_DBG_STS_bit_Err_DAP_OpCode |
		HART_DBG_STS_bit_Err_DbgCmd_NACK;

	if (status & err_bits) {
		LOG_WARNING("Error status: 0x%08x", status);
		return TARGET_UNKNOWN;
	} else if (status & HART_DBG_STS_bit_Rst) {
		return TARGET_RESET;
	} else if (status & HART_DBG_STS_bit_DMODE) {
		return TARGET_HALTED;
	} else {
		return TARGET_RUNNING;
	}
}

static uint32_t
try_to_get_ready(target* const p_target)
{
	uint32_t core_status = sc_rv32_DBG_STATUS_get(p_target);

	if ((core_status & DBG_STATUS_bit_Ready) != 0) {
		return core_status;
	}

	static unsigned const max_retries = 10u;

	for (unsigned i = 2; i <= max_retries; ++i) {
		sc_error_code__get_and_clear(p_target);
		core_status = sc_rv32_DBG_STATUS_get(p_target);

		if ((core_status & DBG_STATUS_bit_Ready) != 0) {
			LOG_DEBUG("Ready: 0x%08X after %d requests", core_status, i);
			return core_status;
		}
	}

	LOG_ERROR("Not ready: 0x%08X after %d requests", core_status, max_retries);
	return core_status;
}

/**	@brief Read DMODE_CAUSE and try to encode to enum target_debug_reason
*/
static inline target_debug_reason
read_debug_cause(target* const p_target)
{
	uint32_t const value = sc_rv32_HART_REGTRANS_read(p_target, HART_DMODE_CAUSE_index);

	if (ERROR_OK != sc_error_code__get(p_target)) {
		return DBG_REASON_UNDEFINED;
	} else if (value & HART_DMODE_CAUSE_bit_Enforce) {
		return DBG_REASON_DBGRQ;
	} else if (value & HART_DMODE_CAUSE_bit_SStep) {
		return DBG_REASON_SINGLESTEP;
	} else if (value & HART_DMODE_CAUSE_bit_Brkpt) {
		return DBG_REASON_BREAKPOINT;
	} else if (value & HART_DMODE_CAUSE_bit_Rst_Exit) {
		return DBG_REASON_DBGRQ;
	} else {
		return DBG_REASON_UNDEFINED;
	}
}

static void
update_debug_reason(target* const p_target)
{
	assert(p_target);
	static char const* reasons_names[] = {
		"DBG_REASON_DBGRQ",
		"DBG_REASON_BREAKPOINT",
		"DBG_REASON_WATCHPOINT",
		"DBG_REASON_WPTANDBKPT",
		"DBG_REASON_SINGLESTEP",
		"DBG_REASON_NOTHALTED",
		"DBG_REASON_EXIT",
		"DBG_REASON_UNDEFINED",
	};
	target_debug_reason const debug_reason = read_debug_cause(p_target);

	if (debug_reason != p_target->debug_reason) {
		LOG_DEBUG("New debug reason: 0x%08X (%s)", (uint32_t)debug_reason, debug_reason >= ARRAY_LEN(reasons_names) ? "unknown" : reasons_names[debug_reason]);
		p_target->debug_reason = debug_reason;
	}
}

static inline void
update_debug_status(target* const p_target)
{
	assert(p_target);
	target_state const old_state = p_target->state;
	/// Only 1 HART available now
	assert(p_target->coreid == 0);
	uint32_t const HART_status = sc_rv32_HART_REGTRANS_read(p_target, HART_DBG_STS_index);
	target_state const new_state =
		ERROR_OK != sc_error_code__get(p_target) ?
		TARGET_UNKNOWN :
		HART_status_bits_to_target_state(HART_status);
	LOG_DEBUG("debug_status: old=%d, new=%d", old_state, new_state);

	if (new_state == old_state) {
		return;
	}

	p_target->state = new_state;

	switch (new_state) {
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

static inline void
check_and_repair_debug_controller_errors(target* const p_target)
{
	// protection against desynchronizing due to external reset
	jtag_add_tlr();
	invalidate_DAP_CTR_cache(p_target);
	uint32_t const IDCODE = sc_rv32_IDCODE_get(p_target);

	if (!sc_rv32__is_IDCODE_valid(p_target, IDCODE)) {
		/// @todo replace by target_reset_examined;
		p_target->examined = false;
		LOG_ERROR("Debug controller/JTAG error! Try to re-examine!");
		sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
		return;
	} else {
		uint32_t core_status = try_to_get_ready(p_target);

		if (0 != (core_status & DBG_STATUS_bit_Lock)) {
			LOG_ERROR("Lock detected: 0x%08X", core_status);
			uint32_t const lock_context = sc_rv32_DC__unlock(p_target);

			if (ERROR_OK != sc_error_code__get(p_target)) {
				/// return with error_code != ERROR_OK if unlock was unsuccsesful
				LOG_ERROR("Unlock unsucsessful with lock_context=0x%8X", lock_context);
				return;
			}

			core_status = sc_rv32_DBG_STATUS_get(p_target);
			LOG_INFO("Lock with lock_context=0x%8X repaired: 0x%08X", lock_context, core_status);
		}

		if (DBG_STATUS_bit_Ready != (core_status & (DBG_STATUS_bit_Lock | DBG_STATUS_bit_Ready))) {
			LOG_ERROR("Core_status with LOCK!: 0x%08X", core_status);
			sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
			return;
		}

		static uint32_t const hart0_err_bits = DBG_STATUS_bit_HART0_Err | DBG_STATUS_bit_HART0_Err_Stky;

		if (0 != (core_status & hart0_err_bits)) {
			LOG_WARNING("Hart errors detected: 0x%08X", core_status);
			sc_rv32_HART0_clear_sticky(p_target);
			sc_error_code__get_and_clear(p_target);
			core_status = sc_rv32_DBG_STATUS_get(p_target);
			LOG_WARNING("Hart errors %s: 0x%08X", core_status & hart0_err_bits ? "not fixed!" : "fixed", core_status);
		}

		static uint32_t const cdsr_err_bits =
			DBG_STATUS_bit_Err |
			DBG_STATUS_bit_Err_HwCore |
			DBG_STATUS_bit_Err_FsmBusy |
			DBG_STATUS_bit_Err_DAP_Opcode;

		if (0 != (core_status & cdsr_err_bits)) {
			LOG_WARNING("Core errors detected: 0x%08X", core_status);
			sc_error_code__get_and_clear(p_target);
			sc_rv32_CORE_clear_errors(p_target);
			sc_error_code__get_and_clear(p_target);
			core_status = sc_rv32_DBG_STATUS_get(p_target);

			if (0 != (core_status & cdsr_err_bits)) {
				LOG_ERROR("Core errors not fixed!: 0x%08X", core_status);
				sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
			} else {
				LOG_INFO("Core errors fixed: 0x%08X", core_status);
			}
		}
	}
}

void
sc_riscv32__update_status(target* const p_target)
{
	LOG_DEBUG("update_status");
	error_code const old_err_code = sc_error_code__get_and_clear(p_target);
	check_and_repair_debug_controller_errors(p_target);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		update_debug_status(p_target);
	}

	sc_error_code__prepend(p_target, old_err_code);
}

static void
sc_rv32_check_that_target_halted(target* const p_target)
{
	sc_riscv32__update_status(p_target);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		if (p_target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			sc_error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
		}
	}
}

/// GP registers accessors
/// @brief Invalidate register
static void
reg__invalidate(reg* const p_reg)
{
	assert(p_reg);

	if (p_reg->exist) {
		if (p_reg->dirty) {
			/// Log error if invalidate dirty (not updated) register
			LOG_ERROR("Invalidate dirty register: %s", p_reg->name);
			target* const p_target = p_reg->arch_info;
			sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
		}

		p_reg->valid = false;
	}
}

static inline void
reg__set_valid_value_to_cache(reg* const p_reg, uint32_t const value)
{
	assert(p_reg);
	assert(p_reg->exist);

	static_assert(CHAR_BIT == 8, "Unsupported char size");
	assert(p_reg->size <= CHAR_BIT * sizeof value);

	LOG_DEBUG("Updating cache from register %s to 0x%08X", p_reg->name, value);

	assert(p_reg->value);
	buf_set_u32(p_reg->value, 0, p_reg->size, value);

	p_reg->valid = true;
	p_reg->dirty = false;
}

static void
reg__set_new_cache_value(reg* const p_reg, uint8_t* const buf)
{
	assert(p_reg);
	assert(p_reg->exist);
	assert(buf);

	switch (p_reg->size) {
	case 32:
		LOG_DEBUG("Set register %s cache to 0x%08X", p_reg->name, buf_get_u32(buf, 0, p_reg->size));
		break;

	case 64:
		LOG_DEBUG("Set register %s cache to 0x%016lX", p_reg->name, buf_get_u64(buf, 0, p_reg->size));
		break;

	default:
		assert(!"Bad register size");
		break;
	}

	assert(p_reg->value);
	buf_cpy(buf, p_reg->value, p_reg->size);

	p_reg->valid = true;
	p_reg->dirty = true;
}

static inline bool
reg__check(reg const* const p_reg)
{
	assert(p_reg);
	if (!p_reg->exist) {
		LOG_ERROR("Register %s not exists", p_reg->name);
		return false;
	} else if (p_reg->dirty && !p_reg->valid) {
		LOG_ERROR("Register %s dirty but not valid", p_reg->name);
		return false;
	} else {
		return true;
	}
}

static inline void
reg_cache__invalidate(reg_cache const* const p_reg_cache)
{
	assert(p_reg_cache);
	assert(!(p_reg_cache->num_regs && !p_reg_cache->reg_list));

	for (size_t i = 0; i < p_reg_cache->num_regs; ++i) {
		reg* const p_reg = &p_reg_cache->reg_list[i];

		if (p_reg->exist) {
			assert(reg__check(&p_reg_cache->reg_list[i]));
			reg__invalidate(&p_reg_cache->reg_list[i]);
		}
	}
}

static void
reg_cache__chain_invalidate(reg_cache* p_reg_cache)
{
	for (; p_reg_cache; p_reg_cache = p_reg_cache->next) {
		reg_cache__invalidate(p_reg_cache);
	}
}

static error_code
reg_x__operation_conditions_check(reg const* const p_reg)
{
	if (!reg__check(p_reg)) {
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	} else if (p_reg->number >= number_of_regs_X) {
		LOG_WARNING("Bad GP register %s id=%d", p_reg->name, p_reg->number);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	} else {
		return ERROR_OK;
	}
}

static void
sc_rv32_check_PC_value(target const* const p_target, uint32_t const previous_pc)
{
	assert(p_target);
	sc_riscv32__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);

	if (p_arch->constants->use_check_pc_unchanged) {
		uint32_t const current_pc = sc_rv32_get_PC(p_target);

		if (current_pc != previous_pc) {
			LOG_ERROR("pc changed from 0x%08X to 0x%08X", previous_pc, current_pc);
			sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
		}
	}
}

static error_code
reg_x__get(reg* const p_reg)
{
	error_code const check_result = reg_x__operation_conditions_check(p_reg);
	if (ERROR_OK != check_result) {
		return check_result;
	} else {
		target* p_target = p_reg->arch_info;
		assert(p_target);
		sc_rv32_check_that_target_halted(p_target);

		if (ERROR_OK == sc_error_code__get(p_target)) {
			if (p_reg->valid) {
				// register cache already valid
				if (p_reg->dirty) {
					LOG_WARNING("Try re-read dirty cache register %s", p_reg->name);
				} else {
					LOG_DEBUG("Try re-read cache register %s", p_reg->name);
				}
			}

			uint32_t const previous_pc = sc_rv32_get_PC(p_target);

			if (ERROR_OK == sc_error_code__get(p_target)) {
				sc_riscv32__Arch const* const p_arch = p_target->arch_info;
				assert(p_arch);
				size_t const instr_step = p_arch->constants->use_pc_advmt_dsbl_bit ? 0u : NUM_BYTES_FOR_BITS(ILEN);

				if (p_arch->constants->use_pc_advmt_dsbl_bit) {
					sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, HART_DBG_CTRL_bit_PC_Advmt_Dsbl);
				}

				if (ERROR_OK == sc_error_code__get(p_target)) {
					sc_rv32_EXEC__setup(p_target);
					int advance_pc_counter = 0;

					if (ERROR_OK != sc_error_code__get(p_target)) {
						return sc_error_code__get_and_clear(p_target);
					}

					// Save p_reg->number register to Debug scratch CSR
					(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRW(p_arch->constants->debug_scratch_CSR, p_reg->number));
					advance_pc_counter += instr_step;

					if (ERROR_OK != sc_error_code__get(p_target)) {
						return sc_error_code__get_and_clear(p_target);
					}

					// Exec jump back to previous instruction and get saved into Debug scratch CSR value
					assert(advance_pc_counter % NUM_BYTES_FOR_BITS(ILEN) == 0);
					uint32_t const value = sc_rv32_EXEC__step(p_target, RISCV_opcode_JAL(0, -advance_pc_counter));
					advance_pc_counter = 0;

					if (ERROR_OK != sc_error_code__get(p_target)) {
						return sc_error_code__get_and_clear(p_target);
					}

					reg__set_valid_value_to_cache(p_reg, value);

					if (p_arch->constants->use_pc_advmt_dsbl_bit) {
						sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, 0);
					}
				} else {
					sc_riscv32__update_status(p_target);
				}

				sc_rv32_check_PC_value(p_target, previous_pc);
			} else {
				sc_riscv32__update_status(p_target);
			}
		}

		return sc_error_code__get_and_clear(p_target);
	}
}

static error_code
reg_x__store(reg* const p_reg)
{
	assert(p_reg);
	target* p_target = p_reg->arch_info;
	assert(p_target);
	uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);

	if (ERROR_OK != sc_error_code__get(p_target)) {
		return sc_error_code__get_and_clear(p_target);
	}

	sc_riscv32__Arch const* const p_arch = p_target->arch_info;

	assert(p_arch);

	size_t const instr_step = p_arch->constants->use_pc_advmt_dsbl_bit ? 0u : NUM_BYTES_FOR_BITS(ILEN);

	if (p_arch->constants->use_pc_advmt_dsbl_bit) {
		sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, HART_DBG_CTRL_bit_PC_Advmt_Dsbl);
	}

	sc_rv32_EXEC__setup(p_target);
	int advance_pc_counter = 0;

	assert(p_reg->value);
	sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));

	if (ERROR_OK != sc_error_code__get(p_target)) {
		return sc_error_code__get_and_clear(p_target);
	}

	assert(p_reg->valid);
	assert(p_reg->dirty);
	(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_reg->number, p_arch->constants->debug_scratch_CSR));
	advance_pc_counter += instr_step;
	p_reg->dirty = false;

	LOG_DEBUG("Store register value 0x%08X from cache to register %s", buf_get_u32(p_reg->value, 0, p_reg->size), p_reg->name);

	if (ERROR_OK != sc_error_code__get(p_target)) {
		return sc_error_code__get_and_clear(p_target);
	}

	/// Correct pc back after each instruction
	assert(advance_pc_counter % NUM_BYTES_FOR_BITS(ILEN) == 0);
	(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_JAL(0, -advance_pc_counter));
	advance_pc_counter = 0;

	if (p_arch->constants->use_pc_advmt_dsbl_bit) {
		sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, 0);
	}

	assert(reg__check(p_reg));
	sc_rv32_check_PC_value(p_target, pc_sample_1);

	return sc_error_code__get_and_clear(p_target);
}

static error_code
reg_x__set(reg* const p_reg, uint8_t* const buf)
{
	error_code const check_result = reg_x__operation_conditions_check(p_reg);
	if (ERROR_OK != check_result) {
		return check_result;
	} else {
		target* p_target = p_reg->arch_info;
		assert(p_target);
		sc_rv32_check_that_target_halted(p_target);

		if (ERROR_OK != sc_error_code__get(p_target)) {
			return sc_error_code__get_and_clear(p_target);
		}

		reg__set_new_cache_value(p_reg, buf);

		/// store dirty register data to HW
		return reg_x__store(p_reg);
	}
}

static reg_arch_type const reg_x_accessors = {
	.get = reg_x__get,
	.set = reg_x__set,
};

static error_code
reg_x0__get(reg* const p_reg)
{
	assert(p_reg);
	assert(p_reg->number == 0u);
	reg__set_valid_value_to_cache(p_reg, 0u);
	return ERROR_OK;
}

static error_code
reg_x0__set(reg* const p_reg, uint8_t* const buf)
{
	assert(p_reg);
	assert(buf);
	LOG_ERROR("Try to write to read-only register");
	assert(p_reg->number == 0u);
	reg__set_valid_value_to_cache(p_reg, 0u);
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
}

static reg_arch_type const reg_x0_accessors = {.get = reg_x0__get,.set = reg_x0__set,};

static reg*
prepare_temporary_GP_register(target const* const p_target, int const after_reg)
{
	assert(p_target);
	assert(p_target->reg_cache);
	reg* const p_reg_list = p_target->reg_cache->reg_list;
	assert(p_reg_list);
	assert(p_target->reg_cache->num_regs >= number_of_regs_X);
	reg* p_valid = NULL;
	reg* p_dirty = NULL;

	for (size_t i = after_reg + 1; i < number_of_regs_X; ++i) {
		assert(reg__check(&p_reg_list[i]));

		if (p_reg_list[i].valid) {
			if (p_reg_list[i].dirty) {
				p_dirty = &p_reg_list[i];
				p_valid = p_dirty;
				break;
			} else if (!p_valid) {
				p_valid = &p_reg_list[i];
			}
		}
	}

	if (!p_dirty) {
		if (!p_valid) {
			assert(after_reg + 1 < number_of_regs_X);
			p_valid = &p_reg_list[after_reg + 1];

			if (ERROR_OK != sc_error_code__update(p_target, reg_x__get(p_valid))) {
				return NULL;
			}
		}

		assert(p_valid);
		assert(p_valid->valid);
		p_valid->dirty = true;
		LOG_DEBUG("Mark temporary register %s dirty", p_valid->name);
		p_dirty = p_valid;
	}

	assert(p_dirty);
	assert(p_dirty->valid);
	assert(p_dirty->dirty);
	return p_dirty;
}

uint32_t
sc_riscv32__csr_get_value(target* const p_target, uint32_t const csr_number)
{
	uint32_t value = 0xBADBAD;
	sc_rv32_check_that_target_halted(p_target);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		/// Find temporary GP register
		reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);
		assert(p_wrk_reg);

		uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);

		if (ERROR_OK == sc_error_code__get(p_target)) {
			sc_riscv32__Arch const* const p_arch = p_target->arch_info;
			assert(p_arch);
			size_t const instr_step = p_arch->constants->use_pc_advmt_dsbl_bit ? 0u : NUM_BYTES_FOR_BITS(ILEN);

			if (p_arch->constants->use_pc_advmt_dsbl_bit) {
				sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, HART_DBG_CTRL_bit_PC_Advmt_Dsbl);
			}

			sc_rv32_EXEC__setup(p_target);
			int advance_pc_counter = 0;

			if (ERROR_OK == sc_error_code__get(p_target)) {
				/// Copy values to temporary register
				(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_wrk_reg->number, csr_number));
				advance_pc_counter += instr_step;

				if (sc_error_code__get(p_target) == ERROR_OK) {
					/// and store temporary register to Debug scratch CSR.
					(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRW(p_arch->constants->debug_scratch_CSR, p_wrk_reg->number));
					advance_pc_counter += instr_step;

					if (sc_error_code__get(p_target) == ERROR_OK) {
						/// Correct pc by jump 2 instructions back and get previous command result.
						assert(advance_pc_counter % NUM_BYTES_FOR_BITS(ILEN) == 0);
						value = sc_rv32_EXEC__step(p_target, RISCV_opcode_JAL(0, -advance_pc_counter));
						advance_pc_counter = 0;
					} else {
						sc_riscv32__update_status(p_target);
					}
				} else {
					sc_riscv32__update_status(p_target);
				}
			} else {
				sc_riscv32__update_status(p_target);
			}

			if (p_arch->constants->use_pc_advmt_dsbl_bit) {
				sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, 0);
			}
		} else {
			sc_riscv32__update_status(p_target);
		}

		sc_rv32_check_PC_value(p_target, pc_sample_1);

		// restore temporary register
		error_code const old_err_code = sc_error_code__get_and_clear(p_target);
		sc_error_code__update(p_target, reg_x__store(p_wrk_reg));
		sc_error_code__prepend(p_target, old_err_code);
		assert(!p_wrk_reg->dirty);
	}

	return value;
}

/// Update pc cache from HW (if non-cached)
static error_code
reg_pc__get(reg* const p_reg)
{
	assert(p_reg);
	assert(p_reg->number == RISCV_regnum_PC);
	assert(reg__check(p_reg));

	/// Find temporary GP register
	target* const p_target = p_reg->arch_info;
	assert(p_target);
	sc_riscv32__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);
	uint32_t const pc_sample = sc_rv32_HART_REGTRANS_read(p_target, HART_PC_SAMPLE_index);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		reg__set_valid_value_to_cache(p_reg, pc_sample);
	} else {
		reg__invalidate(p_reg);
	}

	return sc_error_code__get_and_clear(p_target);
}

static bool
is_RVC_enable(target* const p_target)
{
	sc_riscv32__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);
	uint32_t const isa = get_ISA(p_target);
	return 0 != (isa & (UINT32_C(1) << ('C' - 'A')));
}

static error_code
reg_pc__set(reg* const p_reg, uint8_t* const buf)
{
	assert(p_reg);
	assert(p_reg->number == RISCV_regnum_PC);
	assert(reg__check(p_reg));

	if (!p_reg->valid) {
		LOG_DEBUG("force rewriting of pc register before read");
	}

	target* const p_target = p_reg->arch_info;

	assert(p_target);

	sc_rv32_check_that_target_halted(p_target);

	if (ERROR_OK != sc_error_code__get(p_target)) {
		return sc_error_code__get_and_clear(p_target);
	}

	uint32_t const new_pc = buf_get_u32(buf, 0, p_reg->size);

	/// @note odd address is valid for pc, bit 0 value is ignored.
	if (0 != (new_pc & (1u << 1))) {
		bool const RVC_enable = is_RVC_enable(p_target);

		if (ERROR_OK != sc_error_code__get(p_target)) {
			return sc_error_code__get_and_clear(p_target);
		} else if (!RVC_enable) {
			LOG_ERROR("Unaligned PC: 0x%08X", new_pc);
			sc_error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
			return sc_error_code__get_and_clear(p_target);
		}
	}

	reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);

	assert(p_wrk_reg);

	reg__set_new_cache_value(p_reg, buf);

	sc_riscv32__Arch const* const p_arch = p_target->arch_info;

	assert(p_arch);

	size_t const instr_step = p_arch->constants->use_pc_advmt_dsbl_bit ? 0u : NUM_BYTES_FOR_BITS(ILEN);

	if (p_arch->constants->use_pc_advmt_dsbl_bit) {
		sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, HART_DBG_CTRL_bit_PC_Advmt_Dsbl);
	}

	// Update to HW
	sc_rv32_EXEC__setup(p_target);
	int advance_pc_counter = 0;

	if (ERROR_OK == sc_error_code__get(p_target)) {
		assert(p_reg->value);
		sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));

		if (ERROR_OK == sc_error_code__get(p_target)) {
			// set temporary register value to restoring pc value
			(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_wrk_reg->number, p_arch->constants->debug_scratch_CSR));
			advance_pc_counter += instr_step;

			if (ERROR_OK == sc_error_code__get(p_target)) {
				assert(p_wrk_reg->dirty);

				if (p_arch->constants->use_pc_advmt_dsbl_bit) {
					sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, 0);
					sc_rv32_EXEC__setup(p_target);
				}

				/// and exec JARL to set pc
				(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_JALR(0, p_wrk_reg->number, 0));
				advance_pc_counter = 0;
				assert(p_reg->valid);
				assert(p_reg->dirty);
				p_reg->dirty = false;
			}
		}
	}

	sc_riscv32__update_status(p_target);

#if 0

	if (p_arch->constants->use_pc_advmt_dsbl_bit) {
		HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, 0);
	}

#endif
	// restore temporary register
	error_code const old_err_code = sc_error_code__get_and_clear(p_target);
	sc_error_code__update(p_target, reg_x__store(p_wrk_reg));
	sc_error_code__prepend(p_target, old_err_code);
	assert(!p_wrk_reg->dirty);

	return sc_error_code__get_and_clear(p_target);
}

static reg_arch_type const reg_pc_accessors = {
	.get = reg_pc__get,
	.set = reg_pc__set,
};
static reg_data_type PC_reg_data_type = {
	.type = REG_TYPE_CODE_PTR,
};

static error_code
reg_FPU_S__get(reg* const p_reg)
{
	assert(p_reg);
	assert(p_reg->size == 32);
	assert(RISCV_regnum_FP_first <= p_reg->number && p_reg->number <= RISCV_regnum_FP_last);

	if (!p_reg->exist) {
		LOG_WARNING("FP register %s (#%d) is unavailable", p_reg->name, p_reg->number - RISCV_regnum_FP_first);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	assert(reg__check(p_reg));

	target* const p_target = p_reg->arch_info;
	assert(p_target);

	sc_rv32_check_that_target_halted(p_target);

	if (sc_error_code__get(p_target) != ERROR_OK) {
		return sc_error_code__get_and_clear(p_target);
	}

	/// @todo check that FPU is enabled
	/// Find temporary GP register
	reg* const p_wrk_reg_1 = prepare_temporary_GP_register(p_target, 0);
	assert(p_wrk_reg_1);

	uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);

	if (sc_error_code__get(p_target) == ERROR_OK) {
		sc_riscv32__Arch const* const p_arch = p_target->arch_info;
		assert(p_arch);
		size_t const instr_step = p_arch->constants->use_pc_advmt_dsbl_bit ? 0u : NUM_BYTES_FOR_BITS(ILEN);

		if (p_arch->constants->use_pc_advmt_dsbl_bit) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, HART_DBG_CTRL_bit_PC_Advmt_Dsbl);
		}

		sc_rv32_EXEC__setup(p_target);
		int advance_pc_counter = 0;

		if (sc_error_code__get(p_target) == ERROR_OK) {
			/// Copy values to temporary register
			(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_FMV_X_S(p_wrk_reg_1->number, p_reg->number - RISCV_regnum_FP_first));
			advance_pc_counter += instr_step;

			if (sc_error_code__get(p_target) == ERROR_OK) {
				/// and store temporary register to Debug scratch CSR.
				(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRW(p_arch->constants->debug_scratch_CSR, p_wrk_reg_1->number));
				advance_pc_counter += instr_step;

				if (sc_error_code__get(p_target) == ERROR_OK) {
					assert(advance_pc_counter % NUM_BYTES_FOR_BITS(ILEN) == 0);
					uint32_t const value = sc_rv32_EXEC__step(p_target, RISCV_opcode_JAL(0, -advance_pc_counter));
					advance_pc_counter = 0;

					if (sc_error_code__get(p_target) == ERROR_OK) {
						buf_set_u32(p_reg->value, 0, p_reg->size, value);
						p_reg->valid = true;
						p_reg->dirty = false;
					}
				}
			}
		}

		if (p_arch->constants->use_pc_advmt_dsbl_bit) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, 0);
		}
	}

	sc_rv32_check_PC_value(p_target, pc_sample_1);

	// restore temporary register
	error_code const old_err_code = sc_error_code__get_and_clear(p_target);

	if (sc_error_code__update(p_target, reg_x__store(p_wrk_reg_1)) == ERROR_OK) {
		assert(!p_wrk_reg_1->dirty);
	}

	sc_error_code__prepend(p_target, old_err_code);

	return sc_error_code__get_and_clear(p_target);
}

static error_code
reg_FPU_S__set(reg* const p_reg, uint8_t* const buf)
{
	assert(p_reg);
	assert(p_reg->size == 32);
	assert(RISCV_regnum_FP_first <= p_reg->number && p_reg->number < RISCV_regnum_FP_last);

	if (!p_reg->exist) {
		LOG_WARNING("Register %s is unavailable", p_reg->name);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	assert(reg__check(p_reg));

	target* const p_target = p_reg->arch_info;
	assert(p_target);

	sc_rv32_check_that_target_halted(p_target);

	if (sc_error_code__get(p_target) != ERROR_OK) {
		return sc_error_code__get_and_clear(p_target);
	}

	/// @todo check that FPU is enabled
	/// Find temporary GP register
	reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);
	assert(p_wrk_reg);

	uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);

	if (sc_error_code__get(p_target) == ERROR_OK) {
		sc_riscv32__Arch const* const p_arch = p_target->arch_info;
		assert(p_arch);
		size_t const instr_step = p_arch->constants->use_pc_advmt_dsbl_bit ? 0u : NUM_BYTES_FOR_BITS(ILEN);

		if (p_arch->constants->use_pc_advmt_dsbl_bit) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, HART_DBG_CTRL_bit_PC_Advmt_Dsbl);
		}

		sc_rv32_EXEC__setup(p_target);
		int advance_pc_counter = 0;

		if (sc_error_code__get(p_target) == ERROR_OK) {
			reg__set_new_cache_value(p_reg, buf);

			sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));

			if (sc_error_code__get(p_target) == ERROR_OK) {
				// set temporary register value to restoring pc value
				(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_wrk_reg->number, p_arch->constants->debug_scratch_CSR));
				advance_pc_counter += instr_step;

				if (sc_error_code__get(p_target) == ERROR_OK) {
					assert(p_wrk_reg->dirty);
					assert(0 < p_wrk_reg->number && p_wrk_reg->number < RISCV_regnum_PC);
					(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_FMV_S_X(p_reg->number - RISCV_regnum_FP_first, p_wrk_reg->number));
					advance_pc_counter += instr_step;

					if (sc_error_code__get(p_target) == ERROR_OK) {
						/// Correct pc by jump 2 instructions back and get previous command result.
						assert(advance_pc_counter % NUM_BYTES_FOR_BITS(ILEN) == 0);

						if (advance_pc_counter) {
							(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_JAL(0, -advance_pc_counter));
						}

						advance_pc_counter = 0;
						assert(p_reg->valid);
						assert(p_reg->dirty);
						p_reg->dirty = false;
						LOG_DEBUG("Store register value 0x%08X from cache to register %s", buf_get_u32(p_reg->value, 0, p_reg->size), p_reg->name);
					}
				}
			}
		}

		if (p_arch->constants->use_pc_advmt_dsbl_bit) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, 0);
		}
	}

	sc_rv32_check_PC_value(p_target, pc_sample_1);
	// restore temporary register
	error_code const old_err_code = sc_error_code__get_and_clear(p_target);
	sc_error_code__update(p_target, reg_x__store(p_wrk_reg));
	sc_error_code__prepend(p_target, old_err_code);
	assert(!p_wrk_reg->dirty);
	return sc_error_code__get_and_clear(p_target);
}

static reg_arch_type const reg_FPU_S_accessors = {.get = reg_FPU_S__get,.set = reg_FPU_S__set,};
static reg_data_type FPU_S_reg_data_type = {.type = REG_TYPE_IEEE_SINGLE,};

static error_code
reg_FPU_D__get(reg* const p_reg)
{
	assert(p_reg);
	assert(p_reg->size == 64);
	assert(RISCV_regnum_FP_first <= p_reg->number && p_reg->number <= RISCV_regnum_FP_last);

	if (!p_reg->exist) {
		LOG_WARNING("FP register %s (#%d) is unavailable", p_reg->name, p_reg->number - RISCV_regnum_FP_first);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	assert(reg__check(p_reg));

	target* const p_target = p_reg->arch_info;
	sc_rv32_check_that_target_halted(p_target);

	if (ERROR_OK != sc_error_code__get(p_target)) {
		return sc_error_code__get_and_clear(p_target);
	}

	sc_riscv32__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);
	uint32_t const mcpuid = get_ISA(p_target);

	if (ERROR_OK != sc_error_code__get(p_target)) {
		return sc_error_code__get_and_clear(p_target);
	}

	if (0 == (mcpuid & (BIT_MASK('f' - 'a') | BIT_MASK('d' - 'a')))) {
		LOG_ERROR("FPU is not supported");
		sc_error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
		return sc_error_code__get_and_clear(p_target);
	}

	uint32_t const mstatus = sc_riscv32__csr_get_value(p_target, CSR_mstatus);

	if (ERROR_OK != sc_error_code__get(p_target)) {
		return sc_error_code__get_and_clear(p_target);
	}

	if (0 == ((mstatus >> p_arch->constants->mstatus_FS_offset) & 3)) {
		LOG_ERROR("FPU is disabled");
		sc_error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
		return sc_error_code__get_and_clear(p_target);
	}

	bool const FPU_D = 0 != (mcpuid & BIT_MASK('d' - 'a'));

	/// Find temporary GP register
	reg* const p_wrk_reg_1 = prepare_temporary_GP_register(p_target, 0);
	assert(p_wrk_reg_1);
	reg* const p_wrk_reg_2 = prepare_temporary_GP_register(p_target, p_wrk_reg_1->number);
	assert(p_wrk_reg_2);

	uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		size_t const instr_step = p_arch->constants->use_pc_advmt_dsbl_bit ? 0u : NUM_BYTES_FOR_BITS(ILEN);

		if (p_arch->constants->use_pc_advmt_dsbl_bit) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, HART_DBG_CTRL_bit_PC_Advmt_Dsbl);
		}

		sc_rv32_EXEC__setup(p_target);
		int advance_pc_counter = 0;

		if (ERROR_OK == sc_error_code__get(p_target)) {
			uint32_t const opcode_1 =
				FPU_D ?
				p_arch->constants->opcode_FMV_2X_D(p_wrk_reg_2->number, p_wrk_reg_1->number, p_reg->number - RISCV_regnum_FP_first) :
				RISCV_opcode_FMV_X_S(p_wrk_reg_1->number, p_reg->number - RISCV_regnum_FP_first);

			(void)sc_rv32_EXEC__step(p_target, opcode_1);
			advance_pc_counter += instr_step;

			if (ERROR_OK == sc_error_code__get(p_target)) {
				/// and store temporary register to Debug scratch CSR.
				(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRW(p_arch->constants->debug_scratch_CSR, p_wrk_reg_1->number));
				advance_pc_counter += instr_step;

				if (ERROR_OK == sc_error_code__get(p_target)) {
					uint32_t const value_lo = sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRW(p_arch->constants->debug_scratch_CSR, p_wrk_reg_2->number));
					advance_pc_counter += instr_step;

					if (ERROR_OK == sc_error_code__get(p_target)) {
						/// Correct pc by jump 2 instructions back and get previous command result.
						assert(advance_pc_counter % NUM_BYTES_FOR_BITS(ILEN) == 0);
						uint32_t const value_hi = sc_rv32_EXEC__step(p_target, RISCV_opcode_JAL(0, -advance_pc_counter));
						advance_pc_counter = 0;

						if (ERROR_OK == sc_error_code__get(p_target)) {
							buf_set_u64(p_reg->value, 0, p_reg->size, (FPU_D ? (uint64_t)value_hi << 32 : 0u) | (uint64_t)value_lo);
							p_reg->valid = true;
							p_reg->dirty = false;
						} else {
							sc_riscv32__update_status(p_target);
						}
					} else {
						sc_riscv32__update_status(p_target);
					}
				} else {
					sc_riscv32__update_status(p_target);
				}
			} else {
				sc_riscv32__update_status(p_target);
			}
		} else {
			sc_riscv32__update_status(p_target);
		}

		if (p_arch->constants->use_pc_advmt_dsbl_bit) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, 0);
		}
	}

	sc_rv32_check_PC_value(p_target, pc_sample_1);

	// restore temporary register
	error_code const old_err_code = sc_error_code__get_and_clear(p_target);

	if (ERROR_OK == sc_error_code__update(p_target, reg_x__store(p_wrk_reg_2))) {
		assert(!p_wrk_reg_2->dirty);
	}

	if (ERROR_OK == sc_error_code__update(p_target, reg_x__store(p_wrk_reg_1))) {
		assert(!p_wrk_reg_1->dirty);
	}

	sc_error_code__prepend(p_target, old_err_code);

	return sc_error_code__get_and_clear(p_target);
}

static error_code
reg_FPU_D__set(reg* const p_reg, uint8_t* const buf)
{
	assert(p_reg);
	assert(p_reg->size == 64);
	assert(RISCV_regnum_FP_first <= p_reg->number && p_reg->number < RISCV_regnum_FP_last);

	if (!p_reg->exist) {
		LOG_WARNING("Register %s is unavailable", p_reg->name);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	assert(reg__check(p_reg));

	target* const p_target = p_reg->arch_info;
	assert(p_target);

	sc_rv32_check_that_target_halted(p_target);

	if (ERROR_OK != sc_error_code__get(p_target)) {
		return sc_error_code__get_and_clear(p_target);
	}

	sc_riscv32__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);
	uint32_t const mcpuid = get_ISA(p_target);

	if (ERROR_OK != sc_error_code__get(p_target)) {
		return sc_error_code__get_and_clear(p_target);
	}

	if (0 == (mcpuid & (BIT_MASK('f' - 'a') | BIT_MASK('d' - 'a')))) {
		LOG_ERROR("FPU is not supported");
		sc_error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
		return sc_error_code__get_and_clear(p_target);
	}

	uint32_t const mstatus = sc_riscv32__csr_get_value(p_target, CSR_mstatus);

	if (ERROR_OK != sc_error_code__get(p_target)) {
		return sc_error_code__get_and_clear(p_target);
	}

	if (0 == ((mstatus >> p_arch->constants->mstatus_FS_offset) & 3)) {
		LOG_ERROR("FPU is disabled");
		sc_error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
		return sc_error_code__get_and_clear(p_target);
	}

	bool const FPU_D = 0 != (mcpuid & BIT_MASK('d' - 'a'));

	/// Find temporary GP register
	reg* const p_wrk_reg_1 = prepare_temporary_GP_register(p_target, 0);
	assert(p_wrk_reg_1);
	assert(p_wrk_reg_1->dirty);
	assert(0 < p_wrk_reg_1->number && p_wrk_reg_1->number < RISCV_regnum_PC);

	reg* const p_wrk_reg_2 = prepare_temporary_GP_register(p_target, p_wrk_reg_1->number);
	assert(p_wrk_reg_2);
	assert(p_wrk_reg_2->dirty);
	assert(0 < p_wrk_reg_2->number && p_wrk_reg_2->number < RISCV_regnum_PC);

	uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		size_t const instr_step = p_arch->constants->use_pc_advmt_dsbl_bit ? 0u : NUM_BYTES_FOR_BITS(ILEN);

		if (p_arch->constants->use_pc_advmt_dsbl_bit) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, HART_DBG_CTRL_bit_PC_Advmt_Dsbl);
		}

		sc_rv32_EXEC__setup(p_target);
		int advance_pc_counter = 0;

		if (ERROR_OK == sc_error_code__get(p_target)) {
			reg__set_new_cache_value(p_reg, buf);
			sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));

			if (ERROR_OK == sc_error_code__get(p_target)) {
				// set temporary register value to restoring pc value
				(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_wrk_reg_1->number, p_arch->constants->debug_scratch_CSR));
				advance_pc_counter += instr_step;

				if (ERROR_OK == sc_error_code__get(p_target)) {
					sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(&((uint8_t const*)p_reg->value)[4], 0, p_reg->size));

					if (ERROR_OK == sc_error_code__get(p_target)) {
						(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_wrk_reg_2->number, p_arch->constants->debug_scratch_CSR));
						advance_pc_counter += instr_step;

						if (ERROR_OK == sc_error_code__get(p_target)) {
							uint32_t const opcode_1 =
								FPU_D ?
								p_arch->constants->opcode_FMV_D_2X(p_reg->number - RISCV_regnum_FP_first, p_wrk_reg_2->number, p_wrk_reg_1->number) :
								RISCV_opcode_FMV_S_X(p_reg->number - RISCV_regnum_FP_first, p_wrk_reg_1->number);
							(void)sc_rv32_EXEC__step(p_target, opcode_1);
							advance_pc_counter += instr_step;

							if (ERROR_OK == sc_error_code__get(p_target)) {
								/// Correct pc by jump 2 instructions back and get previous command result.
								assert(advance_pc_counter % NUM_BYTES_FOR_BITS(ILEN) == 0);

								if (advance_pc_counter) {
									(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_JAL(0, -advance_pc_counter));
								}

								advance_pc_counter = 0;
								assert(p_reg->valid);
								assert(p_reg->dirty);
								p_reg->dirty = false;
								LOG_DEBUG("Store register value 0x%016lX from cache to register %s", buf_get_u64(p_reg->value, 0, p_reg->size), p_reg->name);
							}
						}
					}
				}
			}
		}

		sc_riscv32__update_status(p_target);

		if (p_arch->constants->use_pc_advmt_dsbl_bit) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, 0);
		}
	} else {
		sc_riscv32__update_status(p_target);
	}

	sc_rv32_check_PC_value(p_target, pc_sample_1);
	// restore temporary register
	error_code const old_err_code = sc_error_code__get_and_clear(p_target);

	if (ERROR_OK == sc_error_code__update(p_target, reg_x__store(p_wrk_reg_2))) {
		assert(!p_wrk_reg_2->dirty);
	}

	if (ERROR_OK == sc_error_code__update(p_target, reg_x__store(p_wrk_reg_1))) {
		assert(!p_wrk_reg_1->dirty);
	}

	sc_error_code__prepend(p_target, old_err_code);
	return sc_error_code__get_and_clear(p_target);
}

static reg_arch_type const reg_FPU_D_accessors = {.get = reg_FPU_D__get,.set = reg_FPU_D__set,};
static reg_data_type IEEE_single_precision_type = {.type = REG_TYPE_IEEE_SINGLE,.id = "ieee_single",};
static reg_data_type IEEE_double_precision_type = {.type = REG_TYPE_IEEE_DOUBLE,.id = "ieee_double",};
static reg_data_type_union_field FPU_S = {.name = "S",.type = &IEEE_single_precision_type};
static reg_data_type_union_field FPU_D = {.name = "D",.type = &IEEE_double_precision_type,.next = &FPU_S};
static reg_data_type_union FPU_D_or_S = {.fields = &FPU_D};
static reg_data_type FPU_D_reg_data_type = {.type = REG_TYPE_ARCH_DEFINED,.id = "Float_D_or_S",.type_class = REG_TYPE_CLASS_UNION,.reg_type_union = &FPU_D_or_S};

static error_code
reg_csr__get(reg* const p_reg)
{
	assert(p_reg);
	assert(RISCV_regnum_CSR_first <= p_reg->number && p_reg->number <= RISCV_rtegnum_CSR_last);
	uint32_t const csr_number = p_reg->number - RISCV_regnum_CSR_first;
	assert(csr_number < 4096u);

	if (!p_reg->exist) {
		LOG_WARNING("CSR %s (#%d) is unavailable", p_reg->name, csr_number);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	assert(reg__check(p_reg));
	target* const p_target = p_reg->arch_info;
	assert(p_target);
	uint32_t const value = sc_riscv32__csr_get_value(p_target, csr_number);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		reg__set_valid_value_to_cache(p_reg, value);
	}

	return sc_error_code__get_and_clear(p_target);
}

static error_code
reg_csr__set(reg* const p_reg, uint8_t* const buf)
{
	assert(p_reg);
	assert(RISCV_regnum_CSR_first <= p_reg->number && p_reg->number <= RISCV_rtegnum_CSR_last);

	if (!p_reg->exist) {
		LOG_WARNING("Register %s is unavailable", p_reg->name);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	assert(reg__check(p_reg));

	target* const p_target = p_reg->arch_info;
	assert(p_target);

	sc_rv32_check_that_target_halted(p_target);

	if (ERROR_OK != sc_error_code__get(p_target)) {
		return sc_error_code__get_and_clear(p_target);
	}

	reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);

	assert(p_wrk_reg);

	uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		sc_riscv32__Arch const* const p_arch = p_target->arch_info;
		assert(p_arch);
		size_t const instr_step = p_arch->constants->use_pc_advmt_dsbl_bit ? 0u : NUM_BYTES_FOR_BITS(ILEN);

		if (p_arch->constants->use_pc_advmt_dsbl_bit) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, HART_DBG_CTRL_bit_PC_Advmt_Dsbl);
		}

		sc_rv32_EXEC__setup(p_target);
		int advance_pc_counter = 0;

		if (ERROR_OK == sc_error_code__get(p_target)) {
			reg__set_new_cache_value(p_reg, buf);

			sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));

			if (ERROR_OK == sc_error_code__get(p_target)) {
				// set temporary register value
				(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_wrk_reg->number, p_arch->constants->debug_scratch_CSR));
				advance_pc_counter += instr_step;

				if (ERROR_OK == sc_error_code__get(p_target)) {
					assert(p_wrk_reg->dirty);
					assert(p_wrk_reg->number < number_of_regs_X);
					(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRW(p_reg->number - RISCV_regnum_CSR_first, p_wrk_reg->number));
					advance_pc_counter += instr_step;

					if (ERROR_OK == sc_error_code__get(p_target)) {
						/// Correct pc by jump 2 instructions back and get previous command result.
						assert(advance_pc_counter % NUM_BYTES_FOR_BITS(ILEN) == 0);

						if (advance_pc_counter) {
							(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_JAL(0, -advance_pc_counter));
						}

						advance_pc_counter = 0;
						assert(p_reg->valid);
						assert(p_reg->dirty);
						p_reg->dirty = false;
						LOG_DEBUG("Store register value 0x%08X from cache to register %s", buf_get_u32(p_reg->value, 0, p_reg->size), p_reg->name);
					}
				}
			}
		}

		sc_riscv32__update_status(p_target);

		if (p_arch->constants->use_pc_advmt_dsbl_bit) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, 0);
		}
	} else {
		sc_riscv32__update_status(p_target);
	}

	sc_rv32_check_PC_value(p_target, pc_sample_1);
	// restore temporary register
	error_code const old_err_code = sc_error_code__get_and_clear(p_target);
	sc_error_code__update(p_target, reg_x__store(p_wrk_reg));
	sc_error_code__prepend(p_target, old_err_code);
	assert(!p_wrk_reg->dirty);
	return sc_error_code__get_and_clear(p_target);
}

static reg_arch_type const reg_csr_accessors = {
	.get = reg_csr__get,
	.set = reg_csr__set,
};

static reg_cache*
reg_cache__section_create(char const* name, reg const regs_templates[], size_t const num_regs, void* const p_arch_info)
{
	assert(name);
	assert(0 < num_regs);
	assert(p_arch_info);
	reg* const p_dst_array = calloc(num_regs, sizeof(reg));
	reg* p_dst_iter = &p_dst_array[0];
	reg const* p_src_iter = &regs_templates[0];

	for (size_t i = 0; i < num_regs; ++i) {
		*p_dst_iter = *p_src_iter;
		p_dst_iter->value = calloc(1, NUM_BYTES_FOR_BITS(p_src_iter->size));
		p_dst_iter->arch_info = p_arch_info;

		++p_src_iter;
		++p_dst_iter;
	}

	reg_cache const the_reg_cache = {
		.name = name,
		.reg_list = p_dst_array,
		.num_regs = num_regs,
	};

	reg_cache* const p_obj = calloc(1, sizeof(reg_cache));

	assert(p_obj);

	*p_obj = the_reg_cache;

	return p_obj;
}

static void
set_DEMODE_ENBL(target* const p_target, uint32_t const set_value)
{
	sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DMODE_ENBL_index, set_value);
}

static error_code
resume_common(target* const p_target, uint32_t dmode_enabled, int const current, uint32_t const address, int const handle_breakpoints, int const debug_execution)
{
	assert(p_target);
	sc_rv32_check_that_target_halted(p_target);

	if (ERROR_OK != sc_error_code__get(p_target)) {
		return sc_error_code__get_and_clear(p_target);
	}

	// TODO: multiple caches
	// PC cache
	reg* const p_pc = &p_target->reg_cache->reg_list[number_of_regs_X];

	if (!current) {
		// setup new PC
		uint8_t buf[sizeof address];
		buf_set_u32(buf, 0, XLEN, address);

		if (ERROR_OK != sc_error_code__update(p_target, reg_pc__set(p_pc, buf))) {
			return sc_error_code__get_and_clear(p_target);
		}

		assert(!p_pc->dirty);
	}

	if (handle_breakpoints) {
		if (current) {
			// Find breakpoint for current instruction
			sc_error_code__update(p_target, reg_pc__get(p_pc));
			assert(p_pc->value);
			uint32_t const pc = buf_get_u32(p_pc->value, 0, XLEN);
			breakpoint* p_next_bkp = p_target->breakpoints;

			for (; p_next_bkp; p_next_bkp = p_next_bkp->next) {
				if (p_next_bkp->set && (p_next_bkp->address == pc)) {
					break;
				}
			}

			if (p_next_bkp) {
				// exec single step without breakpoint
				sc_riscv32__Arch const* const p_arch = p_target->arch_info;
				assert(p_arch);
				// save breakpoint
				breakpoint next_bkp = *p_next_bkp;
				// remove breakpoint
				sc_error_code__update(p_target, target_remove_breakpoint(p_target, &next_bkp));
				// prepare for single step
				reg_cache__chain_invalidate(p_target->reg_cache);
				// force single step
				set_DEMODE_ENBL(p_target, dmode_enabled | HART_DMODE_ENBL_bit_SStep);
				sc_rv32_DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_unit_id_HART_0 : DBGC_unit_id_HART_1, DBGC_functional_group_HART_DBGCMD);
				// resume for single step
				sc_rv32_DAP_CMD_scan(p_target, DBG_CTRL_index, DBG_CTRL_bit_Resume | DBG_CTRL_bit_Sticky_Clr, NULL);
				// restore breakpoint
				sc_error_code__update(p_target, target_add_breakpoint(p_target, &next_bkp));

				// If resume/halt already done (by single step)
				if (0 != (dmode_enabled & HART_DMODE_ENBL_bit_SStep)) {
					// TODO: extra call
					reg_cache__chain_invalidate(p_target->reg_cache);
					// set status
					p_target->state = debug_execution ? TARGET_DEBUG_RUNNING : TARGET_RUNNING;
					// raise resume event
					target_call_event_callbacks(p_target, debug_execution ? TARGET_EVENT_DEBUG_RESUMED : TARGET_EVENT_RESUMED);
					// setup debug mode
					set_DEMODE_ENBL(p_target, dmode_enabled);
					// set debug reason
					sc_riscv32__update_status(p_target);
					LOG_DEBUG("New debug reason: 0x%08X", DBG_REASON_SINGLESTEP);
					p_target->debug_reason = DBG_REASON_SINGLESTEP;
					// raise halt event
					target_call_event_callbacks(p_target, debug_execution ? TARGET_EVENT_DEBUG_HALTED : TARGET_EVENT_HALTED);
					// and exit
					return sc_error_code__get_and_clear(p_target);
				}
			}
		}

		// dmode_enabled |= HART_DMODE_ENBL_bit_Brkpt;
#if 0
	} else {
		dmode_enabled &= ~HART_DMODE_ENBL_bit_Brkpt;
#endif
	}

	// prepare for execution continue
	reg_cache__chain_invalidate(p_target->reg_cache);
	// enable requested debug mode
	set_DEMODE_ENBL(p_target, dmode_enabled);

	if (ERROR_OK != sc_error_code__get(p_target)) {
		return sc_error_code__get_and_clear(p_target);
	}

	// resume exec
	sc_rv32_DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_unit_id_HART_0 : DBGC_unit_id_HART_1, DBGC_functional_group_HART_DBGCMD);

	if (ERROR_OK != sc_error_code__get(p_target)) {
		LOG_WARNING("DAP_CTRL_REG_set error");
		return sc_error_code__get_and_clear(p_target);
	}

	sc_rv32_DAP_CMD_scan(p_target, DBG_CTRL_index, DBG_CTRL_bit_Resume | DBG_CTRL_bit_Sticky_Clr, NULL);

	if (ERROR_OK != sc_error_code__get(p_target)) {
		return sc_error_code__get_and_clear(p_target);
	}

	// Mark "not halted", set state, raise event
	LOG_DEBUG("New debug reason: 0x%08X", DBG_REASON_NOTHALTED);
	p_target->debug_reason = DBG_REASON_NOTHALTED;
	p_target->state = debug_execution ? TARGET_DEBUG_RUNNING : TARGET_RUNNING;
	target_call_event_callbacks(p_target, debug_execution ? TARGET_EVENT_DEBUG_RESUMED : TARGET_EVENT_RESUMED);

	sc_riscv32__update_status(p_target);
	return sc_error_code__get_and_clear(p_target);
}

static error_code
reset__set(target* const p_target, bool const active)
{
	assert(p_target);
	uint32_t const get_old_value1 = sc_rv32_core_REGTRANS_read(p_target, CORE_DBG_CTRL_index);

	if (sc_error_code__get(p_target) == ERROR_OK) {
		static uint32_t const bit_mask = CORE_DBG_CTRL_bit_HART0_Rst | CORE_DBG_CTRL_bit_Rst;
		uint32_t const set_value = (get_old_value1 & ~bit_mask) | (active ? bit_mask : 0u);
		sc_rv32_CORE_REGTRANS_write(p_target, CORE_DBG_CTRL_index, set_value);

		if (sc_error_code__get(p_target) == ERROR_OK) {
			sc_riscv32__Arch const* const p_arch = p_target->arch_info;
			assert(p_arch);

			if (p_arch->constants->use_verify_core_regtrans_write) {
				uint32_t const get_new_value2 = sc_rv32_core_REGTRANS_read(p_target, CORE_DBG_CTRL_index);

				if (sc_error_code__get(p_target) != ERROR_OK) {
					return sc_error_code__get_and_clear(p_target);
				}

				if ((get_new_value2 & bit_mask) != (set_value & bit_mask)) {
					LOG_ERROR("Fail to verify write: set 0x%08X, but get 0x%08X", set_value, get_new_value2);
					sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
					return sc_error_code__get_and_clear(p_target);
				}
			}

			LOG_DEBUG("update_status");
			sc_riscv32__update_status(p_target);

			if (sc_error_code__get(p_target) == ERROR_OK) {
				if (active) {
					if (p_target->state != TARGET_RESET) {
						/// issue error if we are still running
						LOG_ERROR("Target is not resetting after reset assert");
						sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
					}
				} else {
					if (p_target->state == TARGET_RESET) {
						LOG_ERROR("Target is still in reset after reset deassert");
						sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
					}
				}
			}
		}
	}

	return sc_error_code__get_and_clear(p_target);
}

static reg const def_GP_regs_array[] = {
	// Hard-wired zero
	{.name = "x0",.number = 0,.caller_save = false,.dirty = false,.valid = true,.exist = true,.size = XLEN,.type = &reg_x0_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},

	// Return address
	{.name = "x1",.number = 1,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},

	// Stack pointer
	{.name = "x2",.number = 2,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},

	// Global pointer
	{.name = "x3",.number = 3,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},

	// Thread pointer
	{.name = "x4",.number = 4,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},

	// Temporaries
	{.name = "x5",.number = 5,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x6",.number = 6,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x7",.number = 7,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},

	// Saved register/frame pointer
	{.name = "x8",.number = 8,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},

	// Saved register
	{.name = "x9",.number = 9,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},

	// Function arguments/return values
	{.name = "x10",.number = 10,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x11",.number = 11,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},

	// Function arguments
	{.name = "x12",.number = 12,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x13",.number = 13,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x14",.number = 14,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x15",.number = 15,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x16",.number = 16,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x17",.number = 17,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},

	// Saved registers
	{.name = "x18",.number = 18,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x19",.number = 19,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x20",.number = 20,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x21",.number = 21,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x22",.number = 22,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x23",.number = 23,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x24",.number = 24,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x25",.number = 25,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x26",.number = 26,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x27",.number = 27,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},

	// Temporaries
	{.name = "x28",.number = 28,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x29",.number = 29,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x30",.number = 30,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},
	{.name = "x31",.number = 31,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_GP_regs_name},

	// Program counter
	{.name = "pc",.number = RISCV_regnum_PC,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_pc_accessors,.feature = &feature_riscv_org,.reg_data_type = &PC_reg_data_type,.group = def_GP_regs_name},
};
static char const def_FPU_regs_name[] = "float";
static reg const def_FP_regs_array[] = {
	// FP temporaries
	{.name = "f0",.number = 0 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f1",.number = 1 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f2",.number = 2 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f3",.number = 3 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f4",.number = 4 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f5",.number = 5 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f6",.number = 6 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f7",.number = 7 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},

	// FP saved registers
	{.name = "f8",.number = 8 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f9",.number = 9 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},

	// FP arguments/return values
	{.name = "f10",.number = 10 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f11",.number = 11 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},

	// FP arguments
	{.name = "f12",.number = 12 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f13",.number = 13 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f14",.number = 14 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f15",.number = 15 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f16",.number = 16 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f17",.number = 17 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},

	// FP saved registers
	{.name = "f18",.number = 18 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f19",.number = 19 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f20",.number = 20 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f21",.number = 21 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f22",.number = 22 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f23",.number = 23 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f24",.number = 24 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f25",.number = 25 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f26",.number = 26 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f27",.number = 27 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},

	// FP temporaries
	{.name = "f28",.number = 28 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f29",.number = 29 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f30",.number = 30 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
	{.name = "f31",.number = 31 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = false,.size = FLEN,.type = &reg_FPU_D_accessors,.feature = &feature_riscv_org,.reg_data_type = &FPU_D_reg_data_type,.group = def_FPU_regs_name},
};

static char const def_CSR_regs_name[] = "system";
static reg const CSR_not_exists = {.name = "",.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_csr_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_CSR_regs_name};
static char csr_names[4096][50] = {[0 ... 4095] = {'\0'}};

static void
init_csr_names(void)
{
	static bool csr_names_inited = false;

	if (!csr_names_inited) {
		for (int i = 0; i < 4096; ++i) {
			sprintf(csr_names[i], "csr%d", i);
		}

		csr_names_inited = true;
	}
}

static reg_cache*
reg_cache__CSR_section_create_gdb(char const* name, void* const p_arch_info)
{
	init_csr_names();
	assert(name);
	reg* const p_dst_array = calloc(4096, sizeof(reg));
	{
		for (size_t i = 0; i < 4096; ++i) {
			reg* p_reg = &p_dst_array[i];
			*p_reg = CSR_not_exists;
			// TODO cleanup
			p_reg->name = csr_names[i];
			p_reg->number = i + RISCV_regnum_CSR_first;
			p_reg->value = calloc(1, NUM_BYTES_FOR_BITS(p_reg->size));;
			p_reg->arch_info = p_arch_info;
		}
	}
	reg_cache const the_reg_cache = {
		.name = name,
		.reg_list = p_dst_array,
		.num_regs = 4096,
	};

	reg_cache* const p_obj = calloc(1, sizeof(reg_cache));
	assert(p_obj);
	*p_obj = the_reg_cache;
	return p_obj;
}

void
sc_riscv32__init_regs_cache(target* const p_target)
{
	assert(p_target);
	p_target->reg_cache = reg_cache__section_create(def_GP_regs_name, def_GP_regs_array, ARRAY_LEN(def_GP_regs_array), p_target);
	reg_cache* p_reg_cache_last = p_target->reg_cache;
	p_reg_cache_last = p_reg_cache_last->next = reg_cache__section_create(def_FPU_regs_name, def_FP_regs_array, ARRAY_LEN(def_FP_regs_array), p_target);
	p_reg_cache_last->next = reg_cache__CSR_section_create_gdb(def_CSR_regs_name, p_target);
}

void
sc_riscv32__deinit_target(target* const p_target)
{
	assert(p_target);

	while (p_target->reg_cache) {
		reg_cache* const p_reg_cache = p_target->reg_cache;
		p_target->reg_cache = p_target->reg_cache->next;
		reg* const reg_list = p_reg_cache->reg_list;
		assert(!p_reg_cache->num_regs || reg_list);

		for (unsigned i = 0; i < p_reg_cache->num_regs; ++i) {
			free(reg_list[i].value);
		}

		free(reg_list);

		free(p_reg_cache);
	}

	if (p_target->arch_info) {
		free(p_target->arch_info);
		p_target->arch_info = NULL;
	}
}

error_code
sc_riscv32__target_create(target* const p_target, Jim_Interp* interp)
{
	assert(p_target);
	return ERROR_OK;
}

static void
ajust_target_registers_cache(target* const p_target)
{
	uint32_t const isa = get_ISA(p_target);
	bool const RV_I = 0 != (isa & (UINT32_C(1) << ('I' - 'A')));
	bool const RV_E = 0 != (isa & (UINT32_C(1) << ('E' - 'A')));
	bool const RV_D = 0 != (isa & (UINT32_C(1) << ('D' - 'A')));
	bool const RV_F = 0 != (isa & (UINT32_C(1) << ('F' - 'A')));
	assert(!!(RV_I) ^ !!(RV_E));
	if (RV_I) {
		assert(!RV_E);
		assert(p_target->reg_cache && p_target->reg_cache->reg_list && 33 == p_target->reg_cache->num_regs);
		reg* const p_regs = p_target->reg_cache->reg_list;
		for (int i = 16; i < 32; ++i) {
			p_regs[i].exist = true;
		}
		LOG_INFO("Re-enable RVI upper X registers");
	} else if (RV_E) {
		assert(!RV_I);
		assert(p_target->reg_cache && p_target->reg_cache->reg_list && 33 == p_target->reg_cache->num_regs);
		reg* const p_regs = p_target->reg_cache->reg_list;
		for (int i = 16; i < 32; ++i) {
			p_regs[i].exist = false;
		}
		LOG_INFO("Disable RVE upper X registers");
	}

	if (RV_D) {
		assert(RV_F);
		assert(p_target->reg_cache && p_target->reg_cache->next && p_target->reg_cache->next->reg_list && 32 == p_target->reg_cache->next->num_regs);
		reg* const p_regs = p_target->reg_cache->next->reg_list;
		for (int i = 0; i < 32; ++i) {
			reg* const p_reg = &p_regs[i];
			p_reg->exist = true;
			p_reg->size = 64;
			p_reg->type = &reg_FPU_D_accessors;
			p_reg->reg_data_type = &FPU_D_reg_data_type;
		}
		LOG_INFO("Re-enable RVFD FPU registers");
	} else if (RV_F) {
		assert(p_target->reg_cache && p_target->reg_cache->next && p_target->reg_cache->next->reg_list && 32 == p_target->reg_cache->next->num_regs);
		reg* const p_regs = p_target->reg_cache->next->reg_list;
		for (int i = 0; i < 32; ++i) {
			reg* const p_reg = &p_regs[i];
			p_reg->exist = true;
			p_reg->size = 32;
			p_reg->type = &reg_FPU_S_accessors;
			p_reg->reg_data_type = &FPU_S_reg_data_type;
		}
		LOG_INFO("Configure RVF FPU registers");
	} else {
		assert(p_target->reg_cache && p_target->reg_cache->next && p_target->reg_cache->next->reg_list && 32 == p_target->reg_cache->next->num_regs);
		reg* const p_regs = p_target->reg_cache->next->reg_list;
		for (int i = 0; i < 32; ++i) {
			reg* const p_reg = &p_regs[i];
			p_reg->exist = false;
		}
		LOG_INFO("Disable RV FPU registers");
	}
}

error_code
sc_riscv32__examine(target* const p_target)
{
	assert(p_target);

	for (int i = 0; i < 10; ++i) {
		sc_error_code__get_and_clear(p_target);
		LOG_DEBUG("update_status");
		sc_riscv32__update_status(p_target);

		if (ERROR_OK == sc_error_code__get(p_target)) {
			break;
		}

		LOG_DEBUG("update_status error, retry");
	}

	if (ERROR_OK == sc_error_code__get(p_target)) {
		uint32_t const IDCODE = sc_rv32_IDCODE_get(p_target);

		if (!sc_rv32__is_IDCODE_valid(p_target, IDCODE)) {
			LOG_ERROR("Invalid IDCODE=0x%08X!", IDCODE);
			sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
		} else {
			uint32_t const DBG_ID = sc_rv32_DBG_ID_get(p_target);

			if (!sc_rv32__is_DBG_ID_valid(p_target, DBG_ID)) {
				LOG_ERROR("Unsupported DBG_ID=0x%08X!", DBG_ID);
				sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
			} else {
				LOG_INFO("IDCODE=0x%08X DBG_ID=0x%08X BLD_ID=0x%08X", IDCODE, DBG_ID, sc_rv32_BLD_ID_get(p_target));
				set_DEMODE_ENBL(p_target, HART_DMODE_ENBL_bits_Normal);
				ajust_target_registers_cache(p_target);
				if (ERROR_OK == sc_error_code__get(p_target)) {
					LOG_DEBUG("Examined OK");
					target_set_examined(p_target);
				}
			}
		}
	}

	return sc_error_code__get_and_clear(p_target);
}

error_code
sc_riscv32__poll(target* const p_target)
{
	assert(p_target);
	sc_riscv32__update_status(p_target);
	return sc_error_code__get_and_clear(p_target);
}

error_code
sc_riscv32__arch_state(target* const p_target)
{
	assert(p_target);
	sc_riscv32__update_status(p_target);
	return sc_error_code__get_and_clear(p_target);
}

error_code
sc_riscv32__halt(target* const p_target)
{
	assert(p_target);
	// May be already halted?
	{
		sc_riscv32__update_status(p_target);

		if (ERROR_OK != sc_error_code__get(p_target)) {
			return sc_error_code__get_and_clear(p_target);
		}

		if (p_target->state == TARGET_HALTED) {
			LOG_WARNING("Halt request when target is already in halted state");
			return sc_error_code__get_and_clear(p_target);
		}
	}

	// Try to halt
	{
		sc_rv32_DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_unit_id_HART_0 : DBGC_unit_id_HART_1, DBGC_functional_group_HART_DBGCMD);

		if (ERROR_OK != sc_error_code__get(p_target)) {
			LOG_WARNING("DAP_CTRL_REG_set error");
			return sc_error_code__get_and_clear(p_target);
		}

		sc_rv32_DAP_CMD_scan(p_target, DBG_CTRL_index, DBG_CTRL_bit_Halt | DBG_CTRL_bit_Sticky_Clr, NULL);

		if (ERROR_OK != sc_error_code__get(p_target)) {
			return sc_error_code__get_and_clear(p_target);
		}
	}

	sc_rv32_check_that_target_halted(p_target);
	return sc_error_code__get_and_clear(p_target);
}

error_code
sc_riscv32__resume(target* const p_target, int const current, uint32_t const address, int const handle_breakpoints, int const debug_execution)
{
	LOG_DEBUG("resume: current=%d address=0x%08x handle_breakpoints=%d debug_execution=%d", current, address, handle_breakpoints, debug_execution);
	assert(p_target);
	static uint32_t const dmode_enabled = HART_DMODE_ENBL_bits_Normal;
	return resume_common(p_target, dmode_enabled, current, address, handle_breakpoints, debug_execution);
}

error_code
sc_riscv32__step(target* const p_target, int const current, uint32_t const address, int const handle_breakpoints)
{
	LOG_DEBUG("step: current=%d address=0x%08x handle_breakpoints=%d", current, address, handle_breakpoints);
	assert(p_target);
	// disable halt on SW breakpoint to pass SW breakpoint processing to core
	static uint32_t const dmode_enabled = (HART_DMODE_ENBL_bits_Normal & ~HART_DMODE_ENBL_bit_Brkpt) | HART_DMODE_ENBL_bit_SStep;
	return resume_common(p_target, dmode_enabled, current, address, handle_breakpoints, false);
}

error_code
sc_riscv32__assert_reset(target* const p_target)
{
	LOG_DEBUG("Assert reset");
	return reset__set(p_target, true);
}

error_code
sc_riscv32__deassert_reset(target* const p_target)
{
	LOG_DEBUG("Deassert reset");
	return reset__set(p_target, false);
}

error_code
sc_riscv32__soft_reset_halt(target* const p_target)
{
	LOG_DEBUG("Soft reset called");
	reset__set(p_target, true);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		set_DEMODE_ENBL(p_target, HART_DMODE_ENBL_bits_Normal | HART_DMODE_ENBL_bit_Rst_Exit);

		if (ERROR_OK == sc_error_code__get(p_target)) {
			reset__set(p_target, false);
		} else {
			sc_riscv32__update_status(p_target);
		}
	} else {
		sc_riscv32__update_status(p_target);
	}

	return sc_error_code__get_and_clear(p_target);
}

static void
read_memory_space(target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t* p_buffer, bool const instruction_space)
{
	if (!(size == 1 || size == 2 || size == 4)) {
		LOG_ERROR("Invalid item size %d", size);
		sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
	} else if (address % size != 0) {
		LOG_ERROR("Unaligned access at 0x%08X, for item size %d", address, size);
		sc_error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
	} else {
		while (0 != count) {
			uint32_t physical;
			uint32_t bound;
			sc_riscv32__Arch const* const p_arch = p_target->arch_info;
			assert(p_arch);
			p_arch->constants->virt_to_phis(p_target, address, &physical, &bound, instruction_space);

			if (ERROR_OK != sc_error_code__get(p_target)) {
				break;
			}

			uint32_t const page_count = size * count > bound ? bound / size : count;
			assert(0 != page_count);
			assert(p_buffer);

			if (ERROR_OK != sc_error_code__update(p_target, target_read_phys_memory(p_target, physical, size, page_count, p_buffer))) {
				break;
			}

			uint32_t const bytes = size * page_count;
			p_buffer += bytes;
			address += bytes;
			count -= page_count;
		}
	}
}

static void
write_memory_space(target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t const* p_buffer, bool const instruction_space)
{
	if (!(size == 1 || size == 2 || size == 4)) {
		LOG_ERROR("Invalid item size %d", size);
		sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
	} else if (address % size != 0) {
		LOG_ERROR("Unaligned access at 0x%08X, for item size %d", address, size);
		sc_error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
	} else {
		while (0 != count) {
			uint32_t physical;
			uint32_t bound;
			sc_riscv32__Arch const* const p_arch = p_target->arch_info;
			assert(p_arch);
			p_arch->constants->virt_to_phis(p_target, address, &physical, &bound, instruction_space);

			if (ERROR_OK != sc_error_code__get(p_target)) {
				break;
			}

			uint32_t const page_count = size * count > bound ? bound / size : count;
			assert(0 != page_count);
			assert(p_buffer);

			if (ERROR_OK != sc_error_code__update(p_target, target_write_phys_memory(p_target, physical, size, page_count, p_buffer))) {
				break;
			}

			uint32_t const bytes = size * page_count;
			p_buffer += bytes;
			address += bytes;
			count -= page_count;
		}
	}
}

error_code
sc_riscv32__read_memory(target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t* buffer)
{
	read_memory_space(p_target, address, size, count, buffer, false);
	return sc_error_code__get_and_clear(p_target);
}

error_code
sc_riscv32__write_memory(target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t const* buffer)
{
	write_memory_space(p_target, address, size, count, buffer, false);
	return sc_error_code__get_and_clear(p_target);
}

error_code
sc_riscv32__read_phys_memory(target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t* buffer)
{
	LOG_DEBUG("Read_memory at 0x%08X, %d items, each %d bytes, total %d bytes", address, count, size, count * size);

	/// Check for size
	if (!(size == 1 || size == 2 || size == 4)) {
		LOG_ERROR("Invalid item size %d", size);
		sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
		return sc_error_code__get_and_clear(p_target);
	} else if (address % size != 0) {
		LOG_ERROR("Unaligned access at 0x%08X, for item size %d", address, size);
		sc_error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
		return sc_error_code__get_and_clear(p_target);
	} else {
		/// Check that target halted
		sc_rv32_check_that_target_halted(p_target);

		if (ERROR_OK != sc_error_code__get(p_target)) {
			return sc_error_code__get_and_clear(p_target);
		}

		/// Reserve work register
		reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);
		assert(p_wrk_reg);

		uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);

		if (ERROR_OK == sc_error_code__get(p_target)) {
			/// Define opcode for load item to register
			uint32_t const load_OP =
				size == 4 ? RISCV_opcode_LW(p_wrk_reg->number, p_wrk_reg->number, 0) :
				size == 2 ? RISCV_opcode_LH(p_wrk_reg->number, p_wrk_reg->number, 0) :
				/*size == 1*/RISCV_opcode_LB(p_wrk_reg->number, p_wrk_reg->number, 0);

			sc_riscv32__Arch const* const p_arch = p_target->arch_info;
			assert(p_arch);
			size_t const instr_step = p_arch->constants->use_pc_advmt_dsbl_bit ? 0u : NUM_BYTES_FOR_BITS(ILEN);

			if (p_arch->constants->use_pc_advmt_dsbl_bit) {
				sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, HART_DBG_CTRL_bit_PC_Advmt_Dsbl);
			}

			/// Setup exec operations mode
			sc_rv32_EXEC__setup(p_target);
			int advance_pc_counter = 0;

			if (ERROR_OK == sc_error_code__get(p_target)) {
				assert(buffer);

				/// For count number of items do loop
				while (count--) {
					/// Set address to CSR
					sc_rv32_EXEC__push_data_to_CSR(p_target, address);

					if (ERROR_OK != sc_error_code__get(p_target)) {
						break;
					}

					/// Load address to work register
					(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_wrk_reg->number, p_arch->constants->debug_scratch_CSR));
					advance_pc_counter += instr_step;

					if (ERROR_OK != sc_error_code__get(p_target)) {
						break;
					}

					/// Exec load item to register
					(void)sc_rv32_EXEC__step(p_target, load_OP);
					advance_pc_counter += instr_step;

					if (ERROR_OK != sc_error_code__get(p_target)) {
						break;
					}

					/// Exec store work register to csr
					(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRW(p_arch->constants->debug_scratch_CSR, p_wrk_reg->number));
					advance_pc_counter += instr_step;

					/// get data from csr and jump back to correct pc
					assert(advance_pc_counter % NUM_BYTES_FOR_BITS(ILEN) == 0);
					uint32_t const value = sc_rv32_EXEC__step(p_target, RISCV_opcode_JAL(0, -advance_pc_counter));
					advance_pc_counter = 0;

					if (ERROR_OK != sc_error_code__get(p_target)) {
						break;
					}

					/// store read data to buffer
					buf_set_u32(buffer, 0, CHAR_BIT * size, value);

					/// advance src/dst pointers
					address += size;
					buffer += size;
				}
			}

			if (p_arch->constants->use_pc_advmt_dsbl_bit) {
				sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, 0);
			}
		} else {
			sc_riscv32__update_status(p_target);
		}

		if (ERROR_OK != sc_error_code__get(p_target)) {
			sc_riscv32__update_status(p_target);
		}

		sc_rv32_check_PC_value(p_target, pc_sample_1);

		/// restore temporary register
		error_code const old_err_code = sc_error_code__get_and_clear(p_target);
		sc_error_code__update(p_target, reg_x__store(p_wrk_reg));
		sc_error_code__prepend(p_target, old_err_code);
		assert(!p_wrk_reg->dirty);

		return sc_error_code__get_and_clear(p_target);
	}
}

error_code
sc_riscv32__write_phys_memory(target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t const* buffer)
{
	LOG_DEBUG("Write_memory at 0x%08X, %d items, each %d bytes, total %d bytes", address, count, size, count * size);

	/// Check for size
	if (!(size == 1 || size == 2 || size == 4)) {
		LOG_ERROR("Invalid item size %d", size);
		sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
		return sc_error_code__get_and_clear(p_target);
	}

	/// Check for alignment
	if (address % size != 0) {
		LOG_ERROR("Unaligned access at 0x%08X, for item size %d", address, size);
		sc_error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
		return sc_error_code__get_and_clear(p_target);
	}

	if (count == 0) {
		return sc_error_code__get_and_clear(p_target);
	}

	sc_rv32_check_that_target_halted(p_target);

	if (ERROR_OK != sc_error_code__get(p_target)) {
		return sc_error_code__get_and_clear(p_target);
	}

	uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);
	/// Reserve work register
	reg* const p_addr_reg = prepare_temporary_GP_register(p_target, 0);
	assert(p_addr_reg);
	reg* const p_data_reg = prepare_temporary_GP_register(p_target, p_addr_reg->number);
	assert(p_data_reg);
	assert(p_addr_reg->number != p_data_reg->number);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		sc_riscv32__Arch const* const p_arch = p_target->arch_info;
		assert(p_arch);
		size_t const instr_step = p_arch->constants->use_pc_advmt_dsbl_bit ? 0u : NUM_BYTES_FOR_BITS(ILEN);

		if (p_arch->constants->use_pc_advmt_dsbl_bit) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, HART_DBG_CTRL_bit_PC_Advmt_Dsbl);
		}

		/// Setup exec operations mode
		sc_rv32_EXEC__setup(p_target);
		size_t advance_pc_counter = 0;

		if (ERROR_OK == sc_error_code__get(p_target)) {
			// Set address to CSR
			sc_rv32_EXEC__push_data_to_CSR(p_target, address);

			if (sc_error_code__get(p_target) == ERROR_OK) {
				/// Load address to work register
				(void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_addr_reg->number, p_arch->constants->debug_scratch_CSR));
				advance_pc_counter += instr_step;

				// Opcodes
				uint32_t const instructions[3] = {
					RISCV_opcode_CSRR(p_data_reg->number, p_arch->constants->debug_scratch_CSR),
					(size == 4 ? RISCV_opcode_SW(p_data_reg->number, p_addr_reg->number, 0) :
					 size == 2 ? RISCV_opcode_SH(p_data_reg->number, p_addr_reg->number, 0) :
					 /*size == 1*/ RISCV_opcode_SB(p_data_reg->number, p_addr_reg->number, 0)),
					RISCV_opcode_ADDI(p_addr_reg->number, p_addr_reg->number, size)
				};

				static uint32_t max_pc_offset = (((1u << 20) - 1u) / NUM_BYTES_FOR_BITS(XLEN)) * NUM_BYTES_FOR_BITS(XLEN);

				if (p_arch->constants->use_queuing_for_dr_scans) {
					uint8_t DAP_OPSTATUS = 0;
					uint8_t const data_wr_opcode[1] = {DBGDATA_WR_index};
					static uint8_t const DAP_OPSTATUS_GOOD = DAP_status_good;
					static uint8_t const DAP_STATUS_MASK = DAP_status_MASK;
					scan_field const data_scan_opcode_field = {
						.num_bits = TAP_length_of_DAP_CMD_OPCODE,
						.out_value = data_wr_opcode,
						.in_value = &DAP_OPSTATUS,
						.check_value = &DAP_OPSTATUS_GOOD,
						.check_mask = &DAP_STATUS_MASK,
					};
					scan_field data_scan_fields[TAP_number_of_fields_DAP_CMD] = {{.num_bits = TAP_length_of_DAP_CMD_OPCODE_EXT}, data_scan_opcode_field,};
					uint8_t const instr_exec_opcode[1] = {CORE_EXEC_index};
					scan_field const instr_scan_opcode_field = {
						.num_bits = TAP_length_of_DAP_CMD_OPCODE,
						.out_value = instr_exec_opcode,
						.in_value = &DAP_OPSTATUS,
						.check_value = &DAP_OPSTATUS_GOOD,
						.check_mask = &DAP_STATUS_MASK,
					};
					uint8_t instr_buf[ARRAY_LEN(instructions)][sizeof(uint32_t)];
					scan_field instr_fields[ARRAY_LEN(instructions)][TAP_number_of_fields_DAP_CMD];

					for (size_t i = 0; i < ARRAY_LEN(instructions); ++i) {
						buf_set_u32(instr_buf[i], 0, TAP_length_of_DAP_CMD_OPCODE_EXT, instructions[i]);
						scan_field const fld = {.num_bits = TAP_length_of_DAP_CMD_OPCODE_EXT,.out_value = instr_buf[i]};
						instr_fields[i][0] = fld;
						instr_fields[i][1] = instr_scan_opcode_field;
					}

					assert(buffer);
					data_scan_fields[0].out_value = buffer;
					LOG_DEBUG("Repeat in loop %d times:", count);
					LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X", p_target->cmd_name,
							  data_scan_fields[0].num_bits, buf_get_u32(data_scan_fields[0].out_value, 0, data_scan_fields[0].num_bits),
							  data_scan_fields[1].num_bits, buf_get_u32(data_scan_fields[1].out_value, 0, data_scan_fields[1].num_bits));

					for (unsigned i = 0; i < ARRAY_LEN(instr_fields); ++i) {
						LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X", p_target->cmd_name,
								  instr_fields[i][0].num_bits, buf_get_u32(instr_fields[i][0].out_value, 0, instr_fields[i][0].num_bits),
								  instr_fields[i][1].num_bits, buf_get_u32(instr_fields[i][1].out_value, 0, instr_fields[i][1].num_bits));
					}

					size_t count1 = 0;

					while (sc_error_code__get(p_target) == ERROR_OK && count--) {
						assert(p_target->tap);
						data_scan_fields[0].out_value = buffer;
						jtag_add_dr_scan_check(p_target->tap, ARRAY_LEN(data_scan_fields), data_scan_fields, TAP_IDLE);

						for (unsigned i = 0; i < ARRAY_LEN(instr_fields); ++i) {
							jtag_add_dr_scan_check(p_target->tap, TAP_number_of_fields_DAP_CMD, instr_fields[i], TAP_IDLE);
							advance_pc_counter += instr_step;
						}

						buffer += size;

						if (++count1 >= WRITE_BUFFER_THRESHOLD) {
							LOG_DEBUG("Force jtag_execute_queue_noclear()");
							jtag_execute_queue_noclear();
							count1 = 0;
						}
					}

					LOG_DEBUG("End loop");

					if (!p_arch->constants->use_pc_advmt_dsbl_bit) {
						assert(advance_pc_counter % NUM_BYTES_FOR_BITS(XLEN) == 0);

						while (advance_pc_counter) {
							uint32_t const step_back = advance_pc_counter > max_pc_offset ? max_pc_offset : advance_pc_counter;
							advance_pc_counter -= step_back;
							assert(advance_pc_counter % NUM_BYTES_FOR_BITS(XLEN) == 0);
							uint8_t OP_correct_pc[4];
							buf_set_u32(OP_correct_pc, 0, TAP_length_of_DAP_CMD_OPCODE_EXT, RISCV_opcode_JAL(0, -(int)(step_back)));
							scan_field const instr_pc_correct_fields[2] = {{.num_bits = TAP_length_of_DAP_CMD_OPCODE_EXT,.out_value = OP_correct_pc}, instr_scan_opcode_field};
							LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X", p_target->cmd_name,
									  instr_pc_correct_fields[0].num_bits, buf_get_u32(instr_pc_correct_fields[0].out_value, 0, instr_pc_correct_fields[0].num_bits),
									  instr_pc_correct_fields[1].num_bits, buf_get_u32(instr_pc_correct_fields[1].out_value, 0, instr_pc_correct_fields[1].num_bits));
							jtag_add_dr_scan_check(p_target->tap, TAP_number_of_fields_DAP_CMD, instr_pc_correct_fields, TAP_IDLE);
						}

						assert(advance_pc_counter == 0);
					}

					sc_error_code__update(p_target, jtag_execute_queue());

					if ((DAP_OPSTATUS & DAP_status_MASK) != DAP_status_good) {
						LOG_ERROR("DAP_OPSTATUS == 0x%1X", (uint32_t)DAP_OPSTATUS);
						sc_error_code__update(p_target, ERROR_TARGET_FAILURE);
					}
				} else {
					while (ERROR_OK == sc_error_code__get(p_target) && count--) {
						/// Set data to CSR
						sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(buffer, 0, CHAR_BIT * size));

						if (ERROR_OK != sc_error_code__get(p_target)) {
							break;
						}

						for (unsigned i = 0; i < ARRAY_LEN(instructions); ++i) {
							(void)sc_rv32_EXEC__step(p_target, instructions[i]);

							if (ERROR_OK != sc_error_code__get(p_target)) {
								break;
							}

							advance_pc_counter += instr_step;
						}

						buffer += size;
					}

					if (!p_arch->constants->use_pc_advmt_dsbl_bit) {
						assert(advance_pc_counter % NUM_BYTES_FOR_BITS(XLEN) == 0);

						while ((ERROR_OK == sc_error_code__get(p_target)) && (advance_pc_counter != 0)) {
							uint32_t const step_back = advance_pc_counter > max_pc_offset ? max_pc_offset : advance_pc_counter;
							advance_pc_counter -= step_back;
							assert(advance_pc_counter % NUM_BYTES_FOR_BITS(XLEN) == 0);
							uint32_t const OP_correct_pc = RISCV_opcode_JAL(0, -(int)(step_back));
							(void)sc_rv32_EXEC__step(p_target, OP_correct_pc);
						}
					}
				}
			}
		}

		if (p_arch->constants->use_pc_advmt_dsbl_bit) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, HART_DBG_CTRL_index, 0);
		}
	} else {
		sc_riscv32__update_status(p_target);
	}

	if (ERROR_OK != sc_error_code__get(p_target)) {
		sc_riscv32__update_status(p_target);
	}

	sc_rv32_check_PC_value(p_target, pc_sample_1);

	/// restore temporary registers
	error_code const old_err_code = sc_error_code__get_and_clear(p_target);
	error_code const new_err_code_1 = reg_x__store(p_data_reg);
	assert(!p_data_reg->dirty);
	error_code const new_err_code_2 = reg_x__store(p_addr_reg);
	assert(!p_addr_reg->dirty);
	sc_error_code__update(p_target, old_err_code);
	sc_error_code__update(p_target, new_err_code_1);
	sc_error_code__update(p_target, new_err_code_2);

	return sc_error_code__get_and_clear(p_target);
}

error_code
sc_riscv32__add_breakpoint(target* const p_target, breakpoint* const p_breakpoint)
{
	assert(p_breakpoint);

	if (p_breakpoint->type != BKPT_SOFT) {
		LOG_ERROR("Only software breakpoins available now");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	sc_rv32_check_that_target_halted(p_target);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		bool const RVC_enable = is_RVC_enable(p_target);

		if (!(p_breakpoint->length == NUM_BYTES_FOR_BITS(ILEN) || (RVC_enable && p_breakpoint->length == 2))) {
			LOG_ERROR("Invalid breakpoint size: %d", p_breakpoint->length);
			sc_error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
		} else if (p_breakpoint->address % (RVC_enable ? 2 : 4) != 0) {
			LOG_ERROR("Unaligned breakpoint: 0x%08X", p_breakpoint->address);
			sc_error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
		} else {
			read_memory_space(p_target, p_breakpoint->address, 2, p_breakpoint->length / 2, p_breakpoint->orig_instr, true);

			if (ERROR_OK != sc_error_code__get(p_target)) {
				LOG_ERROR("Can't save original instruction");
			} else {
				uint8_t buffer[4];

				if (p_breakpoint->length == 4) {
					target_buffer_set_u32(p_target, buffer, RISCV_opcode_EBREAK());
				} else if (p_breakpoint->length == 2) {
					target_buffer_set_u16(p_target, buffer, RISCV_opcode_C_EBREAK());
				} else {
					assert(/*logic_error:Bad breakpoint size*/ 0);
				}

				write_memory_space(p_target, p_breakpoint->address, 2, p_breakpoint->length / 2, buffer, true);

				if (ERROR_OK != sc_error_code__get(p_target)) {
					LOG_ERROR("Can't write EBREAK");
				} else {
					p_breakpoint->set = 1;
				}
			}
		}
	}

	return sc_error_code__get_and_clear(p_target);
}

error_code
sc_riscv32__remove_breakpoint(target* const p_target, breakpoint* const p_breakpoint)
{
	sc_rv32_check_that_target_halted(p_target);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		assert(p_breakpoint);
		assert(p_breakpoint->orig_instr);
#if 0
		LOG_INFO("Remove breakpoint at 0x%08x, length=%d (0x%08x)",
				 p_breakpoint->address,
				 p_breakpoint->length,
				 buf_get_u32(p_breakpoint->orig_instr, 0, p_breakpoint->length * CHAR_BIT));
#endif
		write_memory_space(p_target, p_breakpoint->address, 2, p_breakpoint->length / 2, p_breakpoint->orig_instr, true);

		if (ERROR_OK == sc_error_code__get(p_target)) {
			p_breakpoint->set = 0;
		} else {
			sc_riscv32__update_status(p_target);
		}
	} else {
		sc_riscv32__update_status(p_target);
	}

	return sc_error_code__get_and_clear(p_target);
}

/// gdb_server expects valid reg values and will use set method for updating reg values
error_code
sc_riscv32__get_gdb_reg_list(target* const p_target, reg** reg_list[], int* const reg_list_size, target_register_class const reg_class)
{
	assert(p_target);
	assert(reg_list_size);
	assert(reg_class == REG_CLASS_ALL || reg_class == REG_CLASS_GENERAL);

	size_t const num_regs = reg_class == REG_CLASS_ALL ? number_of_regs_GDB : number_of_regs_GP;
	reg** const p_reg_array = calloc(num_regs, sizeof(reg*));
	reg** p_reg_iter = p_reg_array;
	size_t regs_left = num_regs;

	for (reg_cache* p_reg_cache = p_target->reg_cache; p_reg_cache && regs_left; p_reg_cache = p_reg_cache->next) {
		reg* p_reg_list = p_reg_cache->reg_list;

		for (size_t i = 0; i < p_reg_cache->num_regs && regs_left; ++i, --regs_left) {
			*p_reg_iter++ = &p_reg_list[i];
		}
	}

	// out results
	*reg_list = p_reg_array;
	*reg_list_size = num_regs - regs_left;
	return sc_error_code__get_and_clear(p_target);
}

error_code
sc_riscv32__virt2phys(target* p_target, uint32_t address, uint32_t* p_physical)
{
	sc_riscv32__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);
	p_arch->constants->virt_to_phis(p_target, address, p_physical, NULL, false);
	return sc_error_code__get_and_clear(p_target);
}

