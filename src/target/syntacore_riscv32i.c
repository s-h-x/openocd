/**
@file

Syntacore RISC-V target

@author Pavel S. Smirnov
@copyright Syntacore
*/
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
#include <limits.h>

/// Don't make irscan if IR uis the same
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
#define USE_RESUME_AT_SW_BREAKPOINT_EMULATES_SAVED_INSTRUCTION 1
#define USE_PC_ADVMT_DSBL_BIT 0
#define USE_QUEUING_FOR_DR_SCANS 1
#define USE_CHECK_PC_UNCHANGED USE_PC_FROM_PC_SAMPLE
/// TAP IDCODE expected
#define EXPECTED_IDCODE (0xC0DEDEB1u)

/// Lowest required DBG_ID
#define EXPECTED_DBG_ID        (0x00800001u)
/// Mask of DBG_ID version.
/// Required and provided masked values should be equal.
#define DBG_ID_VERSION_MASK    (0xFFFFFF00u)
/// Mask of DBG_ID subversion.
/// Required value should be less or equal to provided subversion.
#define DBG_ID_SUBVERSION_MASK (0x000000FFu)

/// Utility macros
///@{
#define STATIC_ASSERT(e) {enum {___my_static_assert = 1 / (!!(e)) };}
#define ARRAY_LEN(arr) (sizeof (arr) / sizeof (arr)[0])

#define BIT_NUM_TO_MASK(bit_num) (1u << (bit_num))
#define LOW_BITS_MASK(n) (~(~0 << (n)))
#define NUM_BITS_TO_SIZE(num_bits) ( ( (size_t)(num_bits) + (8 - 1) ) / 8 )
///@}

#define EXTRACT_FIELD(bits, first_bit, last_bit) (((bits) >> (first_bit)) & LOW_BITS_MASK((last_bit) + 1u - (first_bit)))
#define MAKE_TYPE_FIELD(TYPE, bits, first_bit, last_bit)     ((((TYPE)(bits)) & LOW_BITS_MASK((last_bit) + 1u - (first_bit))) << (first_bit))

#define RV_INSTR_R_TYPE(func7, rs2, rs1, func3, rd, opcode) ( \
    MAKE_TYPE_FIELD(instr_type, (func7), 25, 31) | \
    MAKE_TYPE_FIELD(instr_type, (rs2),   20, 24) | \
    MAKE_TYPE_FIELD(instr_type, (rs1),   15, 19) | \
    MAKE_TYPE_FIELD(instr_type, (func3), 12, 14) | \
    MAKE_TYPE_FIELD(instr_type, (rd),     7, 11) | \
    MAKE_TYPE_FIELD(instr_type, (opcode), 0,  6))

#define RV_INSTR_I_TYPE(imm_11_00, rs1, func3, rd, opcode) ( \
    MAKE_TYPE_FIELD(instr_type, EXTRACT_FIELD((imm_11_00), 0, 11), 20, 31) | \
    MAKE_TYPE_FIELD(instr_type, (rs1),   15, 19) | \
    MAKE_TYPE_FIELD(instr_type, (func3), 12, 14) | \
    MAKE_TYPE_FIELD(instr_type, (rd),     7, 11) | \
    MAKE_TYPE_FIELD(instr_type, (opcode), 0,  6))

#define RV_INSTR_S_TYPE(imm_11_00, rs2, rs1, func3, opcode) ( \
    MAKE_TYPE_FIELD(instr_type, EXTRACT_FIELD((imm_11_00), 5, 11), 25, 31) | \
    MAKE_TYPE_FIELD(instr_type, (rs2),                                20, 24) | \
    MAKE_TYPE_FIELD(instr_type, (rs1),                                15, 19) | \
    MAKE_TYPE_FIELD(instr_type, (func3),                              12, 14) | \
    MAKE_TYPE_FIELD(instr_type, EXTRACT_FIELD((imm_11_00), 0,  4),  7, 11) | \
    MAKE_TYPE_FIELD(instr_type, (opcode),                              0,  6))

#define RV_INSTR_SB_TYPE(imm_01_12, rs2, rs1, func3, opcode) ( \
    MAKE_TYPE_FIELD(instr_type, EXTRACT_FIELD((imm_01_12), 12, 12), 31, 31) | \
    MAKE_TYPE_FIELD(instr_type, EXTRACT_FIELD((imm_01_12),  5, 10), 25, 30) | \
    MAKE_TYPE_FIELD(instr_type, (rs2),                                 20, 24) | \
    MAKE_TYPE_FIELD(instr_type, (rs1),                                 15, 19) | \
    MAKE_TYPE_FIELD(instr_type, (func3),                               12, 14) | \
    MAKE_TYPE_FIELD(instr_type, EXTRACT_FIELD((imm_01_12),  1,  4),  8, 11) | \
    MAKE_TYPE_FIELD(instr_type, EXTRACT_FIELD((imm_01_12), 11, 11),  7,  7) | \
    MAKE_TYPE_FIELD(instr_type, (opcode),                               0,  6))

#define RV_INSTR_U_TYPE(imm_31_12, rd, opcode) ( \
    MAKE_TYPE_FIELD(instr_type, EXTRACT_FIELD((imm_31_12), 12, 31), 12, 31) | \
    MAKE_TYPE_FIELD(instr_type, (rd),                                   7, 11) | \
    MAKE_TYPE_FIELD(instr_type, (opcode),                               0,  6))

#define RV_INSTR_UJ_TYPE(imm_20_01, rd, opcode) ( \
    MAKE_TYPE_FIELD(instr_type, EXTRACT_FIELD((imm_20_01), 20, 20), 31, 31) | \
    MAKE_TYPE_FIELD(instr_type, EXTRACT_FIELD((imm_20_01),  1, 10), 21, 30) | \
    MAKE_TYPE_FIELD(instr_type, EXTRACT_FIELD((imm_20_01), 11, 11), 20, 20) | \
    MAKE_TYPE_FIELD(instr_type, EXTRACT_FIELD((imm_20_01), 12, 19), 12, 19) | \
    MAKE_TYPE_FIELD(instr_type, (rd),                                   7, 11) | \
    MAKE_TYPE_FIELD(instr_type, (opcode),                               0,  6))

/// RISC-V opcodes
///@{
#define RV_ADD(rd, rs1, rs2)        RV_INSTR_R_TYPE(0x00u, (rs2), (rs1), 0u, (rd), 0x33u)
#define RV_FMV_X_S(rd, rs1)         RV_INSTR_R_TYPE(0x70u, 0u,    (rs1), 0u, (rd), 0x53u)
#define RV_FMV_S_X(rd, rs1)         RV_INSTR_R_TYPE(0x78u, 0u,    (rs1), 0u, (rd), 0x53u)

#define RV_LB(rd, base, imm)        RV_INSTR_I_TYPE((imm), (base), 0u, (rd), 0x03u)
#define RV_LH(rd, base, imm)        RV_INSTR_I_TYPE((imm), (base), 1u, (rd), 0x03u)
#define RV_LW(rd, base, imm)        RV_INSTR_I_TYPE((imm), (base), 2u, (rd), 0x03u)
#define RV_LBU(rd, base, imm)       RV_INSTR_I_TYPE((imm), (base), 4u, (rd), 0x03u)
#define RV_LHU(rd, base, imm)       RV_INSTR_I_TYPE((imm), (base), 5u, (rd), 0x03u)

#define RV_ADDI(rd, rs1, imm)       RV_INSTR_I_TYPE((imm), (rs1),  0u, (rd), 0x13u)
#define RV_JALR(rd, rs1, imm)       RV_INSTR_I_TYPE((imm), (rs1),  0u, (rd), 0x67u)

#define RV_CSRRW(rd, csr, rs1)      RV_INSTR_I_TYPE((csr), (rs1),  1u, (rd), 0x73u)
#define RV_CSRRS(rd, csr, rs1)      RV_INSTR_I_TYPE((csr), (rs1),  2u, (rd), 0x73u)
#define RV_CSRRC(rd, csr, rs1)      RV_INSTR_I_TYPE((csr), (rs1),  3u, (rd), 0x73u)
#define RV_CSRRWI(rd, csr, zimm)    RV_INSTR_I_TYPE((csr), (zimm), 5u, (rd), 0x73u)
#define RV_CSRRSI(rd, csr, zimm)    RV_INSTR_I_TYPE((csr), (zimm), 6u, (rd), 0x73u)
#define RV_CSRRCI(rd, csr, zimm)    RV_INSTR_I_TYPE((csr), (zimm), 7u, (rd), 0x73u)

#define RV_SBREAK()                 RV_INSTR_I_TYPE(1u, 0u, 0u, 0u, 0x73u)

#define RV_SB(rs, base, imm)        RV_INSTR_S_TYPE((imm), (rs), (base), 0u, 0x23)
#define RV_SH(rs, base, imm)        RV_INSTR_S_TYPE((imm), (rs), (base), 1u, 0x23)
#define RV_SW(rs, base, imm)        RV_INSTR_S_TYPE((imm), (rs), (base), 2u, 0x23)

#define RV_AUIPC(rd, imm)           RV_INSTR_U_TYPE((imm), (rd), 0x17u)

#define RV_JAL(rd, imm_20_01)       RV_INSTR_UJ_TYPE((imm_20_01), (rd), 0x6Fu)

#define RV_NOP()                    RV_ADDI(zero, zero, 0u)
///@]

/// RISC-V GP registers id
enum
{
	zero = 0u,
	sp = 14u,
	REG_PC_NUMBER = 32u,
	TOTAL_NUMBER_OF_REGS = 32 + 1 + 32,
};

/// Type of instruction
typedef uint32_t instr_type;

enum arch_bits_numbers
{
	/// Size of RISC-V GP registers in bits
	XLEN = 32u,
	/// Size of RISC-V FP registers in bits
	FLEN = 32u,
	/// Size of RISC-V instruction
	ILEN = 32u,
};

enum RISCV_CSR
{
	/// User Floating-Point CSRs
	/// privilege: URW
	/// @addtogroup floating_point_csrs
	///@{
	/// Floating-Point Accrued Exceptions.
	CSR_fflags = 0x001u,
	/// Floating-Point Dynamic Rounding Mode.
	CSR_frm = 0x002u,
	/// Floating-Point Control and Status Register (frm + fflags).
	CSR_fcsr = 0x003u,
	///@}

	/// User Counter/Timers
	/// privilege: URO
	///@{
	/// Cycle counter for RDCYCLE instruction
	CSR_cycle = 0xC00u,
	/// Timer for RDTIME instruction
	CSR_time = 0xC01u,
	/// Instructions-retired counter for RDINSTRET instruction
	CSR_instret = 0xC02u,
	/// Upper 32 bits of cycle, RV32I onl
	CSR_cycleh = 0xC80u,
	/// Upper 32 bits of time, RV32I only
	CSR_timeh = 0xC81u,
	///  Upper 32 bits of instret, RV32I only.
	CSR_instreth = 0xC82u,
	///@}

	/// Supervisor Trap Setup
	///@{
	/// Supervisor status register
	CSR_sstatus = 0x100u,
	/// Supervisor trap handler base address
	CSR_stvec = 0x101u,
	/// Supervisor interrupt-enable register
	CSR_sie = 0x104u,
	/// Wall-clock timer compare val
	CSR_stimecmp = 0x121u,
	///@}

	/// Supervisor Timer
	///@{
	/// Supervisor wall-clock time register.
	CSR_stime = 0xD01u,
	/// Upper 32 bits of stime, RV32I onl
	CSR_stimeh = 0xD81u,
	///@}

	/// Supervisor Trap Handling
	///@{
	/// Scratch register for supervisor trap handlers
	CSR_sscratch = 0x140u,
	/// Supervisor exception program counter.
	CSR_sepc = 0x141u,
	/// Supervisor trap cause.
	CSR_scause = 0xD42u,
	/// Supervisor bad address.
	CSR_sbadaddr = 0xD43u,
	/// Supervisor interrupt pending
	CSR_sip = 0x144u,
	///@}

	/// Supervisor Protection and Translation
	///@{
	/// Page-table base register
	CSR_sptbr = 0x180u,
	/// Address-space ID.
	CSR_sasid = 0x181u,
	///@}

	/// Supervisor Read/Write Shadow of User Read-Only registers
	///@{
	/// Cycle counter for RDCYCLE instruction.
	CSR_cyclew = 0x900u,
	/// Timer for RDTIME instruction.
	CSR_timew = 0x901u,
	/// Instructions-retired counter for RDINSTRET instruction
	CSR_instretw = 0x902u,
	/// Upper 32 bits of cycle, RV32I only
	CSR_cyclehw = 0x980u,
	/// Upper 32 bits of time, RV32I only
	CSR_timehw = 0x981u,
	/// Upper 32 bits of CSR_instretw, RV32I only
	CSR_instrethw = 0x982u,
	///@}

	/// Hypervisor Trap Setup
	///@{
	/// Hypervisor status register
	CSR_hstatus = 0x200u,
	/// Hypervisor trap handler base address
	CSR_htvec = 0x201u,
	/// Hypervisor trap delegation register.
	CSR_htdeleg = 0x202u,
	/// Hypervisor wall-clock timer compare value.
	CSR_htimecmp = 0x221u,
	///@}

	/// Hypervisor Timer
	///@{
	CSR_htime = 0xE01u,
	CSR_htimeh = 0xE81u,
	///@}

	/// Hypervisor Trap Handling
	///@{
	CSR_hscratch = 0x240u,
	CSR_hepc = 0x241u,
	CSR_hcause = 0x242u,
	CSR_hbadaddr = 0x243u,
	///@}

	/// Hypervisor Read/Write Shadow of Supervisor Read-Only Registers
	///@{
	CSR_stimew = 0xA01u,
	CSR_stimehw = 0xA81u,
	///@}

	/// Machine Information Registers
	/// privilege: MRO
	///@{
	/// @brief CPU description
	CSR_mcpuid = 0xF00u,

	/// @brief Vendor ID and version number
	CSR_mimpid = 0xF01u,

	/// @brief Hardware thread ID
	CSR_mhartid = 0xF10u,
	///@}

	/// Machine Trap Setup
	/// privilege: MRW
	///@{

	/// @brief Machine status register
	///< FS - bits 13:12
	CSR_mstatus = 0x300u,

	/// @brief Machine trap-handler base address
	CSR_mtvec = 0x301u,

	/// @brief Machine trap delegation register
	CSR_mtdeleg = 0x302u,

	/// @brief Machine interrupt-enable register
	CSR_mie = 0x304u,

	/// @brief Machine wall-clock timer compare value
	CSR_mtimecmp = 0x321u,
	///@}

	/// Machine Timers and Counters
	/// privilege: MRW
	///@{

	/// @brief Machine wall-clock time
	CSR_mtime = 0x701u,

	/// @brief Upper 32 bits of mtime, RV32I only
	CSR_mtimeh = 0x741u,
	///@}

	/// Machine Trap Handling
	/// privilege: MRW
	///@{

	/// @brief Scratch register for machine trap handlers.
	CSR_mscratch = 0x340u,
	/// @brief Machine exception program counter
	CSR_mepc = 0x341u,
	/// @brief Machine trap cause
	CSR_mcause = 0x342u,
	/// @brief Machine bad address
	CSR_mbadaddr = 0x343u,
	/// @brief Machine interrupt pending.
	CSR_mip = 0x344u,
	///@}

	/// Machine Protection and Translation
	/// privilege: MRW
	///@{

	/// @brief Base register
	CSR_mbase = 0x380u,

	/// @brief Base register
	CSR_mbound = 0x381u,

	/// @brief Bound register.
	CSR_mibase = 0x382u,

	/// @brief Instruction base register.
	CSR_mibound = 0x383u,

	/// @brief Data base register
	CSR_mdbase = 0x384u,

	/// @brief Data bound register
	CSR_mdbound = 0x385u,
	///@}

	/// Machine Read-Write Shadow of Hypervisor Read-Only Registers
	/// privilege: MRW
	///@{

	/// @brief Hypervisor wall-clock timer
	CSR_htimew = 0xB01u,

	/// @brief Upper 32 bits of hypervisor wall-clock timer, RV32I only.
	CSR_htimehw = 0xB81u,
	///@}

	/// Machine Host-Target Interface (Non-Standard Berkeley Extension)
	/// privilege: MRW
	///@{
	/// @brief Output register to host
	CSR_mtohost = 0x780u,
	/// @brief Input register from host.
	CSR_mfromhost = 0x781u,
	///@}

	/// Debug CSR number
	/// privilege: MRW
	CSR_DBG_SCRATCH = 0x788u,
};

typedef enum mstatus_context_field_e
{
	ext_off = 0,
	ext_initial = 1,
	ext_clean = 2,
	ext_dirty = 3,
} mstatus_context_field_e;

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

enum
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

typedef struct reg_cache reg_cache;
typedef struct reg_arch_type reg_arch_type;
typedef struct target_type target_type;
typedef struct scan_field scan_field;
typedef struct target target;
typedef struct reg reg;

struct sc_rv32i__Arch
{
	int error_code;
	uint8_t last_DAP_ctrl;
	bool use_ir_select_cache;
	bool use_dap_control_cache;
	bool use_verify_dap_control;
	bool use_pc_from_pc_sample;
	bool use_check_pc_unchanged;
	bool use_verify_hart_regtrans_write;
	bool use_verify_core_regtrans_write;
	bool use_resume_at_sw_breakpoint_emulates_saved_instruction;
	bool use_pc_advmt_dsbl_bit;
	bool use_queuing_for_dr_scans;
};
typedef struct sc_rv32i__Arch sc_rv32i__Arch;
enum
{
	DAP_CTRL_INVALID_CODE = 0xFFu,
};

static sc_rv32i__Arch const initial_arch = {
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

static uint8_t const DAP_OPSTATUS_GOOD = DAP_OPSTATUS_OK;
static uint8_t const DAP_STATUS_MASK = DAP_OPSTATUS_MASK;

/// Error code handling
///@{
static int
error_code__get(target const* const restrict p_target)
{
	assert(p_target);
	sc_rv32i__Arch const* const restrict p_arch = p_target->arch_info;
	assert(p_arch);
	return p_arch->error_code;
}

static int
error_code__update(target const* const restrict p_target, int const a_error_code)
{
	assert(p_target);
	sc_rv32i__Arch* const restrict p_arch = p_target->arch_info;
	assert(p_arch);
	if (ERROR_OK == error_code__get(p_target) && ERROR_OK != a_error_code) {
		LOG_DEBUG("Set new error code: %d", a_error_code);
		p_arch->error_code = a_error_code;
	}
	return error_code__get(p_target);
}

static int
error_code__get_and_clear(target const* const restrict p_target)
{
	assert(p_target);
	sc_rv32i__Arch* const const restrict p_arch = p_target->arch_info;
	assert(p_arch);
	int const result = error_code__get(p_target);
	p_arch->error_code = ERROR_OK;
	return result;
}

static inline int
error_code__prepend(target const* const restrict p_target, int const old_err_code)
{
	assert(p_target);
	int const new_err_code = error_code__get_and_clear(p_target);
	error_code__update(p_target, old_err_code);
	error_code__update(p_target, new_err_code);
	return error_code__get(p_target);
}

///@}

/// TAPs methods
/// @{

/** @brief Always perform scan to write instruction register

Method can update error_code, but ignore previous errors
*/
static void
IR_select_force(target const* const restrict p_target, enum TAP_IR_e const new_instr)
{
	assert(p_target);
	assert(p_target->tap);
	assert(p_target->tap->ir_length == TAP_IR_LEN);
	uint8_t out_buffer[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {};
	buf_set_u32(out_buffer, 0, TAP_IR_LEN, new_instr);
	scan_field field = {
		.num_bits = p_target->tap->ir_length,
		.out_value = out_buffer,
	};
	jtag_add_ir_scan(p_target->tap, &field, TAP_IDLE);
	LOG_DEBUG("irscan %s %d", p_target->cmd_name, new_instr);
#if 0
	// force jtag_execute_queue() here because field reference local variable
	if (error_code__update(p_target, jtag_execute_queue()) != ERROR_OK) {
		LOG_ERROR("Error %d", error_code__get(p_target));
	}
#endif
}

/** @brief Cached version of instruction register

Method can update error_code, but ignore previous errors

*/
static void
IR_select(target const* const restrict p_target, enum TAP_IR_e const new_instr)
{
	assert(p_target);
	sc_rv32i__Arch const* const restrict p_arch = p_target->arch_info;
	assert(p_arch);
	if (p_arch->use_ir_select_cache) {
		assert(p_target->tap);
		assert(p_target->tap->ir_length == TAP_IR_LEN);
		/// Skip IR scan if IR is the same
		if (buf_get_u32(p_target->tap->cur_instr, 0u, p_target->tap->ir_length) == new_instr) {
#if 0
			LOG_DEBUG("IR %s resently selected %d", p_target->cmd_name, new_instr);
#endif
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
static uint32_t
read_only_32_bits_regs(target const* const restrict p_target, enum TAP_IR_e ir)
{
	assert(p_target);
	/// Low-level method save but ignore previous errors.
	int const old_err_code = error_code__get_and_clear(p_target);
	/// Error state can be updated in IR_select
	IR_select(p_target, ir);
	if (error_code__get(p_target) != ERROR_OK) {
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
	if (error_code__get(p_target) != ERROR_OK) {
		/// Error state can be updated in DR scan
		LOG_ERROR("JTAG error %d", error_code__get(p_target));
	}

	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_RO_32) <= sizeof(uint32_t));
	uint32_t const result = buf_get_u32(result_buffer, 0, TAP_LEN_DBG_STATUS);
	LOG_DEBUG("drscan %s %d 0 ; # %08X", p_target->cmd_name, field.num_bits, result);

	error_code__prepend(p_target, old_err_code);
	return result;
}

static inline uint32_t
IDCODE_get(target const* const restrict p_target)
{
	assert(p_target);
	STATIC_ASSERT(TAP_LEN_IDCODE == TAP_LEN_RO_32);
	return read_only_32_bits_regs(p_target, TAP_INSTR_IDCODE);
}

static inline uint32_t
DBG_ID_get(target const* const restrict p_target)
{
	assert(p_target);
	STATIC_ASSERT(TAP_LEN_DBG_ID == TAP_LEN_RO_32);
	return read_only_32_bits_regs(p_target, TAP_INSTR_DBG_ID);
}

static inline uint32_t
BLD_ID_get(target const* const restrict p_target)
{
	assert(p_target);
	STATIC_ASSERT(TAP_LEN_BLD_ID == TAP_LEN_RO_32);
	return read_only_32_bits_regs(p_target, TAP_INSTR_BLD_ID);
}

static inline uint32_t
DBG_STATUS_get(target const* const restrict p_target)
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
	if ((result & result_mask) != (good_result & result_mask)) {
		LOG_WARNING("DBG_STATUS == 0x%08X", result);
	}
	return result;
}

/// set unit/group
static uint8_t
DAP_CTRL_REG_set_force(target const* const restrict p_target, uint8_t const set_dap_unit_group)
{
	assert(p_target);
	int const old_err_code = error_code__get_and_clear(p_target);
	IR_select(p_target, TAP_INSTR_DAP_CTRL);
	if (error_code__get(p_target) != ERROR_OK) {
		error_code__prepend(p_target, old_err_code);
		return 0xF8u;
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

	sc_rv32i__Arch* const restrict p_arch = p_target->arch_info;
	assert(p_arch);

	/// set invalid cache value
	p_arch->last_DAP_ctrl = DAP_CTRL_INVALID_CODE;

	jtag_add_dr_scan_check(p_target->tap, 1, &field, TAP_IDLE);

	// enforce jtag_execute_queue() to get status
	if (error_code__update(p_target, jtag_execute_queue()) != ERROR_OK) {
		LOG_ERROR("JTAG error %d", error_code__get(p_target));
	}
	LOG_DEBUG("drscan %s %d 0x%1X ; # %1X", p_target->cmd_name, field.num_bits, set_dap_unit_group, status);
	if ((status & DAP_OPSTATUS_MASK) != DAP_OPSTATUS_OK) {
		LOG_WARNING("TAP status 0x%1X", (uint32_t)status);
	}

	error_code__prepend(p_target, old_err_code);
	return status;
}

/// verify unit/group
static void
DAP_CTRL_REG_verify(target const* const restrict p_target, uint8_t const set_dap_unit_group)
{
	assert(p_target);
	sc_rv32i__Arch const* const restrict p_arch = p_target->arch_info;
	assert(p_arch);
	if (p_arch->use_verify_dap_control) {
		int const old_err_code = error_code__get_and_clear(p_target);
		IR_select(p_target, TAP_INSTR_DAP_CTRL_RD);
		if (error_code__get(p_target) != ERROR_OK) {
			error_code__prepend(p_target, old_err_code);
			return;
		}
		uint8_t get_dap_unit_group = 0;
		STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CTRL) == sizeof get_dap_unit_group);
		scan_field const field =
		{
			.num_bits = TAP_LEN_DAP_CTRL,
			.in_value = &get_dap_unit_group,
		};
		// enforce jtag_execute_queue() to get get_dap_unit_group
		jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);
		error_code__update(p_target, jtag_execute_queue());
		LOG_DEBUG("drscan %s %d 0x%1X ; # %1X", p_target->cmd_name, field.num_bits, 0, get_dap_unit_group);
		if (error_code__get(p_target) != ERROR_OK) {
			LOG_ERROR("JTAG error %d", error_code__get(p_target));
			error_code__prepend(p_target, old_err_code);
			return;
		}
		if (get_dap_unit_group != set_dap_unit_group) {
			LOG_ERROR("Unit/Group verification error: set 0x%1X, but get 0x%1X!", set_dap_unit_group, get_dap_unit_group);
			error_code__update(p_target, ERROR_TARGET_FAILURE);
		}
		error_code__prepend(p_target, old_err_code);
	}
}

static uint32_t
DAP_CMD_scan(target const* const restrict p_target, uint8_t const DAP_OPCODE, uint32_t const DAP_OPCODE_EXT)
{
	assert(p_target);
	int const old_err_code = error_code__get_and_clear(p_target);
	IR_select(p_target, TAP_INSTR_DAP_CMD);
	if (error_code__get(p_target) != ERROR_OK) {
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
	if (error_code__get(p_target) == ERROR_OK) {
		if ((DAP_OPSTATUS & DAP_OPSTATUS_MASK) != DAP_OPSTATUS_OK) {
			LOG_ERROR("DAP_OPSTATUS == 0x%1X", (uint32_t)DAP_OPSTATUS);
			error_code__update(p_target, ERROR_TARGET_FAILURE);
		}
	}

	error_code__prepend(p_target, old_err_code);
	return DBG_DATA;
}

/**
@brief Try to unlock debug controller

@warning Clear previous error_code and set ERROR_TARGET_FAILURE if unlock was unsuccsesful
@return lock context
*/
static uint32_t
debug_controller__unlock(target const* const restrict p_target)
{
	assert(p_target);
	LOG_WARNING("========= Try to unlock ==============");

	sc_rv32i__Arch* const restrict p_arch = p_target->arch_info;
	assert(p_arch);

	uint32_t lock_context = 0xBADC0DEAu;

	assert(p_target->tap);
	assert(p_target->tap->ir_length == TAP_IR_LEN);

	{
		static uint8_t const ir_out_buffer_DAP_CTRL[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {TAP_INSTR_DAP_CTRL};
		/// @todo jtag_add_ir_scan need non-const scan_field
		static scan_field ir_field_DAP_CTRL = {.num_bits = TAP_IR_LEN, .out_value = ir_out_buffer_DAP_CTRL};
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DAP_CTRL);
		jtag_add_ir_scan(p_target->tap, &ir_field_DAP_CTRL, TAP_IDLE);
	}

	{
		static uint8_t const set_dap_unit_group = MAKE_TYPE_FIELD(uint8_t, DBGC_UNIT_ID_HART_0, 2, 3) | MAKE_TYPE_FIELD(uint8_t, DBGC_FGRP_HART_DBGCMD, 0, 1);
		STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CTRL) == sizeof set_dap_unit_group);
		static scan_field const dr_field_DAP_CTRL = {.num_bits = TAP_LEN_DAP_CTRL, .out_value = &set_dap_unit_group};
		p_arch->last_DAP_ctrl = DAP_CTRL_INVALID_CODE;
		LOG_DEBUG("drscan %s %d 0x%1X", p_target->cmd_name, dr_field_DAP_CTRL.num_bits, set_dap_unit_group);
		jtag_add_dr_scan(p_target->tap, 1, &dr_field_DAP_CTRL, TAP_IDLE);
	}

	{
		static uint8_t const ir_out_buffer_DAP_CMD[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {TAP_INSTR_DAP_CMD};
		static scan_field ir_field_DAP_CMD = {.num_bits = TAP_IR_LEN, .out_value = ir_out_buffer_DAP_CMD};
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DAP_CMD);
		jtag_add_ir_scan(p_target->tap, &ir_field_DAP_CMD, TAP_IDLE);
	}

	{
		static uint32_t const dap_opcode_ext_UNLOCK = 0xF0F0A5A5u;
		static uint8_t const dap_opcode_UNLOCK = DBGC_DAP_OPCODE_DBGCMD_UNLOCK;
		scan_field const dr_fields_UNLOCK[2] = {
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT, .out_value = (uint8_t const*)(&dap_opcode_ext_UNLOCK), .in_value = (uint8_t*)(&lock_context)},
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE, .out_value = &dap_opcode_UNLOCK}
		};
		LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X", p_target->cmd_name, dr_fields_UNLOCK[0].num_bits, dap_opcode_ext_UNLOCK, dr_fields_UNLOCK[1].num_bits, dap_opcode_UNLOCK);
		jtag_add_dr_scan(p_target->tap, ARRAY_LEN(dr_fields_UNLOCK), dr_fields_UNLOCK, TAP_IDLE);
		LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X", p_target->cmd_name, dr_fields_UNLOCK[0].num_bits, dap_opcode_ext_UNLOCK, dr_fields_UNLOCK[1].num_bits, dap_opcode_UNLOCK);
		jtag_add_dr_scan(p_target->tap, ARRAY_LEN(dr_fields_UNLOCK), dr_fields_UNLOCK, TAP_IDLE);
	}

	{
		static uint8_t const ir_out_buffer_DBG_STATUS[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {TAP_INSTR_DBG_STATUS};
		static scan_field ir_field_DBG_STATUS = {.num_bits = TAP_IR_LEN, .out_value = ir_out_buffer_DBG_STATUS};
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DBG_STATUS);
		jtag_add_ir_scan(p_target->tap, &ir_field_DBG_STATUS, TAP_IDLE);
	}

	uint32_t status = 0xBADC0DEBu;
	{
		scan_field const dr_field_DBG_STATUS = {.num_bits = TAP_LEN_DBG_STATUS, .out_value = (uint8_t const*)(&status), .in_value = (uint8_t*)(&status)};
		LOG_DEBUG("drscan %s %d 0x%08X", p_target->cmd_name, dr_field_DBG_STATUS.num_bits, status);
		jtag_add_dr_scan(p_target->tap, 1, &dr_field_DBG_STATUS, TAP_IDLE);
	}

	// enforse jtag_execute_queue() to get values
	bool const ok = (error_code__update(p_target, jtag_execute_queue()) == ERROR_OK) && ((status & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_LOCK_BIT)) == 0);
	LOG_DEBUG("%s context=0x%08X, status=0x%08X", ok ? "Unlock succsessful!" : "Unlock unsuccsessful!", lock_context, status);
	return lock_context;
}

static void
HART0_clear_sticky(target* const restrict p_target)
{
	LOG_DEBUG("========= Try to clear HART0 errors ============");
	assert(p_target);
	sc_rv32i__Arch* const restrict p_arch = p_target->arch_info;
	assert(p_arch);
	assert(p_target->tap);
	assert(p_target->tap->ir_length == TAP_IR_LEN);

	{
		uint8_t ir_dap_ctrl_out_buffer[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {};
		buf_set_u32(ir_dap_ctrl_out_buffer, 0, TAP_IR_LEN, TAP_INSTR_DAP_CTRL);
		scan_field ir_dap_ctrl_field = {.num_bits = p_target->tap->ir_length, .out_value = ir_dap_ctrl_out_buffer};
		jtag_add_ir_scan(p_target->tap, &ir_dap_ctrl_field, TAP_IDLE);
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DAP_CTRL);
	}

	{
		/// set invalid cache value
		p_arch->last_DAP_ctrl = DAP_CTRL_INVALID_CODE;

		uint8_t const set_dap_unit_group = 0x1u;
		scan_field const dr_dap_ctrl_field = {.num_bits = TAP_LEN_DAP_CTRL, .out_value = &set_dap_unit_group};
		jtag_add_dr_scan(p_target->tap, 1, &dr_dap_ctrl_field, TAP_IDLE);
	}

	{
		uint8_t ir_dap_cmd_out_buffer[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {};
		buf_set_u32(ir_dap_cmd_out_buffer, 0, TAP_IR_LEN, TAP_INSTR_DAP_CMD);
		scan_field ir_dap_cmd_field = {.num_bits = p_target->tap->ir_length, .out_value = ir_dap_cmd_out_buffer};
		jtag_add_ir_scan(p_target->tap, &ir_dap_cmd_field, TAP_IDLE);
	}

	{
		uint32_t const dap_opcode_ext = BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS);
		uint8_t const dap_opcode = DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL;
		scan_field const fields[2] = {
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT, .out_value = (uint8_t const*)&dap_opcode_ext},
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE, .out_value = &dap_opcode}
		};
		jtag_add_dr_scan(p_target->tap, ARRAY_LEN(fields), fields, TAP_IDLE);
	}
}

static inline uint8_t
REGTRANS_scan_type(bool const write, uint8_t const index)
{
	assert((index & !LOW_BITS_MASK(3)) == 0);
	return MAKE_TYPE_FIELD(uint8_t, !!write, 3, 3) | MAKE_TYPE_FIELD(uint8_t, index, 0, 2);
}

static void
core_clear_errors(target* const restrict p_target)
{
	LOG_DEBUG("========= Try to clear core errors ============");
	assert(p_target);
	sc_rv32i__Arch* const restrict p_arch = p_target->arch_info;
	assert(p_arch);
	assert(p_target->tap);
	assert(p_target->tap->ir_length == TAP_IR_LEN);

	{
		uint8_t ir_dap_ctrl_out_buffer[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {};
		buf_set_u32(ir_dap_ctrl_out_buffer, 0, TAP_IR_LEN, TAP_INSTR_DAP_CTRL);
		scan_field ir_dap_ctrl_field = {.num_bits = p_target->tap->ir_length, .out_value = ir_dap_ctrl_out_buffer};
		jtag_add_ir_scan(p_target->tap, &ir_dap_ctrl_field, TAP_IDLE);
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DAP_CTRL);
	}

	{
		/// set invalid cache value
		p_arch->last_DAP_ctrl = DAP_CTRL_INVALID_CODE;

		uint8_t const set_dap_unit_group = (DBGC_UNIT_ID_CORE << TAP_LEN_DAP_CTRL_FGROUP) | DBGC_FGRP_CORE_REGTRANS;
		scan_field const field = {.num_bits = TAP_LEN_DAP_CTRL, .out_value = &set_dap_unit_group};
		jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);
		LOG_DEBUG("drscan %s %d 0x%1X", p_target->cmd_name, field.num_bits, set_dap_unit_group);
	}

	{
		uint8_t ir_dap_cmd_out_buffer[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {};
		buf_set_u32(ir_dap_cmd_out_buffer, 0, TAP_IR_LEN, TAP_INSTR_DAP_CMD);
		scan_field ir_dap_cmd_field = { .num_bits = p_target->tap->ir_length, .out_value = ir_dap_cmd_out_buffer};
		jtag_add_ir_scan(p_target->tap, &ir_dap_cmd_field, TAP_IDLE);
		LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_INSTR_DAP_CMD);
	}

	{
		uint32_t const dap_opcode_ext = 0xFFFFFFFF;
		uint8_t const dap_opcode = REGTRANS_scan_type(true, DBGC_CORE_REGS_DBG_STS);
		scan_field const fields[2] = {
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT, .out_value = (uint8_t const*)&dap_opcode_ext},
			{.num_bits = TAP_LEN_DAP_CMD_OPCODE, .out_value = &dap_opcode}
		};
		jtag_add_dr_scan(p_target->tap, ARRAY_LEN(fields), fields, TAP_IDLE);
		LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X", p_target->cmd_name, fields[0].num_bits, dap_opcode_ext, fields[1].num_bits, dap_opcode);
	}
}

static void
DAP_CTRL_REG_set(target const* const restrict p_target, enum type_dbgc_unit_id_e const dap_unit, uint8_t const dap_group)
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

	sc_rv32i__Arch* const restrict p_arch = p_target->arch_info;
	assert(p_arch);
	if (p_arch->use_dap_control_cache) {
		if (p_arch->last_DAP_ctrl == set_dap_unit_group) {
#if 0
			LOG_DEBUG("DAP_CTRL_REG of %s already 0x%1X", p_target->cmd_name, set_dap_unit_group);
#endif
			return;
		}
		LOG_DEBUG("DAP_CTRL_REG of %s reset to 0x%1X", p_target->cmd_name, set_dap_unit_group);
	}

	/// Invalidate last_DAP_ctrl
	p_arch->last_DAP_ctrl = DAP_CTRL_INVALID_CODE;

	int const old_err_code = error_code__get_and_clear(p_target);
	uint8_t const status = DAP_CTRL_REG_set_force(p_target, set_dap_unit_group);
	if ((status & DAP_OPSTATUS_MASK) != DAP_OPSTATUS_OK) {
		LOG_WARNING("DAP_CTRL_REG status 0x%1X!", status);
	}

	error_code__get_and_clear(p_target);
	DAP_CTRL_REG_verify(p_target, set_dap_unit_group);
	if (error_code__get(p_target) == ERROR_OK) {
		p_arch->last_DAP_ctrl = set_dap_unit_group;
	} else {
		error_code__update(p_target, ERROR_TARGET_FAILURE);
	}
	error_code__prepend(p_target, old_err_code);
}

static void
REGTRANS_write(target const* const restrict p_target, enum type_dbgc_unit_id_e a_unit, uint8_t const a_fgrp, uint8_t const index, uint32_t const data)
{
	assert(p_target);
	DAP_CTRL_REG_set(p_target, a_unit, a_fgrp);
	if (error_code__get(p_target) != ERROR_OK) {
		return;
	}
	(void)DAP_CMD_scan(p_target, REGTRANS_scan_type(true, index), data);
}

static uint32_t
REGTRANS_read(target const* const restrict p_target, enum type_dbgc_unit_id_e const a_unit, uint8_t const a_fgrp, uint8_t const index)
{
	assert(p_target);
	DAP_CTRL_REG_set(p_target, a_unit, a_fgrp);
	if (error_code__get(p_target) != ERROR_OK) {
		return 0xBADC0DE3u;
	}
	(void)DAP_CMD_scan(p_target, REGTRANS_scan_type(false, index), 0);
	return DAP_CMD_scan(p_target, REGTRANS_scan_type(false, index), 0);
}

static inline void
HART_REGTRANS_write(target const* const restrict p_target, enum type_dbgc_regblock_hart_e const index, uint32_t const data)
{
	assert(p_target);
	REGTRANS_write(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_REGTRANS, index, data);
}

static inline uint32_t
HART_REGTRANS_read(target const* const restrict p_target, enum type_dbgc_regblock_hart_e const index)
{
	assert(p_target);
	return REGTRANS_read(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_REGTRANS, index);
}

static inline void
core_REGTRANS_write(target const* const restrict p_target, enum type_dbgc_regblock_core_e const index, uint32_t const data)
{
	assert(p_target);
	REGTRANS_write(p_target, DBGC_UNIT_ID_CORE, DBGC_FGRP_CORE_REGTRANS, index, data);
}

static inline uint32_t
core_REGTRANS_read(target const* const restrict p_target, enum type_dbgc_regblock_core_e const index)
{
	assert(p_target);
	return REGTRANS_read(p_target, DBGC_UNIT_ID_CORE, DBGC_FGRP_CORE_REGTRANS, index);
}

static inline void
exec__setup(target const* const restrict p_target)
{
	assert(p_target);
	if (error_code__get(p_target) == ERROR_OK) {
		DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_DBGCMD);
	}
}

static inline void
exec__set_csr_data(target const* const restrict p_target, uint32_t const csr_data)
{
	assert(p_target);
	if (error_code__get(p_target) == ERROR_OK) {
		DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_DBGDATA_WR, csr_data);
	}
}

static inline uint32_t
exec__step(target const* const restrict p_target, uint32_t instruction)
{
	assert(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return 0xBADC0DE9u;
	}
	return DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_CORE_EXEC, instruction);
}

static inline uint32_t
get_PC_to_check(target const* const restrict p_target)
{
	assert(p_target);
	sc_rv32i__Arch const* const restrict p_arch = p_target->arch_info;
	assert(p_arch);
	if (p_arch->use_check_pc_unchanged && p_arch->use_pc_from_pc_sample) {
		return HART_REGTRANS_read(p_target, DBGC_HART_REGS_PC_SAMPLE);
	} else {
		return 0xFFFFFFFFu;
	}
}

static inline void
check_PC_unchanged(target const* const restrict p_target, uint32_t const pc_sample_1)
{
	assert(p_target);
	sc_rv32i__Arch const* const restrict p_arch = p_target->arch_info;
	assert(p_arch);
	if (p_arch->use_check_pc_unchanged && p_arch->use_pc_from_pc_sample) {
		uint32_t const pc_sample_2 = get_PC_to_check(p_target);
		if (pc_sample_2 != pc_sample_1) {
			LOG_ERROR("pc changed from 0x%08X to 0x%08X", pc_sample_1, pc_sample_2);
			error_code__update(p_target, ERROR_TARGET_FAILURE);
		}
	}
}
/// @}

static inline int
HART_status_bits_to_target_state(target const* const restrict p_target, uint32_t const status)
{
	assert(p_target);
	static uint32_t const err_bits =
		BIT_NUM_TO_MASK(DBGC_HART_HDSR_ERR_BIT) |
		BIT_NUM_TO_MASK(DBGC_HART_HDSR_ERR_BIT) |
		BIT_NUM_TO_MASK(DBGC_HART_HDSR_ERR_HWTHREAD_BIT) |
		BIT_NUM_TO_MASK(DBGC_HART_HDSR_ERR_DAP_OPCODE_BIT) |
		BIT_NUM_TO_MASK(DBGC_HART_HDSR_ERR_DBGCMD_NACK_BIT) |
		BIT_NUM_TO_MASK(DBGC_HART_HDSR_LOCK_STKY_BIT);

	if (error_code__get(p_target) != ERROR_OK) {
		return TARGET_UNKNOWN;
	} else if (status & err_bits) {
		return TARGET_UNKNOWN;
	} else if (status & BIT_NUM_TO_MASK(DBGC_HART_HDSR_RST_BIT)) {
		return TARGET_RESET;
	} else if (status & BIT_NUM_TO_MASK(DBGC_HART_HDSR_DMODE_BIT)) {
		return TARGET_HALTED;
	} else {
		return TARGET_RUNNING;
	}
}

static enum target_debug_reason
read_debug_cause(target* const restrict p_target)
{
	assert(p_target);
	uint32_t const value = HART_REGTRANS_read(p_target, DBGC_HART_REGS_DMODE_CAUSE);
	if (error_code__get(p_target) != ERROR_OK) {
		return DBG_REASON_UNDEFINED;
	}
	if (value & BIT_NUM_TO_MASK(DBGC_HART_HDMCR_ENFORCE_BIT)) {
		return DBG_REASON_DBGRQ;
	} else if (value & BIT_NUM_TO_MASK(DBGC_HART_HDMCR_SINGLE_STEP_BIT)) {
		return DBG_REASON_SINGLESTEP;
	} else if (value & BIT_NUM_TO_MASK(DBGC_HART_HDMCR_SW_BRKPT_BIT)) {
		return DBG_REASON_BREAKPOINT;
	} else if (value & BIT_NUM_TO_MASK(DBGC_HART_HDMCR_RST_BREAK_BIT)) {
		return DBG_REASON_DBGRQ;
	} else {
		return DBG_REASON_UNDEFINED;
	}
}

static void
update_debug_reason(target* const restrict p_target)
{
	assert(p_target);
	enum target_debug_reason const debug_reason = read_debug_cause(p_target);
	if (debug_reason != p_target->debug_reason) {
		LOG_DEBUG("New debug reason: 0x%08X", (uint32_t)debug_reason);
		p_target->debug_reason = debug_reason;
	}
}

static void
update_status(target* const restrict p_target)
{
	assert(p_target);
	int const old_err_code = error_code__get_and_clear(p_target);
	uint32_t core_status = DBG_STATUS_get(p_target);
	if (0 != (core_status & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_LOCK_BIT))) {
		LOG_WARNING("Lock detected: 0x%08X", core_status);
		uint32_t const lock_context = debug_controller__unlock(p_target);
		if (error_code__get(p_target) != ERROR_OK) {
			/// return with error_code != ERROR_OK if unlock was unsuccsesful
			LOG_ERROR("Unlock unsucsessful with lock_context=0x%8X", lock_context);
			error_code__prepend(p_target, old_err_code);
			return;
		}
		core_status = DBG_STATUS_get(p_target);
		LOG_WARNING("Lock with lock_context=0x%8X fixed: 0x%08X", lock_context, core_status);
	}
	assert(!(core_status & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_LOCK_BIT)));

	uint32_t const hart0_err_bits =
		BIT_NUM_TO_MASK(DBGC_CORE_CDSR_HART0_ERR_BIT) |
		BIT_NUM_TO_MASK(DBGC_CORE_CDSR_HART0_ERR_STKY_BIT);
	if (core_status & hart0_err_bits) {
		LOG_WARNING("Hart errors detected: 0x%08X", core_status);
		HART0_clear_sticky(p_target);
		error_code__get_and_clear(p_target);
		core_status = DBG_STATUS_get(p_target);
		LOG_WARNING("Hart errors %s: 0x%08X", core_status & hart0_err_bits ? "not fixed!" : "fixed", core_status);
	}

	uint32_t const cdsr_err_bits =
		BIT_NUM_TO_MASK(DBGC_CORE_CDSR_ERR_BIT) |
		BIT_NUM_TO_MASK(DBGC_CORE_CDSR_ERR_HWCORE_BIT) |
		BIT_NUM_TO_MASK(DBGC_CORE_CDSR_ERR_FSMBUSY_BIT) |
		BIT_NUM_TO_MASK(DBGC_CORE_CDSR_ERR_DAP_OPCODE_BIT);
	if (core_status & cdsr_err_bits) {
		LOG_WARNING("Core errors detected: 0x%08X", core_status);
		error_code__get_and_clear(p_target);
		core_clear_errors(p_target);
		error_code__get_and_clear(p_target);
		core_status = DBG_STATUS_get(p_target);
		LOG_WARNING("Core errors %s: 0x%08X", core_status & cdsr_err_bits ? "not fixed!" : "fixed", core_status);
	}

	/// Only 1 HART available
	assert(p_target->coreid == 0);
	uint32_t const HART_status = HART_REGTRANS_read(p_target, DBGC_HART_REGS_DBG_STS);
	enum target_state const new_state = HART_status_bits_to_target_state(p_target, HART_status);
	enum target_state const old_state = p_target->state;
	if (new_state != old_state) {
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
			LOG_DEBUG("New debug reason: 0x%08X", DBG_REASON_NOTHALTED);
			p_target->debug_reason = DBG_REASON_NOTHALTED;
			LOG_DEBUG("TARGET_EVENT_RESUMED");
			target_call_event_callbacks(p_target, TARGET_EVENT_RESUMED);
		case TARGET_UNKNOWN:
		default:
			break;
		}
	}
#if 0
	uint32_t const pc_sample = HART_REGTRANS_read(p_target, DBGC_HART_REGS_PC_SAMPLE);
	LOG_DEBUG("pc_sample = 0x%08X", pc_sample);
#endif
	error_code__prepend(p_target, old_err_code);
}

/// GP registers accessors
///@{
static inline void
reg__invalidate(reg* const restrict p_reg)
{
	assert(p_reg);
	if (p_reg->exist) {
		assert(!p_reg->dirty);
		p_reg->valid = false;
	}
}

static inline void
reg__set_valid_value_to_cache(reg* const restrict p_reg, uint32_t const value)
{
	assert(p_reg);
	assert(p_reg->exist);
	STATIC_ASSERT((CHAR_BIT) == 8);
	assert(p_reg->size <= (CHAR_BIT)* sizeof value);
	assert(p_reg->value);
	LOG_DEBUG("Updating cache from register %s <-- 0x%08X", p_reg->name, value);
	buf_set_u32(p_reg->value, 0, p_reg->size, value);
	p_reg->valid = true;
	p_reg->dirty = false;
}

static void
reg__set_new_cache_value(reg* const restrict p_reg, uint8_t* const restrict buf)
{
	assert(p_reg);
	assert(buf);
	assert(p_reg->exist);
	STATIC_ASSERT((CHAR_BIT) == 8);
	assert(p_reg->size <= CHAR_BIT * sizeof(uint32_t));
	LOG_DEBUG("Updating register %s <-- 0x%08X", p_reg->name, buf_get_u32(buf, 0, p_reg->size));
	assert(p_reg->value);
	buf_cpy(buf, p_reg->value, p_reg->size);
	p_reg->valid = true;
	p_reg->dirty = true;
}

static inline bool
reg__check(reg const* const restrict p_reg)
{
	if (!p_reg->exist) {
		LOG_ERROR("Register %s not available", p_reg->name);
		return false;
	} else if (p_reg->dirty && !p_reg->valid) {
		LOG_ERROR("Register %s dirty but not valid", p_reg->name);
		return false;
	} else {
		return true;
	}
}

static void
reg_cache__invalidate(reg_cache const* const restrict p_reg_cache)
{
	assert(p_reg_cache);
	assert(!(p_reg_cache->num_regs && !p_reg_cache->reg_list));
	for (size_t i = 0; i < p_reg_cache->num_regs; ++i) {
		reg * const p_reg = &p_reg_cache->reg_list[i];
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

static void
reg_x__operation_conditions_check(reg const* const restrict p_reg)
{
	assert(p_reg);
	assert(reg__check(p_reg));
	target* p_target = p_reg->arch_info;
	assert(p_target);
	if (!(zero < p_reg->number && p_reg->number < REG_PC_NUMBER)) {
		LOG_WARNING("Bad reg id =%d for register %s", p_reg->number, p_reg->name);
		error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
		return;
	}
	LOG_DEBUG("update_status");
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return;
	}
	if (p_target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
	}
}

static int
reg_x__get(reg* const restrict p_reg)
{
	assert(p_reg);
	reg_x__operation_conditions_check(p_reg);
	target* p_target = p_reg->arch_info;
	assert(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}

	if (p_reg->valid) {
		// register cache already valid
		if (p_reg->dirty) {
			LOG_WARNING("Try re-read dirty cache register %s", p_reg->name);
		} else {
			LOG_DEBUG("Try re-read cache register %s", p_reg->name);
		}
	}

	uint32_t const pc_sample_1 = get_PC_to_check(p_target);
	if (error_code__get(p_target) == ERROR_OK) {
		exec__setup(p_target);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__get_and_clear(p_target);
		}
		int advance_pc_counter = 0;
		// Save p_reg->number register to CSR_DBG_SCRATCH CSR
		(void)exec__step(p_target, RV_CSRRW(zero, CSR_DBG_SCRATCH, p_reg->number));
		advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__get_and_clear(p_target);
		}

		// Exec jump back to previous instruction and get saved into CSR_DBG_SCRATCH CSR value
		assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
		uint32_t const value = exec__step(p_target, RV_JAL(zero, -advance_pc_counter));
		advance_pc_counter = 0;
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__get_and_clear(p_target);
		}
		reg__set_valid_value_to_cache(p_reg, value);
	}
	check_PC_unchanged(p_target, pc_sample_1);
	return error_code__get_and_clear(p_target);
}

static int
reg_x__store(reg* const restrict p_reg)
{
	assert(p_reg);
	target* p_target = p_reg->arch_info;
	assert(p_target);
	exec__setup(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}

	uint32_t const pc_sample_1 = get_PC_to_check(p_target);
	int advance_pc_counter = 0;

	if (error_code__get(p_target) == ERROR_OK) {
		assert(p_reg->value);
		exec__set_csr_data(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__get_and_clear(p_target);
		}
		(void)exec__step(p_target, RV_CSRRW(p_reg->number, CSR_DBG_SCRATCH, zero));
		advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__get_and_clear(p_target);
		}
		assert(p_reg->valid);
		assert(p_reg->dirty);
		p_reg->dirty = false;

		if (advance_pc_counter != 0) {
			/// Correct pc back after each instruction
			assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
			(void)exec__step(p_target, RV_JAL(zero, -advance_pc_counter));
			advance_pc_counter = 0;
			if (error_code__get(p_target) != ERROR_OK) {
				return error_code__get_and_clear(p_target);
			}
		}
		assert(advance_pc_counter == 0);
		assert(reg__check(p_reg));
	}
	check_PC_unchanged(p_target, pc_sample_1);
	return error_code__get_and_clear(p_target);
}

static int
reg_x__set(reg* const restrict p_reg, uint8_t* const restrict buf)
{
	assert(p_reg);
	assert(buf);
	reg_x__operation_conditions_check(p_reg);
	target* p_target = p_reg->arch_info;
	assert(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}

	uint32_t const value = buf_get_u32(buf, 0, XLEN);
	LOG_DEBUG("Updating register %s <-- 0x%08X", p_reg->name, value);

#if 0
	if (p_reg->valid && (buf_get_u32(p_reg->value, 0, XLEN) == value)) {
		// skip same value
		return error_code__clear(p_target);
	}
#endif
	assert(p_reg->value);
	buf_set_u32(p_reg->value, 0, XLEN, value);

	/// store dirty register data to HW
	return reg_x__store(p_reg);
}

static int
reg_x0__get(reg* const restrict p_reg)
{
	reg__set_valid_value_to_cache(p_reg, 0u);
	return ERROR_OK;
}

static int
reg_x0__set(reg* const restrict p_reg, uint8_t* const restrict buf)
{
	assert(p_reg);
	assert(buf);
	LOG_ERROR("Try to write to read-only register");
	reg__set_valid_value_to_cache(p_reg, 0u);
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
}

static reg*
prepare_temporary_GP_register(target const* const restrict p_target, int const after_reg)
{
	assert(p_target);
	reg_cache const* const p_reg_cache = p_target->reg_cache;
	assert(p_reg_cache);
	reg* const p_reg_list = p_reg_cache->reg_list;
	assert(p_reg_list);
	assert(p_reg_cache->num_regs >= REG_PC_NUMBER);
	reg* p_valid = NULL;
	reg* p_dirty = NULL;
	for (size_t i = after_reg + 1; i < REG_PC_NUMBER; ++i) {
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
			assert(after_reg + 1 < REG_PC_NUMBER);
			p_valid = &p_reg_list[after_reg + 1];
			if (error_code__update(p_target, reg_x__get(p_valid)) != ERROR_OK) {
				return NULL;
			}
		}
		assert(p_valid);
		assert(p_valid->valid);
		p_valid->dirty = true;
		p_dirty = p_valid;
	}

	assert(p_dirty);
	assert(p_dirty->valid);
	assert(p_dirty->dirty);
	return p_dirty;
}

#if 0
static mstatus_context_field_e
mstatus_FS__get(target* const restrict p_target)
{
	assert(p_target);
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return ext_off;
	}
	if (p_target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
		return ext_off;
	}
	reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, zero);
	assert(p_wrk_reg);

	exec__setup(p_target);
	int advance_pc_counter = 0;
	if (error_code__get(p_target) == ERROR_OK) {
		(void)exec__step(p_target, RV_CSRRW(p_wrk_reg->number, CSR_mstatus, zero));
		advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
		if (error_code__get(p_target) == ERROR_OK) {
		}
	}
	// restore temporary register
	{
		int const old_err_code = error_code__get_and_clear(p_target);
		error_code__update(p_target, reg_x__store(p_wrk_reg));
		error_code__prepend(p_target, old_err_code);
		assert(!p_wrk_reg->dirty);
	}
	return ext_off;
}
#endif

/// Update pc cache from HW (if non-cached)
static int
reg_pc__get(reg* const restrict p_reg)
{
	assert(p_reg);
	assert(p_reg->number == REG_PC_NUMBER);
	assert(reg__check(p_reg));

	/// Find temporary GP register
	target* const p_target = p_reg->arch_info;
	assert(p_target);
	sc_rv32i__Arch const* const restrict p_arch = p_target->arch_info;
	assert(p_arch);
	if (p_arch->use_pc_from_pc_sample) {
		uint32_t const pc_sample = HART_REGTRANS_read(p_target, DBGC_HART_REGS_PC_SAMPLE);
#if 0
		LOG_DEBUG("Updating cache from register %s <-- 0x%08X", p_reg->name, pc_sample);
#endif
		if (error_code__get(p_target) == ERROR_OK) {
			reg__set_valid_value_to_cache(p_reg, pc_sample);
		} else {
			reg__invalidate(p_reg);
		}
	} else {
		LOG_DEBUG("update_status");
		update_status(p_target);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__get_and_clear(p_target);
		}
		if (p_target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
			return error_code__get_and_clear(p_target);
		}

		reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, zero);
		assert(p_wrk_reg);

		exec__setup(p_target);
		int advance_pc_counter = 0;
		if (error_code__get(p_target) == ERROR_OK) {
			/// Copy pc to temporary register by AUIPC instruction
			(void)exec__step(p_target, RV_AUIPC(p_wrk_reg->number, 0));
			advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
			if (error_code__get(p_target) == ERROR_OK) {
				/// and store temporary register to CSR_DBG_SCRATCH CSR.
				(void)exec__step(p_target, RV_CSRRW(zero, CSR_DBG_SCRATCH, p_wrk_reg->number));
				advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
				if (error_code__get(p_target) == ERROR_OK) {
					/// Correct pc by jump 2 instructions back and get previous command result.
					assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
					uint32_t const value = exec__step(p_target, RV_JAL(zero, -advance_pc_counter));
					advance_pc_counter = 0;
					if (error_code__get(p_target) == ERROR_OK) {
						reg__set_valid_value_to_cache(p_reg, value);
						uint32_t const pc_sample = HART_REGTRANS_read(p_target, DBGC_HART_REGS_PC_SAMPLE);
						LOG_DEBUG("pc_sample = 0x%08X", pc_sample);
					}
				}
			}
		}

		// restore temporary register
		int const old_err_code = error_code__get_and_clear(p_target);
		error_code__update(p_target, reg_x__store(p_wrk_reg));
		error_code__prepend(p_target, old_err_code);
		assert(!p_wrk_reg->dirty);
	}
	return error_code__get_and_clear(p_target);
}

static int
reg_pc__set(reg* const restrict p_reg, uint8_t* const restrict buf)
{
	assert(p_reg);
	assert(p_reg->number == REG_PC_NUMBER);
	assert(reg__check(p_reg));
	if (!p_reg->valid) {
		LOG_DEBUG("force rewriting of pc register before read");
	}

	target* const p_target = p_reg->arch_info;
	assert(p_target);
	LOG_DEBUG("update_status");
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}
	if (p_target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
		return error_code__get_and_clear(p_target);
	}

	reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, zero);
	assert(p_wrk_reg);

	reg__set_new_cache_value(p_reg, buf);

	// Update to HW
	exec__setup(p_target);
	int advance_pc_counter = 0;
	if (error_code__get(p_target) == ERROR_OK) {
		assert(p_reg->value);
		exec__set_csr_data(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));
		if (error_code__get(p_target) == ERROR_OK) {
			// set temporary register value to restoring pc value
			(void)exec__step(p_target, RV_CSRRW(p_wrk_reg->number, CSR_DBG_SCRATCH, zero));
			advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
			if (error_code__get(p_target) == ERROR_OK) {
				assert(p_wrk_reg->dirty);
				/// and exec JARL to set pc
				(void)exec__step(p_target, RV_JALR(zero, p_wrk_reg->number, 0));
				advance_pc_counter = 0;
				assert(p_reg->valid);
				assert(p_reg->dirty);
				p_reg->dirty = false;
			}
		}
	}

	// restore temporary register
	int const old_err_code = error_code__get_and_clear(p_target);
	error_code__update(p_target, reg_x__store(p_wrk_reg));
	error_code__prepend(p_target, old_err_code);
	assert(!p_wrk_reg->dirty);

	return error_code__get_and_clear(p_target);
}

static int
reg_f__get(reg* const restrict p_reg)
{
	assert(p_reg);
	assert(REG_PC_NUMBER < p_reg->number && p_reg->number < TOTAL_NUMBER_OF_REGS);
	if (!p_reg->exist) {
		LOG_WARNING("Register %s is unavailable", p_reg->name);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	assert(reg__check(p_reg));

	target* const p_target = p_reg->arch_info;
	assert(p_target);

	LOG_DEBUG("update_status");
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}
	if (p_target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
		return error_code__get_and_clear(p_target);
	}

	/// @todo check that FPU is enabled
	/// Find temporary GP register
	reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, zero);
	assert(p_wrk_reg);

	uint32_t const pc_sample_1 = get_PC_to_check(p_target);
	if (error_code__get(p_target) == ERROR_OK) {
		exec__setup(p_target);
		int advance_pc_counter = 0;
		if (error_code__get(p_target) == ERROR_OK) {
			/// Copy values to temporary register
			(void)exec__step(p_target, RV_FMV_S_X(p_wrk_reg->number, (p_reg->number - (REG_PC_NUMBER + 1))));
			advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
			if (error_code__get(p_target) == ERROR_OK) {
				/// and store temporary register to CSR_DBG_SCRATCH CSR.
				(void)exec__step(p_target, RV_CSRRW(zero, CSR_DBG_SCRATCH, p_wrk_reg->number));
				advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
				if (error_code__get(p_target) == ERROR_OK) {
					/// Correct pc by jump 2 instructions back and get previous command result.
					assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
					uint32_t const value = exec__step(p_target, RV_JAL(zero, -advance_pc_counter));
					advance_pc_counter = 0;
					if (error_code__get(p_target) == ERROR_OK) {
						reg__set_valid_value_to_cache(p_reg, value);
					}
				}
			}
		}
	}

	check_PC_unchanged(p_target, pc_sample_1);

	// restore temporary register
	int const old_err_code = error_code__get_and_clear(p_target);
	error_code__update(p_target, reg_x__store(p_wrk_reg));
	error_code__prepend(p_target, old_err_code);
	assert(!p_wrk_reg->dirty);

	return error_code__get_and_clear(p_target);
}

static int
reg_f__set(reg* const restrict p_reg, uint8_t* const restrict buf)
{
	assert(p_reg);
	assert(REG_PC_NUMBER < p_reg->number && p_reg->number < TOTAL_NUMBER_OF_REGS);
	if (!p_reg->exist) {
		LOG_WARNING("Register %s is unavailable", p_reg->name);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	assert(reg__check(p_reg));

	target* const p_target = p_reg->arch_info;
	assert(p_target);

	LOG_DEBUG("update_status");
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}
	if (p_target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
		return error_code__get_and_clear(p_target);
	}

	/// @todo check that FPU is enabled
	/// Find temporary GP register
	reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, zero);
	assert(p_wrk_reg);

	uint32_t const pc_sample_1 = get_PC_to_check(p_target);
	if (error_code__get(p_target) == ERROR_OK) {
		exec__setup(p_target);
		int advance_pc_counter = 0;
		if (error_code__get(p_target) == ERROR_OK) {
			reg__set_new_cache_value(p_reg, buf);

			exec__set_csr_data(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));
			if (error_code__get(p_target) == ERROR_OK) {
				// set temporary register value to restoring pc value
				(void)exec__step(p_target, RV_CSRRW(p_wrk_reg->number, CSR_DBG_SCRATCH, zero));
				advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
				if (error_code__get(p_target) == ERROR_OK) {
					assert(p_wrk_reg->dirty);
					assert(zero < p_wrk_reg->number && p_wrk_reg->number < REG_PC_NUMBER);
					(void)exec__step(p_target, RV_FMV_X_S((p_reg->number - (REG_PC_NUMBER + 1)), p_wrk_reg->number));
					advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
					if (error_code__get(p_target) == ERROR_OK) {
						/// Correct pc by jump 2 instructions back and get previous command result.
						assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
						(void)exec__step(p_target, RV_JAL(zero, -advance_pc_counter));
						advance_pc_counter = 0;
						assert(p_reg->valid);
						assert(p_reg->dirty);
						p_reg->dirty = false;
					}
				}
			}
		}
	}

	check_PC_unchanged(p_target, pc_sample_1);
	// restore temporary register
	int const old_err_code = error_code__get_and_clear(p_target);
	error_code__update(p_target, reg_x__store(p_wrk_reg));
	error_code__prepend(p_target, old_err_code);
	assert(!p_wrk_reg->dirty);
	return error_code__get_and_clear(p_target);
}

static reg_cache*
reg_cache__create(char const* name, reg const regs_templates[], size_t const num_regs, void* const p_arch_info)
{
	assert(name);
	assert(0 < num_regs);
	assert(p_arch_info);
	reg* const p_dst_array = calloc(num_regs, sizeof(reg));
	reg* p_dst_iter = &p_dst_array[0];
	reg const* p_src_iter = &regs_templates[0];
	for (size_t i = 0; i < num_regs; ++i) {
		*p_dst_iter = *p_src_iter;
		p_dst_iter->value = calloc(1, NUM_BITS_TO_SIZE(p_src_iter->size));
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
set_DEMODE_ENBL(target* const restrict p_target, uint32_t const set_value)
{
	assert(p_target);
	HART_REGTRANS_write(p_target, DBGC_HART_REGS_DMODE_ENBL, set_value);
	if (error_code__get(p_target) != ERROR_OK) {
		return;
	}

	sc_rv32i__Arch const* const restrict p_arch = p_target->arch_info;
	assert(p_arch);
	if (p_arch->use_verify_hart_regtrans_write) {
		uint32_t const get_value = HART_REGTRANS_read(p_target, DBGC_HART_REGS_DMODE_ENBL);
		if (error_code__get(p_target) != ERROR_OK) {
			return;
		}
		if (get_value != set_value) {
			LOG_ERROR("Write DBGC_HART_REGS_DMODE_ENBL with value 0x%08X, but re-read value is 0x%08X", set_value, get_value);
			error_code__update(p_target, ERROR_TARGET_FAILURE);
			return;
		}
	}
}

static int
resume_common(target* const restrict p_target, uint32_t dmode_enabled, int const current, uint32_t const address, int const handle_breakpoints, int const debug_execution)
{
	assert(p_target);
	LOG_DEBUG("update_status");
	update_status(p_target);
	if (p_target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/// @todo multiple caches
	reg_cache* const p_reg_cache = p_target->reg_cache;
	reg* const p_pc = &p_reg_cache->reg_list[REG_PC_NUMBER];
	if (!current) {
		uint8_t buf[sizeof address];
		buf_set_u32(buf, 0, XLEN, address);
		error_code__update(p_target, reg_pc__set(p_pc, buf));
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__get_and_clear(p_target);
		}
		assert(!p_pc->dirty);
	}
	if (handle_breakpoints) {
		dmode_enabled |= BIT_NUM_TO_MASK(DBGC_HART_HDMER_SW_BRKPT_BIT);

		sc_rv32i__Arch const* const restrict p_arch = p_target->arch_info;
		assert(p_arch);
		if (p_arch->use_resume_at_sw_breakpoint_emulates_saved_instruction && current) {
			// Find breakpoint for current instruction
			error_code__update(p_target, reg_pc__get(p_pc));
			assert(p_pc->value);
			uint32_t const pc = buf_get_u32(p_pc->value, 0, XLEN);
			struct breakpoint* p_next_bkp = p_target->breakpoints;
			for (; p_next_bkp; p_next_bkp = p_next_bkp->next) {
				if (p_next_bkp->set && (p_next_bkp->address == pc)) {
					break;
				}
			}

			if (p_next_bkp) {
				// If next instruction is replaced by breakpoint, then execute saved instruction
				uint32_t const instruction = buf_get_u32(p_next_bkp->orig_instr, 0, ILEN);
				exec__setup(p_target);
				exec__step(p_target, instruction);
				// If HART in single step mode
				if (dmode_enabled & BIT_NUM_TO_MASK(DBGC_HART_HDMER_SINGLE_STEP_BIT)) {
					// then single step already done
					reg_cache__chain_invalidate(p_reg_cache);
					set_DEMODE_ENBL(p_target, dmode_enabled);
#if 0
					target_call_event_callbacks(p_target, TARGET_EVENT_HALTED);
#endif
					LOG_DEBUG("update_status");
					update_status(p_target);
#if 1
					LOG_DEBUG("New debug reason: 0x%08X", DBG_REASON_SINGLESTEP);
					p_target->debug_reason = DBG_REASON_SINGLESTEP;
#endif
					return error_code__get_and_clear(p_target);
				}
			}
		}
	} else {
		dmode_enabled &= !BIT_NUM_TO_MASK(DBGC_HART_HDMER_SW_BRKPT_BIT);
	}

	reg_cache__chain_invalidate(p_reg_cache);
	set_DEMODE_ENBL(p_target, dmode_enabled);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}
	DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_DBGCMD);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}

	(void)DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_RESUME) | BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS));
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}

	LOG_DEBUG("New debug reason: 0x%08X", DBG_REASON_NOTHALTED);
	p_target->debug_reason = DBG_REASON_NOTHALTED;
	target_call_event_callbacks(p_target, debug_execution ? TARGET_EVENT_DEBUG_RESUMED : TARGET_EVENT_RESUMED);

	LOG_DEBUG("update_status");
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}

	return error_code__get_and_clear(p_target);
}

static int
reset__set(target* const restrict p_target, bool const active)
{
	assert(p_target);
	DAP_CTRL_REG_set(p_target, DBGC_UNIT_ID_CORE, DBGC_FGRP_CORE_REGTRANS);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}

	uint32_t const get_old_value1 = core_REGTRANS_read(p_target, DBGC_CORE_REGS_DBG_CTRL);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}
	/// @todo replace literals
	static uint32_t const bit_mask = BIT_NUM_TO_MASK(DBGC_CORE_CDCR_HART0_RST_BIT) | BIT_NUM_TO_MASK(DBGC_CORE_CDCR_RST_BIT);

	uint32_t const set_value = (get_old_value1 & ~bit_mask) | (active ? bit_mask : 0u);
	core_REGTRANS_write(p_target, DBGC_CORE_REGS_DBG_CTRL, set_value);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}

	sc_rv32i__Arch const* const restrict p_arch = p_target->arch_info;
	assert(p_arch);
	if (p_arch->use_verify_core_regtrans_write) {
		// double check
		uint32_t const get_new_value2 = core_REGTRANS_read(p_target, DBGC_CORE_REGS_DBG_CTRL);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__get_and_clear(p_target);
		}
		if ((get_new_value2 & bit_mask) != (set_value & bit_mask)) {
			LOG_ERROR("Fail to verify write: set 0x%08X, but get 0x%08X", set_value, get_new_value2);
			error_code__update(p_target, ERROR_TARGET_FAILURE);
			return error_code__get_and_clear(p_target);
		}
	}

	LOG_DEBUG("update_status");
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}
	if (active) {
		if (p_target->state != TARGET_RESET) {
			/// issue error if we are still running
			LOG_ERROR("RV is not resetting after reset assert");
			error_code__update(p_target, ERROR_TARGET_FAILURE);
		}
	} else {
		if (p_target->state == TARGET_RESET) {
			LOG_ERROR("RV is stiil in reset after reset deassert");
			error_code__update(p_target, ERROR_TARGET_FAILURE);
		}
	}
	return error_code__get_and_clear(p_target);
}

static reg_arch_type const reg_x0_accessors =
{
	.get = reg_x0__get,
	.set = reg_x0__set,
};

static reg_arch_type const reg_x_accessors =
{
	.get = reg_x__get,
	.set = reg_x__set,
};

static reg_arch_type const reg_pc_accessors =
{
	.get = reg_pc__get,
	.set = reg_pc__set,
};

static reg_arch_type const reg_f_accessors =
{
	.get = reg_f__get,
	.set = reg_f__set,
};

static struct reg_feature feature = {
	.name = "org.gnu.gdb.riscv.cpu",
};

static reg const def_regs_array[] = {
	// Hard-wired zero
	{.name = "x0", .number = 0, .caller_save = false, .dirty = false, .valid = true, .exist = true, .size = XLEN, .type = &reg_x0_accessors, .feature = &feature},

	// Return address
	{.name = "x1", .number = 1, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},

	// Stack pointer
	{.name = "x2", .number = 2, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},

	// Global pointer
	{.name = "x3", .number = 3, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},

	// Thread pointer
	{.name = "x4", .number = 4, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},

	// Temporaries
	{.name = "x5", .number = 5, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x6", .number = 6, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x7", .number = 7, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},

	// Saved register/frame pointer
	{.name = "x8", .number = 8, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},

	// Saved register
	{.name = "x9", .number = 9, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},

	// Function arguments/return values
	{.name = "x10", .number = 10, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x11", .number = 11, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},

	// Function arguments
	{.name = "x12", .number = 12, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x13", .number = 13, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x14", .number = 14, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x15", .number = 15, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x16", .number = 16, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x17", .number = 17, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},

	// Saved registers
	{.name = "x18", .number = 18, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x19", .number = 19, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x20", .number = 20, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x21", .number = 21, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x22", .number = 22, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x23", .number = 23, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x24", .number = 24, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x25", .number = 25, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x26", .number = 26, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x27", .number = 27, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},

	// Temporaries
	{.name = "x28", .number = 28, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x29", .number = 29, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x30", .number = 30, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},
	{.name = "x31", .number = 31, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors, .feature = &feature},

	// Program counter
	{.name = "pc", .number = 32, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_pc_accessors, .feature = &feature},

	// FP temporaries
	{.name = "f0", .number = 33, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f1", .number = 34, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f2", .number = 35, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f3", .number = 36, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f4", .number = 37, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f5", .number = 38, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f6", .number = 39, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f7", .number = 40, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},

	// FP saved registers
	{.name = "f8", .number = 41, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f9", .number = 42, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},

	// FP arguments/return values
	{.name = "f10", .number = 43, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f11", .number = 44, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},

	// FP arguments
	{.name = "f12", .number = 45, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f13", .number = 46, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f14", .number = 47, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f15", .number = 48, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f16", .number = 49, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f17", .number = 50, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},

	// FP saved registers
	{.name = "f18", .number = 51, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f19", .number = 52, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f20", .number = 53, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f21", .number = 54, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f22", .number = 55, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f23", .number = 56, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f24", .number = 57, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f25", .number = 58, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f26", .number = 59, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f27", .number = 60, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},

	// FP temporaries
	{.name = "f28", .number = 61, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f29", .number = 62, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f30", .number = 63, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
	{.name = "f31", .number = 64, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors, .feature = &feature},
};

static int
sc_rv32i__init_target(struct command_context *cmd_ctx, target* const restrict p_target)
{
	assert(p_target);
	p_target->reg_cache = reg_cache__create("rv32i", def_regs_array, ARRAY_LEN(def_regs_array), p_target);
	sc_rv32i__Arch* p_arch_info = calloc(1, sizeof(sc_rv32i__Arch));
	*p_arch_info = initial_arch;

	p_target->arch_info = p_arch_info;
	return ERROR_OK;
}

static void
sc_rv32i__deinit_target(target* const restrict p_target)
{
	assert(p_target);
	while (p_target->reg_cache) {
		reg_cache* const p_reg_cache = p_target->reg_cache;
		reg* const reg_list = p_reg_cache->reg_list;
		assert(!p_reg_cache->num_regs || reg_list);
		for (unsigned i = 0; i < p_reg_cache->num_regs; ++i) {
			free(reg_list[i].value);
		}
		free(reg_list);

		p_target->reg_cache = p_reg_cache->next;
		free(p_reg_cache);
	}
	if (p_target->arch_info) {
		free(p_target->arch_info);
		p_target->arch_info = NULL;
	}
}

static int
sc_rv32i__target_create(target* const restrict p_target, struct Jim_Interp *interp)
{
	assert(p_target);
	return ERROR_OK;
}

static int
sc_rv32i__examine(target* const restrict p_target)
{
	assert(p_target);
	for (int i = 0; i < 10; ++i) {
		error_code__get_and_clear(p_target);
		LOG_DEBUG("update_status");
		update_status(p_target);
		if (error_code__get(p_target) == ERROR_OK) {
			break;
		}
	}
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}

	assert(p_target);
	if (!target_was_examined(p_target)) {
		uint32_t const IDCODE = IDCODE_get(p_target);
		uint32_t const DBG_ID = DBG_ID_get(p_target);
		LOG_INFO("IDCODE=0x%08X DBG_ID=0x%08X BLD_ID=0x%08X", IDCODE, DBG_ID, BLD_ID_get(p_target));
		assert(IDCODE == EXPECTED_IDCODE);
		assert((DBG_ID & DBG_ID_VERSION_MASK) == (DBG_ID_VERSION_MASK & EXPECTED_DBG_ID));
#if EXPECTED_DBG_ID & DBG_ID_SUBVERSION_MASK
		assert((DBG_ID & DBG_ID_SUBVERSION_MASK) >= (EXPECTED_DBG_ID & DBG_ID_SUBVERSION_MASK));
#endif
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__get_and_clear(p_target);
		}
		set_DEMODE_ENBL(p_target, NORMAL_DEBUG_ENABLE_MASK);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__get_and_clear(p_target);
		}
	}

	target_set_examined(p_target);

	return error_code__get_and_clear(p_target);
}

static int
sc_rv32i__poll(target* const restrict p_target)
{
	assert(p_target);
	LOG_DEBUG("update_status");
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}
	return error_code__get_and_clear(p_target);
}

static int
sc_rv32i__arch_state(target* const restrict p_target)
{
	assert(p_target);
	LOG_DEBUG("update_status");
	update_status(p_target);
	return error_code__get_and_clear(p_target);
}

static int
sc_rv32i__halt(target* const restrict p_target)
{
	assert(p_target);
	// May be already halted?
	{
		LOG_DEBUG("update_status");
		update_status(p_target);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__get_and_clear(p_target);
		}

		if (p_target->state == TARGET_HALTED) {
			LOG_WARNING("Halt request when RV is already in halted state");
			return error_code__get_and_clear(p_target);
		}
	}

	// Try to halt
	{
		DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_DBGCMD);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__get_and_clear(p_target);
		}

		(void)DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_HALT) | BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS));
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__get_and_clear(p_target);
		}
	}

	LOG_DEBUG("update_status");
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}

	// Verify that in debug mode
	if (p_target->state != TARGET_HALTED) {
		// issue error if we are still running
		LOG_ERROR("RV is not halted after Halt command");
		error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
		return error_code__get_and_clear(p_target);
	}

	return error_code__get_and_clear(p_target);
}

static int
sc_rv32i__resume(target* const restrict p_target, int const current, uint32_t const address, int const handle_breakpoints, int const debug_execution)
{
	assert(p_target);
	uint32_t const dmode_enabled = NORMAL_DEBUG_ENABLE_MASK;
	return resume_common(p_target, dmode_enabled, current, address, handle_breakpoints, debug_execution);
}

static int
sc_rv32i__step(target* const restrict p_target, int const current, uint32_t const address, int const handle_breakpoints)
{
	assert(p_target);
	uint32_t const dmode_enabled = (NORMAL_DEBUG_ENABLE_MASK) | BIT_NUM_TO_MASK(DBGC_HART_HDMER_SINGLE_STEP_BIT);
	return resume_common(p_target, dmode_enabled, current, address, handle_breakpoints, false);
}

static int
sc_rv32i__assert_reset(target* const restrict p_target)
{
	LOG_DEBUG("Reset control");
	assert(p_target);
	return reset__set(p_target, true);
}

static int
sc_rv32i__deassert_reset(target* const restrict p_target)
{
	LOG_DEBUG("Reset control");
	assert(p_target);
	return reset__set(p_target, false);
}

static int
sc_rv32i__soft_reset_halt(target* const restrict p_target)
{
	LOG_DEBUG("Soft reset called");
	assert(p_target);
	reset__set(p_target, true);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}
	set_DEMODE_ENBL(p_target, NORMAL_DEBUG_ENABLE_MASK);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}
	reset__set(p_target, false);

	return error_code__get_and_clear(p_target);
}

static int
sc_rv32i__read_memory(target* const restrict p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t* restrict buffer)
{
	assert(p_target);
	assert(buffer);
	LOG_DEBUG("Read_memory at 0x%08X, %d items, each %d bytes, total %d bytes", address, count, size, count * size);
	/// Check for size
	if (!(size == 1 || size == 2 || size == 4)) {
		LOG_ERROR("Invalid item size %d", size);
		error_code__update(p_target, ERROR_TARGET_FAILURE);
		return error_code__get_and_clear(p_target);
	}

	/// Check for alignment
	if (address % size != 0) {
		LOG_ERROR("Unaligned access at 0x%08X, for item size %d", address, size);
		error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
		return error_code__get_and_clear(p_target);
	}

	/// Check that target halted
	{
		LOG_DEBUG("update_status");
		update_status(p_target);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__get_and_clear(p_target);
		}
		if (p_target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
			return error_code__get_and_clear(p_target);
		}
	}

	/// Reserve work register
	reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, zero);
	assert(p_wrk_reg);

	uint32_t const pc_sample_1 = get_PC_to_check(p_target);
	if (error_code__get(p_target) == ERROR_OK) {
		/// Define opcode for load item to register
		uint32_t const load_OP =
			size == 4 ? RV_LW(p_wrk_reg->number, p_wrk_reg->number, 0) :
			size == 2 ? RV_LH(p_wrk_reg->number, p_wrk_reg->number, 0) :
			/*size == 1*/RV_LB(p_wrk_reg->number, p_wrk_reg->number, 0);

		/// Setup exec operations mode
		exec__setup(p_target);
		int advance_pc_counter = 0;
		if (error_code__get(p_target) == ERROR_OK) {
			/// For count number of items do loop
			while (count--) {
				/// Set address to CSR
				exec__set_csr_data(p_target, address);
				if (error_code__get(p_target) != ERROR_OK) {
					break;
				}

				/// Load address to work register
				(void)exec__step(p_target, RV_CSRRW(p_wrk_reg->number, CSR_DBG_SCRATCH, zero));
				advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
				if (error_code__get(p_target) != ERROR_OK) {
					break;
				}

				/// Exec load item to register
				(void)exec__step(p_target, load_OP);
				advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
				if (error_code__get(p_target) != ERROR_OK) {
					break;
				}

				/// Exec store work register to csr
				(void)exec__step(p_target, RV_CSRRW(zero, CSR_DBG_SCRATCH, p_wrk_reg->number));
				advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);

				/// get data from csr and jump back to correct pc
				assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
				uint32_t const value = exec__step(p_target, RV_JAL(zero, -advance_pc_counter));
				advance_pc_counter = 0;
				if (error_code__get(p_target) != ERROR_OK) {
					break;
				}

				/// store read data to buffer
				buf_set_u32(buffer, 0, 8 * size, value);

				/// advance src/dst pointers
				address += size;
				buffer += size;
			}
		}
	}

	if (error_code__get(p_target) != ERROR_OK) {
		LOG_DEBUG("update_status");
		update_status(p_target);
	}

	check_PC_unchanged(p_target, pc_sample_1);

	/// restore temporary register
	int const old_err_code = error_code__get_and_clear(p_target);
	error_code__update(p_target, reg_x__store(p_wrk_reg));
	error_code__prepend(p_target, old_err_code);
	assert(!p_wrk_reg->dirty);

	return error_code__get_and_clear(p_target);
}

static int
sc_rv32i__write_memory(target* const restrict p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t const* restrict buffer)
{
	assert(p_target);
	assert(buffer);
	LOG_DEBUG("Write_memory at 0x%08X, %d items, each %d bytes, total %d bytes", address, count, size, count * size);
	/// Check for size
	if (!(size == 1 || size == 2 || size == 4)) {
		LOG_ERROR("Invalid item size %d", size);
		error_code__update(p_target, ERROR_TARGET_FAILURE);
		return error_code__get_and_clear(p_target);
	}

	/// Check for alignment
	if (address % size != 0) {
		LOG_ERROR("Unaligned access at 0x%08X, for item size %d", address, size);
		error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
		return error_code__get_and_clear(p_target);
	}

	if (count == 0) {
		return error_code__get_and_clear(p_target);
	}
	/// Check that target halted
	{
		LOG_DEBUG("update_status");
		update_status(p_target);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__get_and_clear(p_target);
		}
		if (p_target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
			return error_code__get_and_clear(p_target);
		}
	}
	/// Reserve work register
	reg* const p_addr_reg = prepare_temporary_GP_register(p_target, zero);
	assert(p_addr_reg);
	reg* const p_data_reg = prepare_temporary_GP_register(p_target, p_addr_reg->number);
	assert(p_data_reg);
	assert(p_addr_reg->number != p_data_reg->number);

	uint32_t const pc_sample_1 = get_PC_to_check(p_target);
	sc_rv32i__Arch const* const restrict p_arch = p_target->arch_info;
	assert(p_arch);
	size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);
	if (p_arch->use_pc_advmt_dsbl_bit) {
		HART_REGTRANS_write(p_target, DBGC_HART_REGS_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
	}
	if (error_code__get(p_target) == ERROR_OK) {
		/// Setup exec operations mode
		exec__setup(p_target);
		size_t advance_pc_counter = 0;
		if (error_code__get(p_target) == ERROR_OK) {
			// Set address to CSR
			exec__set_csr_data(p_target, address);
			if (error_code__get(p_target) == ERROR_OK) {
				/// Load address to work register
				(void)exec__step(p_target, RV_CSRRW(p_addr_reg->number, CSR_DBG_SCRATCH, zero));
				advance_pc_counter += instr_step;

				// Opcodes
				uint32_t const instructions[3] = {
					RV_CSRRW(p_data_reg->number, CSR_DBG_SCRATCH, zero),
					(size == 4 ? RV_SW(p_data_reg->number, p_addr_reg->number, 0) :
					size == 2 ? RV_SH(p_data_reg->number, p_addr_reg->number, 0) :
					/*size == 1*/ RV_SB(p_data_reg->number, p_addr_reg->number, 0)),
					RV_ADDI(p_addr_reg->number, p_addr_reg->number, size)
				};

				static uint32_t max_pc_offset = (((1u << 20) - 1u) / NUM_BITS_TO_SIZE(XLEN)) * NUM_BITS_TO_SIZE(XLEN);
				if (p_arch->use_queuing_for_dr_scans) {
					uint8_t DAP_OPSTATUS = 0;
					uint8_t const data_wr_opcode[1] = {DBGC_DAP_OPCODE_DBGCMD_DBGDATA_WR};
					scan_field const data_scan_opcode_field = {
						.num_bits = TAP_LEN_DAP_CMD_OPCODE,
						.out_value = data_wr_opcode,
						.in_value = &DAP_OPSTATUS,
						.check_value = &DAP_OPSTATUS_GOOD,
						.check_mask = &DAP_STATUS_MASK,
					};
					scan_field data_scan_fields[TAP_NUM_FIELDS_DAP_CMD] = {{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT}, data_scan_opcode_field, };
					uint8_t const instr_exec_opcode[1] = {DBGC_DAP_OPCODE_DBGCMD_CORE_EXEC};
					scan_field const instr_scan_opcode_field = {
						.num_bits = TAP_LEN_DAP_CMD_OPCODE,
						.out_value = instr_exec_opcode,
						.in_value = &DAP_OPSTATUS,
						.check_value = &DAP_OPSTATUS_GOOD,
						.check_mask = &DAP_STATUS_MASK,
					};
					scan_field const instr_fields[ARRAY_LEN(instructions)][TAP_NUM_FIELDS_DAP_CMD] = {
						{{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT, .out_value = (uint8_t const*)(&instructions[0])}, instr_scan_opcode_field},
						{{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT, .out_value = (uint8_t const*)(&instructions[1])}, instr_scan_opcode_field},
						{{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT, .out_value = (uint8_t const*)(&instructions[2])}, instr_scan_opcode_field}
					};
					while (error_code__get(p_target) == ERROR_OK && count--) {
						assert(p_target->tap);
						data_scan_fields[0].out_value = (uint8_t const*)buffer;
						jtag_add_dr_scan_check(p_target->tap, ARRAY_LEN(data_scan_fields), data_scan_fields, TAP_IDLE);
						for (unsigned i = 0; i < ARRAY_LEN(instr_fields); ++i) {
							jtag_add_dr_scan_check(p_target->tap, TAP_NUM_FIELDS_DAP_CMD, instr_fields[i], TAP_IDLE);
							advance_pc_counter += instr_step;
						}
						buffer += size;
					}
					if (!p_arch->use_pc_advmt_dsbl_bit) {
						assert(advance_pc_counter % NUM_BITS_TO_SIZE(XLEN) == 0);
						while (advance_pc_counter) {
							uint32_t const step_back = advance_pc_counter > max_pc_offset ? max_pc_offset : advance_pc_counter;
							advance_pc_counter -= step_back;
							assert(advance_pc_counter % NUM_BITS_TO_SIZE(XLEN) == 0);
							uint32_t const OP_correct_pc = RV_JAL(zero, -(int)(step_back));
							scan_field const instr_pc_correct_fields[2] = {{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT, .out_value = (uint8_t const*)(&OP_correct_pc)}, instr_scan_opcode_field};
							jtag_add_dr_scan_check(p_target->tap, TAP_NUM_FIELDS_DAP_CMD, instr_pc_correct_fields, TAP_IDLE);
						}
						assert(advance_pc_counter == 0);
					}
					error_code__update(p_target, jtag_execute_queue());
					if ((DAP_OPSTATUS & DAP_OPSTATUS_MASK) != DAP_OPSTATUS_OK) {
						LOG_ERROR("DAP_OPSTATUS == 0x%1X", (uint32_t)DAP_OPSTATUS);
						error_code__update(p_target, ERROR_TARGET_FAILURE);
					}
				} else {
					while ((error_code__get(p_target) == ERROR_OK) && count--) {
						/// Set data to CSR
						exec__set_csr_data(p_target, buf_get_u32(buffer, 0, 8 * size));
						if (error_code__get(p_target) != ERROR_OK) {
							break;
						}

						for (unsigned i = 0; i < ARRAY_LEN(instructions); ++i) {
							(void)exec__step(p_target, instructions[i]);
							if (error_code__get(p_target) != ERROR_OK) {
								break;
							}
							advance_pc_counter += instr_step;
						}
						buffer += size;
					}
					if (!p_arch->use_pc_advmt_dsbl_bit) {
						assert(advance_pc_counter % NUM_BITS_TO_SIZE(XLEN) == 0);
						while ((error_code__get(p_target) == ERROR_OK) && (advance_pc_counter != 0)) {
							uint32_t const step_back = advance_pc_counter > max_pc_offset ? max_pc_offset : advance_pc_counter;
							advance_pc_counter -= step_back;
							assert(advance_pc_counter % NUM_BITS_TO_SIZE(XLEN) == 0);
							uint32_t const OP_correct_pc = RV_JAL(zero, -(int)(step_back));
							(void)exec__step(p_target, OP_correct_pc);
						}
					}
				}
			}
		}
	}

	if (error_code__get(p_target) != ERROR_OK) {
		LOG_DEBUG("update_status");
		update_status(p_target);
	}

	if (p_arch->use_pc_advmt_dsbl_bit) {
		HART_REGTRANS_write(p_target, DBGC_HART_REGS_DBG_CTRL, 0u);
	}

	check_PC_unchanged(p_target, pc_sample_1);

	/// restore temporary registers
	int const old_err_code = error_code__get_and_clear(p_target);
	int const new_err_code_1 = reg_x__store(p_data_reg);
	int const new_err_code_2 = reg_x__store(p_addr_reg);
	error_code__update(p_target, old_err_code);
	error_code__update(p_target, new_err_code_1);
	error_code__update(p_target, new_err_code_2);
	assert(!p_data_reg->dirty);
	assert(!p_addr_reg->dirty);

	return error_code__get_and_clear(p_target);
}

static int
sc_rv32i__add_breakpoint(target* const restrict p_target, struct breakpoint* const restrict breakpoint)
{
	assert(p_target);
	assert(breakpoint);
	if (breakpoint->length != NUM_BITS_TO_SIZE(ILEN)) {
		LOG_ERROR("Invalid breakpoint size ( != 4): %d", breakpoint->length);
		error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
		return error_code__get_and_clear(p_target);
	}

	if ((breakpoint->address % NUM_BITS_TO_SIZE(ILEN)) != 0) {
		LOG_ERROR("Unaligned breakpoint: 0x%08X", breakpoint->address);
		error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
		return error_code__get_and_clear(p_target);
	}

	if (breakpoint->type != BKPT_SOFT) {
		LOG_ERROR("Only softawre breakpoins available");
		error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
		return error_code__get_and_clear(p_target);
	}

	LOG_DEBUG("update_status");
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}

	if (p_target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
		return error_code__get_and_clear(p_target);
	}

	/// @todo check for duplicate breakpoints
	assert(breakpoint->orig_instr);
	error_code__update(p_target, target_read_buffer(p_target, breakpoint->address, breakpoint->length, breakpoint->orig_instr));
	if (error_code__get(p_target) != ERROR_OK) {
		LOG_ERROR("Can't save original instruction");
		return error_code__get_and_clear(p_target);
	}

	/// Write SBREAK opcode
	error_code__update(p_target, target_write_u32(p_target, breakpoint->address, RV_SBREAK()));
	if (error_code__get(p_target) != ERROR_OK) {
		LOG_ERROR("Can't write SBREAK");
		return error_code__get_and_clear(p_target);
	}

	/// @todo check set values
	breakpoint->set = 1;

	return error_code__get_and_clear(p_target);
}

static int
sc_rv32i__remove_breakpoint(target* const restrict p_target, struct breakpoint* const restrict breakpoint)
{
	assert(p_target);
	assert(breakpoint);
	if (breakpoint->length != NUM_BITS_TO_SIZE(ILEN)) {
		error_code__update(p_target, ERROR_TARGET_INVALID);
		return error_code__get_and_clear(p_target);
	}
	if ((breakpoint->address % NUM_BITS_TO_SIZE(ILEN)) != 0) {
		error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
		return error_code__get_and_clear(p_target);
	}
	if (breakpoint->type != BKPT_SOFT) {
		error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
		return error_code__get_and_clear(p_target);
	}

	assert(p_target);
	LOG_DEBUG("update_status");
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}
	if (p_target->state != TARGET_HALTED) {
		error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
		return error_code__get_and_clear(p_target);
	}

	assert(breakpoint->orig_instr);
	error_code__update(p_target, target_write_buffer(p_target, breakpoint->address, breakpoint->length, breakpoint->orig_instr));
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__get_and_clear(p_target);
	}
	breakpoint->set = 0;
	return error_code__get_and_clear(p_target);
}

/// gdb_server expects valid reg values and will use set method for updating reg values
static int
sc_rv32i__get_gdb_reg_list(target* const restrict p_target, reg **reg_list[], int* const restrict reg_list_size, enum target_register_class const reg_class)
{
	assert(p_target);
	assert(reg_list_size);
	assert(reg_class == REG_CLASS_ALL || reg_class == REG_CLASS_GENERAL);

	sc_rv32i__Arch *arch_info = p_target->arch_info;
	assert(arch_info);

	size_t const num_regs = reg_class == REG_CLASS_ALL ? TOTAL_NUMBER_OF_REGS : REG_PC_NUMBER + 1;
	reg** p_reg_array = calloc(num_regs, sizeof(reg*));
	reg *a_reg_list = p_target->reg_cache->reg_list;
	for (size_t i = 0; i < num_regs; ++i) {
		p_reg_array[i] = &a_reg_list[i];
	}
	*reg_list = p_reg_array;
	*reg_list_size = num_regs;
	return error_code__get_and_clear(p_target);
}

/// @todo make const
target_type syntacore_riscv32i_target =
{
	.name = "syntacore_riscv32i",

	.poll = sc_rv32i__poll,
	.arch_state = sc_rv32i__arch_state,
	.target_request_data = NULL,

	.halt = sc_rv32i__halt,
	.resume = sc_rv32i__resume,
	.step = sc_rv32i__step,

	.assert_reset = sc_rv32i__assert_reset,
	.deassert_reset = sc_rv32i__deassert_reset,
	.soft_reset_halt = sc_rv32i__soft_reset_halt,

	.get_gdb_reg_list = sc_rv32i__get_gdb_reg_list,

	.read_memory = sc_rv32i__read_memory,
	.write_memory = sc_rv32i__write_memory,

	.read_buffer = NULL,
	.write_buffer = NULL,

	.checksum_memory = NULL,
	.blank_check_memory = NULL,

	.add_breakpoint = sc_rv32i__add_breakpoint,
	.add_context_breakpoint = NULL,
	.add_hybrid_breakpoint = NULL,

	.remove_breakpoint = sc_rv32i__remove_breakpoint,

	.add_watchpoint = NULL,
	.remove_watchpoint = NULL,

	.hit_watchpoint = NULL,

	.run_algorithm = NULL,
	.start_algorithm = NULL,
	.wait_algorithm = NULL,

	.commands = NULL,

	.target_create = sc_rv32i__target_create,
	.target_jim_configure = NULL,
	.target_jim_commands = NULL,

	.examine = sc_rv32i__examine,

	.init_target = sc_rv32i__init_target,
	.deinit_target = sc_rv32i__deinit_target,

	.virt2phys = NULL,
	.read_phys_memory = NULL,
	.write_phys_memory = NULL,

	.mmu = NULL,
	.check_reset = NULL,
	.get_gdb_fileio_info = NULL,
	.gdb_fileio_end = NULL,
	.profiling = NULL,
};
