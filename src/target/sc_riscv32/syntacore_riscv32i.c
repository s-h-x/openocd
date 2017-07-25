/** @file

Syntacore RISC-V target

@copyright Syntacore
*/
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
#include <stdint.h>

/// Size of RISC-V GP registers in bits
#define XLEN (32u)
/// Size of RISC-V FP registers in bits
#define FLEN (64u)
#define FP_enabled !!1
/// Size of RISC-V instruction
#define ILEN (32u)
#define VERIFY_REG_WRITE 0
#define WRITE_BUFFER_THRESHOLD (1u << 18)
#define EXPECTED_IDCODE      (0xC0DEDEB1u)
#define EXPECTED_IDCODE_MASK (0xFFF0FFFFu)
/// Lowest required DBG_ID
#define EXPECTED_DBG_ID        (0x00800001u)
/// Mask of DBG_ID version.
/// Required and provided masked values should be equal.
#define DBG_ID_VERSION_MASK    (0xFFFFFF00u)
/// Mask of DBG_ID subversion.
/// Required value should be less or equal to provided subversion.
#define DBG_ID_SUBVERSION_MASK (0x000000FFu)
/// Don't make irscan if IR is the same
#define USE_IR_SELECT_CACHE 0
/// Don't write DAP_CONTROL if it is the same 
#define USE_DAP_CONTROL_CACHE 0
/// Verify value of DAP_CONTROL after write
#define USE_VERIFY_DAP_CONTROL 1
/// Verify values of HART REGTRANS after write
#define USE_VERIFY_HART_REGTRANS_WRITE 1
/// Verify values of CORE REGTRANS after write
#define USE_VERIFY_CORE_REGTRANS_WRITE 1
#define USE_PC_ADVMT_DSBL_BIT 1
#define USE_QUEUING_FOR_DR_SCANS 1
#define USE_CHECK_PC_UNCHANGED 1

#define STATIC_ASSERT2(COND,LINE) enum {static_assertion_at_line_##LINE= 1 / !!(COND)}
#define STATIC_ASSERT1(COND,LINE) STATIC_ASSERT2(COND,LINE)
#define STATIC_ASSERT(COND)  STATIC_ASSERT1(COND,__LINE__)
/// Bit mask value with low 'n' bits set
#define LOW_BITS_MASK(n) (~(~0 << (n)))
/// @return expression of TYPE with 'first_bit':'last_bit' bits set
#define MAKE_TYPE_FIELD(TYPE, bits, first_bit, last_bit)     ((((TYPE)(bits)) & LOW_BITS_MASK((last_bit) + 1u - (first_bit))) << (first_bit))
/// Number of array 'arr' elements
#define ARRAY_LEN(arr) (sizeof (arr) / sizeof (arr)[0])
/** Number of bytes need for 'num_bits' bits
@param [in] num_bits number of bits
@return number of bytes for 'num_bits' \f$\lceil {\frac{num\_bits}{CHAR\_BIT}} \rceil\f$.
*/
#define NUM_BITS_TO_SIZE(num_bits) ( ( (size_t)(num_bits) + (CHAR_BIT - 1) ) / CHAR_BIT )
#define EXTRACT_FIELD(bits, first_bit, last_bit) (((bits) >> (first_bit)) & LOW_BITS_MASK((last_bit) + 1u - (first_bit)))
#define IS_VALID_UNSIGNED_FIELD(FLD,LEN) ((FLD & ~LOW_BITS_MASK(LEN)) == 0)
#define NORMALIZE_INT_FIELD(FLD, SIGN_BIT, ZEROS) ( ( ( ( -( ( (FLD) >> (SIGN_BIT) ) & LOW_BITS_MASK(1) ) ) << (SIGN_BIT) ) | (FLD) ) & ~LOW_BITS_MASK(ZEROS) )
#define IS_VALID_SIGNED_IMMEDIATE_FIELD(FLD, SIGN_BIT, LOW_ZEROS) ( (FLD) == NORMALIZE_INT_FIELD((FLD), (SIGN_BIT), (LOW_ZEROS)) )
#define CHECK_REG(REG) assert(IS_VALID_UNSIGNED_FIELD(REG,5))
#define CHECK_OPCODE(OPCODE) assert(IS_VALID_UNSIGNED_FIELD(OPCODE,7) && (OPCODE & LOW_BITS_MASK(2)) == LOW_BITS_MASK(2) && (OPCODE & LOW_BITS_MASK(5)) != LOW_BITS_MASK(5))
#define CHECK_IMM_11_00(imm) assert(IS_VALID_SIGNED_IMMEDIATE_FIELD(imm, 11, 0));
#define CHECK_IMM_12_01(imm) assert(IS_VALID_SIGNED_IMMEDIATE_FIELD(imm, 12, 1));
#define CHECK_IMM_20_01(imm) assert(IS_VALID_SIGNED_IMMEDIATE_FIELD(imm, 20, 1));
#define CHECK_FUNC7(F) assert(IS_VALID_UNSIGNED_FIELD(F,7))
#define CHECK_FUNC3(F) assert(IS_VALID_UNSIGNED_FIELD(F,3))
#define BIT_NUM_TO_MASK(bit_num) (1u << (bit_num))

STATIC_ASSERT(CHAR_BIT == 8);

enum
{
    DAP_CTRL_value_INVALID_CODE = 0xFFu,
};

enum
{
    CSR_sptbr = 0x180,
    CSR_mstatus = 0x300,
    CSR_mcpuid = 0xf00,

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

    /// Debug controller CSR
    /// privilege: MRW
    CSR_sc_dbg_scratch = 0x788u,
};
/// RISCV privilege levels
enum
{
    Priv_U = 0x0,
    Priv_S = 0x1,
    Priv_H = 0x2,
    Priv_M = 0x3,
};
/// VM modes
enum
{
    VM_Mbare = 0,
    VM_Mbb = 1,
    VM_Mbbid = 2,
    VM_Sv32 = 8,
    VM_Sv39 = 9,
    VM_Sv48 = 10,
};
enum
{
    TAP_length_of_IR = 4,
    TAP_length_of_RO_32 = 32,
    TAP_length_of_IDCODE = TAP_length_of_RO_32,  ///< mandatory
    TAP_length_of_DBG_ID = TAP_length_of_RO_32,
    TAP_length_of_BLD_ID = TAP_length_of_RO_32,
    TAP_length_of_DBG_STATUS = TAP_length_of_RO_32,
    TAP_length_of_DAP_CTRL_UNIT = 2,
    TAP_length_of_DAP_CTRL_FGROUP = 2,
    TAP_length_of_DAP_CTRL = TAP_length_of_DAP_CTRL_UNIT + TAP_length_of_DAP_CTRL_FGROUP,
    TAP_number_of_fields_DAP_CMD = 2,
    TAP_length_of_DAP_CMD_OPCODE = 4,
    TAP_length_of_DAP_CMD_OPCODE_EXT = 32,
    TAP_length_of_DAP_CMD = TAP_length_of_DAP_CMD_OPCODE + TAP_length_of_DAP_CMD_OPCODE_EXT,
    TAP_length_of_BYPASS = 1,  ///< mandatory
};

/// type_dbgc_core_dbg_sts_reg_bits_e
/// @see TAP_INSTR_DBG_STATUS
enum
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

/// e_DAP_opstatus_bits
/// @see TAP_INSTR_DAP_CMD
/// @see TAP_INSTR_DAP_CTRL
enum
{
    DAP_opstatus_bit_EXCEPT = 0,
    DAP_opstatus_bit_ERROR = 1,
    DAP_opstatus_bit_LOCK = 2,
    DAP_opstatus_bit_READY = 3,
};

/// e_DAP_OPSTATUS
enum
{
    DAP_status_MASK = BIT_NUM_TO_MASK(DAP_opstatus_bit_ERROR) | BIT_NUM_TO_MASK(DAP_opstatus_bit_LOCK) | BIT_NUM_TO_MASK(DAP_opstatus_bit_READY),
    DAP_status_good = BIT_NUM_TO_MASK(DAP_opstatus_bit_READY),
};

/// Units IDs
typedef enum type_dbgc_unit_id_e
{
    DBGC_unit_id_HART_0 = 0,
    DBGC_unit_id_HART_1 = 1,
    DBGC_unit_id_CORE = 3,
} type_dbgc_unit_id_e;

/// Functional groups for HART units
///@{
/// type_dbgc_hart_fgroup_e
enum
{
    /// @see type_dbgc_regblock_hart_e
    DBGC_functional_group_HART_REGTRANS = 0,

    /// @see type_dbgc_dap_cmd_opcode_dbgcmd_e
    DBGC_functional_group_HART_DBGCMD = 1,
};

/// @see DBGC_functional_group_HART_REGTRANS
typedef enum type_dbgc_regblock_hart_e
{
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
} type_dbgc_regblock_hart_e;

/// type_dbgc_hart_dbg_ctrl_reg_bits_e
/// @see DBGC_HART_REGS_DBG_CTRL
enum
{
    /// HART reset bit
    /// @warning not used now
    DBGC_HART_HDCR_RST_BIT = 0,
    /// Disable pc change for instructions emitting from debugger
    DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT = 6,
};

/// type_dbgc_hart_dbg_sts_reg_bits_e
/// @see DBGC_HART_REGS_DBG_STS
enum
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
/// type_dbgc_hart_dmode_enbl_reg_bits_e
/// @see DBGC_HART_register_DMODE_ENBL
enum
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
/// type_dbgc_hart_dmode_cause_reg_bits_e
/// @see DBGC_HART_register_DMODE_CAUSE
enum
{
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
enum
{
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
enum
{
    DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_HALT = 0,
    DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_RESUME = 1,
    DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS = 2,
};

///@}

/// Functional groups for CORE units
///@{
/// type_dbgc_core_fgroup_e
enum
{
    /// @see type_dbgc_regblock_core_e
    DBGC_FGRP_CORE_REGTRANS = 0,
};

/// @see DBGC_FGRP_CORE_REGTRANS
typedef enum type_dbgc_regblock_core_e
{
    DBGC_CORE_REGS_DEBUG_ID = 0,

    /// @see type_dbgc_core_dbg_ctrl_reg_bits_e
    DBGC_CORE_REGS_DBG_CTRL = 1,
    DBGC_CORE_REGS_DBG_STS = 2,
#if 0
    DBGC_CORE_REGS_DBG_CMD = 3,
#endif
} type_dbgc_regblock_core_e;

/// Core Debug Control Register (CORE_DBG_CTRL, CDCR)
/// type_dbgc_core_dbg_ctrl_reg_bits_e
/// @see DBGC_CORE_REGS_DBG_CTRL
enum
{
    DBGC_CORE_CDCR_HART0_RST_BIT = 0,
    DBGC_CORE_CDCR_HART1_RST_BIT = 8,
    DBGC_CORE_CDCR_RST_BIT = 24,
    DBGC_CORE_CDCR_IRQ_DSBL_BIT = 25,
};
///@}

/// IR id
typedef enum TAP_IR_e
{
    TAP_instruction_DBG_ID = 3,
    TAP_instruction_BLD_ID = 4,
    TAP_instruction_DBG_STATUS = 5,
    TAP_instruction_DAP_CTRL = 6,
    TAP_instruction_DAP_CTRL_RD = 7,
    TAP_instruction_DAP_CMD = 8,
    TAP_instruction_IDCODE = 0xE,  ///< recommended
    TAP_instruction_BYPASS = 0xF,  ///< mandatory
} TAP_IR_e;

/// RISC-V GP registers id
enum
{
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

typedef struct sc_rv32i__Arch
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
} sc_rv32i__Arch;

static uint8_t const obj_DAP_opstatus_GOOD = DAP_status_good;
static uint8_t const obj_DAP_status_MASK = DAP_status_MASK;
static reg_data_type GP_reg_data_type = {.type = REG_TYPE_INT32,};
static reg_feature feature_riscv_org = {
    .name = "org.gnu.gdb.riscv.cpu",
};
static char const def_GP_regs_name[] = "general";
static sc_rv32i__Arch const sc_rv32_initial_arch = {
    .error_code = ERROR_OK,
    .last_DAP_ctrl = DAP_CTRL_value_INVALID_CODE,

    .use_ir_select_cache = !!(USE_IR_SELECT_CACHE),
    .use_dap_control_cache = !!(USE_DAP_CONTROL_CACHE),
    .use_verify_dap_control = !!(USE_VERIFY_DAP_CONTROL),
    .use_check_pc_unchanged = !!(USE_CHECK_PC_UNCHANGED),
    .use_verify_hart_regtrans_write = !!(USE_VERIFY_HART_REGTRANS_WRITE),
    .use_verify_core_regtrans_write = !!(USE_VERIFY_CORE_REGTRANS_WRITE),
    .use_pc_advmt_dsbl_bit = !!(USE_PC_ADVMT_DSBL_BIT),
    .use_queuing_for_dr_scans = !!(USE_QUEUING_FOR_DR_SCANS),
};
/// Error code handling
///@{
static int error_code__get(target const* const p_target)
{
    assert(p_target);
    sc_rv32i__Arch const* const p_arch = p_target->arch_info;
    assert(p_arch);
    return p_arch->error_code;
}
static int error_code__update(target const* const p_target, int const a_error_code)
{
    assert(p_target);
    sc_rv32i__Arch* const p_arch = p_target->arch_info;
    assert(p_arch);
    if (ERROR_OK == error_code__get(p_target) && ERROR_OK != a_error_code) {
        LOG_DEBUG("Set new error code: %d", a_error_code);
        p_arch->error_code = a_error_code;
    }
    return error_code__get(p_target);
}
static int error_code__get_and_clear(target const* const p_target)
{
    assert(p_target);
    sc_rv32i__Arch* const const p_arch = p_target->arch_info;
    assert(p_arch);
    int const result = error_code__get(p_target);
    p_arch->error_code = ERROR_OK;
    return result;
}
static int error_code__prepend(target const* const p_target, int const old_err_code)
{
    assert(p_target);
    int const new_err_code = error_code__get_and_clear(p_target);
    error_code__update(p_target, old_err_code);
    error_code__update(p_target, new_err_code);
    return error_code__get(p_target);
}
/// @}

static riscv_short_signed_type csr_to_int(csr_num_type csr)
{
    return NORMALIZE_INT_FIELD(csr, 11, 0);
}
static uint32_t RISCV_opcode_INSTR_R_TYPE(unsigned func7, reg_num_type rs2, reg_num_type rs1, uint8_t func3, reg_num_type rd, uint8_t opcode)
{
    CHECK_OPCODE(opcode);
    CHECK_FUNC3(func3);
    CHECK_FUNC7(func7);
    CHECK_REG(rs2);
    CHECK_REG(rs1);
    CHECK_REG(rd);
    return
        MAKE_TYPE_FIELD(uint32_t, func7, 25, 31) |
        MAKE_TYPE_FIELD(uint32_t, rs2, 20, 24) |
        MAKE_TYPE_FIELD(uint32_t, rs1, 15, 19) |
        MAKE_TYPE_FIELD(uint32_t, func3, 12, 14) |
        MAKE_TYPE_FIELD(uint32_t, rd, 7, 11) |
        MAKE_TYPE_FIELD(uint32_t, opcode, 0, 6);
}
static uint32_t RISCV_opcode_INSTR_I_TYPE(riscv_short_signed_type imm_11_00, reg_num_type rs1, uint8_t func3, reg_num_type rd, uint8_t opcode)
{
    CHECK_OPCODE(opcode);
    CHECK_REG(rd);
    CHECK_REG(rs1);
    CHECK_FUNC3(func3);
    CHECK_IMM_11_00(imm_11_00);
    return
        MAKE_TYPE_FIELD(uint32_t, EXTRACT_FIELD(imm_11_00, 0, 11), 20, 31) |
        MAKE_TYPE_FIELD(uint32_t, rs1, 15, 19) |
        MAKE_TYPE_FIELD(uint32_t, func3, 12, 14) |
        MAKE_TYPE_FIELD(uint32_t, rd, 7, 11) |
        MAKE_TYPE_FIELD(uint32_t, opcode, 0, 6);
}
static uint32_t RISCV_opcode_INSTR_S_TYPE(riscv_short_signed_type imm_11_00, reg_num_type rs2, reg_num_type rs1, unsigned func3, uint8_t opcode)
{
    CHECK_OPCODE(opcode);
    CHECK_REG(rs2);
    CHECK_REG(rs1);
    CHECK_FUNC3(func3);
    CHECK_IMM_11_00(imm_11_00);
    return
        MAKE_TYPE_FIELD(uint32_t, EXTRACT_FIELD(imm_11_00, 5, 11), 25, 31) |
        MAKE_TYPE_FIELD(uint32_t, rs2, 20, 24) |
        MAKE_TYPE_FIELD(uint32_t, rs1, 15, 19) |
        MAKE_TYPE_FIELD(uint32_t, func3, 12, 14) |
        MAKE_TYPE_FIELD(uint32_t, EXTRACT_FIELD(imm_11_00, 0, 4), 7, 11) |
        MAKE_TYPE_FIELD(uint32_t, opcode, 0, 6);
}
static uint32_t RISCV_opcode_INSTR_UJ_TYPE(riscv_signed_type imm_20_01, reg_num_type rd, uint8_t opcode)
{
    CHECK_OPCODE(opcode);
    CHECK_REG(rd);
    CHECK_IMM_20_01(imm_20_01);
    return
        MAKE_TYPE_FIELD(uint32_t, EXTRACT_FIELD(imm_20_01, 20, 20), 31, 31) |
        MAKE_TYPE_FIELD(uint32_t, EXTRACT_FIELD(imm_20_01, 1, 10), 21, 30) |
        MAKE_TYPE_FIELD(uint32_t, EXTRACT_FIELD(imm_20_01, 11, 11), 20, 20) |
        MAKE_TYPE_FIELD(uint32_t, EXTRACT_FIELD(imm_20_01, 12, 19), 12, 19) |
        MAKE_TYPE_FIELD(uint32_t, rd, 7, 11) |
        MAKE_TYPE_FIELD(uint32_t, opcode, 0, 6);
}
static uint32_t RISCV_opcode_FMV_X_S(reg_num_type rd, reg_num_type rs1_fp)
{
    return RISCV_opcode_INSTR_R_TYPE(0x70u, 0u, rs1_fp, 0u, rd, 0x53u);
}
static uint32_t RISCV_opcode_FMV_2X_D(reg_num_type rd_hi, reg_num_type rd_lo, reg_num_type rs1_fp)
{
    return RISCV_opcode_INSTR_R_TYPE(0x70u, rd_hi, rs1_fp, 0u, rd_lo, 0x53u);
}
static uint32_t RISCV_opcode_FMV_S_X(reg_num_type rd_fp, reg_num_type rs1)
{
    return RISCV_opcode_INSTR_R_TYPE(0x78u, 0u, rs1, 0u, rd_fp, 0x53u);
}
static uint32_t RISCV_opcode_FMV_D_2X(reg_num_type rd_fp, reg_num_type rs_hi, reg_num_type rs_lo)
{
    return RISCV_opcode_INSTR_R_TYPE(0x78u, rs_hi, rs_lo, 0u, rd_fp, 0x53u);
}
static uint32_t RISCV_opcode_LB(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
    return RISCV_opcode_INSTR_I_TYPE(imm, rs1, 0u, rd, 0x03u);
}
static uint32_t RISCV_opcode_LH(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
    return RISCV_opcode_INSTR_I_TYPE(imm, rs1, 1u, rd, 0x03u);
}
static uint32_t RISCV_opcode_LW(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
    return RISCV_opcode_INSTR_I_TYPE(imm, rs1, 2u, rd, 0x03u);
}
static uint32_t RISCV_opcode_ADDI(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
    return RISCV_opcode_INSTR_I_TYPE(imm, rs1, 0u, rd, 0x13u);
}
static uint32_t RISCV_opcode_JALR(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
    return RISCV_opcode_INSTR_I_TYPE(imm, rs1, 0u, rd, 0x67u);
}
static uint32_t RISCV_opcode_CSRRW(reg_num_type rd, csr_num_type csr, reg_num_type rs1)
{
    return RISCV_opcode_INSTR_I_TYPE(csr_to_int(csr), rs1, 1u, rd, 0x73u);
}
static uint32_t RISCV_opcode_CSRRS(reg_num_type rd, csr_num_type csr, reg_num_type rs1)
{
    return RISCV_opcode_INSTR_I_TYPE(csr_to_int(csr), rs1, 2u, rd, 0x73u);
}
static uint32_t RISCV_opcode_EBREAK(void)
{
    return RISCV_opcode_INSTR_I_TYPE(1, 0u, 0u, 0u, 0x73u);
}
static uint32_t RISCV_opcode_SB(reg_num_type rs_data, reg_num_type rs1, riscv_short_signed_type imm)
{
    return RISCV_opcode_INSTR_S_TYPE(imm, rs_data, rs1, 0u, 0x23);
}
static uint32_t RISCV_opcode_SH(reg_num_type rs, reg_num_type rs1, riscv_short_signed_type imm)
{
    return RISCV_opcode_INSTR_S_TYPE(imm, rs, rs1, 1u, 0x23);
}
static uint32_t RISCV_opcode_SW(reg_num_type rs, reg_num_type rs1, riscv_short_signed_type imm)
{
    return RISCV_opcode_INSTR_S_TYPE(imm, rs, rs1, 2u, 0x23);
}
static uint32_t RISCV_opcode_JAL(reg_num_type rd, riscv_signed_type imm_20_01)
{
    return RISCV_opcode_INSTR_UJ_TYPE(imm_20_01, rd, 0x6Fu);
}
static uint32_t RISCV_opcode_CSRW(unsigned csr, reg_num_type rs1)
{
    return RISCV_opcode_CSRRW(0, csr, rs1);
}
static uint32_t RISCV_opcode_CSRR(reg_num_type rd, csr_num_type csr)
{
    return RISCV_opcode_CSRRS(rd, csr, 0);
}
static uint16_t RISCV_opcode_C_EBREAK(void)
{
    return 0x9002u;
}
#if 0
static uint32_t RISCV_opcode_INSTR_SB_TYPE(riscv_short_signed_type imm_01_12, reg_num_type rs2, reg_num_type rs1, unsigned func3, uint8_t opcode)
{
    CHECK_OPCODE(opcode);
    CHECK_FUNC3(func3);
    CHECK_REG(rs1);
    CHECK_REG(rs2);
    CHECK_IMM_12_01(imm_01_12);
    return
        MAKE_TYPE_FIELD(uint32_t, EXTRACT_FIELD(imm_01_12, 12, 12), 31, 31) |
        MAKE_TYPE_FIELD(uint32_t, EXTRACT_FIELD(imm_01_12, 5, 10), 25, 30) |
        MAKE_TYPE_FIELD(uint32_t, rs2, 20, 24) |
        MAKE_TYPE_FIELD(uint32_t, rs1, 15, 19) |
        MAKE_TYPE_FIELD(uint32_t, func3, 12, 14) |
        MAKE_TYPE_FIELD(uint32_t, EXTRACT_FIELD(imm_01_12, 1, 4), 8, 11) |
        MAKE_TYPE_FIELD(uint32_t, EXTRACT_FIELD(imm_01_12, 11, 11), 7, 7) |
        MAKE_TYPE_FIELD(uint32_t, opcode, 0, 6);
}
static uint32_t RISCV_opcode_INSTR_U_TYPE(riscv_signed_type imm_31_12, reg_num_type rd, uint8_t opcode)
{
    CHECK_OPCODE(opcode);
    CHECK_REG(rd);
#define CHECK_IMM_31_12(imm) assert(IS_VALID_SIGNED_IMMEDIATE_FIELD(imm, 31, 12));
    CHECK_IMM_31_12(imm_31_12);
#undef CHECK_IMM_31_12
    return
        MAKE_TYPE_FIELD(uint32_t, EXTRACT_FIELD(imm_31_12, 12, 31), 12, 31) |
        MAKE_TYPE_FIELD(uint32_t, rd, 7, 11) |
        MAKE_TYPE_FIELD(uint32_t, opcode, 0, 6);
}
static uint32_t RISCV_opcode_ADD(reg_num_type rd, reg_num_type rs1, reg_num_type rs2)
{
    return RISCV_opcode_INSTR_R_TYPE(0x00u, rs2, rs1, 0u, rd, 0x33u);
}
static uint32_t RISCV_opcode_LBU(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
    return RISCV_opcode_INSTR_I_TYPE(imm, rs1, 4u, rd, 0x03u);
}
static uint32_t RISCV_opcode_LHU(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
    return RISCV_opcode_INSTR_I_TYPE(imm, rs1, 5u, rd, 0x03u);
}
static uint32_t RISCV_opcode_CSRRC(reg_num_type rd, csr_num_type csr, reg_num_type rs1)
{
    return RISCV_opcode_INSTR_I_TYPE(csr_to_int(csr), rs1, 3u, rd, 0x73u);
}
static uint32_t RISCV_opcode_CSRRWI(reg_num_type rd, csr_num_type csr, uint8_t zimm)
{
    return RISCV_opcode_INSTR_I_TYPE(csr_to_int(csr), zimm, 5u, rd, 0x73u);
}
static uint32_t RISCV_opcode_CSRRSI(reg_num_type rd, csr_num_type csr, uint8_t zimm)
{
    return RISCV_opcode_INSTR_I_TYPE(csr_to_int(csr), zimm, 6u, rd, 0x73u);
}
static uint32_t RISCV_opcode_CSRRCI(reg_num_type rd, csr_num_type csr, uint8_t zimm)
{
    return RISCV_opcode_INSTR_I_TYPE(csr_to_int(csr), zimm, 7u, rd, 0x73u);
}
static uint32_t RISCV_opcode_AUIPC(reg_num_type rd, riscv_signed_type imm)
{
    return RISCV_opcode_INSTR_U_TYPE(imm, rd, 0x17u);
}
static uint32_t RISCV_opcode_NOP(void)
{
    return RISCV_opcode_ADDI(0, 0, 0u);
}
#endif

/** @brief Always perform scan to write instruction register

Method can update error_code, but ignore previous errors
*/
static void IR_select_force(target const* const p_target, TAP_IR_e const new_instr)
{
    assert(p_target);
    assert(p_target->tap);
    assert(p_target->tap->ir_length == TAP_length_of_IR);
    uint8_t out_buffer[NUM_BITS_TO_SIZE(TAP_length_of_IR)] = {};
    buf_set_u32(out_buffer, 0, TAP_length_of_IR, new_instr);
    scan_field field = {.num_bits = p_target->tap->ir_length,.out_value = out_buffer};
    jtag_add_ir_scan(p_target->tap, &field, TAP_IDLE);
    LOG_DEBUG("irscan %s %d", p_target->cmd_name, new_instr);
}
/** @brief Cached version of instruction register

Method can update error_code, but ignore previous errors

*/
static void IR_select(target const* const p_target, TAP_IR_e const new_instr)
{
    assert(p_target);
    sc_rv32i__Arch const* const p_arch = p_target->arch_info;
    assert(p_arch);

    if (p_arch->use_ir_select_cache) {
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
/** @brief Common method to retrieve data of read-only 32-bits TAPs

Method store and update error_code, but ignore previous errors and
allows to repair error state of debug controller.
*/
static uint32_t read_only_32_bits_regs(target const* const p_target, TAP_IR_e ir)
{
    assert(p_target);
    uint32_t result = 0xBADC0DE0u;
    /// Low-level method save but ignore previous errors.
    int const old_err_code = error_code__get_and_clear(p_target);
    /// Error state can be updated in IR_select
    IR_select(p_target, ir);

    if (ERROR_OK == error_code__get(p_target)) {
        STATIC_ASSERT(TAP_length_of_RO_32 == 32);
        uint8_t result_buffer[NUM_BITS_TO_SIZE(TAP_length_of_RO_32)] = {};
        scan_field const field = {
            .num_bits = TAP_length_of_RO_32,
            .in_value = result_buffer,
        };
        assert(p_target->tap);
        jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);

        // enforce jtag_execute_queue() to obtain result
        error_code__update(p_target, jtag_execute_queue());
        LOG_DEBUG("drscan %s %d 0 ; # %08X", p_target->cmd_name, field.num_bits, buf_get_u32(result_buffer, 0, TAP_length_of_RO_32));

        if (ERROR_OK != error_code__get(p_target)) {
            /// Error state can be updated in DR scan
            LOG_ERROR("JTAG error %d", error_code__get(p_target));
        } else {
            STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_length_of_RO_32) <= sizeof(uint32_t));
            result = buf_get_u32(result_buffer, 0, TAP_length_of_RO_32);
        }
    }

    error_code__prepend(p_target, old_err_code);
    return result;
}
static uint32_t sc_rv32_IDCODE_get(target const* const p_target)
{
    assert(p_target);
    STATIC_ASSERT(TAP_length_of_IDCODE == TAP_length_of_RO_32);
    return read_only_32_bits_regs(p_target, TAP_instruction_IDCODE);
}
static uint32_t sc_rv32_DBG_ID_get(target const* const p_target)
{
    assert(p_target);
    STATIC_ASSERT(TAP_length_of_DBG_ID == TAP_length_of_RO_32);
    return read_only_32_bits_regs(p_target, TAP_instruction_DBG_ID);
}
static uint32_t sc_rv32_BLD_ID_get(target const* const p_target)
{
    assert(p_target);
    STATIC_ASSERT(TAP_length_of_BLD_ID == TAP_length_of_RO_32);
    return read_only_32_bits_regs(p_target, TAP_instruction_BLD_ID);
}
static uint32_t sc_rv32_DBG_STATUS_get(target const* const p_target)
{
    assert(p_target);
    STATIC_ASSERT(TAP_length_of_DBG_STATUS == TAP_length_of_RO_32);
    uint32_t const result = read_only_32_bits_regs(p_target, TAP_instruction_DBG_STATUS);
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
static void update_DAP_CTRL_cache(target const* const p_target, uint8_t const set_dap_unit_group)
{
    assert(p_target);
    sc_rv32i__Arch* const p_arch = p_target->arch_info;
    assert(p_arch);
    p_arch->last_DAP_ctrl = set_dap_unit_group;
}
static void invalidate_DAP_CTR_cache(target const* const p_target)
{
    update_DAP_CTRL_cache(p_target, DAP_CTRL_value_INVALID_CODE);
}
/// set unit/group
static void DAP_CTRL_REG_set_force(target const* const p_target, uint8_t const set_dap_unit_group)
{
    assert(p_target);
    int const old_err_code = error_code__get_and_clear(p_target);
    IR_select(p_target, TAP_instruction_DAP_CTRL);

    if (ERROR_OK == error_code__get(p_target)) {
        // clear status bits
        uint8_t status = 0;
        STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_length_of_DAP_CTRL) == sizeof status);
        STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_length_of_DAP_CTRL) == sizeof set_dap_unit_group);
        scan_field const field = {
            .num_bits = TAP_length_of_DAP_CTRL,
            .out_value = &set_dap_unit_group,
            .in_value = &status,
            .check_value = &obj_DAP_opstatus_GOOD,
            .check_mask = &obj_DAP_status_MASK,
        };

        invalidate_DAP_CTR_cache(p_target);
        jtag_add_dr_scan_check(p_target->tap, 1, &field, TAP_IDLE);

        // enforce jtag_execute_queue() to get status
        int const jtag_status = jtag_execute_queue();
        LOG_DEBUG("drscan %s %d 0x%1X ; # %1X", p_target->cmd_name, field.num_bits, set_dap_unit_group, status);

        if (ERROR_OK != jtag_status) {
            uint32_t const dbg_status = sc_rv32_DBG_STATUS_get(p_target);
            static uint32_t const dbg_status_check_value = BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT);
            static uint32_t const dbg_status_mask =
                BIT_NUM_TO_MASK(DBGC_CORE_CDSR_LOCK_BIT) |
                BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT);

            if ((dbg_status & dbg_status_mask) != (dbg_status_check_value & dbg_status_mask)) {
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
static void DAP_CTRL_REG_verify(target const* const p_target, uint8_t const set_dap_unit_group)
{
    assert(p_target);
    sc_rv32i__Arch* const p_arch = p_target->arch_info;
    assert(p_arch);
    p_arch->last_DAP_ctrl = DAP_CTRL_value_INVALID_CODE;
    int const old_err_code = error_code__get_and_clear(p_target);
    IR_select(p_target, TAP_instruction_DAP_CTRL_RD);

    if (ERROR_OK == error_code__get(p_target)) {
        uint8_t get_dap_unit_group = 0;
        uint8_t set_dap_unit_group_mask = 0x0Fu;
        STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_length_of_DAP_CTRL) == sizeof get_dap_unit_group);
        scan_field const field = {
            .num_bits = TAP_length_of_DAP_CTRL,
            .in_value = &get_dap_unit_group,
            .check_value = &set_dap_unit_group,
            .check_mask = &set_dap_unit_group_mask,
        };
        // enforce jtag_execute_queue() to get get_dap_unit_group
        jtag_add_dr_scan_check(p_target->tap, 1, &field, TAP_IDLE);
        error_code__update(p_target, jtag_execute_queue());
        LOG_DEBUG("drscan %s %d 0x%1X ; # %1X", p_target->cmd_name, field.num_bits, 0, get_dap_unit_group);

        if (ERROR_OK == error_code__get(p_target)) {
            if (get_dap_unit_group == set_dap_unit_group) {
                p_arch->last_DAP_ctrl = get_dap_unit_group;
            } else {
                LOG_ERROR("Unit/Group verification error: set 0x%1X, but get 0x%1X!", set_dap_unit_group, get_dap_unit_group);
                error_code__update(p_target, ERROR_TARGET_FAILURE);
            }
        }
    }

    error_code__prepend(p_target, old_err_code);
}
static void sc_rv32_DAP_CMD_scan(target const* const p_target, uint8_t const DAP_OPCODE, uint32_t const DAP_OPCODE_EXT, uint32_t* p_result)
{
    assert(p_target);
    int const old_err_code = error_code__get_and_clear(p_target);
    IR_select(p_target, TAP_instruction_DAP_CMD);

    if (ERROR_OK == error_code__get(p_target)) {
        // Input fields
        uint8_t DAP_OPSTATUS = 0;
        STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_length_of_DAP_CMD_OPCODE) == sizeof DAP_OPSTATUS);

        uint8_t dbg_data[NUM_BITS_TO_SIZE(TAP_length_of_DAP_CMD_OPCODE_EXT)] = {};
        uint8_t dap_opcode_ext[NUM_BITS_TO_SIZE(TAP_length_of_DAP_CMD_OPCODE_EXT)] = {};
        buf_set_u32(dap_opcode_ext, 0, TAP_length_of_DAP_CMD_OPCODE_EXT, DAP_OPCODE_EXT);
        scan_field const fields[2] = {
            {.num_bits = TAP_length_of_DAP_CMD_OPCODE_EXT,.out_value = dap_opcode_ext,.in_value = dbg_data},
            {.num_bits = TAP_length_of_DAP_CMD_OPCODE,.out_value = &DAP_OPCODE,.in_value = &DAP_OPSTATUS,.check_value = &obj_DAP_opstatus_GOOD,.check_mask = &obj_DAP_status_MASK,},
        };

        assert(p_target->tap);
        jtag_add_dr_scan_check(p_target->tap, ARRAY_LEN(fields), fields, TAP_IDLE);
        // enforse jtag_execute_queue() to get values
        error_code__update(p_target, jtag_execute_queue());
        LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X ; # %08X %1X", p_target->cmd_name,
                  fields[0].num_bits, DAP_OPCODE_EXT,
                  fields[1].num_bits, DAP_OPCODE,
                  buf_get_u32(dbg_data, 0, TAP_length_of_DAP_CMD_OPCODE_EXT), DAP_OPSTATUS);

        if (ERROR_OK == error_code__get(p_target)) {
            if ((DAP_OPSTATUS & DAP_status_MASK) != DAP_status_good) {
                LOG_ERROR("DAP_OPSTATUS == 0x%1X", (uint32_t)DAP_OPSTATUS);
                error_code__update(p_target, ERROR_TARGET_FAILURE);
            } else if (p_result) {
                *p_result = buf_get_u32(dbg_data, 0, TAP_length_of_DAP_CMD_OPCODE_EXT);
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
static uint32_t sc_rv32_DC__unlock(target const* const p_target)
{
    assert(p_target);
    LOG_WARNING("========= Try to unlock ==============");

    sc_rv32i__Arch* const p_arch = p_target->arch_info;
    assert(p_arch);

    assert(p_target->tap);
    assert(p_target->tap->ir_length == TAP_length_of_IR);

    {
        static uint8_t const ir_out_buffer_DAP_CTRL[NUM_BITS_TO_SIZE(TAP_length_of_IR)] = {TAP_instruction_DAP_CTRL};
        /// @todo jtag_add_ir_scan need non-const scan_field
        static scan_field ir_field_DAP_CTRL = {.num_bits = TAP_length_of_IR,.out_value = ir_out_buffer_DAP_CTRL};
        LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_instruction_DAP_CTRL);
        jtag_add_ir_scan(p_target->tap, &ir_field_DAP_CTRL, TAP_IDLE);
    }

    {
        static uint8_t const set_dap_unit_group = MAKE_TYPE_FIELD(uint8_t, DBGC_unit_id_HART_0, 2, 3) | MAKE_TYPE_FIELD(uint8_t, DBGC_functional_group_HART_DBGCMD, 0, 1);
        STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_length_of_DAP_CTRL) == sizeof set_dap_unit_group);
        static scan_field const dr_field_DAP_CTRL = {.num_bits = TAP_length_of_DAP_CTRL,.out_value = &set_dap_unit_group};
        p_arch->last_DAP_ctrl = DAP_CTRL_value_INVALID_CODE;
        LOG_DEBUG("drscan %s %d 0x%1X", p_target->cmd_name,
                  dr_field_DAP_CTRL.num_bits, set_dap_unit_group);
        jtag_add_dr_scan(p_target->tap, 1, &dr_field_DAP_CTRL, TAP_IDLE);
    }

    {
        static uint8_t const ir_out_buffer_DAP_CMD[NUM_BITS_TO_SIZE(TAP_length_of_IR)] = {TAP_instruction_DAP_CMD};
        static scan_field ir_field_DAP_CMD = {.num_bits = TAP_length_of_IR,.out_value = ir_out_buffer_DAP_CMD};
        LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_instruction_DAP_CMD);
        jtag_add_ir_scan(p_target->tap, &ir_field_DAP_CMD, TAP_IDLE);
    }

    uint8_t lock_context_buf[4] = {};
    {
        static uint8_t const dap_opcode_ext_UNLOCK[4] = {};
        static uint8_t const dap_opcode_UNLOCK = DBGC_DAP_OPCODE_DBGCMD_UNLOCK;
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
        static uint8_t const ir_out_buffer_DBG_STATUS[NUM_BITS_TO_SIZE(TAP_length_of_IR)] = {TAP_instruction_DBG_STATUS};
        static scan_field ir_field_DBG_STATUS = {.num_bits = TAP_length_of_IR,.out_value = ir_out_buffer_DBG_STATUS};
        LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_instruction_DBG_STATUS);
        jtag_add_ir_scan(p_target->tap, &ir_field_DBG_STATUS, TAP_IDLE);
    }

    {
        uint8_t status_buffer[4] = {};
        scan_field const dr_field_DBG_STATUS = {.num_bits = TAP_length_of_DBG_STATUS,.out_value = status_buffer,.in_value = status_buffer};
        jtag_add_dr_scan(p_target->tap, 1, &dr_field_DBG_STATUS, TAP_IDLE);

        // enforse jtag_execute_queue() to get values
        bool const ok =
            ERROR_OK == error_code__update(p_target, jtag_execute_queue()) &&
            0 == (buf_get_u32(status_buffer, 0, TAP_length_of_DAP_CMD_OPCODE_EXT) & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_LOCK_BIT));
        LOG_DEBUG("drscan %s %d 0x%08X ; # 0x%08X", p_target->cmd_name,
                  dr_field_DBG_STATUS.num_bits, 0,
                  buf_get_u32(status_buffer, 0, TAP_length_of_DAP_CMD_OPCODE_EXT));
        LOG_DEBUG("%s context=0x%08X, status=0x%08X", ok ? "Unlock succsessful!" : "Unlock unsuccsessful!",
                  buf_get_u32(lock_context_buf, 0, TAP_length_of_DAP_CMD_OPCODE_EXT),
                  buf_get_u32(status_buffer, 0, TAP_length_of_DAP_CMD_OPCODE_EXT));
    }
    return buf_get_u32(lock_context_buf, 0, TAP_length_of_DAP_CMD_OPCODE_EXT);
}
static void sc_rv32_HART0_clear_sticky(target* const p_target)
{
    LOG_DEBUG("========= Try to clear HART0 errors ============");
    assert(p_target);
    sc_rv32i__Arch* const p_arch = p_target->arch_info;
    assert(p_arch);
    assert(p_target->tap);
    assert(p_target->tap->ir_length == TAP_length_of_IR);

    {
        uint8_t ir_dap_ctrl_out_buffer[NUM_BITS_TO_SIZE(TAP_length_of_IR)] = {};
        buf_set_u32(ir_dap_ctrl_out_buffer, 0, TAP_length_of_IR, TAP_instruction_DAP_CTRL);
        scan_field ir_dap_ctrl_field = {.num_bits = p_target->tap->ir_length,.out_value = ir_dap_ctrl_out_buffer};
        jtag_add_ir_scan(p_target->tap, &ir_dap_ctrl_field, TAP_IDLE);
        LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_instruction_DAP_CTRL);
    }

    {
        /// set invalid cache value
        p_arch->last_DAP_ctrl = DAP_CTRL_value_INVALID_CODE;

        uint8_t const set_dap_unit_group = 0x1u;
        scan_field const dr_dap_ctrl_field = {.num_bits = TAP_length_of_DAP_CTRL,.out_value = &set_dap_unit_group};
        LOG_DEBUG("drscan %s 0x%1X 0x%08X ; ", p_target->cmd_name, dr_dap_ctrl_field.num_bits, set_dap_unit_group);
        jtag_add_dr_scan(p_target->tap, 1, &dr_dap_ctrl_field, TAP_IDLE);
    }

    {
        uint8_t ir_dap_cmd_out_buffer[NUM_BITS_TO_SIZE(TAP_length_of_IR)] = {};
        buf_set_u32(ir_dap_cmd_out_buffer, 0, TAP_length_of_IR, TAP_instruction_DAP_CMD);
        scan_field ir_dap_cmd_field = {.num_bits = p_target->tap->ir_length,.out_value = ir_dap_cmd_out_buffer};
        LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_instruction_DAP_CMD);
        jtag_add_ir_scan(p_target->tap, &ir_dap_cmd_field, TAP_IDLE);
    }

    {
        uint8_t dap_opcode_ext[(TAP_length_of_DAP_CMD_OPCODE_EXT + CHAR_BIT - 1) / CHAR_BIT];
        uint32_t const opcode_ext = BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS);
        buf_set_u32(dap_opcode_ext, 0, TAP_length_of_DAP_CMD_OPCODE_EXT, opcode_ext);
        uint8_t const dap_opcode = DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL;
        scan_field const fields[2] = {
            {.num_bits = TAP_length_of_DAP_CMD_OPCODE_EXT,.out_value = dap_opcode_ext},
            {.num_bits = TAP_length_of_DAP_CMD_OPCODE,.out_value = &dap_opcode}
        };
        LOG_DEBUG("drscan %s 0x%1X 0x%08X 0x%1X 0x%08X ; ", p_target->cmd_name,
                  fields[0].num_bits, opcode_ext,
                  fields[1].num_bits, DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL);
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
static void sc_rv32_CORE_clear_errors(target* const p_target)
{
    LOG_DEBUG("========= Try to clear core errors ============");
    assert(p_target);
    sc_rv32i__Arch* const p_arch = p_target->arch_info;
    assert(p_arch);
    assert(p_target->tap);
    assert(p_target->tap->ir_length == TAP_length_of_IR);

    {
        uint8_t ir_dap_ctrl_out_buffer[NUM_BITS_TO_SIZE(TAP_length_of_IR)] = {};
        buf_set_u32(ir_dap_ctrl_out_buffer, 0, TAP_length_of_IR, TAP_instruction_DAP_CTRL);
        scan_field ir_dap_ctrl_field = {.num_bits = p_target->tap->ir_length,.out_value = ir_dap_ctrl_out_buffer};
        jtag_add_ir_scan(p_target->tap, &ir_dap_ctrl_field, TAP_IDLE);
        LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_instruction_DAP_CTRL);
    }

    {
        /// set invalid cache value
        p_arch->last_DAP_ctrl = DAP_CTRL_value_INVALID_CODE;

        uint8_t const set_dap_unit_group = (DBGC_unit_id_CORE << TAP_length_of_DAP_CTRL_FGROUP) | DBGC_FGRP_CORE_REGTRANS;
        scan_field const field = {.num_bits = TAP_length_of_DAP_CTRL,.out_value = &set_dap_unit_group};
        LOG_DEBUG("drscan %s %d 0x%1X", p_target->cmd_name, field.num_bits, set_dap_unit_group);
        jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);
    }

    {
        uint8_t ir_dap_cmd_out_buffer[NUM_BITS_TO_SIZE(TAP_length_of_IR)] = {};
        buf_set_u32(ir_dap_cmd_out_buffer, 0, TAP_length_of_IR, TAP_instruction_DAP_CMD);
        scan_field ir_dap_cmd_field = {.num_bits = p_target->tap->ir_length,.out_value = ir_dap_cmd_out_buffer};
        jtag_add_ir_scan(p_target->tap, &ir_dap_cmd_field, TAP_IDLE);
        LOG_DEBUG("irscan %s %d", p_target->cmd_name, TAP_instruction_DAP_CMD);
    }

    {
        uint8_t dap_opcode_ext[(TAP_length_of_DAP_CMD_OPCODE_EXT + CHAR_BIT - 1) / CHAR_BIT];
        buf_set_u32(dap_opcode_ext, 0, TAP_length_of_DAP_CMD_OPCODE_EXT, 0xFFFFFFFF);
        uint8_t const dap_opcode = REGTRANS_scan_type(true, DBGC_CORE_REGS_DBG_STS);
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
static void sc_rv32_DAP_CTRL_REG_set(target const* const p_target, type_dbgc_unit_id_e const dap_unit, uint8_t const dap_group)
{
    assert(p_target);
    bool const match_HART_0 = DBGC_unit_id_HART_0 == dap_unit && 0 == p_target->coreid;
    bool const match_HART_1 = DBGC_unit_id_HART_1 == dap_unit && 1 == p_target->coreid;
    bool const HART_unit = match_HART_0 || match_HART_1;
    bool const HART_group = DBGC_functional_group_HART_REGTRANS == dap_group || DBGC_functional_group_HART_DBGCMD == dap_group;
    bool const CORE_unit = DBGC_unit_id_CORE == dap_unit;
    bool const CORE_group = DBGC_FGRP_CORE_REGTRANS == dap_group;
    assert((HART_unit && HART_group) || (CORE_unit && CORE_group));

    uint8_t const set_dap_unit_group =
        MAKE_TYPE_FIELD(uint8_t,
                        MAKE_TYPE_FIELD(uint8_t, dap_unit, TAP_length_of_DAP_CTRL_FGROUP, TAP_length_of_DAP_CTRL_FGROUP + TAP_length_of_DAP_CTRL_UNIT - 1) |
                        MAKE_TYPE_FIELD(uint8_t, dap_group, 0, TAP_length_of_DAP_CTRL_FGROUP - 1),
                        0,
                        TAP_length_of_DAP_CTRL_FGROUP + TAP_length_of_DAP_CTRL_UNIT - 1);

    sc_rv32i__Arch* const p_arch = p_target->arch_info;
    assert(p_arch);

    if (p_arch->use_dap_control_cache) {
        if (p_arch->last_DAP_ctrl == set_dap_unit_group) {
            return;
        }

        LOG_DEBUG("DAP_CTRL_REG of %s reset to 0x%1X", p_target->cmd_name, set_dap_unit_group);
    }

    invalidate_DAP_CTR_cache(p_target);
    int const old_err_code = error_code__get_and_clear(p_target);
    DAP_CTRL_REG_set_force(p_target, set_dap_unit_group);

    if (p_arch->use_verify_dap_control) {
        error_code__get_and_clear(p_target);
        DAP_CTRL_REG_verify(p_target, set_dap_unit_group);

        if (ERROR_OK == error_code__get(p_target)) {
            update_DAP_CTRL_cache(p_target, set_dap_unit_group);
        }
    }

    error_code__prepend(p_target, old_err_code);
}
static void REGTRANS_write(target const* const p_target, type_dbgc_unit_id_e a_unit, uint8_t const a_fgrp, uint8_t const index, uint32_t const data)
{
    assert(p_target);
    sc_rv32_DAP_CTRL_REG_set(p_target, a_unit, a_fgrp);

    if (ERROR_OK != error_code__get(p_target)) {
        LOG_WARNING("DAP_CTRL_REG_set error");
        return;
    } else {
        sc_rv32_DAP_CMD_scan(p_target, REGTRANS_scan_type(true, index), data, NULL);
    }
}
static uint32_t REGTRANS_read(target const* const p_target, type_dbgc_unit_id_e const a_unit, uint8_t const a_fgrp, uint8_t const index)
{
    assert(p_target);
    sc_rv32_DAP_CTRL_REG_set(p_target, a_unit, a_fgrp);

    if (error_code__get(p_target) != ERROR_OK) {
        LOG_WARNING("DAP_CTRL_REG_set error");
        return 0xBADC0DE3u;
    }

    sc_rv32_DAP_CMD_scan(p_target, REGTRANS_scan_type(false, index), 0, NULL);
    uint32_t result;
    sc_rv32_DAP_CMD_scan(p_target, REGTRANS_scan_type(false, index), 0, &result);
    return result;
}
static uint32_t sc_rv32_HART_REGTRANS_read(target const* const p_target, type_dbgc_regblock_hart_e const index)
{
    assert(p_target);
    type_dbgc_unit_id_e const unit = p_target->coreid == 0 ? DBGC_unit_id_HART_0 : DBGC_unit_id_HART_1;
    return REGTRANS_read(p_target, unit, DBGC_functional_group_HART_REGTRANS, index);
}
static void HART_REGTRANS_write(target const* const p_target, type_dbgc_regblock_hart_e const index, uint32_t const set_value)
{
    assert(p_target);
    type_dbgc_unit_id_e const unit = p_target->coreid == 0 ? DBGC_unit_id_HART_0 : DBGC_unit_id_HART_1;
    REGTRANS_write(p_target, unit, DBGC_functional_group_HART_REGTRANS, index, set_value);
}
static void sc_rv32_HART_REGTRANS_write_and_check(target const* const p_target, type_dbgc_regblock_hart_e const index, uint32_t const set_value)
{
    assert(p_target);
    HART_REGTRANS_write(p_target, index, set_value);

    if (error_code__get(p_target) == ERROR_OK) {
        sc_rv32i__Arch const* const p_arch = p_target->arch_info;
        assert(p_arch);

        if (p_arch->use_verify_hart_regtrans_write) {
            uint32_t const get_value = sc_rv32_HART_REGTRANS_read(p_target, index);

            if (get_value != set_value) {
                LOG_ERROR("Write HART_REGTRANS #%d with value 0x%08X, but re-read value is 0x%08X", (uint32_t)index, set_value, get_value);
                error_code__update(p_target, ERROR_TARGET_FAILURE);
            }
        }
    }
}
static uint32_t sc_rv32_core_REGTRANS_read(target const* const p_target, type_dbgc_regblock_core_e const index)
{
    assert(p_target);
    return REGTRANS_read(p_target, DBGC_unit_id_CORE, DBGC_FGRP_CORE_REGTRANS, index);
}
static void sc_rv32_CORE_REGTRANS_write(target const* const p_target, type_dbgc_regblock_core_e const index, uint32_t const data)
{
    assert(p_target);
    REGTRANS_write(p_target, DBGC_unit_id_CORE, DBGC_FGRP_CORE_REGTRANS, index, data);
}
static void sc_rv32_EXEC__setup(target const* const p_target)
{
    assert(p_target);

    if (error_code__get(p_target) == ERROR_OK) {
        sc_rv32_DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_unit_id_HART_0 : DBGC_unit_id_HART_1, DBGC_functional_group_HART_DBGCMD);
    }
}
static void sc_rv32_EXEC__push_data_to_CSR(target const* const p_target, uint32_t const csr_data)
{
    assert(p_target);

    if (error_code__get(p_target) == ERROR_OK) {
        sc_rv32_DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_DBGDATA_WR, csr_data, NULL);
    }
}
static uint32_t sc_rv32_EXEC__step(target const* const p_target, uint32_t instruction)
{
    assert(p_target);
    uint32_t result = 0xBADC0DE9u;

    if (ERROR_OK == error_code__get(p_target)) {
        sc_rv32_DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_CORE_EXEC, instruction, &result);
    }

    return result;
}
static uint32_t sc_rv32_get_PC(target const* const p_target)
{
    assert(p_target);
    sc_rv32i__Arch const* const p_arch = p_target->arch_info;
    assert(p_arch);

    if (p_arch->use_check_pc_unchanged) {
        return sc_rv32_HART_REGTRANS_read(p_target, DBGC_HART_register_PC_SAMPLE);
    } else {
        return 0xFFFFFFFFu;
    }
}
static target_state HART_status_bits_to_target_state(uint32_t const status)
{
    static uint32_t const err_bits =
        BIT_NUM_TO_MASK(DBGC_HART_HDSR_ERR_BIT) |
        BIT_NUM_TO_MASK(DBGC_HART_HDSR_ERR_HWTHREAD_BIT) |
        BIT_NUM_TO_MASK(DBGC_HART_HDSR_ERR_DAP_OPCODE_BIT) |
        BIT_NUM_TO_MASK(DBGC_HART_HDSR_ERR_DBGCMD_NACK_BIT);

    if (status & err_bits) {
        LOG_WARNING("Error status: 0x%08x", status);
        return TARGET_UNKNOWN;
    } else if (status & BIT_NUM_TO_MASK(DBGC_HART_HDSR_RST_BIT)) {
        return TARGET_RESET;
    } else if (status & BIT_NUM_TO_MASK(DBGC_HART_HDSR_DMODE_BIT)) {
        return TARGET_HALTED;
    } else {
        return TARGET_RUNNING;
    }
}
static uint32_t try_to_get_ready(target* const p_target)
{
    uint32_t core_status = sc_rv32_DBG_STATUS_get(p_target);

    if ((core_status & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT)) != 0) {
        return core_status;
    }

    static unsigned const max_retries = 10u;

    for (unsigned i = 2; i <= max_retries; ++i) {
        error_code__get_and_clear(p_target);
        core_status = sc_rv32_DBG_STATUS_get(p_target);

        if ((core_status & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT)) != 0) {
            LOG_DEBUG("Ready: 0x%08X after %d requests", core_status, i);
            return core_status;
        }
    }

    LOG_ERROR("Not ready: 0x%08X after %d requests", core_status, max_retries);
    return core_status;
}
static target_debug_reason read_debug_cause(target* const p_target)
{
    assert(p_target);
    uint32_t const value = sc_rv32_HART_REGTRANS_read(p_target, DBGC_HART_register_DMODE_CAUSE);

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
static void update_debug_reason(target* const p_target)
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
static void update_debug_status(target* const p_target)
{
    target_state const old_state = p_target->state;
    /// Only 1 HART available now
    assert(p_target->coreid == 0);
    uint32_t const HART_status = sc_rv32_HART_REGTRANS_read(p_target, DBGC_HART_register_DBG_STS);
    target_state const new_state =
        ERROR_OK != error_code__get(p_target) ? TARGET_UNKNOWN :
        HART_status_bits_to_target_state(HART_status);
    LOG_DEBUG("debug_status: old=%d, new=%d", old_state, new_state);
#if 1

    if (new_state == old_state) {
        return;
    }

#endif
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
static void check_and_repair_debug_controller_errors(target* const p_target)
{
    // protection from reset
    jtag_add_tlr();
    invalidate_DAP_CTR_cache(p_target);
    uint32_t const IDCODE = sc_rv32_IDCODE_get(p_target);

    if ((EXPECTED_IDCODE & EXPECTED_IDCODE_MASK) != (IDCODE & EXPECTED_IDCODE_MASK)) {
#if 0
        target_reset_examined(p_target);
#else
        p_target->examined = false;
#endif
        LOG_ERROR("Debug controller/JTAG error! Try to re-examine!");
        error_code__update(p_target, ERROR_TARGET_FAILURE);
        return;
    }

    uint32_t core_status = try_to_get_ready(p_target);

    if (0 != (core_status & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_LOCK_BIT))) {
        LOG_ERROR("Lock detected: 0x%08X", core_status);
        uint32_t const lock_context = sc_rv32_DC__unlock(p_target);

        if (ERROR_OK != error_code__get(p_target)) {
            /// return with error_code != ERROR_OK if unlock was unsuccsesful
            LOG_ERROR("Unlock unsucsessful with lock_context=0x%8X", lock_context);
            return;
        }

        core_status = sc_rv32_DBG_STATUS_get(p_target);
        LOG_INFO("Lock with lock_context=0x%8X repaired: 0x%08X", lock_context, core_status);
    }

    if (BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT) != (core_status & (BIT_NUM_TO_MASK(DBGC_CORE_CDSR_LOCK_BIT) | BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT)))) {
        LOG_ERROR("Core_status with LOCK!: 0x%08X", core_status);
        error_code__update(p_target, ERROR_TARGET_FAILURE);
        return;
    }

    static uint32_t const hart0_err_bits = BIT_NUM_TO_MASK(DBGC_CORE_CDSR_HART0_ERR_BIT) | BIT_NUM_TO_MASK(DBGC_CORE_CDSR_HART0_ERR_STKY_BIT);

    if (0 != (core_status & hart0_err_bits)) {
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

    if (0 != (core_status & cdsr_err_bits)) {
        LOG_WARNING("Core errors detected: 0x%08X", core_status);
        error_code__get_and_clear(p_target);
        sc_rv32_CORE_clear_errors(p_target);
        error_code__get_and_clear(p_target);
        core_status = sc_rv32_DBG_STATUS_get(p_target);

        if (0 != (core_status & cdsr_err_bits)) {
            LOG_ERROR("Core errors not fixed!: 0x%08X", core_status);
            error_code__update(p_target, ERROR_TARGET_FAILURE);
        } else {
            LOG_INFO("Core errors fixed: 0x%08X", core_status);
        }
    }
}
static void sc_rv32_update_status(target* const p_target)
{
    LOG_DEBUG("update_status");
    assert(p_target);
    int const old_err_code = error_code__get_and_clear(p_target);
    check_and_repair_debug_controller_errors(p_target);

    if (ERROR_OK == error_code__get(p_target)) {
        update_debug_status(p_target);
    }

    error_code__prepend(p_target, old_err_code);
}
static void sc_rv32_check_that_target_halted(target* const p_target)
{
    sc_rv32_update_status(p_target);

    if (ERROR_OK == error_code__get(p_target)) {
        if (p_target->state != TARGET_HALTED) {
            LOG_ERROR("Target not halted");
            error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
        }
    }
}
/// GP registers accessors
///@{
static inline void reg__invalidate(reg* const p_reg)
{
    assert(p_reg);

    if (p_reg->exist) {
        if (p_reg->dirty) {
            LOG_ERROR("Invalidate dirty register: %s", p_reg->name);
            target* const p_target = p_reg->arch_info;
            error_code__update(p_target, ERROR_TARGET_FAILURE);
        }

        p_reg->valid = false;
    }
}
static inline void reg__set_valid_value_to_cache(reg* const p_reg, uint32_t const value)
{
    assert(p_reg);
    assert(p_reg->exist);

    STATIC_ASSERT(CHAR_BIT == 8);
    assert(p_reg->size <= CHAR_BIT * sizeof value);

    LOG_DEBUG("Updating cache from register %s to 0x%08X", p_reg->name, value);

    assert(p_reg->value);
    buf_set_u32(p_reg->value, 0, p_reg->size, value);

    p_reg->valid = true;
    p_reg->dirty = false;
}
static void reg__set_new_cache_value(reg* const p_reg, uint8_t* const buf)
{
    assert(p_reg);
    assert(buf);

    assert(p_reg->exist);

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
static inline bool reg__check(reg const* const p_reg)
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
static void reg_cache__invalidate(reg_cache const* const p_reg_cache)
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
static void reg_cache__chain_invalidate(reg_cache* p_reg_cache)
{
    for (; p_reg_cache; p_reg_cache = p_reg_cache->next) {
        reg_cache__invalidate(p_reg_cache);
    }
}
static void reg_x__operation_conditions_check(reg const* const p_reg)
{
    assert(p_reg);
    assert(reg__check(p_reg));
    target* p_target = p_reg->arch_info;
    assert(p_target);

    if (p_reg->number >= number_of_regs_X) {
        LOG_WARNING("Bad GP register %s id=%d", p_reg->name, p_reg->number);
        error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
        return;
    }

    sc_rv32_check_that_target_halted(p_target);
}
static void sc_rv32_check_PC_value(target const* const p_target, uint32_t const pc_sample_1)
{
    assert(p_target);
    sc_rv32i__Arch const* const p_arch = p_target->arch_info;
    assert(p_arch);

    if (p_arch->use_check_pc_unchanged) {
        uint32_t const pc_sample_2 = sc_rv32_get_PC(p_target);

        if (pc_sample_2 != pc_sample_1) {
            LOG_ERROR("pc changed from 0x%08X to 0x%08X", pc_sample_1, pc_sample_2);
            error_code__update(p_target, ERROR_TARGET_FAILURE);
        }
    }
}
static int reg_x__get(reg* const p_reg)
{
    assert(p_reg);
    reg_x__operation_conditions_check(p_reg);
    target* p_target = p_reg->arch_info;
    assert(p_target);

    if (ERROR_OK == error_code__get(p_target)) {
        if (p_reg->valid) {
            // register cache already valid
            if (p_reg->dirty) {
                LOG_WARNING("Try re-read dirty cache register %s", p_reg->name);
            } else {
                LOG_DEBUG("Try re-read cache register %s", p_reg->name);
            }
        }

        uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);

        if (ERROR_OK == error_code__get(p_target)) {
            sc_rv32i__Arch const* const p_arch = p_target->arch_info;
            assert(p_arch);
            size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);

            if (p_arch->use_pc_advmt_dsbl_bit) {
                sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
            }

            if (ERROR_OK == error_code__get(p_target)) {
                sc_rv32_EXEC__setup(p_target);
                int advance_pc_counter = 0;

                if (ERROR_OK != error_code__get(p_target)) {
                    return error_code__get_and_clear(p_target);
                }

                // Save p_reg->number register to CSR_DBG_SCRATCH CSR
                (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRW(CSR_sc_dbg_scratch, p_reg->number));
                advance_pc_counter += instr_step;

                if (ERROR_OK != error_code__get(p_target)) {
                    return error_code__get_and_clear(p_target);
                }

                // Exec jump back to previous instruction and get saved into CSR_DBG_SCRATCH CSR value
                assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
                uint32_t const value = sc_rv32_EXEC__step(p_target, RISCV_opcode_JAL(0, -advance_pc_counter));
                advance_pc_counter = 0;

                if (ERROR_OK != error_code__get(p_target)) {
                    return error_code__get_and_clear(p_target);
                }

                reg__set_valid_value_to_cache(p_reg, value);

                if (p_arch->use_pc_advmt_dsbl_bit) {
                    sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, 0);
                }
            } else {
                sc_rv32_update_status(p_target);
            }

            sc_rv32_check_PC_value(p_target, pc_sample_1);
        } else {
            sc_rv32_update_status(p_target);
        }
    }

    return error_code__get_and_clear(p_target);
}
static int reg_x__store(reg* const p_reg)
{
    assert(p_reg);
    target* p_target = p_reg->arch_info;
    assert(p_target);
    uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);

    if (ERROR_OK != error_code__get(p_target)) {
        return error_code__get_and_clear(p_target);
    }

    sc_rv32i__Arch const* const p_arch = p_target->arch_info;

    assert(p_arch);

    size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);

    if (p_arch->use_pc_advmt_dsbl_bit) {
        sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
    }

    sc_rv32_EXEC__setup(p_target);
    int advance_pc_counter = 0;

    assert(p_reg->value);
    sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));

    if (ERROR_OK != error_code__get(p_target)) {
        return error_code__get_and_clear(p_target);
    }

    assert(p_reg->valid);
    assert(p_reg->dirty);
    (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_reg->number, CSR_sc_dbg_scratch));
    advance_pc_counter += instr_step;
    p_reg->dirty = false;

    LOG_DEBUG("Store register value 0x%08X from cache to register %s", buf_get_u32(p_reg->value, 0, p_reg->size), p_reg->name);

    if (ERROR_OK != error_code__get(p_target)) {
        return error_code__get_and_clear(p_target);
    }

#if VERIFY_REG_WRITE
    sc_rv32_EXEC__push_data_to_CSR(p_target, 0xFEEDBEEF);
    (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRW(CSR_sc_dbg_scratch, p_reg->number));
    advance_pc_counter += instr_step;
#endif
    /// Correct pc back after each instruction
    assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
#if VERIFY_REG_WRITE
    uint32_t const value = sc_rv32_EXEC__step(p_target, RISCV_opcode_JAL(0, -advance_pc_counter));

    if (buf_get_u32(p_reg->value, 0, p_reg->size) != value) {
        LOG_ERROR("Register %s write error: write 0x%08X, but re-read 0x%08X", p_reg->name, buf_get_u32(p_reg->value, 0, p_reg->size), value);
    }

#else
    (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_JAL(0, -advance_pc_counter));
#endif
    advance_pc_counter = 0;

    if (p_arch->use_pc_advmt_dsbl_bit) {
        sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, 0);
    }

    assert(reg__check(p_reg));
    sc_rv32_check_PC_value(p_target, pc_sample_1);

    return error_code__get_and_clear(p_target);
}
static int reg_x__set(reg* const p_reg, uint8_t* const buf)
{
    assert(p_reg);
    assert(buf);
    reg_x__operation_conditions_check(p_reg);
    target* p_target = p_reg->arch_info;
    assert(p_target);

    if (ERROR_OK != error_code__get(p_target)) {
        return error_code__get_and_clear(p_target);
    }

    reg__set_new_cache_value(p_reg, buf);

    /// store dirty register data to HW
    return reg_x__store(p_reg);
}
static reg_arch_type const reg_x_accessors = {
    .get = reg_x__get,
    .set = reg_x__set,
};
static int reg_x0__get(reg* const p_reg)
{
    assert(p_reg);
    assert(p_reg->number == 0u);
    reg__set_valid_value_to_cache(p_reg, 0u);
    return ERROR_OK;
}
static int reg_x0__set(reg* const p_reg, uint8_t* const buf)
{
    assert(p_reg);
    assert(buf);
    LOG_ERROR("Try to write to read-only register");
    assert(p_reg->number == 0u);
    reg__set_valid_value_to_cache(p_reg, 0u);
    return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
}
static reg_arch_type const reg_x0_accessors = {.get = reg_x0__get,.set = reg_x0__set,};
static reg* prepare_temporary_GP_register(target const* const p_target, int const after_reg)
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

            if (ERROR_OK != error_code__update(p_target, reg_x__get(p_valid))) {
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
static uint32_t csr_get_value(target* const p_target, uint32_t const csr_number)
{
    uint32_t value = 0xBADBAD;
    assert(p_target);
    sc_rv32_check_that_target_halted(p_target);

    if (ERROR_OK == error_code__get(p_target)) {
        /// Find temporary GP register
        reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);
        assert(p_wrk_reg);

        uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);

        if (ERROR_OK == error_code__get(p_target)) {
            sc_rv32i__Arch const* const p_arch = p_target->arch_info;
            assert(p_arch);
            size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);

            if (p_arch->use_pc_advmt_dsbl_bit) {
                sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
            }

            sc_rv32_EXEC__setup(p_target);
            int advance_pc_counter = 0;

            if (ERROR_OK == error_code__get(p_target)) {
                /// Copy values to temporary register
                (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_wrk_reg->number, csr_number));
                advance_pc_counter += instr_step;

                if (error_code__get(p_target) == ERROR_OK) {
                    /// and store temporary register to CSR_DBG_SCRATCH CSR.
                    (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRW(CSR_sc_dbg_scratch, p_wrk_reg->number));
                    advance_pc_counter += instr_step;

                    if (error_code__get(p_target) == ERROR_OK) {
                        /// Correct pc by jump 2 instructions back and get previous command result.
                        assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
                        value = sc_rv32_EXEC__step(p_target, RISCV_opcode_JAL(0, -advance_pc_counter));
                        advance_pc_counter = 0;
                    } else {
                        sc_rv32_update_status(p_target);
                    }
                } else {
                    sc_rv32_update_status(p_target);
                }
            } else {
                sc_rv32_update_status(p_target);
            }

            if (p_arch->use_pc_advmt_dsbl_bit) {
                sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, 0);
            }
        } else {
            sc_rv32_update_status(p_target);
        }

        sc_rv32_check_PC_value(p_target, pc_sample_1);

        // restore temporary register
        int const old_err_code = error_code__get_and_clear(p_target);
        error_code__update(p_target, reg_x__store(p_wrk_reg));
        error_code__prepend(p_target, old_err_code);
        assert(!p_wrk_reg->dirty);
    }

    return value;
}
static bool is_RVC_enable(target* const p_target)
{
    uint32_t const mcpuid = csr_get_value(p_target, CSR_mcpuid);
    return 0 != (mcpuid & (1u << ('C' - 'A')));
}
/// Update pc cache from HW (if non-cached)
static int reg_pc__get(reg* const p_reg)
{
    assert(p_reg);
    assert(p_reg->number == RISCV_regnum_PC);
    assert(reg__check(p_reg));

    /// Find temporary GP register
    target* const p_target = p_reg->arch_info;
    assert(p_target);
    sc_rv32i__Arch const* const p_arch = p_target->arch_info;
    assert(p_arch);
    uint32_t const pc_sample = sc_rv32_HART_REGTRANS_read(p_target, DBGC_HART_register_PC_SAMPLE);

    if (ERROR_OK == error_code__get(p_target)) {
        reg__set_valid_value_to_cache(p_reg, pc_sample);
    } else {
        reg__invalidate(p_reg);
    }

    return error_code__get_and_clear(p_target);
}
static int reg_pc__set(reg* const p_reg, uint8_t* const buf)
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

    if (ERROR_OK != error_code__get(p_target)) {
        return error_code__get_and_clear(p_target);
    }

    uint32_t const new_pc = buf_get_u32(buf, 0, p_reg->size);

    /// @note odd address is valid for pc, bit 0 value is ignored.
    if (0 != (new_pc & (1u << 1))) {
        bool const RVC_enable = is_RVC_enable(p_target);

        if (ERROR_OK != error_code__get(p_target)) {
            return error_code__get_and_clear(p_target);
        } else if (!RVC_enable) {
            LOG_ERROR("Unaligned PC: 0x%08X", new_pc);
            error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
            return error_code__get_and_clear(p_target);
        }
    }

    reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);

    assert(p_wrk_reg);

    reg__set_new_cache_value(p_reg, buf);

    sc_rv32i__Arch const* const p_arch = p_target->arch_info;

    assert(p_arch);

    size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);

    if (p_arch->use_pc_advmt_dsbl_bit) {
        sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
    }

    // Update to HW
    sc_rv32_EXEC__setup(p_target);
    int advance_pc_counter = 0;

    if (ERROR_OK == error_code__get(p_target)) {
        assert(p_reg->value);
        sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));

        if (ERROR_OK == error_code__get(p_target)) {
            // set temporary register value to restoring pc value
            (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_wrk_reg->number, CSR_sc_dbg_scratch));
            advance_pc_counter += instr_step;

            if (ERROR_OK == error_code__get(p_target)) {
                assert(p_wrk_reg->dirty);

                if (p_arch->use_pc_advmt_dsbl_bit) {
                    sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, 0);
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

    sc_rv32_update_status(p_target);

#if 0

    if (p_arch->use_pc_advmt_dsbl_bit) {
        HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, 0);
    }

#endif
    // restore temporary register
    int const old_err_code = error_code__get_and_clear(p_target);
    error_code__update(p_target, reg_x__store(p_wrk_reg));
    error_code__prepend(p_target, old_err_code);
    assert(!p_wrk_reg->dirty);

    return error_code__get_and_clear(p_target);
}
static reg_arch_type const reg_pc_accessors = {
    .get = reg_pc__get,
    .set = reg_pc__set,
};
static reg_data_type PC_reg_data_type = {
    .type = REG_TYPE_CODE_PTR,
};
#if FLEN == 32

static int reg_fs__get(reg* const p_reg)
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

    if (error_code__get(p_target) != ERROR_OK) {
        return error_code__get_and_clear(p_target);
    }

    /// @todo check that FPU is enabled
    /// Find temporary GP register
    reg* const p_wrk_reg_1 = prepare_temporary_GP_register(p_target, 0);
    assert(p_wrk_reg_1);

    uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);

    if (error_code__get(p_target) == ERROR_OK) {
        sc_rv32i__Arch const* const p_arch = p_target->arch_info;
        assert(p_arch);
        size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);

        if (p_arch->use_pc_advmt_dsbl_bit) {
            sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
        }

        sc_rv32_EXEC__setup(p_target);
        int advance_pc_counter = 0;

        if (error_code__get(p_target) == ERROR_OK) {
            /// Copy values to temporary register
            (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_FMV_X_S(p_wrk_reg_1->number, p_reg->number - RISCV_regnum_FP_first));
            advance_pc_counter += instr_step;

            if (error_code__get(p_target) == ERROR_OK) {
                /// and store temporary register to CSR_DBG_SCRATCH CSR.
                (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRW(CSR_sc_dbg_scratch, p_wrk_reg_1->number));
                advance_pc_counter += instr_step;

                if (error_code__get(p_target) == ERROR_OK) {
                    assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
                    uint32_t const value = sc_rv32_EXEC__step(p_target, RISCV_opcode_JAL(0, -advance_pc_counter));
                    advance_pc_counter = 0;

                    if (error_code__get(p_target) == ERROR_OK) {
                        buf_set_u32(p_reg->value, 0, p_reg->size, value);
                        p_reg->valid = true;
                        p_reg->dirty = false;
                    }
                }
            }
        }

        if (p_arch->use_pc_advmt_dsbl_bit) {
            sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, 0);
        }
    }

    sc_rv32_check_PC_value(p_target, pc_sample_1);

    // restore temporary register
    int const old_err_code = error_code__get_and_clear(p_target);

    if (error_code__update(p_target, reg_x__store(p_wrk_reg_1)) == ERROR_OK) {
        assert(!p_wrk_reg_1->dirty);
    }

    error_code__prepend(p_target, old_err_code);

    return error_code__get_and_clear(p_target);
}
static int reg_fs__set(reg* const p_reg, uint8_t* const buf)
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

    if (error_code__get(p_target) != ERROR_OK) {
        return error_code__get_and_clear(p_target);
    }

    /// @todo check that FPU is enabled
    /// Find temporary GP register
    reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);
    assert(p_wrk_reg);

    uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);

    if (error_code__get(p_target) == ERROR_OK) {
        sc_rv32i__Arch const* const p_arch = p_target->arch_info;
        assert(p_arch);
        size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);

        if (p_arch->use_pc_advmt_dsbl_bit) {
            sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
        }

        sc_rv32_EXEC__setup(p_target);
        int advance_pc_counter = 0;

        if (error_code__get(p_target) == ERROR_OK) {
            reg__set_new_cache_value(p_reg, buf);

            sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));

            if (error_code__get(p_target) == ERROR_OK) {
                // set temporary register value to restoring pc value
                (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_wrk_reg->number, CSR_sc_dbg_scratch));
                advance_pc_counter += instr_step;

                if (error_code__get(p_target) == ERROR_OK) {
                    assert(p_wrk_reg->dirty);
                    assert(0 < p_wrk_reg->number && p_wrk_reg->number < RISCV_regnum_PC);
                    (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_FMV_S_X(p_reg->number - RISCV_regnum_FP_first, p_wrk_reg->number));
                    advance_pc_counter += instr_step;

                    if (error_code__get(p_target) == ERROR_OK) {
                        /// Correct pc by jump 2 instructions back and get previous command result.
                        assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);

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

        if (p_arch->use_pc_advmt_dsbl_bit) {
            sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, 0);
        }
    }

    sc_rv32_check_PC_value(p_target, pc_sample_1);
    // restore temporary register
    int const old_err_code = error_code__get_and_clear(p_target);
    error_code__update(p_target, reg_x__store(p_wrk_reg));
    error_code__prepend(p_target, old_err_code);
    assert(!p_wrk_reg->dirty);
    return error_code__get_and_clear(p_target);
}
static reg_arch_type const reg_f_accessors = {.get = reg_fs__get,.set = reg_fs__set,};
static reg_data_type FP_reg_data_type = {.type = REG_TYPE_IEEE_SINGLE,};

#elif FLEN == 64

static int reg_fd__get(reg* const p_reg)
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
    assert(p_target);

    sc_rv32_check_that_target_halted(p_target);

    if (ERROR_OK != error_code__get(p_target)) {
        return error_code__get_and_clear(p_target);
    }

    uint32_t const mcpuid = csr_get_value(p_target, CSR_mcpuid);

    if (ERROR_OK != error_code__get(p_target)) {
        return error_code__get_and_clear(p_target);
    }

    if (0 == (mcpuid & (BIT_NUM_TO_MASK('f' - 'a') | BIT_NUM_TO_MASK('d' - 'a')))) {
        LOG_ERROR("FPU is not supported");
        error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
        return error_code__get_and_clear(p_target);
    }

    uint32_t const mstatus = csr_get_value(p_target, CSR_mstatus);

    if (ERROR_OK != error_code__get(p_target)) {
        return error_code__get_and_clear(p_target);
    }

    if (0 == (mstatus & (3u << 12))) {
        LOG_ERROR("FPU is disabled");
        error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
        return error_code__get_and_clear(p_target);
    }

    bool const FPU_D = 0 != (mcpuid & BIT_NUM_TO_MASK('d' - 'a'));

    /// Find temporary GP register
    reg* const p_wrk_reg_1 = prepare_temporary_GP_register(p_target, 0);
    assert(p_wrk_reg_1);
    reg* const p_wrk_reg_2 = prepare_temporary_GP_register(p_target, p_wrk_reg_1->number);
    assert(p_wrk_reg_2);

    uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);

    if (ERROR_OK == error_code__get(p_target)) {
        sc_rv32i__Arch const* const p_arch = p_target->arch_info;
        assert(p_arch);
        size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);

        if (p_arch->use_pc_advmt_dsbl_bit) {
            sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
        }

        sc_rv32_EXEC__setup(p_target);
        int advance_pc_counter = 0;

        if (ERROR_OK == error_code__get(p_target)) {
            uint32_t const opcode_1 =
                FPU_D ?
                RISCV_opcode_FMV_2X_D(p_wrk_reg_2->number, p_wrk_reg_1->number, p_reg->number - RISCV_regnum_FP_first) :
                RISCV_opcode_FMV_X_S(p_wrk_reg_1->number, p_reg->number - RISCV_regnum_FP_first);

            (void)sc_rv32_EXEC__step(p_target, opcode_1);
            advance_pc_counter += instr_step;

            if (ERROR_OK == error_code__get(p_target)) {
                /// and store temporary register to CSR_DBG_SCRATCH CSR.
                (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRW(CSR_sc_dbg_scratch, p_wrk_reg_1->number));
                advance_pc_counter += instr_step;

                if (ERROR_OK == error_code__get(p_target)) {
                    uint32_t const value_lo = sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRW(CSR_sc_dbg_scratch, p_wrk_reg_2->number));
                    advance_pc_counter += instr_step;

                    if (ERROR_OK == error_code__get(p_target)) {
                        /// Correct pc by jump 2 instructions back and get previous command result.
                        assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
                        uint32_t const value_hi = sc_rv32_EXEC__step(p_target, RISCV_opcode_JAL(0, -advance_pc_counter));
                        advance_pc_counter = 0;

                        if (ERROR_OK == error_code__get(p_target)) {
                            buf_set_u64(p_reg->value, 0, p_reg->size, (FPU_D ? (uint64_t)value_hi << 32 : 0u) | (uint64_t)value_lo);
                            p_reg->valid = true;
                            p_reg->dirty = false;
                        } else {
                            sc_rv32_update_status(p_target);
                        }
                    } else {
                        sc_rv32_update_status(p_target);
                    }
                } else {
                    sc_rv32_update_status(p_target);
                }
            } else {
                sc_rv32_update_status(p_target);
            }
        } else {
            sc_rv32_update_status(p_target);
        }

        if (p_arch->use_pc_advmt_dsbl_bit) {
            sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, 0);
        }
    }

    sc_rv32_check_PC_value(p_target, pc_sample_1);

    // restore temporary register
    int const old_err_code = error_code__get_and_clear(p_target);

    if (ERROR_OK == error_code__update(p_target, reg_x__store(p_wrk_reg_2))) {
        assert(!p_wrk_reg_2->dirty);
    }

    if (ERROR_OK == error_code__update(p_target, reg_x__store(p_wrk_reg_1))) {
        assert(!p_wrk_reg_1->dirty);
    }

    error_code__prepend(p_target, old_err_code);

    return error_code__get_and_clear(p_target);
}
static int reg_fd__set(reg* const p_reg, uint8_t* const buf)
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

    if (ERROR_OK != error_code__get(p_target)) {
        return error_code__get_and_clear(p_target);
    }

    uint32_t const mcpuid = csr_get_value(p_target, CSR_mcpuid);

    if (ERROR_OK != error_code__get(p_target)) {
        return error_code__get_and_clear(p_target);
    }

    if (0 == (mcpuid & (BIT_NUM_TO_MASK('f' - 'a') | BIT_NUM_TO_MASK('d' - 'a')))) {
        LOG_ERROR("FPU is not supported");
        error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
        return error_code__get_and_clear(p_target);
    }

    uint32_t const mstatus = csr_get_value(p_target, CSR_mstatus);

    if (ERROR_OK != error_code__get(p_target)) {
        return error_code__get_and_clear(p_target);
    }

    if (0 == (mstatus & (3u << 12))) {
        LOG_ERROR("FPU is disabled");
        error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
        return error_code__get_and_clear(p_target);
    }

    bool const FPU_D = 0 != (mcpuid & BIT_NUM_TO_MASK('d' - 'a'));

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

    if (ERROR_OK == error_code__get(p_target)) {
        sc_rv32i__Arch const* const p_arch = p_target->arch_info;
        assert(p_arch);
        size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);

        if (p_arch->use_pc_advmt_dsbl_bit) {
            sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
        }

        sc_rv32_EXEC__setup(p_target);
        int advance_pc_counter = 0;

        if (ERROR_OK == error_code__get(p_target)) {
            reg__set_new_cache_value(p_reg, buf);
            sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));

            if (ERROR_OK == error_code__get(p_target)) {
                // set temporary register value to restoring pc value
                (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_wrk_reg_1->number, CSR_sc_dbg_scratch));
                advance_pc_counter += instr_step;

                if (ERROR_OK == error_code__get(p_target)) {
                    sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(&((uint8_t const*)p_reg->value)[4], 0, p_reg->size));

                    if (ERROR_OK == error_code__get(p_target)) {
                        (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_wrk_reg_2->number, CSR_sc_dbg_scratch));
                        advance_pc_counter += instr_step;

                        if (ERROR_OK == error_code__get(p_target)) {
                            uint32_t const opcode_1 =
                                FPU_D ?
                                RISCV_opcode_FMV_D_2X(p_reg->number - RISCV_regnum_FP_first, p_wrk_reg_2->number, p_wrk_reg_1->number) :
                                RISCV_opcode_FMV_S_X(p_reg->number - RISCV_regnum_FP_first, p_wrk_reg_1->number);
                            (void)sc_rv32_EXEC__step(p_target, opcode_1);
                            advance_pc_counter += instr_step;

                            if (ERROR_OK == error_code__get(p_target)) {
                                /// Correct pc by jump 2 instructions back and get previous command result.
                                assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);

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

        sc_rv32_update_status(p_target);

        if (p_arch->use_pc_advmt_dsbl_bit) {
            sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, 0);
        }
    } else {
        sc_rv32_update_status(p_target);
    }

    sc_rv32_check_PC_value(p_target, pc_sample_1);
    // restore temporary register
    int const old_err_code = error_code__get_and_clear(p_target);

    if (ERROR_OK == error_code__update(p_target, reg_x__store(p_wrk_reg_2))) {
        assert(!p_wrk_reg_2->dirty);
    }

    if (ERROR_OK == error_code__update(p_target, reg_x__store(p_wrk_reg_1))) {
        assert(!p_wrk_reg_1->dirty);
    }

    error_code__prepend(p_target, old_err_code);
    return error_code__get_and_clear(p_target);
}
static reg_arch_type const reg_f_accessors = {.get = reg_fd__get,.set = reg_fd__set,};
static reg_data_type FP_s_type = {.type = REG_TYPE_IEEE_SINGLE,.id = "ieee_single",};
static reg_data_type FP_d_type = {.type = REG_TYPE_IEEE_DOUBLE,.id = "ieee_double",};
static reg_data_type_union_field FP_s = {.name = "S",.type = &FP_s_type};
static reg_data_type_union_field FP_d = {.name = "D",.type = &FP_d_type,.next = &FP_s};
static reg_data_type_union FP_s_or_d = {.fields = &FP_d};
static reg_data_type FP_reg_data_type = {.type = REG_TYPE_ARCH_DEFINED,.id = "Float_D_or_S",.type_class = REG_TYPE_CLASS_UNION,.reg_type_union = &FP_s_or_d};

#else

#error Invalid FLEN

#endif

static int reg_csr__get(reg* const p_reg)
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
    uint32_t const value = csr_get_value(p_target, csr_number);

    if (ERROR_OK == error_code__get(p_target)) {
        reg__set_valid_value_to_cache(p_reg, value);
    }

    return error_code__get_and_clear(p_target);
}
static int reg_csr__set(reg* const p_reg, uint8_t* const buf)
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

    if (ERROR_OK != error_code__get(p_target)) {
        return error_code__get_and_clear(p_target);
    }

    reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);

    assert(p_wrk_reg);

    uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);

    if (ERROR_OK == error_code__get(p_target)) {
        sc_rv32i__Arch const* const p_arch = p_target->arch_info;
        assert(p_arch);
        size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);

        if (p_arch->use_pc_advmt_dsbl_bit) {
            sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
        }

        sc_rv32_EXEC__setup(p_target);
        int advance_pc_counter = 0;

        if (ERROR_OK == error_code__get(p_target)) {
            reg__set_new_cache_value(p_reg, buf);

            sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));

            if (ERROR_OK == error_code__get(p_target)) {
                // set temporary register value
                (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_wrk_reg->number, CSR_sc_dbg_scratch));
                advance_pc_counter += instr_step;

                if (ERROR_OK == error_code__get(p_target)) {
                    assert(p_wrk_reg->dirty);
                    assert(p_wrk_reg->number < number_of_regs_X);
                    (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRW(p_reg->number - RISCV_regnum_CSR_first, p_wrk_reg->number));
                    advance_pc_counter += instr_step;

                    if (ERROR_OK == error_code__get(p_target)) {
                        /// Correct pc by jump 2 instructions back and get previous command result.
                        assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);

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

        sc_rv32_update_status(p_target);

        if (p_arch->use_pc_advmt_dsbl_bit) {
            sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, 0);
        }
    } else {
        sc_rv32_update_status(p_target);
    }

    sc_rv32_check_PC_value(p_target, pc_sample_1);
    // restore temporary register
    int const old_err_code = error_code__get_and_clear(p_target);
    error_code__update(p_target, reg_x__store(p_wrk_reg));
    error_code__prepend(p_target, old_err_code);
    assert(!p_wrk_reg->dirty);
    return error_code__get_and_clear(p_target);
}
static reg_arch_type const reg_csr_accessors = {
    .get = reg_csr__get,
    .set = reg_csr__set,
};
static reg_cache* reg_cache__section_create(char const* name, reg const regs_templates[], size_t const num_regs, void* const p_arch_info)
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
static void set_DEMODE_ENBL(target* const p_target, uint32_t const set_value)
{
    sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DMODE_ENBL, set_value);
}
static int resume_common(target* const p_target, uint32_t dmode_enabled, int const current, uint32_t const address, int const handle_breakpoints, int const debug_execution)
{
    assert(p_target);
    sc_rv32_check_that_target_halted(p_target);

    if (ERROR_OK != error_code__get(p_target)) {
        return error_code__get_and_clear(p_target);
    }

    // TODO: multiple caches
    // PC cache
    reg* const p_pc = &p_target->reg_cache->reg_list[number_of_regs_X];

    if (!current) {
        // setup new PC
        uint8_t buf[sizeof address];
        buf_set_u32(buf, 0, XLEN, address);

        if (ERROR_OK != error_code__update(p_target, reg_pc__set(p_pc, buf))) {
            return error_code__get_and_clear(p_target);
        }

        assert(!p_pc->dirty);
    }

    if (handle_breakpoints) {
        if (current) {
            // Find breakpoint for current instruction
            error_code__update(p_target, reg_pc__get(p_pc));
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
                sc_rv32i__Arch const* const p_arch = p_target->arch_info;
                assert(p_arch);
                // save breakpoint
                breakpoint next_bkp = *p_next_bkp;
                // remove breakpoint
                error_code__update(p_target, target_remove_breakpoint(p_target, &next_bkp));
                // prepare for single step
                reg_cache__chain_invalidate(p_target->reg_cache);
                // force single step
                set_DEMODE_ENBL(p_target, dmode_enabled | BIT_NUM_TO_MASK(DBGC_HART_HDMER_SINGLE_STEP_BIT));
                sc_rv32_DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_unit_id_HART_0 : DBGC_unit_id_HART_1, DBGC_functional_group_HART_DBGCMD);
                // resume for single step
                sc_rv32_DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_RESUME) | BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS), NULL);
                // restore breakpoint
                error_code__update(p_target, target_add_breakpoint(p_target, &next_bkp));

                // If resume/halt already done (by single step)
                if (0 != (dmode_enabled & BIT_NUM_TO_MASK(DBGC_HART_HDMER_SINGLE_STEP_BIT))) {
                    // TODO: extra call
                    reg_cache__chain_invalidate(p_target->reg_cache);
                    // set status
                    p_target->state = debug_execution ? TARGET_DEBUG_RUNNING : TARGET_RUNNING;
                    // raise resume event
                    target_call_event_callbacks(p_target, debug_execution ? TARGET_EVENT_DEBUG_RESUMED : TARGET_EVENT_RESUMED);
                    // setup debug mode
                    set_DEMODE_ENBL(p_target, dmode_enabled);
                    // set debug reason
                    sc_rv32_update_status(p_target);
                    LOG_DEBUG("New debug reason: 0x%08X", DBG_REASON_SINGLESTEP);
                    p_target->debug_reason = DBG_REASON_SINGLESTEP;
                    // raise halt event
                    target_call_event_callbacks(p_target, debug_execution ? TARGET_EVENT_DEBUG_HALTED : TARGET_EVENT_HALTED);
                    // and exit
                    return error_code__get_and_clear(p_target);
                }
            }
        }

        // dmode_enabled |= BIT_NUM_TO_MASK(DBGC_HART_HDMER_SW_BRKPT_BIT);
#if 0
    } else {
        dmode_enabled &= ~BIT_NUM_TO_MASK(DBGC_HART_HDMER_SW_BRKPT_BIT);
#endif
    }

    // prepare for execution continue
    reg_cache__chain_invalidate(p_target->reg_cache);
    // enable requested debug mode
    set_DEMODE_ENBL(p_target, dmode_enabled);

    if (ERROR_OK != error_code__get(p_target)) {
        return error_code__get_and_clear(p_target);
    }

    // resume exec
    //@{
    sc_rv32_DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_unit_id_HART_0 : DBGC_unit_id_HART_1, DBGC_functional_group_HART_DBGCMD);

    if (ERROR_OK != error_code__get(p_target)) {
        LOG_WARNING("DAP_CTRL_REG_set error");
        return error_code__get_and_clear(p_target);
    }

    sc_rv32_DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_RESUME) | BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS), NULL);

    if (ERROR_OK != error_code__get(p_target)) {
        return error_code__get_and_clear(p_target);
    }

    //@}

    // Mark "not halted", set state, raise event
    LOG_DEBUG("New debug reason: 0x%08X", DBG_REASON_NOTHALTED);
    p_target->debug_reason = DBG_REASON_NOTHALTED;
    p_target->state = debug_execution ? TARGET_DEBUG_RUNNING : TARGET_RUNNING;
    target_call_event_callbacks(p_target, debug_execution ? TARGET_EVENT_DEBUG_RESUMED : TARGET_EVENT_RESUMED);

    sc_rv32_update_status(p_target);
    return error_code__get_and_clear(p_target);
}
static int reset__set(target* const p_target, bool const active)
{
    assert(p_target);
    uint32_t const get_old_value1 = sc_rv32_core_REGTRANS_read(p_target, DBGC_CORE_REGS_DBG_CTRL);

    if (error_code__get(p_target) == ERROR_OK) {
        static uint32_t const bit_mask = BIT_NUM_TO_MASK(DBGC_CORE_CDCR_HART0_RST_BIT) | BIT_NUM_TO_MASK(DBGC_CORE_CDCR_RST_BIT);
        uint32_t const set_value = (get_old_value1 & ~bit_mask) | (active ? bit_mask : 0u);
        sc_rv32_CORE_REGTRANS_write(p_target, DBGC_CORE_REGS_DBG_CTRL, set_value);

        if (error_code__get(p_target) == ERROR_OK) {
            sc_rv32i__Arch const* const p_arch = p_target->arch_info;
            assert(p_arch);

            if (p_arch->use_verify_core_regtrans_write) {
                uint32_t const get_new_value2 = sc_rv32_core_REGTRANS_read(p_target, DBGC_CORE_REGS_DBG_CTRL);

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
            sc_rv32_update_status(p_target);

            if (error_code__get(p_target) == ERROR_OK) {
                if (active) {
                    if (p_target->state != TARGET_RESET) {
                        /// issue error if we are still running
                        LOG_ERROR("Target is not resetting after reset assert");
                        error_code__update(p_target, ERROR_TARGET_FAILURE);
                    }
                } else {
                    if (p_target->state == TARGET_RESET) {
                        LOG_ERROR("Target is still in reset after reset deassert");
                        error_code__update(p_target, ERROR_TARGET_FAILURE);
                    }
                }
            }
        }
    }

    return error_code__get_and_clear(p_target);
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
static char const def_FP_regs_name[] = "float";
static reg const def_FP_regs_array[] = {
    // FP temporaries
    {.name = "f0",.number = 0 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f1",.number = 1 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f2",.number = 2 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f3",.number = 3 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f4",.number = 4 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f5",.number = 5 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f6",.number = 6 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f7",.number = 7 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},

    // FP saved registers
    {.name = "f8",.number = 8 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f9",.number = 9 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},

    // FP arguments/return values
    {.name = "f10",.number = 10 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f11",.number = 11 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},

    // FP arguments
    {.name = "f12",.number = 12 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f13",.number = 13 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f14",.number = 14 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f15",.number = 15 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f16",.number = 16 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f17",.number = 17 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},

    // FP saved registers
    {.name = "f18",.number = 18 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f19",.number = 19 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f20",.number = 20 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f21",.number = 21 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f22",.number = 22 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f23",.number = 23 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f24",.number = 24 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f25",.number = 25 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f26",.number = 26 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f27",.number = 27 + RISCV_regnum_FP_first,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},

    // FP temporaries
    {.name = "f28",.number = 28 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f29",.number = 29 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f30",.number = 30 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
    {.name = "f31",.number = 31 + RISCV_regnum_FP_first,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org,.reg_data_type = &FP_reg_data_type,.group = def_FP_regs_name},
};
static char const def_CSR_regs_name[] = "system";
static reg const CSR_not_exists = {.name = "",.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_csr_accessors,.feature = &feature_riscv_org,.reg_data_type = &GP_reg_data_type,.group = def_CSR_regs_name};
static char csr_names[4096][50] = {[0 ... 4095] = {'\0'}};
static void init_csr_names(void)
{
    static bool csr_names_inited = false;

    if (!csr_names_inited) {
        for (int i = 0; i < 4096; ++i) {
            sprintf(csr_names[i], "csr%d", i);
        }

        csr_names_inited = true;
    }
}
static reg_cache* reg_cache__CSR_section_create_gdb(char const* name, void* const p_arch_info)
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
            p_reg->value = calloc(1, NUM_BITS_TO_SIZE(p_reg->size));;
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
static void sc_rv32_init_regs_cache(target* const p_target)
{
    assert(p_target);
    reg_cache* p_reg_cache_last = p_target->reg_cache = reg_cache__section_create(def_GP_regs_name, def_GP_regs_array, ARRAY_LEN(def_GP_regs_array), p_target);
    p_reg_cache_last = p_reg_cache_last->next = reg_cache__section_create(def_FP_regs_name, def_FP_regs_array, ARRAY_LEN(def_FP_regs_array), p_target);
#if 0
    p_reg_cache_last->next = reg_cache__CSR_section_create(def_CSR_regs_name, def_CSR_regs_array, ARRAY_LEN(def_CSR_regs_array), p_target);
#else
    p_reg_cache_last->next = reg_cache__CSR_section_create_gdb(def_CSR_regs_name, p_target);
#endif
}
static int sc_rv32i__init_target(command_context* cmd_ctx, target* const p_target)
{
    sc_rv32_init_regs_cache(p_target);

    sc_rv32i__Arch* p_arch_info = calloc(1, sizeof(sc_rv32i__Arch));
    assert(p_arch_info);
    *p_arch_info = sc_rv32_initial_arch;

    p_target->arch_info = p_arch_info;
    return ERROR_OK;
}
static void sc_rv32i__deinit_target(target* const p_target)
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
static int sc_rv32i__target_create(target* const p_target, Jim_Interp* interp)
{
    assert(p_target);
    return ERROR_OK;
}
static int sc_rv32i__examine(target* const p_target)
{
    assert(p_target);

    for (int i = 0; i < 10; ++i) {
        error_code__get_and_clear(p_target);
        LOG_DEBUG("update_status");
        sc_rv32_update_status(p_target);

        if (ERROR_OK == error_code__get(p_target)) {
            break;
        }

        LOG_DEBUG("update_status error, retry");
    }

    if (ERROR_OK == error_code__get(p_target)) {
        uint32_t const IDCODE = sc_rv32_IDCODE_get(p_target);

        if ((IDCODE & EXPECTED_IDCODE_MASK) != (EXPECTED_IDCODE & EXPECTED_IDCODE_MASK)) {
            LOG_ERROR("Invalid IDCODE=0x%08X!", IDCODE);
            error_code__update(p_target, ERROR_TARGET_FAILURE);
        } else {
            uint32_t const DBG_ID = sc_rv32_DBG_ID_get(p_target);

            if ((DBG_ID & DBG_ID_VERSION_MASK) != (DBG_ID_VERSION_MASK & EXPECTED_DBG_ID) ||
                (DBG_ID & DBG_ID_SUBVERSION_MASK) < (EXPECTED_DBG_ID & DBG_ID_SUBVERSION_MASK)) {
                LOG_ERROR("Unsupported DBG_ID=0x%08X!", DBG_ID);
                error_code__update(p_target, ERROR_TARGET_FAILURE);
            } else {
                LOG_INFO("IDCODE=0x%08X DBG_ID=0x%08X BLD_ID=0x%08X", IDCODE, DBG_ID, sc_rv32_BLD_ID_get(p_target));
                set_DEMODE_ENBL(p_target, NORMAL_DEBUG_ENABLE_MASK);

                if (ERROR_OK == error_code__get(p_target)) {
                    LOG_DEBUG("Examined OK");
                    target_set_examined(p_target);
                }
            }
        }
    }

    return error_code__get_and_clear(p_target);
}
static int sc_rv32i__poll(target* const p_target)
{
    assert(p_target);
    sc_rv32_update_status(p_target);
    return error_code__get_and_clear(p_target);
}
static int sc_rv32i__arch_state(target* const p_target)
{
    assert(p_target);
    sc_rv32_update_status(p_target);
    return error_code__get_and_clear(p_target);
}
static int sc_rv32i__halt(target* const p_target)
{
    assert(p_target);
    // May be already halted?
    {
        sc_rv32_update_status(p_target);

        if (ERROR_OK != error_code__get(p_target)) {
            return error_code__get_and_clear(p_target);
        }

        if (p_target->state == TARGET_HALTED) {
            LOG_WARNING("Halt request when target is already in halted state");
            return error_code__get_and_clear(p_target);
        }
    }

    // Try to halt
    {
        sc_rv32_DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_unit_id_HART_0 : DBGC_unit_id_HART_1, DBGC_functional_group_HART_DBGCMD);

        if (ERROR_OK != error_code__get(p_target)) {
            LOG_WARNING("DAP_CTRL_REG_set error");
            return error_code__get_and_clear(p_target);
        }

        sc_rv32_DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_HALT) | BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS), NULL);

        if (ERROR_OK != error_code__get(p_target)) {
            return error_code__get_and_clear(p_target);
        }
    }

    sc_rv32_check_that_target_halted(p_target);
    return error_code__get_and_clear(p_target);
}
static int sc_rv32i__resume(target* const p_target, int const current, uint32_t const address, int const handle_breakpoints, int const debug_execution)
{
    LOG_DEBUG("resume: current=%d address=0x%08x handle_breakpoints=%d debug_execution=%d", current, address, handle_breakpoints, debug_execution);
    assert(p_target);
    static uint32_t const dmode_enabled = NORMAL_DEBUG_ENABLE_MASK;
    return resume_common(p_target, dmode_enabled, current, address, handle_breakpoints, debug_execution);
}
static int sc_rv32i__step(target* const p_target, int const current, uint32_t const address, int const handle_breakpoints)
{
    LOG_DEBUG("step: current=%d address=0x%08x handle_breakpoints=%d", current, address, handle_breakpoints);
    assert(p_target);
    // disable halt on SW breakpoint to pass SW breakpoint processing to core
    static uint32_t const dmode_enabled = (NORMAL_DEBUG_ENABLE_MASK & ~BIT_NUM_TO_MASK(DBGC_HART_HDMER_SW_BRKPT_BIT)) | BIT_NUM_TO_MASK(DBGC_HART_HDMER_SINGLE_STEP_BIT);
    return resume_common(p_target, dmode_enabled, current, address, handle_breakpoints, false);
}
static int sc_rv32i__assert_reset(target* const p_target)
{
    LOG_DEBUG("Reset control");
    assert(p_target);
    return reset__set(p_target, true);
}
static int sc_rv32i__deassert_reset(target* const p_target)
{
    LOG_DEBUG("Reset control");
    assert(p_target);
    return reset__set(p_target, false);
}
static int sc_rv32i__soft_reset_halt(target* const p_target)
{
    LOG_DEBUG("Soft reset called");
    assert(p_target);
    reset__set(p_target, true);

    if (ERROR_OK == error_code__get(p_target)) {
        set_DEMODE_ENBL(p_target, NORMAL_DEBUG_ENABLE_MASK | BIT_NUM_TO_MASK(DBGC_HART_HDMER_RST_EXIT_BRK_BIT));

        if (ERROR_OK == error_code__get(p_target)) {
            reset__set(p_target, false);
        } else {
            sc_rv32_update_status(p_target);
        }
    } else {
        sc_rv32_update_status(p_target);
    }

    return error_code__get_and_clear(p_target);
}
static int sc_rv32i__mmu(target* p_target, int* p_mmu_enabled)
{
    assert(p_target);
    uint32_t const mstatus = csr_get_value(p_target, CSR_mstatus);

    if (ERROR_OK == error_code__get(p_target)) {
        uint32_t const privilege_level = (mstatus >> 1) & LOW_BITS_MASK(2);
        assert(p_mmu_enabled);

        if (privilege_level == Priv_M || privilege_level == Priv_H) {
            *p_mmu_enabled = 0;
        } else {
            uint32_t const VM = (mstatus >> 17) & LOW_BITS_MASK(21 - 16);

            switch (VM) {
            case VM_Mbb:
            case VM_Mbbid:
            case VM_Sv32:
            case VM_Sv39:
            case VM_Sv48:
                *p_mmu_enabled = 1;
                break;

            case VM_Mbare:
            default:
                *p_mmu_enabled = 0;
                break;
            }
        }
    } else {
        sc_rv32_update_status(p_target);
    }

    return error_code__get_and_clear(p_target);
}
static void virt_to_phis(target* p_target, uint32_t address, uint32_t* p_physical, uint32_t* p_bound, bool const instruction_space)
{
    assert(p_target);
    uint32_t const mstatus = csr_get_value(p_target, CSR_mstatus);

    if (ERROR_OK == error_code__get(p_target)) {
        uint32_t const PRV = (mstatus >> 1) & LOW_BITS_MASK(2);
        uint32_t const VM = PRV == Priv_M || PRV == Priv_H ? VM_Mbare : (mstatus >> 17) & LOW_BITS_MASK(21 - 16);
        assert(p_physical);

        switch (VM) {
        case VM_Mbare:
            *p_physical = address;

            if (p_bound) {
                *p_bound = UINT32_MAX;
            }

            break;

        case VM_Mbb:
        case VM_Mbbid:
            {
                uint32_t const bound = csr_get_value(p_target, VM == VM_Mbb ? CSR_mbound : /*VM == VM_Mbbid*/instruction_space ? CSR_mibound : CSR_mdbound);

                if (ERROR_OK == error_code__get(p_target)) {
                    if (!(address < bound)) {
                        error_code__update(p_target, ERROR_TARGET_TRANSLATION_FAULT);
                    } else {
                        uint32_t const base = csr_get_value(p_target, VM_Mbb ? CSR_mbase : /*VM == VM_Mbbid*/instruction_space ? CSR_mibase : CSR_mdbase);

                        if (ERROR_OK == error_code__get(p_target)) {
                            *p_physical = address + base;

                            if (p_bound) {
                                *p_bound = bound - address;
                            }
                        } else {
                            sc_rv32_update_status(p_target);
                        }
                    }
                } else {
                    sc_rv32_update_status(p_target);
                }
            }
            break;

        case VM_Sv32:
            {
                static uint32_t const offset_mask = LOW_BITS_MASK(10) << 2;
                uint32_t const main_page = csr_get_value(p_target, CSR_sptbr);

                if (ERROR_OK == error_code__get(p_target)) {
                    // lower bits should be zero
                    assert(0 == (main_page & LOW_BITS_MASK(12)));
                    uint32_t const offset_bits1 = address >> 20 & offset_mask;
                    uint8_t pte1_buf[4];

                    if (ERROR_OK == error_code__update(p_target, target_read_phys_memory(p_target, main_page | offset_bits1, 4, 1, pte1_buf))) {
                        uint32_t const pte1 = buf_get_u32(pte1_buf, 0, 32);

                        if (0 == (pte1 & BIT_NUM_TO_MASK(0))) {
                            error_code__update(p_target, ERROR_TARGET_TRANSLATION_FAULT);
                        } else if ((pte1 >> 1 & LOW_BITS_MASK(4)) >= 2) {
                            *p_physical = (pte1 << 2 & ~LOW_BITS_MASK(22)) | (address & LOW_BITS_MASK(22));

                            if (p_bound) {
                                *p_bound = BIT_NUM_TO_MASK(22) - (address & LOW_BITS_MASK(22));
                            }
                        } else {
                            uint32_t const base_0 = pte1 << 2 & ~LOW_BITS_MASK(12);
                            uint32_t const offset_bits0 = address >> 10 & offset_mask;
                            uint8_t pte0_buf[4];

                            if (ERROR_OK == error_code__update(p_target, target_read_phys_memory(p_target, base_0 | offset_bits0, 4, 1, pte0_buf))) {
                                uint32_t const pte0 = buf_get_u32(pte0_buf, 0, 32);

                                if (0 == (pte0 & BIT_NUM_TO_MASK(0)) || (pte0 >> 1 & LOW_BITS_MASK(4)) < 2) {
                                    error_code__update(p_target, ERROR_TARGET_TRANSLATION_FAULT);
                                } else {
                                    *p_physical = (pte0 << 2 & ~LOW_BITS_MASK(12)) | (address & LOW_BITS_MASK(12));

                                    if (p_bound) {
                                        *p_bound = BIT_NUM_TO_MASK(12) - (address & LOW_BITS_MASK(12));
                                    }
                                }
                            } else {
                                sc_rv32_update_status(p_target);
                            }
                        }
                    } else {
                        sc_rv32_update_status(p_target);
                    }
                } else {
                    sc_rv32_update_status(p_target);
                }
            }
            break;

        case VM_Sv39:
        case VM_Sv48:
        default:
            error_code__update(p_target, ERROR_TARGET_TRANSLATION_FAULT);
            break;
        }
    } else {
        sc_rv32_update_status(p_target);
    }
}
static int sc_rv32i__virt2phys(target* p_target, uint32_t address, uint32_t* p_physical)
{
    virt_to_phis(p_target, address, p_physical, NULL, false);
    return error_code__get_and_clear(p_target);
}
static void read_memory_space(target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t* p_buffer, bool const instruction_space)
{
    assert(p_target);

    if (!(size == 1 || size == 2 || size == 4)) {
        LOG_ERROR("Invalid item size %d", size);
        error_code__update(p_target, ERROR_TARGET_FAILURE);
    } else if (address % size != 0) {
        LOG_ERROR("Unaligned access at 0x%08X, for item size %d", address, size);
        error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
    } else {
        while (0 != count) {
            uint32_t physical;
            uint32_t bound;
            virt_to_phis(p_target, address, &physical, &bound, instruction_space);

            if (ERROR_OK != error_code__get(p_target)) {
                break;
            }

            uint32_t const page_count = size * count > bound ? bound / size : count;
            assert(0 != page_count);
            assert(p_buffer);

            if (ERROR_OK != error_code__update(p_target, target_read_phys_memory(p_target, physical, size, page_count, p_buffer))) {
                break;
            }

            uint32_t const bytes = size * page_count;
            p_buffer += bytes;
            address += bytes;
            count -= page_count;
        }
    }
}
static void write_memory_space(target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t const* p_buffer, bool const instruction_space)
{
    assert(p_target);

    if (!(size == 1 || size == 2 || size == 4)) {
        LOG_ERROR("Invalid item size %d", size);
        error_code__update(p_target, ERROR_TARGET_FAILURE);
    } else if (address % size != 0) {
        LOG_ERROR("Unaligned access at 0x%08X, for item size %d", address, size);
        error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
    } else {
        while (0 != count) {
            uint32_t physical;
            uint32_t bound;
            virt_to_phis(p_target, address, &physical, &bound, instruction_space);

            if (ERROR_OK != error_code__get(p_target)) {
                break;
            }

            uint32_t const page_count = size * count > bound ? bound / size : count;
            assert(0 != page_count);
            assert(p_buffer);

            if (ERROR_OK != error_code__update(p_target, target_write_phys_memory(p_target, physical, size, page_count, p_buffer))) {
                break;
            }

            uint32_t const bytes = size * page_count;
            p_buffer += bytes;
            address += bytes;
            count -= page_count;
        }
    }
}
static int sc_rv32i__read_memory(target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t* buffer)
{
    read_memory_space(p_target, address, size, count, buffer, false);
    return error_code__get_and_clear(p_target);
}
static int sc_rv32i__write_memory(target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t const* buffer)
{
    write_memory_space(p_target, address, size, count, buffer, false);
    return error_code__get_and_clear(p_target);
}
static int sc_rv32i__read_phys_memory(target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t* buffer)
{
    assert(p_target);
    assert(buffer);
    LOG_DEBUG("Read_memory at 0x%08X, %d items, each %d bytes, total %d bytes", address, count, size, count * size);

    /// Check for size
    if (!(size == 1 || size == 2 || size == 4)) {
        LOG_ERROR("Invalid item size %d", size);
        error_code__update(p_target, ERROR_TARGET_FAILURE);
        return error_code__get_and_clear(p_target);
    } else if (address % size != 0) {
        LOG_ERROR("Unaligned access at 0x%08X, for item size %d", address, size);
        error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
        return error_code__get_and_clear(p_target);
    } else {
        /// Check that target halted
        sc_rv32_check_that_target_halted(p_target);

        if (ERROR_OK != error_code__get(p_target)) {
            return error_code__get_and_clear(p_target);
        }

        /// Reserve work register
        reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);
        assert(p_wrk_reg);

        uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);

        if (ERROR_OK == error_code__get(p_target)) {
            /// Define opcode for load item to register
            uint32_t const load_OP =
                size == 4 ? RISCV_opcode_LW(p_wrk_reg->number, p_wrk_reg->number, 0) :
                size == 2 ? RISCV_opcode_LH(p_wrk_reg->number, p_wrk_reg->number, 0) :
                /*size == 1*/RISCV_opcode_LB(p_wrk_reg->number, p_wrk_reg->number, 0);

            sc_rv32i__Arch const* const p_arch = p_target->arch_info;
            assert(p_arch);
            size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);

            if (p_arch->use_pc_advmt_dsbl_bit) {
                sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
            }

            /// Setup exec operations mode
            sc_rv32_EXEC__setup(p_target);
            int advance_pc_counter = 0;

            if (ERROR_OK == error_code__get(p_target)) {
                /// For count number of items do loop
                while (count--) {
                    /// Set address to CSR
                    sc_rv32_EXEC__push_data_to_CSR(p_target, address);

                    if (ERROR_OK != error_code__get(p_target)) {
                        break;
                    }

                    /// Load address to work register
                    (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_wrk_reg->number, CSR_sc_dbg_scratch));
                    advance_pc_counter += instr_step;

                    if (ERROR_OK != error_code__get(p_target)) {
                        break;
                    }

                    /// Exec load item to register
                    (void)sc_rv32_EXEC__step(p_target, load_OP);
                    advance_pc_counter += instr_step;

                    if (ERROR_OK != error_code__get(p_target)) {
                        break;
                    }

                    /// Exec store work register to csr
                    (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRW(CSR_sc_dbg_scratch, p_wrk_reg->number));
                    advance_pc_counter += instr_step;

                    /// get data from csr and jump back to correct pc
                    assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
                    uint32_t const value = sc_rv32_EXEC__step(p_target, RISCV_opcode_JAL(0, -advance_pc_counter));
                    advance_pc_counter = 0;

                    if (ERROR_OK != error_code__get(p_target)) {
                        break;
                    }

                    /// store read data to buffer
                    buf_set_u32(buffer, 0, CHAR_BIT * size, value);

                    /// advance src/dst pointers
                    address += size;
                    buffer += size;
                }
            }

            if (p_arch->use_pc_advmt_dsbl_bit) {
                sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, 0);
            }
        } else {
            sc_rv32_update_status(p_target);
        }

        if (ERROR_OK != error_code__get(p_target)) {
            sc_rv32_update_status(p_target);
        }

        sc_rv32_check_PC_value(p_target, pc_sample_1);

        /// restore temporary register
        int const old_err_code = error_code__get_and_clear(p_target);
        error_code__update(p_target, reg_x__store(p_wrk_reg));
        error_code__prepend(p_target, old_err_code);
        assert(!p_wrk_reg->dirty);

        return error_code__get_and_clear(p_target);
    }
}
static int sc_rv32i__write_phys_memory(target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t const* buffer)
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

    sc_rv32_check_that_target_halted(p_target);

    if (ERROR_OK != error_code__get(p_target)) {
        return error_code__get_and_clear(p_target);
    }

    uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);
    /// Reserve work register
    reg* const p_addr_reg = prepare_temporary_GP_register(p_target, 0);
    assert(p_addr_reg);
    reg* const p_data_reg = prepare_temporary_GP_register(p_target, p_addr_reg->number);
    assert(p_data_reg);
    assert(p_addr_reg->number != p_data_reg->number);

    if (ERROR_OK == error_code__get(p_target)) {
        sc_rv32i__Arch const* const p_arch = p_target->arch_info;
        assert(p_arch);
        size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);

        if (p_arch->use_pc_advmt_dsbl_bit) {
            sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
        }

        /// Setup exec operations mode
        sc_rv32_EXEC__setup(p_target);
        size_t advance_pc_counter = 0;

        if (ERROR_OK == error_code__get(p_target)) {
            // Set address to CSR
            sc_rv32_EXEC__push_data_to_CSR(p_target, address);

            if (error_code__get(p_target) == ERROR_OK) {
                /// Load address to work register
                (void)sc_rv32_EXEC__step(p_target, RISCV_opcode_CSRR(p_addr_reg->number, CSR_sc_dbg_scratch));
                advance_pc_counter += instr_step;

                // Opcodes
                uint32_t const instructions[3] = {
                    RISCV_opcode_CSRR(p_data_reg->number, CSR_sc_dbg_scratch),
                    (size == 4 ? RISCV_opcode_SW(p_data_reg->number, p_addr_reg->number, 0) :
                     size == 2 ? RISCV_opcode_SH(p_data_reg->number, p_addr_reg->number, 0) :
                     /*size == 1*/ RISCV_opcode_SB(p_data_reg->number, p_addr_reg->number, 0)),
                    RISCV_opcode_ADDI(p_addr_reg->number, p_addr_reg->number, size)
                };

                static uint32_t max_pc_offset = (((1u << 20) - 1u) / NUM_BITS_TO_SIZE(XLEN)) * NUM_BITS_TO_SIZE(XLEN);

                if (p_arch->use_queuing_for_dr_scans) {
                    uint8_t DAP_OPSTATUS = 0;
                    uint8_t const data_wr_opcode[1] = {DBGC_DAP_OPCODE_DBGCMD_DBGDATA_WR};
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
                    uint8_t const instr_exec_opcode[1] = {DBGC_DAP_OPCODE_DBGCMD_CORE_EXEC};
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

                    while (error_code__get(p_target) == ERROR_OK && count--) {
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

                    if (!p_arch->use_pc_advmt_dsbl_bit) {
                        assert(advance_pc_counter % NUM_BITS_TO_SIZE(XLEN) == 0);

                        while (advance_pc_counter) {
                            uint32_t const step_back = advance_pc_counter > max_pc_offset ? max_pc_offset : advance_pc_counter;
                            advance_pc_counter -= step_back;
                            assert(advance_pc_counter % NUM_BITS_TO_SIZE(XLEN) == 0);
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

                    error_code__update(p_target, jtag_execute_queue());

                    if ((DAP_OPSTATUS & DAP_status_MASK) != DAP_status_good) {
                        LOG_ERROR("DAP_OPSTATUS == 0x%1X", (uint32_t)DAP_OPSTATUS);
                        error_code__update(p_target, ERROR_TARGET_FAILURE);
                    }
                } else {
                    while (ERROR_OK == error_code__get(p_target) && count--) {
                        /// Set data to CSR
                        sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(buffer, 0, CHAR_BIT * size));

                        if (ERROR_OK != error_code__get(p_target)) {
                            break;
                        }

                        for (unsigned i = 0; i < ARRAY_LEN(instructions); ++i) {
                            (void)sc_rv32_EXEC__step(p_target, instructions[i]);

                            if (ERROR_OK != error_code__get(p_target)) {
                                break;
                            }

                            advance_pc_counter += instr_step;
                        }

                        buffer += size;
                    }

                    if (!p_arch->use_pc_advmt_dsbl_bit) {
                        assert(advance_pc_counter % NUM_BITS_TO_SIZE(XLEN) == 0);

                        while ((ERROR_OK == error_code__get(p_target)) && (advance_pc_counter != 0)) {
                            uint32_t const step_back = advance_pc_counter > max_pc_offset ? max_pc_offset : advance_pc_counter;
                            advance_pc_counter -= step_back;
                            assert(advance_pc_counter % NUM_BITS_TO_SIZE(XLEN) == 0);
                            uint32_t const OP_correct_pc = RISCV_opcode_JAL(0, -(int)(step_back));
                            (void)sc_rv32_EXEC__step(p_target, OP_correct_pc);
                        }
                    }
                }
            }
        }

        if (p_arch->use_pc_advmt_dsbl_bit) {
            sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_register_DBG_CTRL, 0);
        }
    } else {
        sc_rv32_update_status(p_target);
    }

    if (ERROR_OK != error_code__get(p_target)) {
        sc_rv32_update_status(p_target);
    }

    sc_rv32_check_PC_value(p_target, pc_sample_1);

    /// restore temporary registers
    int const old_err_code = error_code__get_and_clear(p_target);
    int const new_err_code_1 = reg_x__store(p_data_reg);
    assert(!p_data_reg->dirty);
    int const new_err_code_2 = reg_x__store(p_addr_reg);
    assert(!p_addr_reg->dirty);
    error_code__update(p_target, old_err_code);
    error_code__update(p_target, new_err_code_1);
    error_code__update(p_target, new_err_code_2);

    return error_code__get_and_clear(p_target);
}
static int sc_rv32i__add_breakpoint(target* const p_target, breakpoint* const p_breakpoint)
{
    assert(p_target);
    assert(p_breakpoint);

    if (p_breakpoint->type != BKPT_SOFT) {
        LOG_ERROR("Only software breakpoins available");
        return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
    }

    sc_rv32_check_that_target_halted(p_target);

    if (ERROR_OK == error_code__get(p_target)) {
        bool const RVC_enable = is_RVC_enable(p_target);

        if (!(p_breakpoint->length == NUM_BITS_TO_SIZE(ILEN) || (RVC_enable && p_breakpoint->length == 2))) {
            LOG_ERROR("Invalid breakpoint size: %d", p_breakpoint->length);
            error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
        } else if (p_breakpoint->address % (RVC_enable ? 2 : 4) != 0) {
            LOG_ERROR("Unaligned breakpoint: 0x%08X", p_breakpoint->address);
            error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
        } else {
            read_memory_space(p_target, p_breakpoint->address, 2, p_breakpoint->length / 2, p_breakpoint->orig_instr, true);

            if (ERROR_OK != error_code__get(p_target)) {
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

                if (ERROR_OK != error_code__get(p_target)) {
                    LOG_ERROR("Can't write EBREAK");
                } else {
                    p_breakpoint->set = 1;
                }
            }
        }
    }

    return error_code__get_and_clear(p_target);
}
static int sc_rv32i__remove_breakpoint(target* const p_target, breakpoint* const p_breakpoint)
{
    assert(p_target);
    assert(p_breakpoint);
    sc_rv32_check_that_target_halted(p_target);

    if (ERROR_OK == error_code__get(p_target)) {
        assert(p_breakpoint->orig_instr);
#if 0
        LOG_INFO("Remove breakpoint at 0x%08x, length=%d (0x%08x)",
                 p_breakpoint->address,
                 p_breakpoint->length,
                 buf_get_u32(p_breakpoint->orig_instr, 0, p_breakpoint->length * CHAR_BIT));
#endif
        write_memory_space(p_target, p_breakpoint->address, 2, p_breakpoint->length / 2, p_breakpoint->orig_instr, true);

        if (ERROR_OK == error_code__get(p_target)) {
            p_breakpoint->set = 0;
        } else {
            sc_rv32_update_status(p_target);
        }
    } else {
        sc_rv32_update_status(p_target);
    }

    return error_code__get_and_clear(p_target);
}
/// gdb_server expects valid reg values and will use set method for updating reg values
static int sc_rv32i__get_gdb_reg_list(target* const p_target, reg** reg_list[], int* const reg_list_size, target_register_class const reg_class)
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
    return error_code__get_and_clear(p_target);
}
/// @todo make const
target_type syntacore_riscv32i_target = {
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

    .virt2phys = sc_rv32i__virt2phys,
    .read_phys_memory = sc_rv32i__read_phys_memory,
    .write_phys_memory = sc_rv32i__write_phys_memory,

    .mmu = sc_rv32i__mmu,
    .check_reset = NULL,
    .get_gdb_fileio_info = NULL,
    .gdb_fileio_end = NULL,
    .profiling = NULL,
};
