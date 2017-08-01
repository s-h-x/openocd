#ifndef TARGET_SC_RV32_COMMON_H_
#define TARGET_SC_RV32_COMMON_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target/target.h"
#include "target/register.h"
#include "target/breakpoints.h"
#include "jimtcl/jim.h"

#include <stdbool.h>

/// Bit operation macros
/// @{

/// Simple mask with only single bit 'bit_num' is set
#define BIT_MASK(bit_num) (1u << (bit_num))

/// Bit mask value with low 'n' bits set
#define LOW_BITS_MASK(n) (~(~0 << (n)))

/// @}

typedef struct target target;
typedef struct reg reg;
typedef struct breakpoint breakpoint;
typedef struct command_context command_context;
typedef struct target_type target_type;
typedef uint8_t reg_num_type;
typedef uint16_t csr_num_type;
typedef enum target_register_class target_register_class;
typedef int error_code;
typedef uint32_t rv_instruction32_type;

enum {
	/// Marker of invalid value of Debug Access Port multiplexer control
	DAP_CTRL_value_INVALID_CODE = 0xFFu,
};

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
