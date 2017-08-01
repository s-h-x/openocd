/** @file

	Syntacore scr1 RISC-V target

	@copyright Syntacore
	@author sps (https://github.com/aka-sps)

	@defgroup SC_RV32 Syntacore RISC-V target
*/
#include "sc_rv32_common.h"

#include "target/target_type.h"
#include "helper/log.h"

/// RISC-V Privileged ISA 1.9 CSR
enum
{
	CSR_misa = 0x301u,
};

static void
scr1__virt_to_phis(target* p_target, uint32_t address, uint32_t* p_physical, uint32_t* p_bound, bool const instruction_space)
{
	assert(p_physical);
	*p_physical = address;

	if (p_bound) {
		*p_bound = UINT32_MAX;
	}
}

/// SC custom instruction copy FPU double precision register value to two 32-bits GP registers (based on D-extention opcode)
static rv_instruction32_type
RISCV_opcode_FMV_2X_D(reg_num_type rd_hi, reg_num_type rd_lo, reg_num_type rs1_fp)
{
	return RISCV_opcode_INSTR_R_TYPE(0x71u, rd_hi, rs1_fp, 0u, rd_lo, 0x53u);
}

/// SC custom instruction to combine from two GP registers values to FPU double precision register value (based on D-extention opcode)
static rv_instruction32_type
RISCV_opcode_FMV_D_2X(reg_num_type rd_fp, reg_num_type rs_hi, reg_num_type rs_lo)
{
	return RISCV_opcode_INSTR_R_TYPE(0x79u, rs_hi, rs_lo, 0u, rd_fp, 0x53u);
}

static uint32_t
get_isa_CSR_1_10(target* const p_target)
{
	return sc_rv32__csr_get_value(p_target, CSR_misa);
}

/// @todo Privileged Instruction 1.10 version
static uint32_t
scr1__get_mstatus_FS(uint32_t const mstatus)
{
	/// @todo replace 13 by 1.10 FS bit offset
	return (mstatus >> 13) & 3u;
}

static sc_riscv32__Arch_constants scr1_constants = {
	.use_ir_select_cache = false,
	.use_dap_control_cache = false,
	.use_verify_dap_control = true,
	.use_check_pc_unchanged = true,
	.use_verify_hart_regtrans_write = true,
	.use_verify_core_regtrans_write = true,
	.use_pc_advmt_dsbl_bit = true,
	.use_queuing_for_dr_scans = true,
	.expected_idcode = 0xC0D1DEB1u,
	.expected_idcode_mask = 0xFFFFFFFFu,
	.expected_dbg_id = 0x00810000u,
	.debug_scratch_CSR = 0x7C8u,
	.opcode_FMV_D_2X = &RISCV_opcode_FMV_D_2X,
	.opcode_FMV_2X_D = &RISCV_opcode_FMV_2X_D,
	.get_mstatus_FS = &scr1__get_mstatus_FS,
	.get_isa_CSR = &get_isa_CSR_1_10,
	.virt_to_phis = &scr1__virt_to_phis
};

static sc_riscv32__Arch const scr1_initial_arch = {
	.error_code = ERROR_OK,
	.last_DAP_ctrl = DAP_CTRL_value_INVALID_CODE,
	.constants = &scr1_constants
};

static error_code
scr1__init_target(command_context* cmd_ctx, target* const p_target)
{
	sc_rv32_init_regs_cache(p_target);

	sc_riscv32__Arch* p_arch_info = calloc(1, sizeof(sc_riscv32__Arch));
	assert(p_arch_info);
	*p_arch_info = scr1_initial_arch;

	p_target->arch_info = p_arch_info;
	return ERROR_OK;
}

static error_code
scr1__mmu(target* p_target, int* p_mmu_enabled)
{
	*p_mmu_enabled = 0;

	return ERROR_OK;
}

/// @todo make const
target_type scr1_target = {
	.name = "scr1",

	.poll = sc_riscv32__poll,
	.arch_state = sc_riscv32__arch_state,
	.target_request_data = NULL,

	.halt = sc_riscv32__halt,
	.resume = sc_riscv32__resume,
	.step = sc_riscv32__step,

	.assert_reset = sc_riscv32__assert_reset,
	.deassert_reset = sc_riscv32__deassert_reset,
	.soft_reset_halt = sc_riscv32__soft_reset_halt,

	.get_gdb_reg_list = sc_riscv32__get_gdb_reg_list,

	.read_memory = sc_riscv32__read_memory,
	.write_memory = sc_riscv32__write_memory,

	.read_buffer = NULL,
	.write_buffer = NULL,

	.checksum_memory = NULL,
	.blank_check_memory = NULL,

	.add_breakpoint = sc_riscv32__add_breakpoint,
	.add_context_breakpoint = NULL,
	.add_hybrid_breakpoint = NULL,

	.remove_breakpoint = sc_riscv32__remove_breakpoint,

	.add_watchpoint = NULL,
	.remove_watchpoint = NULL,

	.hit_watchpoint = NULL,

	.run_algorithm = NULL,
	.start_algorithm = NULL,
	.wait_algorithm = NULL,

	.commands = NULL,

	.target_create = sc_riscv32__target_create,
	.target_jim_configure = NULL,
	.target_jim_commands = NULL,

	.examine = sc_riscv32__examine,

	.init_target = scr1__init_target,
	.deinit_target = sc_riscv32__deinit_target,

	.virt2phys = sc_riscv32__virt2phys,
	.read_phys_memory = sc_riscv32__read_phys_memory,
	.write_phys_memory = sc_riscv32__write_phys_memory,

	.mmu = scr1__mmu,
	.check_reset = NULL,
	.get_gdb_fileio_info = NULL,
	.gdb_fileio_end = NULL,
	.profiling = NULL,
};
