/** @file

	Syntacore RISC-V target

	@copyright Syntacore 2016, 2017
	@author sps (https://github.com/aka-sps)
*/
#include "sc_rv32_common.h"

#include "target/target_type.h"
#include "helper/log.h"
#include "helper/binarybuffer.h"

#include <assert.h>

/// RISC-V Privileged ISA 1.7 CSR
enum
{
	CSR_sptbr_Pr_ISA_1_7 = 0x180,

	/// Machine Protection and Translation
	/// privilege: MRW
	///@{

	/// @brief Base register
	CSR_mbase_Pr_ISA_1_7 = 0x380u,

	/// @brief Base register
	CSR_mbound_Pr_ISA_1_7 = 0x381u,

	/// @brief Bound register.
	CSR_mibase_Pr_ISA_1_7 = 0x382u,

	/// @brief Instruction base register.
	CSR_mibound_Pr_ISA_1_7 = 0x383u,

	/// @brief Data base register
	CSR_mdbase_Pr_ISA_1_7 = 0x384u,

	/// @brief Data bound register
	CSR_mdbound_Pr_ISA_1_7 = 0x385u,
	///@}
};

/// RISC-V Privileged ISA 1.7 levels
enum
{
	Priv_U = 0x0,
	Priv_S = 0x1,
	Priv_H = 0x2,
	Priv_M = 0x3,
};

/// RISC-V Privileged ISA 1.7 VM modes
enum
{
	VM_Mbare = 0,
	VM_Mbb = 1,
	VM_Mbbid = 2,
	VM_Sv32 = 8,
	VM_Sv39 = 9,
	VM_Sv48 = 10,
};

static error_code
scrx_1_7__virt_to_phis(target* p_target, uint32_t address, uint32_t* p_physical, uint32_t* p_bound, bool const instruction_space)
{
	uint32_t const mstatus = sc_riscv32__csr_get_value(p_target, CSR_mstatus);

	if (ERROR_OK != sc_error_code__get(p_target)) {
		return sc_riscv32__update_status(p_target);
	} else {
		/// @todo Privileged Instruction 1.7 version
		uint32_t const PRV = (mstatus >> 1) & LOW_BITS_MASK(2);
		/// @todo Privileged Instruction 1.7 version
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
				uint32_t const bound = sc_riscv32__csr_get_value(p_target, VM == VM_Mbb ? CSR_mbound_Pr_ISA_1_7 : /*VM == VM_Mbbid*/instruction_space ? CSR_mibound_Pr_ISA_1_7 : CSR_mdbound_Pr_ISA_1_7);

				if (ERROR_OK == sc_error_code__get(p_target)) {
					if (!(address < bound)) {
						sc_error_code__update(p_target, ERROR_TARGET_TRANSLATION_FAULT);
					} else {
						uint32_t const base = sc_riscv32__csr_get_value(p_target, VM_Mbb ? CSR_mbase_Pr_ISA_1_7 : /*VM == VM_Mbbid*/instruction_space ? CSR_mibase_Pr_ISA_1_7 : CSR_mdbase_Pr_ISA_1_7);

						if (ERROR_OK == sc_error_code__get(p_target)) {
							*p_physical = address + base;

							if (p_bound) {
								*p_bound = bound - address;
							}
						} else {
							sc_riscv32__update_status(p_target);
							if (!p_target->examined) {
								return sc_error_code__get(p_target);
							}
						}
					}
				} else {
					sc_riscv32__update_status(p_target);
					if (!p_target->examined) {
						return sc_error_code__get(p_target);
					}
				}
			}
			break;

		case VM_Sv32:
			{
				static uint32_t const offset_mask = LOW_BITS_MASK(10) << 2;
				uint32_t const main_page = sc_riscv32__csr_get_value(p_target, CSR_sptbr_Pr_ISA_1_7);

				if (ERROR_OK == sc_error_code__get(p_target)) {
					// lower bits should be zero
					assert(0 == (main_page & LOW_BITS_MASK(12)));
					uint32_t const offset_bits1 = address >> 20 & offset_mask;
					uint8_t pte1_buf[4];

					if (ERROR_OK == sc_error_code__update(p_target, target_read_phys_memory(p_target, main_page | offset_bits1, 4, 1, pte1_buf))) {
						uint32_t const pte1 = buf_get_u32(pte1_buf, 0, 32);

						if (0 == (pte1 & BIT_MASK(0))) {
							sc_error_code__update(p_target, ERROR_TARGET_TRANSLATION_FAULT);
						} else if ((pte1 >> 1 & LOW_BITS_MASK(4)) >= 2) {
							*p_physical = (pte1 << 2 & ~LOW_BITS_MASK(22)) | (address & LOW_BITS_MASK(22));

							if (p_bound) {
								*p_bound = BIT_MASK(22) - (address & LOW_BITS_MASK(22));
							}
						} else {
							uint32_t const base_0 = pte1 << 2 & ~LOW_BITS_MASK(12);
							uint32_t const offset_bits0 = address >> 10 & offset_mask;
							uint8_t pte0_buf[4];

							if (ERROR_OK == sc_error_code__update(p_target, target_read_phys_memory(p_target, base_0 | offset_bits0, 4, 1, pte0_buf))) {
								uint32_t const pte0 = buf_get_u32(pte0_buf, 0, 32);

								if (0 == (pte0 & BIT_MASK(0)) || (pte0 >> 1 & LOW_BITS_MASK(4)) < 2) {
									sc_error_code__update(p_target, ERROR_TARGET_TRANSLATION_FAULT);
								} else {
									*p_physical = (pte0 << 2 & ~LOW_BITS_MASK(12)) | (address & LOW_BITS_MASK(12));

									if (p_bound) {
										*p_bound = BIT_MASK(12) - (address & LOW_BITS_MASK(12));
									}
								}
							} else {
								sc_riscv32__update_status(p_target);
								if (!p_target->examined) {
									return sc_error_code__get(p_target);
								}
							}
						}
					} else {
						sc_riscv32__update_status(p_target);
						if (!p_target->examined) {
							return sc_error_code__get(p_target);
						}
					}
				} else {
					sc_riscv32__update_status(p_target);
					if (!p_target->examined) {
						return sc_error_code__get(p_target);
					}
				}
			}
			break;

		case VM_Sv39:
		case VM_Sv48:
		default:
			return sc_error_code__update(p_target, ERROR_TARGET_TRANSLATION_FAULT);
			break;
		}
	}
	return sc_error_code__get(p_target);
}

static sc_riscv32__Arch_constants const scrx_constants = {
	.use_ir_select_cache = false,
	.use_dap_control_cache = false,
	.use_verify_dap_control = true,
	.use_verify_hart_regtrans_write = true,
	.use_verify_core_regtrans_write = true,
	.use_queuing_for_dr_scans = true,
	.expected_dbg_id = 0x00800001u,
	.debug_scratch_CSR = 0x788u,
	.mstatus_FS_offset = 12u,
	.opcode_FMV_D_2X = &sc_RISCV_opcode_S_FMV_D_2X,
	.opcode_FMV_2X_D = &sc_RISCV_opcode_S_FMV_2X_D,
	.virt_to_phis = &scrx_1_7__virt_to_phis
};

static sc_riscv32__Arch const scrx_initial_arch = {
	.error_code = ERROR_OK,
	.last_DAP_ctrl = DAP_CTRL_value_INVALID_CODE,
	.constants = &scrx_constants
};

static error_code
scrx__init_target(command_context* cmd_ctx, target* const p_target)
{
	sc_riscv32__init_regs_cache(p_target);

	sc_riscv32__Arch* p_arch_info = calloc(1, sizeof(sc_riscv32__Arch));
	assert(p_arch_info);
	*p_arch_info = scrx_initial_arch;

	p_target->arch_info = p_arch_info;
	return ERROR_OK;
}

static error_code
scrx_1_7__mmu(target* p_target, int* p_mmu_enabled)
{
	uint32_t const mstatus = sc_riscv32__csr_get_value(p_target, CSR_mstatus);

	if (ERROR_OK == sc_error_code__get(p_target)) {
		/// @todo Privileged Instruction 1.7 version
		uint32_t const privilege_level = (mstatus >> 1) & LOW_BITS_MASK(2);
		assert(p_mmu_enabled);

		/// @todo Privileged Instruction 1.7 version
		if (privilege_level == Priv_M || privilege_level == Priv_H) {
			*p_mmu_enabled = 0;
		} else {
			/// @todo Privileged Instruction 1.7 version
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
		sc_riscv32__update_status(p_target);
	}

	return sc_error_code__get_and_clear(p_target);
}

/// @todo make const
target_type syntacore_riscv32i_target = {
	.name = "syntacore_riscv32i",

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

	.init_target = scrx__init_target,
	.deinit_target = sc_riscv32__deinit_target,

	.virt2phys = sc_riscv32__virt2phys,
	.read_phys_memory = sc_riscv32__read_phys_memory,
	.write_phys_memory = sc_riscv32__write_phys_memory,

	.mmu = scrx_1_7__mmu,
	.check_reset = NULL,
	.get_gdb_fileio_info = NULL,
	.gdb_fileio_end = NULL,
	.profiling = NULL,
};
