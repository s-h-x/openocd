/** @file

Syntacore RISC-V target

@copyright Syntacore
*/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "sc_rv32i__Arch.h"
#include "Syntacore_RV_DC.h"
#include "riscv.h"
#include "target/target.h"
#include "target/target_type.h"
#include "target/breakpoints.h"
#include "target/register.h"
#include "helper/log.h"
#include "helper/binarybuffer.h"
#include "helper/types.h"
#include "jtag/jtag.h"
#include "static_assert.h"

#include <stdbool.h>
#include <limits.h>
#include <memory.h>
#include <limits.h>

/// TAP IDCODE expected
#define FP_enabled !!1
#define VERIFY_REG_WRITE 1
#define WRITE_BUFFER_THRESHOLD (1u << 18)
/// Utility macros
///@{

#define LOW_BITS_MASK(n) (~(~0 << (n)))
#define MAKE_TYPE_FIELD(TYPE, bits, first_bit, last_bit)     ((((TYPE)(bits)) & LOW_BITS_MASK((last_bit) + 1u - (first_bit))) << (first_bit))

#define ARRAY_LEN(arr) (sizeof (arr) / sizeof (arr)[0])

#define NUM_BITS_TO_SIZE(num_bits) ( ( (size_t)(num_bits) + (8 - 1) ) / 8 )
///@}

/// RISC-V GP registers id
enum
{
	REG_PC_NUMBER = 0xFFFFFFFFu,
	NUMBER_OF_X_REGS = 32u,
	NUMBER_OF_GP_REGS = NUMBER_OF_X_REGS + 1u,
	NUMBER_OF_F_REGS = 32u,
	NUMBER_OF_CSR_REGS = 1u,
};

enum arch_bits_numbers
{
	/// Size of RISC-V GP registers in bits
	XLEN = 32u,
	/// Size of RISC-V FP registers in bits
	FLEN = 32u,
	/// Size of RISC-V instruction
	ILEN = 32u,
};

static uint8_t const DAP_OPSTATUS_GOOD = DAP_OPSTATUS_OK;
static uint8_t const DAP_STATUS_MASK = DAP_OPSTATUS_MASK;

/// GP registers accessors
///@{
static inline void reg__invalidate(struct reg* const p_reg)
{
	assert(p_reg);
	if ( p_reg->exist ) {
		if ( p_reg->dirty ) {
			LOG_ERROR("Invalidate dirty register: %s", p_reg->name);
			struct target* const p_target = p_reg->arch_info;
			error_code__update(p_target, ERROR_TARGET_FAILURE);
		}
		p_reg->valid = false;
	}
}
static inline void reg__set_valid_value_to_cache(struct reg* const p_reg, uint32_t const value)
{
	assert(p_reg);
	assert(p_reg->exist);

	STATIC_ASSERT((CHAR_BIT) == 8);
	assert(p_reg->size <= (CHAR_BIT) * sizeof value);

	LOG_DEBUG("Updating cache from register %s to 0x%08X", p_reg->name, value);

	assert(p_reg->value);
	buf_set_u32(p_reg->value, 0, p_reg->size, value);

	p_reg->valid = true;
	p_reg->dirty = false;
}
static void reg__set_new_cache_value(struct reg* const p_reg, uint8_t* const buf)
{
	assert(p_reg);
	assert(buf);

	assert(p_reg->exist);

	STATIC_ASSERT((CHAR_BIT) == 8);
	assert(p_reg->size <= (CHAR_BIT) * sizeof(uint32_t));

	LOG_DEBUG("Set register %s cache to 0x%08X", p_reg->name, buf_get_u32(buf, 0, p_reg->size));

	assert(p_reg->value);
	buf_cpy(buf, p_reg->value, p_reg->size);

	p_reg->valid = true;
	p_reg->dirty = true;
}
static inline bool reg__check(struct reg const* const p_reg)
{
	if ( !p_reg->exist ) {
		LOG_ERROR("Register %s not available", p_reg->name);
		return false;
	} else if ( p_reg->dirty && !p_reg->valid ) {
		LOG_ERROR("Register %s dirty but not valid", p_reg->name);
		return false;
	} else {
		return true;
	}
}
static void reg_cache__invalidate(struct reg_cache const* const p_reg_cache)
{
	assert(p_reg_cache);

	assert(!(p_reg_cache->num_regs && !p_reg_cache->reg_list));
	for ( size_t i = 0; i < p_reg_cache->num_regs; ++i ) {
		struct reg *const p_reg = &p_reg_cache->reg_list[i];
		if ( p_reg->exist ) {
			assert(reg__check(&p_reg_cache->reg_list[i]));
			reg__invalidate(&p_reg_cache->reg_list[i]);
		}
	}
}
static void reg_cache__chain_invalidate(struct reg_cache* p_reg_cache)
{
	for ( ; p_reg_cache; p_reg_cache = p_reg_cache->next ) {
		reg_cache__invalidate(p_reg_cache);
	}
}
static void reg_x__operation_conditions_check(struct reg const* const p_reg)
{
	assert(p_reg);
	assert(reg__check(p_reg));
	struct target* p_target = p_reg->arch_info;
	assert(p_target);
	if ( !(p_reg->number < NUMBER_OF_X_REGS) ) {
		LOG_WARNING("Bad GP register %s id=%d", p_reg->name, p_reg->number);
		error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
		return;
	}
	sc_rv32_check_that_target_halted(p_target);
}
static int reg_x__get(struct reg* const p_reg)
{
	assert(p_reg);
	reg_x__operation_conditions_check(p_reg);
	struct target* p_target = p_reg->arch_info;
	assert(p_target);
	if ( error_code__get(p_target) != ERROR_OK ) {
		return error_code__get_and_clear(p_target);
	}

	if ( p_reg->valid ) {
		// register cache already valid
		if ( p_reg->dirty ) {
			LOG_WARNING("Try re-read dirty cache register %s", p_reg->name);
		} else {
			LOG_DEBUG("Try re-read cache register %s", p_reg->name);
		}
	}

	uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);
	if ( error_code__get(p_target) == ERROR_OK ) {
		struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
		assert(p_arch);
		size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);
		if ( p_arch->use_pc_advmt_dsbl_bit ) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
		}
		sc_rv32_EXEC__setup(p_target);
		int advance_pc_counter = 0;
		if ( error_code__get(p_target) != ERROR_OK ) {
			return error_code__get_and_clear(p_target);
		}
		// Save p_reg->number register to CSR_DBG_SCRATCH CSR
		(void)sc_rv32_EXEC__step(p_target, RV_CSRW(CSR_DBG_SCRATCH, p_reg->number));
		advance_pc_counter += instr_step;
		if ( error_code__get(p_target) != ERROR_OK ) {
			return error_code__get_and_clear(p_target);
		}

		// Exec jump back to previous instruction and get saved into CSR_DBG_SCRATCH CSR value
		assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
		uint32_t const value = sc_rv32_EXEC__step(p_target, RV_JAL(0, -advance_pc_counter));
		advance_pc_counter = 0;
		if ( error_code__get(p_target) != ERROR_OK ) {
			return error_code__get_and_clear(p_target);
		}
		reg__set_valid_value_to_cache(p_reg, value);
		if ( p_arch->use_pc_advmt_dsbl_bit ) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, 0);
		}
	}
	sc_rv32_check_PC_value(p_target, pc_sample_1);
	return error_code__get_and_clear(p_target);
}
static int reg_x__store(struct reg* const p_reg)
{
	assert(p_reg);
	struct target* p_target = p_reg->arch_info;
	assert(p_target);
	uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);

	if ( error_code__get(p_target) != ERROR_OK ) {
		return error_code__get_and_clear(p_target);
	}
	struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);
	size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);
	if ( p_arch->use_pc_advmt_dsbl_bit ) {
		sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
	}
	sc_rv32_EXEC__setup(p_target);
	int advance_pc_counter = 0;

	assert(p_reg->value);
	sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));
	if ( error_code__get(p_target) != ERROR_OK ) {
		return error_code__get_and_clear(p_target);
	}
	assert(p_reg->valid);
	assert(p_reg->dirty);
	(void)sc_rv32_EXEC__step(p_target, RV_CSRR(p_reg->number, CSR_DBG_SCRATCH));
	advance_pc_counter += instr_step;
	p_reg->dirty = false;

	LOG_DEBUG("Store register value 0x%08X from cache to register %s", buf_get_u32(p_reg->value, 0, p_reg->size), p_reg->name);

	if ( error_code__get(p_target) != ERROR_OK ) {
		return error_code__get_and_clear(p_target);
	}

#if VERIFY_REG_WRITE
	sc_rv32_EXEC__push_data_to_CSR(p_target, 0xFEEDBEEF);
	(void)sc_rv32_EXEC__step(p_target, RV_CSRW(CSR_DBG_SCRATCH, p_reg->number));
	advance_pc_counter += instr_step;
#endif
	/// Correct pc back after each instruction
	assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
#if VERIFY_REG_WRITE
	uint32_t const value = sc_rv32_EXEC__step(p_target, RV_JAL(0, -advance_pc_counter));
	if ( buf_get_u32(p_reg->value, 0, p_reg->size) != value ) {
		LOG_ERROR("Register %s write error: write 0x%08X, but re-read 0x%08X", p_reg->name, buf_get_u32(p_reg->value, 0, p_reg->size), value);
	}
#else
	(void)sc_rv32_EXEC__step(p_target, RV_JAL(0, -advance_pc_counter));
#endif
	advance_pc_counter = 0;

	if ( p_arch->use_pc_advmt_dsbl_bit ) {
		sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, 0);
	}
	assert(reg__check(p_reg));
	sc_rv32_check_PC_value(p_target, pc_sample_1);

	return error_code__get_and_clear(p_target);
}
static int reg_x__set(struct reg* const p_reg, uint8_t* const buf)
{
	assert(p_reg);
	assert(buf);
	reg_x__operation_conditions_check(p_reg);
	struct target* p_target = p_reg->arch_info;
	assert(p_target);
	if ( error_code__get(p_target) != ERROR_OK ) {
		return error_code__get_and_clear(p_target);
	}

	reg__set_new_cache_value(p_reg, buf);

	/// store dirty register data to HW
	return reg_x__store(p_reg);
}
static int reg_x0__get(struct reg* const p_reg)
{
	assert(p_reg);
	assert(p_reg->number == 0u);
	reg__set_valid_value_to_cache(p_reg, 0u);
	return ERROR_OK;
}
static int reg_x0__set(struct reg* const p_reg, uint8_t* const buf)
{
	assert(p_reg);
	assert(buf);
	LOG_ERROR("Try to write to read-only register");
	assert(p_reg->number == 0u);
	reg__set_valid_value_to_cache(p_reg, 0u);
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
}
static struct reg* prepare_temporary_GP_register(struct target const* const p_target, int const after_reg)
{
	assert(p_target);
	assert(p_target->reg_cache);
	struct reg* const p_reg_list = p_target->reg_cache->reg_list;
	assert(p_reg_list);
	assert(p_target->reg_cache->num_regs >= NUMBER_OF_X_REGS);
	struct reg* p_valid = NULL;
	struct reg* p_dirty = NULL;
	for ( size_t i = after_reg + 1; i < NUMBER_OF_X_REGS; ++i ) {
		assert(reg__check(&p_reg_list[i]));
		if ( p_reg_list[i].valid ) {
			if ( p_reg_list[i].dirty ) {
				p_dirty = &p_reg_list[i];
				p_valid = p_dirty;
				break;
			} else if ( !p_valid ) {
				p_valid = &p_reg_list[i];
			}
		}
	}

	if ( !p_dirty ) {
		if ( !p_valid ) {
			assert(after_reg + 1 < NUMBER_OF_X_REGS);
			p_valid = &p_reg_list[after_reg + 1];
			if ( error_code__update(p_target, reg_x__get(p_valid)) != ERROR_OK ) {
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
static uint32_t csr_get_value(struct target* const p_target, uint32_t const csr_number)
{
	uint32_t value = 0xBADBAD;
	assert(p_target);
	sc_rv32_check_that_target_halted(p_target);
	if ( error_code__get(p_target) == ERROR_OK ) {
		/// Find temporary GP register
		struct reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);
		assert(p_wrk_reg);

		uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);
		if ( error_code__get(p_target) == ERROR_OK ) {
			struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
			assert(p_arch);
			size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);
			if ( p_arch->use_pc_advmt_dsbl_bit ) {
				sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
			}
			sc_rv32_EXEC__setup(p_target);
			int advance_pc_counter = 0;
			if ( error_code__get(p_target) == ERROR_OK ) {
				/// Copy values to temporary register
				(void)sc_rv32_EXEC__step(p_target, RV_CSRR(p_wrk_reg->number, csr_number));
				advance_pc_counter += instr_step;
				if ( error_code__get(p_target) == ERROR_OK ) {
					/// and store temporary register to CSR_DBG_SCRATCH CSR.
					(void)sc_rv32_EXEC__step(p_target, RV_CSRW(CSR_DBG_SCRATCH, p_wrk_reg->number));
					advance_pc_counter += instr_step;
					if ( error_code__get(p_target) == ERROR_OK ) {
						/// Correct pc by jump 2 instructions back and get previous command result.
						assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
						value = sc_rv32_EXEC__step(p_target, RV_JAL(0, -advance_pc_counter));
						advance_pc_counter = 0;
					}
				}
			}
			if ( p_arch->use_pc_advmt_dsbl_bit ) {
				sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, 0);
			}
		}

		if ( error_code__get(p_target) == ERROR_OK ) {
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
static bool is_RVC_enable(struct target* const p_target)
{
	uint32_t const mcpuid = csr_get_value(p_target, CSR_mcpuid);
	return 0 != (mcpuid & (1u << ('C' - 'A')));
}
/// Update pc cache from HW (if non-cached)
static int reg_pc__get(struct reg* const p_reg)
{
	assert(p_reg);
	assert(p_reg->number == REG_PC_NUMBER);
	assert(reg__check(p_reg));

	/// Find temporary GP register
	struct target* const p_target = p_reg->arch_info;
	assert(p_target);
	struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);
	if ( p_arch->use_pc_from_pc_sample ) {
		uint32_t const pc_sample = sc_rv32_HART_REGTRANS_read(p_target, DBGC_HART_REGS_PC_SAMPLE);
		if ( error_code__get(p_target) == ERROR_OK ) {
			reg__set_valid_value_to_cache(p_reg, pc_sample);
		} else {
			reg__invalidate(p_reg);
		}
	} else {
		sc_rv32_check_that_target_halted(p_target);
		if ( error_code__get(p_target) != ERROR_OK ) {
			return error_code__get_and_clear(p_target);
		}
		struct reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);
		assert(p_wrk_reg);

		size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);
		if ( p_arch->use_pc_advmt_dsbl_bit ) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
		}
		sc_rv32_EXEC__setup(p_target);
		int advance_pc_counter = 0;
		if ( error_code__get(p_target) == ERROR_OK ) {
			/// Copy pc to temporary register by AUIPC instruction
			(void)sc_rv32_EXEC__step(p_target, RV_AUIPC(p_wrk_reg->number, 0));
			advance_pc_counter += instr_step;
			if ( error_code__get(p_target) == ERROR_OK ) {
				/// and store temporary register to CSR_DBG_SCRATCH CSR.
				(void)sc_rv32_EXEC__step(p_target, RV_CSRW(CSR_DBG_SCRATCH, p_wrk_reg->number));
				advance_pc_counter += instr_step;
				if ( error_code__get(p_target) == ERROR_OK ) {
					/// Correct pc by jump 2 instructions back and get previous command result.
					assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
					uint32_t const value = sc_rv32_EXEC__step(p_target, RV_JAL(0, -advance_pc_counter));
					advance_pc_counter = 0;
					if ( error_code__get(p_target) == ERROR_OK ) {
						reg__set_valid_value_to_cache(p_reg, value);
						uint32_t const pc_sample = sc_rv32_HART_REGTRANS_read(p_target, DBGC_HART_REGS_PC_SAMPLE);
						LOG_DEBUG("pc_sample = 0x%08X", pc_sample);
					}
				}
			}
		}
		if ( p_arch->use_pc_advmt_dsbl_bit ) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, 0);
		}
		// restore temporary register
		int const old_err_code = error_code__get_and_clear(p_target);
		error_code__update(p_target, reg_x__store(p_wrk_reg));
		error_code__prepend(p_target, old_err_code);
		assert(!p_wrk_reg->dirty);
	}
	return error_code__get_and_clear(p_target);
}
static int reg_pc__set(struct reg* const p_reg, uint8_t* const buf)
{
	assert(p_reg);
	assert(p_reg->number == REG_PC_NUMBER);
	assert(reg__check(p_reg));
	if ( !p_reg->valid ) {
		LOG_DEBUG("force rewriting of pc register before read");
	}

	struct target* const p_target = p_reg->arch_info;
	assert(p_target);
	sc_rv32_check_that_target_halted(p_target);
	if ( error_code__get(p_target) != ERROR_OK ) {
		return error_code__get_and_clear(p_target);
	}

	uint32_t const new_pc = buf_get_u32(buf, 0, p_reg->size);
	/// @note odd address is valid for pc, bit 0 value is ignored.
	if ( 0 != (new_pc & (1u << 1)) ) {
		bool const RVC_enable = is_RVC_enable(p_target);
		if ( error_code__get(p_target) != ERROR_OK ) {
			return error_code__get_and_clear(p_target);
		} else if ( !RVC_enable ) {
			LOG_ERROR("Unaligned PC: 0x%08X", new_pc);
			error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
			return error_code__get_and_clear(p_target);
		}
	}

	struct reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);
	assert(p_wrk_reg);

	reg__set_new_cache_value(p_reg, buf);

	struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
	assert(p_arch);
	size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);
	if ( p_arch->use_pc_advmt_dsbl_bit ) {
		sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
	}
	// Update to HW
	sc_rv32_EXEC__setup(p_target);
	int advance_pc_counter = 0;
	if ( error_code__get(p_target) == ERROR_OK ) {
		assert(p_reg->value);
		sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));
		if ( error_code__get(p_target) == ERROR_OK ) {
			// set temporary register value to restoring pc value
			(void)sc_rv32_EXEC__step(p_target, RV_CSRR(p_wrk_reg->number, CSR_DBG_SCRATCH));
			advance_pc_counter += instr_step;
			if ( error_code__get(p_target) == ERROR_OK ) {
				assert(p_wrk_reg->dirty);
				if ( p_arch->use_pc_advmt_dsbl_bit ) {
					sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, 0);
					sc_rv32_EXEC__setup(p_target);
				}
				/// and exec JARL to set pc
				(void)sc_rv32_EXEC__step(p_target, RV_JALR(0, p_wrk_reg->number, 0));
				advance_pc_counter = 0;
				assert(p_reg->valid);
				assert(p_reg->dirty);
				p_reg->dirty = false;
			}
		}
	}

#if 0
	if ( p_arch->use_pc_advmt_dsbl_bit ) {
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
static int reg_f__get(struct reg* const p_reg)
{
	assert(p_reg);
	assert(p_reg->number < NUMBER_OF_F_REGS);
	if ( !p_reg->exist ) {
		LOG_WARNING("FP register %s (#%d) is unavailable", p_reg->name, p_reg->number);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	assert(reg__check(p_reg));

	struct target* const p_target = p_reg->arch_info;
	assert(p_target);

	sc_rv32_check_that_target_halted(p_target);
	if ( error_code__get(p_target) != ERROR_OK ) {
		return error_code__get_and_clear(p_target);
	}

	/// @todo check that FPU is enabled
	/// Find temporary GP register
	struct reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);
	assert(p_wrk_reg);

	uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);
	if ( error_code__get(p_target) == ERROR_OK ) {
		struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
		assert(p_arch);
		size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);
		if ( p_arch->use_pc_advmt_dsbl_bit ) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
		}
		sc_rv32_EXEC__setup(p_target);
		int advance_pc_counter = 0;
		if ( error_code__get(p_target) == ERROR_OK ) {
			/// Copy values to temporary register
			(void)sc_rv32_EXEC__step(p_target, RV_FMV_X_S(p_wrk_reg->number, p_reg->number));
			advance_pc_counter += instr_step;
			if ( error_code__get(p_target) == ERROR_OK ) {
				/// and store temporary register to CSR_DBG_SCRATCH CSR.
				(void)sc_rv32_EXEC__step(p_target, RV_CSRW(CSR_DBG_SCRATCH, p_wrk_reg->number));
				advance_pc_counter += instr_step;
				if ( error_code__get(p_target) == ERROR_OK ) {
					/// Correct pc by jump 2 instructions back and get previous command result.
					assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
					uint32_t const value = sc_rv32_EXEC__step(p_target, RV_JAL(0, -advance_pc_counter));
					advance_pc_counter = 0;
					if ( error_code__get(p_target) == ERROR_OK ) {
						reg__set_valid_value_to_cache(p_reg, value);
					}
				}
			}
		}
		if ( p_arch->use_pc_advmt_dsbl_bit ) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, 0);
		}
	}

	sc_rv32_check_PC_value(p_target, pc_sample_1);

	// restore temporary register
	int const old_err_code = error_code__get_and_clear(p_target);
	if ( error_code__update(p_target, reg_x__store(p_wrk_reg)) == ERROR_OK ) {
		assert(!p_wrk_reg->dirty);
	}
	error_code__prepend(p_target, old_err_code);

	return error_code__get_and_clear(p_target);
}
static int reg_f__set(struct reg* const p_reg, uint8_t* const buf)
{
	assert(p_reg);
	assert(p_reg->number < NUMBER_OF_F_REGS);
	if ( !p_reg->exist ) {
		LOG_WARNING("Register %s is unavailable", p_reg->name);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	assert(reg__check(p_reg));

	struct target* const p_target = p_reg->arch_info;
	assert(p_target);

	sc_rv32_check_that_target_halted(p_target);
	if ( error_code__get(p_target) != ERROR_OK ) {
		return error_code__get_and_clear(p_target);
	}

	/// @todo check that FPU is enabled
	/// Find temporary GP register
	struct reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);
	assert(p_wrk_reg);

	uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);
	if ( error_code__get(p_target) == ERROR_OK ) {
		struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
		assert(p_arch);
		size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);
		if ( p_arch->use_pc_advmt_dsbl_bit ) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
		}
		sc_rv32_EXEC__setup(p_target);
		int advance_pc_counter = 0;
		if ( error_code__get(p_target) == ERROR_OK ) {
			reg__set_new_cache_value(p_reg, buf);

			sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));
			if ( error_code__get(p_target) == ERROR_OK ) {
				// set temporary register value to restoring pc value
				(void)sc_rv32_EXEC__step(p_target, RV_CSRR(p_wrk_reg->number, CSR_DBG_SCRATCH));
				advance_pc_counter += instr_step;
				if ( error_code__get(p_target) == ERROR_OK ) {
					assert(p_wrk_reg->dirty);
					assert(0 < p_wrk_reg->number && p_wrk_reg->number < NUMBER_OF_F_REGS);
					(void)sc_rv32_EXEC__step(p_target, RV_FMV_S_X(p_reg->number, p_wrk_reg->number));
					advance_pc_counter += instr_step;
					if ( error_code__get(p_target) == ERROR_OK ) {
						/// Correct pc by jump 2 instructions back and get previous command result.
						assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
						if ( advance_pc_counter ) {
							(void)sc_rv32_EXEC__step(p_target, RV_JAL(0, -advance_pc_counter));
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
		if ( p_arch->use_pc_advmt_dsbl_bit ) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, 0);
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
static int reg_csr__get(struct reg* const p_reg)
{
	assert(p_reg);
	uint32_t const csr_number = p_reg->number;
	assert(csr_number < 4096u);
	if ( !p_reg->exist ) {
		LOG_WARNING("CSR %s (#%d) is unavailable", p_reg->name, csr_number);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	assert(reg__check(p_reg));
	struct target* const p_target = p_reg->arch_info;
	assert(p_target);
	uint32_t const value = csr_get_value(p_target, csr_number);
	if ( error_code__get(p_target) == ERROR_OK ) {
		reg__set_valid_value_to_cache(p_reg, value);
	}
	return error_code__get_and_clear(p_target);
}
static int reg_csr__set(struct reg* const p_reg, uint8_t* const buf)
{
	assert(p_reg);
	assert(p_reg->number < 4096u);
	if ( !p_reg->exist ) {
		LOG_WARNING("Register %s is unavailable", p_reg->name);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	assert(reg__check(p_reg));

	struct target* const p_target = p_reg->arch_info;
	assert(p_target);

	sc_rv32_check_that_target_halted(p_target);
	if ( error_code__get(p_target) != ERROR_OK ) {
		return error_code__get_and_clear(p_target);
	}

	struct reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);
	assert(p_wrk_reg);

	uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);
	if ( error_code__get(p_target) == ERROR_OK ) {
		struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
		assert(p_arch);
		size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);
		if ( p_arch->use_pc_advmt_dsbl_bit ) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
		}
		sc_rv32_EXEC__setup(p_target);
		int advance_pc_counter = 0;
		if ( error_code__get(p_target) == ERROR_OK ) {
			reg__set_new_cache_value(p_reg, buf);

			sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));
			if ( error_code__get(p_target) == ERROR_OK ) {
				// set temporary register value
				(void)sc_rv32_EXEC__step(p_target, RV_CSRR(p_wrk_reg->number, CSR_DBG_SCRATCH));
				advance_pc_counter += instr_step;
				if ( error_code__get(p_target) == ERROR_OK ) {
					assert(p_wrk_reg->dirty);
					assert(p_wrk_reg->number < NUMBER_OF_X_REGS);
					(void)sc_rv32_EXEC__step(p_target, RV_CSRW(p_reg->number, p_wrk_reg->number));
					advance_pc_counter += instr_step;
					if ( error_code__get(p_target) == ERROR_OK ) {
						/// Correct pc by jump 2 instructions back and get previous command result.
						assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
						if ( advance_pc_counter ) {
							(void)sc_rv32_EXEC__step(p_target, RV_JAL(0, -advance_pc_counter));
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
		if ( p_arch->use_pc_advmt_dsbl_bit ) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, 0);
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
static struct reg_cache* reg_cache__section_create(char const* name, struct reg const regs_templates[], size_t const num_regs, void* const p_arch_info)
{
	assert(name);
	assert(0 < num_regs);
	assert(p_arch_info);
	struct reg* const p_dst_array = calloc(num_regs, sizeof(struct reg));
	struct reg* p_dst_iter = &p_dst_array[0];
	struct reg const* p_src_iter = &regs_templates[0];
	for ( size_t i = 0; i < num_regs; ++i ) {
		*p_dst_iter = *p_src_iter;
		p_dst_iter->value = calloc(1, NUM_BITS_TO_SIZE(p_src_iter->size));
		p_dst_iter->arch_info = p_arch_info;

		++p_src_iter;
		++p_dst_iter;
	}
	struct reg_cache const the_reg_cache = {
		.name = name,
		.reg_list = p_dst_array,
		.num_regs = num_regs,
	};

	struct reg_cache* const p_obj = calloc(1, sizeof(struct reg_cache));
	assert(p_obj);
	*p_obj = the_reg_cache;
	return p_obj;
}
static void set_DEMODE_ENBL(struct target* const p_target, uint32_t const set_value)
{
	sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DMODE_ENBL, set_value);
}
static int resume_common(struct target* const p_target, uint32_t dmode_enabled, int const current, uint32_t const address, int const handle_breakpoints, int const debug_execution)
{
	assert(p_target);
	sc_rv32_check_that_target_halted(p_target);
	if ( error_code__get(p_target) != ERROR_OK ) {
		return error_code__get_and_clear(p_target);
	}

	/// @todo multiple caches
	struct reg* const p_pc = &p_target->reg_cache->reg_list[NUMBER_OF_X_REGS];
	if ( !current ) {
		uint8_t buf[sizeof address];
		buf_set_u32(buf, 0, XLEN, address);
		error_code__update(p_target, reg_pc__set(p_pc, buf));
		if ( error_code__get(p_target) != ERROR_OK ) {
			return error_code__get_and_clear(p_target);
		}
		assert(!p_pc->dirty);
	}
	if ( handle_breakpoints ) {
		if ( current ) {
			// Find breakpoint for current instruction
			error_code__update(p_target, reg_pc__get(p_pc));
			assert(p_pc->value);
			uint32_t const pc = buf_get_u32(p_pc->value, 0, XLEN);
			struct breakpoint* p_next_bkp = p_target->breakpoints;
			for ( ; p_next_bkp; p_next_bkp = p_next_bkp->next ) {
				if ( p_next_bkp->set && (p_next_bkp->address == pc) ) {
					break;
				}
			}

			if ( p_next_bkp ) {
				// If next instruction is replaced by breakpoint, then execute saved instruction
				struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
				assert(p_arch);
				if ( p_arch->use_resume_at_sw_breakpoint_emulates_saved_instruction ) {
					uint32_t const instruction = buf_get_u32(p_next_bkp->orig_instr, 0, p_next_bkp->length * 8);
					sc_rv32_EXEC__setup(p_target);
					sc_rv32_EXEC__step(p_target, instruction);
					// If HART in single step mode
				} else {
					struct breakpoint next_bkp = *p_next_bkp;
					error_code__update(p_target, target_remove_breakpoint(p_target, &next_bkp));
					reg_cache__chain_invalidate(p_target->reg_cache);
					set_DEMODE_ENBL(p_target, dmode_enabled | BIT_NUM_TO_MASK(DBGC_HART_HDMER_SINGLE_STEP_BIT));
					sc_rv32_DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_DBGCMD);
					(void)sc_rv32_DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_RESUME) | BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS));
					error_code__update(p_target, target_add_breakpoint(p_target, &next_bkp));
				}
				if ( dmode_enabled & BIT_NUM_TO_MASK(DBGC_HART_HDMER_SINGLE_STEP_BIT) ) {
					// then single step already done
					reg_cache__chain_invalidate(p_target->reg_cache);
					set_DEMODE_ENBL(p_target, dmode_enabled);
#if 0
					target_call_event_callbacks(p_target, TARGET_EVENT_HALTED);
#endif
					LOG_DEBUG("update_status");
					sc_rv32_update_status(p_target);
#if 1
					LOG_DEBUG("New debug reason: 0x%08X", DBG_REASON_SINGLESTEP);
					p_target->debug_reason = DBG_REASON_SINGLESTEP;
#endif
					return error_code__get_and_clear(p_target);
				}
			}
		}
		// dmode_enabled |= BIT_NUM_TO_MASK(DBGC_HART_HDMER_SW_BRKPT_BIT);
	} else {
		dmode_enabled &= ~BIT_NUM_TO_MASK(DBGC_HART_HDMER_SW_BRKPT_BIT);
	}

	reg_cache__chain_invalidate(p_target->reg_cache);
	set_DEMODE_ENBL(p_target, dmode_enabled);
	if ( error_code__get(p_target) != ERROR_OK ) {
		return error_code__get_and_clear(p_target);
	}
	sc_rv32_DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_DBGCMD);
	if ( error_code__get(p_target) != ERROR_OK ) {
		LOG_WARNING("DAP_CTRL_REG_set error");
		return error_code__get_and_clear(p_target);
	}

	(void)sc_rv32_DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_RESUME) | BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS));
	if ( error_code__get(p_target) != ERROR_OK ) {
		return error_code__get_and_clear(p_target);
	}

	LOG_DEBUG("New debug reason: 0x%08X", DBG_REASON_NOTHALTED);
	p_target->debug_reason = DBG_REASON_NOTHALTED;
	target_call_event_callbacks(p_target, debug_execution ? TARGET_EVENT_DEBUG_RESUMED : TARGET_EVENT_RESUMED);

	LOG_DEBUG("update_status");
	sc_rv32_update_status(p_target);
	if ( error_code__get(p_target) != ERROR_OK ) {
		return error_code__get_and_clear(p_target);
	}

	return error_code__get_and_clear(p_target);
}
static int reset__set(struct target* const p_target, bool const active)
{
	assert(p_target);
	sc_rv32_DAP_CTRL_REG_set(p_target, DBGC_UNIT_ID_CORE, DBGC_FGRP_CORE_REGTRANS);
	if ( error_code__get(p_target) == ERROR_OK ) {
		uint32_t const get_old_value1 = sc_rv32_core_REGTRANS_read(p_target, DBGC_CORE_REGS_DBG_CTRL);
		if ( error_code__get(p_target) == ERROR_OK ) {
			static uint32_t const bit_mask = BIT_NUM_TO_MASK(DBGC_CORE_CDCR_HART0_RST_BIT) | BIT_NUM_TO_MASK(DBGC_CORE_CDCR_RST_BIT);
			uint32_t const set_value = (get_old_value1 & ~bit_mask) | (active ? bit_mask : 0u);
			sc_rv32_CORE_REGTRANS_write(p_target, DBGC_CORE_REGS_DBG_CTRL, set_value);
			if ( error_code__get(p_target) == ERROR_OK ) {
				struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
				assert(p_arch);
				if ( p_arch->use_verify_core_regtrans_write ) {
					uint32_t const get_new_value2 = sc_rv32_core_REGTRANS_read(p_target, DBGC_CORE_REGS_DBG_CTRL);
					if ( error_code__get(p_target) != ERROR_OK ) {
						return error_code__get_and_clear(p_target);
					}
					if ( (get_new_value2 & bit_mask) != (set_value & bit_mask) ) {
						LOG_ERROR("Fail to verify write: set 0x%08X, but get 0x%08X", set_value, get_new_value2);
						error_code__update(p_target, ERROR_TARGET_FAILURE);
						return error_code__get_and_clear(p_target);
					}
				}
				LOG_DEBUG("update_status");
				sc_rv32_update_status(p_target);
				if ( error_code__get(p_target) == ERROR_OK ) {
					if ( active ) {
						if ( p_target->state != TARGET_RESET ) {
							/// issue error if we are still running
							LOG_ERROR("Target is not resetting after reset assert");
							error_code__update(p_target, ERROR_TARGET_FAILURE);
						}
					} else {
						if ( p_target->state == TARGET_RESET ) {
							LOG_ERROR("Target is still in reset after reset deassert");
							error_code__update(p_target, ERROR_TARGET_FAILURE);
						}
					}
				}
			}
		}
	}

	return error_code__get_and_clear(p_target);
}

static struct reg_arch_type const reg_x0_accessors =
{
	.get = reg_x0__get,
	.set = reg_x0__set,
};

static struct reg_arch_type const reg_x_accessors =
{
	.get = reg_x__get,
	.set = reg_x__set,
};

static struct reg_arch_type const reg_pc_accessors =
{
	.get = reg_pc__get,
	.set = reg_pc__set,
};

static struct reg_arch_type const reg_f_accessors =
{
	.get = reg_f__get,
	.set = reg_f__set,
};

static struct reg_arch_type const reg_csr_accessors =
{
	.get = reg_csr__get,
	.set = reg_csr__set,
};
static struct reg_feature feature_riscv_org = {
	.name = "org.gnu.gdb.riscv.cpu",
};

static char const def_GP_regs_name[] = "rv32i";
static struct reg const def_GP_regs_array[] = {
	// Hard-wired zero
	{.name = "x0",.number = 0,.caller_save = false,.dirty = false,.valid = true,.exist = true,.size = XLEN,.type = &reg_x0_accessors,.feature = &feature_riscv_org},

	// Return address
	{.name = "x1",.number = 1,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},

	// Stack pointer
	{.name = "x2",.number = 2,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},

	// Global pointer
	{.name = "x3",.number = 3,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},

	// Thread pointer
	{.name = "x4",.number = 4,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},

	// Temporaries
	{.name = "x5",.number = 5,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x6",.number = 6,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x7",.number = 7,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},

	// Saved register/frame pointer
	{.name = "x8",.number = 8,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},

	// Saved register
	{.name = "x9",.number = 9,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},

	// Function arguments/return values
	{.name = "x10",.number = 10,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x11",.number = 11,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},

	// Function arguments
	{.name = "x12",.number = 12,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x13",.number = 13,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x14",.number = 14,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x15",.number = 15,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x16",.number = 16,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x17",.number = 17,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},

	// Saved registers
	{.name = "x18",.number = 18,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x19",.number = 19,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x20",.number = 20,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x21",.number = 21,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x22",.number = 22,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x23",.number = 23,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x24",.number = 24,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x25",.number = 25,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x26",.number = 26,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x27",.number = 27,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},

	// Temporaries
	{.name = "x28",.number = 28,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x29",.number = 29,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x30",.number = 30,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},
	{.name = "x31",.number = 31,.caller_save = true,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_x_accessors,.feature = &feature_riscv_org},

	// Program counter
	{.name = "pc",.number = REG_PC_NUMBER,.caller_save = false,.dirty = false,.valid = false,.exist = true,.size = XLEN,.type = &reg_pc_accessors,.feature = &feature_riscv_org},
};

static char const def_FP_regs_name[] = "rv32if";
static struct reg const def_FP_regs_array[] = {
	// FP temporaries
	{.name = "f0",.number = 0,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f1",.number = 1,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f2",.number = 2,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f3",.number = 3,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f4",.number = 4,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f5",.number = 5,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f6",.number = 6,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f7",.number = 7,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},

	// FP saved registers
	{.name = "f8",.number = 8,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f9",.number = 9,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},

	// FP arguments/return values
	{.name = "f10",.number = 10,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f11",.number = 11,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},

	// FP arguments
	{.name = "f12",.number = 12,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f13",.number = 13,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f14",.number = 14,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f15",.number = 15,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f16",.number = 16,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f17",.number = 17,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},

	// FP saved registers
	{.name = "f18",.number = 18,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f19",.number = 19,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f20",.number = 20,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f21",.number = 21,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f22",.number = 22,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f23",.number = 23,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f24",.number = 24,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f25",.number = 25,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f26",.number = 26,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f27",.number = 27,.caller_save = false,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},

	// FP temporaries
	{.name = "f28",.number = 28,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f29",.number = 29,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f30",.number = 30,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
	{.name = "f31",.number = 31,.caller_save = true,.dirty = false,.valid = false,.exist = FP_enabled,.size = FLEN,.type = &reg_f_accessors,.feature = &feature_riscv_org},
};

static char const def_CSR_regs_name[] = "rv32iCSR";
static struct reg const def_CSR_regs_array[] = {
#define DEF_ITEM(NAME) {.name = #NAME, .number = CSR_##NAME, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_csr_accessors, .feature = &feature_riscv_org}
	DEF_ITEM(mcpuid),
	DEF_ITEM(mimpid),
	DEF_ITEM(mhartid),
	DEF_ITEM(mstatus),
	DEF_ITEM(mtvec),
	DEF_ITEM(mepc),
	DEF_ITEM(mcause),
	DEF_ITEM(mbadaddr),
	DEF_ITEM(mip),
	DEF_ITEM(mie),
	DEF_ITEM(mtime),
	DEF_ITEM(mtimeh),
	DEF_ITEM(mtimecmp),
	DEF_ITEM(fflags),
	DEF_ITEM(frm),
};
#undef DEF_ITEM

static void sc_rv32_init_regs_cache(struct target* const p_target)
{
	assert(p_target);
#if 0  // TODO check
	assert(!p_target->reg_cache);
#endif
	struct reg_cache* p_reg_cache_last = p_target->reg_cache = reg_cache__section_create(def_GP_regs_name, def_GP_regs_array, ARRAY_LEN(def_GP_regs_array), p_target);
	p_reg_cache_last = p_reg_cache_last->next = reg_cache__section_create(def_FP_regs_name, def_FP_regs_array, ARRAY_LEN(def_FP_regs_array), p_target);
	p_reg_cache_last->next = reg_cache__section_create(def_CSR_regs_name, def_CSR_regs_array, ARRAY_LEN(def_CSR_regs_array), p_target);
}
static int sc_rv32i__init_target(struct command_context *cmd_ctx, struct target* const p_target)
{
	sc_rv32_init_regs_cache(p_target);

	struct sc_rv32i__Arch* p_arch_info = calloc(1, sizeof(struct sc_rv32i__Arch));
	assert(p_arch_info);
	*p_arch_info = sc_rv32_initial_arch;

	p_target->arch_info = p_arch_info;
	return ERROR_OK;
}
static void sc_rv32i__deinit_target(struct target* const p_target)
{
	assert(p_target);
	while ( p_target->reg_cache ) {
		struct reg_cache* const p_reg_cache = p_target->reg_cache;
		p_target->reg_cache = p_target->reg_cache->next;
		struct reg* const reg_list = p_reg_cache->reg_list;
		assert(!p_reg_cache->num_regs || reg_list);
		for ( unsigned i = 0; i < p_reg_cache->num_regs; ++i ) {
			free(reg_list[i].value);
		}
		free(reg_list);

		free(p_reg_cache);
	}
	if ( p_target->arch_info ) {
		free(p_target->arch_info);
		p_target->arch_info = NULL;
	}
}
static int sc_rv32i__target_create(struct target* const p_target, struct Jim_Interp *interp)
{
	assert(p_target);
	return ERROR_OK;
}
static int sc_rv32i__examine(struct target* const p_target)
{
	assert(p_target);
	for ( int i = 0; i < 10; ++i ) {
		error_code__get_and_clear(p_target);
		LOG_DEBUG("update_status");
		sc_rv32_update_status(p_target);
		if ( error_code__get(p_target) == ERROR_OK ) {
			break;
		}
		LOG_DEBUG("update_status error, retry");
	}

	if ( error_code__get(p_target) == ERROR_OK ) {
		uint32_t const IDCODE = sc_rv32_IDCODE_get(p_target);
		if ( IDCODE != EXPECTED_IDCODE ) {
			LOG_ERROR("Invalid IDCODE=0x%08X!", IDCODE);
			error_code__update(p_target, ERROR_TARGET_FAILURE);
		} else {
			uint32_t const DBG_ID = sc_rv32_DBG_ID_get(p_target);
			if ( (DBG_ID & DBG_ID_VERSION_MASK) != (DBG_ID_VERSION_MASK & EXPECTED_DBG_ID) ||
				(DBG_ID & DBG_ID_SUBVERSION_MASK) < (EXPECTED_DBG_ID & DBG_ID_SUBVERSION_MASK) ) {
				LOG_ERROR("Unsupported DBG_ID=0x%08X!", DBG_ID);
				error_code__update(p_target, ERROR_TARGET_FAILURE);
			} else {
				LOG_INFO("IDCODE=0x%08X DBG_ID=0x%08X BLD_ID=0x%08X", IDCODE, DBG_ID, sc_rv32_BLD_ID_get(p_target));
				set_DEMODE_ENBL(p_target, NORMAL_DEBUG_ENABLE_MASK);
				if ( error_code__get(p_target) == ERROR_OK ) {
					LOG_DEBUG("Examined OK");
					target_set_examined(p_target);
				}
			}
		}
	}

	return error_code__get_and_clear(p_target);
}
static int sc_rv32i__poll(struct target* const p_target)
{
	assert(p_target);
	LOG_DEBUG("update_status");
	sc_rv32_update_status(p_target);
	if ( error_code__get(p_target) != ERROR_OK ) {
		return error_code__get_and_clear(p_target);
	}
	return error_code__get_and_clear(p_target);
}
static int sc_rv32i__arch_state(struct target* const p_target)
{
	assert(p_target);
	LOG_DEBUG("update_status");
	sc_rv32_update_status(p_target);
	return error_code__get_and_clear(p_target);
}
static int sc_rv32i__halt(struct target* const p_target)
{
	assert(p_target);
	// May be already halted?
	{
		LOG_DEBUG("update_status");
		sc_rv32_update_status(p_target);
		if ( error_code__get(p_target) != ERROR_OK ) {
			return error_code__get_and_clear(p_target);
		}

		if ( p_target->state == TARGET_HALTED ) {
			LOG_WARNING("Halt request when target is already in halted state");
			return error_code__get_and_clear(p_target);
		}
	}

	// Try to halt
	{
		sc_rv32_DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_DBGCMD);
		if ( error_code__get(p_target) != ERROR_OK ) {
			LOG_WARNING("DAP_CTRL_REG_set error");
			return error_code__get_and_clear(p_target);
		}

		(void)sc_rv32_DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_HALT) | BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS));
		if ( error_code__get(p_target) != ERROR_OK ) {
			return error_code__get_and_clear(p_target);
		}
	}

	LOG_DEBUG("update_status");
	sc_rv32_update_status(p_target);
	if ( error_code__get(p_target) != ERROR_OK ) {
		return error_code__get_and_clear(p_target);
	}

	// Verify that in debug mode
	sc_rv32_check_that_target_halted(p_target);
	if ( error_code__get(p_target) != ERROR_OK ) {
		return error_code__get_and_clear(p_target);
	}

	return error_code__get_and_clear(p_target);
}
static int sc_rv32i__resume(struct target* const p_target, int const current, uint32_t const address, int const handle_breakpoints, int const debug_execution)
{
	LOG_DEBUG("resume: current=%d address=0x%08x handle_breakpoints=%d debug_execution=%d", current, address, handle_breakpoints, debug_execution);
	assert(p_target);
	uint32_t const dmode_enabled = NORMAL_DEBUG_ENABLE_MASK;
	return resume_common(p_target, dmode_enabled, current, address, handle_breakpoints, debug_execution);
}
static int sc_rv32i__step(struct target* const p_target, int const current, uint32_t const address, int const handle_breakpoints)
{
	LOG_DEBUG("step: current=%d address=0x%08x handle_breakpoints=%d", current, address, handle_breakpoints);
	assert(p_target);
	uint32_t const dmode_enabled = (NORMAL_DEBUG_ENABLE_MASK & ~BIT_NUM_TO_MASK(DBGC_HART_HDMER_SW_BRKPT_BIT)) | BIT_NUM_TO_MASK(DBGC_HART_HDMER_SINGLE_STEP_BIT);
	return resume_common(p_target, dmode_enabled, current, address, handle_breakpoints, false);
}
static int sc_rv32i__assert_reset(struct target* const p_target)
{
	LOG_DEBUG("Reset control");
	assert(p_target);
	return reset__set(p_target, true);
}
static int sc_rv32i__deassert_reset(struct target* const p_target)
{
	LOG_DEBUG("Reset control");
	assert(p_target);
	return reset__set(p_target, false);
}
static int sc_rv32i__soft_reset_halt(struct target* const p_target)
{
	LOG_DEBUG("Soft reset called");
	assert(p_target);
	reset__set(p_target, true);
	if ( error_code__get(p_target) == ERROR_OK ) {
		set_DEMODE_ENBL(p_target, NORMAL_DEBUG_ENABLE_MASK | BIT_NUM_TO_MASK(DBGC_HART_HDMER_RST_EXIT_BRK_BIT));
		if ( error_code__get(p_target) == ERROR_OK ) {
			reset__set(p_target, false);
		}
	}

	return error_code__get_and_clear(p_target);
}
static int sc_rv32i__mmu(struct target *p_target, int *p_mmu_enabled)
{
	assert(p_target);
	uint32_t const mstatus = csr_get_value(p_target, CSR_mstatus);
	if ( error_code__get(p_target) == ERROR_OK ) {
		uint32_t const PRV = (mstatus >> 1) & LOW_BITS_MASK(2);
		assert(p_mmu_enabled);
		if ( PRV == Priv_M || PRV == Priv_H ) {
			*p_mmu_enabled = 0;
		} else {
			uint32_t const VM = (mstatus >> 17) & LOW_BITS_MASK(21 - 16);
			switch ( VM ) {
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
	}
	return error_code__get_and_clear(p_target);
}
static void virt_to_phis(struct target *p_target, uint32_t address, uint32_t *p_physical, uint32_t* p_bound, bool const instruction_space)
{
	assert(p_target);
	uint32_t const mstatus = csr_get_value(p_target, CSR_mstatus);
	if ( error_code__get(p_target) == ERROR_OK ) {
		uint32_t const PRV = (mstatus >> 1) & LOW_BITS_MASK(2);
		uint32_t const VM = PRV == Priv_M || PRV == Priv_H ? VM_Mbare : (mstatus >> 17) & LOW_BITS_MASK(21 - 16);
		assert(p_physical);
		switch ( VM ) {
		case VM_Mbare:
			*p_physical = address;
			if ( p_bound ) {
				*p_bound = UINT32_MAX;
			}
			break;

		case VM_Mbb:
		case VM_Mbbid:
			{
				uint32_t const bound = csr_get_value(p_target, VM == VM_Mbb ? CSR_mbound : /*VM == VM_Mbbid*/instruction_space ? CSR_mibound : CSR_mdbound);
				if ( error_code__get(p_target) == ERROR_OK ) {
					if ( !(address < bound) ) {
						error_code__update(p_target, ERROR_TARGET_TRANSLATION_FAULT);
					} else {
						uint32_t const base = csr_get_value(p_target, VM_Mbb ? CSR_mbase : /*VM == VM_Mbbid*/instruction_space ? CSR_mibase : CSR_mdbase);
						if ( error_code__get(p_target) == ERROR_OK ) {
							*p_physical = address + base;
							if ( p_bound ) {
								*p_bound = bound - address;
							}
						}
					}
				}
			}
			break;

		case VM_Sv32:
			{
				static uint32_t const offset_mask = LOW_BITS_MASK(10) << 2;
				uint32_t const main_page = csr_get_value(p_target, CSR_sptbr);
				if ( ERROR_OK == error_code__get(p_target) ) {
					// lower bits should be zero
					assert(0 == (main_page & LOW_BITS_MASK(12)));
					uint32_t const offset_bits1 = address >> 20 & offset_mask;
					uint32_t pte1;
					if ( ERROR_OK == error_code__update(p_target, target_read_phys_memory(p_target, main_page | offset_bits1, 4, 1, (uint8_t *)&pte1)) ) {
						if ( 0 == (pte1 & BIT_NUM_TO_MASK(0)) ) {
							error_code__update(p_target, ERROR_TARGET_TRANSLATION_FAULT);
						} else if ( (pte1 >> 1 & LOW_BITS_MASK(4)) >= 2 ) {
							*p_physical = (pte1 << 2 & ~LOW_BITS_MASK(22)) | (address & LOW_BITS_MASK(22));
							if ( p_bound ) {
								*p_bound = BIT_NUM_TO_MASK(22) - (address & LOW_BITS_MASK(22));
							}
						} else {
							uint32_t const base_0 = pte1 << 2 & ~LOW_BITS_MASK(12);
							uint32_t const offset_bits0 = address >> 10 & offset_mask;
							uint32_t pte0;
							if ( ERROR_OK == error_code__update(p_target, target_read_phys_memory(p_target, base_0 | offset_bits0, 4, 1, (uint8_t *)&pte0)) ) {
								if ( 0 == (pte0 & BIT_NUM_TO_MASK(0)) || (pte0 >> 1 & LOW_BITS_MASK(4)) < 2 ) {
									error_code__update(p_target, ERROR_TARGET_TRANSLATION_FAULT);
								} else {
									*p_physical = (pte0 << 2 & ~LOW_BITS_MASK(12)) | (address & LOW_BITS_MASK(12));
									if ( p_bound ) {
										*p_bound = BIT_NUM_TO_MASK(12) - (address & LOW_BITS_MASK(12));
									}
								}
							}
						}
					}
				}
			}
			break;

		case VM_Sv39:
		case VM_Sv48:
		default:
			error_code__update(p_target, ERROR_TARGET_TRANSLATION_FAULT);
			break;
		}
	}
}
static int sc_rv32i__virt2phys(struct target *p_target, uint32_t address, uint32_t *p_physical)
{
	virt_to_phis(p_target, address, p_physical, NULL, false);
	return error_code__get_and_clear(p_target);
}
static void read_memory_space(struct target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t* p_buffer, bool const instruction_space)
{
	assert(p_target);
	if ( !(size == 1 || size == 2 || size == 4) ) {
		LOG_ERROR("Invalid item size %d", size);
		error_code__update(p_target, ERROR_TARGET_FAILURE);
	} else if ( address % size != 0 ) {
		LOG_ERROR("Unaligned access at 0x%08X, for item size %d", address, size);
		error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
	} else {
		while ( 0 != count ) {
			uint32_t physical;
			uint32_t bound;
			virt_to_phis(p_target, address, &physical, &bound, instruction_space);
			if ( ERROR_OK != error_code__get(p_target) ) {
				break;
			}
			uint32_t const page_count = size * count > bound ? bound / size : count;
			assert(0 != page_count);
			assert(p_buffer);
			if ( ERROR_OK != error_code__update(p_target, target_read_phys_memory(p_target, physical, size, page_count, p_buffer)) ) {
				break;
			}
			uint32_t const bytes = size * page_count;
			p_buffer += bytes;
			address += bytes;
			count -= page_count;
		}
	}
}
static void write_memory_space(struct target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t const* p_buffer, bool const instruction_space)
{
	assert(p_target);
	if ( !(size == 1 || size == 2 || size == 4) ) {
		LOG_ERROR("Invalid item size %d", size);
		error_code__update(p_target, ERROR_TARGET_FAILURE);
	} else if ( address % size != 0 ) {
		LOG_ERROR("Unaligned access at 0x%08X, for item size %d", address, size);
		error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
	} else {
		while ( 0 != count ) {
			uint32_t physical;
			uint32_t bound;
			virt_to_phis(p_target, address, &physical, &bound, instruction_space);
			if ( ERROR_OK != error_code__get(p_target) ) {
				break;
			}
			uint32_t const page_count = size * count > bound ? bound / size : count;
			assert(0 != page_count);
			assert(p_buffer);
			if ( ERROR_OK != error_code__update(p_target, target_write_phys_memory(p_target, physical, size, page_count, p_buffer)) ) {
				break;
			}
			uint32_t const bytes = size * page_count;
			p_buffer += bytes;
			address += bytes;
			count -= page_count;
		}
	}
}
static int sc_rv32i__read_memory(struct target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t* buffer)
{
	read_memory_space(p_target, address, size, count, buffer, false);
	return error_code__get_and_clear(p_target);
}
static int sc_rv32i__write_memory(struct target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t const* buffer)
{
	write_memory_space(p_target, address, size, count, buffer, false);
	return error_code__get_and_clear(p_target);
}
static int sc_rv32i__read_phys_memory(struct target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t* buffer)
{
	assert(p_target);
	assert(buffer);
	LOG_DEBUG("Read_memory at 0x%08X, %d items, each %d bytes, total %d bytes", address, count, size, count * size);
	/// Check for size
	if ( !(size == 1 || size == 2 || size == 4) ) {
		LOG_ERROR("Invalid item size %d", size);
		error_code__update(p_target, ERROR_TARGET_FAILURE);
		return error_code__get_and_clear(p_target);
	} else if ( address % size != 0 ) {
		LOG_ERROR("Unaligned access at 0x%08X, for item size %d", address, size);
		error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
		return error_code__get_and_clear(p_target);
	} else {
		/// Check that target halted
		sc_rv32_check_that_target_halted(p_target);
		if ( error_code__get(p_target) != ERROR_OK ) {
			return error_code__get_and_clear(p_target);
		}

		/// Reserve work register
		struct reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);
		assert(p_wrk_reg);

		uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);
		if ( error_code__get(p_target) == ERROR_OK ) {
			/// Define opcode for load item to register
			uint32_t const load_OP =
				size == 4 ? RV_LW(p_wrk_reg->number, p_wrk_reg->number, 0) :
				size == 2 ? RV_LH(p_wrk_reg->number, p_wrk_reg->number, 0) :
				/*size == 1*/RV_LB(p_wrk_reg->number, p_wrk_reg->number, 0);

			struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
			assert(p_arch);
			size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);
			if ( p_arch->use_pc_advmt_dsbl_bit ) {
				sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
			}
			/// Setup exec operations mode
			sc_rv32_EXEC__setup(p_target);
			int advance_pc_counter = 0;
			if ( error_code__get(p_target) == ERROR_OK ) {
				/// For count number of items do loop
				while ( count-- ) {
					/// Set address to CSR
					sc_rv32_EXEC__push_data_to_CSR(p_target, address);
					if ( error_code__get(p_target) != ERROR_OK ) {
						break;
					}

					/// Load address to work register
					(void)sc_rv32_EXEC__step(p_target, RV_CSRR(p_wrk_reg->number, CSR_DBG_SCRATCH));
					advance_pc_counter += instr_step;
					if ( error_code__get(p_target) != ERROR_OK ) {
						break;
					}

					/// Exec load item to register
					(void)sc_rv32_EXEC__step(p_target, load_OP);
					advance_pc_counter += instr_step;
					if ( error_code__get(p_target) != ERROR_OK ) {
						break;
					}

					/// Exec store work register to csr
					(void)sc_rv32_EXEC__step(p_target, RV_CSRW(CSR_DBG_SCRATCH, p_wrk_reg->number));
					advance_pc_counter += instr_step;

					/// get data from csr and jump back to correct pc
					assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
					uint32_t const value = sc_rv32_EXEC__step(p_target, RV_JAL(0, -advance_pc_counter));
					advance_pc_counter = 0;
					if ( error_code__get(p_target) != ERROR_OK ) {
						break;
					}

					/// store read data to buffer
					buf_set_u32(buffer, 0, 8 * size, value);

					/// advance src/dst pointers
					address += size;
					buffer += size;
				}
			}
			if ( p_arch->use_pc_advmt_dsbl_bit ) {
				sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, 0);
			}
		}

		if ( error_code__get(p_target) != ERROR_OK ) {
			LOG_DEBUG("update_status");
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
static int sc_rv32i__write_phys_memory(struct target* const p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t const* buffer)
{
	assert(p_target);
	assert(buffer);
	LOG_DEBUG("Write_memory at 0x%08X, %d items, each %d bytes, total %d bytes", address, count, size, count * size);
	/// Check for size
	if ( !(size == 1 || size == 2 || size == 4) ) {
		LOG_ERROR("Invalid item size %d", size);
		error_code__update(p_target, ERROR_TARGET_FAILURE);
		return error_code__get_and_clear(p_target);
	}

	/// Check for alignment
	if ( address % size != 0 ) {
		LOG_ERROR("Unaligned access at 0x%08X, for item size %d", address, size);
		error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
		return error_code__get_and_clear(p_target);
	}

	if ( count == 0 ) {
		return error_code__get_and_clear(p_target);
	}

	sc_rv32_check_that_target_halted(p_target);
	if ( error_code__get(p_target) != ERROR_OK ) {
		return error_code__get_and_clear(p_target);
	}

	uint32_t const pc_sample_1 = sc_rv32_get_PC(p_target);
	/// Reserve work register
	struct reg* const p_addr_reg = prepare_temporary_GP_register(p_target, 0);
	assert(p_addr_reg);
	struct reg* const p_data_reg = prepare_temporary_GP_register(p_target, p_addr_reg->number);
	assert(p_data_reg);
	assert(p_addr_reg->number != p_data_reg->number);

	if ( error_code__get(p_target) == ERROR_OK ) {
		struct sc_rv32i__Arch const* const p_arch = p_target->arch_info;
		assert(p_arch);
		size_t const instr_step = p_arch->use_pc_advmt_dsbl_bit ? 0u : NUM_BITS_TO_SIZE(ILEN);
		if ( p_arch->use_pc_advmt_dsbl_bit ) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT));
		}
		/// Setup exec operations mode
		sc_rv32_EXEC__setup(p_target);
		size_t advance_pc_counter = 0;
		if ( error_code__get(p_target) == ERROR_OK ) {
			// Set address to CSR
			sc_rv32_EXEC__push_data_to_CSR(p_target, address);
			if ( error_code__get(p_target) == ERROR_OK ) {
				/// Load address to work register
				(void)sc_rv32_EXEC__step(p_target, RV_CSRR(p_addr_reg->number, CSR_DBG_SCRATCH));
				advance_pc_counter += instr_step;

				// Opcodes
				uint32_t const instructions[3] = {
					RV_CSRR(p_data_reg->number, CSR_DBG_SCRATCH),
					(size == 4 ? RV_SW(p_data_reg->number, p_addr_reg->number, 0) :
					size == 2 ? RV_SH(p_data_reg->number, p_addr_reg->number, 0) :
					/*size == 1*/ RV_SB(p_data_reg->number, p_addr_reg->number, 0)),
					RV_ADDI(p_addr_reg->number, p_addr_reg->number, size)
				};

				static uint32_t max_pc_offset = (((1u << 20) - 1u) / NUM_BITS_TO_SIZE(XLEN)) * NUM_BITS_TO_SIZE(XLEN);
				if ( p_arch->use_queuing_for_dr_scans ) {
					uint8_t DAP_OPSTATUS = 0;
					uint8_t const data_wr_opcode[1] = {DBGC_DAP_OPCODE_DBGCMD_DBGDATA_WR};
					struct scan_field const data_scan_opcode_field = {
						.num_bits = TAP_LEN_DAP_CMD_OPCODE,
						.out_value = data_wr_opcode,
						.in_value = &DAP_OPSTATUS,
						.check_value = &DAP_OPSTATUS_GOOD,
						.check_mask = &DAP_STATUS_MASK,
					};
					struct scan_field data_scan_fields[TAP_NUM_FIELDS_DAP_CMD] = {{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT}, data_scan_opcode_field,};
					uint8_t const instr_exec_opcode[1] = {DBGC_DAP_OPCODE_DBGCMD_CORE_EXEC};
					struct scan_field const instr_scan_opcode_field = {
						.num_bits = TAP_LEN_DAP_CMD_OPCODE,
						.out_value = instr_exec_opcode,
						.in_value = &DAP_OPSTATUS,
						.check_value = &DAP_OPSTATUS_GOOD,
						.check_mask = &DAP_STATUS_MASK,
					};
					struct scan_field const instr_fields[ARRAY_LEN(instructions)][TAP_NUM_FIELDS_DAP_CMD] = {
						{{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT,.out_value = (uint8_t const*)(&instructions[0])}, instr_scan_opcode_field},
						{{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT,.out_value = (uint8_t const*)(&instructions[1])}, instr_scan_opcode_field},
						{{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT,.out_value = (uint8_t const*)(&instructions[2])}, instr_scan_opcode_field}
					};

					data_scan_fields[0].out_value = (uint8_t const*)buffer;
					LOG_DEBUG("Repeat in loop %d times:", count);
					LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X", p_target->cmd_name,
						data_scan_fields[0].num_bits, buf_get_u32(data_scan_fields[0].out_value, 0, data_scan_fields[0].num_bits),
						data_scan_fields[1].num_bits, buf_get_u32(data_scan_fields[1].out_value, 0, data_scan_fields[1].num_bits));
					for ( unsigned i = 0; i < ARRAY_LEN(instr_fields); ++i ) {
						LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X", p_target->cmd_name,
							instr_fields[i][0].num_bits, buf_get_u32(instr_fields[i][0].out_value, 0, instr_fields[i][0].num_bits),
							instr_fields[i][1].num_bits, buf_get_u32(instr_fields[i][1].out_value, 0, instr_fields[i][1].num_bits));
					}

					size_t count1 = 0;
					while ( error_code__get(p_target) == ERROR_OK && count-- ) {
						assert(p_target->tap);
						data_scan_fields[0].out_value = (uint8_t const*)buffer;
						jtag_add_dr_scan_check(p_target->tap, ARRAY_LEN(data_scan_fields), data_scan_fields, TAP_IDLE);
						for ( unsigned i = 0; i < ARRAY_LEN(instr_fields); ++i ) {
							jtag_add_dr_scan_check(p_target->tap, TAP_NUM_FIELDS_DAP_CMD, instr_fields[i], TAP_IDLE);
							advance_pc_counter += instr_step;
						}
						buffer += size;
						if ( ++count1 >= WRITE_BUFFER_THRESHOLD ) {
							LOG_DEBUG("Force jtag_execute_queue_noclear()");
							jtag_execute_queue_noclear();
							count1 = 0;
						}
					}
					LOG_DEBUG("End loop");
					if ( !p_arch->use_pc_advmt_dsbl_bit ) {
						assert(advance_pc_counter % NUM_BITS_TO_SIZE(XLEN) == 0);
						while ( advance_pc_counter ) {
							uint32_t const step_back = advance_pc_counter > max_pc_offset ? max_pc_offset : advance_pc_counter;
							advance_pc_counter -= step_back;
							assert(advance_pc_counter % NUM_BITS_TO_SIZE(XLEN) == 0);
							uint32_t const OP_correct_pc = RV_JAL(0, -(int)(step_back));
							struct scan_field const instr_pc_correct_fields[2] = {{.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT,.out_value = (uint8_t const*)(&OP_correct_pc)}, instr_scan_opcode_field};
							LOG_DEBUG("drscan %s %d 0x%08X %d 0x%1X", p_target->cmd_name,
								instr_pc_correct_fields[0].num_bits, buf_get_u32(instr_pc_correct_fields[0].out_value, 0, instr_pc_correct_fields[0].num_bits),
								instr_pc_correct_fields[1].num_bits, buf_get_u32(instr_pc_correct_fields[1].out_value, 0, instr_pc_correct_fields[1].num_bits));
							jtag_add_dr_scan_check(p_target->tap, TAP_NUM_FIELDS_DAP_CMD, instr_pc_correct_fields, TAP_IDLE);
						}
						assert(advance_pc_counter == 0);
					}
					error_code__update(p_target, jtag_execute_queue());
					if ( (DAP_OPSTATUS & DAP_OPSTATUS_MASK) != DAP_OPSTATUS_OK ) {
						LOG_ERROR("DAP_OPSTATUS == 0x%1X", (uint32_t)DAP_OPSTATUS);
						error_code__update(p_target, ERROR_TARGET_FAILURE);
					}
				} else {
					while ( (error_code__get(p_target) == ERROR_OK) && count-- ) {
						/// Set data to CSR
						sc_rv32_EXEC__push_data_to_CSR(p_target, buf_get_u32(buffer, 0, 8 * size));
						if ( error_code__get(p_target) != ERROR_OK ) {
							break;
						}

						for ( unsigned i = 0; i < ARRAY_LEN(instructions); ++i ) {
							(void)sc_rv32_EXEC__step(p_target, instructions[i]);
							if ( error_code__get(p_target) != ERROR_OK ) {
								break;
							}
							advance_pc_counter += instr_step;
						}
						buffer += size;
					}
					if ( !p_arch->use_pc_advmt_dsbl_bit ) {
						assert(advance_pc_counter % NUM_BITS_TO_SIZE(XLEN) == 0);
						while ( (error_code__get(p_target) == ERROR_OK) && (advance_pc_counter != 0) ) {
							uint32_t const step_back = advance_pc_counter > max_pc_offset ? max_pc_offset : advance_pc_counter;
							advance_pc_counter -= step_back;
							assert(advance_pc_counter % NUM_BITS_TO_SIZE(XLEN) == 0);
							uint32_t const OP_correct_pc = RV_JAL(0, -(int)(step_back));
							(void)sc_rv32_EXEC__step(p_target, OP_correct_pc);
						}
					}
				}
			}
		}
		if ( p_arch->use_pc_advmt_dsbl_bit ) {
			sc_rv32_HART_REGTRANS_write_and_check(p_target, DBGC_HART_REGS_DBG_CTRL, 0);
		}
	}

	if ( error_code__get(p_target) != ERROR_OK ) {
		LOG_DEBUG("update_status");
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
static	int sc_rv32i__add_breakpoint(struct target* const p_target, struct breakpoint* const p_breakpoint)
{
	assert(p_target);
	assert(p_breakpoint);
	if ( p_breakpoint->type != BKPT_SOFT ) {
		LOG_ERROR("Only software breakpoins available");
		error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
	} else {
		sc_rv32_check_that_target_halted(p_target);
		if ( ERROR_OK == error_code__get(p_target) ) {
			bool const RVC_enable = is_RVC_enable(p_target);
			if ( !(p_breakpoint->length == NUM_BITS_TO_SIZE(ILEN) || (RVC_enable && p_breakpoint->length == 2)) ) {
				LOG_ERROR("Invalid breakpoint size: %d", p_breakpoint->length);
				error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
			} else if ( p_breakpoint->address % (RVC_enable ? 2 : 4) != 0 ) {
				LOG_ERROR("Unaligned breakpoint: 0x%08X", p_breakpoint->address);
				error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
			} else {
				read_memory_space(p_target, p_breakpoint->address, 2, p_breakpoint->length / 2, p_breakpoint->orig_instr, true);
				if ( ERROR_OK != error_code__get(p_target) ) {
					LOG_ERROR("Can't save original instruction");
				} else {
					uint8_t buffer[4];
					if ( p_breakpoint->length == 4 ) {
						target_buffer_set_u32(p_target, buffer, RV_EBREAK());
					} else if ( p_breakpoint->length == 2 ) {
						target_buffer_set_u16(p_target, buffer, RV_C_EBREAK());
					} else {
						assert(/*logic_error:Bad breakpoint size*/ 0);
					}

					write_memory_space(p_target, p_breakpoint->address, 2, p_breakpoint->length / 2, buffer, true);
					if ( ERROR_OK != error_code__get(p_target) ) {
						LOG_ERROR("Can't write EBREAK");
					} else {
						p_breakpoint->set = 1;
					}
				}
			}
		}
	}
	return error_code__get_and_clear(p_target);
}
static int sc_rv32i__remove_breakpoint(struct target* const p_target, struct breakpoint* const p_breakpoint)
{
	assert(p_target);
	assert(p_breakpoint);
	if ( (p_breakpoint->length == 4 || p_breakpoint->length == 2) ) {
		error_code__update(p_target, ERROR_TARGET_INVALID);
	} else if ( 0 != p_breakpoint->address % 2 ) {
		error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
	} else if ( !(p_breakpoint->type == BKPT_SOFT && p_breakpoint->set) ) {
		error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
	} else {
		assert(p_target);
		sc_rv32_check_that_target_halted(p_target);
		if ( error_code__get(p_target) == ERROR_OK ) {
			assert(p_breakpoint->orig_instr);
			LOG_INFO("Remove breakpoint at 0x%08x, length=%d (0x%08x)", p_breakpoint->address, p_breakpoint->length, (p_breakpoint->length == 4 ? *(uint32_t const*)p_breakpoint->orig_instr : (uint32_t)(*(uint16_t const*)p_breakpoint->orig_instr)));
			write_memory_space(p_target, p_breakpoint->address, 2, p_breakpoint->length / 2, p_breakpoint->orig_instr, true);
			if ( ERROR_OK == error_code__get(p_target) ) {
				p_breakpoint->set = 0;
			}
		}
	}
	return error_code__get_and_clear(p_target);
}
static size_t total_number_of_regs(struct reg_cache const *p_reg_cache)
{
	size_t total = 0;
	for ( ; p_reg_cache; p_reg_cache = p_reg_cache->next ) {
		total += p_reg_cache->num_regs;
	}
	return total;
}
/// gdb_server expects valid reg values and will use set method for updating reg values
static int sc_rv32i__get_gdb_reg_list(struct target* const p_target, struct reg **reg_list[], int* const reg_list_size, enum target_register_class const reg_class)
{
	assert(p_target);
	assert(reg_list_size);
	assert(reg_class == REG_CLASS_ALL || reg_class == REG_CLASS_GENERAL);

	size_t const num_regs = reg_class == REG_CLASS_ALL ? total_number_of_regs(p_target->reg_cache) : NUMBER_OF_GP_REGS;
	struct reg** const p_reg_array = calloc(num_regs, sizeof(struct reg*));
	struct reg** p_reg_iter = p_reg_array;
	size_t regs_left = num_regs;
	for ( struct reg_cache *p_reg_cache = p_target->reg_cache; p_reg_cache && regs_left; p_reg_cache = p_reg_cache->next ) {
		struct reg *p_reg_list = p_reg_cache->reg_list;
		for ( size_t i = 0; i < p_reg_cache->num_regs && regs_left; ++i, --regs_left ) {
			*p_reg_iter++ = &p_reg_list[i];
		}
	}

	// out results
	*reg_list = p_reg_array;
	*reg_list_size = num_regs - regs_left;
	return error_code__get_and_clear(p_target);
}
/// @todo make const
struct target_type syntacore_riscv32i_target =
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

	.virt2phys = sc_rv32i__virt2phys,
	.read_phys_memory = sc_rv32i__read_phys_memory,
	.write_phys_memory = sc_rv32i__write_phys_memory,

	.mmu = sc_rv32i__mmu,
	.check_reset = NULL,
	.get_gdb_fileio_info = NULL,
	.gdb_fileio_end = NULL,
	.profiling = NULL,
};
