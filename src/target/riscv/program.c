#include "program.h"

#include "opcodes.h"

#include "target/register.h"

/* Program interface. */
void
riscv_program_init(struct riscv_program *const p,
	struct target *const target)
{
	assert(p);
	memset(p, 0, sizeof *p);
	p->target = target;
	memset(p->debug_buffer, -1, sizeof p->debug_buffer);
	p->target_xlen = riscv_xlen(target);
}

static inline int
riscv_write_debug_buffer(struct target *const target,
	int const index,
	riscv_insn_t const insn)
{
	struct riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi && rvi->write_debug_buffer);
	return rvi->write_debug_buffer(target, index, insn);
}

int
riscv_program_write(struct riscv_program *const program)
{
	assert(program && program->target);

	for (unsigned i = 0; i < program->instruction_count; ++i) {
		LOG_DEBUG("%s: %p: debug_buffer[%02x] = DASM(0x%08" PRIx32 ")",
			program->target->cmd_name,
			program,
			i,
			program->debug_buffer[i]);

		int const result =
			riscv_write_debug_buffer(program->target, i, program->debug_buffer[i]);

		if (ERROR_OK != result)
			return result;
	}

	return ERROR_OK;
}

static inline int
riscv_execute_debug_buffer(struct target *const target)
{
	struct riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi && rvi->execute_debug_buffer);
	return rvi->execute_debug_buffer(target);
}

static inline riscv_insn_t
riscv_read_debug_buffer(struct target *const target,
	int const index)
{
	struct riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi && rvi->read_debug_buffer);
	return rvi->read_debug_buffer(target, index);
}

static size_t
riscv_debug_buffer_size(struct target *const target)
{
	struct riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi);
	int const hart = riscv_current_hartid(target);
	assert(0<= hart && hart < RISCV_MAX_HARTS);
	return rvi->harts[hart].debug_buffer_size;
}

/** Add ebreak and execute the program. */
int
riscv_program_exec(struct riscv_program *const p,
	struct target *const t)
{
	keep_alive();

	riscv_reg_t saved_registers[GDB_REGNO_XPR31 + 1];

	for (size_t i = GDB_REGNO_ZERO + 1; i <= GDB_REGNO_XPR31; ++i) {
		if (p->writes_xreg[i]) {
			LOG_DEBUG("%s: Saving register %d as used by program", t->cmd_name, (int)(i));
			int const result = riscv_get_register(t, &saved_registers[i], i);

			if (ERROR_OK != result)
				return result;
		}
	}

	{
		int const result = riscv_program_ebreak(p);

		if (ERROR_OK != result) {
			LOG_ERROR("%s: Unable to write ebreak", t->cmd_name);
			for (size_t i = 0; i < riscv_debug_buffer_size(p->target); ++i)
				LOG_ERROR("%s: ram[%02" PRIxPTR "]: DASM(0x%08" PRIx32 ") [0x%08" PRIx32 "]",
					t->cmd_name,
					i,
					p->debug_buffer[i],
					p->debug_buffer[i]);
			return result;
		}

	}

	{
		int const result = riscv_program_write(p);

		if (ERROR_OK != result)
			return result;
	}

	{
		int const result = riscv_execute_debug_buffer(t);

		if (ERROR_OK != result) {
			LOG_DEBUG("%s: Unable to execute program %p", t->cmd_name, p);
			return result;
		}
	}

	for (size_t i = 0; i < riscv_debug_buffer_size(p->target); ++i)
		if (i >= riscv_debug_buffer_size(p->target))
			p->debug_buffer[i] = riscv_read_debug_buffer(t, i);

	for (size_t i = GDB_REGNO_ZERO; i <= GDB_REGNO_XPR31; ++i)
		if (p->writes_xreg[i])
			riscv_set_register(t, i, saved_registers[i]);

	return ERROR_OK;
}

int
riscv_program_swr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, sw(d, b, offset));
}

int riscv_program_shr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, sh(d, b, offset));
}

int riscv_program_sbr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, sb(d, b, offset));
}

int riscv_program_lwr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, lw(d, b, offset));
}

int riscv_program_lhr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, lh(d, b, offset));
}

int riscv_program_lbr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, lb(d, b, offset));
}

int riscv_program_csrr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno csr)
{
	assert(csr >= GDB_REGNO_CSR0 && csr <= GDB_REGNO_CSR4095);
	return riscv_program_insert(p, csrrs(d, GDB_REGNO_ZERO, csr - GDB_REGNO_CSR0));
}

int riscv_program_csrw(struct riscv_program *p, enum gdb_regno s, enum gdb_regno csr)
{
	assert(csr >= GDB_REGNO_CSR0);
	return riscv_program_insert(p, csrrw(GDB_REGNO_ZERO, s, csr - GDB_REGNO_CSR0));
}

int
riscv_program_fence_i(struct riscv_program *p)
{
	return riscv_program_insert(p, fence_i());
}

int
riscv_program_fence(struct riscv_program *p)
{
	return riscv_program_insert(p, fence());
}

int
riscv_program_ebreak(struct riscv_program *p)
{
	struct target *const target = p->target;
	struct riscv_info_t *const rvi = riscv_info(target);
	assert(rvi);

	if (riscv_debug_buffer_size(p->target) == p->instruction_count && rvi->impebreak)
		return ERROR_OK;

	return riscv_program_insert(p, ebreak());
}

int
riscv_program_addi(struct riscv_program *p,
	enum gdb_regno d,
	enum gdb_regno s,
	int16_t u)
{
	return riscv_program_insert(p, addi(d, s, u));
}

int
riscv_program_insert(struct riscv_program *p, riscv_insn_t i)
{
	assert(p && p->target);
	if (p->instruction_count >= riscv_debug_buffer_size(p->target)) {
		LOG_ERROR("%s: Unable to insert instruction:"
				"  instruction_count=%d"
				"  buffer size      =%d",
				p->target->cmd_name,
				(int)p->instruction_count,
				(int)riscv_debug_buffer_size(p->target)
		);

		return ERROR_BUF_TOO_SMALL;
	}

	p->debug_buffer[p->instruction_count] = i;
	++p->instruction_count;
	return ERROR_OK;
}
