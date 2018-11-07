#ifndef TARGET__RISCV__ASM_H
#define TARGET__RISCV__ASM_H

#include "riscv.h"

#include <assert.h>

/** Version-independent functions that we don't want in the main address space.
	@{
*/
static uint32_t __attribute__((unused))
load(const struct target *target, unsigned rd, unsigned base, uint16_t offset)
{
	switch (riscv_xlen(target)) {
	case 32:
		return lw(rd, base, offset);

	case 64:
		return ld(rd, base, offset);

		/** @bug no default case*/
	}

	assert(0);
	return 0; /*< Silence -Werror=return-type */
}

static uint32_t __attribute__((unused))
store(const struct target *target, unsigned src, unsigned base, uint16_t offset)
{
	switch (riscv_xlen(target)) {
	case 32:
		return sw(src, base, offset);

	case 64:
		return sd(src, base, offset);

		/** @bug no default case*/
	}

	assert(0);
	return 0; /*< Silence -Werror=return-type */
}
/** @} */

#endif
