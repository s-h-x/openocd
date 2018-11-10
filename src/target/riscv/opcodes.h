#ifndef TARGET_RISCV_OPCODES_H_
#define TARGET_RISCV_OPCODES_H_

#include "encoding.h"
#include <stdint.h>

#define ZERO	0
#define T0      5
#define S0      8
#define S1      9

static inline uint32_t bits(uint32_t value, unsigned hi, unsigned lo)
{
	return (value >> lo) & ((1 << (hi + 1 - lo)) - 1);
}

static inline uint32_t bit(uint32_t value, unsigned b)
{
	return (value >> b) & 1;
}

static inline uint32_t jal(unsigned rd, uint32_t imm)
{
	return
		(bit(imm, 20) << 31) |
		(bits(imm, 10, 1) << 21) |
		(bit(imm, 11) << 20) |
		(bits(imm, 19, 12) << 12) |
		(rd << 7) |
		MATCH_JAL;
}

static inline uint32_t csrsi(unsigned csr, uint16_t imm)
{
	return
		(csr << 20) |
		(bits(imm, 4, 0) << 15) |
		MATCH_CSRRSI;
}

static inline uint32_t sw(unsigned src, unsigned base, uint16_t offset)
{
	return
		(bits(offset, 11, 5) << 25) |
		(src << 20) |
		(base << 15) |
		(bits(offset, 4, 0) << 7) |
		MATCH_SW;
}

static inline uint32_t sd(unsigned src, unsigned base, uint16_t offset)
{
	return
		(bits(offset, 11, 5) << 25) |
		(src << 20) |
		(base << 15) |
		(bits(offset, 4, 0) << 7) |
		MATCH_SD;
}

static inline uint32_t sh(unsigned src, unsigned base, uint16_t offset)
{
	return
		(bits(offset, 11, 5) << 25) |
		(src << 20) |
		(base << 15) |
		(bits(offset, 4, 0) << 7) |
		MATCH_SH;
}

static inline uint32_t sb(unsigned src, unsigned base, uint16_t offset)
{
	return
		(bits(offset, 11, 5) << 25) |
		(src << 20) |
		(base << 15) |
		(bits(offset, 4, 0) << 7) |
		MATCH_SB;
}

static inline uint32_t ld(unsigned rd, unsigned base, uint16_t offset)
{
	return
		(bits(offset, 11, 0) << 20) |
		(base << 15) |
		(bits(rd, 4, 0) << 7) |
		MATCH_LD;
}

static inline uint32_t lw(unsigned rd, unsigned base, uint16_t offset)
{
	return
		(bits(offset, 11, 0) << 20) |
		(base << 15) |
		(bits(rd, 4, 0) << 7) |
		MATCH_LW;
}

static inline uint32_t lh(unsigned rd, unsigned base, uint16_t offset)
{
	return
		(bits(offset, 11, 0) << 20) |
		(base << 15) |
		(bits(rd, 4, 0) << 7) |
		MATCH_LH;
}

static inline uint32_t lb(unsigned rd, unsigned base, uint16_t offset)
{
	return
		(bits(offset, 11, 0) << 20) |
		(base << 15) |
		(bits(rd, 4, 0) << 7) |
		MATCH_LB;
}

static inline uint32_t csrw(unsigned source, unsigned csr)
{
	return
		(csr << 20) |
		(source << 15) |
		MATCH_CSRRW;
}

static inline uint32_t addi(unsigned dest, unsigned src, uint16_t imm)
{
	return
		(bits(imm, 11, 0) << 20) |
		(src << 15) |
		(dest << 7) |
		MATCH_ADDI;
}

static inline uint32_t csrr(unsigned rd, unsigned csr)
{
	return (csr << 20) | (rd << 7) | MATCH_CSRRS;
}

static inline uint32_t csrrs(unsigned rd, unsigned rs, unsigned csr)
{
	return (csr << 20) | (rs << 15) | (rd << 7) | MATCH_CSRRS;
}

static inline uint32_t csrrw(unsigned rd, unsigned rs, unsigned csr)
{
	return (csr << 20) | (rs << 15) | (rd << 7) | MATCH_CSRRW;
}

static inline uint32_t fsw(unsigned src, unsigned base, uint16_t offset)
{
	return
		(bits(offset, 11, 5) << 25) |
		(bits(src, 4, 0) << 20) |
		(base << 15) |
		(bits(offset, 4, 0) << 7) |
		MATCH_FSW;
}

static inline uint32_t fsd(unsigned src, unsigned base, uint16_t offset)
{
	return
		(bits(offset, 11, 5) << 25) |
		(bits(src, 4, 0) << 20) |
		(base << 15) |
		(bits(offset, 4, 0) << 7) |
		MATCH_FSD;
}

static inline uint32_t flw(unsigned dest, unsigned base, uint16_t offset)
{
	return
		(bits(offset, 11, 0) << 20) |
		(base << 15) |
		(bits(dest, 4, 0) << 7) |
		MATCH_FLW;
}

static inline uint32_t fld(unsigned dest, unsigned base, uint16_t offset)
{
	return (bits(offset, 11, 0) << 20) |
		(base << 15) |
		(bits(dest, 4, 0) << 7) |
		MATCH_FLD;
}

static inline uint32_t fmv_x_w(unsigned dest, unsigned src)
{
	return src << 15 |
		dest << 7 |
		MATCH_FMV_X_W;
}

static inline uint32_t fmv_x_d(unsigned dest, unsigned src)
{
	return src << 15 |
		dest << 7 |
		MATCH_FMV_X_D;
}

static inline uint32_t fmv_w_x(unsigned dest, unsigned src)
{
	return src << 15 |
		dest << 7 |
		MATCH_FMV_W_X;
}

static inline uint32_t fmv_d_x(unsigned dest, unsigned src)
{
	return
		src << 15 |
		dest << 7 |
		MATCH_FMV_D_X;
}

static inline uint32_t ebreak(void)
{
	return MATCH_EBREAK;
}
static inline uint32_t ebreak_c(void)
{
	return MATCH_C_EBREAK;
}

static inline uint32_t wfi(void)
{
	return MATCH_WFI;
}

static inline uint32_t fence_i(void)
{
	return MATCH_FENCE_I;
}

static inline uint32_t lui(unsigned dest, uint32_t imm)
{
	return
		(bits(imm, 19, 0) << 12) |
		(dest << 7) |
		MATCH_LUI;
}

static inline uint32_t xori(unsigned dest, unsigned src, uint16_t imm)
{
	return
		(bits(imm, 11, 0) << 20) |
		(src << 15) |
		(dest << 7) |
		MATCH_XORI;
}

static inline uint32_t srli(unsigned dest, unsigned src, uint8_t shamt)
{
	return
		(bits(shamt, 4, 0) << 20) |
		(src << 15) |
		(dest << 7) |
		MATCH_SRLI;
}

static inline uint32_t fence(void)
{
	return MATCH_FENCE;
}

static inline uint32_t auipc(unsigned dest)
{
	return MATCH_AUIPC | (dest << 7);
}

#endif  /* TARGET_RISCV_OPCODES_H_ */
