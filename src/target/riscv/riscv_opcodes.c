#include "riscv.h"
#include <assert.h>

#define LOW_BITS_MASK(n) (~(~0 << (n)))
#define MAKE_TYPE_FIELD(TYPE, bits, first_bit, last_bit)     ((((TYPE)(bits)) & LOW_BITS_MASK((last_bit) + 1u - (first_bit))) << (first_bit))

#define EXTRACT_FIELD(bits, first_bit, last_bit) (((bits) >> (first_bit)) & LOW_BITS_MASK((last_bit) + 1u - (first_bit)))
#define CHECK_REG(REG) assert(IS_VALID_UNSIFNED_FIELD(REG,5))
#define CHECK_OPCODE(OPCODE) assert(IS_VALID_UNSIFNED_FIELD(OPCODE,7) && (OPCODE & LOW_BITS_MASK(2)) == LOW_BITS_MASK(2) && (OPCODE & LOW_BITS_MASK(5)) != LOW_BITS_MASK(5))
#define CHECK_IMM_11_00(imm) assert(IS_VALID_SIGNED_IMMEDIATE_FIELD(imm, 11, 0));
#define CHECK_IMM_12_01(imm) assert(IS_VALID_SIGNED_IMMEDIATE_FIELD(imm, 12, 1));
#define CHECK_IMM_20_01(imm) assert(IS_VALID_SIGNED_IMMEDIATE_FIELD(imm, 20, 1));
#define CHECK_IMM_31_12(imm) assert(IS_VALID_SIGNED_IMMEDIATE_FIELD(imm, 31, 12));
#define CHECK_FUNC7(F) assert(IS_VALID_UNSIFNED_FIELD(F,7))
#define CHECK_FUNC3(F) assert(IS_VALID_UNSIFNED_FIELD(F,3))
#define NORMALIZE_INT_FIELD(FLD, SIGN_BIT, ZEROS) ( ( ( ( -( ( (FLD) >> (SIGN_BIT) ) & LOW_BITS_MASK(1) ) ) << (SIGN_BIT) ) | (FLD) ) & ~LOW_BITS_MASK(ZEROS) )
#define IS_VALID_SIGNED_IMMEDIATE_FIELD(FLD, SIGN_BIT, LOW_ZEROS) ( (FLD) == NORMALIZE_INT_FIELD((FLD), (SIGN_BIT), (LOW_ZEROS)) )
#define IS_VALID_UNSIFNED_FIELD(FLD,LEN) ((FLD & ~LOW_BITS_MASK(LEN)) == 0)

static riscv_short_signed_type csr_to_int(csr_num_type csr)
{
	return NORMALIZE_INT_FIELD(csr, 11, 0);
}

static uint32_t RV_INSTR_R_TYPE(unsigned func7, reg_num_type rs2, reg_num_type rs1, uint8_t func3, reg_num_type rd, uint8_t opcode)
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
static uint32_t RV_INSTR_I_TYPE(riscv_short_signed_type imm_11_00, reg_num_type rs1, uint8_t func3, reg_num_type rd, uint8_t opcode)
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
static uint32_t RV_INSTR_S_TYPE(riscv_short_signed_type imm_11_00, reg_num_type rs2, reg_num_type rs1, unsigned func3, uint8_t opcode)
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
#if 0
static uint32_t RV_INSTR_SB_TYPE(riscv_short_signed_type imm_01_12, reg_num_type rs2, reg_num_type rs1, unsigned func3, uint8_t opcode)
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
#endif
static uint32_t RV_INSTR_U_TYPE(riscv_signed_type imm_31_12, reg_num_type rd, uint8_t opcode)
{
	CHECK_OPCODE(opcode);
	CHECK_REG(rd);
	CHECK_IMM_31_12(imm_31_12);
	return
		MAKE_TYPE_FIELD(uint32_t, EXTRACT_FIELD(imm_31_12, 12, 31), 12, 31) |
		MAKE_TYPE_FIELD(uint32_t, rd, 7, 11) |
		MAKE_TYPE_FIELD(uint32_t, opcode, 0, 6);
}
static uint32_t RV_INSTR_UJ_TYPE(riscv_signed_type imm_20_01, reg_num_type rd, uint8_t opcode)
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

uint32_t RV_ADD(reg_num_type rd, reg_num_type rs1, reg_num_type rs2)
{
	return RV_INSTR_R_TYPE(0x00u, rs2, rs1, 0u, rd, 0x33u);
}
uint32_t RV_FMV_X_S(reg_num_type rd, reg_num_type rs1_fp)
{
	return RV_INSTR_R_TYPE(0x70u, 0u, rs1_fp, 0u, rd, 0x53u);
}
uint32_t RV_FMV_S_X(reg_num_type rd_fp, reg_num_type rs1)
{
	return RV_INSTR_R_TYPE(0x78u, 0u, rs1, 0u, rd_fp, 0x53u);
}
uint32_t RV_LB(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RV_INSTR_I_TYPE(imm, rs1, 0u, rd, 0x03u);
}
uint32_t RV_LH(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RV_INSTR_I_TYPE(imm, rs1, 1u, rd, 0x03u);
}
uint32_t RV_LW(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RV_INSTR_I_TYPE(imm, rs1, 2u, rd, 0x03u);
}
uint32_t RV_LBU(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RV_INSTR_I_TYPE(imm, rs1, 4u, rd, 0x03u);
}
uint32_t RV_LHU(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RV_INSTR_I_TYPE(imm, rs1, 5u, rd, 0x03u);
}
uint32_t RV_ADDI(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RV_INSTR_I_TYPE(imm, rs1, 0u, rd, 0x13u);
}
uint32_t RV_JALR(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RV_INSTR_I_TYPE(imm, rs1, 0u, rd, 0x67u);
}
uint32_t RV_CSRRW(reg_num_type rd, csr_num_type csr, reg_num_type rs1)
{
	return RV_INSTR_I_TYPE(csr_to_int(csr), rs1, 1u, rd, 0x73u);
}
uint32_t RV_CSRRS(reg_num_type rd, csr_num_type csr, reg_num_type rs1)
{
	return RV_INSTR_I_TYPE(csr_to_int(csr), rs1, 2u, rd, 0x73u);
}
uint32_t RV_CSRRC(reg_num_type rd, csr_num_type csr, reg_num_type rs1)
{
	return RV_INSTR_I_TYPE(csr_to_int(csr), rs1, 3u, rd, 0x73u);
}
uint32_t RV_CSRRWI(reg_num_type rd, csr_num_type csr, uint8_t zimm)
{
	return RV_INSTR_I_TYPE(csr_to_int(csr), zimm, 5u, rd, 0x73u);
}
uint32_t RV_CSRRSI(reg_num_type rd, csr_num_type csr, uint8_t zimm)
{
	return RV_INSTR_I_TYPE(csr_to_int(csr), zimm, 6u, rd, 0x73u);
}
uint32_t RV_CSRRCI(reg_num_type rd, csr_num_type csr, uint8_t zimm)
{
	return RV_INSTR_I_TYPE(csr_to_int(csr), zimm, 7u, rd, 0x73u);
}
uint32_t RV_EBREAK(void)
{
	return RV_INSTR_I_TYPE(1, 0u, 0u, 0u, 0x73u);
}
uint32_t RV_SB(reg_num_type rs_data, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RV_INSTR_S_TYPE(imm, rs_data, rs1, 0u, 0x23);
}
uint32_t RV_SH(reg_num_type rs, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RV_INSTR_S_TYPE(imm, rs, rs1, 1u, 0x23);
}
uint32_t RV_SW(reg_num_type rs, reg_num_type rs1, riscv_short_signed_type imm)
{
	return RV_INSTR_S_TYPE(imm, rs, rs1, 2u, 0x23);
}
uint32_t RV_AUIPC(reg_num_type rd, riscv_signed_type imm)
{
	return RV_INSTR_U_TYPE(imm, rd, 0x17u);
}
uint32_t RV_JAL(reg_num_type rd, riscv_signed_type imm_20_01)
{
	return RV_INSTR_UJ_TYPE(imm_20_01, rd, 0x6Fu);
}
uint32_t RV_NOP(void)
{
	return RV_ADDI(0, 0, 0u);
}
uint32_t RV_CSRW(unsigned csr, reg_num_type rs1)
{
	return RV_CSRRW(0, csr, rs1);
}
uint32_t RV_CSRR(reg_num_type rd, csr_num_type csr)
{
	return RV_CSRRS(rd, csr, 0);
}

uint16_t RV_C_EBREAK(void)
{
	return 0x9002u;
}
