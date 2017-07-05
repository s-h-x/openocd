/** @file

Syntacore RISC-V target

@copyright Syntacore
*/
#ifndef RISCV_I_H_
#define RISCV_I_H_

#include <stdint.h>

typedef uint8_t reg_num_type;
typedef int32_t riscv_signed_type;
typedef int16_t riscv_short_signed_type;
typedef uint16_t csr_num_type;

enum e_RISCV_CSRs
{
    CSR_fflags = 0x1,
    CSR_frm = 0x2,
    CSR_fcsr = 0x3,
    CSR_cycle = 0xc00,
    CSR_time = 0xc01,
    CSR_instret = 0xc02,
    CSR_stats = 0xc0,
    CSR_uarch0 = 0xcc0,
    CSR_uarch1 = 0xcc1,
    CSR_uarch2 = 0xcc2,
    CSR_uarch3 = 0xcc3,
    CSR_uarch4 = 0xcc4,
    CSR_uarch5 = 0xcc5,
    CSR_uarch6 = 0xcc6,
    CSR_uarch7 = 0xcc7,
    CSR_uarch8 = 0xcc8,
    CSR_uarch9 = 0xcc9,
    CSR_uarch10 = 0xcca,
    CSR_uarch11 = 0xccb,
    CSR_uarch12 = 0xccc,
    CSR_uarch13 = 0xccd,
    CSR_uarch14 = 0xcce,
    CSR_uarch15 = 0xccf,
    CSR_sstatus = 0x100,
    CSR_stvec = 0x101,
    CSR_sie = 0x104,
    CSR_sscratch = 0x140,
    CSR_sepc = 0x141,
    CSR_sip = 0x144,
    CSR_sptbr = 0x180,
    CSR_sasid = 0x181,
    CSR_cyclew = 0x900,
    CSR_timew = 0x901,
    CSR_instretw = 0x902,
    CSR_stime = 0xd01,
    CSR_scause = 0xd42,
    CSR_sbadaddr = 0xd43,
    CSR_stimew = 0xa01,
    CSR_mstatus = 0x300,
    CSR_mtvec = 0x301,
    CSR_mtdeleg = 0x302,
    CSR_mie = 0x304,
    CSR_mtimecmp = 0x321,
    CSR_mscratch = 0x340,
    CSR_mepc = 0x341,
    CSR_mcause = 0x342,
    CSR_mbadaddr = 0x343,
    CSR_mip = 0x344,
    CSR_mtime = 0x701,
    CSR_mcpuid = 0xf00,
    CSR_mimpid = 0xf01,
    CSR_mhartid = 0xf10,
    CSR_mtohost = 0x780,
    CSR_mfromhost = 0x781,
    CSR_mreset = 0x782,
    CSR_mipi = 0x783,
    CSR_miobase = 0x784,
    CSR_cycleh = 0xc80,
    CSR_timeh = 0xc81,
    CSR_instreth = 0xc82,
    CSR_cyclehw = 0x980,
    CSR_timehw = 0x981,
    CSR_instrethw = 0x982,
    CSR_stimeh = 0xd81,
    CSR_stimehw = 0xa81,
    CSR_mtimecmph = 0x361,
    CSR_mtimeh = 0x741,

    /// Supervisor Trap Setup
    ///@{
    /// Wall-clock timer compare val
    CSR_stimecmp = 0x121u,
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

    /// Debug controller CSR
    /// privilege: MRW
    CSR_sc_dbg_scratch = 0x788u,
};

enum e_CSR_mstatus_context_field
{
    ext_off = 0,
    ext_initial = 1,
    ext_clean = 2,
    ext_dirty = 3,
};
enum e_RISCV_privilege_levels
{
    Priv_U = 0x0,
    Priv_S = 0x1,
    Priv_H = 0x2,
    Priv_M = 0x3,
};

enum e_VM_modes
{
    VM_Mbare = 0,
    VM_Mbb = 1,
    VM_Mbbid = 2,
    VM_Sv32 = 8,
    VM_Sv39 = 9,
    VM_Sv48 = 10,
};

/// RISC-V opcodes
///@{

#if 0
uint32_t RISCV_opcode_ADD(reg_num_type rd, reg_num_type rs1, reg_num_type rs2);
#endif
uint32_t RISCV_opcode_FMV_X_S(reg_num_type rd, reg_num_type rs1_fp);
uint32_t RISCV_opcode_FMV_S_X(reg_num_type rd_fp, reg_num_type rs1);
uint32_t RISCV_opcode_LB(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RISCV_opcode_LH(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RISCV_opcode_LW(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm);
#if 0
uint32_t RISCV_opcode_LBU(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RISCV_opcode_LHU(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm);
#endif
uint32_t RISCV_opcode_ADDI(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RISCV_opcode_JALR(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RISCV_opcode_CSRRW(reg_num_type rd, csr_num_type csr, reg_num_type rs1);
uint32_t RISCV_opcode_CSRRS(reg_num_type rd, csr_num_type csr, reg_num_type rs1);
uint32_t RISCV_opcode_CSRRC(reg_num_type rd, csr_num_type csr, reg_num_type rs1);
uint32_t RISCV_opcode_CSRRWI(reg_num_type rd, csr_num_type csr, uint8_t zimm);
uint32_t RISCV_opcode_CSRRSI(reg_num_type rd, csr_num_type csr, uint8_t zimm);
uint32_t RISCV_opcode_CSRRCI(reg_num_type rd, csr_num_type csr, uint8_t zimm);
uint32_t RISCV_opcode_EBREAK(void);
uint16_t RISCV_opcode_C_EBREAK(void);
uint32_t RISCV_opcode_SB(reg_num_type rs_data, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RISCV_opcode_SH(reg_num_type rs, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RISCV_opcode_SW(reg_num_type rs, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RISCV_opcode_AUIPC(reg_num_type rd, riscv_signed_type imm);
uint32_t RISCV_opcode_JAL(reg_num_type rd, riscv_signed_type imm_20_01);
uint32_t RISCV_opcode_NOP(void);
uint32_t RISCV_opcode_CSRW(unsigned csr, reg_num_type rs1);
uint32_t RISCV_opcode_CSRR(reg_num_type rd, csr_num_type csr);

uint32_t RISCV_opcode_FMV_2X_D(reg_num_type rd_hi, reg_num_type rd_lo, reg_num_type rs1_fp);
uint32_t RISCV_opcode_FMV_D_2X(reg_num_type rd_fp, reg_num_type rs_hi, reg_num_type rs_lo);
///@]

#endif  // RISCV_I_H_
