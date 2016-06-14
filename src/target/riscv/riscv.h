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
#define CSR_FFLAGS 0x1
#define CSR_FRM 0x2
#define CSR_FCSR 0x3
#define CSR_CYCLE 0xc00
#define CSR_TIME 0xc01
#define CSR_INSTRET 0xc02
#define CSR_STATS 0xc0
#define CSR_UARCH0 0xcc0
#define CSR_UARCH1 0xcc1
#define CSR_UARCH2 0xcc2
#define CSR_UARCH3 0xcc3
#define CSR_UARCH4 0xcc4
#define CSR_UARCH5 0xcc5
#define CSR_UARCH6 0xcc6
#define CSR_UARCH7 0xcc7
#define CSR_UARCH8 0xcc8
#define CSR_UARCH9 0xcc9
#define CSR_UARCH10 0xcca
#define CSR_UARCH11 0xccb
#define CSR_UARCH12 0xccc
#define CSR_UARCH13 0xccd
#define CSR_UARCH14 0xcce
#define CSR_UARCH15 0xccf
#define CSR_SSTATUS 0x100
#define CSR_STVEC 0x101
#define CSR_SIE 0x104
#define CSR_SSCRATCH 0x140
#define CSR_SEPC 0x141
#define CSR_SIP 0x144
#define CSR_SPTBR 0x180
#define CSR_SASID 0x181
#define CSR_CYCLEW 0x900
#define CSR_TIMEW 0x901
#define CSR_INSTRETW 0x902
#define CSR_STIME 0xd01
#define CSR_SCAUSE 0xd42
#define CSR_SBADADDR 0xd43
#define CSR_STIMEW 0xa01
#define CSR_MSTATUS 0x300
#define CSR_MTVEC 0x301
#define CSR_MTDELEG 0x302
#define CSR_MIE 0x304
#define CSR_MTIMECMP 0x321
#define CSR_MSCRATCH 0x340
#define CSR_MEPC 0x341
#define CSR_MCAUSE 0x342
#define CSR_MBADADDR 0x343
#define CSR_MIP 0x344
#define CSR_MTIME 0x701
#define CSR_MCPUID 0xf00
#define CSR_MIMPID 0xf01
#define CSR_MHARTID 0xf10
#define CSR_MTOHOST 0x780
#define CSR_MFROMHOST 0x781
#define CSR_MRESET 0x782
#define CSR_MIPI 0x783
#define CSR_MIOBASE 0x784
#define CSR_CYCLEH 0xc80
#define CSR_TIMEH 0xc81
#define CSR_INSTRETH 0xc82
#define CSR_CYCLEHW 0x980
#define CSR_TIMEHW 0x981
#define CSR_INSTRETHW 0x982
#define CSR_STIMEH 0xd81
#define CSR_STIMEHW 0xa81
#define CSR_MTIMECMPH 0x361
#define CSR_MTIMEH 0x741

#define DECLARE_CSR(NAME,VALUE) CSR_##NAME = VALUE,
enum RISCV_CSR
{
DECLARE_CSR(fflags, CSR_FFLAGS)
DECLARE_CSR(frm, CSR_FRM)
DECLARE_CSR(fcsr, CSR_FCSR)
DECLARE_CSR(cycle, CSR_CYCLE)
DECLARE_CSR(time, CSR_TIME)
DECLARE_CSR(instret, CSR_INSTRET)
DECLARE_CSR(stats, CSR_STATS)
DECLARE_CSR(uarch0, CSR_UARCH0)
DECLARE_CSR(uarch1, CSR_UARCH1)
DECLARE_CSR(uarch2, CSR_UARCH2)
DECLARE_CSR(uarch3, CSR_UARCH3)
DECLARE_CSR(uarch4, CSR_UARCH4)
DECLARE_CSR(uarch5, CSR_UARCH5)
DECLARE_CSR(uarch6, CSR_UARCH6)
DECLARE_CSR(uarch7, CSR_UARCH7)
DECLARE_CSR(uarch8, CSR_UARCH8)
DECLARE_CSR(uarch9, CSR_UARCH9)
DECLARE_CSR(uarch10, CSR_UARCH10)
DECLARE_CSR(uarch11, CSR_UARCH11)
DECLARE_CSR(uarch12, CSR_UARCH12)
DECLARE_CSR(uarch13, CSR_UARCH13)
DECLARE_CSR(uarch14, CSR_UARCH14)
DECLARE_CSR(uarch15, CSR_UARCH15)
DECLARE_CSR(sstatus, CSR_SSTATUS)
DECLARE_CSR(stvec, CSR_STVEC)
DECLARE_CSR(sie, CSR_SIE)
DECLARE_CSR(sscratch, CSR_SSCRATCH)
DECLARE_CSR(sepc, CSR_SEPC)
DECLARE_CSR(sip, CSR_SIP)
DECLARE_CSR(sptbr, CSR_SPTBR)
DECLARE_CSR(sasid, CSR_SASID)
DECLARE_CSR(cyclew, CSR_CYCLEW)
DECLARE_CSR(timew, CSR_TIMEW)
DECLARE_CSR(instretw, CSR_INSTRETW)
DECLARE_CSR(stime, CSR_STIME)
DECLARE_CSR(scause, CSR_SCAUSE)
DECLARE_CSR(sbadaddr, CSR_SBADADDR)
DECLARE_CSR(stimew, CSR_STIMEW)
DECLARE_CSR(mstatus, CSR_MSTATUS)
DECLARE_CSR(mtvec, CSR_MTVEC)
DECLARE_CSR(mtdeleg, CSR_MTDELEG)
DECLARE_CSR(mie, CSR_MIE)
DECLARE_CSR(mtimecmp, CSR_MTIMECMP)
DECLARE_CSR(mscratch, CSR_MSCRATCH)
DECLARE_CSR(mepc, CSR_MEPC)
DECLARE_CSR(mcause, CSR_MCAUSE)
DECLARE_CSR(mbadaddr, CSR_MBADADDR)
DECLARE_CSR(mip, CSR_MIP)
DECLARE_CSR(mtime, CSR_MTIME)
DECLARE_CSR(mcpuid, CSR_MCPUID)
DECLARE_CSR(mimpid, CSR_MIMPID)
DECLARE_CSR(mhartid, CSR_MHARTID)
DECLARE_CSR(mtohost, CSR_MTOHOST)
DECLARE_CSR(mfromhost, CSR_MFROMHOST)
DECLARE_CSR(mreset, CSR_MRESET)
DECLARE_CSR(mipi, CSR_MIPI)
DECLARE_CSR(miobase, CSR_MIOBASE)
DECLARE_CSR(cycleh, CSR_CYCLEH)
DECLARE_CSR(timeh, CSR_TIMEH)
DECLARE_CSR(instreth, CSR_INSTRETH)
DECLARE_CSR(cyclehw, CSR_CYCLEHW)
DECLARE_CSR(timehw, CSR_TIMEHW)
DECLARE_CSR(instrethw, CSR_INSTRETHW)
DECLARE_CSR(stimeh, CSR_STIMEH)
DECLARE_CSR(stimehw, CSR_STIMEHW)
DECLARE_CSR(mtimecmph, CSR_MTIMECMPH)
DECLARE_CSR(mtimeh, CSR_MTIMEH)

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
CSR_SC_DBG_SCRATCH = 0x788u,
};
#undef DECLARE_CSR
enum mstatus_context_field_e
{
	ext_off = 0,
	ext_initial = 1,
	ext_clean = 2,
	ext_dirty = 3,
};
enum Privilege
{
	Priv_U = 0x0,
	Priv_S = 0x1,
	Priv_H = 0x2,
	Priv_M = 0x3,
};

enum VM_mode
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

uint32_t RV_ADD(reg_num_type rd, reg_num_type rs1, reg_num_type rs2);
uint32_t RV_FMV_X_S(reg_num_type rd, reg_num_type rs1_fp);
uint32_t RV_FMV_S_X(reg_num_type rd_fp, reg_num_type rs1);
uint32_t RV_LB(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RV_LH(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RV_LW(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RV_LBU(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RV_LHU(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RV_ADDI(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RV_JALR(reg_num_type rd, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RV_CSRRW(reg_num_type rd, csr_num_type csr, reg_num_type rs1);
uint32_t RV_CSRRS(reg_num_type rd, csr_num_type csr, reg_num_type rs1);
uint32_t RV_CSRRC(reg_num_type rd, csr_num_type csr, reg_num_type rs1);
uint32_t RV_CSRRWI(reg_num_type rd, csr_num_type csr, uint8_t zimm);
uint32_t RV_CSRRSI(reg_num_type rd, csr_num_type csr, uint8_t zimm);
uint32_t RV_CSRRCI(reg_num_type rd, csr_num_type csr, uint8_t zimm);
uint32_t RV_EBREAK(void);
uint16_t RV_C_EBREAK(void);
uint32_t RV_SB(reg_num_type rs_data, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RV_SH(reg_num_type rs, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RV_SW(reg_num_type rs, reg_num_type rs1, riscv_short_signed_type imm);
uint32_t RV_AUIPC(reg_num_type rd, riscv_signed_type imm);
uint32_t RV_JAL(reg_num_type rd, riscv_signed_type imm_20_01);
uint32_t RV_NOP(void);
uint32_t RV_CSRW(unsigned csr, reg_num_type rs1);
uint32_t RV_CSRR(reg_num_type rd, csr_num_type csr);

uint32_t RV_FMV_X2_S(reg_num_type rd_hi, reg_num_type rd_lo, reg_num_type rs1_fp);
uint32_t RV_FMV_S_X2(reg_num_type rd_fp, reg_num_type rs_lo, reg_num_type rs_hi);
///@]

#endif  // RISCV_I_H_
