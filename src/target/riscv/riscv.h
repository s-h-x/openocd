/**
@file

Syntacore RISC-V target

@author Pavel S. Smirnov
@copyright Syntacore
*/
#ifndef RISCV_I_H_
#define RISCV_I_H_

#include <stdint.h>

typedef uint8_t reg_num_type;
typedef int32_t riscv_signed_type;
typedef int16_t riscv_short_signed_type;
typedef uint16_t csr_num_type;

enum RISCV_CSR
{
	/// User Floating-Point CSRs
	/// privilege: URW
	/// @addtogroup floating_point_csrs
	///@{
	/// Floating-Point Accrued Exceptions.
	CSR_fflags = 0x001u,
	/// Floating-Point Dynamic Rounding Mode.
	CSR_frm = 0x002u,
	/// Floating-Point Control and Status Register (frm + fflags).
	CSR_fcsr = 0x003u,
	///@}

	/// User Counter/Timers
	/// privilege: URO
	///@{
	/// Cycle counter for RDCYCLE instruction
	CSR_cycle = 0xC00u,
	/// Timer for RDTIME instruction
	CSR_time = 0xC01u,
	/// Instructions-retired counter for RDINSTRET instruction
	CSR_instret = 0xC02u,
	/// Upper 32 bits of cycle, RV32I onl
	CSR_cycleh = 0xC80u,
	/// Upper 32 bits of time, RV32I only
	CSR_timeh = 0xC81u,
	///  Upper 32 bits of instret, RV32I only.
	CSR_instreth = 0xC82u,
	///@}

	/// Supervisor Trap Setup
	///@{
	/// Supervisor status register
	CSR_sstatus = 0x100u,
	/// Supervisor trap handler base address
	CSR_stvec = 0x101u,
	/// Supervisor interrupt-enable register
	CSR_sie = 0x104u,
	/// Wall-clock timer compare val
	CSR_stimecmp = 0x121u,
	///@}

	/// Supervisor Timer
	///@{
	/// Supervisor wall-clock time register.
	CSR_stime = 0xD01u,
	/// Upper 32 bits of stime, RV32I onl
	CSR_stimeh = 0xD81u,
	///@}

	/// Supervisor Trap Handling
	///@{
	/// Scratch register for supervisor trap handlers
	CSR_sscratch = 0x140u,
	/// Supervisor exception program counter.
	CSR_sepc = 0x141u,
	/// Supervisor trap cause.
	CSR_scause = 0xD42u,
	/// Supervisor bad address.
	CSR_sbadaddr = 0xD43u,
	/// Supervisor interrupt pending
	CSR_sip = 0x144u,
	///@}

	/// Supervisor Protection and Translation
	///@{
	/// Page-table base register
	CSR_sptbr = 0x180u,
	/// Address-space ID.
	CSR_sasid = 0x181u,
	///@}

	/// Supervisor Read/Write Shadow of User Read-Only registers
	///@{
	/// Cycle counter for RDCYCLE instruction.
	CSR_cyclew = 0x900u,
	/// Timer for RDTIME instruction.
	CSR_timew = 0x901u,
	/// Instructions-retired counter for RDINSTRET instruction
	CSR_instretw = 0x902u,
	/// Upper 32 bits of cycle, RV32I only
	CSR_cyclehw = 0x980u,
	/// Upper 32 bits of time, RV32I only
	CSR_timehw = 0x981u,
	/// Upper 32 bits of CSR_instretw, RV32I only
	CSR_instrethw = 0x982u,
	///@}

	/// Hypervisor Trap Setup
	///@{
	/// Hypervisor status register
	CSR_hstatus = 0x200u,
	/// Hypervisor trap handler base address
	CSR_htvec = 0x201u,
	/// Hypervisor trap delegation register.
	CSR_htdeleg = 0x202u,
	/// Hypervisor wall-clock timer compare value.
	CSR_htimecmp = 0x221u,
	///@}

	/// Hypervisor Timer
	///@{
	CSR_htime = 0xE01u,
	CSR_htimeh = 0xE81u,
	///@}

	/// Hypervisor Trap Handling
	///@{
	CSR_hscratch = 0x240u,
	CSR_hepc = 0x241u,
	CSR_hcause = 0x242u,
	CSR_hbadaddr = 0x243u,
	///@}

	/// Hypervisor Read/Write Shadow of Supervisor Read-Only Registers
	///@{
	CSR_stimew = 0xA01u,
	CSR_stimehw = 0xA81u,
	///@}

	/// Machine Information Registers
	/// privilege: MRO
	///@{
	/// @brief CPU description
	CSR_mcpuid = 0xF00u,

	/// @brief Vendor ID and version number
	CSR_mimpid = 0xF01u,

	/// @brief Hardware thread ID
	CSR_mhartid = 0xF10u,
	///@}

	/// Machine Trap Setup
	/// privilege: MRW
	///@{

	/// @brief Machine status register
	///< FS - bits 13:12
	CSR_mstatus = 0x300u,

	/// @brief Machine trap-handler base address
	CSR_mtvec = 0x301u,

	/// @brief Machine trap delegation register
	CSR_mtdeleg = 0x302u,

	/// @brief Machine interrupt-enable register
	CSR_mie = 0x304u,

	/// @brief Machine wall-clock timer compare value
	CSR_mtimecmp = 0x321u,
	///@}

	/// Machine Timers and Counters
	/// privilege: MRW
	///@{

	/// @brief Machine wall-clock time
	CSR_mtime = 0x701u,

	/// @brief Upper 32 bits of mtime, RV32I only
	CSR_mtimeh = 0x741u,
	///@}

	/// Machine Trap Handling
	/// privilege: MRW
	///@{

	/// @brief Scratch register for machine trap handlers.
	CSR_mscratch = 0x340u,
	/// @brief Machine exception program counter
	CSR_mepc = 0x341u,
	/// @brief Machine trap cause
	CSR_mcause = 0x342u,
	/// @brief Machine bad address
	CSR_mbadaddr = 0x343u,
	/// @brief Machine interrupt pending.
	CSR_mip = 0x344u,
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

	/// Machine Read-Write Shadow of Hypervisor Read-Only Registers
	/// privilege: MRW
	///@{

	/// @brief Hypervisor wall-clock timer
	CSR_htimew = 0xB01u,

	/// @brief Upper 32 bits of hypervisor wall-clock timer, RV32I only.
	CSR_htimehw = 0xB81u,
	///@}

	/// Machine Host-Target Interface (Non-Standard Berkeley Extension)
	/// privilege: MRW
	///@{
	/// @brief Output register to host
	CSR_mtohost = 0x780u,
	/// @brief Input register from host.
	CSR_mfromhost = 0x781u,
	///@}

	/// Debug CSR number
	/// privilege: MRW
	CSR_DBG_SCRATCH = 0x788u,
};

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
///@]

#endif  // RISCV_I_H_
