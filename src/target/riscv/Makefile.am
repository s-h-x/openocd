include $(top_srcdir)/common.mk

noinst_LTLIBRARIES = libriscv.la
libriscv_la_SOURCES = $(RISCV_SRC)

RISCV_SRC = \
	syntacore_riscv32i.c \
	riscv_opcodes.c \
	Syntacore_RV_DC.c \
	sc_rv32i__Arch.c

noinst_HEADERS = \
	riscv.h \
	Syntacore_RV_DC.h \
	sc_rv32i__Arch.h
