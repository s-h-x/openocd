/** @file
 * Support for RISC-V, debug version 0.11. This was never an officially adopted
 * spec, but SiFive made some silicon that uses it.
 */

#include "riscv.h"
#include "opcodes.h"

#include "target/algorithm.h"
#include "target/target_type.h"
#include "jtag/jtag.h"
#include "target/register.h"
#include "target/breakpoints.h"

/*
 * Since almost everything can be accomplish by scanning the dbus register, all
 * functions here assume dbus is already selected. The exception are functions
 * called directly by OpenOCD, which can't assume anything about what's
 * currently in IR. They should set IR to dbus explicitly.
 */

/*
 * Code structure
 *
 * At the bottom of the stack are the OpenOCD JTAG functions:
 *     jtag_add_[id]r_scan
 *     jtag_execute_query
 *     jtag_add_runtest
 *
 * There are a few functions to just instantly shift a register and get its
 * value:
 *    dtmcontrol_scan
 *    idcode_scan
 *    dbus_scan
 *
 * Because doing one scan and waiting for the result is slow, most functions
 * batch up a bunch of dbus writes and then execute them all at once. They use
 * the scans "class" for this:
 *    scans_new
 *    scans_delete
 *    scans_execute
 *    scans_add_...
 * Usually you new(), call a bunch of add functions, then execute() and look
 * at the results by calling scans_get...()
 *
 * Optimized functions will directly use the scans class above, but slightly
 * lazier code will use the cache functions that in turn use the scans
 * functions:
 *    cache_get...
 *    cache_set...
 *    cache_write
 * cache_set... update a local structure, which is then synced to the target
 * with cache_write(). Only Debug RAM words that are actually changed are sent
 * to the target. Afterwards use cache_get... to read results.
 */

#define CACHE_NO_READ	128

#define DEBUG_ROM_START		0x800
#define DEBUG_ROM_RESUME	(DEBUG_ROM_START + 4)
#define DEBUG_ROM_EXCEPTION	(DEBUG_ROM_START + 8)
#define DEBUG_RAM_START		0x400

#define SETHALTNOT				0x10c

/* JTAG registers. */

#define DTMCONTROL_DBUS_RESET		(1<<16)
#define DTMCONTROL_IDLE				(7<<10)
#define DTMCONTROL_ADDRBITS			(0xf<<4)
#define DTMCONTROL_VERSION			(0xf)

/**
	@todo Move to common file
*/
#define DBUS						0x11
#define DBUS_OP_START				0
#define DBUS_OP_SIZE				2

/** Constants for legacy SiFive hardware breakpoints. */
enum CSR_BPCONTROL_e {
	CSR_BPCONTROL_X			= (1<<0),
	CSR_BPCONTROL_W			= (1<<1),
	CSR_BPCONTROL_R			= (1<<2),
	CSR_BPCONTROL_U			= (1<<3),
	CSR_BPCONTROL_S			= (1<<4),
	CSR_BPCONTROL_H			= (1<<5),
	CSR_BPCONTROL_M			= (1<<6),
	CSR_BPCONTROL_BPMATCH	= (0xf<<7),
	CSR_BPCONTROL_BPACTION	= (0xff<<11),
};

enum dbus_op_e {
	DBUS_OP_NOP = 0,
	DBUS_OP_READ = 1,
	DBUS_OP_WRITE = 2
};
typedef enum dbus_op_e dbus_op_t;

enum dbus_status_e {
	DBUS_STATUS_SUCCESS = 0,
	DBUS_STATUS_FAILED = 2,
	DBUS_STATUS_BUSY = 3
};
typedef enum dbus_status_e dbus_status_t;
#define DBUS_DATA_START				2
#define DBUS_DATA_SIZE				34
#define DBUS_ADDRESS_START			36

enum riscv_error_e {
	RE_OK,
	RE_FAIL,
	RE_AGAIN
};
typedef enum riscv_error_e riscv_error_t;

enum slot_e {
	SLOT0,
	SLOT1,
	SLOT_LAST,
};
typedef enum slot_e slot_t;

/* Debug Bus registers. */

#define DMCONTROL				0x10
#define DMCONTROL_INTERRUPT		(UINT64_C(1) << 33)
#define DMCONTROL_HALTNOT		(UINT64_C(1) << 32)
#define DMCONTROL_BUSERROR		(7 << 19)
#define DMCONTROL_SERIAL		(3 << 16)
#define DMCONTROL_AUTOINCREMENT	(1 << 15)
#define DMCONTROL_ACCESS		(7 << 12)
#define DMCONTROL_HARTID		(0x3ff << 2)
#define DMCONTROL_NDRESET		(1 << 1)
#define DMCONTROL_FULLRESET		1

#define DMINFO					0x11
#define DMINFO_ABUSSIZE			(0x7fU<<25)
#define DMINFO_SERIALCOUNT		(0xf<<21)
#define DMINFO_ACCESS128		(1<<20)
#define DMINFO_ACCESS64			(1<<19)
#define DMINFO_ACCESS32			(1<<18)
#define DMINFO_ACCESS16			(1<<17)
#define DMINFO_ACCESS8			(1<<16)
#define DMINFO_DRAMSIZE			(0x3f<<10)
#define DMINFO_AUTHENTICATED	(1<<5)
#define DMINFO_AUTHBUSY			(1<<4)
#define DMINFO_AUTHTYPE			(3<<2)
#define DMINFO_VERSION			3

/* Info about the core being debugged. */

#define DBUS_ADDRESS_UNKNOWN	0xffff

#define DRAM_CACHE_SIZE		16

struct memory_cache_line {
	uint32_t data;
	bool valid;
	bool dirty;
};

struct riscv_011_info_s {
	/** Number of address bits in the dbus register. */
	uint8_t addrbits;

	/** Number of words in Debug RAM. */
	unsigned dramsize;

	uint64_t dcsr;
	uint64_t dpc;
	uint64_t tselect;
	bool tselect_dirty;

	/** The value that mstatus actually has on the target right now. This is not
	 * the value we present to the user. That one may be stored in the
	 * reg_cache. */
	uint64_t mstatus_actual;

	struct memory_cache_line dram_cache[DRAM_CACHE_SIZE];

	/** Number of run-test/idle cycles the target requests we do after each dbus
	 * access. */
	unsigned dtmcontrol_idle;

	/** This value is incremented every time a dbus access comes back as "busy".
	 * It's used to determine how many run-test/idle cycles to feed the target
	 * in between accesses. */
	unsigned dbus_busy_delay;

	/** This value is incremented every time we read the debug interrupt as
	 * high.  It's used to add extra run-test/idle cycles after setting debug
	 * interrupt high, so ideally we never have to perform a whole extra scan
	 * before the interrupt is cleared. */
	unsigned interrupt_high_delay;

	bool need_strict_step;
	bool never_halted;
};
typedef struct riscv_011_info_s riscv_011_info_t;

struct bits_s {
	bool haltnot;
	bool interrupt;
};
typedef struct bits_s bits_t;

struct scans_s {
	/** Number of scans that space is reserved for. */
	unsigned scan_count;

	/** Size reserved in memory for each scan, in bytes. */
	unsigned scan_size;
	unsigned next_scan;
	uint8_t *in;
	uint8_t *out;
	struct scan_field *field;
	struct target const *target;
};

typedef struct scans_s scans_t;

/* Necessary prototypes. */
static int riscv_011_poll(struct target *const target);
static int riscv_011_get_register(struct target *const target, riscv_reg_t *value, int hartid,
		int regid);

/* Utility functions. */

#define DEBUG_LENGTH	264

/** Version-independent functions that we don't want in the main address space. */
/**@{*/
static inline uint32_t
load(struct target const *const target,
	unsigned const rd,
	unsigned const base,
	uint16_t const offset)
{
	switch (riscv_xlen(target)) {
	case 32:
		return lw(rd, base, offset);

	case 64:
		return ld(rd, base, offset);

	default:
		break;
	}

	assert(0);
	/* Silence -Werror=return-type */
	return 0;
}

static inline uint32_t
store(struct target const *const target,
	unsigned const src,
	unsigned const base,
	uint16_t const offset)
{
	switch (riscv_xlen(target)) {
	case 32:
		return sw(src, base, offset);

	case 64:
		return sd(src, base, offset);

	default:
		break;
	}

	assert(0);
	/* Silence -Werror=return-type */
	return 0;
}
/**@}*/

static riscv_011_info_t *
__attribute__((pure))
get_info(const struct target *const target)
{
	assert(target);
	struct riscv_info_t const *const info = target->arch_info;
	assert(info);
	return info->version_specific;
}

static unsigned
slot_offset(struct target const *const target,
	slot_t const slot)
{
	riscv_011_info_t const *const info = get_info(target);
	assert(info);

	switch (riscv_xlen(target)) {
		case 32:
			switch (slot) {
				case SLOT0: return 4;
				case SLOT1: return 5;
				case SLOT_LAST: return info->dramsize-1;
				/**
				@bug no default case
				*/
			}
			break;

		case 64:
			switch (slot) {
				case SLOT0:
					return 4;

				case SLOT1:
					return 6;

				case SLOT_LAST:
					return info->dramsize-2;

				/**
				@bug no default case
				*/
			}
			break;

		/**
		@bug no default case
		*/
	}

	LOG_ERROR("%s: slot_offset called with xlen=%d, slot=%d",
		target_name(target), riscv_xlen(target), slot);
	assert(0);
	return 0; /* Silence -Werror=return-type */
}

static uint32_t
load_slot(struct target const *const target,
	unsigned const dest,
	slot_t const slot)
{
	unsigned const offset = DEBUG_RAM_START + 4 * slot_offset(target, slot);
	return load(target, dest, ZERO, offset);
}

static uint32_t
store_slot(struct target const *const target,
	unsigned const src,
	slot_t const slot)
{
	unsigned const offset = DEBUG_RAM_START + 4 * slot_offset(target, slot);
	return store(target, src, ZERO, offset);
}

static inline uint16_t
dram_address(unsigned const index)
{
	return index < 0x10 ? index : 0x40 + index - 0x10;
}

static int
increase_dbus_busy_delay(struct target *const target)
{
	riscv_011_info_t *const info = get_info(target);
	assert(info);
	info->dbus_busy_delay += info->dbus_busy_delay / 10 + 1;
	LOG_DEBUG("%s: dtmcontrol_idle=%d, dbus_busy_delay=%d, interrupt_high_delay=%d",
		target_name(target), info->dtmcontrol_idle, info->dbus_busy_delay, info->interrupt_high_delay);

	return dtmcontrol_scan(target->tap, DTMCONTROL_DBUS_RESET, NULL);
}

static void
increase_interrupt_high_delay(struct target *const target)
{
	riscv_011_info_t *info = get_info(target);
	assert(info);

	info->interrupt_high_delay += info->interrupt_high_delay / 10 + 1;
	LOG_DEBUG("%s: dtmcontrol_idle=%d, dbus_busy_delay=%d, interrupt_high_delay=%d", target_name(target),
			info->dtmcontrol_idle, info->dbus_busy_delay,
			info->interrupt_high_delay);
}

static void
add_dbus_scan(struct target const *const target,
	struct scan_field *field,
	uint8_t *out_value,
	uint8_t *in_value,
	dbus_op_t op,
	uint16_t address,
	uint64_t data)
{
	riscv_011_info_t *info = get_info(target);
	assert(info);
	assert(field);
	field->num_bits = info->addrbits + DBUS_OP_SIZE + DBUS_DATA_SIZE;
	field->in_value = in_value;
	field->out_value = out_value;

	buf_set_u64(out_value, DBUS_OP_START, DBUS_OP_SIZE, op);
	buf_set_u64(out_value, DBUS_DATA_START, DBUS_DATA_SIZE, data);
	buf_set_u64(out_value, DBUS_ADDRESS_START, info->addrbits, address);

	jtag_add_dr_scan(target->tap, 1, field, TAP_IDLE);

	int idle_count = info->dtmcontrol_idle + info->dbus_busy_delay;

	if (data & DMCONTROL_INTERRUPT)
		idle_count += info->interrupt_high_delay;

	if (idle_count)
		jtag_add_runtest(idle_count, TAP_IDLE);
}

static void
riscv_011_dump_field(struct scan_field const *const field)
{
	static const char *const op_string[] = {"nop", "r", "w", "?"};
	static const char *const status_string[] = {"+", "?", "F", "b"};

	if (debug_level < LOG_LVL_DEBUG)
		return;

	assert(field);
	assert(field->out_value);
	uint64_t const out_value = buf_get_u64(field->out_value, 0, field->num_bits);
	unsigned const out_op = (out_value >> DBUS_OP_START) & ((1 << DBUS_OP_SIZE) - 1);
	char const out_interrupt = ((out_value >> DBUS_DATA_START) & DMCONTROL_INTERRUPT) ? 'i' : '.';
	char const out_haltnot = ((out_value >> DBUS_DATA_START) & DMCONTROL_HALTNOT) ? 'h' : '.';
	unsigned const out_data = out_value >> 2;
	unsigned const out_address = out_value >> DBUS_ADDRESS_START;

	assert(field->in_value);
	uint64_t const in_value = buf_get_u64(field->in_value, 0, field->num_bits);
	unsigned const in_op = (in_value >> DBUS_OP_START) & ((1 << DBUS_OP_SIZE) - 1);
	char const in_interrupt = ((in_value >> DBUS_DATA_START) & DMCONTROL_INTERRUPT) ? 'i' : '.';
	char const in_haltnot = ((in_value >> DBUS_DATA_START) & DMCONTROL_HALTNOT) ? 'h' : '.';
	unsigned const in_data = in_value >> 2;
	unsigned const in_address = in_value >> DBUS_ADDRESS_START;

	LOG_DEBUG("%db %s %c%c:%08x @%02x -> %s %c%c:%08x @%02x",
			field->num_bits,
			op_string[out_op], out_interrupt, out_haltnot, out_data,
			out_address,
			status_string[in_op], in_interrupt, in_haltnot, in_data,
			in_address);
}

static dbus_status_t
dbus_scan(struct target *const target,
	uint16_t *const address_in,
	uint64_t *const data_in,
	dbus_op_t const op,
	uint16_t const address_out,
	uint64_t const data_out)
{
	riscv_011_info_t *const info = get_info(target);
	assert(info);
	assert(info->addrbits != 0);

	uint8_t out_buffer[8];
	uint8_t in_buffer[8] = {0};
	struct scan_field field = {
		.num_bits = info->addrbits + DBUS_OP_SIZE + DBUS_DATA_SIZE,
		.out_value = out_buffer,
		.in_value = in_buffer
	};

	buf_set_u64(out_buffer, DBUS_OP_START, DBUS_OP_SIZE, op);
	buf_set_u64(out_buffer, DBUS_DATA_START, DBUS_DATA_SIZE, data_out);
	buf_set_u64(out_buffer, DBUS_ADDRESS_START, info->addrbits, address_out);

	/* Assume dbus is already selected. */
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	int const idle_count =
		info->dtmcontrol_idle + info->dbus_busy_delay;

	if (idle_count)
		jtag_add_runtest(idle_count, TAP_IDLE);

	{
		int const error_code = jtag_execute_queue();

		if (ERROR_OK != error_code) {
			LOG_ERROR("%s: dbus_scan failed jtag scan", target_name(target));
			return DBUS_STATUS_FAILED;
		}
	}

	if (data_in)
		*data_in = buf_get_u64(in_buffer, DBUS_DATA_START, DBUS_DATA_SIZE);

	if (address_in)
		*address_in = buf_get_u32(in_buffer, DBUS_ADDRESS_START, info->addrbits);

	riscv_011_dump_field(&field);

	return buf_get_u32(in_buffer, DBUS_OP_START, DBUS_OP_SIZE);
}

static uint64_t
dbus_read(struct target *const target,
	uint16_t const address)
{
	/* If the previous read/write was to the same address, we will get the read data
	 * from the previous access.
	 * While somewhat nonintuitive, this is an efficient way to get the data.
	 */

	unsigned i = 0;

	uint64_t value;
	dbus_status_t status;
	uint16_t address_in;
	do {
		status = dbus_scan(target, &address_in, &value, DBUS_OP_READ, address, 0);

		if (status == DBUS_STATUS_BUSY)
			increase_dbus_busy_delay(target);
		if (status == DBUS_STATUS_FAILED) {
			LOG_ERROR("%s: dbus_read(0x%x) failed!", target_name(target), address);
			return 0;
		}
	} while ((status == DBUS_STATUS_BUSY || address_in != address) && i++ < 256);

	if (status != DBUS_STATUS_SUCCESS)
		LOG_ERROR("%s: failed read from 0x%x; value=0x%" PRIx64 ", status=%d\n",
			target_name(target), address, value, status);

	return value;
}

static void
dbus_write(struct target *const target,
	uint16_t const address,
	uint64_t const value)
{
	dbus_status_t status = DBUS_STATUS_BUSY;
	unsigned i = 0;

	while (status == DBUS_STATUS_BUSY && i++ < 256) {
		status = dbus_scan(target, NULL, NULL, DBUS_OP_WRITE, address, value);

		if (status == DBUS_STATUS_BUSY)
			increase_dbus_busy_delay(target);
	}

	if (status != DBUS_STATUS_SUCCESS)
		LOG_ERROR("%s: failed to write 0x%" PRIx64 " to 0x%x; status=%d",
			target_name(target), value, address, status);
}

static scans_t *
scans_new(struct target *const target, unsigned scan_count)
{
	scans_t *scans = malloc(sizeof(scans_t));
	assert(scans);
	scans->scan_count = scan_count;
	/* This code also gets called before xlen is detected. */
	scans->scan_size = riscv_xlen(target) ? 2 + riscv_xlen(target) / 8 : 2 + 128 / 8;
	scans->next_scan = 0;
	scans->in = calloc(scans->scan_size, scans->scan_count);
	scans->out = calloc(scans->scan_size, scans->scan_count);
	scans->field = calloc(scans->scan_count, sizeof(struct scan_field));
	scans->target = target;
	return scans;
}

static scans_t *scans_delete(scans_t *scans)
{
	assert(scans);
	free(scans->field);
	free(scans->out);
	free(scans->in);
	free(scans);
	return NULL;
}

static void scans_reset(scans_t *scans)
{
	scans->next_scan = 0;
}

static void scans_dump(scans_t *scans)
{
	for (unsigned i = 0; i < scans->next_scan; ++i)
		riscv_011_dump_field(&scans->field[i]);
}

static int scans_execute(scans_t *scans)
{
	{
		int const error_code = jtag_execute_queue();

		if (ERROR_OK != error_code) {
			LOG_ERROR("failed jtag scan: %d", error_code);
			return error_code;
		}
	}

	scans_dump(scans);

	return ERROR_OK;
}

/** Add a 32-bit dbus write to the scans structure. */
static void
scans_add_write32(scans_t *const scans,
	uint16_t const address,
	uint32_t const data,
	bool set_interrupt)
{
	unsigned const i = scans->next_scan;
	int const data_offset = scans->scan_size * i;
	add_dbus_scan(scans->target, &scans->field[i], scans->out + data_offset,
			scans->in + data_offset, DBUS_OP_WRITE, address,
			(set_interrupt ? DMCONTROL_INTERRUPT : 0) | DMCONTROL_HALTNOT | data);
	++scans->next_scan;
	assert(scans->next_scan <= scans->scan_count);
}

/** Add a 32-bit dbus write for an instruction that jumps to the beginning of
 * debug RAM. */
static void scans_add_write_jump(scans_t *scans, uint16_t address,
		bool set_interrupt)
{
	scans_add_write32(scans, address,
			jal(0, (uint32_t) (DEBUG_ROM_RESUME - (DEBUG_RAM_START + 4*address))),
			set_interrupt);
}

/** Add a 32-bit dbus write for an instruction that loads from the indicated
 * slot. */
static void scans_add_write_load(scans_t *scans, uint16_t address,
		unsigned reg, slot_t slot, bool set_interrupt)
{
	scans_add_write32(scans, address, load_slot(scans->target, reg, slot),
			set_interrupt);
}

/** Add a 32-bit dbus write for an instruction that stores to the indicated
 * slot. */
static void scans_add_write_store(scans_t *scans, uint16_t address,
		unsigned reg, slot_t slot, bool set_interrupt)
{
	scans_add_write32(scans, address, store_slot(scans->target, reg, slot),
			set_interrupt);
}

/** Add a 32-bit dbus read. */
static void
scans_add_read32(scans_t *scans,
	uint16_t const address,
	bool const set_interrupt)
{
	assert(scans->next_scan < scans->scan_count);
	unsigned const i = scans->next_scan;
	int const data_offset = scans->scan_size * i;
	add_dbus_scan(scans->target, &scans->field[i], scans->out + data_offset,
			scans->in + data_offset, DBUS_OP_READ, address,
			(set_interrupt ? DMCONTROL_INTERRUPT : 0) | DMCONTROL_HALTNOT);
	++scans->next_scan;
}

/** Add one or more scans to read the indicated slot. */
static void scans_add_read(scans_t *scans, slot_t slot, bool set_interrupt)
{
	const struct target *const target = scans->target;
	switch (riscv_xlen(target)) {
		case 32:
			scans_add_read32(scans, slot_offset(target, slot), set_interrupt);
			break;
		case 64:
			scans_add_read32(scans, slot_offset(target, slot), false);
			scans_add_read32(scans, slot_offset(target, slot) + 1, set_interrupt);
			break;
	}
}

static uint32_t scans_get_u32(scans_t *scans, unsigned index,
		unsigned first, unsigned num)
{
	return buf_get_u32(scans->in + scans->scan_size * index, first, num);
}

static uint64_t scans_get_u64(scans_t *scans, unsigned index,
		unsigned first, unsigned num)
{
	return buf_get_u64(scans->in + scans->scan_size * index, first, num);
}

/* end of scans class */

static uint32_t dram_read32(struct target *const target, unsigned index)
{
	uint16_t address = dram_address(index);
	uint32_t value = dbus_read(target, address);
	return value;
}

static void dram_write32(struct target *const target, unsigned index, uint32_t value,
		bool set_interrupt)
{
	uint64_t dbus_value = DMCONTROL_HALTNOT | value;
	if (set_interrupt)
		dbus_value |= DMCONTROL_INTERRUPT;
	dbus_write(target, dram_address(index), dbus_value);
}

/** Read the haltnot and interrupt bits. */
static bits_t
read_bits(struct target *const target)
{
	uint64_t value;
	dbus_status_t status;
	uint16_t address_in;
	riscv_011_info_t *info = get_info(target);

	bits_t err_result = {
		.haltnot = 0,
		.interrupt = 0
	};

	do {
		unsigned i = 0;
		do {
			status = dbus_scan(target, &address_in, &value, DBUS_OP_READ, 0, 0);
			if (status == DBUS_STATUS_BUSY) {
				if (address_in == (1<<info->addrbits) - 1 &&
						value == (1ULL<<DBUS_DATA_SIZE) - 1) {
					LOG_ERROR("%s: TDO seems to be stuck high.", target_name(target));
					return err_result;
				}
				increase_dbus_busy_delay(target);
			} else if (status == DBUS_STATUS_FAILED) {
				/**
				@todo return an actual error
				*/
				return err_result;
			}
		} while (status == DBUS_STATUS_BUSY && i++ < 256);

		if (i >= 256) {
			LOG_ERROR("%s: Failed to read from 0x%x; status=%d", target_name(target), address_in, status);
			return err_result;
		}
	} while (address_in > 0x10 && address_in != DMCONTROL);

	bits_t result = {
		.haltnot = get_field(value, DMCONTROL_HALTNOT),
		.interrupt = get_field(value, DMCONTROL_INTERRUPT)
	};
	return result;
}

static int
wait_for_debugint_clear(struct target *const target,
	bool const ignore_first)
{
	time_t const start = time(NULL);

	if (ignore_first) {
		/* Throw away the results of the first read, since they'll contain the
		 * result of the read that happened just before debugint was set.
		 * (Assuming the last scan before calling this function was one that
		 * sets debugint.) */
		read_bits(target);
	}

	for (;;) {
		bits_t bits = read_bits(target);
		if (!bits.interrupt)
			return ERROR_OK;

		if (time(NULL) > start + riscv_command_timeout_sec) {
			LOG_ERROR("%s: Timed out waiting for debug int to clear."
					"Increase timeout with riscv set_command_timeout_sec.",
					target_name(target));
			return ERROR_TARGET_TIMEOUT;
		}
	}
}

static int
dram_check32(struct target *const target,
	unsigned const index,
	uint32_t const expected)
{
	uint16_t const address = dram_address(index);
	uint32_t const actual = dbus_read(target, address);

	if (expected != actual) {
		LOG_ERROR("%s: Wrote 0x%x to Debug RAM at %d, but read back 0x%x",
				target_name(target),
				expected,
				index,
				actual);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static void
cache_set32(struct target *const target,
	unsigned const index,
	uint32_t const data)
{
	riscv_011_info_t *const info = get_info(target);
	assert(info);

	struct memory_cache_line *const p_cache_line =
		info->dram_cache + index;

	if (data == p_cache_line->valid && p_cache_line->data) {
		/* This is already preset on the target. */
		LOG_DEBUG("%s: cache[0x%x] = 0x%08" PRIx32 ": DASM(0x%" PRIx32 ") (hit)",
				target_name(target), index, data, data);
		return;
	}

	LOG_DEBUG("%s: cache[0x%x] = 0x%08x: DASM(0x%x)",
			target_name(target), index, data, data);
	p_cache_line->data = data;
	p_cache_line->valid = true;
	p_cache_line->dirty = true;
}

static void
cache_set(struct target *const target,
	slot_t const slot,
	uint64_t const data)
{
	unsigned const offset = slot_offset(target, slot);
	cache_set32(target, offset, data);

	if (riscv_xlen(target) > 32)
		cache_set32(target, offset + 1, data >> 32);
}

static void
cache_set_jump(struct target *const target,
	unsigned const index)
{
	cache_set32(target, index,
			jal(0, (uint32_t) (DEBUG_ROM_RESUME - (DEBUG_RAM_START + 4*index))));
}

static void
cache_set_load(struct target *const target,
	unsigned const index,
	unsigned const reg,
	slot_t const slot)
{
	uint16_t const offset = DEBUG_RAM_START + 4 * slot_offset(target, slot);
	cache_set32(target, index, load(target, reg, ZERO, offset));
}

static void cache_set_store(struct target *const target, unsigned index,
		unsigned reg, slot_t slot)
{
	uint16_t offset = DEBUG_RAM_START + 4 * slot_offset(target, slot);
	cache_set32(target, index, store(target, reg, ZERO, offset));
}

static int
dump_debug_ram(struct target *const target)
{
	for (unsigned i = 0; i < DRAM_CACHE_SIZE; ++i) {
		uint32_t value = dram_read32(target, i);
		LOG_ERROR("%s: Debug RAM 0x%x: 0x%08x",
				target_name(target), i, value);
	}
	return ERROR_TARGET_FAILURE;
}

/* Call this if the code you just ran writes to debug RAM entries 0 through 3. */
static void
cache_invalidate(struct target *const target)
{
	riscv_011_info_t *info = get_info(target);
	for (unsigned i = 0; i < info->dramsize; ++i) {
		info->dram_cache[i].valid = false;
		info->dram_cache[i].dirty = false;
	}
}

/* Called by cache_write() after the program has run. Also call this if you're
 * running programs without calling cache_write(). */
static void
cache_clean(struct target *const target)
{
	riscv_011_info_t *info = get_info(target);

	for (unsigned i = 0; i < info->dramsize; ++i) {
		if (i >= 4)
			info->dram_cache[i].valid = false;
		info->dram_cache[i].dirty = false;
	}
}

static int
cache_check(struct target *const target)
{
	riscv_011_info_t *info = get_info(target);
	int error = 0;

	for (unsigned i = 0; i < info->dramsize; ++i)
		if (
			info->dram_cache[i].valid &&
			!info->dram_cache[i].dirty &&
			ERROR_OK != dram_check32(target, i, info->dram_cache[i].data)
			)
			++error;

	if (error) {
		return dump_debug_ram(target);
	}

	return ERROR_OK;
}

/** Write cache to the target, and optionally run the program.
 * Then read the value at address into the cache, assuming address < 128. */
static int cache_write(struct target *const target, unsigned address, bool run)
{
	LOG_DEBUG("%s: enter", target_name(target));
	riscv_011_info_t *info = get_info(target);
	scans_t *scans = scans_new(target, info->dramsize + 2);

	unsigned last = info->dramsize;
	for (unsigned i = 0; i < info->dramsize; ++i) {
		if (info->dram_cache[i].dirty)
			last = i;
	}

	if (last == info->dramsize) {
		/* Nothing needs to be written to RAM. */
		dbus_write(target, DMCONTROL, DMCONTROL_HALTNOT | (run ? DMCONTROL_INTERRUPT : 0));

	} else {
		for (unsigned i = 0; i < info->dramsize; ++i) {
			if (info->dram_cache[i].dirty) {
				bool set_interrupt = (i == last && run);
				scans_add_write32(scans, i, info->dram_cache[i].data,
						set_interrupt);
			}
		}
	}

	if (run || address < CACHE_NO_READ) {
		/* Throw away the results of the first read, since it'll contain the
		 * result of the read that happened just before debugint was set. */
		scans_add_read32(scans, address, false);

		/* This scan contains the results of the read the caller requested, as
		 * well as an interrupt bit worth looking at. */
		scans_add_read32(scans, address, false);
	}

	{
		int const error_code = scans_execute(scans);

		if (ERROR_OK != error_code) {
			scans_delete(scans);
			LOG_ERROR("%s: JTAG execute failed.", target_name(target));
			return error_code;
		}
	}

	int errors = 0;

	for (unsigned i = 0; i < scans->next_scan; ++i) {
		dbus_status_t const status =
			scans_get_u32(scans, i, DBUS_OP_START, DBUS_OP_SIZE);

		switch (status) {
			case DBUS_STATUS_SUCCESS:
				break;

			case DBUS_STATUS_FAILED:
				LOG_ERROR("%s: Debug RAM write failed. Hardware error?", target_name(target));
				scans_delete(scans);
				return ERROR_TARGET_FAILURE;

			case DBUS_STATUS_BUSY:
				++errors;
				break;

			default:
				LOG_ERROR("%s: Got invalid bus access status: %d",
					target_name(target), status);
				scans_delete(scans);
				return ERROR_TARGET_FAILURE;
		}
	}

	if (errors) {
		increase_dbus_busy_delay(target);

		/* Try again, using the slow careful code.
		 * Write all RAM, just to be extra cautious. */
		for (unsigned i = 0; i < info->dramsize; ++i) {
			if (i == last && run)
				dram_write32(target, last, info->dram_cache[last].data, true);
			else
				dram_write32(target, i, info->dram_cache[i].data, false);

			info->dram_cache[i].dirty = false;
		}

		if (run)
			cache_clean(target);

		{
			int const err = wait_for_debugint_clear(target, true);
			if (ERROR_OK != err) {
				LOG_ERROR("%s: Debug interrupt didn't clear.", target_name(target));
				dump_debug_ram(target);
				scans_delete(scans);
				return err;
			}
		}
	} else {
		if (run) {
			cache_clean(target);
		} else {
			for (unsigned i = 0; i < info->dramsize; ++i)
				info->dram_cache[i].dirty = false;
		}

		if (run || address < CACHE_NO_READ) {
			int interrupt = scans_get_u32(scans, scans->next_scan-1,
					DBUS_DATA_START + 33, 1);
			if (interrupt) {
				increase_interrupt_high_delay(target);
				/* Slow path wait for it to clear. */
				int const err = wait_for_debugint_clear(target, false);
				if (ERROR_OK != err) {
					LOG_ERROR("%s: Debug interrupt didn't clear.", target_name(target));
					dump_debug_ram(target);
					scans_delete(scans);
					return err;
				}
			} else {
				/* We read a useful value in that last scan. */
				unsigned const read_addr =
					scans_get_u32(scans, scans->next_scan-1, DBUS_ADDRESS_START, info->addrbits);
				if (read_addr != address) {
					LOG_INFO("%s: Got data from 0x%x but expected it from 0x%x",
							target_name(target),
							read_addr, address);
				}
				info->dram_cache[read_addr].data =
					scans_get_u32(scans, scans->next_scan-1, DBUS_DATA_START, 32);
				info->dram_cache[read_addr].valid = true;
			}
		}
	}

	scans_delete(scans);
	LOG_DEBUG("%s: exit", target_name(target));

	return ERROR_OK;
}

static uint32_t
cache_get32(struct target *const target,
	unsigned const address)
{
	riscv_011_info_t *info = get_info(target);
	assert(info);

	if (!info->dram_cache[address].valid) {
		info->dram_cache[address].data = dram_read32(target, address);
		info->dram_cache[address].valid = true;
	}

	return info->dram_cache[address].data;
}

static uint64_t
cache_get(struct target *const target,
	slot_t const slot)
{
	unsigned offset = slot_offset(target, slot);
	uint64_t value = cache_get32(target, offset);

	if (riscv_xlen(target) > 32)
		value |= (uint64_t)(cache_get32(target, offset + 1)) << 32;

	return value;
}

/* Write instruction that jumps from the specified word in Debug RAM to resume
 * in Debug ROM. */
static void
dram_write_jump(struct target *const target, unsigned index,
		bool set_interrupt)
{
	dram_write32(target, index,
			jal(0, (uint32_t) (DEBUG_ROM_RESUME - (DEBUG_RAM_START + 4*index))),
			set_interrupt);
}

static int
wait_for_state(struct target *const target,
	enum target_state const state)
{
	time_t const start = time(NULL);

	for (;;) {
		{
			int const error_code = riscv_011_poll(target);

			if (ERROR_OK != error_code)
				return error_code;
		}

		if (state == target->state)
			return ERROR_OK;

		if (start + riscv_command_timeout_sec < time(NULL)) {
			LOG_ERROR("%s: Timed out waiting for state %d. "
					"Increase timeout with riscv set_command_timeout_sec.",
					target_name(target), state);
			return ERROR_TARGET_TIMEOUT;
		}
	}
}

static int
read_csr(struct target *const target,
	uint64_t *const value,
	uint32_t const csr)
{
	cache_set32(target, 0, csrr(S0, csr));
	cache_set_store(target, 1, S0, SLOT0);
	cache_set_jump(target, 2);
	{
		int const error_code = cache_write(target, 4, true);
		if (ERROR_OK != error_code)
			return error_code;
	}

	assert(value);
	*value = cache_get(target, SLOT0);
	LOG_DEBUG("%s: csr 0x%x = 0x%" PRIx64,
		target_name(target), csr, *value);

	riscv_011_info_t *const info = get_info(target);
	assert(info);
	uint32_t const exception = cache_get32(target, info->dramsize-1);
	if (exception) {
		LOG_WARNING("%s: Got exception 0x%x when reading %s",
					target_name(target),
					exception,
					gdb_regno_name(GDB_REGNO_CSR0 + csr));
		*value = ~0;
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int
write_csr(struct target *const target, uint32_t csr, uint64_t value)
{
	LOG_DEBUG("%s: csr 0x%x <- 0x%" PRIx64,
		target_name(target), csr, value);
	cache_set_load(target, 0, S0, SLOT0);
	cache_set32(target, 1, csrw(S0, csr));
	cache_set_jump(target, 2);
	cache_set(target, SLOT0, value);
	return cache_write(target, 4, true);
}

static int
write_gpr(struct target *const target, unsigned gpr, uint64_t value)
{
	cache_set_load(target, 0, gpr, SLOT0);
	cache_set_jump(target, 1);
	cache_set(target, SLOT0, value);
	return cache_write(target, 4, true);
}

static int
maybe_read_tselect(struct target *const target)
{
	riscv_011_info_t *const info = get_info(target);

	if (info->tselect_dirty) {
		int const error_code = read_csr(target, &info->tselect, CSR_TSELECT);

		if (ERROR_OK != error_code)
			return error_code;

		info->tselect_dirty = false;
	}

	return ERROR_OK;
}

static int
maybe_write_tselect(struct target *const target)
{
	riscv_011_info_t *const info = get_info(target);

	if (!info->tselect_dirty) {
		int const error_code = write_csr(target, CSR_TSELECT, info->tselect);

		if (ERROR_OK != error_code)
			return error_code;

		info->tselect_dirty = true;
	}

	return ERROR_OK;
}

static int
execute_resume(struct target *const target,
	bool const step)
{
	riscv_011_info_t *const info = get_info(target);

	LOG_DEBUG("%s: step=%d",
		target_name(target), step);

	maybe_write_tselect(target);

	/**
		@todo check if dpc is dirty (which also is true if an exception was hit at any time)
	*/
	cache_set_load(target, 0, S0, SLOT0);
	cache_set32(target, 1, csrw(S0, CSR_DPC));
	cache_set_jump(target, 2);
	cache_set(target, SLOT0, info->dpc);
	{
		int const error_code = cache_write(target, 4, true);

		if (ERROR_OK != error_code)
			return error_code;
	}

	struct reg *mstatus_reg = &target->reg_cache->reg_list[GDB_REGNO_MSTATUS];
	if (mstatus_reg->valid) {
		uint64_t mstatus_user = buf_get_u64(mstatus_reg->value, 0, riscv_xlen(target));
		if (mstatus_user != info->mstatus_actual) {
			cache_set_load(target, 0, S0, SLOT0);
			cache_set32(target, 1, csrw(S0, CSR_MSTATUS));
			cache_set_jump(target, 2);
			cache_set(target, SLOT0, mstatus_user);
			{
				int const error_code = cache_write(target, 4, true);

				if (ERROR_OK != error_code)
					return error_code;
			}
		}
	}

	info->dcsr |= DCSR_EBREAKM | DCSR_EBREAKH | DCSR_EBREAKS | DCSR_EBREAKU;
	info->dcsr &= ~DCSR_HALT;

	if (step)
		info->dcsr |= DCSR_STEP;
	else
		info->dcsr &= ~DCSR_STEP;

	dram_write32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16), false);
	dram_write32(target, 1, csrw(S0, CSR_DCSR), false);
	dram_write32(target, 2, fence_i(), false);
	dram_write_jump(target, 3, false);

	/* Write DCSR value, set interrupt and clear haltnot. */
	uint64_t dbus_value = DMCONTROL_INTERRUPT | info->dcsr;
	dbus_write(target, dram_address(4), dbus_value);

	cache_invalidate(target);

	{
		int const error_code = wait_for_debugint_clear(target, true);

		if (ERROR_OK != error_code) {
			LOG_ERROR("%s: Debug interrupt didn't clear.", target_name(target));
			return error_code;
		}
	}

	target->state = TARGET_RUNNING;
	register_cache_invalidate(target->reg_cache);

	return ERROR_OK;
}

static int
resume(struct target *const target,
	int const debug_execution,
	bool const step)
{
	if (debug_execution) {
		LOG_ERROR("%s: TODO: debug_execution is true",
			target_name(target));
		return ERROR_TARGET_INVALID;
	}

	return execute_resume(target, step);
}

static uint64_t
reg_cache_get(struct target *const target,
	unsigned const number)
{
	struct reg *r = &target->reg_cache->reg_list[number];
	if (!r->valid) {
		LOG_ERROR("%s: Register cache entry for %d is invalid!",
				target_name(target),
				number);
		assert(r->valid);
	}
	uint64_t value = buf_get_u64(r->value, 0, r->size);
	LOG_DEBUG("%s: %s = 0x%" PRIx64, target_name(target), r->name, value);
	return value;
}

static void
reg_cache_set(struct target *const target,
	unsigned const number,
	uint64_t const value)
{
	assert(target->reg_cache->reg_list && number < target->reg_cache->num_regs);
	struct reg *const r = target->reg_cache->reg_list + number;
	assert(r);
	LOG_DEBUG("%s: %s <= 0x%" PRIx64,
		target_name(target), r->name, value);
	r->valid = true;
	buf_set_u64(r->value, 0, r->size, value);
}

static int update_mstatus_actual(struct target *const target)
{
	struct reg *mstatus_reg = &target->reg_cache->reg_list[GDB_REGNO_MSTATUS];
	if (mstatus_reg->valid) {
		/* We previously made it valid. */
		return ERROR_OK;
	}

	/* Force reading the register. In that process mstatus_actual will be
	 * updated. */
	riscv_reg_t mstatus;
	return riscv_011_get_register(target, &mstatus, 0, GDB_REGNO_MSTATUS);
}

static int
register_read(struct target *const target,
	riscv_reg_t *const value,
	int const regnum)
{
	if (GDB_REGNO_CSR0 <= regnum && regnum <= GDB_REGNO_CSR4095) {
		cache_set32(target, 0, csrr(S0, regnum - GDB_REGNO_CSR0));
		cache_set_store(target, 1, S0, SLOT0);
		cache_set_jump(target, 2);
	} else {
		LOG_ERROR("%s: Don't know how to read register %d", target_name(target), regnum);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	{
		int const error_code = cache_write(target, 4, true);

		if (ERROR_OK != error_code)
			return error_code;
	}

	riscv_011_info_t *const info = get_info(target);
	assert(info);
	uint32_t const exception = cache_get32(target, info->dramsize-1);
	assert(value);
	if (exception) {
		LOG_WARNING("%s: Got exception 0x%x when reading %s",
					target_name(target), exception, gdb_regno_name(regnum));
		*value = ~0;
		return ERROR_TARGET_FAILURE;
	}

	*value = cache_get(target, SLOT0);
	LOG_DEBUG("%s: reg[%d]=0x%" PRIx64,
			target_name(target), regnum, *value);

	if (GDB_REGNO_MSTATUS == regnum)
		info->mstatus_actual = *value;

	return ERROR_OK;
}

/* Write the register. No caching or games. */
static int
register_write(struct target *const target,
	unsigned const number,
	uint64_t const value)
{
	riscv_011_info_t *const info = get_info(target);

	maybe_write_tselect(target);

	if (S0 == number) {
		cache_set_load(target, 0, S0, SLOT0);
		cache_set32(target, 1, csrw(S0, CSR_DSCRATCH));
		cache_set_jump(target, 2);
	} else if (S1 == number) {
		cache_set_load(target, 0, S0, SLOT0);
		cache_set_store(target, 1, S0, SLOT_LAST);
		cache_set_jump(target, 2);
	} else if (number <= GDB_REGNO_XPR31) {
		cache_set_load(target, 0, number - GDB_REGNO_ZERO, SLOT0);
		cache_set_jump(target, 1);
	} else if (GDB_REGNO_PC == number) {
		info->dpc = value;
		return ERROR_OK;
	} else if (GDB_REGNO_FPR0 <= number && number <= GDB_REGNO_FPR31) {
		{
			int const error_code = update_mstatus_actual(target);

			if (ERROR_OK != error_code)
				return error_code;
		}

		unsigned i = 0;

		if (0 == (info->mstatus_actual & MSTATUS_FS)) {
			info->mstatus_actual = set_field(info->mstatus_actual, MSTATUS_FS, 1);
			cache_set_load(target, i++, S0, SLOT1);
			cache_set32(target, i++, csrw(S0, CSR_MSTATUS));
			cache_set(target, SLOT1, info->mstatus_actual);
		}

		cache_set32(target,
			i++,
			(32 == riscv_xlen(target) ? flw : fld)(
				number - GDB_REGNO_FPR0,
				0,
				DEBUG_RAM_START + 16)
			);
		cache_set_jump(target, i++);
	} else if (GDB_REGNO_CSR0 <= number && number <= GDB_REGNO_CSR4095) {
		cache_set_load(target, 0, S0, SLOT0);
		cache_set32(target, 1, csrw(S0, number - GDB_REGNO_CSR0));
		cache_set_jump(target, 2);

		if (GDB_REGNO_MSTATUS == number)
			info->mstatus_actual = value;
	} else if (GDB_REGNO_PRIV == number) {
		info->dcsr = set_field(info->dcsr, DCSR_PRV, value);
		return ERROR_OK;
	} else {
		LOG_ERROR("%s: Don't know how to write register %d", target_name(target), number);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	cache_set(target, SLOT0, value);
	{
		int const error_code = cache_write(target, info->dramsize - 1, true);

		if (ERROR_OK != error_code)
			return error_code;
	}

	{
		uint32_t const exception = cache_get32(target, info->dramsize - 1);
		if (exception) {
			LOG_WARNING("%s: Got exception 0x%" PRIx32 " when writing %s",
						target_name(target), exception, gdb_regno_name(number));
			return ERROR_TARGET_FAILURE;
		}
	}

	return ERROR_OK;
}

static int
riscv_011_get_register(struct target *const target,
	riscv_reg_t *value,
	int hartid,
	int regid)
{
	assert(hartid == 0);
	riscv_011_info_t *info = get_info(target);

	maybe_write_tselect(target);

	assert(value);
	if (regid <= GDB_REGNO_XPR31) {
		*value = reg_cache_get(target, regid);
	} else if (regid == GDB_REGNO_PC) {
		*value = info->dpc;
	} else if (GDB_REGNO_FPR0 <= regid && regid <= GDB_REGNO_FPR31) {
		{
			int const error_code = update_mstatus_actual(target);

			if (ERROR_OK != error_code)
				return error_code;
		}

		unsigned i = 0;

		if ((info->mstatus_actual & MSTATUS_FS) == 0) {
			info->mstatus_actual = set_field(info->mstatus_actual, MSTATUS_FS, 1);
			cache_set_load(target, i++, S0, SLOT1);
			cache_set32(target, i++, csrw(S0, CSR_MSTATUS));
			cache_set(target, SLOT1, info->mstatus_actual);
		}

		if (riscv_xlen(target) == 32)
			cache_set32(target, i++, fsw(regid - GDB_REGNO_FPR0, 0, DEBUG_RAM_START + 16));
		else
			cache_set32(target, i++, fsd(regid - GDB_REGNO_FPR0, 0, DEBUG_RAM_START + 16));
		cache_set_jump(target, i++);

		{
			int const error_code = cache_write(target, 4, true);

			if (ERROR_OK != error_code)
				return error_code;
		}
	} else if (regid == GDB_REGNO_PRIV) {
		*value = get_field(info->dcsr, DCSR_PRV);
	} else {
		int const error_code = register_read(target, value, regid);

		if (ERROR_OK != error_code)
			return error_code;
	}

	if (regid == GDB_REGNO_MSTATUS)
		target->reg_cache->reg_list[regid].valid = true;

	return ERROR_OK;
}

static int
riscv_011_set_register(struct target *const target, int hartid, int regid, uint64_t value)
{
	assert(hartid == 0);
	return register_write(target, regid, value);
}

static int riscv_011_halt(struct target *const target)
{
	LOG_DEBUG("%s: riscv_halt()", target_name(target));
	select_dmi(target->tap);

	cache_set32(target, 0, csrsi(CSR_DCSR, DCSR_HALT));
	cache_set32(target, 1, csrr(S0, CSR_MHARTID));
	cache_set32(target, 2, sw(S0, ZERO, SETHALTNOT));
	cache_set_jump(target, 3);

	{
		int const error_code = cache_write(target, 4, true);

		if (ERROR_OK != error_code) {
			LOG_ERROR("%s: cache_write() failed.", target_name(target));
			return error_code;
		}
	}

	return ERROR_OK;
}

static int riscv_011_init_target(struct command_context *cmd_ctx,
		struct target *const target)
{
	LOG_DEBUG("%s: init", target_name(target));
	struct riscv_info_t *const generic_info = target->arch_info;
	generic_info->get_register = riscv_011_get_register;
	generic_info->set_register = riscv_011_set_register;

	generic_info->version_specific = calloc(1, sizeof(riscv_011_info_t));
	if (!generic_info->version_specific)
		LOG_ERROR("%s: Fatal - not enough memory", target_name(target));
		return ERROR_TARGET_INVALID;

	/**
	@todo Now we assume 32-bit until we discover the real value in examine().
	*/
	generic_info->harts[0].xlen = 32;
	riscv_init_registers(target);

	return ERROR_OK;
}

static void riscv_011_deinit_target(struct target *const target)
{
	LOG_DEBUG("%s: riscv_deinit_target()", target_name(target));
	struct riscv_info_t *const info = target->arch_info;
	free(info->version_specific);
	info->version_specific = NULL;
}

static int
riscv_011_examine(struct target *const target)
{
	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */

	uint32_t dtmcontrol;
	{
		int const error_code = dtmcontrol_scan(target->tap, 0, &dtmcontrol);

		if (ERROR_OK != error_code) {
			LOG_ERROR("%s: fatal: examine failure, JTAG/TAP error",
				target_name(target));
			return error_code;
		}
	}
	LOG_DEBUG("%s: dtmcontrol=0x%" PRIx32, target_name(target), dtmcontrol);
	LOG_DEBUG("%s:  addrbits=%" PRId32, target_name(target), get_field(dtmcontrol, DTMCONTROL_ADDRBITS));
	LOG_DEBUG("%s:  version=%"  PRId32, target_name(target), get_field(dtmcontrol, DTMCONTROL_VERSION));
	LOG_DEBUG("%s:  idle=%" PRId32, target_name(target), get_field(dtmcontrol, DTMCONTROL_IDLE));

	if (0 == dtmcontrol) {
		LOG_ERROR("%s: dtmcontrol is 0. Check JTAG connectivity/board power.", target_name(target));
		return ERROR_TARGET_FAILURE;
	}

	if (get_field(dtmcontrol, DTMCONTROL_VERSION) != 0) {
		LOG_ERROR("%s: Unsupported DTM version %" PRId32 ". (dtmcontrol=0x%" PRIx32 ")",
			target_name(target), get_field(dtmcontrol, DTMCONTROL_VERSION), dtmcontrol);
		return ERROR_TARGET_INVALID;
	}

	struct riscv_info_t *const r = riscv_info(target);
	assert(r);
	r->hart_count = 1;

	riscv_011_info_t *const info = get_info(target);
	assert(info);
	info->addrbits = get_field(dtmcontrol, DTMCONTROL_ADDRBITS);
	info->dtmcontrol_idle = get_field(dtmcontrol, DTMCONTROL_IDLE);

	if (0 == info->dtmcontrol_idle) {
		/* Some old SiFive cores don't set idle but need it to be 1. */
		uint32_t idcode;
		{
			int const error_code = idcode_scan(target->tap, &idcode);

			if (ERROR_OK != error_code) {
				LOG_ERROR("%s: fatal: examine failure, JTAG/TAP error",
					target_name(target));
				return error_code;
			}
		}

		if (UINT32_C(0x10E31913) == idcode)
			info->dtmcontrol_idle = 1;
	}

	uint32_t const dminfo = dbus_read(target, DMINFO);
	LOG_DEBUG("%s: dminfo: 0x%08x", target_name(target), dminfo);
	LOG_DEBUG("%s:   abussize=0x%x", target_name(target), get_field(dminfo, DMINFO_ABUSSIZE));
	LOG_DEBUG("%s:   serialcount=0x%x", target_name(target), get_field(dminfo, DMINFO_SERIALCOUNT));
	LOG_DEBUG("%s:   access128=%d", target_name(target), get_field(dminfo, DMINFO_ACCESS128));
	LOG_DEBUG("%s:   access64=%d", target_name(target), get_field(dminfo, DMINFO_ACCESS64));
	LOG_DEBUG("%s:   access32=%d", target_name(target), get_field(dminfo, DMINFO_ACCESS32));
	LOG_DEBUG("%s:   access16=%d", target_name(target), get_field(dminfo, DMINFO_ACCESS16));
	LOG_DEBUG("%s:   access8=%d", target_name(target), get_field(dminfo, DMINFO_ACCESS8));
	LOG_DEBUG("%s:   dramsize=0x%x", target_name(target), get_field(dminfo, DMINFO_DRAMSIZE));
	LOG_DEBUG("%s:   authenticated=0x%x", target_name(target), get_field(dminfo, DMINFO_AUTHENTICATED));
	LOG_DEBUG("%s:   authbusy=0x%x", target_name(target), get_field(dminfo, DMINFO_AUTHBUSY));
	LOG_DEBUG("%s:   authtype=0x%x", target_name(target), get_field(dminfo, DMINFO_AUTHTYPE));
	LOG_DEBUG("%s:   version=0x%x", target_name(target), get_field(dminfo, DMINFO_VERSION));

	if (get_field(dminfo, DMINFO_VERSION) != 1) {
		LOG_ERROR("%s: OpenOCD only supports Debug Module version 1, not %d "
				"(dminfo=0x%x)", target_name(target), get_field(dminfo, DMINFO_VERSION), dminfo);
		return ERROR_TARGET_INVALID;
	}

	info->dramsize = get_field(dminfo, DMINFO_DRAMSIZE) + 1;

	if (get_field(dminfo, DMINFO_AUTHTYPE) != 0) {
		LOG_ERROR("%s: Authentication required by RISC-V core but not "
				"supported by OpenOCD. dminfo=0x%x", target_name(target), dminfo);
		return ERROR_TARGET_INVALID;
	}

	/**
	@todo Pretend this is a 32-bit system until we have found out the true value.
	*/
	r->harts[0].xlen = 32;

	/* Figure out XLEN, and test writing all of Debug RAM while we're at it. */
	cache_set32(target, 0, xori(S1, ZERO, -1));
	/* 0xffffffff  0xffffffff:ffffffff  0xffffffff:ffffffff:ffffffff:ffffffff */
	cache_set32(target, 1, srli(S1, S1, 31));
	/* 0x00000001  0x00000001:ffffffff  0x00000001:ffffffff:ffffffff:ffffffff */
	cache_set32(target, 2, sw(S1, ZERO, DEBUG_RAM_START));
	cache_set32(target, 3, srli(S1, S1, 31));
	/* 0x00000000  0x00000000:00000003  0x00000000:00000003:ffffffff:ffffffff */
	cache_set32(target, 4, sw(S1, ZERO, DEBUG_RAM_START + 4));
	cache_set_jump(target, 5);
	for (unsigned i = 6; i < info->dramsize; ++i)
		cache_set32(target, i, i * 0x01020304);

	cache_write(target, 0, false);

	/* Check that we can actually read/write dram. */
	{
		int const error_code = cache_check(target);

		if (ERROR_OK != error_code)
			return error_code;
	}

	cache_write(target, 0, true);
	cache_invalidate(target);

	uint32_t const word0 = cache_get32(target, 0);
	uint32_t const word1 = cache_get32(target, 1);
	struct riscv_info_t *const generic_info = target->arch_info;
	assert(generic_info);

	if (word0 == 1 && word1 == 0) {
		generic_info->harts[0].xlen = 32;
	} else if (word0 == 0xffffffff && word1 == 3) {
		generic_info->harts[0].xlen = 64;
	} else if (word0 == 0xffffffff && word1 == 0xffffffff) {
		generic_info->harts[0].xlen = 128;
	} else {
		uint32_t const exception = cache_get32(target, info->dramsize-1);
		LOG_ERROR("%s: Failed to discover xlen; word0=0x%x, word1=0x%x, exception=0x%x",
			target_name(target), word0, word1, exception);
		return dump_debug_ram(target);
	}

	LOG_DEBUG("%s: Discovered XLEN is %d", target_name(target), riscv_xlen(target));

	if (ERROR_OK != read_csr(target, &r->harts[0].misa, CSR_MISA)) {
		static unsigned const old_csr_misa = 0xf10;
		LOG_WARNING("%s: Failed to read misa at 0x%x; trying 0x%x.", target_name(target), CSR_MISA,
				old_csr_misa);
		{
			int const error_code = read_csr(target, &r->harts[0].misa, old_csr_misa);

			if (ERROR_OK != error_code) {
				/* Maybe this is an old core that still has $misa at the old
				 * address. */
				LOG_ERROR("%s: Failed to read misa at 0x%x.", target_name(target), old_csr_misa);
				return ERROR_TARGET_INVALID;
			}
		}
	}

	{
		/* Update register list to match discovered XLEN/supported extensions. */
		int const error_code = riscv_init_registers(target);

		if (ERROR_OK != error_code)
			return error_code;
	}

	info->never_halted = true;

	{
		int const error_code = riscv_011_poll(target);

		if (ERROR_OK != error_code)
			return error_code;
	}

	target_set_examined(target);

	{
		int const error_code = riscv_set_current_hartid(target, 0);

		if (ERROR_OK != error_code)
			return error_code;
	}

	for (size_t i = 0; i < 32; ++i)
		reg_cache_set(target, i, -1);

	LOG_INFO("%s: Examined RISCV core; XLEN=%d, misa=0x%" PRIx64,
		target_name(target), riscv_xlen(target), r->harts[0].misa);

	return ERROR_OK;
}

static riscv_error_t
handle_halt_routine(struct target *const target)
{
	riscv_011_info_t *const info = get_info(target);

	scans_t *scans = scans_new(target, 256);

	/* Read all GPRs as fast as we can, because gdb is going to ask for them
	 * anyway. Reading them one at a time is much slower. */

	/* Write the jump back to address 1. */
	scans_add_write_jump(scans, 1, false);
	for (int reg = 1; reg < 32; ++reg) {
		if (reg == S0 || reg == S1)
			continue;

		/* Write store instruction. */
		scans_add_write_store(scans, 0, reg, SLOT0, true);

		/* Read value. */
		scans_add_read(scans, SLOT0, false);
	}

	/* Write store of s0 at index 1. */
	scans_add_write_store(scans, 1, S0, SLOT0, false);
	/* Write jump at index 2. */
	scans_add_write_jump(scans, 2, false);

	/* Read S1 from debug RAM */
	scans_add_write_load(scans, 0, S0, SLOT_LAST, true);
	/* Read value. */
	scans_add_read(scans, SLOT0, false);

	/* Read S0 from dscratch */
	unsigned csr[] = {CSR_DSCRATCH, CSR_DPC, CSR_DCSR};

	for (unsigned i = 0; i < DIM(csr); ++i) {
		scans_add_write32(scans, 0, csrr(S0, csr[i]), true);
		scans_add_read(scans, SLOT0, false);
	}

	/* Final read to get the last value out. */
	scans_add_read32(scans, 4, false);

	{
		int error_code = scans_execute(scans);

		if (ERROR_OK != error_code) {
			LOG_ERROR("%s: JTAG execute failed: %d", target_name(target), error_code);
			goto error;
		}
	}

	unsigned dbus_busy = 0;
	unsigned interrupt_set = 0;
	unsigned result = 0;
	uint64_t value = 0;
	reg_cache_set(target, 0, 0);

	/* The first scan result is the result from something old we don't care about. */
	for (unsigned i = 1; i < scans->next_scan && dbus_busy == 0; ++i) {
		dbus_status_t status =
			scans_get_u32(scans, i, DBUS_OP_START, DBUS_OP_SIZE);
		uint64_t data = scans_get_u64(scans, i, DBUS_DATA_START, DBUS_DATA_SIZE);
		uint32_t address =
			scans_get_u32(scans, i, DBUS_ADDRESS_START, info->addrbits);

		switch (status) {
			case DBUS_STATUS_SUCCESS:
				break;

			case DBUS_STATUS_FAILED:
				LOG_ERROR("%s: Debug access failed. Hardware error?", target_name(target));
				goto error;

			case DBUS_STATUS_BUSY:
				++dbus_busy;
				break;

			default:
				LOG_ERROR("%s: Got invalid bus access status: %d", target_name(target), status);
				goto error;
		}

		if (data & DMCONTROL_INTERRUPT) {
			++interrupt_set;
			break;
		}

		if (address == 4 || address == 5) {
			unsigned reg;

			switch (result) {
				case 0:
					reg = 1;
					break;

				case 1:
					reg = 2;
					break;

				case 2:
					reg = 3;
					break;

				case 3:
					reg = 4;
					break;

				case 4:
					reg = 5;
					break;

				case 5:
					reg = 6;
					break;

				case 6:
					reg = 7;
					break;

					/* S0 */
					/* S1 */
				case 7:
					reg = 10;
					break;

				case 8:
					reg = 11;
					break;

				case 9:
					reg = 12;
					break;

				case 10:
					reg = 13;
					break;

				case 11:
					reg = 14;
					break;

				case 12:
					reg = 15;
					break;

				case 13:
					reg = 16;
					break;

				case 14:
					reg = 17;
					break;

				case 15:
					reg = 18;
					break;

				case 16:
					reg = 19;
					break;

				case 17:
					reg = 20;
					break;

				case 18:
					reg = 21;
					break;

				case 19:
					reg = 22;
					break;

				case 20:
					reg = 23;
					break;

				case 21:
					reg = 24;
					break;

				case 22:
					reg = 25;
					break;

				case 23:
					reg = 26;
					break;

				case 24:
					reg = 27;
					break;

				case 25:
					reg = 28;
					break;

				case 26:
					reg = 29;
					break;

				case 27:
					reg = 30;
					break;

				case 28:
					reg = 31;
					break;

				case 29:
					reg = S1;
					break;

				case 30:
					reg = S0;
					break;

				case 31:
					reg = CSR_DPC;
					break;

				case 32:
					reg = CSR_DCSR;
					break;

				default:
					assert(0);
					LOG_ERROR("%s: Got invalid register result %d", target_name(target), result);
					goto error;
			}

			if (riscv_xlen(target) == 32) {
				reg_cache_set(target, reg, data & 0xffffffff);
				++result;
			} else if (riscv_xlen(target) == 64) {
				if (address == 4) {
					value = data & 0xffffffff;
				} else if (address == 5) {
					reg_cache_set(target, reg, ((data & 0xffffffff) << 32) | value);
					value = 0;
					++result;
				}
			}
		}
	}

	scans_delete(scans);

	if (dbus_busy) {
		increase_dbus_busy_delay(target);
		return RE_AGAIN;
	}
	if (interrupt_set) {
		increase_interrupt_high_delay(target);
		return RE_AGAIN;
	}

	/**
	@todo get rid of those 2 variables and talk to the cache directly.
	*/
	info->dpc = reg_cache_get(target, CSR_DPC);
	info->dcsr = reg_cache_get(target, CSR_DCSR);

	cache_invalidate(target);

	return RE_OK;

error:
	scans_delete(scans);
	return RE_FAIL;
}

static int
handle_halt(struct target *const target,
	bool const announce)
{
	assert(target);
	target->state = TARGET_HALTED;

	riscv_error_t re;
	do {
		/**
		@bug Is possible infinite loop?
		*/
		re = handle_halt_routine(target);
	} while (RE_AGAIN == re);

	if (re != RE_OK) {
		LOG_ERROR("%s: handle_halt_routine failed", target_name(target));
		return ERROR_TARGET_FAILURE;
	}

	riscv_011_info_t *const info = get_info(target);
	assert(info);
	int const cause = get_field(info->dcsr, DCSR_CAUSE);

	switch (cause) {
		case DCSR_CAUSE_SWBP:
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;

		case DCSR_CAUSE_HWBP:
			target->debug_reason = DBG_REASON_WATCHPOINT;
			/* If we halted because of a data trigger, gdb doesn't know to do
			 * the disable-breakpoints-step-enable-breakpoints dance. */
			info->need_strict_step = true;
			break;

		case DCSR_CAUSE_DEBUGINT:
			target->debug_reason = DBG_REASON_DBGRQ;
			break;

		case DCSR_CAUSE_STEP:
			target->debug_reason = DBG_REASON_SINGLESTEP;
			break;

		case DCSR_CAUSE_HALT:
		default:
			LOG_ERROR("%s: Invalid halt cause %d in DCSR (0x%" PRIx64 ")", target_name(target),
					cause, info->dcsr);
	}

	if (info->never_halted) {
		info->never_halted = false;

		{
			int error_code = maybe_read_tselect(target);

			if (ERROR_OK != error_code)
				return error_code;
		}

		riscv_enumerate_triggers(target);
	}

	if (target->debug_reason == DBG_REASON_BREAKPOINT) {
		int retval;
		if (riscv_semihosting(target, &retval) != 0)
			return retval;
	}

	if (announce)
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	char const *const cause_string[] = {
		"none",
		"software breakpoint",
		"hardware trigger",
		"debug interrupt",
		"step",
		"halt"
	};

	/* This is logged to the user so that gdb will show it when a user types
	 * 'monitor reset init'. At that time gdb appears to have the pc cached
	 * still so if a user manually inspects the pc it will still have the old
	 * value. */
	LOG_USER("%s: halted at 0x%" PRIx64 " due to %s",
		target_name(target),
		info->dpc,
		cause_string[cause]);

	return ERROR_OK;
}

static int
poll_target(struct target *const target,
	bool const announce)
{
	select_dmi(target->tap);

	/* Inhibit debug logging during poll(), which isn't usually interesting and
	 * just fills up the screen/logs with clutter. */
	int old_debug_level = debug_level;
	if (debug_level >= LOG_LVL_DEBUG)
		debug_level = LOG_LVL_INFO;
	bits_t bits = read_bits(target);
	debug_level = old_debug_level;

	if (bits.haltnot && bits.interrupt) {
		target->state = TARGET_DEBUG_RUNNING;
		LOG_DEBUG("%s: debug running", target_name(target));
	} else if (bits.haltnot && !bits.interrupt) {
		if (target->state != TARGET_HALTED)
			return handle_halt(target, announce);
	} else if (!bits.haltnot && bits.interrupt) {
		/* Target is halting. There is no state for that, so don't change anything. */
		LOG_DEBUG("%s: halting", target_name(target));
	} else if (!bits.haltnot && !bits.interrupt) {
		target->state = TARGET_RUNNING;
	}

	return ERROR_OK;
}

/* Execute a step, and wait for reentry into Debug Mode. */
static int full_step(struct target *const target, bool announce)
{
	{
		int const error_code = execute_resume(target, true);

		if (ERROR_OK != error_code)
			return error_code;
	}

	time_t const start = time(NULL);

	for (;;) {
		{
			int const error_code = poll_target(target, announce);

			if (ERROR_OK != error_code)
				return error_code;
		}

		if (TARGET_DEBUG_RUNNING != target->state)
			break;

		if (riscv_command_timeout_sec + start < time(NULL)) {
			LOG_ERROR("%s: Timed out waiting for step to complete."
					"Increase timeout with riscv set_command_timeout_sec",
					target_name(target));
			return ERROR_TARGET_TIMEOUT;
		}
	}

	return ERROR_OK;
}

static int
strict_step(struct target *const target, bool announce)
{
	riscv_011_info_t *const info = get_info(target);

	LOG_DEBUG("%s: enter", target_name(target));

	for (struct watchpoint *watchpoint = target->watchpoints; watchpoint; watchpoint = watchpoint->next) {
		target_remove_watchpoint(target, watchpoint);
	}

	{
		int const error_code = full_step(target, announce);

		if (ERROR_OK != error_code)
			return error_code;
	}

	for (struct watchpoint *watchpoint = target->watchpoints; watchpoint; watchpoint = watchpoint->next) {
		target_add_watchpoint(target, watchpoint);
	}

	info->need_strict_step = false;

	return ERROR_OK;
}

static int
riscv_011_step(struct target *const target,
	int const current,
	target_addr_t const address,
	int const handle_breakpoints)
{
	select_dmi(target->tap);

	if (!current) {
		if (riscv_xlen(target) > 32) {
			LOG_WARNING("%s: Asked to resume at 32-bit PC on %d-bit target.",
						target_name(target),
						riscv_xlen(target));
		}
		{
			int const error_code = register_write(target, GDB_REGNO_PC, address);

			if (ERROR_OK != error_code)
				return error_code;
		}
	}

	riscv_011_info_t *const info = get_info(target);
	assert(info);

	return
		info->need_strict_step || handle_breakpoints ? strict_step(target, true) :
		full_step(target, false);
}

static int
riscv_011_poll(struct target *const target)
{
	return poll_target(target, true);
}

static int
riscv_011_resume(struct target *const target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution)
{
	riscv_011_info_t *info = get_info(target);

	select_dmi(target->tap);

	if (!current) {
		if (riscv_xlen(target) > 32) {
			LOG_WARNING("%s: Asked to resume at 32-bit PC on %d-bit target.", target_name(target),
					riscv_xlen(target));
		}

		{
			int const error_code = register_write(target, GDB_REGNO_PC, address);

			if (ERROR_OK != error_code)
				return error_code;
		}
	}

	if (info->need_strict_step || handle_breakpoints) {
		int const error_code = strict_step(target, false);

		if (ERROR_OK != error_code)
			return error_code;
	}

	return resume(target, debug_execution, false);
}

static int riscv_011_assert_reset(struct target *const target)
{
	assert(target);

	/**
	@todo Maybe what I implemented here is more like soft_reset_halt()?
	*/
	select_dmi(target->tap);

	{
		/* The only assumption we can make is that the TAP was reset. */
		int const error_code = wait_for_debugint_clear(target, true);

		if (ERROR_OK != error_code) {
			LOG_ERROR("%s: Debug interrupt didn't clear.", target_name(target));
			return error_code;
		}
	}

	riscv_011_info_t *info = get_info(target);
	assert(info);

	/* Not sure what we should do when there are multiple cores.
	 * Here just reset the single hart we're talking to. */
	info->dcsr |=
		DCSR_EBREAKM | DCSR_EBREAKH | DCSR_EBREAKS | DCSR_EBREAKU | DCSR_HALT;

	info->dcsr |= target->reset_halt ? DCSR_NDRESET : DCSR_FULLRESET;
	dram_write32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16), false);
	dram_write32(target, 1, csrw(S0, CSR_DCSR), false);

	/* We shouldn't actually need the jump because a reset should happen. */
	dram_write_jump(target, 2, false);
	dram_write32(target, 4, info->dcsr, true);
	cache_invalidate(target);

	target->state = TARGET_RESET;

	return ERROR_OK;
}

static int
riscv_011_deassert_reset(struct target *const target)
{
	select_dmi(target->tap);
	return wait_for_state(target, target->reset_halt ? TARGET_HALTED : TARGET_RUNNING);
}

static int
riscv_011_read_memory(struct target *const target,
	target_addr_t const address,
	uint32_t const size,
	uint32_t const count,
	uint8_t *const buffer)
{
	select_dmi(target->tap);

	cache_set32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16));

	int error_code = ERROR_OK;

	switch (size) {
		case 1:
			cache_set32(target, 1, lb(S1, S0, 0));
			cache_set32(target, 2, sw(S1, ZERO, DEBUG_RAM_START + 16));
			break;

		case 2:
			cache_set32(target, 1, lh(S1, S0, 0));
			cache_set32(target, 2, sw(S1, ZERO, DEBUG_RAM_START + 16));
			break;

		case 4:
			cache_set32(target, 1, lw(S1, S0, 0));
			cache_set32(target, 2, sw(S1, ZERO, DEBUG_RAM_START + 16));
			break;

		default:
			LOG_ERROR("%s: Unsupported size: %d", target_name(target), size);
			return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	cache_set_jump(target, 3);
	cache_write(target, CACHE_NO_READ, false);

	riscv_011_info_t *info = get_info(target);
	const unsigned max_batch_size = 256;
	scans_t *scans = scans_new(target, max_batch_size);

	uint32_t result_value = 0x777;
	uint32_t i = 0;
	while (i < count + 3) {
		unsigned batch_size = MIN(count + 3 - i, max_batch_size);
		scans_reset(scans);

		for (unsigned j = 0; j < batch_size; ++j) {
			if (i + j == count) {
				/* Just insert a read so we can scan out the last value. */
				scans_add_read32(scans, 4, false);
			} else if (i + j >= count + 1) {
				/* And check for errors. */
				scans_add_read32(scans, info->dramsize-1, false);
			} else {
				/* Write the next address and set interrupt. */
				uint32_t offset = size * (i + j);
				scans_add_write32(scans, 4, address + offset, true);
			}
		}

		{
			int const error_code_1 = scans_execute(scans);

			if (ERROR_OK != error_code_1) {
				LOG_ERROR("%s: JTAG execute failed: %d", target_name(target), error_code_1);
				scans_delete(scans);
				cache_clean(target);
				return error_code_1;
			}
		}

		int dbus_busy = 0;
		int execute_busy = 0;
		for (unsigned j = 0; j < batch_size; ++j) {
			dbus_status_t const status =
				scans_get_u32(scans, j, DBUS_OP_START, DBUS_OP_SIZE);

			switch (status) {
				case DBUS_STATUS_SUCCESS:
					break;

				case DBUS_STATUS_FAILED:
					LOG_ERROR("%s: Debug RAM write failed. Hardware error?", target_name(target));
					goto error;

				case DBUS_STATUS_BUSY:
					++dbus_busy;
					break;

				default:
					LOG_ERROR("%s: Got invalid bus access status: %d", target_name(target), status);
					return ERROR_TARGET_FAILURE;
			}

			uint64_t const data = scans_get_u64(scans, j, DBUS_DATA_START, DBUS_DATA_SIZE);

			if (0 != (data & DMCONTROL_INTERRUPT))
				++execute_busy;

			if (i + j == count + 2) {
				result_value = data;
			} else if (i + j > 1) {
				uint32_t const offset = size * (i + j - 2);

				switch (size) {
					case 1:
						buffer[offset] = data;
						break;

					case 2:
						buffer[offset + 0] = (uint8_t)(data >> 0 * CHAR_BIT);
						buffer[offset + 1] = (uint8_t)(data >> 1 * CHAR_BIT);
						break;

					case 4:
						buffer[offset + 0] = (uint8_t)(data >> 0 * CHAR_BIT);
						buffer[offset + 1] = (uint8_t)(data >> 1 * CHAR_BIT);
						buffer[offset + 2] = (uint8_t)(data >> 2 * CHAR_BIT);
						buffer[offset + 3] = (uint8_t)(data >> 3 * CHAR_BIT);
						break;

					/**
					@bug no default case
					*/
				}
			}

			LOG_DEBUG("%s: j=%d status=%d data=%09" PRIx64, target_name(target), j, status, data);
		}

		if (dbus_busy)
			increase_dbus_busy_delay(target);

		if (execute_busy)
			increase_interrupt_high_delay(target);

		if (dbus_busy || execute_busy) {
			wait_for_debugint_clear(target, false);

			/* Retry. */
			LOG_INFO("%s: Retrying memory read starting from 0x%" TARGET_PRIxADDR
					" with more delays", target_name(target), address + size * i);
		} else {
			i += batch_size;
		}
	}

	if (result_value != 0) {
		LOG_USER("%s: Core got an exception (0x%x) while reading from 0x%"
				TARGET_PRIxADDR, target_name(target), result_value, address + size * (count - 1));
		if (count > 1) {
			LOG_USER("%s: (It may have failed between 0x%" TARGET_PRIxADDR
					" and 0x%" TARGET_PRIxADDR " as well, but we "
					"didn't check then.)", target_name(target),
					address, address + size * (count-2) + size - 1);
		}
		goto error;
	}
	goto finish;

error:
	error_code = ERROR_TARGET_FAILURE;

finish:
	scans_delete(scans);
	cache_clean(target);
	return error_code;
}

static int setup_write_memory(struct target *const target, uint32_t size)
{
	switch (size) {
		case 1:
			cache_set32(target, 0, lb(S0, ZERO, DEBUG_RAM_START + 16));
			cache_set32(target, 1, sb(S0, T0, 0));
			break;

		case 2:
			cache_set32(target, 0, lh(S0, ZERO, DEBUG_RAM_START + 16));
			cache_set32(target, 1, sh(S0, T0, 0));
			break;

		case 4:
			cache_set32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16));
			cache_set32(target, 1, sw(S0, T0, 0));
			break;

		default:
			LOG_ERROR("%s: Unsupported size: %d", target_name(target), size);
			return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	cache_set32(target, 2, addi(T0, T0, size));
	cache_set_jump(target, 3);
	cache_write(target, 4, false);

	return ERROR_OK;
}

static int
riscv_011_write_memory(struct target *const target,
	target_addr_t const address,
	uint32_t const size,
	uint32_t const count,
	uint8_t const *const buffer)
{
	riscv_011_info_t *info = get_info(target);
	select_dmi(target->tap);

	/* Set up the address. */
	cache_set_store(target, 0, T0, SLOT1);
	cache_set_load(target, 1, T0, SLOT0);
	cache_set_jump(target, 2);
	cache_set(target, SLOT0, address);
	{
		int const error_code = cache_write(target, 5, true);

		if (ERROR_OK != error_code)
			return error_code;
	}

	uint64_t const t0 = cache_get(target, SLOT1);
	LOG_DEBUG("%s: t0 is 0x%" PRIx64, target_name(target), t0);

	{
		int const error_code = setup_write_memory(target, size);

		if (ERROR_OK != error_code)
			return error_code;
	}

	static unsigned const max_batch_size = 256;
	scans_t *const scans = scans_new(target, max_batch_size);

	uint32_t result_value = 0x777;

	for (uint32_t i = 0; i < count + 2;) {
		unsigned batch_size = MIN(count + 2 - i, max_batch_size);
		scans_reset(scans);

		for (unsigned j = 0; j < batch_size; ++j) {
			if (i + j >= count) {
				/* Check for an exception. */
				scans_add_read32(scans, info->dramsize-1, false);
			} else {
				/* Write the next value and set interrupt. */
				uint32_t value;
				uint32_t offset = size * (i + j);

				switch (size) {
					case 1:
						value = (uint32_t)(buffer[offset]) << 0 * CHAR_BIT;
						break;

					case 2:
						value =
							(uint32_t)(buffer[offset + 0]) << 0 * CHAR_BIT | 
							(uint32_t)(buffer[offset + 1]) << 1 * CHAR_BIT;
						break;

					case 4:
						value =
							(uint32_t)(buffer[offset + 0]) << 0 * CHAR_BIT |
							(uint32_t)(buffer[offset + 1]) << 1 * CHAR_BIT |
							(uint32_t)(buffer[offset + 2]) << 2 * CHAR_BIT |
							(uint32_t)(buffer[offset + 3]) << 3 * CHAR_BIT;
						break;

					default:
						goto error;
				}

				scans_add_write32(scans, 4, value, true);
			}
		}

		{
			int const error_code = scans_execute(scans);

			if (ERROR_OK != error_code) {
				LOG_ERROR("%s: JTAG execute failed: %d", target_name(target), error_code);
				scans_delete(scans);
				cache_clean(target);
				return error_code;
			}
		}

		int dbus_busy = 0;
		int execute_busy = 0;

		for (unsigned j = 0; j < batch_size; ++j) {
			dbus_status_t status = scans_get_u32(scans, j, DBUS_OP_START,
					DBUS_OP_SIZE);

			switch (status) {
				case DBUS_STATUS_SUCCESS:
					break;

				case DBUS_STATUS_FAILED:
					LOG_ERROR("%s: Debug RAM write failed. Hardware error?", target_name(target));
					goto error;

				case DBUS_STATUS_BUSY:
					++dbus_busy;
					break;

				default:
					LOG_ERROR("%s: Got invalid bus access status: %d", target_name(target), status);
					return ERROR_TARGET_FAILURE;
			}

			int const interrupt = scans_get_u32(scans, j, DBUS_DATA_START + 33, 1);

			if (interrupt)
				++execute_busy;

			if (i + j == count + 1)
				result_value = scans_get_u32(scans, j, DBUS_DATA_START, 32);
		}

		if (dbus_busy)
			increase_dbus_busy_delay(target);

		if (execute_busy)
			increase_interrupt_high_delay(target);

		if (dbus_busy || execute_busy) {
			wait_for_debugint_clear(target, false);

			/* Retry.
			 * Set t0 back to what it should have been at the beginning of this
			 * batch. */
			LOG_INFO("%s: Retrying memory write starting from 0x%" TARGET_PRIxADDR " with more delays",
				target_name(target), address + size * i);

			cache_clean(target);

			{
				int const error_code = write_gpr(target, T0, address + size * i);

				if (ERROR_OK != error_code) {
					scans_delete(scans);
					cache_clean(target);
					return error_code;
				}
			}

			{
				int const error_code = setup_write_memory(target, size);

				if (ERROR_OK != error_code) {
					scans_delete(scans);
					cache_clean(target);
					return error_code;
				}
			}
		} else {
			i += batch_size;
		}
	}

	if (result_value != 0) {
		LOG_ERROR("%s: Core got an exception (0x%x) while writing to 0x%" TARGET_PRIxADDR,
			target_name(target), result_value, address + size * (count - 1));

		if (count > 1) {
			LOG_ERROR("%s: (It may have failed between 0x%" TARGET_PRIxADDR
				" and 0x%" TARGET_PRIxADDR
				" as well, but we didn't check then.)",
				target_name(target), address, address + size * (count - 2) + size - 1);
		}
		goto error;
	}

	scans_delete(scans);
	cache_clean(target);
	return register_write(target, T0, t0);

error:
	scans_delete(scans);
	cache_clean(target);
	return ERROR_TARGET_FAILURE;
}

static int
riscv_011_arch_state(struct target *const target)
{
	return ERROR_OK;
}

struct target_type const riscv_011_target = {
	.name = "riscv",

	.init_target = riscv_011_init_target,
	.deinit_target = riscv_011_deinit_target,
	.examine = riscv_011_examine,

	/* poll current target status */
	.poll = riscv_011_poll,

	.halt = riscv_011_halt,
	.resume = riscv_011_resume,
	.step = riscv_011_step,

	.assert_reset = riscv_011_assert_reset,
	.deassert_reset = riscv_011_deassert_reset,

	.read_memory = riscv_011_read_memory,
	.write_memory = riscv_011_write_memory,

	.arch_state = riscv_011_arch_state,
};
