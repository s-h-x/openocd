#ifndef TARGET_RISCV_SCANS_H_
#define TARGET_RISCV_SCANS_H_

#include "target/target.h"

enum riscv_scan_type_e {
	RISCV_SCAN_TYPE_INVALID,
	RISCV_SCAN_TYPE_NOP,
	RISCV_SCAN_TYPE_READ,
	RISCV_SCAN_TYPE_WRITE,
};

/** A batch of multiple JTAG scans, which are grouped together to avoid the
 * overhead of some JTAG adapters when sending single commands.  This is
 * designed to support block copies, as that's what we actually need to go
 * fast. */
struct riscv_batch {
	struct target *target;

	size_t allocated_scans;
	size_t used_scans;

	size_t idle_count;

	uint8_t *data_out;
	uint8_t *data_in;
	struct scan_field *fields;

	/** In JTAG we scan out the previous value's output when performing a
	 * scan.  This is a pain for users, so we just provide them the
	 * illusion of not having to do this by eliding all but the last NOP.
	 * */
	enum riscv_scan_type_e last_scan;

	/* The read keys. */
	size_t *read_keys;
	size_t read_keys_used;
};

/**	Allocates (or frees) a new scan set.

	"scans" is the maximum number of JTAG scans that can be issued to this object,
	and idle is the number of JTAG idle cycles between every real scan.
*/
struct riscv_batch *
	__attribute__((warn_unused_result))
	riscv_batch_alloc(struct target *target, size_t scans, size_t idle);

void
riscv_batch_free(struct riscv_batch *batch);

/** Checks to see if this batch is full. */
static inline bool
riscv_batch_full(struct riscv_batch const *const batch)
{
	return batch->allocated_scans < batch->used_scans + 4;
}

/** Executes this scan batch. */
int
__attribute__((warn_unused_result))
riscv_batch_run(struct riscv_batch *batch);

/** Adds a DMI write to this batch. */
void
riscv_batch_add_dmi_write(struct riscv_batch *batch,
	unsigned address,
	uint64_t data);

/**	DMI reads must be handled in two parts:
	the first one schedules a read and provides a key,
	the second one actually obtains the value of that read.
*/
/**@{*/
size_t riscv_batch_add_dmi_read(struct riscv_batch *batch, unsigned address);
uint64_t riscv_batch_get_dmi_read(struct riscv_batch const *batch, size_t key);
/**@}*/

#endif  /* TARGET_RISCV_SCANS_H_ */
