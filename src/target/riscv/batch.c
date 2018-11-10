#include "batch.h"

#include "debug_defines.h"
#include "riscv.h"

#include "jtag/jtag.h"

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

static void
dump_field(struct scan_field const *const field)
{
	static const char *const op_string[] = {"-", "r", "w", "?"};
	static const char *const status_string[] = {"+", "?", "F", "b"};

	if (debug_level < LOG_LVL_DEBUG)
		return;

	assert(field->out_value != NULL);
	uint64_t const out = buf_get_u64(field->out_value, 0, field->num_bits);
	unsigned const out_op = get_field(out, DTM_DMI_OP);
	unsigned const out_data = get_field(out, DTM_DMI_DATA);
	unsigned const out_address = out >> DTM_DMI_ADDRESS_OFFSET;

	if (field->in_value) {
		uint64_t const in = buf_get_u64(field->in_value, 0, field->num_bits);
		unsigned const in_op = get_field(in, DTM_DMI_OP);
		unsigned const in_data = get_field(in, DTM_DMI_DATA);
		unsigned const in_address = in >> DTM_DMI_ADDRESS_OFFSET;

		log_printf_lf(LOG_LVL_DEBUG,
			__FILE__, __LINE__, __PRETTY_FUNCTION__,
			"%db %s %08x @%02x -> %s %08x @%02x",
			field->num_bits,
			op_string[out_op], out_data, out_address,
			status_string[in_op], in_data, in_address);
	} else {
		log_printf_lf(LOG_LVL_DEBUG,
			__FILE__, __LINE__, __PRETTY_FUNCTION__, "%db %s %08x @%02x -> ?",
			field->num_bits, op_string[out_op], out_data, out_address);
	}
}

struct riscv_batch *
	riscv_batch_alloc(struct target *const target,
		size_t scans,
		size_t const idle)
{
	scans += 4;
	struct riscv_batch *out = malloc(sizeof(*out));
	assert(out);
	memset(out, 0, sizeof(*out));
	out->target = target;
	out->allocated_scans = scans;
	out->used_scans = 0;
	out->idle_count = idle;
	out->data_out = malloc(sizeof(*out->data_out) * (scans) * sizeof(uint64_t));
	out->data_in  = malloc(sizeof(*out->data_in)  * (scans) * sizeof(uint64_t));
	out->fields = malloc(sizeof(*out->fields) * (scans));
	out->last_scan = RISCV_SCAN_TYPE_INVALID;
	out->read_keys = malloc(sizeof(*out->read_keys) * (scans));
	out->read_keys_used = 0;
	return out;
}

void
riscv_batch_free(struct riscv_batch *batch)
{
	free(batch->data_in);
	free(batch->data_out);
	free(batch->fields);
	free(batch);
}

bool
riscv_batch_full(struct riscv_batch *batch)
{
	return batch->used_scans > (batch->allocated_scans - 4);
}

int
riscv_batch_run(struct riscv_batch *batch)
{
	assert(batch && batch->target);

	if (batch->used_scans == 0) {
		LOG_DEBUG("%s: Ignoring empty batch.", batch->target->cmd_name);
		return ERROR_OK;
	}

	keep_alive();

	riscv_batch_add_nop(batch);

	for (size_t i = 0; i < batch->used_scans; ++i) {
		jtag_add_dr_scan(batch->target->tap, 1, batch->fields + i, TAP_IDLE);
		if (batch->idle_count > 0)
			jtag_add_runtest(batch->idle_count, TAP_IDLE);
	}

	int const result = jtag_execute_queue();
	if (ERROR_OK != result) {
		LOG_ERROR("%s: Unable to execute JTAG queue", batch->target->cmd_name);
		return result;
	}

	for (size_t i = 0; i < batch->used_scans; ++i)
		dump_field(batch->fields + i);

	return ERROR_OK;
}

void
riscv_batch_add_dmi_write(struct riscv_batch *const batch,
	unsigned const address,
	uint64_t const data)
{
	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *field = batch->fields + batch->used_scans;
	field->num_bits = riscv_dmi_write_u64_bits(batch->target);
	uint8_t *p_tmp = batch->data_out + batch->used_scans * sizeof(uint64_t);
	field->out_value = p_tmp;
	field->in_value  = batch->data_in  + batch->used_scans * sizeof(uint64_t);
	riscv_fill_dmi_write_u64(batch->target, p_tmp, address, data);
	riscv_fill_dmi_nop_u64(batch->target, field->in_value);
	batch->last_scan = RISCV_SCAN_TYPE_WRITE;
	++batch->used_scans;
}

size_t
riscv_batch_add_dmi_read(struct riscv_batch *const batch,
	unsigned const address)
{
	assert(batch && batch->used_scans < batch->allocated_scans);
	struct scan_field *const field = batch->fields + batch->used_scans;
	field->num_bits = riscv_dmi_write_u64_bits(batch->target);
	uint8_t *p_tmp = batch->data_out + batch->used_scans * sizeof(uint64_t);
	field->out_value = p_tmp;
	field->in_value  = batch->data_in  + batch->used_scans * sizeof(uint64_t);
	riscv_fill_dmi_read_u64(batch->target, p_tmp, address);
	riscv_fill_dmi_nop_u64(batch->target, field->in_value);
	batch->last_scan = RISCV_SCAN_TYPE_READ;
	++batch->used_scans;

	/* FIXME We get the read response back on the next scan.  For now I'm
	 * just sticking a NOP in there, but this should be coelesced away. */
	riscv_batch_add_nop(batch);

	batch->read_keys[batch->read_keys_used] = batch->used_scans - 1;
	return ++batch->read_keys_used;
}

uint64_t
riscv_batch_get_dmi_read(struct riscv_batch const *const batch,
	size_t const key)
{
	assert(key < batch->read_keys_used);
	size_t index = batch->read_keys[key];
	assert(index <= batch->used_scans);
	uint8_t const *const base = batch->data_in + 8 * index;
	return
		(uint64_t)base[0] << 0 * CHAR_BIT |
		(uint64_t)base[1] << 1 * CHAR_BIT |
		(uint64_t)base[2] << 2 * CHAR_BIT |
		(uint64_t)base[3] << 3 * CHAR_BIT |
		(uint64_t)base[4] << 4 * CHAR_BIT |
		(uint64_t)base[5] << 5 * CHAR_BIT |
		(uint64_t)base[6] << 6 * CHAR_BIT |
		(uint64_t)base[7] << 7 * CHAR_BIT;
}

void riscv_batch_add_nop(struct riscv_batch *batch)
{
	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *field = batch->fields + batch->used_scans;
	field->num_bits = riscv_dmi_write_u64_bits(batch->target);
	uint8_t *p_tmp = batch->data_out + batch->used_scans * sizeof(uint64_t);
	field->in_value  = batch->data_in  + batch->used_scans * sizeof(uint64_t);
	field->out_value = p_tmp;
	riscv_fill_dmi_nop_u64(batch->target, p_tmp);
	riscv_fill_dmi_nop_u64(batch->target, field->in_value);
	batch->last_scan = RISCV_SCAN_TYPE_NOP;
	++batch->used_scans;
}

