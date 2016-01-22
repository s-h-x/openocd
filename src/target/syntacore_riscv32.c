#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <helper/jim.h>
#include <helper/log.h>
#include <helper/binarybuffer.h>
#include <jtag/jtag.h>

#include "target.h"
#include "target_type.h"
#include "breakpoints.h"

#include <helper/types.h>
#include "register.h"

#define CHECK_MEMORY_TRANSACTIONS 0
#define FAST_LOAD 1

static uint32_t const DEBUG_MEMORY_ACCESS_CMD = 0x304;
static uint32_t const DEBUG_MEMORY_ACCESS_CMD_START = 0x2;
static uint32_t const DEBUG_MEMORY_ACCESS_CMD_WRITE = 0x4;
static uint32_t const DEBUG_MEMORY_ACCESS_CMD_SEQ = 0x108;

static uint32_t const DEBUG_MEMORY_ACCESS_CMD_ALL_BYTES = 0xF0;

static uint32_t const DEBUG_MEMORY_ACCESS_COUNT = 0x314;

static uint32_t const DEBUG_MEMORY_ACCESS_ADDRESS = 0x308;
static uint32_t const DEBUG_MEMORY_ACCESS_WR_DATA = 0x30C;
static uint32_t const DEBUG_MEMORY_ACCESS_RD_DATA = 0x310;

static uint32_t const DEBUG_REGISTERS_ACCESS_BASE = 0x200;

static uint32_t const DEBUG_CONTROL_STATUS = 0x100;
static uint32_t const DEBUG_CONTROL_STATUS_HALTED = 0x40000000;
static uint32_t const DEBUG_CONTROL_STATUS_IRQ_DISABLE = 0x00020000;

static uint32_t const DEBUG_CONTROL_COMMAND = 0x104;
static uint32_t const DEBUG_CONTROL_COMMAND_HALT = 0x80000000;
static uint32_t const DEBUG_CONTROL_COMMAND_STEP = 0x04000000;
static uint32_t const DEBUG_CONTROL_COMMAND_RESUME = 0x40000000;

static uint32_t const DEBUG_CONTROL_BREAK = 0x10C;
static uint32_t const DEBUG_CONTROL_RSTOFBE = 0x80000000;

static uint32_t const DEBUG_CONTROL_HALT = 0x110;
static uint32_t const DEBUG_CONTROL_HALT_INT3 = 0x01000000;
static uint32_t const DEBUG_CONTROL_HALT_RST = 0x80000000;
static uint32_t const DEBUG_CONTROL_PWR_RST = 0x114;
static uint32_t const DEBUG_CONTROL_PWR_RST_HRESET = 0x00000004;

//TAP registers
static uint32_t const TAP_CMD = 4;
static uint32_t const TAP_CMD_LEN = 3;
static uint32_t const TAP_CMD_RD = 0;
static uint32_t const TAP_CMD_WR = 1;

static uint32_t const TAP_ADDR = 5;
static uint32_t const TAP_ADDR_LEN = 12;

static uint32_t const TAP_WR = 7;
static uint32_t const TAP_WR_LEN = 32;

static uint32_t const TAP_RD = 8;
static uint32_t const TAP_RD_LEN = 32;

static uint32_t const TAP_START = 9;
static uint32_t const TAP_START_LEN = 2;
static uint32_t const TAP_START_CMD = 1;

static uint32_t const TAP_STREAM = 0xA;
static uint32_t const TAP_STREAM_LEN = 34;

struct syntacore_riscv32_arch
{
    struct reg_cache *core_cache;
};

struct syntacore_riscv32_register
{
    uint32_t id;
    struct target* target;
};

int syntacore_riscv32_poll(struct target *target);
int syntacore_riscv32_target_create(struct target *target, Jim_Interp *interp);
int syntacore_riscv32_arch_state(struct target *target);
int syntacore_riscv32_init_target(struct command_context *cmd_ctx, struct target *target);

int syntacore_riscv32_halt(struct target *target);
int syntacore_riscv32_resume(struct target *target, int current, uint32_t address, int handle_breakpoints, int debug_execution);
int syntacore_riscv32_step(struct target *target, int current, uint32_t address, int handle_breakpoints);

int syntacore_riscv32_assert_reset(struct target *target);
int syntacore_riscv32_deassert_reset(struct target *target);
int syntacore_riscv32_soft_reset_halt(struct target *target);

int syntacore_riscv32_get_gdb_reg_list(struct target *target, struct reg **reg_list[], int *reg_list_size);

int syntacore_riscv32_read_memory(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);
int syntacore_riscv32_write_memory(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);

int syntacore_riscv32_examine(struct target *target);
int syntacore_riscv32_bulk_write_memory(struct target *target,
        uint32_t address, uint32_t count, uint8_t *buffer);

int syntacore_riscv32_add_breakpoint(struct target *target, struct breakpoint *breakpoint);
int syntacore_riscv32_remove_breakpoint(struct target *target, struct breakpoint *breakpoint);


//Low level access to debug controller
int
syntacore_riscv32_debug_write_register(struct target * target, uint32_t const address, uint32_t const data);
int
syntacore_riscv32_debug_read_register(struct target * target, uint32_t const address, uint32_t *const data);

int
syntacore_riscv32_read_mem_word(struct target * target, uint32_t const address, uint32_t* const data);
int
syntacore_riscv32_write_mem_word(struct target * target, uint32_t const address, uint32_t const data);

int
syntacore_riscv32_save_context(struct target * target);
int
syntacore_riscv32_restore_context(struct target*  target);

int
syntacore_riscv32_write_mem_chunk(
    struct target * target,
    uint32_t const address,
    uint32_t * const data,
    uint32_t const count);

int
syntacore_riscv32_jtag_set_instruction(struct target* target, int new_instr)
{
    struct jtag_tap *tap;

    tap = target->tap;
    if (tap == NULL)
        return ERROR_FAIL;

    if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != (uint32_t)new_instr)
    {
        struct scan_field field;
        uint32_t t = new_instr;

        field.tap = tap;
        field.num_bits = tap->ir_length;
        field.out_value = (void*)&t;
        field.in_value = NULL;

        jtag_add_ir_scan(1, &field, jtag_get_end_state());
    }

    return ERROR_OK;
}

struct target_type syntacore_riscv32_target =
{
    .name = "syntacore_riscv32",

    .poll = syntacore_riscv32_poll,
    .arch_state = syntacore_riscv32_arch_state,

    .target_request_data = NULL,

    .halt = syntacore_riscv32_halt,
    .resume = syntacore_riscv32_resume,
    .step = syntacore_riscv32_step,

    .assert_reset = syntacore_riscv32_assert_reset,
    .deassert_reset = syntacore_riscv32_deassert_reset,
    .soft_reset_halt = syntacore_riscv32_soft_reset_halt,

    .get_gdb_reg_list = syntacore_riscv32_get_gdb_reg_list,

    .read_memory = syntacore_riscv32_read_memory,
    .write_memory = syntacore_riscv32_write_memory,
    //.bulk_write_memory = mips_m4k_bulk_write_memory,
    //.checksum_memory = mips_m4k_checksum_memory,
    //.blank_check_memory = NULL,

    //.run_algorithm = mips32_run_algorithm,

    .add_breakpoint = syntacore_riscv32_add_breakpoint,
    .remove_breakpoint = syntacore_riscv32_remove_breakpoint,
    //.add_watchpoint = mips_m4k_add_watchpoint,
    //.remove_watchpoint = mips_m4k_remove_watchpoint,
    .bulk_write_memory = syntacore_riscv32_bulk_write_memory,

    .target_create = syntacore_riscv32_target_create,
    .init_target = syntacore_riscv32_init_target,
    .examine = syntacore_riscv32_examine,
};

char* syntacore_riscv32_core_reg_list[] =
{
    "eax", "ecx", "edx", "ebx", "esp", "ebp", "esi", "edi",
    "eip", "eflags", "cs", "ss", "ds", "es", "fs", "gs"
};

int
syntacore_riscv32_set_core_reg(struct reg *reg, uint8_t *buf)
{
    struct syntacore_riscv32_register* nc_reg = (struct syntacore_riscv32_register*)reg->arch_info;
    if (nc_reg->id < 10) //some i386 registers do not exits in NC
    {
        //LOG_INFO("updating cache for %s register", reg->name);
    }
    else
    {
        return ERROR_OK;
    }
    uint32_t value = buf_get_u32(buf, 0, 32);
    
    if (nc_reg->target->state != TARGET_HALTED)
    {
        return ERROR_TARGET_NOT_HALTED;
    }
    buf_set_u32(reg->value, 0, 32, value);
    reg->dirty = 1;
    reg->valid = 1;
    
    return ERROR_OK;
}

int
syntacore_riscv32_get_core_reg(struct reg *reg)
{
    LOG_WARNING("syntacore_riscv32_get_core_reg IS NOT IMPLEMENTED");
    return ERROR_OK;
}

static const struct reg_arch_type syntacore_riscv32_reg_type = {
    .get = syntacore_riscv32_get_core_reg,
    .set = syntacore_riscv32_set_core_reg,
};


int
syntacore_riscv32_target_create(struct target *target, Jim_Interp *interp)
{
    unsigned i = 0;
    const unsigned num_regs = 16;
    //struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
    
    struct syntacore_riscv32_arch *arch_info = calloc(1, sizeof(struct syntacore_riscv32_arch));
    struct reg *reg_list = malloc(sizeof(struct reg) * num_regs);
    //LOG_INFO("syntacore_riscv32_target_create");
    target->arch_info = arch_info;

    //reg cache initialization
    arch_info->core_cache = malloc(sizeof(struct reg_cache));
    
    arch_info->core_cache->name = "syntacore_riscv32 registers";
    arch_info->core_cache->next = NULL;
    arch_info->core_cache->reg_list = reg_list;
    arch_info->core_cache->num_regs = num_regs;
    
    for (i = 0; i < num_regs; i++)
    {
        reg_list[i].name = syntacore_riscv32_core_reg_list[i];
        reg_list[i].size = 32;
        reg_list[i].value = calloc(1, 4);
        *((int*)reg_list[i].value) = 0x100 + i;
        reg_list[i].dirty = 0;
        reg_list[i].valid = 0;
        reg_list[i].type = &syntacore_riscv32_reg_type;
        reg_list[i].arch_info = malloc(sizeof(struct syntacore_riscv32_register));
        ((struct syntacore_riscv32_register*)reg_list[i].arch_info)->id = i;
        ((struct syntacore_riscv32_register*)reg_list[i].arch_info)->target = target;
    }
    return ERROR_OK;
}

int nc_poll_requested = 0;

int
syntacore_riscv32_poll(struct target *target)
{
    //LOG_INFO("syntacore_riscv32_poll");
    int retval = 0;
    uint32_t nc_state = 0;
    if ((retval = syntacore_riscv32_debug_read_register(target, DEBUG_CONTROL_STATUS, &nc_state)) != ERROR_OK)
    {
        return retval;
    }
    if (nc_state & DEBUG_CONTROL_STATUS_HALTED)
    {
        target->state = TARGET_HALTED;
        if (nc_poll_requested)
        {
            if (nc_poll_requested == DBG_REASON_SINGLESTEP)
            {
                //enable interrupts
                syntacore_riscv32_debug_write_register(target, DEBUG_CONTROL_COMMAND, 0x0);
            }
            target->debug_reason = nc_poll_requested;
            target_call_event_callbacks(target, TARGET_EVENT_HALTED);
            nc_poll_requested = 0;
            syntacore_riscv32_save_context(target); //update reg cache
        }
        else
        {
            target->debug_reason = DBG_REASON_DBGRQ;
        }
    }
    return ERROR_OK;
}

int
syntacore_riscv32_arch_state(struct target *target)
{
    //LOG_INFO("syntacore_riscv32_arch_state");
    return ERROR_OK;
}

int
syntacore_riscv32_halt(struct target *target)
{
    //LOG_INFO("syntacore_riscv32_halt");
    int retval = 0;
    uint32_t nc_state = 0;
    //check NC state
    if ((retval = syntacore_riscv32_debug_read_register(target, DEBUG_CONTROL_STATUS, &nc_state)) != ERROR_OK)
    {
        return retval;
    }
    if (nc_state & DEBUG_CONTROL_STATUS_HALTED)
    {
        LOG_ERROR("Halt request when NC is already in halted state");
    }
    //issue Halt command
    if ((retval = syntacore_riscv32_debug_write_register(target, DEBUG_CONTROL_COMMAND, DEBUG_CONTROL_COMMAND_HALT)) != ERROR_OK)
    {
        return retval;
    }
    //issue error if we are still running
    if ((retval = syntacore_riscv32_debug_read_register(target, DEBUG_CONTROL_STATUS, &nc_state)) != ERROR_OK)
    {
        return retval;
    }
    if (!(nc_state & DEBUG_CONTROL_STATUS_HALTED))
    {
        LOG_ERROR("NC is not halted after Halt command");
    }
    if (nc_poll_requested)
    {
        //nc_poll_requested = 0;
    }
    target->state = TARGET_HALTED;
    target->debug_reason = DBG_REASON_DBGRQ;
    return ERROR_OK;
}

int
syntacore_riscv32_resume(struct target *target, int current, uint32_t address, int handle_breakpoints, int debug_execution)
{
    syntacore_riscv32_restore_context(target); //upload reg values into HW
    syntacore_riscv32_debug_write_register(target, DEBUG_CONTROL_COMMAND, DEBUG_CONTROL_COMMAND_RESUME);
    //poll state <- don't do this
    //target_call_event_callbacks(target, TARGET_EVENT_HALTED);
    nc_poll_requested = DBG_REASON_BREAKPOINT;
    target->state = TARGET_RUNNING;
    //syntacore_riscv32_save_context(target); //update reg cache
    //LOG_INFO("syntacore_riscv32_resume");
    return ERROR_OK;
}

int
syntacore_riscv32_step(struct target *target, int current, uint32_t address, int handle_breakpoints)
{
    syntacore_riscv32_restore_context(target); //upload reg values into HW
    syntacore_riscv32_debug_write_register(target, DEBUG_CONTROL_STATUS, DEBUG_CONTROL_STATUS_IRQ_DISABLE);
    syntacore_riscv32_debug_write_register(target, DEBUG_CONTROL_COMMAND, DEBUG_CONTROL_COMMAND_STEP);
    //poll state <- don't do this
    //target_call_event_callbacks(target, TARGET_EVENT_HALTED);
    nc_poll_requested = DBG_REASON_SINGLESTEP;
    target->state = TARGET_RUNNING;
    //syntacore_riscv32_save_context(target); //update reg cache
    //LOG_INFO("syntacore_riscv32_step");
    return ERROR_OK;
}

int
syntacore_riscv32_assert_reset(struct target *target)
{
    //LOG_INFO("syntacore_riscv32_assert_reset");
    int retval = 0;
    if ((retval = syntacore_riscv32_debug_write_register(target, DEBUG_CONTROL_PWR_RST, DEBUG_CONTROL_PWR_RST_HRESET)) != ERROR_OK)
    {
        return retval;
    }
    return ERROR_OK;
}

int
syntacore_riscv32_deassert_reset(struct target *target)
{
    //LOG_INFO("syntacore_riscv32_deassert_reset");
    int retval = 0;
    if ((retval = syntacore_riscv32_debug_write_register(target, DEBUG_CONTROL_PWR_RST, 0)) != ERROR_OK)
    {
        return retval;
    }
    return ERROR_OK;
}

int
syntacore_riscv32_soft_reset_halt(struct target *target)
{
    //LOG_INFO("syntacore_riscv32_soft_reset_halt");
    int retval = 0;
    //assert
    if ((retval = syntacore_riscv32_debug_write_register(target, DEBUG_CONTROL_PWR_RST, DEBUG_CONTROL_PWR_RST_HRESET)) != ERROR_OK)
    {
        return retval;
    }
    //...and deassert reset
    if ((retval = syntacore_riscv32_debug_write_register(target, DEBUG_CONTROL_PWR_RST, 0)) != ERROR_OK)
    {
        return retval;
    }
    return ERROR_OK;
}

int
syntacore_riscv32_bulk_write_memory(struct target *target,
        uint32_t address, uint32_t count, uint8_t *buffer)
{
    return syntacore_riscv32_write_memory(target, address, 4, count, buffer);
}

//gdb_server expects valid reg values and will use set method for updating reg values
int
syntacore_riscv32_get_gdb_reg_list(struct target *target, struct reg **reg_list[], int *reg_list_size)
{
    int i;
    struct syntacore_riscv32_arch *arch_info = (struct syntacore_riscv32_arch *)target->arch_info;

    //LOG_INFO("syntacore_riscv32_get_gdb_reg_list");
    
    *reg_list = malloc(sizeof(struct reg*) * 16);
    
    for (i = 0; i < 16; ++i)
    {
        (*reg_list)[i] = &arch_info->core_cache->reg_list[i];
    }
    *reg_list_size = 16;
    return ERROR_OK;
}

int
syntacore_riscv32_read_memory(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
    //LOG_INFO("syntacore_riscv32_read_memory at %08X, %d bytes", address, size * count);
    unsigned i = 0; //byte count
    uint32_t x = 0; //buffer
    int retval = 0;
    unsigned const buffer_size = count * size;
    unsigned word_start_offset = 0;
    if (address & 0x3) //Address is not aligned
    {
        if ((retval = syntacore_riscv32_read_mem_word(target, address & (~0x3), &x)) != ERROR_OK)
        {
            return retval;
        }
        while ((address + i) & 0x3 && i != buffer_size)
        {
            *(buffer + i) = ((uint8_t*)(&x))[(address + i) & 0x3];
            ++i;
        }
    }
    for (;i + 4 <= buffer_size; i += 4)
    {
        if ((retval = syntacore_riscv32_read_mem_word(target, address + i, (uint32_t*)(buffer + i))) != ERROR_OK)
        {
            return retval;
        }
    }
    if (buffer_size == i) return ERROR_OK;
    if ((retval = syntacore_riscv32_read_mem_word(target, address + i, &x)) != ERROR_OK)
    {
        return retval;
    }
    word_start_offset = i;
    while (i != buffer_size)
    {
        *(buffer + i) = ((uint8_t*)(&x))[(i - word_start_offset) & 0x3]; 
        ++i;
    }
    return ERROR_OK;
}

int
syntacore_riscv32_write_memory(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
    //LOG_INFO("syntacore_riscv32_write_memory at %08X, %d bytes", address, size * count);
    unsigned i = 0; //byte count
    uint32_t x = 0; //buffer
    int retval = 0;
    unsigned word_start_offset = 0;
    unsigned const buffer_size = count * size;
    if (address & 0x3) //Address is not aligned
    {
        if ((retval = syntacore_riscv32_read_mem_word(target, address & (~0x3), &x)) != ERROR_OK)
        {
            return retval;
        }
        while ((address + i) & 0x3 && i != buffer_size)
        {
            ((uint8_t*)(&x))[(address + i) & 0x3] = *(buffer + i);
            ++i;
        }
        if ((retval = syntacore_riscv32_write_mem_word(target, address & (~0x3), x)) != ERROR_OK)
        {
            return retval;
        }
    }
    uint32_t chunk_write_size = (buffer_size - i) & ~0x3;
    if (chunk_write_size < 8)
    {
        for (; i + 4 <= buffer_size; i += 4)
        {
            if ((retval = syntacore_riscv32_write_mem_word(target, address + i, *(uint32_t*)(buffer + i))) != ERROR_OK)
            {
                return retval;
            }
        }
    }
    else
    {
        syntacore_riscv32_write_mem_chunk(target, address + i, (uint32_t*)(buffer + i), chunk_write_size / 4);
        i += chunk_write_size;
    }

    if (buffer_size == i) return ERROR_OK;
    if ((retval = syntacore_riscv32_read_mem_word(target, address + i, &x)) != ERROR_OK)
    {
        return retval;
    }
    word_start_offset = i;
    while (i != buffer_size)
    {
        ((uint8_t*)(&x))[(i - word_start_offset) & 0x3] = *(buffer + i);
        ++i;
    }
    if ((retval = syntacore_riscv32_write_mem_word(target, (address + i) & ~0x3, x)) != ERROR_OK)
    {
        return retval;
    }
    return ERROR_OK;
}

int syntacore_riscv32_init_target(struct command_context *cmd_ctx, struct target *target)
{
    //LOG_INFO("syntacore_riscv32_init_target");
    return ERROR_OK;
}

int syntacore_riscv32_examine(struct target *target)
{
    //initialize register values
#if 0
    uint32_t id;

    syntacore_riscv32_debug_read_register(target, 0, &id);
    if (id != 0x8086DEBC)
    {
        LOG_ERROR("Id register has wrong value %08X", id);
        return ERROR_FAIL;
    }
#endif
    syntacore_riscv32_debug_write_register(target, DEBUG_CONTROL_PWR_RST, DEBUG_CONTROL_PWR_RST_HRESET);
    syntacore_riscv32_debug_write_register(target, DEBUG_CONTROL_BREAK, DEBUG_CONTROL_RSTOFBE);
    syntacore_riscv32_debug_write_register(target, DEBUG_CONTROL_HALT, DEBUG_CONTROL_HALT_INT3);
    syntacore_riscv32_debug_write_register(target, DEBUG_CONTROL_PWR_RST, 0);
    target->state = TARGET_HALTED;
    syntacore_riscv32_save_context(target);
    target_set_examined(target);
    //LOG_INFO("syntacore_riscv32_examine");
    return ERROR_OK;
}

struct {
    unsigned used;
    unsigned address;
} hw_bps[4];

int
syntacore_riscv32_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
    int i = 0;
    int retval = 0;
    unsigned hw_ctrl = 0;
    int bp = 0;
    //LOG_INFO("syntacore_riscv32_add_breakpoint");
    if (breakpoint->length != 1)
    {
        return ERROR_FAIL;
    }
    else if (breakpoint->type == BKPT_HARD)
    {
        //Find unused HW breakpoint
        //TODO: Maybe it is useful to check hardware state here(at least once)
        for (i = 0; i < 4; ++i)
        {
            if (!hw_bps[i].used)
            {
                bp = i;
                break;
            }
        }
        if (4 == i) //All HW breakpoint are occupied
        {
            return ERROR_FAIL;
        }
        hw_bps[bp].used = 1;
        hw_bps[bp].address = breakpoint->address;
        for (i = 0; i < 4; ++i)
        {
            if (hw_bps[i].used)
            {
                hw_ctrl |= 1 << (i * 2 + 1);
            }
        }
        if ((retval = syntacore_riscv32_debug_write_register(target, 0x39C, hw_ctrl)) != ERROR_OK)
        {
            return retval;
        }
        if ((retval = syntacore_riscv32_debug_write_register(target, 0x380 + bp * 4, breakpoint->address)) != ERROR_OK)
        {
            return retval;
        }
        //LOG_INFO("Hardware breakpoint at address %X", breakpoint->address);
    }
    else
    {
        if ((retval = target_read_memory(target, breakpoint->address, breakpoint->length, 1, breakpoint->orig_instr)) != ERROR_OK)
        {
            return retval;
        }
        if ((retval = target_write_u8(target, breakpoint->address, 0xCC)) != ERROR_OK)
        {
            return retval;
        }
        breakpoint->set = 1;
    }
    return ERROR_OK;
}

int
syntacore_riscv32_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
    int i = 0;
    int retval = 0;
    unsigned hw_ctrl = 0;
    //LOG_INFO("syntacore_riscv32_remove_breakpoint");
    if (breakpoint->length != 1)
    {
        return ERROR_FAIL;
    }
    else if (breakpoint->type == BKPT_HARD)
    {
        for (i = 0; i < 4; ++i)
        {
            if (hw_bps[i].address == breakpoint->address)
            {
                break;
            }
        }
        if (4 == i) //The breakpoint requested for removal is not found
        {
            return ERROR_FAIL;
        }
        hw_bps[i].used = 0;
        for (i = 0; i < 4; ++i)
        {
            if (hw_bps[i].used)
            {
                hw_ctrl |= 1 << (i * 2 + 1);
            }
        }
        if ((retval = syntacore_riscv32_debug_write_register(target, 0x39C, hw_ctrl)) != ERROR_OK)
        {
            return retval;
        }
        //LOG_INFO("Hardware breakpoint removed: %X", breakpoint->address);
    }
    else
    {
        if ((retval = target_write_u8(target, breakpoint->address, *breakpoint->orig_instr)) != ERROR_OK)
        {
            return retval;
        }
    }
    return ERROR_OK;
}

int
syntacore_riscv32_read_mem_word(struct target * target, uint32_t const address, uint32_t* const data)
{
    int retval = 0;
#if CHECK_MEMORY_TRANSACTIONS
    uint32_t transaction_status = 0;
#endif
    if ((retval = syntacore_riscv32_debug_write_register(target, DEBUG_MEMORY_ACCESS_ADDRESS, address)) != ERROR_OK)
    {
        return retval;
    }
    if ((retval = syntacore_riscv32_debug_write_register(target, DEBUG_MEMORY_ACCESS_CMD, DEBUG_MEMORY_ACCESS_CMD_START)) != ERROR_OK)
    {
        return retval;
    }
#if CHECK_MEMORY_TRANSACTIONS
    //check status register
    if ((retval = syntacore_riscv32_debug_read_register(target, DEBUG_MEMORY_ACCESS_CMD, &transaction_status)) != ERROR_OK)
    {
        return retval;
    }
    //poll status register until transaction is finished
    while (transaction_status & DEBUG_MEMORY_ACCESS_CMD_START)
    {
        if ((retval = syntacore_riscv32_debug_read_register(target, DEBUG_MEMORY_ACCESS_CMD, &transaction_status)) != ERROR_OK)
        {
            return retval;
        }
    }
#endif
    if ((retval = syntacore_riscv32_debug_read_register(target, DEBUG_MEMORY_ACCESS_RD_DATA, data)) != ERROR_OK)
    {
        return retval;
    }
    //LOG_INFO("MR A %08X D %08X", address, *data);
    return ERROR_OK;
}

int
syntacore_riscv32_write_mem_chunk(
    struct target * target,
    uint32_t const address,
    uint32_t * const data,
    uint32_t const count)
{
    struct scan_field field;
    uint32_t out_data[2];
    int retval = 0;
    jtag_set_end_state(TAP_IDLE);

    //LOG_INFO("chunk write A: %08X C: %d", address, count);

    if (count > 0xFFFF)
    {
        LOG_ERROR("syntacore_riscv32_write_mem_chunk error");
    }
    
    if ((retval = syntacore_riscv32_debug_write_register(target, DEBUG_MEMORY_ACCESS_ADDRESS, address)) != ERROR_OK)
    {
        return retval;
    }
    if ((retval = syntacore_riscv32_debug_write_register(target, DEBUG_MEMORY_ACCESS_COUNT, count & 0xFFFF)) != ERROR_OK)
    {
        return retval;
    }
    //start transaction
    if ((retval = syntacore_riscv32_debug_write_register(target,
        DEBUG_MEMORY_ACCESS_CMD, DEBUG_MEMORY_ACCESS_CMD_START
        | DEBUG_MEMORY_ACCESS_CMD_SEQ | DEBUG_MEMORY_ACCESS_CMD_WRITE | DEBUG_MEMORY_ACCESS_CMD_ALL_BYTES)) != ERROR_OK)
    {
        return retval;
    }
    
    syntacore_riscv32_jtag_set_instruction(target, TAP_ADDR);
    field.num_bits = TAP_ADDR_LEN;
    field.tap = target->tap;
    field.in_value = NULL;
    out_data[0] = DEBUG_MEMORY_ACCESS_WR_DATA;
    field.out_value = (void*)&out_data[0];
    jtag_add_dr_scan(1, &field, jtag_get_end_state());

    syntacore_riscv32_jtag_set_instruction(target, TAP_STREAM);
    out_data[1] = TAP_START_CMD;
    field.num_bits = TAP_STREAM_LEN;
    for (uint32_t i = 0; i < count; ++i)
    {
        out_data[0] = data[i];
        field.out_value = (void*)out_data;
        jtag_add_dr_scan(1, &field, jtag_get_end_state());
    }
    //LOG_INFO("command buffer initialization is done");
    if (jtag_execute_queue() != ERROR_OK)
    {
        LOG_ERROR("mem chunk write failed");
        return ERROR_FAIL;
    }
    //LOG_INFO("WR: A %08X D %08X", address, data);
    return ERROR_OK;
}

#if FAST_LOAD
int
syntacore_riscv32_write_mem_word(struct target * target, uint32_t const address, uint32_t const data)
{
    static int first_run_flag = 1;
    if (first_run_flag)
    {
        LOG_INFO("Optimized memory load enabled");
        first_run_flag = 0;
    }
    
    struct scan_field field;
    uint32_t out_data;
    jtag_set_end_state(TAP_IDLE);
    
    syntacore_riscv32_jtag_set_instruction(target, TAP_CMD);
    field.tap = target->tap;
    field.num_bits = TAP_CMD_LEN;
    out_data = TAP_CMD_WR;
    field.out_value = (void*)&out_data;
    field.in_value = NULL;
    jtag_add_dr_scan(1, &field, jtag_get_end_state());
    
    syntacore_riscv32_jtag_set_instruction(target, TAP_ADDR);
    field.num_bits = TAP_ADDR_LEN;
    out_data = DEBUG_MEMORY_ACCESS_ADDRESS;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(1, &field, jtag_get_end_state());
    
    syntacore_riscv32_jtag_set_instruction(target, TAP_WR);
    field.num_bits = TAP_WR_LEN;
    out_data = address;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(1, &field, jtag_get_end_state());
    
    syntacore_riscv32_jtag_set_instruction(target, TAP_START);
    field.num_bits = TAP_START_LEN;
    out_data = TAP_START_CMD;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(1, &field, jtag_get_end_state());
    //-----------------------------------------------
    syntacore_riscv32_jtag_set_instruction(target, TAP_ADDR);
    field.num_bits = TAP_ADDR_LEN;
    out_data = DEBUG_MEMORY_ACCESS_WR_DATA;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(1, &field, jtag_get_end_state());
    
    syntacore_riscv32_jtag_set_instruction(target, TAP_WR);
    field.num_bits = TAP_WR_LEN;
    out_data = data;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(1, &field, jtag_get_end_state());
    
    syntacore_riscv32_jtag_set_instruction(target, TAP_START);
    field.num_bits = TAP_START_LEN;
    out_data = TAP_START_CMD;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(1, &field, jtag_get_end_state());
    //----------------------------------------------
    syntacore_riscv32_jtag_set_instruction(target, TAP_ADDR);
    field.num_bits = TAP_ADDR_LEN;
    out_data = DEBUG_MEMORY_ACCESS_CMD;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(1, &field, jtag_get_end_state());
    
    syntacore_riscv32_jtag_set_instruction(target, TAP_WR);
    field.num_bits = TAP_WR_LEN;
    out_data = DEBUG_MEMORY_ACCESS_CMD_START | DEBUG_MEMORY_ACCESS_CMD_WRITE
        | DEBUG_MEMORY_ACCESS_CMD_ALL_BYTES;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(1, &field, jtag_get_end_state());
    
    syntacore_riscv32_jtag_set_instruction(target, TAP_START);
    field.num_bits = TAP_START_LEN;
    out_data = TAP_START_CMD;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(1, &field, jtag_get_end_state());

    if (jtag_execute_queue() != ERROR_OK)
    {
        LOG_ERROR("register read failed");
        return ERROR_FAIL;
    }
    //LOG_INFO("WR: A %08X D %08X", address, data);
    return ERROR_OK;
}
#else
int
syntacore_riscv32_write_mem_word(struct target * target, uint32_t const address, uint32_t const data)
{
    //LOG_INFO("MW A %08X D %08X", address, data);
    int retval = 0;
#if CHECK_MEMORY_TRANSACTIONS
    uint32_t transaction_status = 0;
#endif
    if ((retval = syntacore_riscv32_debug_write_register(target, DEBUG_MEMORY_ACCESS_ADDRESS, address)) != ERROR_OK)
    {
        return retval;
    }
    if ((retval = syntacore_riscv32_debug_write_register(target, DEBUG_MEMORY_ACCESS_WR_DATA, data)) != ERROR_OK)
    {
        return retval;
    }
    //start transaction
    if ((retval = syntacore_riscv32_debug_write_register(target,
        DEBUG_MEMORY_ACCESS_CMD, DEBUG_MEMORY_ACCESS_CMD_START | DEBUG_MEMORY_ACCESS_CMD_WRITE | 0xF0)) != ERROR_OK)
    {
        return retval;
    }
#if CHECK_MEMORY_TRANSACTIONS
    //check status register
    if ((retval = syntacore_riscv32_debug_read_register(target, DEBUG_MEMORY_ACCESS_CMD, &transaction_status)) != ERROR_OK)
    {
        return retval;
    }
    //poll status register until transaction is finished
    while (transaction_status & DEBUG_MEMORY_ACCESS_CMD_START)
    {
        if ((retval = syntacore_riscv32_debug_read_register(target, DEBUG_MEMORY_ACCESS_CMD, &transaction_status)) != ERROR_OK)
        {
            return retval;
        }
    }
#endif
    return ERROR_OK;
}
#endif

#if 0
int
syntacore_riscv32_debug_write_register(struct target * target, uint32_t const address, uint32_t const data)
{
    //struct jtag_tap *tap = target->tap;
    if (address < 0x300) //ignore memory transactions
    {
        LOG_INFO("write reg at %X, value %08X", address, data);
    }
    if (!target_was_examined(target))
    {
        return ERROR_TARGET_NOT_EXAMINED;
    }
    return ERROR_OK;
}

int
syntacore_riscv32_debug_read_register(struct target * target, uint32_t const address, uint32_t *const data)
{
    static uint32_t i = 0;
    //struct jtag_tap *tap = target->tap;
    if (address < 0x300) //ignore memory transactions
    {
        LOG_INFO("read reg at %X", address);
    }
    if (!target_was_examined(target))
    {
        return ERROR_TARGET_NOT_EXAMINED;
    }
    ++i;
    //dummy regs values
    switch (address)
    {
    case 0x100:
        *data = 0x40000000;
        break;
    default:
        *data = i;
    }
    return ERROR_OK;
}
#endif

int
syntacore_riscv32_debug_write_register(struct target * target, uint32_t const address, uint32_t const data)
{
    struct scan_field field_cmd, field_addr, field_start, field_data;
    uint32_t cmd, addr, start, data_in;
    jtag_set_end_state(TAP_IDLE);
    
    syntacore_riscv32_jtag_set_instruction(target, TAP_CMD);
    field_cmd.tap = target->tap;
    field_cmd.num_bits = TAP_CMD_LEN;
    field_cmd.out_value = (void*)&cmd;
    cmd = TAP_CMD_WR;
    field_cmd.in_value = NULL;
    jtag_add_dr_scan(1, &field_cmd, jtag_get_end_state());
    
    syntacore_riscv32_jtag_set_instruction(target, TAP_ADDR);
    field_addr.tap = target->tap;
    field_addr.num_bits = TAP_ADDR_LEN;
    field_addr.out_value = (void*)&addr;
    addr = address;
    field_addr.in_value = NULL;
    jtag_add_dr_scan(1, &field_addr, jtag_get_end_state());
    
    syntacore_riscv32_jtag_set_instruction(target, TAP_WR);
    field_data.tap = target->tap;
    field_data.num_bits = TAP_WR_LEN;
    data_in = data;
    field_data.out_value = (void*)&data_in;
    field_data.in_value = NULL;
    jtag_add_dr_scan(1, &field_data, jtag_get_end_state());
    
    syntacore_riscv32_jtag_set_instruction(target, TAP_START);
    field_start.tap = target->tap;
    field_start.num_bits = TAP_START_LEN;
    field_start.out_value = (void*)&start;
    start = TAP_START_CMD;
    field_start.in_value = NULL;
    jtag_add_dr_scan(1, &field_start, jtag_get_end_state());

    if (jtag_execute_queue() != ERROR_OK)
    {
        LOG_ERROR("register read failed");
        return ERROR_FAIL;
    }
    //LOG_INFO("WR: A %08X D %08X", address, data);
    return ERROR_OK;
}

int
syntacore_riscv32_debug_read_register(struct target * target, uint32_t const address, uint32_t *const data)
{
    struct scan_field field_cmd, field_addr, field_start, field_data;
    uint32_t cmd, addr, start, data_in;
    jtag_set_end_state(TAP_IDLE);
    
    syntacore_riscv32_jtag_set_instruction(target, TAP_CMD);
    field_cmd.tap = target->tap;
    field_cmd.num_bits = TAP_CMD_LEN;
    field_cmd.out_value = (void*)&cmd;
    cmd = TAP_CMD_RD;
    field_cmd.in_value = (void*)&cmd;
    jtag_add_dr_scan(1, &field_cmd, jtag_get_end_state());
  
    syntacore_riscv32_jtag_set_instruction(target, TAP_ADDR);
    field_addr.tap = target->tap;
    field_addr.num_bits = TAP_ADDR_LEN;
    field_addr.out_value = (void*)&addr;
    addr = address;
    field_addr.in_value = (void*)&addr;
    jtag_add_dr_scan(1, &field_addr, jtag_get_end_state());
    
    syntacore_riscv32_jtag_set_instruction(target, TAP_START);
    field_start.tap = target->tap;
    field_start.num_bits = TAP_START_LEN;
    field_start.out_value =(void*)&start;
    start = 0x1;
    field_start.in_value = (void*)&start;
    jtag_add_dr_scan(1, &field_start, jtag_get_end_state());
    
   
    syntacore_riscv32_jtag_set_instruction(target, TAP_RD);
    field_data.tap = target->tap;
    field_data.num_bits = TAP_RD_LEN;
    field_data.out_value = NULL;
    data_in = 0;
    field_data.in_value = (void*)&data_in;
    jtag_add_dr_scan(1, &field_data, jtag_get_end_state());
    
    if (jtag_execute_queue() != ERROR_OK)
    {
        LOG_ERROR("register read failed");
        return ERROR_FAIL;
    }
    *data = data_in;
    if (address >= 0x200 && address <= 0x300)
    {
        //LOG_INFO("RR: A %08X D %08X", address, data_in);
    }
    return ERROR_OK;
}

///update cache 
int
syntacore_riscv32_save_context(struct target * target)
{
    int retval;
    uint32_t nc_state;
    //issue error if we are still running
    if ((retval = syntacore_riscv32_debug_read_register(target, DEBUG_CONTROL_STATUS, &nc_state)) != ERROR_OK)
    {
        return retval;
    }
    if (!(nc_state & DEBUG_CONTROL_STATUS_HALTED))
    {
        LOG_ERROR("NC is not halted syntacore_riscv32_save_context");
    }
    struct syntacore_riscv32_arch *arch_info = (struct syntacore_riscv32_arch *)target->arch_info;
    for (int i = 0; i < 10; ++i) //NC has only 10 regs
    {
        if ((retval = syntacore_riscv32_debug_read_register(target,
            DEBUG_REGISTERS_ACCESS_BASE + i * 4,
            (uint32_t*)(arch_info->core_cache->reg_list[i].value))) != ERROR_OK)
        {
            return retval;
        }
        arch_info->core_cache->reg_list[i].dirty = 0;
        arch_info->core_cache->reg_list[i].valid = 1;
    }
    return ERROR_OK;
}

///update register values in HW from cache
int
syntacore_riscv32_restore_context(struct target*  target)
{
    struct syntacore_riscv32_arch *arch_info = (struct syntacore_riscv32_arch *)target->arch_info;
    uint32_t reg_value;
    int retval;
    uint32_t nc_state;
    //issue error if we are still running
    if ((retval = syntacore_riscv32_debug_read_register(target, DEBUG_CONTROL_STATUS, &nc_state)) != ERROR_OK)
    {
        return retval;
    }
    if (!(nc_state & DEBUG_CONTROL_STATUS_HALTED))
    {
        LOG_ERROR("NC is not halted syntacore_riscv32_restore_context");
    }
    for (int i = 0; i < 10; ++i) //NC has only 10 regs
    {
        //if (arch_info->core_cache->reg_list[i].dirty)
        //{
            reg_value = *(uint32_t*)(arch_info->core_cache->reg_list[i].value);
            if ((retval = syntacore_riscv32_debug_write_register(
                target, DEBUG_REGISTERS_ACCESS_BASE + i * 4, reg_value)) != ERROR_OK)
            {
                return retval;
            }
        //}
        arch_info->core_cache->reg_list[i].dirty = 0;
        arch_info->core_cache->reg_list[i].valid = 1;
    }
    return ERROR_OK;
}

