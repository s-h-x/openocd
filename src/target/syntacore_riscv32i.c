#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

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
// #define LOCAL_LOG_INFO LOG_INFO
#define LOCAL_LOG_INFO(expr,...) do {} while(0)

struct arch
{
    struct reg_cache *core_cache;
};

struct Register
{
    uint32_t id;
    struct target* target;
};

typedef
struct hw_bps_struct
{
    unsigned used;
    unsigned address;
} hw_bps_type;

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

/// TAP registers
///@{
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
///@}

static hw_bps_type hardware_breakpoints[4];

static int nc_poll_requested = 0;

static int
jtag_set_instruction(struct target* target, int new_instr)
{
    struct jtag_tap* tap = target->tap;
    if ( !tap ) {
        return ERROR_FAIL;
    }

    if ( buf_get_u32(tap->cur_instr, 0u, tap->ir_length) != (uint32_t)new_instr ) {
        uint32_t t = new_instr;
        struct scan_field field = {
            .num_bits = tap->ir_length,
            .out_value = (void*)&t,
            .in_value = NULL
        };
        jtag_add_ir_scan(tap, &field, TAP_IDLE);
    }

    return ERROR_OK;
}

// Low level access to debug controller
static int
debug_read_register(struct target * target, uint32_t const address, uint32_t *const data)
{
    {
        jtag_set_instruction(target, TAP_CMD);
        uint32_t cmd = TAP_CMD_RD;
        struct scan_field field_cmd = {
            .num_bits = TAP_CMD_LEN,
            .out_value = (void*)&cmd,
            .in_value = (void*)&cmd
        };
        jtag_add_dr_scan(target->tap, 1, &field_cmd, TAP_IDLE);
    }

    {
        jtag_set_instruction(target, TAP_ADDR);
        uint32_t addr = address;
        struct scan_field field_addr =
        {
            .num_bits = TAP_ADDR_LEN,
            .out_value = (void*)&addr,
            .in_value = (void*)&addr
        };
        jtag_add_dr_scan(target->tap, 1, &field_addr, TAP_IDLE);
    }

    {
        jtag_set_instruction(target, TAP_START);
        uint32_t start = 0x1;
        struct scan_field field_start = {
            .num_bits = TAP_START_LEN,
            .out_value = (void*)&start,
            .in_value = (void*)&start
        };
        jtag_add_dr_scan(target->tap, 1, &field_start, TAP_IDLE);
    }

    {
        uint32_t data_in;
        {
            jtag_set_instruction(target, TAP_RD);
            data_in = 0;
            struct scan_field field_data = {
                .num_bits = TAP_RD_LEN,
                .out_value = NULL,
                .in_value = (void*)&data_in
            };
            jtag_add_dr_scan(target->tap, 1, &field_data, TAP_IDLE);
        }

        if ( jtag_execute_queue() != ERROR_OK ) {
            LOG_ERROR("register read failed");
            return ERROR_FAIL;
        }
        *data = data_in;
        if ( address >= 0x200 && address <= 0x300 ) {
            LOCAL_LOG_INFO("RR: A %08X D %08X", address, data_in);
        }
    }
    return ERROR_OK;
}

static int
debug_write_register(struct target * target, uint32_t const address, uint32_t const data)
{
    {
        jtag_set_instruction(target, TAP_CMD);
        uint32_t cmd = TAP_CMD_WR;
        struct scan_field field_cmd = {
            .num_bits = TAP_CMD_LEN,
            .out_value = (void*)&cmd,
            .in_value = NULL
        };
        jtag_add_dr_scan(target->tap, 1, &field_cmd, TAP_IDLE);
    }

    {
        jtag_set_instruction(target, TAP_ADDR);
        uint32_t addr = address;
        struct scan_field field_addr = {
            .num_bits = TAP_ADDR_LEN,
            .out_value = (void*)&addr,
            .in_value = NULL
        };
        jtag_add_dr_scan(target->tap, 1, &field_addr, TAP_IDLE);
    }

    {
        jtag_set_instruction(target, TAP_WR);
        uint32_t data_in = data;
        struct scan_field field_data = {
            .num_bits = TAP_WR_LEN,
            .out_value = (void*)&data_in,
            .in_value = NULL
        };
        jtag_add_dr_scan(target->tap, 1, &field_data, TAP_IDLE);
    }

    {
        jtag_set_instruction(target, TAP_START);
        uint32_t start = TAP_START_CMD;
        struct scan_field field_start = {
            .num_bits = TAP_START_LEN,
            .out_value = (void*)&start,
            .in_value = NULL
        };
        jtag_add_dr_scan(target->tap, 1, &field_start, TAP_IDLE);
    }

    if ( jtag_execute_queue() != ERROR_OK ) {
        LOG_ERROR("register read failed");
        return ERROR_FAIL;
    }
    LOCAL_LOG_INFO("WR: A %08X D %08X", address, data);
    return ERROR_OK;
}

/// update cache
static int
save_context(struct target * target)
{
    int retval;
    uint32_t nc_state;
    // issue error if we are still running
    if ( (retval = debug_read_register(target, DEBUG_CONTROL_STATUS, &nc_state)) == ERROR_OK ) {
        if ( !(nc_state & DEBUG_CONTROL_STATUS_HALTED) ) {
            LOG_ERROR("NC is not halted save_context");
        }
        struct arch *arch_info = (struct arch *)target->arch_info;
        // NC has only 10 regs
        for ( int i = 0; i < 10; ++i ) {
            if ( (retval = debug_read_register(target, DEBUG_REGISTERS_ACCESS_BASE + i * 4, (uint32_t*)(arch_info->core_cache->reg_list[i].value))) != ERROR_OK ) {
                break;
            }
            arch_info->core_cache->reg_list[i].dirty = 0;
            arch_info->core_cache->reg_list[i].valid = 1;
        }
    }

    return retval;
}

static int
this_target_poll(struct target *target)
{
    LOCAL_LOG_INFO("poll");
    int retval;
    uint32_t nc_state = 0;
    if ( (retval = debug_read_register(target, DEBUG_CONTROL_STATUS, &nc_state)) == ERROR_OK ) {
        if ( nc_state & DEBUG_CONTROL_STATUS_HALTED ) {
            target->state = TARGET_HALTED;
            if ( nc_poll_requested ) {
                if ( nc_poll_requested == DBG_REASON_SINGLESTEP ) {
                    // enable interrupts
                    debug_write_register(target, DEBUG_CONTROL_COMMAND, 0x0);
                }
                target->debug_reason = nc_poll_requested;
                target_call_event_callbacks(target, TARGET_EVENT_HALTED);
                nc_poll_requested = 0;
                save_context(target);  // update reg cache
            } else {
                target->debug_reason = DBG_REASON_DBGRQ;
            }
        }
    }
    return retval;
}

static int
this_target_arch_state(struct target *target)
{
    LOCAL_LOG_INFO("arch_state");
    return ERROR_OK;
}

static int
this_target_init(struct command_context *cmd_ctx, struct target *target)
{
    LOCAL_LOG_INFO("init_target");
    return ERROR_OK;
}

static int
this_target_halt(struct target *target)
{
    LOCAL_LOG_INFO("halt");
    int retval;
    uint32_t nc_state = 0;
    if ( (retval = debug_read_register(target, DEBUG_CONTROL_STATUS, &nc_state)) == ERROR_OK ) {
        if ( nc_state & DEBUG_CONTROL_STATUS_HALTED ) {
            LOG_ERROR("Halt request when NC is already in halted state");
        }
        // issue Halt command
        if ( (retval = debug_write_register(target, DEBUG_CONTROL_COMMAND, DEBUG_CONTROL_COMMAND_HALT)) == ERROR_OK ) {
            // issue error if we are still running
            if ( (retval = debug_read_register(target, DEBUG_CONTROL_STATUS, &nc_state)) == ERROR_OK ) {
                if ( !(nc_state & DEBUG_CONTROL_STATUS_HALTED) ) {
                    LOG_ERROR("NC is not halted after Halt command");
                }
#if 0
                if ( nc_poll_requested ) {
                    nc_poll_requested = 0;
                }
#endif
                target->state = TARGET_HALTED;
                target->debug_reason = DBG_REASON_DBGRQ;
            }
        }
    }
    return retval;
}

/// update register values in HW from cache
static int
restore_context(struct target*  target)
{
    struct arch *arch_info = (struct arch *)target->arch_info;
    int retval;
    uint32_t nc_state;
    // issue error if we are still running
    if ( (retval = debug_read_register(target, DEBUG_CONTROL_STATUS, &nc_state)) == ERROR_OK ) {
        if ( !(nc_state & DEBUG_CONTROL_STATUS_HALTED) ) {
            LOG_ERROR("NC is not halted restore_context");
        }
        // NC has only 10 regs
        for ( int i = 0; i < 10; ++i ) {
            uint32_t const reg_value = *(uint32_t*)(arch_info->core_cache->reg_list[i].value);
            if ( (retval = debug_write_register(target, DEBUG_REGISTERS_ACCESS_BASE + i * 4, reg_value)) != ERROR_OK ) {
                break;
            }
            arch_info->core_cache->reg_list[i].dirty = 0;
            arch_info->core_cache->reg_list[i].valid = 1;
        }
    }
    return retval;
}

static int
this_target_resume(struct target *target, int current, uint32_t address, int handle_breakpoints, int debug_execution)
{
    restore_context(target);  // upload reg values into HW
    debug_write_register(target, DEBUG_CONTROL_COMMAND, DEBUG_CONTROL_COMMAND_RESUME);
    nc_poll_requested = DBG_REASON_BREAKPOINT;
    target->state = TARGET_RUNNING;
    return ERROR_OK;
}

static int
this_target_step(struct target *target, int current, uint32_t address, int handle_breakpoints)
{
    restore_context(target);  // upload reg values into HW
    debug_write_register(target, DEBUG_CONTROL_STATUS, DEBUG_CONTROL_STATUS_IRQ_DISABLE);
    debug_write_register(target, DEBUG_CONTROL_COMMAND, DEBUG_CONTROL_COMMAND_STEP);
    nc_poll_requested = DBG_REASON_SINGLESTEP;
    target->state = TARGET_RUNNING;
    return ERROR_OK;
}

static int
this_target_assert_reset(struct target *target)
{
    LOCAL_LOG_INFO("assert_reset");
    int retval = 0;
    if ( (retval = debug_write_register(target, DEBUG_CONTROL_PWR_RST, DEBUG_CONTROL_PWR_RST_HRESET)) != ERROR_OK ) {
        return retval;
    }
    return ERROR_OK;
}

static int
this_target_deassert_reset(struct target *target)
{
    LOCAL_LOG_INFO("deassert_reset");
    int retval = 0;
    if ( (retval = debug_write_register(target, DEBUG_CONTROL_PWR_RST, 0)) != ERROR_OK ) {
        return retval;
    }
    return ERROR_OK;
}

static int
this_target_soft_reset_halt(struct target *target)
{
    LOCAL_LOG_INFO("soft_reset_halt");
    int retval;
    // assert reset
    if ( (retval = debug_write_register(target, DEBUG_CONTROL_PWR_RST, DEBUG_CONTROL_PWR_RST_HRESET)) == ERROR_OK ) {
        // ...and deassert reset
        retval = debug_write_register(target, DEBUG_CONTROL_PWR_RST, 0);
    }
    return retval;
}

static int
this_target_get_gdb_reg_list(struct target *target, struct reg **reg_list[], int *reg_list_size, enum target_register_class reg_class)
{
    LOCAL_LOG_INFO("get_gdb_reg_list");
    struct arch *arch_info = (struct arch *)target->arch_info;

    *reg_list = malloc(sizeof(struct reg*) * 16);

    for ( int i = 0; i < 16; ++i ) {
        (*reg_list)[i] = &arch_info->core_cache->reg_list[i];
    }
    *reg_list_size = 16;
    return ERROR_OK;
}

/// gdb_server expects valid reg values and will use set method for updating reg values
static int
read_mem_word(struct target * target, uint32_t const address, uint32_t* const data)
{
    int retval = 0;
    if ( (retval = debug_write_register(target, DEBUG_MEMORY_ACCESS_ADDRESS, address)) != ERROR_OK ) {
        return retval;
    }
    if ( (retval = debug_write_register(target, DEBUG_MEMORY_ACCESS_CMD, DEBUG_MEMORY_ACCESS_CMD_START)) != ERROR_OK ) {
        return retval;
    }
#if CHECK_MEMORY_TRANSACTIONS
    {
        uint32_t transaction_status = 0;
        // check status register
        if ( (retval = debug_read_register(target, DEBUG_MEMORY_ACCESS_CMD, &transaction_status)) != ERROR_OK ) {
            return retval;
        }
        // poll status register until transaction is finished
        while (transaction_status & DEBUG_MEMORY_ACCESS_CMD_START) {
            if ((retval = debug_read_register(target, DEBUG_MEMORY_ACCESS_CMD, &transaction_status)) != ERROR_OK) {
                return retval;
            }
        }
    }
#endif
    if ( (retval = debug_read_register(target, DEBUG_MEMORY_ACCESS_RD_DATA, data)) != ERROR_OK ) {
        return retval;
    }
    LOCAL_LOG_INFO("MR A %08X D %08X", address, *data);
    return ERROR_OK;
}

static int
this_target_read_memory(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
    LOCAL_LOG_INFO("read_memory at %08X, %d bytes", address, size * count);
    unsigned i = 0;  // byte count
    uint32_t x = 0;  // buffer
    int retval = 0;
    unsigned const buffer_size = count * size;
    // Address is not aligned
    if ( address & 0x3 ) {
        if ( (retval = read_mem_word(target, address & (~0x3), &x)) != ERROR_OK ) {
            return retval;
        }
        while ( (address + i) & 0x3 && i != buffer_size ) {
            *(buffer + i) = ((uint8_t*)(&x))[(address + i) & 0x3];
            ++i;
        }
    }
    for ( ; i + 4 <= buffer_size; i += 4 ) {
        if ( (retval = read_mem_word(target, address + i, (uint32_t*)(buffer + i))) != ERROR_OK ) {
            return retval;
        }
    }
    if ( buffer_size == i ) {
        return ERROR_OK;
    }
    if ( (retval = read_mem_word(target, address + i, &x)) != ERROR_OK ) {
        return retval;
    }
    unsigned const word_start_offset = i;
    for ( ; i != buffer_size; ++i ) {
        *(buffer + i) = ((uint8_t*)(&x))[(i - word_start_offset) & 0x3];
    }
    return ERROR_OK;
}

static int
write_mem_word(struct target * target, uint32_t const address, uint32_t const data)
{
#if FAST_LOAD
    static int first_run_flag = 1;
    if ( first_run_flag ) {
        LOCAL_LOG_INFO("Optimized memory load enabled");
        first_run_flag = 0;
    }

    struct scan_field field;
    uint32_t out_data;

    jtag_set_instruction(target, TAP_CMD);
    field.num_bits = TAP_CMD_LEN;
    out_data = TAP_CMD_WR;
    field.out_value = (void*)&out_data;
    field.in_value = NULL;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

    jtag_set_instruction(target, TAP_ADDR);
    field.num_bits = TAP_ADDR_LEN;
    out_data = DEBUG_MEMORY_ACCESS_ADDRESS;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

    jtag_set_instruction(target, TAP_WR);
    field.num_bits = TAP_WR_LEN;
    out_data = address;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

    jtag_set_instruction(target, TAP_START);
    field.num_bits = TAP_START_LEN;
    out_data = TAP_START_CMD;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);


    jtag_set_instruction(target, TAP_ADDR);
    field.num_bits = TAP_ADDR_LEN;
    out_data = DEBUG_MEMORY_ACCESS_WR_DATA;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

    jtag_set_instruction(target, TAP_WR);
    field.num_bits = TAP_WR_LEN;
    out_data = data;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

    jtag_set_instruction(target, TAP_START);
    field.num_bits = TAP_START_LEN;
    out_data = TAP_START_CMD;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);


    jtag_set_instruction(target, TAP_ADDR);
    field.num_bits = TAP_ADDR_LEN;
    out_data = DEBUG_MEMORY_ACCESS_CMD;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

    jtag_set_instruction(target, TAP_WR);
    field.num_bits = TAP_WR_LEN;
    out_data = DEBUG_MEMORY_ACCESS_CMD_START | DEBUG_MEMORY_ACCESS_CMD_WRITE
        | DEBUG_MEMORY_ACCESS_CMD_ALL_BYTES;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

    jtag_set_instruction(target, TAP_START);
    field.num_bits = TAP_START_LEN;
    out_data = TAP_START_CMD;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

    if ( jtag_execute_queue() != ERROR_OK ) {
        LOG_ERROR("register read failed");
        return ERROR_FAIL;
    }
    LOCAL_LOG_INFO("WR: A %08X D %08X", address, data);
#else
    LOCAL_LOG_INFO("MW A %08X D %08X", address, data);
    int retval = 0;
    if ( (retval = debug_write_register(target, DEBUG_MEMORY_ACCESS_ADDRESS, address)) != ERROR_OK ) {
        return retval;
    }
    if ( (retval = debug_write_register(target, DEBUG_MEMORY_ACCESS_WR_DATA, data)) != ERROR_OK ) {
        return retval;
    }
    // start transaction
    if ( (retval = debug_write_register(target,
        DEBUG_MEMORY_ACCESS_CMD, DEBUG_MEMORY_ACCESS_CMD_START | DEBUG_MEMORY_ACCESS_CMD_WRITE | 0xF0)) != ERROR_OK ) {
        return retval;
    }
#if CHECK_MEMORY_TRANSACTIONS
    {
        uint32_t transaction_status = 0;
        // check status register
        if ( (retval = debug_read_register(target, DEBUG_MEMORY_ACCESS_CMD, &transaction_status)) != ERROR_OK ) {
            return retval;
        }
        // poll status register until transaction is finished
        while ( transaction_status & DEBUG_MEMORY_ACCESS_CMD_START ) {
            if ( (retval = debug_read_register(target, DEBUG_MEMORY_ACCESS_CMD, &transaction_status)) != ERROR_OK ) {
                return retval;
            }
        }
    }
#endif
#endif
    return ERROR_OK;
}


static int
this_target_examine(struct target *target)
{
    // initialize register values
    debug_write_register(target, DEBUG_CONTROL_PWR_RST, DEBUG_CONTROL_PWR_RST_HRESET);
    debug_write_register(target, DEBUG_CONTROL_BREAK, DEBUG_CONTROL_RSTOFBE);
    debug_write_register(target, DEBUG_CONTROL_HALT, DEBUG_CONTROL_HALT_INT3);
    debug_write_register(target, DEBUG_CONTROL_PWR_RST, 0);
    target->state = TARGET_HALTED;
    save_context(target);
    target_set_examined(target);
    LOCAL_LOG_INFO("examine");
    return ERROR_OK;
}

static int
this_target_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
    int retval = 0;
    unsigned hw_ctrl = 0;
    int bp = 0;
    static size_t const total_number_of_hw_breakpoints = sizeof hardware_breakpoints / sizeof hardware_breakpoints[0];
    LOCAL_LOG_INFO("add_breakpoint");
    if ( breakpoint->length != 1 ) {
        return ERROR_FAIL;
    } else if ( breakpoint->type == BKPT_HARD ) {
        // Find unused HW breakpoint
        /// @todo Maybe it is useful to check hardware state here(at least once)
        {
            size_t i;
            for ( i = 0; i < total_number_of_hw_breakpoints; ++i ) {
                if ( !hardware_breakpoints[i].used ) {
                    bp = i;
                    break;
                }
            }
            // All HW breakpoint are occupied
            if ( total_number_of_hw_breakpoints == i ) {
                return ERROR_FAIL;
            }
        }
        hardware_breakpoints[bp].used = 1;
        hardware_breakpoints[bp].address = breakpoint->address;
        {
             for ( size_t i = 0; i < total_number_of_hw_breakpoints; ++i ) {
                if ( hardware_breakpoints[i].used ) {
                    hw_ctrl |= 1 << (i * 2 + 1);
                }
            }
        }
        if ( (retval = debug_write_register(target, 0x39C, hw_ctrl)) != ERROR_OK ) {
            return retval;
        }
        if ( (retval = debug_write_register(target, 0x380 + bp * 4, breakpoint->address)) != ERROR_OK ) {
            return retval;
        }
        LOCAL_LOG_INFO("Hardware breakpoint at address %X", breakpoint->address);
    } else {
        if ( (retval = this_target_read_memory(target, breakpoint->address, breakpoint->length, 1, breakpoint->orig_instr)) != ERROR_OK ) {
            return retval;
        }
        if ( (retval = target_write_u8(target, breakpoint->address, 0xCC)) != ERROR_OK ) {
            return retval;
        }
        breakpoint->set = 1;
    }
    return ERROR_OK;
}

static int
this_target_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
    int retval = 0;
    LOCAL_LOG_INFO("remove_breakpoint");
    if ( breakpoint->length != 1 ) {
        return ERROR_FAIL;
    }
    if ( breakpoint->type == BKPT_HARD ) {
        unsigned hw_ctrl = 0;
        int i = 0;
        for ( i = 0; i < 4; ++i ) {
            if ( hardware_breakpoints[i].address == breakpoint->address ) {
                break;
            }
        }
        // The breakpoint requested for removal is not found
        if ( 4 == i ) {
            return ERROR_FAIL;
        }
        hardware_breakpoints[i].used = 0;
        for ( i = 0; i < 4; ++i ) {
            if ( hardware_breakpoints[i].used ) {
                hw_ctrl |= 1 << (i * 2 + 1);
            }
        }
        if ( (retval = debug_write_register(target, 0x39C, hw_ctrl)) != ERROR_OK ) {
            return retval;
        }
        LOCAL_LOG_INFO("Hardware breakpoint removed: %X", breakpoint->address);
        return ERROR_OK;
    } else {
        return target_write_u8(target, breakpoint->address, *breakpoint->orig_instr);
    }
}

static int
write_mem_chunk(struct target* target, uint32_t const address, uint32_t * const data, uint32_t const count)
{
    int retval = 0;

    LOCAL_LOG_INFO("chunk write A: %08X C: %d", address, count);
    if ( count > 0xFFFF ) {
        LOG_ERROR("write_mem_chunk error");
    }

    if ( (retval = debug_write_register(target, DEBUG_MEMORY_ACCESS_ADDRESS, address)) != ERROR_OK ) {
        return retval;
    }
    if ( (retval = debug_write_register(target, DEBUG_MEMORY_ACCESS_COUNT, count & 0xFFFF)) != ERROR_OK ) {
        return retval;
    }

    // start transaction
    if ( (retval =
        debug_write_register(target, DEBUG_MEMORY_ACCESS_CMD, DEBUG_MEMORY_ACCESS_CMD_START | DEBUG_MEMORY_ACCESS_CMD_SEQ | DEBUG_MEMORY_ACCESS_CMD_WRITE | DEBUG_MEMORY_ACCESS_CMD_ALL_BYTES)
        ) != ERROR_OK ) {
        return retval;
    }

    jtag_set_instruction(target, TAP_ADDR);
    {
        uint32_t out_data[2];
        struct scan_field field;
        out_data[0] = DEBUG_MEMORY_ACCESS_WR_DATA;
        field.num_bits = TAP_ADDR_LEN;
        field.in_value = NULL;
        field.out_value = (void*)&out_data[0];
        jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

        jtag_set_instruction(target, TAP_STREAM);
        out_data[1] = TAP_START_CMD;
        field.num_bits = TAP_STREAM_LEN;
        for ( uint32_t i = 0; i < count; ++i ) {
            out_data[0] = data[i];
            field.out_value = (void*)out_data;
            jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
        }
    }
    LOCAL_LOG_INFO("command buffer initialization is done");
    if ( jtag_execute_queue() != ERROR_OK ) {
        LOG_ERROR("mem chunk write failed");
        return ERROR_FAIL;
    }
    LOCAL_LOG_INFO("WR: A %08X D %08X", address, data);
    return ERROR_OK;
}

static int
this_target_write_memory(struct target *target, uint32_t address, uint32_t size, uint32_t count, const uint8_t *buffer)
{
    LOCAL_LOG_INFO("write_memory at %08X, %d bytes", address, size * count);
    unsigned i = 0;  // byte count
    uint32_t x = 0;  // buffer
    int retval = 0;
    unsigned word_start_offset = 0;
    unsigned const buffer_size = count * size;
    // Address is not aligned
    if ( address & 0x3 ) {
        if ( (retval = read_mem_word(target, address & (~0x3), &x)) != ERROR_OK ) {
            return retval;
        }
        while ( (address + i) & 0x3 && i != buffer_size ) {
            ((uint8_t*)(&x))[(address + i) & 0x3] = *(buffer + i);
            ++i;
        }
        if ( (retval = write_mem_word(target, address & (~0x3), x)) != ERROR_OK ) {
            return retval;
        }
    }
    uint32_t const chunk_write_size = (buffer_size - i) & ~0x3u;
    if ( chunk_write_size < 8 ) {
        for ( ; i + 4 <= buffer_size; i += 4 ) {
            if ( (retval = write_mem_word(target, address + i, *(uint32_t*)(buffer + i))) != ERROR_OK ) {
                return retval;
            }
        }
    } else {
        write_mem_chunk(target, address + i, (uint32_t*)(buffer + i), chunk_write_size / 4);
        i += chunk_write_size;
    }

    if ( buffer_size == i ) {
        return ERROR_OK;
    }
    if ( (retval = read_mem_word(target, address + i, &x)) != ERROR_OK ) {
        return retval;
    }
    word_start_offset = i;
    for ( ; i != buffer_size; ++i ) {
        ((uint8_t*)&x)[(i - word_start_offset) & 0x3] = buffer[i];
    }

    return write_mem_word(target, (address + i) & ~0x3, x);
}

static int
set_core_reg(struct reg *reg, uint8_t *buf)
{
    struct Register* nc_reg = (struct Register*)reg->arch_info;
    if ( nc_reg->id >= 10 ) {
        return ERROR_OK;
    }
    LOCAL_LOG_INFO("updating cache for %s register", reg->name);
    uint32_t const value = buf_get_u32(buf, 0, 32);

    if ( nc_reg->target->state != TARGET_HALTED ) {
        return ERROR_TARGET_NOT_HALTED;
    }
    buf_set_u32(reg->value, 0, 32, value);
    reg->dirty = 1;
    reg->valid = 1;

    return ERROR_OK;
}

int
get_core_reg(struct reg *reg)
{
    LOG_WARNING("get_core_reg IS NOT IMPLEMENTED");
    return ERROR_OK;
}

static struct reg_arch_type const reg_type =
{
    .get = get_core_reg,
    .set = set_core_reg,
};

static int
this_target_create(struct target *target, Jim_Interp *interp)
{
    LOCAL_LOG_INFO("target_create");
    static unsigned const num_regs = 16;

    struct arch* arch_info = calloc(1, sizeof(struct arch));
    struct reg* reg_list = calloc(num_regs, sizeof(struct reg));
    target->arch_info = arch_info;

    // reg cache initialization
    arch_info->core_cache = malloc(sizeof(struct reg_cache));

    arch_info->core_cache->name = "syntacore_riscv32 registers";
    arch_info->core_cache->next = NULL;
    arch_info->core_cache->reg_list = reg_list;
    arch_info->core_cache->num_regs = num_regs;

    static char const* core_reg_list[] = {
        "eax",
        "ecx",
        "edx",
        "ebx",
        "esp",
        "ebp",
        "esi",
        "edi",
        "eip",
        "eflags",
        "cs",
        "ss",
        "ds",
        "es",
        "fs",
        "gs"};

    struct reg* p_reg = &reg_list[0];
    for ( unsigned i = 0; i < num_regs; ++i, ++p_reg ) {
        p_reg->name = core_reg_list[i];
        p_reg->size = 32;
        p_reg->value = calloc(1, 4);
        *((int*)p_reg->value) = 0x100 + i;
        p_reg->dirty = 0;
        p_reg->valid = 0;
        p_reg->type = &reg_type;
        struct Register* p_item = malloc(sizeof(struct Register));
        p_item->id = i;
        p_item->target = target;
        p_reg->arch_info = p_item;
    }
    return ERROR_OK;
}

struct target_type syntacore_riscv32i_target =
{
    .name = "syntacore_riscv32",

    .poll = this_target_poll,
    .arch_state = this_target_arch_state,

    .target_request_data = NULL,

    .halt = this_target_halt,
    .resume = this_target_resume,
    .step = this_target_step,

    .assert_reset = this_target_assert_reset,
    .deassert_reset = this_target_deassert_reset,
    .soft_reset_halt = this_target_soft_reset_halt,

    .get_gdb_reg_list = this_target_get_gdb_reg_list,

    .read_memory = this_target_read_memory,
    .write_memory = this_target_write_memory,
    .add_breakpoint = this_target_add_breakpoint,
    .remove_breakpoint = this_target_remove_breakpoint,
    .target_create = this_target_create,
    .init_target = this_target_init,
    .examine = this_target_examine,
};
