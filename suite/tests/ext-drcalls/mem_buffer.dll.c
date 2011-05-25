/* **********************************************************
 * Copyright (c) 2011 MIT  All rights reserved.
 * **********************************************************/

/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of VMware, Inc. nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL VMWARE, INC. OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#include "dr_api.h"
#include "dr_calls.h"

#ifdef WINDOWS
# define EXPORT __declspec(dllexport)
# define NOINLINE __declspec(noinline)
#else
# define EXPORT __attribute__((visibility("default")))
# define NOINLINE __attribute__((noinline))
#endif

static app_pc start_pc;
static app_pc stop_pc;
static bool do_instrumentation;

typedef struct _memop_t {
    ptr_uint_t ea;
    ptr_uint_t pc;
    uint size;
    uint write;
} memop_t;

#define BUFFER_SIZE 1024

static uint pos;
static memop_t buffer[BUFFER_SIZE];
/* Test suite will want only count, spec will want neither. */
static bool print_count = true;
static bool print_accesses = true;
static unsigned long long num_accesses = 0;
static unsigned long long num_unaligned = 0;

NOINLINE static void
flush_buffer(void)
{
    int i;
    void *dc = dr_get_current_drcontext();

    for (i = 0; i < pos; i++) {
        if ((buffer[i].ea & (buffer[i].size - 1)) == 0) {
            continue;
        }
        num_unaligned++;
        if (print_accesses) {
            dr_fprintf(STDERR, "Unaligned %s access to ea "PFX" at pc "PFX" of"
                       " size %u\n",
                       (buffer[i].write ? "write" : "read"),
                       buffer[i].ea,
                       buffer[i].pc,
                       buffer[i].size);
        }
    }
    num_accesses += pos;
    pos = 0;
}

/* An ideal clean call for partial inlining: has a fast path with a single
 * conditional jump and return, and a slow path with a bit of control flow
 * (ternary expr) and a call to printf.
 */
EXPORT void
buffer_memop(ptr_uint_t ea, ptr_uint_t pc, uint size, bool write)
{
    buffer[pos].ea = ea;
    buffer[pos].pc = pc;
    buffer[pos].size = size;
    buffer[pos].write = write;
    pos++;
    if (pos >= BUFFER_SIZE) {
        flush_buffer();
    }
}

static void
instrument_mem(void *dc, instrlist_t *ilist, instr_t *where, int pos,
               bool write)
{
    opnd_t memop;
    reg_id_t arg_reg_id = DR_REG_XBX;
    opnd_t arg_reg = opnd_create_reg(arg_reg_id);

    if (write) {
        memop = instr_get_dst(where, pos);
    } else {
        memop = instr_get_src(where, pos);
    }
    uint opsize = opnd_size_in_bytes(opnd_get_size(memop));
    opnd_set_size(&memop, OPSZ_lea);

    dr_save_reg(dc, ilist, where, arg_reg_id, SPILL_SLOT_1);
    instrlist_meta_preinsert(ilist, where,
                             INSTR_CREATE_lea(dc, arg_reg, memop));
    drcalls_insert_call(dc, ilist, where, (void*)buffer_memop, false, 4,
                        arg_reg, OPND_CREATE_INTPTR(instr_get_app_pc(where)),
                        OPND_CREATE_INT32(opsize), OPND_CREATE_INT32(write));
    dr_restore_reg(dc, ilist, where, arg_reg_id, SPILL_SLOT_1);
}

static dr_emit_flags_t
event_bb(void *dc, void *entry_pc, instrlist_t *bb, bool for_trace,
         bool translating)
{
    instr_t *instr;
    int i;

    if (entry_pc == start_pc) {
        do_instrumentation = true;
    }
    if (entry_pc == stop_pc) {
        do_instrumentation = false;
    }
    if (!do_instrumentation) {
        return DR_EMIT_DEFAULT;
    }

    for (instr = instrlist_first(bb); instr != NULL;
         instr = instr_get_next(instr)) {
        /* Some nop instructions have memory operands as a way of varying the
         * operation size.  We don't want those. */
        if (instr_is_nop(instr))
            continue;
        if (instr_get_app_pc(instr) == NULL)
            continue;
        if (instr_reads_memory(instr)) {
            for (i = 0; i < instr_num_srcs(instr); i++) {
                if (opnd_is_memory_reference(instr_get_src(instr, i))) {
                    instrument_mem(dc, bb, instr, i, false);
                }
            }
        }
        if (instr_writes_memory(instr)) {
            for (i = 0; i < instr_num_dsts(instr); i++) {
                if (opnd_is_memory_reference(instr_get_dst(instr, i))) {
                    instrument_mem(dc, bb, instr, i, true);
                }
            }
        }
    }

    drcalls_done(dc, bb);
    return DR_EMIT_DEFAULT;
}

static void
event_exit(void)
{
    flush_buffer();

    if (print_count) {
        dr_fprintf(STDERR,
                   "Memory accesses: %llu\n"
                   "Unaligned memory accesses: %llu\n",
                   num_accesses,
                   num_unaligned);
    }

    drcalls_exit();
}

DR_EXPORT void
dr_init(client_id_t id)
{
    const char *opts = dr_get_options(id);
    if (opts != NULL && strcmp("count_only", opts) == 0) {
        print_count = true;
        print_accesses = false;
    } else if (opts != NULL && strcmp("hide_results", opts) == 0) {
        print_count = false;
        print_accesses = false;
    }
    drcalls_init();
    dr_register_bb_event(event_bb);
    dr_register_exit_event(event_exit);
    module_data_t *exe = dr_lookup_module_by_name("drcalls.mem_buffer");
    start_pc = (app_pc)dr_get_proc_address(exe->handle, "start_monitor");
    stop_pc = (app_pc)dr_get_proc_address(exe->handle, "stop_monitor");
    dr_free_module_data(exe);
}
