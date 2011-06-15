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

#ifdef LINUX
# define _GNU_SOURCE
#endif

#include "dr_api.h"
#include "dr_calls.h"

#ifdef LINUX
# include <dlfcn.h>
#endif

#include <stdlib.h>
#include <string.h>

#ifdef WINDOWS
# define EXPORT __declspec(dllexport)
# define NOINLINE __declspec(noinline)
#else
# define EXPORT __attribute__((visibility("default")))
# define NOINLINE __attribute__((noinline))
#endif

typedef struct _memop_t {
    ptr_uint_t ea;
    ptr_uint_t pc;
    uint size;
    uint write;
} memop_t;

#define BUFFER_SIZE 1024

static uint pos;
static memop_t buffer[BUFFER_SIZE];

/* Client options. */
/* Whether we print out the total number of accesses and the number of unaligned
 * accesses. */
static bool print_count = false;
/* Whether we print out diagnostic information for each unaligned access. */
static bool print_accesses = false;
/* Whether we lookup application symbols from PCs. */
static bool print_symbols = false;
/* Controls whether we buffer memory accesses before reading them. */
static bool use_buffer = false;

static unsigned long long num_accesses = 0;
static unsigned long long num_unaligned = 0;

NOINLINE static void
diagnose_access(ptr_uint_t ea, app_pc pc, uint size, bool write)
{
    if (print_accesses) {
        dr_fprintf(STDOUT,
                   "Unaligned %s access to ea "PFX" at pc "PFX" of size %d\n",
                   (write ? "write" : "read"), ea, pc, size);
        if (print_symbols) {
#ifdef LINUX
            Dl_info info;
            int r;
            const char *symbol = "<unknown>";
            const char *image = "<unknown>";
            r = dladdr(pc, &info);
            if (r) {
                if (info.dli_sname)
                    symbol = info.dli_sname;
                if (info.dli_fname)
                    image = info.dli_fname;
            }
            dr_fprintf(STDOUT, "pc: "PFX", image: %s, symbol: %s.\n",
                       pc, image, symbol);
#endif
        }
    }
}

/* An ideal clean call for partial inlining: has a fast path with a single
 * conditional jump and return, and a slow path with a bit of control flow
 * (ternary expr) and a call to printf.
 *
 * Checks that 'ea' was aligned to 'size', and if not prints out a warning about
 * an unaligned memory access.
 */
static void
check_access(ptr_uint_t ea, app_pc pc, uint size, bool write)
{
    num_accesses++;
    /* Check alignment.  Assumes size is a power of two. */
    if ((ea & (size - 1)) == 0) {
        return;
    }

    num_unaligned++;
    diagnose_access(ea, pc, size, write);
}

NOINLINE static void
flush_buffer(void)
{
    int i;

    for (i = 0; i < pos; i++) {
        check_access(buffer[i].ea,
                     (app_pc)buffer[i].pc,
                     buffer[i].size,
                     buffer[i].write);
    }
    pos = 0;
}

/* An ideal clean call for partial inlining: has a fast path with a single
 * conditional jump and return, and a slow path with a bit of control flow
 * (ternary expr) and a call to printf.
 */
void
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
    void *callee;

    if (write) {
        memop = instr_get_dst(where, pos);
    } else {
        memop = instr_get_src(where, pos);
    }
    uint opsize = opnd_size_in_bytes(opnd_get_size(memop));
    opnd_set_size(&memop, OPSZ_lea);

    if (use_buffer) {
        callee = (void*)buffer_memop;
    } else {
        callee = (void*)check_access;
    }
    drcalls_insert_call(dc, ilist, where, callee, false, 4,
                        memop, OPND_CREATE_INTPTR(instr_get_app_pc(where)),
                        OPND_CREATE_INT32(opsize), OPND_CREATE_INT32(write));
}

static dr_emit_flags_t
event_bb(void *dc, void *entry_pc, instrlist_t *bb, bool for_trace,
         bool translating)
{
    instr_t *instr;
    int i;

    for (instr = instrlist_first(bb); instr != NULL;
         instr = instr_get_next(instr)) {
        /* Some nop instructions have memory operands as a way of varying the
         * operation size.  We don't want those. */
        if (instr_is_nop(instr) || instr_get_opcode(instr) == OP_nop_modrm)
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
    const char *client_args = dr_get_options(id);
    int opt_calls;
    const char *opt_calls_str;
    bool use_tls = false;

    drcalls_init();

    /* Braindead arument parsing. */
    if (strstr(client_args, "print_count"))    print_count    = true;
    if (strstr(client_args, "print_accesses")) print_accesses = true;
    if (strstr(client_args, "print_symbols"))  print_symbols  = true;
    if (strstr(client_args, "use_buffer"))     use_buffer     = true;
    if (strstr(client_args, "use_tls"))        use_tls        = true;

    opt_calls_str = strstr(client_args, "opt_calls");
    if (opt_calls_str) {
        opt_calls = atoi(opt_calls_str + strlen("opt_calls"));
        drcalls_set_optimization(opt_calls);
    }

    drcalls_set_use_tls_inline(use_tls);

    dr_register_bb_event(event_bb);
    dr_register_exit_event(event_exit);
}
