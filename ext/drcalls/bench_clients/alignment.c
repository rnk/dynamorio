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

#include "stdlib.h"

bool show_results;

/* An ideal clean call for partial inlining: has a fast path with a single
 * conditional jump and return, and a slow path with a bit of control flow
 * (ternary expr) and a call to printf.
 *
 * Checks that 'ea' was aligned to 'size', and if not prints out a warning about
 * an unaligned memory access.
 */
void
clean_call(ptr_uint_t ea, app_pc pc, uint size, bool write)
{
    /* Check alignment.  Assumes size is a power of two. */
    if ((ea & (size - 1)) == 0) {
        return;
    }

    /* XXX: Instead of ifdef, use global variable to prevent the compiler from
     * optimizing this away.  It's important for benchmarking purposes. */
    if (show_results) {
        dr_fprintf(STDERR,
                   "Unaligned %s access to ea "PFX" at pc "PFX" of size %d\n",
                   (write ? "write" : "read"), ea, pc, size);
    }
}

static void
instrument_mem(void *dc, instrlist_t *ilist, instr_t *where, int pos,
               bool write)
{
    opnd_t memop;

    if (write) {
        memop = instr_get_dst(where, pos);
    } else {
        memop = instr_get_src(where, pos);
    }
    uint opsize = opnd_size_in_bytes(opnd_get_size(memop));
    opnd_set_size(&memop, OPSZ_lea);

    drcalls_insert_call(dc, ilist, where, (void*)clean_call, false, 4,
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

    return DR_EMIT_DEFAULT;
}

DR_EXPORT void
dr_init(client_id_t id)
{
    const char *client_args = dr_get_options(id);
    int opt_calls = atoi(client_args);

    drcalls_init();
    if (opt_calls < 0)
        opt_calls = 3;
    drcalls_set_optimization(opt_calls);
    dr_register_bb_event(event_bb);
    dr_register_exit_event(drcalls_exit);
#ifdef SHOW_RESULTS
    show_results = true;
#else
    show_results = false;
#endif
}
