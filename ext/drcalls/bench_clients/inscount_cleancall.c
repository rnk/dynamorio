/* ******************************************************************************
 * Copyright (c) 2011 Massachusetts Institute of Technology  All rights reserved.
 * ******************************************************************************/

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
 * * Neither the name of MIT nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL MIT OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

/* inscount client that relies on DynamoRIO inlining the clean call. */

#include "dr_api.h"
#include "dr_calls.h"
#include "instr_builder.h"

#include "inscount_common.h"

const char *client_name = "inscount_cleancall";

static void (*inc_count)(void);

static byte inc_count_buf[128];

static void
codegen_inc_count(void *dc)
{
    instr_builder_t ib;
    INSTR_BUILDER_INIT(ib, dc, instrlist_create(dc), NULL, /*meta=*/false);

    BUILD(ib, push,   REG(XBP));
    BUILD(ib, mov_ld, REG(XBP),                     REG(XSP));
    BUILD(ib, mov_ld, REG(XAX),                     MEM_REL(&global_count, 8));
    BUILD(ib, add,    REG(XAX),                     IMM(1, 4));
    BUILD(ib, mov_st, MEM_REL(&global_count, 8),    REG(XAX));
    INSERT(ib, INSTR_CREATE_leave(dc));
    INSERT(ib, INSTR_CREATE_ret(dc));

    instrlist_encode(dc, ib.ilist, &inc_count_buf[0], false);
    inc_count = (void (*)(void)) &inc_count_buf[0];
    instrlist_clear_and_destroy(dc, ib.ilist);
    dr_memory_protect((void*)inc_count, 128,
                      DR_MEMPROT_EXEC|DR_MEMPROT_WRITE|DR_MEMPROT_READ);
}

dr_emit_flags_t
event_basic_block(void *drcontext, void *tag, instrlist_t *bb,
                  bool for_trace, bool translating)
{
    instr_t *instr;
    if (inc_count == NULL) {
        codegen_inc_count(drcontext);
    }
    for (instr = instrlist_first(bb); instr != NULL;
         instr = instr_get_next(instr)) {
        drcalls_insert_call(drcontext, bb, instr, (void *)inc_count,
                            false /* save fpstate */, 0);
    }

    drcalls_done(drcontext, bb);

    return DR_EMIT_DEFAULT;
}
