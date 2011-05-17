/* *******************************************************************************
 * Copyright (c) 2011 Massachusetts Institute of Technology  All rights reserved.
 * *******************************************************************************/

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

/* Ensure that we preserve mcontext layout in the slowpath when doing partial
 * inlining, since the client could call dr_get_mcontext.
 */

#include "dr_api.h"

static app_pc app_func_pc;
static void *instrumentation_pc;
static int monitoring = 0;

static dr_emit_flags_t
event_basic_block(void *dc, void *entry_pc, instrlist_t *bb,
                  bool for_trace, bool translating)
{
    instr_t *where = instrlist_first(bb);
    if (entry_pc == app_func_pc) {
        dr_fprintf(STDERR, "instrumenting app_func entry\n");
        drcalls_insert_call(dc, bb, where, instrumentation_pc, false, 0);
    }
}

static void
event_exit(void)
{
    dr_nonheap_free(instrumentation_pc, 4096);
}

static void
read_xax_from_mcontext(void)
{
    ptr_uint_t xax_val;
    dr_mcontext_t mcontext = {sizeof(mcontext),};
    void *dc = dr_get_current_drcontext();
    dr_get_mcontext(dc, &mcontext);
    xax_val = mcontext.xax;
    dr_fprintf(STDERR, "xax_val: 0x%08x\n", xax_val);
}

#define RWX_PROT (DR_MEMPROT_EXEC|DR_MEMPROT_READ|DR_MEMPROT_WRITE)
#define APP instrlist_meta_append

static void *
encode_new_rwx_mem(void *dc, instrlist_t *ilist)
{
    byte *rwx_mem = dr_nonheap_alloc(4096, RWX_PROT);
    byte *end_pc = instrlist_encode(dc, ilist, rwx_mem, /* has tgts */true);
    DR_ASSERT_MSG((ptr_uint_t)end_pc < (ptr_uint_t)rwx_mem + 4096,
                  "Code too big to encode!");
    instrlist_clear_and_destroy(dc, ilist);
    return rwx_mem;
}

static void
codegen_nonleaf_callee(void *dc)
{
    instrlist_t *ilist = instrlist_create(dc);
    opnd_t eax = opnd_create_reg(DR_REG_EAX);
    instr_t *done = INSTR_CREATE_label(dc);

    /* Normal prologue. */
    APP(ilist, INSTR_CREATE_push(dc, opnd_create_reg(DR_REG_XBP)));
    APP(ilist, INSTR_CREATE_mov_st
        (dc, opnd_create_reg(DR_REG_XBP), opnd_create_reg(DR_REG_XSP)));
    /* 2 registers maintains 16-byte alignment. */
    APP(ilist, INSTR_CREATE_push(dc, opnd_create_reg(DR_REG_XAX)));
    APP(ilist, INSTR_CREATE_push(dc, opnd_create_reg(DR_REG_XBX)));

    /* if (monitoring) {
     *   read_xax_from_mcontext();
     * }
     * monitoring++;
     */
    APP(ilist, INSTR_CREATE_mov_ld
        (dc, eax, OPND_CREATE_ABSMEM(&monitoring, OPSZ_4)));
    APP(ilist, INSTR_CREATE_test(dc, eax, eax));
    APP(ilist, INSTR_CREATE_jcc(dc, OP_jz, opnd_create_instr(done)));
    APP(ilist, INSTR_CREATE_call
        (dc, opnd_create_pc((app_pc)read_xax_from_mcontext)));
    APP(ilist, done);
    APP(ilist, INSTR_CREATE_inc(dc, OPND_CREATE_ABSMEM(&monitoring, OPSZ_4)));

    /* Reverse of pushes. */
    APP(ilist, INSTR_CREATE_pop(dc, opnd_create_reg(DR_REG_XBX)));
    APP(ilist, INSTR_CREATE_pop(dc, opnd_create_reg(DR_REG_XAX)));

    /* Normal epilogue. */
    APP(ilist, INSTR_CREATE_leave(dc));
    APP(ilist, INSTR_CREATE_ret(dc));

    instrumentation_pc = encode_new_rwx_mem(dc, ilist);
}

DR_EXPORT void
dr_init(client_id_t id)
{
    void *dc = dr_get_current_drcontext();

    dr_register_bb_event(event_basic_block);
    dr_register_exit_event(event_exit);

    /* Lookup pcs. */
    module_data_t *exe = dr_lookup_module_by_name("drcalls.preserve_mcontext");
    app_func_pc = (app_pc)dr_get_proc_address(exe->handle, "app_func");
    dr_free_module_data(exe);

    /* Codegen callees. */
    codegen_nonleaf_callee(dc);
}
