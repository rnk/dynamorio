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

/* An attempt to make generating and inserting instructions into ilists easier.
 */

#ifndef DRCALLS_INSTR_BUILDER_H
#define DRCALLS_INSTR_BUILDER_H

#if 0
/* Code to test with when ready. */

/* Emit the slowpath back to the callee for partially inlined functions.
 * Return the slowpath entry point. */
static app_pc
emit_partial_slowpath(void *dc, callee_info_t *ci)
{
    byte *entry;
    instrlist_t *ilist;
    opnd_t xsp = opnd_create_reg(DR_REG_XSP);
    uint realignment;
    instr_builder_t ib;

    dr_log(dc, LOG_CLEANCALL, 3,
           "CLEANCALL: emitting partial inline slowpath\n");

    /* Generate the clean call ilist.  Arguments should be materialized into
     * registers at the callsite.  Application register values are already on
     * the stack.
     */
    ilist = instrlist_create(dc);
    //INSTR_BUILDER_INIT(ib, dc, ilist, NULL, true);
    ib.dcontext = dc;
    ib.ilist = ilist;
    ib.where = NULL;
    ib.meta = true;
    /* Re-align stack to 16 bytes to prepare for call. */
    realignment = (16 - sizeof(reg_t));
    BUILD(ib, lea, REG(XSP), MEM(XSP, -realignment, lea));
    insert_slowpath_mc(dc, ci, /*save=*/true, ilist);
    BUILD(ib, call, opnd_create_pc(ci->start));
    insert_slowpath_mc(dc, ci, /*save=*/false, ilist);
    /* Un-align stack to get back to ret addr. */
    BUILD(ib, lea, REG(XSP), MEM(XSP, realignment, lea));
    BUILD(ib, ret);

    dr_log(dc, LOG_CACHE, 3, "drcalls: emitting partial slowpath\n");
    entry = code_cache_emit(dc, ilist);

    instrlist_clear_and_destroy(dc, ilist);

    ci->partial_pc = entry;
    return entry;
}
#endif /* 0 */

typedef struct _instr_builder_t {
    void *dcontext;
    instrlist_t *ilist;
    instr_t *where;
    bool meta;
} instr_builder_t;

#define BUILD(ib, opname, ...) \
    do { \
        instr_t *instr = INSTR_CREATE_##opname (ib.dcontext, __VA_ARGS__); \
        instr_set_ok_to_mangle(instr, ib.meta); \
        instrlist_preinsert(ib.ilist, ib.where, instr); \
    } while (0)

#define REG(r) opnd_create_reg(DR_REG_##r)

#define IMM(v, sz) opnd_create_immed_int(v, OPSZ_##sz)

#define MEM(base, disp, sz) \
    opnd_create_base_disp(DR_REG_##base, DR_REG_NULL, 0, disp, OPSZ_##sz)
#define MEM_PTR(base, disp) MEM(base, disp, sizeof(void*))

/* sz is pasted with OPSZ_ so you can use lea or PTR for sz. */
#define MEM_IDX(base, idx, scale, disp, sz) \
    opnd_create_base_disp(DR_REG_##base, DR_REG_##idx, scale, disp, OPSZ_##sz)

#define INSTR_BUILDER_INIT(ib_, dc_, ilist_, where_, meta_) \
    do { \
        ib_.dcontext = (dc_); \
        ib_.ilist = (ilist_); \
        ib_.where = (where_); \
        ib_.meta = (meta_); \
    } while (0)

#endif /* DRCALLS_INSTR_BUILDER_H */
