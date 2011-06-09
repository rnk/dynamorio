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

typedef struct _instr_builder_t {
    void *dcontext;
    instrlist_t *ilist;
    instr_t *where;
    bool meta;
} instr_builder_t;

#define BUILD(ib, opname, ...) \
    do { \
        instr_t *instr = INSTR_CREATE_##opname (ib.dcontext, __VA_ARGS__); \
        instr_set_ok_to_mangle(instr, !ib.meta); \
        instrlist_preinsert(ib.ilist, ib.where, instr); \
    } while (0)

#define INSERT(ib, instr) \
    do { \
        instr_set_ok_to_mangle(instr, !ib.meta); \
        instrlist_preinsert(ib.ilist, ib.where, instr); \
    } while (0)

#define REG(r) opnd_create_reg(DR_REG_##r)

#define IMM(v, sz) opnd_create_immed_int(v, OPSZ_##sz)

#define MEM(base, disp, sz) \
    opnd_create_base_disp(DR_REG_##base, DR_REG_NULL, 0, disp, OPSZ_##sz)
#define MEM_PTR(base, disp) MEM(base, disp, sizeof(void*))

#define MEM_REL(addr, sz) opnd_create_rel_addr((void*)addr, OPSZ_##sz)

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
