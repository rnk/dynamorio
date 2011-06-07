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

#ifndef DRCALLS_INLINE_H
#define DRCALLS_INLINE_H

/* Call-site specific information. */
typedef struct _clean_call_info_t {
    void *callee;
    opnd_t *args;
    uint num_args;
    bool save_fpstate;
    bool opt_inline;
    bool should_align;
    bool save_all_regs;
    bool skip_save_aflags;
    bool skip_clear_eflags;
    uint num_xmms_skip;
    bool xmm_skip[NUM_XMM_REGS];
    uint num_regs_skip;
    bool reg_skip[NUM_GP_REGS];
    void *callee_info;  /* callee information */
    instrlist_t *ilist; /* instruction list for inline optimization */
} clean_call_info_t;

void inline_init(void);
void inline_exit(void);
clean_call_info_t *analyze_clean_call(void *dcontext, instr_t *where,
                                      void *callee, bool save_fpstate,
                                      uint num_args, opnd_t *args);
void expand_and_optimize_bb(void *dc, instrlist_t *bb);
void clean_call_info_destroy(void *dc, clean_call_info_t *cci);

uint opt_cleancall;

/* The max number of instructions try to decode from a function. */
#define MAX_NUM_FUNC_INSTRS 1024
/* the max number of instructions the callee can have for inline. */
#define MAX_NUM_INLINE_INSTRS 20
#define NUM_SCRATCH_SLOTS 10 /* XXX: Can support more now that we use dstack. */

/* TODO(rnk): Circular dependency. */
struct _callee_info_t;
app_pc emit_partial_slowpath(void *dc, struct _callee_info_t *ci);

/* Pseudo-instruction opcodes. */
/* TODO(rnk): Do these really belong here? */
enum {
    DRC_OP_call = OP_AFTER_LAST,
    DRC_OP_dstack,
    DRC_OP_appstack,
};

#endif /* DRCALLS_INLINE_H */
