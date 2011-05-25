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

#ifndef DRCALLS_OPTIMIZE_H
#define DRCALLS_OPTIMIZE_H

/* Dead register analysis abstraction.  Drive it by looping over instructions in
 * reverse.  At end of loop body, call liveness_update. */
typedef struct _liveness_t {
    bool reg_live[NUM_GP_REGS];
    /* Flags that are live.  Uses EFLAGS_READ_* form of the masks. */
    uint flags_live;
} liveness_t;

#define IS_LIVE(r) (liveness->reg_live[reg_to_pointer_sized(r) - DR_REG_XAX])
#define IS_DEAD(r) !IS_LIVE(r)
#define SET_LIVE(r, d) \
    (liveness->reg_live[reg_to_pointer_sized(r) - DR_REG_XAX] = (d))

void liveness_init(liveness_t *liveness, const callee_info_t *ci);
void liveness_update(liveness_t *liveness, instr_t *instr);
bool liveness_instr_is_live(liveness_t *liveness, instr_t *instr);

/* Optimizations. */
void dce_and_copy_prop(void *dc, const callee_info_t *ci);
void reuse_registers(void *dc, const callee_info_t *ci);
void try_avoid_flags(void *dc, const callee_info_t *ci);
void try_fold_immeds(void *dc, void *dc_alloc, instrlist_t *ilist);

#endif /* DRCALLS_OPTIMIZE_H */
