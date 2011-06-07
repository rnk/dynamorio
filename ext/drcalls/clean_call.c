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

/* Logic for generating clean calls. */

#include "dr_api.h"

#include "core_compat.h"
#include "mcontext.h"

#include <string.h>

static reg_id_t
shrink_reg_for_param(reg_id_t regular, opnd_t arg)
{
#ifdef X64
    if (opnd_get_size(arg) == OPSZ_4) { /* we ignore var-sized */
        /* PR 250976 #2: leave 64-bit only if an immed w/ top bit set (we
         * assume user wants sign-extension; that is after all what happens
         * on a push of a 32-bit immed) */
        if (!opnd_is_immed_int(arg) ||
            (opnd_get_immed_int(arg) & 0x80000000) == 0)
            return reg_64_to_32(regular);
    }
#endif
    return regular;
}

void
materialize_arg_into_reg(void *dc, instrlist_t *ilist, instr_t *where,
                         uint framesize, bool regs_from_mc[NUM_GP_REGS],
                         opnd_t arg, reg_id_t reg)
{
    opnd_t dst_opnd;

    reg = shrink_reg_for_param(reg, arg);
    dst_opnd = opnd_create_reg(reg);
    if (opnd_is_immed_int(arg)) {
        PRE(ilist, where, INSTR_CREATE_mov_imm (dc, dst_opnd, arg));
    } else if (opnd_is_reg(arg)) {
        reg_id_t src = reg_to_pointer_sized(opnd_get_reg(arg));
        if (src == DR_REG_XSP) {
            DR_ASSERT_MSG(false, "Not yet tested");
            PRE(ilist, where, INSTR_CREATE_mov_ld
                (dc, dst_opnd, opnd_get_tls_xax(dc)));
        } else {
            if (regs_from_mc[src - DR_REG_XAX]) {
                /* If we used src in the inline code, we need to restore it
                 * to the application value before reading it again.
                 */
                insert_mc_reg_restore(dc, framesize, ilist, where, src);
                /* Set the dst of the mov_ld to dst_opnd. */
                /* TODO(rnk): May be an opnd size issue. */
                instr_set_dst(instr_get_prev(where), 0, dst_opnd);
                DR_ASSERT_MSG(false, "Not yet tested");
            } else {
                PRE(ilist, where,
                    INSTR_CREATE_mov_ld(dc, dst_opnd, arg));
            }
        }
    } else {
        PRE(ilist, where, INSTR_CREATE_mov_ld(dc, dst_opnd, arg));
    }
}

/* TODO(rnk): De-duplicate from inline.c and leancall.c. */
static void
insert_arg_setup(void *dc, instrlist_t *ilist, instr_t *where, uint framesize,
                 uint num_args, opnd_t *args)
{
    uint i;
    bool regs_from_mc[NUM_GP_REGS];
    uint num_reg_args = dr_num_reg_parm();

    num_reg_args = MIN(num_args, num_reg_args);

    memset(regs_from_mc, 0, sizeof(regs_from_mc));
    for (i = 0; i < num_reg_args; i++) {
        materialize_arg_into_reg(dc, ilist, where, framesize, regs_from_mc,
                                 args[i], dr_reg_parm(i));
    }
    if (num_args <= num_reg_args)
        return;

    for (i = num_reg_args; i < num_args; i++) {
        /* push it on the stack */
        DR_ASSERT_MSG(false, "stack args unsupported ATM");
    }
}

void
insert_clean_call(void *dc, instrlist_t *ilist, instr_t *where, void *callee,
                  bool fpstate, uint num_args, opnd_t *args)
{
    uint framesize = ALIGN_FORWARD_UINT(sizeof(dr_mcontext_t), 16);
    bool save_regs[NUM_GP_REGS];
    int i;

    /* save all */
    for (i = 0; i < NUM_GP_REGS; i++)
        save_regs[i] = true;

    insert_switch_to_dstack(dc, framesize, ilist, where);
    insert_mc_regs(dc, framesize, /*save=*/true, save_regs, ilist, where);
    insert_mc_flags_save(dc, framesize, ilist, where);
    insert_mc_xmm_regs(dc, framesize, /*save=*/true, ilist, where);
    /* TODO(rnk): fpstate */

    insert_arg_setup(dc, ilist, where, framesize, num_args, args);
    PRE(ilist, where, INSTR_CREATE_call(dc, opnd_create_pc(callee)));

    insert_mc_xmm_regs(dc, framesize, /*save=*/false, ilist, where);
    insert_mc_flags_restore(dc, framesize, ilist, where);
    insert_mc_regs(dc, framesize, /*save=*/false, save_regs, ilist, where);
    insert_switch_to_appstack(dc, framesize, ilist, where);
}
