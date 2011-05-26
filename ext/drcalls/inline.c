/* *******************************************************************************
 * Copyright (c) 2010-2011 Massachusetts Institute of Technology  All rights reserved.
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

/* Logic for inserting inlined code.  Also contains call site analysis.  All
 * callee analysis lives in callee.c. */

#include "dr_api.h"
#include "hashtable.h"

#include <stddef.h> /* for offsetof */
#include <string.h> /* for memcmp */

/* internal includes */
#include "callee.h"
#include "code_cache.h"
#include "core_compat.h"
#include "inline.h"
#include "optimize.h"
#include "instr_builder.h"

static callee_info_t     default_callee_info;
static clean_call_info_t default_clean_call_info;

/* Optimization level of calls. */
uint opt_cleancall = 3;

/* Prototypes for analysis data structures. */
static void clean_call_info_init(clean_call_info_t *cci, void *callee,
                                 bool save_fpstate, opnd_t *args, uint num_args);

void
inline_init(void)
{
    callee_info_init(&default_callee_info);
    clean_call_info_init(&default_clean_call_info, NULL, false, NULL, 0);
    callee_info_table_init();
}

void
inline_exit(void)
{
    callee_info_table_destroy();
}

static clean_call_info_t *
clean_call_info_create(void *dc, void *callee, bool fpstate, opnd_t *args,
                       uint num_args)
{
    clean_call_info_t *cci = dr_thread_alloc(dc, sizeof(*cci));
    clean_call_info_init(cci, callee, fpstate, args, num_args);
    return cci;
}

void
clean_call_info_destroy(void *dc, clean_call_info_t *cci)
{
    if (cci == NULL)
        return;
    if (cci->args != NULL)
        dr_thread_free(dc, cci->args, sizeof(opnd_t) * cci->num_args);
    dr_thread_free(dc, cci, sizeof(*cci));
}

static void
clean_call_info_init(clean_call_info_t *cci, void *callee, bool save_fpstate,
                     opnd_t *args, uint num_args)
{
    memset(cci, 0, sizeof(*cci));
    cci->callee        = callee;
    cci->num_args      = num_args;
    if (args != NULL) {
        cci->args = dr_thread_alloc(dr_get_current_drcontext(),
                                    sizeof(opnd_t) * num_args);
        memcpy(cci->args, args, sizeof(opnd_t) * num_args);
    }
    cci->save_fpstate  = save_fpstate;
    cci->save_all_regs = true;
    cci->should_align  = true;
    cci->callee_info   = &default_callee_info;
}

/****************************************************************************/
/* clean call optimization code */

static void
analyze_clean_call_aflags(void *dcontext, clean_call_info_t *cci,
                          instr_t *where)
{
    callee_info_t *ci = cci->callee_info;
    instr_t *instr;

    /* If there's a flags read, we clear the flags.  If there's a write or read,
     * we save them, because a read creates a clear which is a write. */
    cci->skip_clear_eflags = !ci->read_aflags;
    cci->skip_save_aflags  = !(ci->write_aflags || ci->read_aflags);
    /* XXX: this is a more aggressive optimization by analyzing the ilist
     * to be instrumented. The client may change the ilist, which violate
     * the analysis result. For example, 
     * I do not need save the aflags now if an instruction
     * after "where" updating all aflags, but later the client can
     * insert an instruction reads the aflags before that instruction.
     */
    if (opt_cleancall > 1 && !cci->skip_save_aflags) {
        for (instr = where; instr != NULL; instr = instr_get_next(instr)) {
            uint flags = instr_get_arith_flags(instr);
            if (TESTANY(EFLAGS_READ_6, flags) || instr_is_cti(instr))
                break;
            if (TESTALL(EFLAGS_WRITE_6, flags)) {
                dr_log(dcontext, LOG_CLEANCALL, 2,
                       "drcalls: inserting clean call "PFX
                       ", skip saving aflags.\n", ci->start);
                cci->skip_save_aflags = true;
                break;
            }
        }
    }
}

static void
analyze_clean_call_regs(void *dcontext, clean_call_info_t *cci)
{
    uint i;
    callee_info_t *info = cci->callee_info;

    /* 1. xmm registers */
    for (i = 0; i < NUM_XMM_REGS; i++) {
        if (info->xmm_used[i]) {
            cci->xmm_skip[i] = false;
        } else {
            cci->xmm_skip[i] = true;
            cci->num_xmms_skip++;
        }
    }
    if (opt_cleancall > 2 && cci->num_xmms_skip != NUM_XMM_REGS)
        cci->should_align = false;
    /* 2. general purpose registers */
    /* set regs not to be saved for clean call */
    for (i = 0; i < NUM_GP_REGS; i++) {
        if (info->reg_used[i]) {
            cci->reg_skip[i] = false;
        } else {
            cci->reg_skip[i] = true;
            cci->num_regs_skip++;
        }
    }
    /* we need save/restore rax if save aflags because rax is used */
    if (!cci->skip_save_aflags && cci->reg_skip[0]) {
        dr_log(dcontext, LOG_CLEANCALL, 3,
               "drcalls: if inserting clean call "PFX
               ", cannot skip saving reg xax.\n", info->start);
        cci->reg_skip[0] = false;
        cci->num_regs_skip--;
    }
}

static void
analyze_clean_call_args(void *dcontext,
                        clean_call_info_t *cci,
                        opnd_t *args)
{
    uint i, j, num_regparm;
    callee_info_t *ci = cci->callee_info;

    num_regparm = cci->num_args < dr_num_reg_parm() ? cci->num_args : dr_num_reg_parm();
    /* If a param uses a reg, DR need restore register value, which assumes
     * the full context switch with dr_mcontext_t layout,
     * in which case we need keep dr_mcontext_t layout.
     */
    cci->save_all_regs = false;
    for (i = 0; i < cci->num_args; i++) {
        if (opnd_is_reg(args[i]))
            cci->save_all_regs = true;
        for (j = 0; j < num_regparm; j++) {
            if (opnd_uses_reg(args[i], dr_reg_parm(j)))
                cci->save_all_regs = true;
        }
    }
    if (!ci->is_leaf) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "drcalls: callee "PFX" is not a leaf, "
               "save all regs in dr_mcontext_t layout.\n",
            ci->start);
        cci->save_all_regs = true;
    }
}

static void
analyze_clean_call_inline(void *dcontext, clean_call_info_t *cci)
{
    uint num_slots;
    callee_info_t *info = cci->callee_info;
    bool opt_inline = true;

    if (opt_cleancall <= 1) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "drcalls: fail inlining clean call "PFX", opt_cleancall %d.\n",
               info->start, opt_cleancall);
        opt_inline = false;
    }
    if (cci->num_args > IF_X64_ELSE(dr_num_reg_parm(), 1)) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "drcalls: fail inlining clean call "PFX", "
               "number of args %d > IF_X64_ELSE(NUM_REGPARM, 1).\n",
            info->start, cci->num_args);
        opt_inline = false;
    }
    if (cci->save_fpstate) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "drcalls: fail inlining clean call "PFX", saving fpstate.\n",
               info->start);
        opt_inline = false;
    }
    if (!info->opt_inline) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "drcalls: fail inlining clean call "PFX", complex callee.\n",
               info->start);
        opt_inline = false;
    }
    if (opt_inline) {
        /* Now check if have enough scratch slots */
        num_slots = NUM_GP_REGS - cci->num_regs_skip;
        if (info->has_locals) {
            /* one more slot for local variable */
            num_slots++;
        }
        if (!cci->skip_save_aflags) {
            /* one more slot for store app's eflags */
            num_slots++;
        }
#ifndef X64
        if (cci->num_args > 0 && cci->reg_skip[0]) {
            /* we use eax to put arg into tls */
            cci->num_regs_skip--;
            cci->reg_skip[0] = false;
            num_slots++;
        }
#endif
        if (num_slots > NUM_SCRATCH_SLOTS) {
            dr_log(dcontext, LOG_CLEANCALL, 2,
                   "drcalls: fail inlining clean call "PFX
                   " need %d slots > available slots.\n",
                   info->start, num_slots, NUM_SCRATCH_SLOTS);
            opt_inline = false;
        }
    }
    if (!opt_inline) {
        if (cci->save_all_regs) {
            dr_log(dcontext, LOG_CLEANCALL, 2,
                   "drcalls: inserting clean call "PFX
                   ", save all regs in dr_mcontext_t layout.\n",
                   info->start);
            cci->num_regs_skip = 0;
            memset(cci->reg_skip, 0, sizeof(bool) * NUM_GP_REGS);
            cci->num_xmms_skip = 0;
            memset(cci->xmm_skip, 0, sizeof(bool) * NUM_XMM_REGS);
            cci->skip_save_aflags = false;
            cci->should_align = true;
        } else {
            uint i;
            for (i = 0; i < NUM_GP_REGS; i++) {
                if (!cci->reg_skip[i] && info->callee_save_regs[i]) {
                    cci->reg_skip[i] = true;
                    cci->num_regs_skip++;
                }
            }
        }
        if (cci->num_xmms_skip == NUM_XMM_REGS) {
            STATS_INC(cleancall_xmm_skipped);
        }
        if (cci->skip_save_aflags) {
            STATS_INC(cleancall_aflags_save_skipped);
        }
        if (cci->skip_clear_eflags) {
            STATS_INC(cleancall_aflags_clear_skipped);
        }
    } else {
        /* TODO(rnk): Which is right!? */
        /* Use global dcontext, since we apply callee optimizations again,
         * which assume instrs are allocated using global dcontext. */
        cci->ilist = instrlist_clone(dcontext, info->ilist);
    }
    cci->opt_inline = opt_inline;
}

clean_call_info_t *
analyze_clean_call(void *dcontext, instr_t *where, void *callee,
                   bool save_fpstate, uint num_args, opnd_t *args)
{
    callee_info_t *ci;
    clean_call_info_t *cci;

    CLIENT_ASSERT(callee != NULL, "Clean call target is NULL");
    /* 1. init clean_call_info */
    cci = clean_call_info_create(dcontext, callee, save_fpstate, args, num_args);
    /* 2. check runtime optimization options */
    if (opt_cleancall == 0)
        return cci;
    ci = callee_info_analyze(dcontext, callee, num_args);

    if (ci->num_args != num_args) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "drcalls: failing to inline callee "PFX": "
               "originally called with %d args now called with %d args\n",
               callee, ci->num_args, num_args);
        return cci;
    }

    cci->callee_info = ci;
    if (ci->bailout) {
        /* TODO(rnk): Doesn't this write race with other readers?  Should
         * callee info be conservatively reinitialized on bailout in the
         * callee analysis code? */
        callee_info_init(ci);
        ci->start = (app_pc)callee;
        return cci;
    }
    /* 5. aflags optimization analysis */
    analyze_clean_call_aflags(dcontext, cci, where);
    /* 6. register optimization analysis */
    analyze_clean_call_regs(dcontext, cci);
    /* 7. check arguments */
    analyze_clean_call_args(dcontext, cci, args);
    /* 8. inline optimization analysis */
    analyze_clean_call_inline(dcontext, cci);
    return cci;
}

/* Maps DR_REG_* enums to offsets into dr_mcontext_t.  Relies on register enum
 * ordering in instr.h.  Indexed by DR_REG_X* - DR_REG_XAX. */
const uint reg_mc_offset[NUM_GP_REGS] = {
    offsetof(dr_mcontext_t, xax),
    offsetof(dr_mcontext_t, xcx),
    offsetof(dr_mcontext_t, xdx),
    offsetof(dr_mcontext_t, xbx),
    offsetof(dr_mcontext_t, xsp),
    offsetof(dr_mcontext_t, xbp),
    offsetof(dr_mcontext_t, xsi),
    offsetof(dr_mcontext_t, xdi)
#ifdef X64
    ,
    offsetof(dr_mcontext_t, r8 ),
    offsetof(dr_mcontext_t, r9 ),
    offsetof(dr_mcontext_t, r10),
    offsetof(dr_mcontext_t, r11),
    offsetof(dr_mcontext_t, r12),
    offsetof(dr_mcontext_t, r13),
    offsetof(dr_mcontext_t, r14),
    offsetof(dr_mcontext_t, r15)
#endif
};

#define XFLAGS_OFFSET offsetof(dr_mcontext_t, xflags)

/* Turn an offset into dr_mcontext_t into an opnd_t that will access that slot
 * for this callee.  For example, mc_frame_opnd(ci, XAX_OFFSET) gets a memory
 * operand for the XAX slot.
 */
static opnd_t
mc_frame_opnd(uint framesize, uint mc_offset)
{
    uint frame_offset = framesize - sizeof(dr_mcontext_t) + mc_offset;
    return OPND_CREATE_MEMPTR(DR_REG_XSP, frame_offset);
}

/* Saves a register to the mcontext at the base of the stack.  Assumes XSP is
 * at dstack - framesize.
 */
static void
insert_mc_reg_save(void *dc, uint framesize, instrlist_t *ilist,
                   instr_t *where, reg_id_t reg)
{
    PRE(ilist, where, INSTR_CREATE_mov_st
        (dc, mc_frame_opnd(framesize, reg_mc_offset[reg - DR_REG_XAX]),
         opnd_create_reg(reg)));
}

/* Loads a register from the mcontext at the base of the stack.  Assumes XSP is
 * at dstack - framesize.
 */
static void
insert_mc_reg_restore(void *dc, uint framesize, instrlist_t *ilist,
                      instr_t *where, reg_id_t reg)
{
    PRE(ilist, where, INSTR_CREATE_mov_ld
        (dc, opnd_create_reg(reg),
         mc_frame_opnd(framesize, reg_mc_offset[reg - DR_REG_XAX])));
}

/* Saves aflags to the mcontext at the base of the stack.  Assumes XSP is at
 * dstack - framesize.  Assumes that XAX is dead and can be used as scratch.
 */
static void
insert_mc_flags_save(void *dc, uint framesize, instrlist_t *ilist,
                     instr_t *where)
{
    PRE(ilist, where, INSTR_CREATE_lahf(dc));
    PRE(ilist, where, INSTR_CREATE_setcc
        (dc, OP_seto, opnd_create_reg(DR_REG_AL)));
    PRE(ilist, where, INSTR_CREATE_mov_st
        (dc, mc_frame_opnd(framesize, XFLAGS_OFFSET),
         opnd_create_reg(DR_REG_XAX)));
}

/* Saves aflags to the mcontext at the base of the stack.  Assumes XSP is at
 * dstack - framesize.  Assumes that XAX is dead and can be used as scratch.
 */
static void
insert_mc_flags_restore(void *dc, uint framesize, instrlist_t *ilist,
                        instr_t *where)
{
    PRE(ilist, where, INSTR_CREATE_mov_ld
        (dc, opnd_create_reg(DR_REG_XAX),
         mc_frame_opnd(framesize, XFLAGS_OFFSET)));
    PRE(ilist, where, INSTR_CREATE_add
        (dc, opnd_create_reg(DR_REG_AL), OPND_CREATE_INT8(0x7F)));
    PRE(ilist, where, INSTR_CREATE_sahf(dc));
}

static opnd_t
opnd_get_tls_xax(void *dc)
{
    /* XXX: We access the XAX slot by first requesting an opnd for slot 3, which
     * is xbx's slot, and we know xax is one before xbx, so we subtract the size
     * of a slot from the displacement. */
    /* TODO(rnk): This is a shameful way to access XAX, but I don't
     * actually want to expose this information to clients.  */
    opnd_t slot = dr_reg_spill_slot_opnd(dc, SPILL_SLOT_3);
    opnd_set_disp(&slot, opnd_get_disp(slot) - sizeof(reg_t));
    return slot;
}

static opnd_t
opnd_get_tls_dcontext(void *dc)
{
    /* XXX: We access the dcontext slot by first requesting an opnd for slot 1,
     * which is xdx's slot, and we know xdx is one before dcontext, so we add
     * the size of a slot to the displacement. */
    opnd_t slot = dr_reg_spill_slot_opnd(dc, SPILL_SLOT_1);
    opnd_set_disp(&slot, opnd_get_disp(slot) + sizeof(reg_t));
    return slot;
}

static uint
get_dstack_offset(void)
{
    /* XXX: Relying on undocumented DynamoRIO internals: dcontext is not exposed
     * to extensions, but dstack is part of the offset-critical portion of
     * dcontext, so it's offset is unlikely to change.  We hardcode the size
     * here.
     *
     * Layout is the following:
     * dcontext {
     *   upcontext {
     *     priv_mcontext {
     *       gprs
     *       flags
     *       pc
     *       padding
     *       SSE
     *     }
     *     int
     *     bool
     *   }
     *   ptr
     *   ptr
     *   ptr
     *   dstack
     * }
     */
#ifdef X64
# ifdef WINDOWS
#  define NUM_XMM_SLOTS 6 /* xmm0-5 */
# else
#  define NUM_XMM_SLOTS 16 /* xmm0-15 */
# endif
# define PRE_XMM_PADDING 16
#else
# define NUM_XMM_SLOTS 8 /* xmm0-7 */
# define PRE_XMM_PADDING 24
#endif
#define XMM_SAVED_REG_SIZE  32
    uint psz = sizeof(reg_t);
    uint priv_size = IF_X64_ELSE(16, 8) * psz + 2 * psz;
    priv_size += PRE_XMM_PADDING + NUM_XMM_SLOTS * XMM_SAVED_REG_SIZE;
    priv_size = ALIGN_FORWARD_UINT(priv_size + sizeof(int) + sizeof(bool), psz);
    return priv_size + 3 * psz;
}

static void
insert_switch_to_dstack(void *dc, const callee_info_t *ci, instrlist_t *ilist,
                        instr_t *where)
{
    PRE(ilist, where, instr_create_0dst_1src
        (dc, DRC_OP_dstack, OPND_CREATE_INT32(ci->framesize)));
}

static uint
get_framesize(instr_t *instr)
{
    ASSERT(instr_get_opcode(instr) == DRC_OP_dstack ||
           instr_get_opcode(instr) == DRC_OP_appstack);
    return (uint)opnd_get_immed_int(instr_get_src(instr, 0));
}

static void
expand_op_dstack(void *dc, instrlist_t *ilist, instr_t *where)
{
    opnd_t xsp = opnd_create_reg(DR_REG_XSP);
    uint framesize = get_framesize(where);

    /* Spill XSP to TLS_XAX_SLOT because it's not exposed to the client. */
    PRE(ilist, where, INSTR_CREATE_mov_st(dc, opnd_get_tls_xax(dc), xsp));
    /* Switch to dstack. */
    PRE(ilist, where, INSTR_CREATE_mov_ld(dc, xsp, opnd_get_tls_dcontext(dc)));
    PRE(ilist, where, INSTR_CREATE_mov_ld
        (dc, xsp, OPND_CREATE_MEMPTR(DR_REG_XSP, get_dstack_offset())));
    /* Make room for a partially initialized mcontext
     * structure. */
    PRE(ilist, where, INSTR_CREATE_lea
        (dc, xsp, OPND_CREATE_MEM_lea(DR_REG_XSP, DR_REG_NULL, 0, -framesize)));
}

static void
insert_switch_to_appstack(void *dc, const callee_info_t *ci, instrlist_t *ilist,
                          instr_t *where)
{
    PRE(ilist, where, instr_create_0dst_1src
        (dc, DRC_OP_appstack, OPND_CREATE_INT32(ci->framesize)));
}

static void
expand_op_appstack(void *dc, instrlist_t *ilist, instr_t *where)
{
    opnd_t xsp = opnd_create_reg(DR_REG_XSP);
    PRE(ilist, where, INSTR_CREATE_mov_ld(dc, xsp, opnd_get_tls_xax(dc)));
}

static void
insert_inline_reg_save(void *dc, clean_call_info_t *cci,
                       instrlist_t *ilist, instr_t *where, opnd_t *args)
{
    callee_info_t *ci = cci->callee_info;
    int i;

    ASSERT(cci->num_xmms_skip == NUM_XMM_REGS);
    for (i = 0; i < NUM_GP_REGS; i++) {
        if (!cci->reg_skip[i]) {
            reg_id_t reg = DR_REG_XAX + (reg_id_t)i;
            dr_log(dc, LOG_CLEANCALL, 2,
                   "drcalls: inlining clean call "PFX", saving reg %s.\n",
                   ci->start, get_register_name(reg));
            insert_mc_reg_save(dc, ci->framesize, ilist, where, reg);
        }
    }

    if (!cci->skip_save_aflags) {
        DR_ASSERT_MSG(!cci->reg_skip[0], "XAX must be saved to save aflags!");
        insert_mc_flags_save(dc, ci->framesize, ilist, where);
    }
}

static void
insert_inline_reg_restore(void *dc, clean_call_info_t *cci,
                          instrlist_t *ilist, instr_t *where)
{
    int i;
    callee_info_t *ci = cci->callee_info;

    /* aflags is first because it uses XAX. */
    if (!cci->skip_save_aflags) {
        insert_mc_flags_restore(dc, ci->framesize, ilist, where);
    }

    /* Now restore all registers. */
    for (i = 0; i < NUM_GP_REGS; i++) {
        if (!cci->reg_skip[i]) {
            reg_id_t reg = DR_REG_XAX + (reg_id_t)i;
            dr_log(dc, LOG_CLEANCALL, 2,
                   "drcalls: inlining clean call "PFX", restoring reg %s.\n",
                   ci->start, get_register_name(reg));
            insert_mc_reg_restore(dc, ci->framesize, ilist, where, reg);
        }
    }
}

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

static void
insert_inline_arg_setup(void *dc, clean_call_info_t *cci, instrlist_t *ilist,
                        instr_t *where, opnd_t *args, bool is_slowpath)
{
    uint i;
    callee_info_t *ci = cci->callee_info;

    if (cci->num_args == 0)
        return;

    ASSERT(cci->num_args <= IF_X64_ELSE(dr_num_reg_parm(), 1));
    for (i = 0; i < cci->num_args; i++) {
        reg_id_t reg = IF_X64_ELSE(dr_reg_parm(i), DR_REG_XAX);
        opnd_t dst_opnd;
        if (!ci->reg_used[reg - DR_REG_XAX]) {
            if (!is_slowpath) {
                dr_log(dc, LOG_CLEANCALL, 2,
                       "drcalls: skipping arg setup for dead reg %s\n",
                       get_register_name(reg));
                continue;
            }
        }
        reg = shrink_reg_for_param(reg, args[i]);
        dst_opnd = opnd_create_reg(reg);
        dr_log(dc, LOG_CLEANCALL, 2,
               "drcalls: inlining clean call "PFX", passing arg via reg %s.\n",
               ci->start, get_register_name(reg));
        if (opnd_is_immed_int(args[i])) {
            PRE(ilist, where, INSTR_CREATE_mov_imm (dc, dst_opnd, args[i]));
        } else if (opnd_is_reg(args[i])) {
            reg_id_t src = reg_to_pointer_sized(opnd_get_reg(args[i]));
            if (src == DR_REG_XSP) {
                DR_ASSERT_MSG(false, "Not yet tested");
                PRE(ilist, where, INSTR_CREATE_mov_ld
                    (dc, dst_opnd, opnd_get_tls_xax(dc)));
            } else {
                if (is_slowpath && ci->reg_used[src - DR_REG_XAX]) {
                    /* If we used src in the inline code, we need to restore it
                     * to the application value before reading it again.
                     */
                    opnd_t mc_src = mc_frame_opnd
                        (ci->framesize, reg_mc_offset[src - DR_REG_XAX]);
                    PRE(ilist, where,
                        INSTR_CREATE_mov_ld(dc, dst_opnd, mc_src));
                    DR_ASSERT_MSG(false, "Not yet tested");
                } else {
                    PRE(ilist, where,
                        INSTR_CREATE_mov_ld(dc, dst_opnd, args[i]));
                }
            }
        } else {
            PRE(ilist, where, INSTR_CREATE_mov_ld(dc, dst_opnd, args[i]));
        }
    }
#ifndef X64
    ASSERT(!cci->reg_skip[0]);
    /* Move xax to the local variable stack slot.  We can use the local
     * variable stack slot because we only allow at most one local stack
     * access, so callee either does not use the argument, or the local stack
     * access is the arg.
     */
    dr_log(dc, LOG_CLEANCALL, 2,
           "drcalls: inlining clean call "PFX", passing arg via slot %d.\n",
           ci->start, NUM_SCRATCH_SLOTS - 1);
    PRE(ilist, where, INSTR_CREATE_mov_st
        (dc, OPND_CREATE_MEMPTR(DR_REG_XSP, 0), opnd_create_reg(DR_REG_XAX)));
#endif
}

/* Insert call-site specific code for switching from the partially inlined fast
 * path to the out-of-line slowpath.  For example, arguments may need to be
 * rematerialized.
 */
static void
insert_inline_slowpath(void *dc, clean_call_info_t *cci, opnd_t *args)
{
    callee_info_t *ci = cci->callee_info;
    instrlist_t *ilist = cci->ilist;
    instr_t *instr;
    instr_t *slowpath_call = NULL;
    uint i;

    ASSERT(ci->opt_partial);
    for (instr = instrlist_first(ilist); instr != NULL;
         instr = instr_get_next(instr)) {
        /* If we find the slowpath, insert call-site specific setup. */
        if (ci->opt_partial && instr_is_call(instr) &&
            opnd_is_pc(instr_get_target(instr)) &&
            opnd_get_pc(instr_get_target(instr)) == ci->partial_pc) {
            slowpath_call = instr;
            break;
        }
    }
    ASSERT(slowpath_call != NULL);

    /* XXX: If the callee didn't use XAX and did use flags, we may have saved
     * XAX in order to save flags inline.  The slowpath assumes the inline code
     * always saves XAX if there is any flags usage.  If aflags were dead so we
     * didn't save flags and hence XAX, we will need to save XAX here. */
    if (!ci->reg_used[DR_REG_XAX - DR_REG_XAX] &&
        (ci->read_aflags || ci->write_aflags) &&
        cci->skip_save_aflags &&
        cci->reg_skip[DR_REG_XAX - DR_REG_XAX]) {
        insert_mc_reg_save(dc, ci->framesize, ilist, slowpath_call, DR_REG_XAX);
        insert_mc_reg_restore(dc, ci->framesize, ilist,
                              instr_get_next(slowpath_call), DR_REG_XAX);
    }

    /* Arg setup may use registers that weren't used inline, and therefore
     * won't be saved. */
    for (i = 0; i < cci->num_args; i++) {
        reg_id_t reg = IF_X64_ELSE(dr_reg_parm(i), DR_REG_XAX);
        if (!ci->reg_used[reg - DR_REG_XAX]) {
            insert_mc_reg_save(dc, ci->framesize, ilist, slowpath_call, reg);
            insert_mc_reg_restore(dc, ci->framesize, ilist,
                                  instr_get_next(slowpath_call), reg);
        }
    }

    /* Assert that reg_skip and reg_used agree, except in the case of XAX,
     * which we deal with above. */
    for (i = 0; i < NUM_GP_REGS; i++) {
        DR_ASSERT_MSG(ci->reg_used[i] == !cci->reg_skip[i] ||
                      (i == 0 && !cci->reg_skip[i]),
                      "reg_used and reg_skip don't agree!");
    }

    /* TODO(rnk): Could use an analysis to see which args are still live, since
     * chances are most are.  The slowpath is not executed frequently, but
     * optimizing it should reduce code size. */
    insert_inline_arg_setup(dc, cci, ilist, slowpath_call, args, true);
}

void
insert_inline_clean_call(void *dcontext, clean_call_info_t *cci,
                         instrlist_t *ilist, instr_t *where)
{
    callee_info_t *ci = cci->callee_info;
    instrlist_t *callee = cci->ilist;
    opnd_t *args = cci->args;
    instr_t *instr;

    /* Insert argument setup.  Do some optimizations to try to fold the
     * arguments in. */
    /* TODO(rnk): We can't run the full set of optimizations because they
     * assume the ilist was allocated from GLOBAL_DCONTEXT, which this is not.
     */
    insert_inline_arg_setup(dcontext, cci, callee, instrlist_first(callee),
                            args, false);
    try_fold_immeds(dcontext, dcontext, callee);
    if (ci->opt_partial) {
        insert_inline_slowpath(dcontext, cci, args);
    }

    ASSERT(cci->ilist != NULL);
    insert_switch_to_dstack(dcontext, ci, ilist, where);
    insert_inline_reg_save(dcontext, cci, ilist, where, args);
    instr = instrlist_first(callee);
    while (instr != NULL) {
        instrlist_remove(callee, instr);
        instr_set_translation(instr, NULL);
        /* The inlined code might cause access violation, so set to may fault.
         * XXX: anything else we should do?
         */
        instr_set_meta_may_fault(instr, true);
        instrlist_meta_preinsert(ilist, where, instr);
        instr = instrlist_first(callee);
    }
    instrlist_destroy(dcontext, callee);
    cci->ilist = NULL;
    insert_inline_reg_restore(dcontext, cci, ilist, where);
    insert_switch_to_appstack(dcontext, ci, ilist, where);
}

/* Save or restore all application registers that weren't saved inline. */
static void
insert_slowpath_mc_regs(void *dc, callee_info_t *ci, bool save,
                        instrlist_t *ilist)
{
    reg_id_t reg;
    int i;
    uint framesize = ci->framesize + 16;  /* ret addr + alignment */

    for (reg = DR_REG_XAX; reg < DR_REG_XAX + NUM_GP_REGS; reg++) {
        bool is_arg;
        /* If the register was used inline it was saved inline. */
        if (ci->reg_used[reg - DR_REG_XAX])
            continue;
        /* If the register is XSP, we treat it separately. */
        if (reg == DR_REG_XSP)
            continue;
        /* XXX: If flags had to be saved, then XAX was saved inline and
         * clobbered, so we should *not* save it. */
        if (reg == DR_REG_XAX && (ci->read_aflags || ci->write_aflags))
            continue;
        /* If the register was used to pass an argument, it must have been
         * saved inline.  The callee may not actually *use* the argument
         * inline, so we have to check this case as well. */
        is_arg = false;
        for (i = 0; i < ci->num_args; i++) {
            if (dr_reg_parm(i) == reg) {
                is_arg = true;
                break;
            }
        }
        if (is_arg)
            continue;

        if (save)
            insert_mc_reg_save(dc, framesize, ilist, NULL, reg);
        else
            insert_mc_reg_restore(dc, framesize, ilist, NULL, reg);
    }
}

/* Save or restore flags if it wasn't saved inline. */
static void
insert_slowpath_mc_flags(void *dc, callee_info_t *ci, bool save,
                         instrlist_t *ilist)
{
    /* Save flags, but only if they weren't already saved, which happens if
     * there was either a read or a write (a read implies a clear, which is a
     * write). */
    /* TODO(rnk): Unduplicate this logic with cci->skip_save_aflags. */
    if (!(ci->read_aflags || ci->write_aflags)) {
        /* TODO(rnk): This clobbers XAX, which could be used to pass args into
         * slowpath. */
        /* The + 16 is for ret addr + alignment. */
        if (save)
            insert_mc_flags_save(dc, ci->framesize + 16, ilist, NULL);
        else
            insert_mc_flags_restore(dc, ci->framesize + 16, ilist, NULL);
    }
}

static uint
move_mm_reg_opcode(bool aligned16, bool aligned32)
{
    if (YMM_ENABLED()) {
        /* must preserve ymm registers */
        return (aligned32 ? OP_vmovdqa : OP_vmovdqu);
    }
    else if (proc_has_feature(FEATURE_SSE2)) {
        return (aligned16 ? OP_movdqa : OP_movdqu);
    } else {
        CLIENT_ASSERT(proc_has_feature(FEATURE_SSE), "running on unsupported processor");
        return (aligned16 ? OP_movaps : OP_movups);
    }
}

static void
insert_slowpath_mc(void *dc, callee_info_t *ci, bool save,
                   instrlist_t *ilist)
{
    uint framesize = ci->framesize + 16;  /* ret addr + alignment */

    /* Save/restore flags uses XAX, so on save it must come after regs, and on
     * restore it must come first. */
    if (save) {
        insert_slowpath_mc_regs(dc, ci, save, ilist);
        insert_slowpath_mc_flags(dc, ci, save, ilist);
    } else {
        insert_slowpath_mc_flags(dc, ci, save, ilist);
        insert_slowpath_mc_regs(dc, ci, save, ilist);
    }

    /* Save XMM regs, if appropriate. */
    if (dr_mcontext_xmm_fields_valid()) {
        /* PR 264138: we must preserve xmm0-5 if on a 64-bit kernel */
        /* We align the stack ourselves, so we assume aligned SSE ops are OK. */
        int i;
        uint opcode = move_mm_reg_opcode(/*aligned16=*/true, /*aligned32=*/true);
        for (i = 0; i < NUM_XMM_SAVED; i++) {
            uint mc_offset = offsetof(dr_mcontext_t, ymm) + i * XMM_SAVED_REG_SIZE;
            uint frame_offset = framesize - sizeof(dr_mcontext_t) + mc_offset;
            opnd_t reg = opnd_create_reg(REG_SAVED_XMM0 + (reg_id_t)i);
            opnd_t mem = opnd_create_base_disp(DR_REG_XSP, DR_REG_NULL, 0,
                                               frame_offset, OPSZ_16);
            /* If we're saving, it's reg -> mem, otherwise mem -> reg. */
            opnd_t src = save ? reg : mem;
            opnd_t dst = save ? mem : reg;
            APP(ilist, instr_create_1dst_1src(dc, opcode, dst, src));
        }
    }

    /* FIXME i#433: need DR cxt switch and clean call to preserve ymm */
}

/* Emit the slowpath back to the callee for partially inlined functions.
 * Return the slowpath entry point. */
app_pc
emit_partial_slowpath(void *dc, callee_info_t *ci)
{
    byte *entry;
    instrlist_t *ilist;
    opnd_t xsp = opnd_create_reg(DR_REG_XSP);
    uint realignment;

    dr_log(dc, LOG_CLEANCALL, 3,
           "drcalls: emitting partial inline slowpath\n");

    /* Generate the clean call ilist.  Arguments should be materialized into
     * registers at the callsite.  Application register values are already on
     * the stack. */
    ilist = instrlist_create(dc);
    /* Re-align stack to 16 bytes to prepare for call. */
    realignment = (16 - sizeof(reg_t));
    APP(ilist, INSTR_CREATE_lea
        (dc, xsp, OPND_CREATE_MEM_lea(DR_REG_XSP, DR_REG_NULL, 0, -realignment)));
    insert_slowpath_mc(dc, ci, /*save=*/true, ilist);

    /* Emit side-entry code based on original callee entry. */
    APP(ilist, INSTR_CREATE_call(dc, opnd_create_pc(ci->start)));

    insert_slowpath_mc(dc, ci, /*save=*/false, ilist);
    /* Un-align stack to get back to ret addr. */
    APP(ilist, INSTR_CREATE_lea
        (dc, xsp, OPND_CREATE_MEM_lea(DR_REG_XSP, DR_REG_NULL, 0, realignment)));
    APP(ilist, INSTR_CREATE_ret(dc));

    dr_log(dc, LOG_CACHE, 3, "drcalls: emitting partial slowpath\n");
    entry = code_cache_emit(dc, ilist);

    instrlist_clear_and_destroy(dc, ilist);

    ci->partial_pc = entry;
    return entry;
}

void
schedule_call_upwards(void *dc, clean_call_info_t *cci, instrlist_t *ilist,
                      instr_t *call)
{
    instr_t *instr;
    bool regs_in_args[NUM_GP_REGS];
    int i, j;

    memset(regs_in_args, 0, sizeof(regs_in_args));
    for (i = 0; i < cci->num_args; i++) {
        opnd_t arg = cci->args[i];
        for (j = 0; j < opnd_num_regs_used(arg); j++) {
            reg_id_t reg = opnd_get_reg_used(arg, j);
            reg = reg_to_pointer_sized(reg);
            regs_in_args[reg - DR_REG_XAX] = true;
        }
    }

    for (instr = instr_get_prev(call); instr != NULL;
         instr = instr_get_prev(instr)) {
        if (instr_get_opcode(instr) == DRC_OP_call) {
            /* Move the call after the one we found. */
            instrlist_remove(ilist, call);
            POST(ilist, instr, call);
            break;
        }

        /* We can't move a call that reads a register across one that uses it.
         */
        for (i = 0; i < NUM_GP_REGS; i++) {
            reg_id_t reg = DR_REG_XAX + (reg_id_t)i;
            if (regs_in_args[i] && instr_writes_to_reg(instr, reg)) {
                dr_log(dc, LOG_CLEANCALL, 3,
                       "drcalls: stopping call reschedule for write of arg reg\n");
                break;
            }
        }
    }
}

void
expand_and_optimize_bb(void *dc, instrlist_t *bb)
{
    instr_t *instr;
    instr_t *next_instr;
    bool found_call = false;

    /* Reschedule. */
    for (instr = instrlist_first(bb); instr != NULL; instr = next_instr) {
        clean_call_info_t *cci; 
        callee_info_t *ci;
        opnd_t *args;

        next_instr = instr_get_next(instr);
        if (instr_get_opcode(instr) != DRC_OP_call)
            continue;
        if (!found_call) {
            /* Don't touch the first call.  We could try to schedule it
             * downwards, but we don't yet. */
            found_call = true;
            continue;
        }

        cci = (clean_call_info_t *)instr_get_note(instr);
        ci = (callee_info_t *)cci->callee_info;
        args = cci->args;

        /* For now we only touch non-partially inlined calls, since we might
         * disturb the values in mcontext. */
        if (ci->opt_partial) {
            continue;
        }

        schedule_call_upwards(dc, cci, bb, instr);
    }

    for (instr = instrlist_first(bb); instr != NULL; instr = next_instr) {
        next_instr = instr_get_next(instr);

        if (instr_get_opcode(instr) == DRC_OP_call) {
            clean_call_info_t *cci = (clean_call_info_t *)instr_get_note(instr);
            insert_inline_clean_call(dc, cci, bb, next_instr);
            remove_and_destroy(dc, bb, instr);
            clean_call_info_destroy(dc, cci);
        }
    }

    for (instr = instrlist_first(bb); instr != NULL; instr = next_instr) {
        bool pseudo_instr;
        uint opc = instr_get_opcode(instr);
        next_instr = instr_get_next(instr);

        if (next_instr != NULL) {
            uint next_opc = instr_get_opcode(next_instr);

            if (opc == DRC_OP_appstack &&
                next_opc == DRC_OP_dstack &&
                get_framesize(instr) == get_framesize(next_instr)) {
                /* If we switch to appstack and immediately back to dstack with the
                 * same framesizes, just stay on dstack. */
                instr_t *tmp = instr_get_prev(instr);
                dr_log(dc, LOG_CLEANCALL, 3,
                       "drcalls: deleting appstack/dstack pair\n");
                remove_and_destroy(dc, bb, instr);
                remove_and_destroy(dc, bb, next_instr);
                next_instr = tmp;
                continue;
            }
            /* TODO(rnk): This is incorrect, should have pseudo-op.  Also
             * restore might be needed if reg is live-in to call. */
            if (opc == OP_mov_ld && next_opc == OP_mov_st) {
                /* Match "reg restore ; reg save". */
                opnd_t reg = instr_get_dst(instr, 0);
                opnd_t mem = instr_get_src(instr, 0);
                if (opnd_same(reg, instr_get_src(next_instr, 0)) &&
                    opnd_same(mem, instr_get_dst(next_instr, 0))) {
                    instr_t *tmp = instr_get_prev(instr);
                    dr_log(dc, LOG_CLEANCALL, 3,
                           "drcalls: deleting restore/save pair\n");
                    remove_and_destroy(dc, bb, instr);
                    remove_and_destroy(dc, bb, next_instr);
                    next_instr = tmp;
                    continue;
                }
            }
        }

        pseudo_instr = true;
        switch (opc) {
        case DRC_OP_dstack:
            expand_op_dstack(dc, bb, instr);
            break;
        case DRC_OP_appstack:
            expand_op_appstack(dc, bb, instr);
            break;
        default:
            pseudo_instr = false;
            break;
        }
        if (pseudo_instr) {
            remove_and_destroy(dc, bb, instr);
        }
    }

    redundant_load_elim(dc, dc, bb);
    dead_store_elim(dc, dc, bb);
}
