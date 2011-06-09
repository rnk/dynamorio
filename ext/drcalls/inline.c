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

#include <stddef.h> /* offsetof */
#include <string.h> /* for memcmp */

/* internal includes */
#include "callee.h"
#include "clean_call.h"
#include "code_cache.h"
#include "core_compat.h"
#include "inline.h"
#include "mcontext.h"
#include "optimize.h"

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
    rewrite_opnd_table_init();
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

    num_regparm = (cci->num_args < dr_num_reg_parm() ?
                   cci->num_args : dr_num_reg_parm());
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
            /* we use eax to put arg onto stack */
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

static void
insert_inline_reg_save(void *dc, clean_call_info_t *cci,
                       bool reg_used[NUM_GP_REGS], instrlist_t *ilist,
                       instr_t *where)
{
    callee_info_t *ci = cci->callee_info;
    int i;

    for (i = 0; i < NUM_GP_REGS; i++) {
        if (reg_used[i]) {
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
                          bool reg_used[NUM_GP_REGS], instrlist_t *ilist,
                          instr_t *where)
{
    int i;
    callee_info_t *ci = cci->callee_info;

    /* aflags is first because it uses XAX. */
    if (!cci->skip_save_aflags) {
        insert_mc_flags_restore(dc, ci->framesize, ilist, where);
    }

    /* Now restore all registers. */
    for (i = 0; i < NUM_GP_REGS; i++) {
        if (reg_used[i]) {
            reg_id_t reg = DR_REG_XAX + (reg_id_t)i;
            dr_log(dc, LOG_CLEANCALL, 2,
                   "drcalls: inlining clean call "PFX", restoring reg %s.\n",
                   ci->start, get_register_name(reg));
            insert_mc_reg_restore(dc, ci->framesize, ilist, where, reg);
        }
    }
}

static void
insert_inline_arg_setup(void *dc, clean_call_info_t *cci, instrlist_t *ilist,
                        instr_t *where, opnd_t *args, bool is_slowpath)
{
    uint i;
    callee_info_t *ci = cci->callee_info;
    bool regs_from_mc[NUM_GP_REGS];

    if (cci->num_args == 0)
        return;

    if (is_slowpath) {
        /* If we're in the slowpath, any registers used inline will have been
         * saved and possibly clobbered, so we reload them from the mcontext. */
        memcpy(regs_from_mc, ci->reg_used, sizeof(regs_from_mc));
    } else {
        memset(regs_from_mc, 0, sizeof(regs_from_mc));
    }

    ASSERT(cci->num_args <= IF_X64_ELSE(dr_num_reg_parm(), 1));
    for (i = 0; i < cci->num_args; i++) {
        reg_id_t reg = IF_X64_ELSE(dr_reg_parm(i), DR_REG_XAX);
        if (!ci->reg_used[reg - DR_REG_XAX]) {
            if (!is_slowpath) {
                dr_log(dc, LOG_CLEANCALL, 2,
                       "drcalls: skipping arg setup for dead reg %s\n",
                       get_register_name(reg));
                continue;
            }
        }
        dr_log(dc, LOG_CLEANCALL, 2,
               "drcalls: inlining clean call "PFX", passing arg via reg %s.\n",
               ci->start, get_register_name(reg));
        materialize_arg_into_reg(dc, ilist, where, ci->framesize, regs_from_mc,
                                 args[i], reg);
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
insert_inline_slowpath(void *dc, clean_call_info_t *cci,
                       bool reg_used[NUM_GP_REGS], opnd_t *args)
{
    callee_info_t *ci = cci->callee_info;
    instrlist_t *ilist = cci->ilist;
    instr_t *instr;
    instr_t *slowpath_call = NULL;
    uint i, j;
    bool reg_save_slowpath[NUM_GP_REGS];

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

    /* reg_used[i] is the registers used by the callee after args are folded in
     * during inlining.  It is a subset of ci->reg_used, which are the registers
     * used by the optimized callee.
     *
     * During the normal inline reg save, we save:
     *  reg_used[i]
     * During the shared slowpath, we save:
     *  !ci->reg_used[i]
     * Here, we save:
     *  ci->reg_used[i] && !reg_used[i]
     *
     * Compute these explicitly?  Into:
     * reg_save_inline   - always saved inline
     * reg_save_shared   - always saved out of line
     * reg_save_slowpath - saved on the transition out of line
     *
     * reg_save_inline = reg_used + xax(if flags)
     * reg_save_shared = ~ci->reg_used - arg_regs
     * reg_save_slowpath = ~(reg_save_inline | reg_save_shared)
     */

    memset(reg_save_slowpath, 0, sizeof(reg_save_slowpath));
    for (i = 0; i < NUM_GP_REGS; i++) {
        reg_id_t reg = DR_REG_XAX + (reg_id_t)i;
        bool saved_inline = reg_used[i];
        bool saved_shared = !ci->reg_used[i];
        /* Arg regs are not saved in the shared slowpath. */
        for (j = 0; j < cci->num_args; j++)
            if (reg == dr_reg_parm(j))
                saved_shared = false;
        reg_save_slowpath[i] = !(saved_inline || saved_shared);
    }

    /* XXX: If the callee didn't use XAX and did use flags, we may have saved
     * XAX in order to save flags inline.  The slowpath assumes the inline code
     * always saves XAX if there is any flags usage.  If aflags were dead so we
     * didn't save flags and hence XAX, we will need to save XAX here. */
    if (!ci->reg_used[DR_REG_XAX - DR_REG_XAX] &&
        (ci->read_aflags || ci->write_aflags) &&
        cci->skip_save_aflags &&
        !reg_used[DR_REG_XAX - DR_REG_XAX]) {
        reg_save_slowpath[DR_REG_XAX - DR_REG_XAX] = true;
    }

    insert_mc_regs(dc, ci->framesize, /*save=*/true, reg_save_slowpath, ilist,
                   slowpath_call);
    /* TODO(rnk): Could use an analysis to see which args are still live, since
     * chances are most are.  The slowpath is not executed frequently, but
     * optimizing it should reduce code size. */
    insert_inline_arg_setup(dc, cci, ilist, slowpath_call, args, true);
    insert_mc_regs(dc, ci->framesize, /*save=*/false, reg_save_slowpath, ilist,
                   instr_get_next(slowpath_call));
}

#define FOR_EACH_OPND_IN_INSTR(instr, opnd_var, is_dst_var, body) \
    do { \
        int i; \
        int num_srcs = instr_num_srcs(instr); \
        int num_dsts = instr_num_dsts(instr); \
        for (i = 0; i < num_srcs + num_dsts; i++) { \
            bool is_dst_var = (i < num_srcs); \
            opnd_t opnd_var = (is_dst_var ? \
                               instr_get_src(instr, i) : \
                               instr_get_dst(instr, i - num_srcs)); \
            body \
    } while (0)

static void
compute_regs_used(void *dc, instrlist_t *ilist, clean_call_info_t *cci,
                  bool reg_used[NUM_GP_REGS])
{
    instr_t *instr;
    int i, j;
    memset(reg_used, 0, sizeof(bool) * NUM_GP_REGS);
    for (instr = instrlist_first(ilist); instr != NULL;
         instr = instr_get_next(instr)) {
        for (i = 0; i < instr_num_srcs(instr); i++) {
            opnd_t opnd = instr_get_src(instr, i);
            for (j = 0; j < opnd_num_regs_used(opnd); j++) {
                reg_id_t reg = reg_to_pointer_sized(opnd_get_reg_used(opnd, j));
                if (!reg_is_gpr(reg)) continue;
                reg_used[reg - DR_REG_XAX] = true;
            }
        }
        for (i = 0; i < instr_num_dsts(instr); i++) {
            opnd_t opnd = instr_get_dst(instr, i);
            for (j = 0; j < opnd_num_regs_used(opnd); j++) {
                reg_id_t reg = reg_to_pointer_sized(opnd_get_reg_used(opnd, j));
                if (!reg_is_gpr(reg)) continue;
                reg_used[reg - DR_REG_XAX] = true;
            }
        }
    }
    /* XSP is saved in TLS. */
    reg_used[DR_REG_XSP - DR_REG_XAX] = false;
    /* we need save/restore rax if save aflags because rax is used */
    if (!cci->skip_save_aflags) {
        dr_log(dc, LOG_CLEANCALL, 3,
               "drcalls: cannot skip saving reg xax.\n");
        reg_used[0] = true;
    }
}

void
insert_inline_clean_call(void *dcontext, clean_call_info_t *cci,
                         instrlist_t *ilist, instr_t *where)
{
    callee_info_t *ci = cci->callee_info;
    instrlist_t *callee = cci->ilist;
    opnd_t *args = cci->args;
    instr_t *instr;
    bool reg_used[NUM_GP_REGS];

    /* Insert argument setup.  Do some optimizations to try to fold the
     * arguments in. */
    /* TODO(rnk): We can't run the full set of optimizations because they
     * assume the ilist was allocated from GLOBAL_DCONTEXT, which this is not.
     */
    insert_inline_arg_setup(dcontext, cci, callee, instrlist_first(callee),
                            args, false);
    try_fold_immeds(dcontext, dcontext, callee);
    fold_leas(dcontext, dcontext, callee);
    compute_regs_used(dcontext, callee, cci, reg_used);
    if (ci->opt_partial) {
        insert_inline_slowpath(dcontext, cci, reg_used, args);
    }

    ASSERT(cci->ilist != NULL);
    insert_switch_to_dstack(dcontext, ci->framesize, ilist, where);
    insert_inline_reg_save(dcontext, cci, reg_used, ilist, where);
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
    insert_inline_reg_restore(dcontext, cci, reg_used, ilist, where);
    insert_switch_to_appstack(dcontext, ci->framesize, ilist, where);
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

static void
insert_slowpath_mc(void *dc, callee_info_t *ci, bool save, instrlist_t *ilist)
{
    uint framesize = ci->framesize + 16;  /* ret addr + alignment */
    bool reg_save_shared[NUM_GP_REGS];
    reg_id_t reg;
    int i;

    /* Calculate which GPRs to save in the slowpath. */
    memset(reg_save_shared, 0, sizeof(reg_save_shared));  /* Default to none. */
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
        reg_save_shared[reg - DR_REG_XAX] = true;
    }

    /* Save/restore flags uses XAX, so on save it must come after regs, and on
     * restore it must come first. */
    if (save) {
        insert_mc_regs(dc, framesize, save, reg_save_shared, ilist, NULL);
        insert_slowpath_mc_flags(dc, ci, save, ilist);
        insert_mc_xmm_regs(dc, framesize, save, ilist, NULL);
    } else {
        insert_mc_xmm_regs(dc, framesize, save, ilist, NULL);
        insert_slowpath_mc_flags(dc, ci, save, ilist);
        insert_mc_regs(dc, framesize, save, reg_save_shared, ilist, NULL);
    }
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
    fold_leas(dc, dc, bb);
}
