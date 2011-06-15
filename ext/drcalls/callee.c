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

/* This module is responsible for callee decoding/analysis. */

#include "dr_api.h"
#include "hashtable.h"

#include "callee.h"
#include "core_compat.h"
#include "inline.h"
#include "optimize.h"

#include <string.h>

/* Fwd decls. */
static void decode_callee_ilist(void *dc, callee_info_t *ci);
static void analyze_callee_ilist(void *dc, callee_info_t *ci);

/* Hashtable for storing callee analysis info. */
static hashtable_t *callee_info_table;
/* We only free callee info at exit, when callee_info_table_exit is true. */
static bool callee_info_table_exit = false;
#define CALLEE_INFO_TABLE_BITS 6

/****************************************************************************/
/* Callee info manipulation functions. */

void
callee_info_init(callee_info_t *ci)
{
    uint i;
    memset(ci, 0, sizeof(*ci));
    ci->bailout = true;
    /* to be conservative */
    ci->has_locals    = true;
    ci->write_aflags  = true;
    ci->read_aflags   = true;
    ci->tls_used      = true;
    ci->stack_complex = true;
    ci->has_cti       = true;
    /* We use loop here and memset in analyze_callee_regs_usage later.
     * We could reverse the logic and use memset to set the value below,
     * but then later in analyze_callee_regs_usage, we have to use the loop.
     */
    /* assuming all xmm registers are used */
    ci->num_xmms_used = NUM_XMM_REGS;
    for (i = 0; i < NUM_XMM_REGS; i++)
        ci->xmm_used[i] = true;
    /* assuming all gp registers are used */
    ci->num_regs_used = NUM_XMM_REGS;
    for (i = 0; i < NUM_GP_REGS; i++)
        ci->reg_used[i] = true;
    for (i = 0; i < NUM_GP_REGS; i++)
        ci->reg_used[i] = true;
    /* The frame should hold at least the mcontext. */
    ci->framesize = sizeof(dr_mcontext_t);
}

static callee_info_t *
callee_info_create(app_pc start, uint num_args)
{
    callee_info_t *ci;

    ci = dr_global_alloc(sizeof(*ci));
    callee_info_init(ci);
    ci->start = start;
    ci->num_args = num_args;
    return ci;
}

static void
callee_info_free(callee_info_t *ci)
{
    ASSERT(callee_info_table_exit);
    if (ci->ilist != NULL) {
        ASSERT(ci->opt_inline);
        instrlist_clear_and_destroy(GLOBAL_DCONTEXT, ci->ilist);
    }
    if (ci->check_ilist != NULL) {
        instrlist_clear_and_destroy(GLOBAL_DCONTEXT, ci->check_ilist);
    }
    if (ci->fast_ilist != NULL) {
        instrlist_clear_and_destroy(GLOBAL_DCONTEXT, ci->fast_ilist);
    }
    dr_global_free(ci, sizeof(*ci));
}

void
callee_info_table_init(void)
{
    callee_info_table = dr_global_alloc(sizeof(*callee_info_table));
    hashtable_init_ex(callee_info_table,
                      CALLEE_INFO_TABLE_BITS,
                      HASH_INTPTR,
                      /* str_dup */ false,
                      /* sync */ false,
                      (void(*)(void*)) callee_info_free,
                      /* hash key */ NULL,
                      /* cmp key */ NULL
                      );
}

void
callee_info_table_destroy(void)
{
    callee_info_table_exit = true;
    hashtable_delete(callee_info_table);
    dr_global_free(callee_info_table, sizeof(*callee_info_table));
}

static callee_info_t *
callee_info_table_lookup(void *callee)
{
    callee_info_t *ci;
    hashtable_lock(callee_info_table);
    ci = hashtable_lookup(callee_info_table, callee);
    hashtable_unlock(callee_info_table);
    /* We only delete the callee_info from the callee_info_table when destroy
     * the table on exit, so we can keep the ci without holding the lock.
     */
    return ci;
}

static callee_info_t *
callee_info_table_add(callee_info_t *new_ci)
{
    callee_info_t *ci;
    hashtable_lock(callee_info_table);
    ci = hashtable_lookup(callee_info_table, new_ci->start);
    if (ci == NULL) {
        ci = new_ci;
        hashtable_add(callee_info_table, new_ci->start, new_ci);
    } else {
        /* Have one in the table, free the new one and use existing one. 
         * We cannot free the existing one in the table as it might be used by 
         * other thread without holding the lock.
         * Since we assume callee should never be changed, they should have
         * the same content of ci.
         */
        callee_info_free(new_ci);
    }
    hashtable_unlock(callee_info_table);
    return ci;
}

callee_info_t *
callee_info_analyze(void *dc, void *callee, uint num_args)
{
    callee_info_t *ci;

    /* Search if callee was analyzed before. */
    ci = callee_info_table_lookup(callee);
    if (ci != NULL)
        return ci;

    /* This callee is not seen before, decode, analyze, and optimize it. */
    STATS_INC(cleancall_analyzed);
    dr_log(dc, LOG_CLEANCALL, 2,
           "drcalls: analyze callee "PFX"\n", callee);
    ci = callee_info_create((app_pc)callee, num_args);
    decode_callee_ilist(dc, ci);
    if (!ci->bailout)
        analyze_callee_ilist(dc, ci);
    return callee_info_table_add(ci);
}

/****************************************************************************/
/* Callee decoding. */

/* Decode instruction from callee and return the next_pc to be decoded. */
static app_pc
decode_callee_instr(void *dcontext, callee_info_t *ci, app_pc instr_pc)
{
    instrlist_t *ilist = ci->ilist;
    instr_t *instr;
    app_pc   next_pc = NULL;

    instr = instr_create(GLOBAL_DCONTEXT);
    instrlist_meta_append(ilist, instr);
    ci->num_instrs++;
    TRY_EXCEPT(dcontext, {
        next_pc = decode(GLOBAL_DCONTEXT, instr_pc, instr);
    }, { /* EXCEPT */
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "drcalls: crash on decoding callee instruction at: "PFX"\n",
               instr_pc);
        ASSERT_CURIOSITY(false && "crashed while decoding clean call");
        ci->bailout = true;
        return NULL;
    });
    if (!instr_valid(instr)) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "drcalls: decoding invalid instruction at: "PFX"\n", instr_pc);
        ci->bailout = true;
        return NULL;
    }
    instr_set_translation(instr, instr_pc);
    DOLOG(3, LOG_CLEANCALL, {
        disassemble_with_info(dcontext, instr_pc, dr_get_logfile(dcontext),
                              true, true);
    });
    return next_pc;
}

/* Uses a heuristic to decide if the target pc is reasonably within the current
 * callee.  If the callee tail calls to a nearby helper, we don't really mind if
 * we end up including the helper in the callee.
 */
static bool
tgt_pc_in_callee(app_pc tgt_pc, callee_info_t *ci)
{
    return ((ptr_uint_t)ci->start <= (ptr_uint_t)tgt_pc &&
            (ptr_uint_t)tgt_pc < (ptr_uint_t)ci->start + 4096);
}

/* Callee decoder.  Starts at the entry pc and remembers farthest forward
 * target pc that doesn't look like a tail call.  Decodes until that pc is
 * encountered and then onwards towards the next tail call, backwards jmp, or
 * ret.
 */
static void
decode_callee_ilist(void *dc, callee_info_t *ci)
{
    app_pc cur_pc;
    app_pc max_tgt_pc = NULL;
    bool decode_next = true;

    dr_log(dc, LOG_CLEANCALL, 2,
           "drcalls: decoding callee starting at: "PFX"\n", ci->start);
    ci->ilist = instrlist_create(GLOBAL_DCONTEXT);
    ci->bailout = false;
    cur_pc = ci->start;

    while (decode_next) {
        instr_t *instr;
        app_pc next_pc;
        app_pc tgt_pc = NULL;

        /* Sanity check whether we want to decode this pc. */
        if (!tgt_pc_in_callee(cur_pc, ci)) {
            dr_log(dc, LOG_CLEANCALL, 2,
                   "drcalls: jump target "PFX" too far, bailing from decode\n",
                   cur_pc);
            ci->bailout = true;
            break;
        }

        next_pc = decode_callee_instr(dc, ci, cur_pc);
        if (next_pc == NULL) {  /* bailout */
            dr_log(dc, LOG_CLEANCALL, 2,
                   "drcalls: decode returned NULL\n");
            break;
        }
        if (ci->num_instrs > MAX_NUM_FUNC_INSTRS) {
            dr_log(dc, LOG_CLEANCALL, 2,
                   "drcalls: too many instructions, bailing from decode\n");
            ci->bailout = true;
            break;
        }

        instr = instrlist_last(ci->ilist);

        /* Remember the maximum branch target if it looks like it's within the
         * callee.  */
        if (instr_is_ubr(instr) || instr_is_cbr(instr)) {
            opnd_t tgt = instr_get_target(instr);
            if (opnd_is_pc(tgt)) {
                tgt_pc = opnd_get_pc(tgt);
                if (tgt_pc_in_callee(tgt_pc, ci)) {
                    max_tgt_pc = (app_pc)MAX((ptr_uint_t)max_tgt_pc,
                                             (ptr_uint_t)tgt_pc);
                }
            }
        }

        /* Stop decoding after a ret. */
        if (instr_is_return(instr)) {
            dr_log(dc, LOG_CLEANCALL, 2,
                   "drcalls: stop decode for ret\n");
            decode_next = false;
        }

        if (instr_is_ubr(instr) && tgt_pc != NULL) {
            /* Stop decoding after backwards jmps. */
            if ((ptr_uint_t)tgt_pc < (ptr_uint_t)cur_pc) {
                dr_log(dc, LOG_CLEANCALL, 2,
                       "drcalls: stop decode for bwds jmp from "PFX" to "PFX"\n",
                       cur_pc, tgt_pc);
                decode_next = false;
            }

            /* Stop decoding after tail calls */
            if (!tgt_pc_in_callee(tgt_pc, ci)) {
                dr_log(dc, LOG_CLEANCALL, 2,
                       "drcalls: stop decode for probable tail call at "PFX
                       " to tgt "PFX"\n", cur_pc, tgt_pc);
                decode_next = false;
            }
        }

        /* Keep decoding if we haven't hit the max jump target. */
        if (!decode_next && (ptr_uint_t)max_tgt_pc > (ptr_uint_t)cur_pc) {
            decode_next = true;
        }

        cur_pc = next_pc;
    }

    if (ci->bailout) {
        instrlist_clear_and_destroy(GLOBAL_DCONTEXT, ci->ilist);
        ci->ilist = NULL;
        return;
    }
}

/****************************************************************************/
/* Callee analysis and optimization. */

/* Create and insert a new label before 'where'.  Sets the app_pc of label to
 * the app_pc of where.
 */
static instr_t *
insert_new_label(void *dc, instrlist_t *ilist, instr_t *where)
{
    instr_t *label = INSTR_CREATE_label(dc);
    instr_set_translation(label, instr_get_app_pc(where));
    PRE(ilist, where, label);
    return label;
}

static void
resolve_internal_brs(void *dcontext, callee_info_t *ci)
{
    instrlist_t *ilist = ci->ilist;
    instr_t *cti, *tgt;
    app_pc   tgt_pc;

    /* Check that no target pc of any branch is in a middle of an instruction,
     * and replace target pc with label before the target instr.
     */
    for (cti = instrlist_first(ilist); cti != NULL; cti = instr_get_next(cti)) {
        instr_t *label;
        /* Ignore indirect branches, syscalls, calls, etc for now.  They may be
         * part of code that we choose not to inline. */
        if (!(instr_is_cbr(cti) || instr_is_ubr(cti)))
            continue;
        tgt_pc = opnd_get_pc(instr_get_target(cti));
        if (!tgt_pc_in_callee(tgt_pc, ci))
            continue;  /* Probably a tail call.  We've logged it previously. */
        for (tgt  = instrlist_first(ilist);
             tgt != NULL;
             tgt  = instr_get_next(tgt)) {
            if (tgt_pc == instr_get_app_pc(tgt))
                break;
        }
        if (tgt == NULL) {
            /* cannot find a target instruction, bail out */
            dr_log(dcontext, LOG_CLEANCALL, 2,
                   "drcalls: bail out on strange internal branch at: "PFX
                   " to "PFX"\n", instr_get_app_pc(cti), tgt_pc);
            ci->bailout = true;
            break;
        }
        /* TODO(rnk): This creates unique label for every jump target in some
         * unspecified order.  Is that bad?
         */
        label = insert_new_label(GLOBAL_DCONTEXT, ilist, tgt);
        instr_set_target(cti, opnd_create_instr(label));
    }
}

/* Check if there is any control flow we couldn't follow.
 */
static void
analyze_callee_cti(void *dc, callee_info_t *ci)
{
    bool is_leaf = true;
    bool has_cti = false;
    instr_t *instr;
    for (instr = instrlist_first(ci->ilist); instr != NULL;
         instr = instr_get_next(instr)) {
        if (instr_is_syscall(instr) || instr_is_interrupt(instr)) {
            dr_log(dc, LOG_CLEANCALL, 2,
                   "drcalls: not leaf, syscall or interrupt at "PFX"\n",
                   instr_get_app_pc(instr));
            is_leaf = false;
            has_cti = true;  /* syscalls etc. count as control flow. */
        }
        if (!instr_is_cti(instr))
            continue;
        /* XXX: We assume callees do not push arbitrary return addresses and ret
         * to them.  */
        if (instr_is_return(instr))
            continue;
        has_cti = true;
        if (instr_is_call(instr) || instr_is_mbr(instr)) {
            dr_log(dc, LOG_CLEANCALL, 2,
                   "drcalls: not leaf, call or indirect jump at "PFX"\n",
                   instr_get_app_pc(instr));
            is_leaf = false;
        }
        if ((instr_is_cbr(instr) || instr_is_ubr(instr)) &&
            opnd_is_pc(instr_get_target(instr))) {
            dr_log(dc, LOG_CLEANCALL, 2,
                   "drcalls: not leaf, unresolved jump at "PFX"\n",
                   instr_get_app_pc(instr));
            is_leaf = false;
        }
    }
    ci->is_leaf = is_leaf;
    ci->has_cti = has_cti;
}

static bool
instr_is_mov_ld_xsp(instr_t *instr)
{
    return (instr_get_opcode(instr) == OP_mov_ld &&
            opnd_same(instr_get_src(instr, 0),
                      OPND_CREATE_MEMPTR(DR_REG_XSP, 0)));
}

/* Rewrite PIC code to materialize application PC directly.  Two major PIC
 * styles:
 * 1. call next_pc; pop r1;
 * 2. call pic_func;
 *    and in pic_func: mov [%xsp] %r1; ret;
 */
static void
rewrite_pic_code(void *dcontext, callee_info_t *ci)
{
    instr_t *call_instr;
    instr_t *next_instr;
    instrlist_t *ilist = ci->ilist;

    for (call_instr  = instrlist_first(ilist);
         call_instr != NULL;
         call_instr  = next_instr) {
        app_pc cur_pc, next_pc;
        instr_t *new_instr;
        bool is_pic = false;
        opnd_t pic_reg;
        opnd_t tgt;
        app_pc tgt_pc = NULL;

        next_instr = instr_get_next(call_instr);
        if (!instr_is_call(call_instr))
            continue;
        if (next_instr == NULL) {
            ASSERT_CURIOSITY(false && "call is last instruction in ilist?");
            break;
        }

        cur_pc = instr_get_app_pc(call_instr);
        tgt = instr_get_target(call_instr);
        if (opnd_is_pc(tgt))
            tgt_pc = opnd_get_pc(tgt);
        /* Will be NULL if we inserted next_instr. */
        next_pc = instr_get_app_pc(next_instr);

        if (tgt_pc != NULL && tgt_pc == next_pc) {
            /* "pop %r1" or "mov [%rsp] %r1" */
            if (instr_get_opcode(next_instr) == OP_pop ||
                instr_is_mov_ld_xsp(next_instr)) {
                is_pic = true;
                pic_reg = instr_get_dst(next_instr, 0);
                /* Delete mov or pop. */
                instrlist_remove(ilist, next_instr);
                instr_destroy(GLOBAL_DCONTEXT, next_instr);
            }
        } else if (tgt_pc != NULL) {
            /* a callout */
            instr_t mov_ins, ret_ins;
            app_pc tmp_pc;
            instr_init(dcontext, &mov_ins);
            instr_init(dcontext, &ret_ins);
            TRY_EXCEPT(dcontext, {
                tmp_pc = decode(dcontext, tgt_pc, &mov_ins);
                tmp_pc = decode(dcontext, tmp_pc, &ret_ins);
            }, {
                ASSERT_CURIOSITY(false && "crashed while decoding clean call");
                /* Frees any instr internal memory and reinitializes.  Following
                 * checks will fail. */
                instr_reset(dcontext, &mov_ins);
                instr_reset(dcontext, &ret_ins);
            });
            if (instr_is_mov_ld_xsp(&mov_ins) &&
                instr_is_return(&ret_ins)) {
                pic_reg = instr_get_dst(&mov_ins, 0);
                is_pic = true;
            }
            instr_free(dcontext, &mov_ins);
            instr_free(dcontext, &ret_ins);
        }

        if (!is_pic)
            continue;

        ASSERT(opnd_is_reg(pic_reg));
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "drcalls: special PIC code at: "PFX"\n", cur_pc);

        /* Insert "mov next_pc r1". */
        /* XXX: The memory on top of stack will not be next_pc, and the stack
         * may become unaligned. */
        new_instr = INSTR_CREATE_mov_imm
            (GLOBAL_DCONTEXT, pic_reg, OPND_CREATE_INTPTR(next_pc));
        instr_set_translation(new_instr, cur_pc);
        instrlist_preinsert(ilist, call_instr, new_instr);

        /* Remove call. */
        instrlist_remove(ilist, call_instr);
        instr_destroy(GLOBAL_DCONTEXT, call_instr);
    }
}

/* Remove and delete an instruction from an ilist.  This helper routine exists
 * to avoid frequent typos where different instructions are passed to the remove
 * and destory routines.
 */
void
remove_and_destroy(void *dc, instrlist_t *ilist, instr_t *instr)
{
    instrlist_remove(ilist, instr);
    instr_destroy(dc, instr);
}

static void
analyze_callee_regs_usage(void *dcontext, callee_info_t *ci)
{
    instrlist_t *ilist = ci->ilist;
    instr_t *instr;
    uint i;

    ci->num_xmms_used = 0;
    memset(ci->xmm_used, 0, sizeof(bool) * NUM_XMM_REGS);
    ci->num_regs_used = 0;
    memset(ci->reg_used, 0, sizeof(bool) * NUM_GP_REGS);
    ci->write_aflags = false;
    for (instr  = instrlist_first(ilist);
         instr != NULL;
         instr  = instr_get_next(instr)) {
        /* XXX: this is not efficient as instr_uses_reg will iterate over
         * every operands, and the total would be (NUM_REGS * NUM_OPNDS)
         * for each instruction. However, since this will be only called 
         * once for each clean call callee, it will have little performance
         * impact unless there are a lot of different clean call callees.
         */
        /* XMM registers usage */
        for (i = 0; i < NUM_XMM_REGS; i++) {
            if (!ci->xmm_used[i] &&
                instr_uses_reg(instr, (DR_REG_XMM0 + (reg_id_t)i))) {
                dr_log(dcontext, LOG_CLEANCALL, 2,
                       "drcalls: callee "PFX" uses XMM%d at "PFX"\n", 
                       ci->start, i, instr_get_app_pc(instr));
                ci->xmm_used[i] = true;
                ci->num_xmms_used++;
            }
        }
        /* General purpose registers */
        for (i = 0; i < NUM_GP_REGS; i++) {
            if (!ci->reg_used[i] &&
                instr_uses_reg(instr, (DR_REG_XAX + (reg_id_t)i))) {
                dr_log(dcontext, LOG_CLEANCALL, 2,
                       "drcalls: callee "PFX" uses REG %s at "PFX"\n", 
                       ci->start, get_register_name(DR_REG_XAX + (reg_id_t)i),
                    instr_get_app_pc(instr));
                ci->reg_used[i] = true;
                ci->num_regs_used++;
            }
        }
        /* callee update aflags */
        if (!ci->write_aflags) {
            if (TESTANY(EFLAGS_WRITE_6, instr_get_arith_flags(instr))) {
                dr_log(dcontext, LOG_CLEANCALL, 2,
                       "drcalls: callee "PFX" updates aflags\n", ci->start);
                ci->write_aflags = true;
            }
        }
    }
    /* we treat %xsp separately. */
    if (ci->reg_used[DR_REG_XSP - DR_REG_XAX]) {
        ci->reg_used[DR_REG_XSP - DR_REG_XAX] = false;
        ci->num_regs_used--;
    }

    /* check if callee read aflags from caller */
    /* set it false for the case of empty callee. */
    ci->read_aflags = false;
    for (instr  = instrlist_first(ilist);
         instr != NULL;
         instr  = instr_get_next(instr)) {
        uint flags = instr_get_arith_flags(instr);
        if (TESTANY(EFLAGS_READ_6, flags)) {
            ci->read_aflags = true;
            break;
        }
        if (TESTALL(EFLAGS_WRITE_6, flags))
            break;
        if (instr_is_return(instr))
            break;
        /* TODO(rnk): Is this necessary?  Catches jrcxz. */
        if (instr_is_cti(instr)) {
            ci->read_aflags = true;
            break;
        }
    }
    if (ci->read_aflags) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "drcalls: callee "PFX" reads aflags from caller\n", ci->start);
    }
}

/* Find all control flow instructions that leave the function.
 */
#define MAX_EXIT_POINTS 10
static int
analyze_find_exit_instrs(void *dc, callee_info_t *ci, instr_t **bots)
{
    int num_bots = 0;
    instr_t *instr;

    for (instr = instrlist_first(ci->ilist); instr != NULL;
         instr = instr_get_next(instr)) {
        instr_t *exit_instr = NULL;
        if (instr_is_return(instr)) {
            exit_instr = instr;
        }
        if (instr_is_ubr(instr)) {
            opnd_t tgt = instr_get_target(instr);
            if (opnd_is_pc(tgt) && !tgt_pc_in_callee(opnd_get_pc(tgt), ci)) {
                exit_instr = instr;
            }
        }
        if (exit_instr != NULL) {
            if (num_bots >= MAX_EXIT_POINTS) {
                return num_bots;
            }
            bots[num_bots++] = exit_instr;
        }
    }

    return num_bots;
}

/* Returns true if the instruction is a control flow instruction or jump target.
 */
static bool
is_cti_or_label_instr(instr_t *instr)
{
    return (instr_is_cti(instr) || instr_is_label(instr));
}

/* Returns false if the instruction is a control flow instruction, a stack
 * instruction, or a jump target.  Otherwise returns true.
 */
static bool
is_boring_instr(instr_t *instr)
{
    if (instr_uses_reg(instr, DR_REG_XBP) || instr_uses_reg(instr, DR_REG_XSP))
        return false;
    return !is_cti_or_label_instr(instr);
}

/* Advance forward in the ilist until we find an instruction that modifies the
 * stack, does control flow, or is a label.
 */
static instr_t *
skip_boring_instrs_fwd(instr_t *top)
{
    for (; top != NULL; top = instr_get_next(top))
        if (!is_boring_instr(top))
            break;
    return top;
}

/* Advance backwards through the ilist until we find an instruction that
 * modifies the stack, does control flow, or is a label.
 */
static void
skip_boring_instrs_bwd(instr_t **bots, int num)
{
    int i;
    for (i = 0; i < num; i++) {
        instr_t *bot = bots[i];
        for (; bot != NULL; bot = instr_get_prev(bot))
            if (!is_boring_instr(bot))
                break;
        bots[i] = bot;
    }
}

typedef bool (*pred_fn_t)(instr_t *item, void *data);

/* Takes a predicate function pointer, a data value to pass to all predicate
 * calls, and an array of instr_t pointers and number of elements.  If the
 * predicate returns true for all instrs, return true, otherwise false.
 */
static bool
all_satisfy(pred_fn_t pred, void *data, instr_t **instrs, int num)
{
    int i;
    for (i = 0; i < num; i++) {
        if (!pred(instrs[i], data)) {
            return false;
        }
    }
    return true;
}

/* Similar to all_satisfy, except return true if any predicates succeed.
 */
static bool
any_satisfy(pred_fn_t pred, void *data, instr_t **instrs, int num)
{
    int i;
    for (i = 0; i < num; i++) {
        if (pred(instrs[i], data)) {
            return true;
        }
    }
    return false;
}

static bool
pred_not_null(instr_t *instr, void *data)
{
    return instr != NULL;
}

/* TODO(rnk): Only one use, maybe should re-inline it. */
static instr_t *
remove_top_bot_pairs(instrlist_t *ilist, instr_t *top, instr_t **bots,
                     int num_bots)
{
    instr_t *instr;
    int i;

    instr = instr_get_next(top);
    remove_and_destroy(GLOBAL_DCONTEXT, ilist, top);
    top = instr;
    for (i = 0; i < num_bots; i++) {
        instr = instr_get_prev(bots[i]);
        remove_and_destroy(GLOBAL_DCONTEXT, ilist, bots[i]);
        bots[i] = instr;
    }
    return top;
}

/* Checks if instruction is equivalent to "mov %src => %dst".  This is needed
 * because mov can be encoded with mov_ld or mov_st opcodes. */
static bool
is_mov_reg2reg(instr_t *instr, reg_id_t dst, reg_id_t src)
{
    int opc = instr_get_opcode(instr);
    return ((opc == OP_mov_ld || opc == OP_mov_st) &&
            opnd_same(instr_get_src(instr, 0), opnd_create_reg(src)) &&
            opnd_same(instr_get_dst(instr, 0), opnd_create_reg(dst)));
}

/* Remove two instructions from the same ilist so long as they aren't the same
 * instruction. */
static void
remove_both_instrs(void *dc, instrlist_t *ilist,
                   instr_t *instr1, instr_t *instr2)
{
    ASSERT(instr1 != NULL && instr2 != NULL);
    if (instr1 == instr2) {
        instr2 = NULL;
    }
    remove_and_destroy(dc, ilist, instr1);
    if (instr2 != NULL) {
        remove_and_destroy(dc, ilist, instr2);
    }
}

/* Find enter/leave pair.  Frame setup may maintain xbp as frame ptr or not.  If
 * maintaining frame ptr, there are two prologue and epilogue sequences.
 * Prologue:
 * enter $framesize, $0
 * push xbp ; ... ; mov xsp => xbp
 * Epilogue:
 * leave
 * mov xbp => xsp ; ... ; pop xbp
 *
 * We should match any combination of the above epilogues and prologues.
 * push+mov+leave seems most common.
 *
 * If omitting frame pointer, xbp is just callee-saved and xsp is adjusted
 * directly:
 * push xbp ; sub $framesize xsp ; ... ; add $framesize xbp ; pop xbp
 *
 * Returns new top after removing xbp save if found.
 *
 * Sets ci->frame_size if 'enter' is used to set up frame.
 */
static instr_t *
analyze_enter_leave(void *dc, callee_info_t *ci, instr_t *top,
                    instr_t **bots, int num_bots)
{
    opnd_t xbp, xsp;
    instr_t *push_xbp;    /* Push of xbp.  Can be push or enter. */
    instr_t *save_xsp;    /* Copy of xsp to xbp.  Can be mov or enter. */
    instr_t *restore_xsp; /* Restore of xsp from xbp.  Can be mov or leave. */
    instr_t *pop_xbp;     /* Pop of xbp.  Can be pop or leave. */
    /* Callees are single-entry, but may have multiple exits using different
     * prologues.  However, they should all undo the operations of the prologue.
     * These arrays track individual epilogue instructions for each exit point
     * of the callee.
     */
    instr_t *restore_xsps[MAX_EXIT_POINTS];
    instr_t *pop_xbps[MAX_EXIT_POINTS];
    int i;
    /* Reference instructions for comparison. */
    instr_t *cmp_push_xbp;
    instr_t *cmp_pop_xbp;

    /* for easy of comparison, create push xbp, pop xbp */
    xbp = opnd_create_reg(DR_REG_XBP);
    xsp = opnd_create_reg(DR_REG_XSP);
    cmp_push_xbp    = INSTR_CREATE_push(dc, xbp);
    cmp_pop_xbp     = INSTR_CREATE_pop (dc, xbp);

    /* Determine prologue style: do we push xbp, do we save xsp. */
    push_xbp = NULL;
    save_xsp = NULL;
    if (instr_get_opcode(top) == OP_enter) {
        /* Enter pushes xbp, copies xsp to xbp, and subtracts from xsp. */
        push_xbp = top;
        save_xsp = top;
        ci->frame_size = opnd_get_immed_int(instr_get_src(top, 0));
        ASSERT_CURIOSITY(opnd_get_immed_int(instr_get_src(top, 1)) == 0 &&
                         "Callee uses 'enter' with non-zero second argument?");
    } else if (instr_same(cmp_push_xbp, top)) {
        instr_t *maybe_save = skip_boring_instrs_fwd(instr_get_next(top));
        push_xbp = top;
        /* Instr is "mov xsp => xbp". */
        if (is_mov_reg2reg(maybe_save, DR_REG_XBP, DR_REG_XSP)) {
            save_xsp = maybe_save;
        }
    }
    if (push_xbp == NULL) {
        dr_log(dc, LOG_CLEANCALL, 2,
               "drcalls: callee does not save xbp before using stack at "PFX"\n",
               instr_get_app_pc(top));
    }

    /* Determine epilogue styles. */
    for (i = 0; i < num_bots; i++) {
        instr_t *bot = bots[i];
        pop_xbp = NULL;
        restore_xsp = NULL;
        if (bot == NULL) {
            /* Do nothing, callee does not touch xbp. */
        } else if (instr_get_opcode(bot) == OP_leave) {
            /* Leave does both. */
            pop_xbp = bot;
            restore_xsp = bot;
        } else if (instr_same(bot, cmp_pop_xbp)) {
            instr_t *maybe_restore = instr_get_prev(bot);
            pop_xbp = bot;
            skip_boring_instrs_bwd(&maybe_restore, 1);
            /* Instr is "mov xbp => xsp". */
            if (is_mov_reg2reg(maybe_restore, DR_REG_XSP, DR_REG_XBP)) {
                restore_xsp = maybe_restore;
            }
        }
        pop_xbps[i] = pop_xbp;
        restore_xsps[i] = restore_xsp;
    }

    dr_log(dc, LOG_CLEANCALL, 2,
           "drcalls: prologue pushes %%xbp at "PFX", sets fp at "PFX"\n",
           push_xbp == NULL ? NULL : instr_get_app_pc(push_xbp),
           save_xsp == NULL ? NULL : instr_get_app_pc(save_xsp));
    for (i = 0; i < num_bots; i++) {
        dr_log(dc, LOG_CLEANCALL, 2,
               "drcalls: epilogue sets sp to fp at "PFX" pops %%xbp at "PFX"\n",
               restore_xsps[i] == NULL ? NULL : instr_get_app_pc(restore_xsps[i]),
               pop_xbps    [i] == NULL ? NULL : instr_get_app_pc(pop_xbps    [i]));
    }

    /* If prologue and epilogue style match, remove prologue and epilogue
     * instructions. */
    if (push_xbp != NULL && save_xsp != NULL &&
        all_satisfy(pred_not_null, NULL, pop_xbps, num_bots) &&
        all_satisfy(pred_not_null, NULL, restore_xsps, num_bots)) {
        /* Using xbp as frame pointer, delete both instrs if not same. */
        top = instr_get_next(save_xsp);
        ci->xbp_is_fp = true;
        remove_both_instrs(GLOBAL_DCONTEXT, ci->ilist, push_xbp, save_xsp);
        for (i = 0; i < num_bots; i++) {
            bots[i] = instr_get_prev(restore_xsps[i]); /* Earliest instr. */
            remove_both_instrs(GLOBAL_DCONTEXT, ci->ilist,
                               pop_xbps[i], restore_xsps[i]);
        }
    }
    else if (push_xbp != NULL && save_xsp == NULL &&
             all_satisfy(pred_not_null, NULL, pop_xbps, num_bots) &&
             !any_satisfy(pred_not_null, NULL, restore_xsps, num_bots)) {
        /* Not using xbp as frame pointer, delete push/pop instrs only. */
        top = instr_get_next(push_xbp);
        remove_and_destroy(GLOBAL_DCONTEXT, ci->ilist, push_xbp);
        for (i = 0; i < num_bots; i++) {
            /* New bottom is one earlier than pop. */
            bots[i] = instr_get_prev(pop_xbps[i]);
            remove_and_destroy(GLOBAL_DCONTEXT, ci->ilist, pop_xbps[i]);
        }
    }
    else {
        dr_log(dc, LOG_CLEANCALL, 2,
               "drcalls: Unable to match prologue to epilogue.\n");
        push_xbp = NULL;
        save_xsp = NULL;
    }

    if (push_xbp != NULL) {
        /* Function preserves xbp. */
        ci->callee_save_regs[DR_REG_XBP - DR_REG_XAX] = true;
    }

    /* Destroy instructions used for comparison. */
    instr_destroy(dc, cmp_push_xbp);
    instr_destroy(dc, cmp_pop_xbp);

    return top;
}

/* We use push/pop pattern to detect callee saved registers, and assume that the
 * code later won't change those saved value on the stack.
 */
static void
analyze_callee_setup(void *dcontext, callee_info_t *ci)
{
    instrlist_t *ilist = ci->ilist;
    instr_t *top;
    instr_t *bots[MAX_EXIT_POINTS];
    int num_bots, i;

    ASSERT(ilist != NULL);
    /* Function prologues and epilogues are frequently rescheduled so that
     * register-to-register instructions are intermixed with both.  Our analysis
     * ignores instructions in the prologue and epilogue that don't touch the
     * stack and ignore them.
     *
     * Furthermore, while we assume a single entry point to the function,
     * multiple exit points are common, and they may be tail calls.  To handle
     * this, we maintain a short list of exit points and perform our matching on
     * each.
     */
    top = instrlist_first(ilist);
    if (top == NULL || instr_get_next(top) == NULL) {
        /* zero or one instruction only, no callee save */
        return;
    }
    num_bots = analyze_find_exit_instrs(dcontext, ci, bots);
    if (num_bots >= MAX_EXIT_POINTS) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "drcalls: bailing from setup analysis, too many exits\n");
        return;
    }

    top = skip_boring_instrs_fwd(top);
    skip_boring_instrs_bwd(bots, num_bots);

    /* Skip rets and jmps.  Other analyses need to see them. */
    for (i = 0; i < num_bots; i++) {
        if (instr_is_return(bots[i]) || instr_is_ubr(bots[i])) {
            bots[i] = instr_get_prev(bots[i]);
        }
    }
    skip_boring_instrs_bwd(bots, num_bots);

    /* Find and delete enter/leave or equivalent instructions.  We assume these
     * come before register saves and restores. */
    top = analyze_enter_leave(dcontext, ci, top, bots, num_bots);

    /* get the rest callee save regs */
    /* XXX: the callee save may be corrupted by memory update on the stack. */
    /* XXX: the callee save may use mov instead of push/pop */
    while (top != NULL && all_satisfy(pred_not_null, NULL, bots, num_bots)) {
        opnd_t reg_opnd;
        reg_id_t reg_id;
        /* Ignore nops and boring instructions. */
        top = skip_boring_instrs_fwd(top);
        skip_boring_instrs_bwd(bots, num_bots);
        /* If not in the first/last bb, break. */
        if (top == NULL || is_cti_or_label_instr(top))
            break;
        if (!all_satisfy(pred_not_null, NULL, bots, num_bots) ||
            any_satisfy((pred_fn_t)is_cti_or_label_instr, NULL, bots, num_bots))
            break;

        /* If we're doing a push on entry, all pops must match. */
        if (instr_get_opcode(top) == OP_push) {
            instr_t *pop_reg;
            reg_opnd = instr_get_src(top, 0);
            reg_id = opnd_get_reg(reg_opnd);
            pop_reg = INSTR_CREATE_pop(dcontext, reg_opnd);
            if (!all_satisfy((pred_fn_t)instr_same, pop_reg, bots, num_bots)) {
                instr_destroy(dcontext, pop_reg);
                break;
            }
            instr_destroy(dcontext, pop_reg);
        } else {
            break;  /* top was not a register save. */
        }

        /* TODO(rnk): Check that register was not clobbered before push. */

        /* it is callee save reg. */
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "drcalls: callee "PFX" callee-saves reg %s at "PFX"\n",
               ci->start, get_register_name(reg_id), instr_get_app_pc(top));
        ci->callee_save_regs[reg_id - DR_REG_XAX] = true;

        /* Remove & destroy the push/pop pairs, we do our own save/restore if
         * inlined. */
        top = remove_top_bot_pairs(ilist, top, bots, num_bots);
    }
}

static void
analyze_callee_tls(void *dcontext, callee_info_t *ci)
{
    /* access to TLS means we do need to swap/preserve TEB/PEB fields
     * for library isolation (errno, etc.)
     */
    instr_t *instr;
    int i;
    ci->tls_used = false;
    for (instr = instrlist_first(ci->ilist); instr != NULL;
         instr = instr_get_next(instr)) {
        /* we assume any access via app's tls is to app errno. */
        for (i = 0; i < instr_num_srcs(instr); i++) {
            opnd_t opnd = instr_get_src(instr, i);
            if (opnd_is_far_base_disp(opnd))
                ci->tls_used = true;
        }
        for (i = 0; i < instr_num_dsts(instr); i++) {
            opnd_t opnd = instr_get_dst(instr, i);
            if (opnd_is_far_base_disp(opnd))
                ci->tls_used = true;
        }
    }
    if (ci->tls_used) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "drcalls: callee "PFX" accesses far memory\n", ci->start);
    }
}

static bool
is_fastpath(instr_t *instr)
{
    int length = 0;
    for (; instr != NULL; instr = instr_get_next(instr)) {
        if (instr_is_return(instr)) {
            return true;
        }
        if (instr_is_cti(instr)) {
            return false;
        }
        if (length > 5) {
            return false;
        }
        length++;
    }
    return false;
}

/* Update clobber set with register writes by instr. */
static void
clobbered_update(bool clobbered[NUM_GP_REGS], instr_t *instr)
{
    int i;
    for (i = 0; i < instr_num_dsts(instr); i++) {
        opnd_t opnd = instr_get_dst(instr, i);
        if (opnd_is_reg(opnd)) {
            reg_id_t reg = reg_to_pointer_sized(opnd_get_reg(opnd));
            clobbered[reg - DR_REG_XAX] = true;
        }
    }
}

static bool
clobbered_reads(bool clobbered[NUM_GP_REGS], instr_t *instr)
{
    int i;
    for (i = 0; i < NUM_GP_REGS; i++) {
        reg_id_t reg = DR_REG_XAX + (reg_id_t)i;
        if (clobbered[i] && instr_reads_from_reg(instr, reg))
            return true;
    }
    return false;
}

/* Defers non-stack writes into the fastpath, allowing the partial inline check
 * to occur before any stores.
 *
 * We need to be careful that we don't move any reads across writes to the same
 * location.  For memory, we stop at the first memory read.  For registers, we
 * maintain two sets: a liveness set for the check, and a clobber set for things
 * that were left in the entry block.
 *
 * The live set starts at the jcc with everything dead, since we remat args and
 * start again from scratch, and is updated with every instruction left in the
 * entry block.
 *
 * The clobber set is updated with register destinations left in the entry
 * block, since any instruction that reads a clobbered reg cannot be deferred.
 *
 * An instruction can be deferred if it is dead it reads no clobbered registers.
 */
/* TODO(rnk): Are there any side-effects aside from memory writes? */
static bool
defer_side_effects(void *dc, callee_info_t *ci, instr_t *jcc, instr_t *fastpath,
                   bool check_only)
{
    instr_t *instr;
    instr_t *prev_instr;
    liveness_t liveness_;
    liveness_t *liveness = &liveness_;
    /* For now, we don't track clobbering flags, and cannot defer any flags
     * reading or writing instructions. */
    bool clobbered[NUM_GP_REGS];
    bool crossed_read = false;

    if (!check_only) {
        dr_log(dc, LOG_CLEANCALL, 3,
               "drcalls: defering memory writes for real\n");
    }

    liveness_init(liveness, ci);
    memset(clobbered, 0, sizeof(clobbered));

    for (instr = jcc; instr != NULL; instr = prev_instr) {
        bool is_live;
        bool can_defer = !crossed_read;

        prev_instr = instr_get_prev(instr);
        if (!crossed_read && instr_reads_memory(instr)) {
            if (check_only) {
                dr_log(dc, LOG_CLEANCALL, 3,
                       "drcalls: stopped defer, found mem read at "PFX"\n",
                       instr_get_app_pc(instr));
            }
            crossed_read = true;
        }

        /* In this case, we don't consider memory writes "live" since we won't
         * cross any dependent memory reads.
         */
        is_live = (liveness_instr_is_live(liveness, instr) ||
                   instr_is_cti(instr) || instr_is_label(instr));
        if (is_live) {
            dr_log(dc, LOG_CLEANCALL, 3,
                   "drcalls: can't defer instr, is live for jcc at "PFX"\n",
                   instr_get_app_pc(instr));
            can_defer = false;
        }

        /* Check if this instr uses clobbered regs. */
        /* TODO(rnk): DTRT wrt flags. */
        if (clobbered_reads(clobbered, instr)) {
            dr_log(dc, LOG_CLEANCALL, 3,
                   "drcalls: can't defer instr, reads clobbered reg at "PFX"\n",
                   instr_get_app_pc(instr));
            can_defer = false;
        }

        if (can_defer) {
            dr_log(dc, LOG_CLEANCALL, 3, "drcalls: deferred instr at "PFX"\n",
                   instr_get_app_pc(instr));
            instrlist_remove(ci->ilist, instr);
            POST(ci->ilist, fastpath, instr);
        } else {
            if (instr_writes_memory(instr)) {
                dr_log(dc, LOG_CLEANCALL, 3,
                       "drcalls: found un-deferable memory write at "PFX"\n",
                       instr_get_app_pc(instr));
                return false;
            }
            clobbered_update(clobbered, instr);
            liveness_update(liveness, instr);
        }
    }

    return true;
}

/* Return true if instr is Jcc instruction.  These are known to be easy to
 * logically invert.
 */
static bool
instr_is_jcc(instr_t *instr)
{
    uint opc = instr_get_opcode(instr);
    /* Relies on opcode ordering. */
    return ((opc >= OP_jo && opc <= OP_jnle) ||
            (opc >= OP_jo_short && opc <= OP_jnle_short));
}

/* Turns callees with a fast path into an ilist that can be inlined.  Relies on
 * control flow being resolved, but rets still being present.
 */
static void
analyze_callee_partial(void *dc, callee_info_t *ci)
{
    instr_t *first_cti;
    instr_t *tgt_instr;
    instr_t *fallthrough_instr;
    instr_t *fastpath_start;
    instr_t *slowpath_start;
    instr_t *slowpath_end;
    instr_t *instr;
    instr_t *next_instr;
    opnd_t tgt;
    bool taken_fast, fallthrough_fast;

    if (opt_cleancall < 3) {
        dr_log(dc, LOG_CLEANCALL, 3,
               "drcalls: partial inlining disabled: opt_cleancall: %d.\n",
               opt_cleancall);
        return;
    }

    /* Find first branch target or control flow instruction. */
    for (first_cti  = instrlist_first(ci->ilist);
         first_cti != NULL;
         first_cti  = instr_get_next(first_cti)) {
        if (instr_is_cti(first_cti)) {
            break;
        }
    }
    if (first_cti == NULL || instr_is_return(first_cti)) {
        return;  /* No control flow, can't partial inline. */
    }
    if (!instr_is_cbr(first_cti)) {
        dr_log(dc, LOG_CLEANCALL, 2,
               "drcalls: cannot partially inline calllee "PFX": "
               "first cti is not cbr at "PFX".\n",
               ci->start, instr_get_app_pc(first_cti));
        return;
    }

    /* Find target. */
    tgt = instr_get_target(first_cti);
    if (!opnd_is_instr(tgt)) {
        dr_log(dc, LOG_CLEANCALL, 2,
               "drcalls: cannot partially inline calllee "PFX": "
               "control flow to non-label at "PFX".\n",
               ci->start, instr_get_app_pc(first_cti));
        return;
    }
    tgt_instr = opnd_get_instr(tgt);
    fallthrough_instr = instr_get_next(first_cti);

    /* Find fastpath, taken or fallthrough. */
    taken_fast = is_fastpath(tgt_instr);
    fallthrough_fast = is_fastpath(fallthrough_instr);
    if (taken_fast && fallthrough_fast) {
        dr_log(dc, LOG_CLEANCALL, 2,
               "drcalls: both paths at "PFX" and "PFX" are fast.\n",
               instr_get_app_pc(fallthrough_instr), instr_get_app_pc(tgt_instr));
        return;
    } else if (taken_fast) {
        fastpath_start = tgt_instr;
        slowpath_start = fallthrough_instr;
    } else if (fallthrough_fast) {
        fastpath_start = fallthrough_instr;
        slowpath_start = tgt_instr;
    } else {
        dr_log(dc, LOG_CLEANCALL, 2,
               "drcalls: cannot partially inline calllee "PFX": "
               "neither path is fast for cbr at "PFX".\n",
               ci->start, instr_get_app_pc(first_cti));
        return;
    }

    /* Make sure slowpath_start is a label.  Should only happen if fallthrough
     * is slow.  We set a note on the label so different call sites can find it
     * to handle call site specific saves, restores, or arg setup. */
    if (!fallthrough_fast && !instr_is_label(slowpath_start)) {
        slowpath_start =
            insert_new_label(GLOBAL_DCONTEXT, ci->ilist, slowpath_start);
    }
    DR_ASSERT(instr_is_label(slowpath_start));

    /* Remove slowpath instructions. */
    for (instr = instr_get_next(slowpath_start); instr != NULL; instr = next_instr) {
        /* Skip fastpath code if we encounter it. */
        if (instr == fastpath_start) {
            while (!instr_is_return(instr)) {
                instr = instr_get_next(instr);
            }
            instr = instr_get_next(instr);
        }
        if (instr == NULL)
            break;
        next_instr = instr_get_next(instr);
        remove_and_destroy(GLOBAL_DCONTEXT, ci->ilist, instr);
    }

    /* If the target was the slowpath, that means 'ret' is not the last
     * instruction.  Try to invert the condition to put the slowpath on the
     * fallthrough path:
     *
     * ... entry
     * jle tgt
     * ... fastpath_start
     * ret
     * tgt:
     * ... slowpath_start
     *
     * To:
     *
     * ... entry
     * jg tgt
     * ... slowpath_start
     * tgt:
     * ... fastpath_start
     * ret
     */
    if (fallthrough_fast && instr_is_jcc(first_cti)) {
        if (!instr_is_label(fastpath_start)) {
            fastpath_start =
                insert_new_label(GLOBAL_DCONTEXT, ci->ilist, fastpath_start);
        }
        /* Move the slowpath to be right after the jcc. */
        instrlist_remove(ci->ilist, slowpath_start);
        POST(ci->ilist, first_cti, slowpath_start);
        /* Invert the jcc and set the target to the fast path. */
        instr_invert_cbr(first_cti);
        instr_set_target(first_cti, opnd_create_instr(fastpath_start));
        dr_log(dc, LOG_CLEANCALL, 3,
               "drcalls: inverted jcc for partial inlining\n");
    }

    /* Defer all side effects until after the check. */
    if (defer_side_effects(dc, ci, first_cti, fastpath_start, /*check=*/true)) {
        defer_side_effects(dc, ci, first_cti, fastpath_start, /*check=*/false);
    } else {
        dr_log(dc, LOG_CLEANCALL, 3, "drcalls: failed partial inline: "
               "unable to defer entry side effects to fast path\n");
        return;
    }

    dr_log(dc, LOG_CLEANCALL, 2,
           "drcalls: callee is a candidate for partial inlining.\n");
    ci->opt_partial = true;

    /* Circular dependency issue: we can't emit the slowpath for the partially
     * inlined code until we've done further analysis of register and stack
     * usage.  We leave this call to the end label and fix it up later.
     *
     * Code should look like:
     * jcc fastpath
     *   call outofline
     *   jmp done
     * fastpath:
     *   ...
     * done:
     * ret
     */
    /* Put done label before ret. */

    /* Insert call. */
    slowpath_end = instr_get_next(slowpath_start);
    ci->partial_label = INSTR_CREATE_label(GLOBAL_DCONTEXT);
    PRE(ci->ilist, slowpath_end, INSTR_CREATE_call
        (GLOBAL_DCONTEXT, opnd_create_instr(ci->partial_label)));
    /* We insert partial_label to avoid memory leaks on bailout. */
    PRE(ci->ilist, slowpath_end, ci->partial_label);

    /* XXX: Mark the call instr *non-meta* to indicate to optimizations that no
     * registers are live-in.  This is the only reason we can pretend there is
     * no control flow even in partial inlining, because in the slowpath there
     * is no register use of consequence.
     */
    instr_set_ok_to_mangle(instr_get_next(slowpath_start), true);

    /* We need a jmp if slowpath_end is not right before ret. */
    for (instr = slowpath_end; instr != NULL; instr = instr_get_next(instr))
        if (!instr_is_label(instr)) break;
    if (instr != NULL && !instr_is_return(instr)) {
        instr_t *done_label;
        /* Find the return instruction from the end. */
        for (instr = instrlist_last(ci->ilist); instr != NULL;
             instr = instr_get_prev(instr))
            if (instr_is_return(instr)) break;
        /* Insert a label before it. */
        done_label = insert_new_label(GLOBAL_DCONTEXT, ci->ilist, instr);
        PRE(ci->ilist, slowpath_end, INSTR_CREATE_jmp_short
            (GLOBAL_DCONTEXT, opnd_create_instr(done_label)));
    }

    /* Re-calculate the number of instructions, since we just deleted a bunch.
     */
    ci->num_instrs = 0;
    for (instr = instrlist_first(ci->ilist); instr != NULL;
         instr = instr_get_next(instr)) {
        if (!instr_is_label(instr))
            ci->num_instrs++;
    }
}

/* Tries to see if we can coalesce checks for this callee. */
/* XXX: Problem with check coalescing:
 *
 * Original code:
 * ld pos
 * inc pos
 * check pos
 * st pos
 * st buffer
 *
 * Coalesced checks:
 * ld pos
 * inc pos
 * check pos
 * ld pos
 * inc pos
 * check pos
 *
 * pos is not saved, so checks will pass if first check passes.  :(
 *
 * In model, can be:
 * <call>
 * <appcode>
 * <call>
 * <appcode>
 * <call>
 *
 * Can assume appcode doesn't touch riprel/absmem globals.
 *
 * PROBLEM: Can't assume buffer doesn't alias pos.
 */
static __attribute__((unused)) void
analyze_callee_coalesce_check(void *dc, callee_info_t *ci)
{
    instr_t *instr;
    instr_t *jcc_instr;
    instr_t *fastpath_start;
    //liveness_t liveness_;
    //liveness_t *liveness = &liveness_;

    ci->opt_coalesce_checks = false;
    if (!ci->opt_partial) {
        dr_log(dc, LOG_CLEANCALL, 3,
               "drcalls: can't coalesce checks, not partial inline\n");
        return;
    }

    ci->check_ilist = instrlist_create(GLOBAL_DCONTEXT);

    /* Clone entry block. */
    for (instr = instrlist_first(ci->ilist); !instr_is_cti(instr);
         instr = instr_get_next(instr)) {
        instrlist_append(ci->check_ilist, instr_clone(GLOBAL_DCONTEXT, instr));
    }

    /* Instr is now partial check instr, which jumps to fastpath if check
     * passes.  Invert it and remove the target, which has a dangling pointer
     * to a label. */
    if (!instr_is_jcc(instr)) {
        dr_log(dc, LOG_CLEANCALL, 3,
               "drcalls: can't coalesce, cannot invert check\n");
        return;
    }
    jcc_instr = instr_clone(GLOBAL_DCONTEXT, instr);
    instr_invert_cbr(jcc_instr);
    instr_set_target(jcc_instr, opnd_create_pc(NULL));
    instrlist_append(ci->check_ilist, jcc_instr);

    /* Create the fast ilist by cloning the check ilist and removing the check.
     * Clean up later with DCE. */
    ci->fast_ilist = instrlist_clone(GLOBAL_DCONTEXT, ci->check_ilist);
    remove_and_destroy(GLOBAL_DCONTEXT, ci->fast_ilist,
                       instrlist_last(ci->fast_ilist));

    /* Append the fastpath. */
    fastpath_start = opnd_get_instr(instr_get_target(instr));
    for (instr = fastpath_start; instr != NULL; instr = instr_get_next(instr)) {
        instrlist_append(ci->fast_ilist, instr_clone(GLOBAL_DCONTEXT, instr));
    }

    instrlist_disassemble(dc, NULL, ci->check_ilist, STDOUT);
    instrlist_disassemble(dc, NULL, ci->fast_ilist, STDOUT);
}

static reg_id_t
get_fp_reg(callee_info_t *ci)
{
    return (ci->xbp_is_fp ? DR_REG_XBP : DR_REG_XSP);
}

static bool
opnd_is_stack_access(callee_info_t *ci, opnd_t opnd)
{
    return (opnd_is_base_disp(opnd) && opnd_get_base(opnd) == get_fp_reg(ci));
}

/* Look for stack accesses.  Convert them to use our scratch slots. */
static void
analyze_callee_stack_usage(void *dcontext, callee_info_t *ci)
{
    instr_t *instr;
    instr_t *next_instr;
    opnd_t opnd, mem_ref, slot;
    int i;

    mem_ref = opnd_create_null();
    ci->stack_complex = false;
    ci->has_locals = false;
    for (instr  = instrlist_first(ci->ilist);
         instr != NULL;
         instr  = next_instr) {
        uint opc = instr_get_opcode(instr);
        next_instr = instr_get_next(instr);

        if (opc == OP_ret) {
            /* Ignore writes to XSP from returns. */
            continue;
        }

        /* sanity checks on stack usage */
        if (instr_writes_to_reg(instr, DR_REG_XBP) && ci->xbp_is_fp) {
            /* xbp must not be changed if xbp is used for frame pointer */
            dr_log(dcontext, LOG_CLEANCALL, 1,
                   "drcalls: callee "PFX" cannot be inlined: XBP is updated.\n",
                   ci->start);
            ci->stack_complex = true;
            break;
        } else if (instr_writes_to_reg(instr, DR_REG_XSP)) {
            /* TODO(rnk): For partial inlining, it would be best to pull this
             * analysis code out so we can know the frame size.  For now we
             * cannot partially inline callees that adjust the stack in the
             * prologue. */
            /* stack pointer update, we only allow:
             * lea [xsp, disp] => xsp
             * xsp + imm_int => xsp
             * xsp - imm_int => xsp
             */
            if (ci->has_locals) {
                /* we do not allow stack adjustment after accessing the stack */
                ci->stack_complex = true;
            }
            if (opc == OP_lea) {
                /* lea [xsp, disp] => xsp */
                opnd = instr_get_src(instr, 0);
                if (!opnd_is_base_disp(opnd)           ||
                    opnd_get_base(opnd)  != DR_REG_XSP ||
                    opnd_get_index(opnd) != DR_REG_NULL)
                    ci->stack_complex = true;
            } else if (opc == OP_sub || opc == OP_add) {
                /* xsp +/- int => xsp */
                if (!opnd_is_immed_int(instr_get_src(instr, 0)))
                    ci->stack_complex = true;
            } else if (opc == OP_call && ci->opt_partial) {
                opnd = instr_get_target(instr);
                if (opnd_is_instr(opnd) &&
                    opnd_get_instr(opnd) == ci->partial_label) {
                    /* Don't let the rest of this loop examine this call
                     * instruction for local variable reads/writes; it won't
                     * understand it. */
                    continue;
                }
            } else {
                /* other cases like push/pop are not allowed */
                ci->stack_complex = true;
            }
            if (!ci->stack_complex) {
                /* Delete frame adjust, we do our own. */
                dr_log(dcontext, LOG_CLEANCALL, 3,
                       "drcalls: removing frame adjustment at "PFX".\n",
                       instr_get_app_pc(instr));
                remove_and_destroy(GLOBAL_DCONTEXT, ci->ilist, instr);
                continue;
            } else {
                dr_log(dcontext, LOG_CLEANCALL, 1,
                       "drcalls: callee "PFX" cannot be inlined: "
                       "complicated stack pointer update at "PFX".\n",
                    ci->start, instr_get_app_pc(instr));
                break;
            }
        } else if (instr_reg_in_src(instr, get_fp_reg(ci))) {
            /* Detect stack address leakage */
            /* lea [xsp/xbp] */
            if (opc == OP_lea)
                ci->stack_complex = true;
            /* any direct use reg xsp or xbp */
            for (i = 0; i < instr_num_srcs(instr); i++) {
                opnd_t src = instr_get_src(instr, i);
                if (opnd_is_reg(src)) {
                    reg_id_t src_reg = opnd_get_reg(src);
                    if (reg_overlap(get_fp_reg(ci), src_reg))
                        break;
                }
            }
            if (i != instr_num_srcs(instr))
                ci->stack_complex = true;
            if (ci->stack_complex) {
                dr_log(dcontext, LOG_CLEANCALL, 1,
                       "drcalls: callee "PFX" cannot be inlined: "
                       "stack pointer leaked "PFX".\n",
                    ci->start, instr_get_app_pc(instr));
                break;
            }
        }

        /* Check how many stack variables the callee has.  We will not inline
         * the callee if it has more than one stack variable.  */
        if (instr_reads_memory(instr)) {
            for (i = 0; i < instr_num_srcs(instr); i++) {
                opnd = instr_get_src(instr, i);
                if (!opnd_is_stack_access(ci, opnd))
                    continue;
                if (!ci->has_locals) {
                    /* We see the first one, rembmer it. */
                    mem_ref = opnd;
                    ci->has_locals = true;
                } else if (!opnd_same(opnd, mem_ref)) {
                    /* Check if it is the same stack var as the one we saw.
                     * If different, no inline. 
                     */
                    dr_log(dcontext, LOG_CLEANCALL, 1,
                           "drcalls: callee "PFX" cannot be inlined: "
                           "more than one stack location is accessed "PFX".\n",
                        ci->start, instr_get_app_pc(instr));
                    break;
                }
                /* replace the stack location with the last stack slot. */
                slot = OPND_CREATE_MEMPTR(DR_REG_XSP, 0);
                opnd_set_size(&slot, opnd_get_size(mem_ref));
                instr_set_src(instr, i, slot);
            }
            if (i != instr_num_srcs(instr)) {
                ci->stack_complex = true;
                break;
            }
        }
        if (instr_writes_memory(instr)) {
            for (i = 0; i < instr_num_dsts(instr); i++) {
                opnd = instr_get_dst(instr, i);
                if (!opnd_is_stack_access(ci, opnd))
                    continue;
                if (!ci->has_locals) {
                    mem_ref = opnd;
                    ci->has_locals = true;
                } else if (!opnd_same(opnd, mem_ref)) {
                    /* currently we only allows one stack refs */
                    dr_log(dcontext, LOG_CLEANCALL, 1,
                           "drcalls: callee "PFX" cannot be inlined: "
                           "more than one stack location is accessed "PFX".\n",
                        ci->start, instr_get_app_pc(instr));
                    break;
                }
                /* replace the stack location with the last stack slot. */
                slot = OPND_CREATE_MEMPTR(DR_REG_XSP, 0);
                opnd_set_size(&slot, opnd_get_size(mem_ref));
                instr_set_dst(instr, i, slot);
            }
            if (i != instr_num_dsts(instr)) {
                ci->stack_complex = true;
                break;
            }
        }
    }

    if (ci->has_locals) {
        ci->framesize += sizeof(reg_t);
    }
    ci->framesize = ALIGN_FORWARD_UINT(ci->framesize, 16);
}

static void
analyze_callee_inline(void *dcontext, callee_info_t *ci)
{
    bool opt_inline = true;
    instr_t *instr;

    /* a set of condition checks */
    if (opt_cleancall < 2) {
        dr_log(dcontext, LOG_CLEANCALL, 1,
               "drcalls: callee "PFX" cannot be inlined: opt_cleancall: %d.\n",
               ci->start, opt_cleancall);
        opt_inline = false;
    }
    if (ci->num_instrs > MAX_NUM_INLINE_INSTRS) {
        dr_log(dcontext, LOG_CLEANCALL, 1,
               "drcalls: callee "PFX" cannot be inlined: num of instrs: %d.\n",
               ci->start, ci->num_instrs);
        opt_inline = false;
    }
    if (!ci->is_leaf && !ci->opt_partial) {
        dr_log(dcontext, LOG_CLEANCALL, 1,
               "drcalls: callee "PFX" cannot be inlined: "
               "has complex control flow.\n", ci->start);
        opt_inline = false;
    }
    if (ci->num_xmms_used != 0) {
        dr_log(dcontext, LOG_CLEANCALL, 1,
               "drcalls: callee "PFX" cannot be inlined: uses XMM.\n",
               ci->start);
        opt_inline = false;
    }
    if (ci->tls_used) {
        dr_log(dcontext, LOG_CLEANCALL, 1,
               "drcalls: callee "PFX" cannot be inlined: accesses TLS.\n",
               ci->start);
        opt_inline = false;
    }
    if (ci->num_regs_used > NUM_SCRATCH_SLOTS) {
        dr_log(dcontext, LOG_CLEANCALL, 1,
               "drcalls: callee "PFX" cannot be inlined: "
               "uses too many scratch slots: %d > %d.\n",
               ci->start, ci->num_regs_used, NUM_SCRATCH_SLOTS);
        opt_inline = false;
    }
    if (ci->stack_complex) {
        dr_log(dcontext, LOG_CLEANCALL, 1,
               "drcalls: callee "PFX" cannot be inlined: "
               "complex stack usage.\n", ci->start);
        opt_inline = false;
    }

    /* Remove trailing ret if we're going to inline. */
    /* TODO(rnk): Handle ret not being last instruction and multiple rets.
     * Idea: Create label at last ilist instruction and insert short jumps to
     * it. */
    instr = instrlist_last(ci->ilist);
    if (instr != NULL) {
        if (instr_is_return(instr)) {
            remove_and_destroy(GLOBAL_DCONTEXT, ci->ilist, instr);
        } else {
            dr_log(dcontext, LOG_CLEANCALL, 1,
                   "drcalls: callee "PFX" cannot be inlined: "
                   "last instruction is not return.\n", ci->start);
            opt_inline = false;
        }
    }

    ci->opt_inline = opt_inline;
    if (!opt_inline) {
        instrlist_clear_and_destroy(GLOBAL_DCONTEXT, ci->ilist);
        ci->ilist = NULL;
        return;
    }

    /* If we succeeded in partial inlining, emit the slowpath and fix up the
     * jmp to the label. */
    /* XXX: May emit multiple times if racing to insert callee in multiple
     * threads.
     */
    if (ci->opt_partial) {
        app_pc slowpath_pc = emit_partial_slowpath(dcontext, ci);
        for (instr = instrlist_first(ci->ilist); instr != NULL;
             instr = instr_get_next(instr)) {
            if (instr_is_call(instr) &&
                opnd_is_instr(instr_get_target(instr)) &&
                opnd_get_instr(instr_get_target(instr)) == ci->partial_label) {
                /* TODO(rnk): Reachability on X64, or does MAP_32BIT take care
                 * of this? */
                instr_set_target(instr, opnd_create_pc(slowpath_pc));
                ci->partial_label = NULL;
                break;
            }
        }
    }
}

static void
analyze_callee_ilist(void *dc, callee_info_t *ci)
{
    ASSERT(!ci->bailout && ci->ilist != NULL);
    if (opt_cleancall < 1) {
        /* Only optimization at opt 0 is skipping unused registers. */
        analyze_callee_regs_usage(dc, ci);
        instrlist_clear_and_destroy(GLOBAL_DCONTEXT, ci->ilist);
        ci->ilist = NULL;
    } else {
        resolve_internal_brs(dc, ci);
        if (ci->bailout) {
            /* Happens for internal branches.  We let indirect branches through
             * for further analysis. */
            instrlist_clear_and_destroy(GLOBAL_DCONTEXT, ci->ilist);
            ci->ilist = NULL;
            return;
        }
        rewrite_pic_code(dc, ci);
        analyze_callee_cti(dc, ci);
        analyze_callee_setup(dc, ci);
        if (opt_cleancall >= 3) {
            analyze_callee_partial(dc, ci);
            /* Optimizations are expensive, avoid wasting time on huge callees.
             */
            if (ci->num_instrs > 2 * MAX_NUM_INLINE_INSTRS) {
                dr_log(dc, LOG_CLEANCALL, 3,
                       "drcalls: skipping optimizing huge callee ilist.\n");
            } else {
                dce_and_copy_prop(dc, ci);
                reuse_registers(dc, ci);
                try_fold_immeds(dc, GLOBAL_DCONTEXT, ci->ilist);
                try_avoid_flags(dc, ci);
                fold_leas(dc, GLOBAL_DCONTEXT, ci->ilist);
                redundant_load_elim(dc, GLOBAL_DCONTEXT, ci->ilist);
                dead_store_elim(dc, GLOBAL_DCONTEXT, ci->ilist);
                dce_and_copy_prop(dc, ci);
                remove_jmp_next_instr(dc, GLOBAL_DCONTEXT, ci->ilist);
            }
            //analyze_callee_coalesce_check(dc, ci);
        }
        analyze_callee_regs_usage(dc, ci);
        analyze_callee_tls(dc, ci);
        analyze_callee_stack_usage(dc, ci);
        analyze_callee_inline(dc, ci);
        if (ci->ilist != NULL) {
            dr_log(dc, LOG_CLEANCALL, 3,
                   "drcalls: callee ilist after optimization/analysis:\n");
            instrlist_disassemble(dc, NULL, ci->ilist, dr_get_logfile(dc));
        }
    }
}
