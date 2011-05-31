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

/* This module is responsible for optimizing instrumentation ilists. */

#include "dr_api.h"
#include "callee.h"

#include "optimize.h"

#include <limits.h> /* for INT_MAX */
#include <string.h>

/* Dead register analysis abstraction.  Drive it by looping over instructions
 * in reverse.  At end of loop body, call reg_dead_update. */

void
liveness_init(liveness_t *liveness, const callee_info_t *ci)
{
    int i;
    /* At callee end, all registers are considered dead except XSP. */
    for (i = 0; i < NUM_GP_REGS; i++)
        liveness->reg_live[i] = false;
    SET_LIVE(DR_REG_XSP, true);
    if (ci->xbp_is_fp)
        SET_LIVE(DR_REG_XBP, true);
    liveness->flags_live = 0;
}

void
liveness_update(liveness_t *liveness, instr_t *instr)
{
    uint flags = instr_get_eflags(instr);
    int i;

    /* Mark all flags written to as dead.  */
    liveness->flags_live &= ~(EFLAGS_WRITE_TO_READ(flags & EFLAGS_WRITE_ALL));
    /* Mark all flags read from as live.  */
    liveness->flags_live |= (flags & EFLAGS_READ_ALL);

    /* TODO(rnk): Can mark destination register as dead here, but will
     * break if there is control flow, for example rax is still live here:
     * mov %rbx, %rax
     * test %rcx, %rcx
     * jz next
     * xor %rax, %rax
     * next:
     * mov %rax, global
     */

    /* Mark all regs read from as live.  */
    for (i = 0; i < NUM_GP_REGS; i++) {
        if (instr_reads_from_reg(instr, DR_REG_XAX + (reg_id_t)i)) {
            SET_LIVE(DR_REG_XAX + (reg_id_t)i, true);
        }
    }

    /* TODO(rnk): To support partial inlining, we may need to change this to
     * consider argument registers live-in to the call.
     */
}

/* Rewrite all reads of 'old' in 'opnd' to 'new', or return false.
 */
/* TODO(rnk): This doesn't handle al/ah type stuff.  ie, if we want to replace
 * RAX w/ RBX we need to rewrite BH to AH and not AL. */
static void
rewrite_opnd_reg(void *dc, instr_t *instr, opnd_t *opnd,
                 reg_id_t old, reg_id_t new, bool check_only)
{
    int i;
    new = reg_to_pointer_sized(new);
    IF_X64(new = reg_64_to_32(new);)
    for (i = 0; i < opnd_num_regs_used(*opnd); i++) {
        reg_id_t reg_used = opnd_get_reg_used(*opnd, i);
        if (reg_overlap(reg_used, old)) {
            reg_id_t new_sized = reg_32_to_opsz(new, reg_get_size(reg_used));
            if (!check_only) {
                dr_log(dc, LOG_CLEANCALL, 3,
                       "drcalls: rewriting reg %s to %s at "PFX"\n",
                       get_register_name(reg_used), get_register_name(new_sized),
                       instr_get_app_pc(instr));
            }
            opnd_replace_reg(opnd, reg_used, new_sized);
        }
    }
}

/* Wrapper for instr_is_encoding_possible that works on labels. */
static bool
can_encode(instr_t *instr)
{
    return instr_is_label(instr) || instr_is_encoding_possible(instr);
}

static bool
rewrite_reg_dst(void *dc, instr_t *instr, reg_id_t old, reg_id_t new,
                bool check_only)
{
    bool encodable;
    opnd_t opnd;

    /* Only rewrite instrs writing a single reg dst. */
    if (!instr_num_dsts(instr) == 1)
        return false;
    opnd = instr_get_dst(instr, 0);
    if (!opnd_is_reg(opnd))
        return false;

    /* Avoid side-effects on original instruction if we're just checking. */
    if (check_only)
        instr = instr_clone(dc, instr);

    opnd = instr_get_dst(instr, 0);
    rewrite_opnd_reg(dc, instr, &opnd, old, new, check_only);
    instr_set_dst(instr, 0, opnd);

    encodable = can_encode(instr);

    if (check_only)
        instr_destroy(dc, instr);

    return encodable;
}

/* Rewrite reads from in instr from 'old' to 'new'.  Returns true or false if
 * the resulting instruction is encodable.  If check_only is true, the
 * instruction will not be modified regardless of success or failure. */
static bool
rewrite_reg_reads(void *dc, instr_t *instr, reg_id_t old, reg_id_t new,
                  bool check_only)
{
    bool encodable;
    int i;

    /* Avoid side-effects on original instruction if we're just checking. */
    if (check_only)
        instr = instr_clone(dc, instr);

    /* srcs */
    for (i = 0; i < instr_num_srcs(instr); i++) {
        opnd_t opnd = instr_get_src(instr, i);
        rewrite_opnd_reg(dc, instr, &opnd, old, new, check_only);
        instr_set_src(instr, i, opnd);
    }

    /* dsts */
    for (i = 0; i < instr_num_dsts(instr); i++) {
        opnd_t opnd = instr_get_dst(instr, i);
        if (!opnd_is_base_disp(opnd))
            continue;  /* The only reads in dsts are regs in base/disp opnds. */
        rewrite_opnd_reg(dc, instr, &opnd, old, new, check_only);
        instr_set_dst(instr, i, opnd);
    }

    encodable = can_encode(instr);

    if (check_only)
        instr_destroy(dc, instr);

    return encodable;
}

/* Find the next full clobber of 'reg', if it exists.  If there is a
 * subregister write that does not fully clobber the register, return false.
 */
static bool
find_next_full_clobber(void *dc, instr_t *start, reg_id_t reg,
                       instr_t **end)
{
    instr_t *instr;
    int i;

    /* Find the next write of dst.  If it's a complicated subregister write, we
     * leave it alone.  Otherwise, we can rewrite up until that instruction. */
    for (instr = instr_get_next(start); instr != NULL;
         instr = instr_get_next(instr)) {
        if (!instr_writes_to_reg(instr, reg))
            continue;

        for (i = 0; i < instr_num_dsts(instr); i++) {
            opnd_t d = instr_get_dst(instr, i);
            if (opnd_is_reg(d) && reg_overlap(opnd_get_reg(d), reg)) {
                opnd_size_t sz = opnd_get_size(d);
                if (sz == OPSZ_4 || sz == OPSZ_8) {
                    /* dst is fully clobbered here. */
                    *end = instr;
                    return true;
                } else {
                    /* Subregister write, clobber is not full. */
                    return false;
                }
            }
        }
    }
    if (instr == NULL)
        *end = NULL;
    return true;
}

/* Rewrite all uses of reg 'old' to 'new' over the live range of 'old' starting
 * at start.  Rewrites the destination of start, and all reads up until the
 * next write that fully clobbers 'old'.  Returns false without making any
 * changes on failure. */
static bool
rewrite_live_range(void *dc, instr_t *start, reg_id_t old, reg_id_t new)
{
    instr_t *instr;
    instr_t *last_instr = NULL;
    DEBUG_DECLARE(bool ok);

    /* We only rewrite up until the next instruction that fully clobbers the
     * destination register.  If the update is not a full clobber (ie a write
     * of a subreg) then we bail. */
    if (!find_next_full_clobber(dc, start, old, &last_instr))
        return false;
    /* If the write contains any reads, make sure to update them.  For example,
     * if %rax is old and %rbx is new and the following is last_instr:
     * lea -24(%rax), %rax
     * It should be rewritten as well to become:
     * lea -24(%rbx), %rax
     */
    if (last_instr != NULL)
        last_instr = instr_get_next(last_instr);

    /* First check if we can rewrite the dst. */
    if (!rewrite_reg_dst(dc, start, old, new, /*check_only=*/true))
        return false;
    /* Find all reads of old and check if we can rewrite. */
    for (instr = instr_get_next(start); instr != last_instr;
         instr = instr_get_next(instr))
        if (!rewrite_reg_reads(dc, instr, old, new, /*check_only=*/true))
            return false;

    /* Now that we know this will work, actually rewrite the registers. */
    IF_DEBUG(ok =) rewrite_reg_dst(dc, start, old, new, /*check_only=*/false);
    ASSERT(ok);
    for (instr = instr_get_next(start); instr != last_instr;
         instr = instr_get_next(instr)) {
        IF_DEBUG(ok =) rewrite_reg_reads(dc, instr, old, new,
                                         /*check_only=*/false);
        ASSERT(ok);
    }
    return true;
}

/* Propagate copies from dead source registers to reduce register pressure.
 * Removes and destroys 'copy' and returns true if successful, otherwise
 * returns false.
 */
static bool
propagate_copy(void *dc, instrlist_t *ilist, instr_t *copy)
{
    reg_id_t src = opnd_get_reg(instr_get_src(copy, 0));
    reg_id_t dst = opnd_get_reg(instr_get_dst(copy, 0));
    opnd_size_t copy_sz = reg_get_size(dst);

    /* Let's not get into copy sizes that don't clobber the entire GPR. */
    if (!(copy_sz == OPSZ_4 || copy_sz == OPSZ_8))
        return false;

    dr_log(dc, LOG_CLEANCALL, 3,
           "drcalls: propagating copy at "PFX"\n", instr_get_app_pc(copy));

    if (!rewrite_live_range(dc, copy, dst, src))
        return false;

    /* Delete the copy. */
    remove_and_destroy(GLOBAL_DCONTEXT, ilist, copy);

    return true;
}

static bool
can_use_immed_for_opnd_reg(ptr_int_t val, reg_id_t dst_reg, reg_id_t opnd_reg)
{
    /* We can fold if opnd_reg is dst_reg or if it's the 64-bit version of
     * dst_reg and val is less than INT_MAX.  If it is larger, it will be sign
     * extended when the code requires zero extension. */
    return (opnd_reg == dst_reg ||
            IF_X64_ELSE((opnd_reg == reg_32_to_64(dst_reg) &&
                         val <= INT_MAX), false));
}

static bool
rewrite_reg_immed(void *dc, instr_t *instr, reg_id_t reg,
                  ptr_int_t val, bool check_only)
{
    opnd_t imm = opnd_create_immed_int(val, OPSZ_4);
    bool failed = false;
    int i;

    /* Avoid side-effects on original instruction if we're just checking. */
    if (check_only)
        instr = instr_clone(dc, instr);

    for (i = 0; i < instr_num_srcs(instr); i++) {
        opnd_t opnd = instr_get_src(instr, i);
        if (!opnd_uses_reg(opnd, reg))
            continue;
        if (opnd_is_reg(opnd)) {
            reg_id_t opnd_reg = opnd_get_reg(opnd);
            if (can_use_immed_for_opnd_reg(val, reg, opnd_reg)) {
                instr_set_src(instr, i, imm);
            } else {
                dr_log(dc, LOG_CLEANCALL, 3,
                       "drcalls: unable to fold due to subreg use at "PFX"\n",
                       instr_get_app_pc(instr));
                failed = true;
                break;
            }
        } else if (opnd_is_base_disp(opnd)) {
            ptr_int_t disp = opnd_get_disp(opnd);
            reg_id_t base = opnd_get_base(opnd);
            reg_id_t index = opnd_get_index(opnd);
            int scale = opnd_get_scale(opnd);
            opnd_size_t sz = opnd_get_size(opnd);
            if (can_use_immed_for_opnd_reg(val, reg, index)) {
                disp += val * scale;
                index = DR_REG_NULL;
                scale = 0;
            } else if (can_use_immed_for_opnd_reg(val, reg, base)) {
                disp += val;
                if (scale == 1) {
                    base = index;
                    index = DR_REG_NULL;
                    scale = 0;
                } else if (scale == 2) {
                    base = index;
                    index = index;
                    scale = 1;
                } else {
                    dr_log(dc, LOG_CLEANCALL, 3,
                           "drcalls: can't fold immed with scale %d\n", scale);
                    failed = true;
                    break;
                }
            } else {
                dr_log(dc, LOG_CLEANCALL, 3,
                       "drcalls: unable to fold immed in subreg use at "PFX"\n",
                       instr_get_app_pc(instr));
                failed = true;
                break;
            }
            instr_set_src(instr, i, opnd_create_base_disp
                          (base, index, scale, disp, sz));
        } else {
            dr_log(dc, LOG_CLEANCALL, 3,
                   "drcalls: unrecognized use of reg at "PFX"\n",
                   instr_get_app_pc(instr));
            failed = true;
            break;
        }
        if (!check_only) {
            dr_log(dc, LOG_CLEANCALL, 3,
                   "drcalls: folded immediate into src opnd at "PFX"\n",
                   instr_get_app_pc(instr));
        }
    }

    for (i = 0; i < instr_num_dsts(instr); i++) {
        opnd_t opnd = instr_get_src(instr, i);
        /* Only base-disp opnds read regs. */
        if (!opnd_is_base_disp(opnd))
            continue;
        if (opnd_uses_reg(opnd, reg)) {
            dr_log(dc, LOG_CLEANCALL, 3,
                   "drcalls: can't yet fold immediate into dst opnd at "PFX"\n",
                   instr_get_app_pc(instr));
            failed = true;
            break;
        }
    }

    failed |= !can_encode(instr);

    if (check_only)
        instr_destroy(dc, instr);

    return !failed;
}

static bool
rewrite_live_range_immed(void *dc, instr_t *start, instr_t *end,
                         reg_id_t reg, ptr_int_t val, bool check_only)
{
    instr_t *instr;

    ASSERT(reg_get_size(reg) == OPSZ_4);
    for (instr = instr_get_next(start); instr != end;
         instr = instr_get_next(instr))
        if (!rewrite_reg_immed(dc, instr, reg, val, check_only))
            return false;
    return true;
}

static reg_id_t
pick_used_dead_reg(void *dc, instrlist_t *ilist, liveness_t *liveness,
                   instr_t *reuse_instr)
{
    bool reg_used[NUM_GP_REGS];
    instr_t *instr;
    uint i;
    reg_id_t reg;

    /* Compute registers used before reuse_instr. */
    memset(reg_used, 0, sizeof(reg_used));
    for (instr = instrlist_first(ilist); instr != reuse_instr;
         instr = instr_get_next(instr)) {
        for (i = 0; i < NUM_GP_REGS; i++) {
            if (instr_uses_reg(instr, DR_REG_XAX + (reg_id_t)i))
                reg_used[i] = true;
        }
    }
    /* Also count registers read from by reuse_instr. */
    for (i = 0; i < NUM_GP_REGS; i++) {
        if (instr_reads_from_reg(reuse_instr, DR_REG_XAX + (reg_id_t)i))
            reg_used[i] = true;
    }

    /* Find first dead register that we've already used. */
    for (i = 0; i < NUM_GP_REGS; i++)
        if (reg_used[i] && IS_DEAD(DR_REG_XAX + (reg_id_t)i))
            break;
    if (i == NUM_GP_REGS)
        return DR_REG_NULL;
    reg = DR_REG_XAX + (reg_id_t)i;
    ASSERT(reg_used[i] && IS_DEAD(reg));
    return reg;
}

static reg_id_t
pick_dead_reg(void *dc, liveness_t *liveness)
{
    int i;

    for (i = 0; i < NUM_GP_REGS; i++) {
        reg_id_t reg = DR_REG_XAX + (reg_id_t)i;
        if (IS_DEAD(reg))
            return reg;
    }
    return DR_REG_NULL;
}

/* If this is a reg-to-reg instruction that uses a new register when there are
 * existing dead registers that we've already used, try to reuse registers.
 * This creates less saves and restores.  */
static void
try_reuse_register(void *dc, instrlist_t *ilist, liveness_t *liveness,
                   instr_t *reuse_instr)
{
    opnd_t opnd = instr_get_dst(reuse_instr, 0);
    reg_id_t dst = opnd_get_reg(opnd);
    reg_id_t new;

    new = pick_used_dead_reg(dc, ilist, liveness, reuse_instr);
    if (new == DR_REG_NULL)
        return;

    dr_log(dc, LOG_CLEANCALL, 3,
           "drcalls: trying to reuse dead register %s instead of %s at "PFX"\n",
           get_register_name(new), get_register_name(dst),
           instr_get_app_pc(reuse_instr));

    /* Rewrite dst and all further uses of dst to new. */
    if (rewrite_live_range(dc, reuse_instr, dst, new)) {
        dr_log(dc, LOG_CLEANCALL, 3,
               "drcalls: succeeded reusing dead register %s instead of %s at "
               PFX"\n",
               get_register_name(new), get_register_name(dst),
               instr_get_app_pc(reuse_instr));
    }
}

/* For movs from immediate to register, try to propagate immediate into
 * instructions that use it.
 */
static bool
fold_mov_immed(void *dc, instrlist_t *ilist, instr_t *mov_imm)
{
    opnd_t imm = instr_get_src(mov_imm, 0);
    opnd_t dst = instr_get_dst(mov_imm, 0);
    ptr_int_t val = opnd_get_immed_int(imm);
    reg_id_t reg = opnd_get_reg(dst);
    instr_t *end = NULL;
    DEBUG_DECLARE(bool ok;)

    /* For now we only deal with 32-bit immediates, since pointer values are
     * trickier to rewrite. */
    /* TODO(rnk): If pointer value is < 4 GB, can use reladdr memory operands
     * instead of materializing the pointer as an immediate.  */
    if (opnd_get_size(dst) != OPSZ_4)
        return false;

    if (!find_next_full_clobber(dc, mov_imm, reg, &end))
        return false;
    if (end != NULL)
        end = instr_get_next(end);

    if (end) {
        dr_log(dc, LOG_CLEANCALL, 3,
               "drcalls: mov imm live range ends at "PFX"\n",
               instr_get_app_pc(end));
    }
    dr_log(dc, LOG_CLEANCALL, 3,
           "drcalls: attempting to fold mov_imm at "PFX"\n",
           instr_get_app_pc(mov_imm));
    if (!rewrite_live_range_immed(dc, mov_imm, end, reg, val, true))
        return false;
    IF_DEBUG(ok =) rewrite_live_range_immed(dc, mov_imm, end, reg, val, false);
    ASSERT(ok);
    dr_log(dc, LOG_CLEANCALL, 3,
           "drcalls: folded immediate mov at "PFX"\n",
           instr_get_app_pc(mov_imm));
    return true;
}

static bool
addsub_to_lea(void *dc, const callee_info_t *ci, liveness_t *liveness,
              instr_t *instr, bool check_only)
{
    opnd_t src = instr_get_src(instr, 0);
    opnd_t dst = instr_get_dst(instr, 0);
    reg_id_t dst_reg;
    ASSERT(opnd_same(dst, instr_get_src(instr, 1)));
    if (check_only) {
        dr_log(dc, LOG_CLEANCALL, 3,
               "drcalls: attempting add -> lea for: ");
        instr_disassemble(dc, instr, dr_get_logfile(dc));
        dr_log(dc, LOG_CLEANCALL, 3, "\n");
    }

    /* Extract memory operands from add into temp register loads and stores. */
    if (opnd_is_memory_reference(src) || opnd_is_memory_reference(dst)) {
        reg_id_t tmp_reg;
        opnd_t tmp;
        opnd_t memref = opnd_is_memory_reference(src) ? src : dst;

        ASSERT(!(opnd_is_memory_reference(src) &&
                 opnd_is_memory_reference(dst)));
        tmp_reg = pick_used_dead_reg(dc, ci->ilist, liveness, instr);
        if (tmp_reg == DR_REG_NULL) {
            if (check_only) {
                dr_log(dc, LOG_CLEANCALL, 3,
                       "drcalls: can't steal used dead reg, getting unused\n");
            }
            tmp_reg = pick_dead_reg(dc, liveness);
            if (tmp_reg == DR_REG_NULL) {
                dr_log(dc, LOG_CLEANCALL, 3,
                       "drcalls: can't steal dead reg, add -> lea failed\n");
                return false;
            }
        }
        /* Match the register size to the memref size. */
        IF_X64(tmp_reg = reg_64_to_32(tmp_reg);)
        tmp_reg = reg_32_to_opsz(tmp_reg, opnd_get_size(memref));
        tmp = opnd_create_reg(tmp_reg);

        if (!check_only) {
            /* Insert load in either case. */
            PRE(ci->ilist, instr, INSTR_CREATE_mov_ld
                (GLOBAL_DCONTEXT, tmp, memref));
            /* Insert store if memref is dst. */
            if (opnd_is_memory_reference(dst)) {
                POST(ci->ilist, instr, INSTR_CREATE_mov_st
                     (GLOBAL_DCONTEXT, memref, tmp));
            }
        }

        /* Replace memref w/ tmp. */
        src = opnd_is_memory_reference(src) ? tmp : src;
        dst = opnd_is_memory_reference(dst) ? tmp : dst;
    }

    /* Should be of form: add r/i, r */
    dst_reg = opnd_get_reg(dst);
    if (opnd_is_immed_int(src)) {
        /* lea +/-IMM(%dst), %dst */
        if (!check_only) {
            ptr_int_t disp = opnd_get_immed_int(src);
            if (instr_get_opcode(instr) == OP_sub)
                disp = -disp;
            PRE(ci->ilist, instr, INSTR_CREATE_lea
                (GLOBAL_DCONTEXT, dst,
                 OPND_CREATE_MEM_lea(dst_reg, DR_REG_NULL, 0, disp)));
            remove_and_destroy(GLOBAL_DCONTEXT, ci->ilist, instr);
        }
    } else if (opnd_is_reg(src)) {
        reg_id_t src_reg = opnd_get_reg(src);
        if (instr_get_opcode(instr) == OP_sub && !IS_DEAD(src_reg)) {
            dr_log(dc, LOG_CLEANCALL, 3,
                   "drcalls: sub -> lea failed, needs dead src reg\n");
            return false;
        }
        if (!check_only) {
            /* lea 0(%src, %dst, 1), %dst */
            PRE(ci->ilist, instr, INSTR_CREATE_lea
                (GLOBAL_DCONTEXT, dst,
                 OPND_CREATE_MEM_lea(dst_reg, src_reg, 1, 0)));
            remove_and_destroy(GLOBAL_DCONTEXT, ci->ilist, instr);
        }
    } else {
        dr_log(dc, LOG_CLEANCALL, 3,
               "drcalls: add -> lea failed, unrecognized src opnd\n");
        return false;
    }
    return true;
}

static bool
optimize_avoid_flags(void *dc, const callee_info_t *ci, bool check_only)
{
    liveness_t liveness_;
    liveness_t *liveness = &liveness_;
    instrlist_t *ilist = ci->ilist;
    instr_t *instr, *prev_instr;

    liveness_init(liveness, ci);
    for (instr = instrlist_last(ilist); instr != NULL; instr = prev_instr) {
        prev_instr = instr_get_prev(instr);

        /* Update before analysis to consider registers live-in to this
         * instruction to be live.  We can't use them as temp registers. */
        liveness_update(liveness, instr);

        switch (instr_get_opcode(instr)) {
        case OP_sub:
        case OP_add:
            if (!addsub_to_lea(dc, ci, liveness, instr, check_only))
                return false;
            break;
        case OP_jz:
            /* TODO(rnk): Do cmp -> not+lea trick. */
            return false;
        default:
            if (TESTANY(EFLAGS_WRITE_6, instr_get_arith_flags(instr)) ||
                TESTANY(EFLAGS_READ_6, instr_get_arith_flags(instr))) {
                dr_log(dc, LOG_CLEANCALL, 3,
                       "drcalls: flags required for instr at "PFX", "
                       "flags avoidance not profitable\n",
                       instr_get_app_pc(instr));
                return false;
            }
        }
    }
    return true;
}

void
try_avoid_flags(void *dc, const callee_info_t *ci)
{
    if (optimize_avoid_flags(dc, ci, /*check=*/true)) {
        IF_DEBUG(bool ok =) optimize_avoid_flags(dc, ci, /*check=*/false);
        ASSERT(ok);
    }
}

bool
liveness_instr_is_live(liveness_t *liveness, instr_t *instr)
{
    bool instr_is_live = false;
    int i;
    uint flags_written;

    /* If only dsts are regs, and all dsts are dead, can delete instr. */
    for (i = 0; i < instr_num_dsts(instr); i++) {
        opnd_t dst = instr_get_dst(instr, i);
        /* Our callers handle memory writes separately. */
        if (opnd_is_memory_reference(dst)) continue;
        if (!opnd_is_reg(dst)) break;
        if (!IS_DEAD(opnd_get_reg(dst))) break;
    }
    /* If the loop exited before completion, then the destinations are
     * live. */
    if (i < instr_num_dsts(instr))
        instr_is_live |= true;
    /* If the instr writes live flags it's live. */
    flags_written =
            EFLAGS_WRITE_TO_READ(EFLAGS_WRITE_ALL & instr_get_eflags(instr));
    if (TESTANY(liveness->flags_live, flags_written))
        instr_is_live |= true;
    return instr_is_live;
}

/* Remove and destroy instructions whose only destinations are dead registers.
 * Assumes there are no calls or backwards branches in ilist.
 * TODO(rnk): What about other control flow?
 */
void
dce_and_copy_prop(void *dc, const callee_info_t *ci)
{
    liveness_t liveness_;
    liveness_t *liveness = &liveness_;
    instrlist_t *ilist = ci->ilist;
    instr_t *instr;
    instr_t *prev_instr;

    liveness_init(liveness, ci);

    for (instr = instrlist_last(ilist); instr != NULL; instr = prev_instr) {
        int opc;
        bool is_live;
        prev_instr = instr_get_prev(instr);

        /* If the instr writes live regs, writes live flags, is a cti, is a
         * label, or writes memory then it's live.  */
        is_live = (liveness_instr_is_live(liveness, instr) ||
                   instr_writes_memory(instr) ||
                   instr_is_cti(instr) ||
                   instr_is_label(instr));
        if (!is_live) {
            dr_log(dc, LOG_CLEANCALL, 3,
                   "drcalls: removing dead instr at "PFX".\n",
                   instr_get_app_pc(instr));
            remove_and_destroy(GLOBAL_DCONTEXT, ilist, instr);
            continue;
        }

        /* If this is a copy from a dead src reg, we may be able to reduce
         * register pressure by deleting it and using src direcly. */
        opc = instr_get_opcode(instr);
        if ((opc == OP_mov_ld || opc == OP_mov_st) &&
            opnd_is_reg(instr_get_src(instr, 0)) &&
            opnd_is_reg(instr_get_dst(instr, 0))) {
            reg_id_t src_reg = opnd_get_reg(instr_get_src(instr, 0));
            if (IS_DEAD(src_reg)) {
                if (propagate_copy(dc, ilist, instr)) {
                    /* Src reg is live now. */
                    SET_LIVE(src_reg, true);
                    continue;
                }
            }
        }

        liveness_update(liveness, instr);
    }
}

void
reuse_registers(void *dc, const callee_info_t *ci)
{
    liveness_t liveness_;
    liveness_t *liveness = &liveness_;
    instrlist_t *ilist = ci->ilist;
    instr_t *instr;
    instr_t *prev_instr;

    /* Do a second pass and attempt to reuse registers.  We don't do this on
     * the first pass or we may attempt to reuse registers that we don't end up
     * using after DCE. */
    liveness_init(liveness, ci);
    for (instr = instrlist_last(ilist); instr != NULL; instr = prev_instr) {
        prev_instr = instr_get_prev(instr);

        /* If this is a reg-to-reg instruction that uses a new register when
         * there are existing dead registers that we've already used, try to
         * reuse registers.  This creates less saves and restores.  */
        if (!instr_is_cti(instr) &&
            instr_num_dsts(instr) == 1 &&
            opnd_is_reg(instr_get_dst(instr, 0))) {
            try_reuse_register(dc, ilist, liveness, instr);
        }

        liveness_update(liveness, instr);
    }
}

/* Folds "mov $imm -> %reg ; use %reg" to use $imm in place of %reg if possible.
 * dc_alloc is the dcontext used to allocate the ilist, usually either global or
 * thread local.
 */
void
try_fold_immeds(void *dc, void *dc_alloc, instrlist_t *ilist)
{
    instr_t *instr;
    instr_t *next_instr;

    for (instr = instrlist_first(ilist); instr != NULL; instr = next_instr) {
        next_instr = instr_get_next(instr);
        if (instr_is_mov(instr) &&
            opnd_is_immed_int(instr_get_src(instr, 0)) &&
            opnd_is_reg(instr_get_dst(instr, 0))) {
            if (fold_mov_immed(dc, ilist, instr)) {
                remove_and_destroy(dc_alloc, ilist, instr);
            }
        }
    }
}

/* Eliminate redundant loads.
 *
 * Before:
 * st r1 -> rel/abs
 * ...  # no writes of r1 or memory
 * ld rel/abs -> r2
 *
 * After:
 * st r1 -> rel/abs
 * ...
 * mov r1 -> r2  # or nothing if r1 == r2
 */
void
redundant_load_elim(void *dc, void *dc_alloc, instrlist_t *ilist)
{
    instr_t *instr;
    instr_t *next_instr;
    instr_t *store = NULL;
    instr_t *load = NULL;
    opnd_t src = opnd_create_null();
    opnd_t mem_ref = opnd_create_null();
    opnd_t dst_reg = opnd_create_null();
    reg_id_t base_reg = DR_REG_NULL;
    reg_id_t idx_reg = DR_REG_NULL;

    for (instr = instrlist_first(ilist); instr != NULL;
         instr = next_instr) {
        next_instr = instr_get_next(instr);

        /* If it's an app instruction, ignore it, only touch instrumentation. */
        if (instr_ok_to_mangle(instr)) {
            store = NULL;
            continue;
        }

        if (store == NULL) {
            if (instr_get_opcode(instr) == OP_mov_st) {
                store = instr;
                src = instr_get_src(instr, 0);
                mem_ref = instr_get_dst(instr, 0);

                /* If the memory operand reads any operands, we need to make
                 * sure those aren't udpated between the store and the load. */
                if (opnd_is_base_disp(mem_ref)) {
                    base_reg = opnd_get_base(mem_ref);
                    idx_reg = opnd_get_index(mem_ref);
                }
            }
        } else {
            if (instr_get_opcode(instr) == OP_mov_ld &&
                opnd_same(mem_ref, instr_get_src(instr, 0))) {
                load = instr;
                dst_reg = instr_get_dst(load, 0);
                if (opnd_same(src, dst_reg)) {
                    /* Can delete load. */
                    dr_log(dc, LOG_CLEANCALL, 3,
                           "drcalls: RLE: deleted redundant load: ");
                    instr_disassemble(dc, load, dr_get_logfile(dc));
                    dr_log(dc, LOG_CLEANCALL, 3, "\n");

                    remove_and_destroy(dc_alloc, ilist, load);
                } else {
                    /* Can change to mov r1 -> r2. */
                    dr_log(dc, LOG_CLEANCALL, 3,
                           "drcalls: RLE: changed load to reg copy: ");
                    instr_disassemble(dc, load, dr_get_logfile(dc));
                    dr_log(dc, LOG_CLEANCALL, 3, "\n");

                    instr_set_src(load, 0, src);
                }
                continue;
            }

            /* To avoid aliasing issues, we punt on any intermediate store, as
             * well as any cti or label, or clobbering or read registers. */
            if (instr_writes_memory(instr) ||
                instr_is_label(instr) ||
                instr_is_cti(instr) ||
                instr_writes_to_reg(instr, base_reg) ||
                instr_writes_to_reg(instr, idx_reg) ||
                (opnd_is_reg(src) &&
                 instr_writes_to_reg(instr, opnd_get_reg(src)))) {
                store = NULL;
            }
        }
    }
}

/* Dead store elimination.
 *
 * Before:
 * st op1 -> rel/abs
 * ...  # no cti
 * st op2 -> rel/abs
 *
 * After:
 * ...
 * st op2 -> rel/abs
 */
void
dead_store_elim(void *dc, void *dc_alloc, instrlist_t *ilist)
{
    instr_t *instr;
    instr_t *next_instr;
    instr_t *store = NULL;
    opnd_t mem_ref = opnd_create_null();
    reg_id_t base_reg = DR_REG_NULL;
    reg_id_t idx_reg = DR_REG_NULL;

    for (instr = instrlist_first(ilist); instr != NULL;
         instr = next_instr) {
        next_instr = instr_get_next(instr);

        /* If it's an app instruction, ignore it, only touch instrumentation. */
        if (instr_ok_to_mangle(instr)) {
            if (store != NULL) {
#ifdef VERBOSE
                dr_log(dc, LOG_CLEANCALL, 3,
                       "drcalls: DSE: blocked by non-meta instr: ");
                instr_disassemble(dc, instr, dr_get_logfile(dc));
                dr_log(dc, LOG_CLEANCALL, 3, "\n");
#endif
                store = NULL;
            }
            continue;
        }

        if (instr_get_opcode(instr) == OP_mov_st) {
            if (store == NULL) {
                store = instr;
                mem_ref = instr_get_dst(instr, 0);

                /* If the memory operand reads any operands, we need to make
                 * sure those aren't udpated between the store and the load. */
                if (opnd_is_base_disp(mem_ref)) {
                    base_reg = opnd_get_base(mem_ref);
                    idx_reg = opnd_get_index(mem_ref);
                }

#ifdef VERBOSE
                dr_log(dc, LOG_CLEANCALL, 3,
                       "drcalls: DSE: found store: ");
                instr_disassemble(dc, store, dr_get_logfile(dc));
                dr_log(dc, LOG_CLEANCALL, 3, "\n");
#endif
                continue;
            } else if (opnd_same(mem_ref, instr_get_dst(instr, 0))) {
                /* Can delete load. */
                dr_log(dc, LOG_CLEANCALL, 3,
                       "drcalls: DSE: deleted dead store: ");
                instr_disassemble(dc, store, dr_get_logfile(dc));
                dr_log(dc, LOG_CLEANCALL, 3, "\n");

                remove_and_destroy(dc_alloc, ilist, store);
                /* Note, this is store to the same mem_ref, base_reg, and
                 * idx_reg, try to DSE it next. */
                store = instr;
                continue;
            }
        }

        if (store != NULL &&
            (instr_is_label(instr) ||
             instr_is_cti(instr) ||
             instr_reads_memory(instr) ||
             instr_writes_memory(instr) ||
             instr_writes_to_reg(instr, base_reg) ||
             instr_writes_to_reg(instr, idx_reg))) {
#ifdef VERBOSE
            dr_log(dc, LOG_CLEANCALL, 3,
                   "drcalls: DSE: blocked by clobber: ");
            instr_disassemble(dc, instr, dr_get_logfile(dc));
            dr_log(dc, LOG_CLEANCALL, 3, "\n");
#endif

            store = NULL;
        }
    }
}

/* Remove jumps to the next instruction.  This comes up with partial inlining,
 * where we generate the code, but don't know that the code between the jmp and
 * the label is dead yet.
 */
void
remove_jmp_next_instr(void *dc, void *dc_alloc, instrlist_t *ilist)
{
    instr_t *instr;
    instr_t *next_instr;

    for (instr = instrlist_first(ilist); instr != NULL;
         instr = next_instr) {
        next_instr = instr_get_next(instr);

        if (instr_is_ubr(instr) &&
            next_instr != NULL &&
            instr_is_label(next_instr)) {
            opnd_t tgt = instr_get_target(instr);
            if (opnd_is_instr(tgt) &&
                opnd_get_instr(tgt) == next_instr) {
                instr_t *tmp = instr_get_prev(instr);
                dr_log(dc, LOG_CLEANCALL, 3,
                       "drcalls: removed jmp to next instr.\n");
                remove_and_destroy(dc_alloc, ilist, instr);
                remove_and_destroy(dc_alloc, ilist, next_instr);
                next_instr = tmp;
            }
        }
    }
}
