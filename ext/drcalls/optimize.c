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

/* We don't have a good solution for conditionally disassembling to the log, so
 * this ifdef VERBOSE controls the more verbose optimization logging. */
//#define VERBOSE

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

/******************************************************************************
 * Operand rewriting support.
 */

/* Wrapper for instr_is_encoding_possible that works on labels. */
static bool
can_encode(instr_t *instr)
{
    return instr_is_label(instr) || instr_is_encoding_possible(instr);
}

/* We can fold if use_reg is exactly imm_reg or if it's the 64-bit version of
 * imm_reg and val is less than INT_MAX.  If it is larger, it will be sign
 * extended when the code requires zero extension. */
static bool
can_use_immed_for_opnd_reg(ptr_int_t val, reg_id_t imm_reg, reg_id_t use_reg)
{
    return (use_reg == imm_reg ||
            IF_X64_ELSE((use_reg == reg_to_pointer_sized(imm_reg) &&
                         val <= INT_MAX), false));
}

/* Operand types.  We're outside of DR core, so we can't use opnd.kind.
 * Therefore, we make our own. */
typedef enum {
    OPND_REG,
    OPND_IMMED_INT,
    OPND_BASE_DISP,
    OPND_OTHER,
    OPND_LAST
} rewrite_opnd_kind_t;

static rewrite_opnd_kind_t
rewrite_opnd_kind(opnd_t opnd)
{
    if (opnd_is_reg(opnd))       return OPND_REG;
    if (opnd_is_immed_int(opnd)) return OPND_IMMED_INT;
    if (opnd_is_base_disp(opnd)) return OPND_BASE_DISP;
    return OPND_OTHER;
}

typedef opnd_t (*rewrite_opnd_fn_t)(void *dc, reg_id_t reg, opnd_t new_opnd,
                                    opnd_t old_opnd);
static rewrite_opnd_fn_t rewrite_table[OPND_LAST][OPND_LAST];

/******************************************************************************
 * Operand rewriting functions.
 */

/* TODO(rnk): This doesn't handle al/ah type stuff.  ie, if we want to replace
 * RAX w/ RBX we need to rewrite AH to BH and not BL. */
static opnd_t
rewrite_reg_into_reg(void *dc, reg_id_t reg, opnd_t new_opnd, opnd_t old_opnd)
{
    reg_id_t new = opnd_get_reg(new_opnd);
    reg_id_t old = opnd_get_reg(old_opnd);
    opnd_size_t sz = opnd_get_size(old_opnd);

    ASSERT(reg_overlap(reg, old));

    new = reg_to_pointer_sized(new);
    IF_X64(new = reg_64_to_32(new));
    new = reg_32_to_opsz(new, sz);
    return opnd_create_reg(new);
}

static opnd_t
rewrite_reg_into_memref(void *dc, reg_id_t reg, opnd_t new_opnd, opnd_t old_opnd)
{
    reg_id_t new = opnd_get_reg(new_opnd);

    reg_id_t base  = opnd_get_base(old_opnd);
    reg_id_t index = opnd_get_index(old_opnd);
    int      scale = opnd_get_scale(old_opnd);
    int      disp  = opnd_get_disp(old_opnd);
    opnd_size_t sz = opnd_get_size(old_opnd);

    new = reg_to_pointer_sized(new);
    IF_X64(new = reg_64_to_32(new));

    if (reg_overlap(base, reg))
        base = reg_32_to_opsz(new, reg_get_size(base));
    if (reg_overlap(index, reg))
        index = reg_32_to_opsz(new, reg_get_size(index));

    return opnd_create_base_disp(base, index, scale, disp, sz);
}

static opnd_t
rewrite_immed_into_reg(void *dc, reg_id_t reg, opnd_t new_opnd, opnd_t old_opnd)
{
    ptr_int_t imm_val = opnd_get_immed_int(new_opnd);
    reg_id_t use_reg = opnd_get_reg(old_opnd);
    if (can_use_immed_for_opnd_reg(imm_val, reg, use_reg)) {
        return new_opnd;
    } else {
        return opnd_create_null();
    }
}

static opnd_t
rewrite_immed_into_memref(void *dc, reg_id_t reg, opnd_t new_opnd, opnd_t old_opnd)
{
    ptr_int_t imm_val = opnd_get_immed_int(new_opnd);
    reg_id_t base  = opnd_get_base(old_opnd);
    reg_id_t index = opnd_get_index(old_opnd);
    int      scale = opnd_get_scale(old_opnd);
    int      disp  = opnd_get_disp(old_opnd);
    opnd_size_t sz = opnd_get_size(old_opnd);

    if (can_use_immed_for_opnd_reg(imm_val, reg, index)) {
        /* If reg was used as the index register, we can drop the index,
         * multiply immediate by scale, and encode that. */
        disp += imm_val * scale;
        index = DR_REG_NULL;
        scale = 0;
    } else if (can_use_immed_for_opnd_reg(imm_val, reg, base)) {
        /* If reg was used as the base register, we can drop the base and add
         * imm_val to disp. */
        disp += imm_val;
        base = DR_REG_NULL;
        /* If we don't need scale, we can use plain base disp for a simpler
         * operand. */
        if (scale == 1) {
            base = index;
            index = DR_REG_NULL;
            scale = 0;
        }
    } else {
        return opnd_create_null();
    }

    return opnd_create_base_disp(base, index, scale, disp, sz);
}

static opnd_t
rewrite_memref_into_reg(void *dc, reg_id_t reg, opnd_t new_opnd, opnd_t old_opnd)
{
    reg_id_t    new_base  = opnd_get_base (new_opnd);
    reg_id_t    new_index = opnd_get_index(new_opnd);
    int         new_disp  = opnd_get_disp (new_opnd);

    if (new_base == DR_REG_NULL && new_index == DR_REG_NULL) {
        /* Basically, this is a mov immediate. */
        return OPND_CREATE_INT32(new_disp);
    }
    return opnd_create_null();
}

static opnd_t
rewrite_memref_into_memref(void *dc, reg_id_t reg, opnd_t new_opnd, opnd_t old_opnd)
{
    reg_id_t    old_base  = opnd_get_base (old_opnd);
    reg_id_t    old_index = opnd_get_index(old_opnd);
    int         old_scale = opnd_get_scale(old_opnd);
    int         old_disp  = opnd_get_disp (old_opnd);
    opnd_size_t old_sz    = opnd_get_size (old_opnd);

    reg_id_t    new_base  = opnd_get_base (new_opnd);
    reg_id_t    new_index = opnd_get_index(new_opnd);
    int         new_scale = opnd_get_scale(new_opnd);
    int         new_disp  = opnd_get_disp (new_opnd);
    opnd_size_t new_sz    = opnd_get_size (new_opnd);

    ASSERT(new_sz == OPSZ_lea);

    if (old_base == reg) {
        /* If used in base reg, new_disp will not be multiplied by scale. */
        old_disp += new_disp;
        if (old_index == DR_REG_NULL) {
            /* lea 0x10(rbp, rsi, 4) -> rax ; mov 0x10(rax) */
            /* mov 0x20(rsp, rdi, 4) */
            old_base  = new_base;
            old_index = new_index;
            old_scale = new_scale;
        } else if (new_index == DR_REG_NULL) {
            /* lea 0x10(rsp) -> rax ; mov 0x10(rax, rdi, 4) */
            /* mov 0x20(rsp, rdi, 4) */
            old_base = new_base;
        } else if (new_base == DR_REG_NULL) {
            if (old_scale == 1) {
                /* lea 0x00(,%rax,8) -> %r10 ; mov %rsi -> 0x08(%r10,%r9,1) */
                /* mov %rsi -> 0x08(%r9,%rax,8) */
                old_base = old_index;
                old_index = new_index;
                old_scale = new_scale;
            } else if (new_scale == 1) {
                /* lea 0x00(,%rax,1) -> %r10 ; mov %rsi -> 0x08(%r10,%r9,8) */
                /* mov %rsi -> 0x08(%rax,%r9,8) */
                /* XXX: When would a compiler *ever* generate this? */
                old_base = new_index;
            } else {
                return opnd_create_null();
            }
        } else {
            return opnd_create_null();
        }
    } else if (old_index == reg) {
        return opnd_create_null();
    } else {
        return opnd_create_null();
    }
    return opnd_create_base_disp(old_base, old_index, old_scale, old_disp,
                                 old_sz);
}

/*****************************************************************************/
/* Live range rewriting. */

void
rewrite_opnd_table_init(void)
{
    rewrite_table[OPND_REG      ][OPND_REG      ] = rewrite_reg_into_reg;
    rewrite_table[OPND_REG      ][OPND_BASE_DISP] = rewrite_reg_into_memref;
    rewrite_table[OPND_IMMED_INT][OPND_REG      ] = rewrite_immed_into_reg;
    rewrite_table[OPND_IMMED_INT][OPND_BASE_DISP] = rewrite_immed_into_memref;
    rewrite_table[OPND_BASE_DISP][OPND_REG      ] = rewrite_memref_into_reg;
    rewrite_table[OPND_BASE_DISP][OPND_BASE_DISP] = rewrite_memref_into_memref;
}

static void
log_unable_combine_opnds(void *dc, opnd_t new_opnd, opnd_t old_opnd)
{
    dr_log(dc, LOG_CLEANCALL, 3,
           "drcalls: unable to combine opnd \"");
    opnd_disassemble(dc, new_opnd, dr_get_logfile(dc));
    dr_log(dc, LOG_CLEANCALL, 3, "\" into \"");
    opnd_disassemble(dc, old_opnd, dr_get_logfile(dc));
    dr_log(dc, LOG_CLEANCALL, 3, "\"\n");
}

static opnd_t
rewrite_reg_in_opnd(void *dc, reg_id_t reg, opnd_t new_opnd, opnd_t old_opnd)
{
    rewrite_opnd_kind_t new_kind = rewrite_opnd_kind(new_opnd);
    rewrite_opnd_kind_t old_kind = rewrite_opnd_kind(old_opnd);
    rewrite_opnd_fn_t rewrite_fn = rewrite_table[new_kind][old_kind];

    if (rewrite_fn == NULL) {
        log_unable_combine_opnds(dc, new_opnd, old_opnd);
        return opnd_create_null();
    } else {
        opnd_t combined_opnd = rewrite_fn(dc, reg, new_opnd, old_opnd);
        if (opnd_is_null(combined_opnd)) {
            log_unable_combine_opnds(dc, new_opnd, old_opnd);
        } else {
            dr_log(dc, LOG_CLEANCALL, 3,
                   "drcalls: combined opnds \"");
            opnd_disassemble(dc, new_opnd, dr_get_logfile(dc));
            dr_log(dc, LOG_CLEANCALL, 3, "\" and \"");
            opnd_disassemble(dc, old_opnd, dr_get_logfile(dc));
            dr_log(dc, LOG_CLEANCALL, 3, "\" into \"");
            opnd_disassemble(dc, combined_opnd, dr_get_logfile(dc));
            dr_log(dc, LOG_CLEANCALL, 3, "\"\n");
        }
        return combined_opnd;
    }
}

static bool
rewrite_reg_in_instr(void *dc, instr_t *instr, reg_id_t reg, opnd_t new_opnd,
                     bool check_only)
{
    opnd_t combined_opnd;
    bool failed = false;
    int i;

    /* Avoid side-effects on original instruction if we're just checking. */
    if (check_only)
        instr = instr_clone(dc, instr);

    /* Iterate all opnds that read reg, and rewrite the ones that do. */
    for (i = 0; i < instr_num_srcs(instr); i++) {
        opnd_t old_opnd = instr_get_src(instr, i);
        if (!opnd_uses_reg(old_opnd, reg))
            continue;
        combined_opnd = rewrite_reg_in_opnd(dc, reg, new_opnd, old_opnd);
        if (opnd_is_null(combined_opnd)) {
            failed = true;
            break;
        }
        instr_set_src(instr, i, combined_opnd);
    }

    for (i = 0; i < instr_num_dsts(instr); i++) {
        opnd_t old_opnd = instr_get_dst(instr, i);
        /* Only base-disp opnds read regs. */
        if (!opnd_is_base_disp(old_opnd))
            continue;
        if (!opnd_uses_reg(old_opnd, reg))
            continue;
        combined_opnd = rewrite_reg_in_opnd(dc, reg, new_opnd, old_opnd);
        if (opnd_is_null(combined_opnd)) {
            failed = true;
            break;
        }
        instr_set_dst(instr, i, combined_opnd);
    }

    /* Massage operands to instructions so they can encode. */
    if (instr_get_opcode(instr) == OP_test) {
        opnd_t opnd0 = instr_get_src(instr, 0);
        opnd_t opnd1 = instr_get_src(instr, 1);
        if (!opnd_is_reg(opnd0) && opnd_is_reg(opnd1)) {
            /* test is commutative, flip so it can encode. */
            instr_set_src(instr, 0, opnd1);
            instr_set_src(instr, 1, opnd0);
        }
    }

    failed |= !can_encode(instr);

    if (check_only)
        instr_destroy(dc, instr);

    return !failed;
}

static bool
rewrite_reg_in_dst(void *dc, instr_t *instr, reg_id_t reg, opnd_t new_opnd,
                   bool check_only)
{
    opnd_t dst_opnd = instr_get_dst(instr, 0);
    bool success;

    if (check_only)
        instr = instr_clone(dc, instr);

    dst_opnd = rewrite_reg_in_opnd(dc, reg, new_opnd, dst_opnd);
    instr_set_dst(instr, 0, dst_opnd);

    success = can_encode(instr);

    if (check_only)
        instr_destroy(dc, instr);

    return success;
}

static bool
rewrite_live_range_check(void *dc, instr_t *def, instr_t *end, reg_id_t reg,
                         opnd_t opnd, bool check_only)
{
    instr_t *instr;

    /* When rewriting live ranges with other registers, we usually want to
     * rewrite the def instr, too, like in reuse_registers.  When folding
     * immediates, it will fail, so we don't. */
    if (opnd_is_reg(opnd)) {
        if (!rewrite_reg_in_dst(dc, def, reg, opnd, check_only))
            return false;
    }

    for (instr = instr_get_next(def); instr != end;
         instr = instr_get_next(instr)) {
        if (opnd_is_base_disp(opnd) && instr_get_next(instr) != end) {
            /* If the opnd we're folding is a memref and this isn't the last
             * instruction in the live range, make sure none of the registers it
             * uses have been clobbered.
             */
            int i;
            for (i = 0; i < opnd_num_regs_used(opnd); i++) {
                reg_id_t reg = opnd_get_reg_used(opnd, i);
                if (instr_writes_to_reg(instr, reg)) {
                    return false;
                }
            }
        }
        if (!rewrite_reg_in_instr(dc, instr, reg, opnd, check_only)) {
            return false;
        }
    }
    return true;
}

/* Find the instruction after the next full clobber of 'reg', if it exists.  If
 * there is a subregister write that does not fully clobber the register, return
 * false.  Considers application instructions to clobber all regs.
 */
static bool
find_next_full_clobber(void *dc, instr_t *start, reg_id_t reg, instr_t **end)
{
    instr_t *instr;
    int i;

    /* Find the next write of dst.  If it's a complicated subregister write, we
     * leave it alone.  Otherwise, we can rewrite up until that instruction. */
    for (instr = instr_get_next(start); instr != NULL;
         instr = instr_get_next(instr)) {

        if (instr_is_call(instr)) {
            if (instr_ok_to_mangle(instr)) {
                /* XXX: This is a call out for partial inlining.  Nothing lives
                 * in or out of this. */
                continue;
            } else {
                /* Everything is live-in to a *meta* call, since that means
                 * we've materialized args for it and we're optimizing the
                 * entire bb.
                 */
                *end = NULL;
                return false;
            }
        }

        /* Nothing is live across application instrs. */
        if (instr_ok_to_mangle(instr)) {
            dr_log(dc, LOG_CLEANCALL, 3,
                   "drcalls: live range ends at app instr\n");
            *end = instr;
            return true;
        }

        if (!instr_writes_to_reg(instr, reg))
            continue;

        for (i = 0; i < instr_num_dsts(instr); i++) {
            opnd_t d = instr_get_dst(instr, i);
            if (opnd_is_reg(d) && reg_overlap(opnd_get_reg(d), reg)) {
                opnd_size_t sz = opnd_get_size(d);
                if (sz == OPSZ_4 || sz == OPSZ_8) {
                    /* dst is fully clobbered here. */
                    *end = instr_get_next(instr);
                    return true;
                } else {
                    /* Subregister write, clobber is not full. */
                    *end = NULL;
                    return false;
                }
            }
        }
    }
    if (instr == NULL)
        *end = NULL;
    return true;
}

static bool
try_rewrite_live_range(void *dc, instr_t *def, opnd_t opnd)
{
    instr_t *end = NULL;
    reg_id_t reg;
    DEBUG_DECLARE(bool ok);

    ASSERT(instr_num_dsts(def) == 1);

    reg = opnd_get_reg(instr_get_dst(def, 0));
    if (!find_next_full_clobber(dc, def, reg, &end)) {
        dr_log(dc, LOG_CLEANCALL, 3,
               "drcalls: unable to determine live range\n");
        return false;
    }

    if (!rewrite_live_range_check(dc, def, end, reg, opnd, true))
        return false;
    IF_DEBUG(ok =) rewrite_live_range_check(dc, def, end, reg, opnd, false);
    ASSERT(ok);
    return true;
}

/*****************************************************************************/
/* Optimizations. */

/* Propagate copies from dead source registers to reduce register pressure.
 * Removes and destroys 'copy' and returns true if successful, otherwise
 * returns false.
 */
static bool
propagate_copy(void *dc, instrlist_t *ilist, instr_t *copy)
{
    opnd_t src = instr_get_src(copy, 0);
    opnd_size_t copy_sz = opnd_get_size(src);

    /* Let's not get into copy sizes that don't clobber the entire GPR. */
    if (!(copy_sz == OPSZ_4 || copy_sz == OPSZ_8))
        return false;

    dr_log(dc, LOG_CLEANCALL, 3,
           "drcalls: propagating copy at "PFX"\n", instr_get_app_pc(copy));

    if (!try_rewrite_live_range(dc, copy, src))
        return false;

    /* Delete the copy. */
    remove_and_destroy(GLOBAL_DCONTEXT, ilist, copy);

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
    if (try_rewrite_live_range(dc, reuse_instr, opnd_create_reg(new))) {
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
    ptr_int_t val = opnd_get_immed_int(imm);

    /* For now we only deal with 32-bit immediates, since pointer values are
     * trickier to rewrite. */
    /* TODO(rnk): Worry about overflow when combining with other displacements.
     */
#ifdef X64
    if (val < INT_MIN || val > INT_MAX)
        return false;
#endif

    dr_log(dc, LOG_CLEANCALL, 3,
           "drcalls: attempting to fold immediate: ");
    instr_disassemble(dc, mov_imm, dr_get_logfile(dc));
    dr_log(dc, LOG_CLEANCALL, 3, "\n");
    if (try_rewrite_live_range(dc, mov_imm, imm)) {
        dr_log(dc, LOG_CLEANCALL, 3,
               "drcalls: folded mov_imm at "PFX"\n",
               instr_get_app_pc(mov_imm));
        return true;
    } else {
        dr_log(dc, LOG_CLEANCALL, 3,
               "drcalls: failed fold mov_imm\n",
               instr_get_app_pc(mov_imm));
        return false;
    }
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
        if (!is_live || instr_is_nop(instr)) {
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

#ifdef X64
static opnd_t
combine_riprel_opnds(void *dc, reg_id_t reg, opnd_t new_opnd, opnd_t old_opnd)
{
    ptr_int_t new_ptr = (ptr_int_t)opnd_get_addr(new_opnd);
    ptr_int_t old_ptr = (ptr_int_t)opnd_get_addr(old_opnd);
    opnd_size_t sz = opnd_get_size(old_opnd);
    ptr_int_t diff = old_ptr - new_ptr;
    if (-256 < diff && diff < 256) {
        /* Only fold if the difference isn't likely to make displacements based
         * off of it bigger. */
        return opnd_create_base_disp(reg, DR_REG_NULL, 0, diff, sz);
    }
    return old_opnd;
}

static void
try_merge_riprel(void *dc, instr_t *lea, opnd_t memref)
{
    instr_t *instr;
    instr_t *end = NULL;
    reg_id_t reg = opnd_get_reg(instr_get_dst(lea, 0));
    int i;

    find_next_full_clobber(dc, lea, reg, &end);
    for (instr = instr_get_next(lea); instr != end;
         instr = instr_get_next(instr)) {
        for (i = 0; i < instr_num_srcs(instr); i++) {
            opnd_t opnd = instr_get_src(instr, i);
            if (opnd_is_rel_addr(opnd)) {
                opnd_t after_opnd = combine_riprel_opnds(dc, reg, memref, opnd);
                instr_set_src(instr, i, after_opnd);
            }
        }
        for (i = 0; i < instr_num_dsts(instr); i++) {
            opnd_t opnd = instr_get_dst(instr, i);
            if (opnd_is_rel_addr(opnd)) {
                opnd_t after_opnd = combine_riprel_opnds(dc, reg, memref, opnd);
                instr_set_dst(instr, i, after_opnd);
            }
        }
    }
}
#endif

/* Try to fold "lea memref -> %reg ; use reg in memref" into the use. */
void
fold_leas(void *dc, void *dc_alloc, instrlist_t *ilist)
{
    instr_t *instr;
    instr_t *next_instr;

    for (instr = instrlist_first(ilist); instr != NULL; instr = next_instr) {
        next_instr = instr_get_next(instr);
        /* Don't touch app instrs. */
        if (instr_ok_to_mangle(instr))
            continue;

        if (instr_get_opcode(instr) == OP_lea) {
            opnd_t memref = instr_get_src(instr, 0);
            if (opnd_is_base_disp(memref) &&
                opnd_get_segment(memref) == DR_REG_NULL) {
                dr_log(dc, LOG_CLEANCALL, 3,
                       "drcalls: fold_lea: trying to fold lea: ");
                instr_disassemble(dc, instr, dr_get_logfile(dc));
                dr_log(dc, LOG_CLEANCALL, 3, "\n");
                if (try_rewrite_live_range(dc, instr, memref)) {
                    dr_log(dc, LOG_CLEANCALL, 3,
                           "drcalls: folded lea\n");
                    /* TODO(rnk): We've rewritten all uses of the dst reg in the
                     * ilist, but it could be live-out.  If we choose to handle
                     * this case, we should leave the instr and let DCE handle
                     * that. */
                    remove_and_destroy(dc_alloc, ilist, instr);
                } else {
                    dr_log(dc, LOG_CLEANCALL, 3,
                           "drcalls: failed to fold lea\n");
                }
            }
#ifdef X64
            else if (opnd_is_rel_addr(memref)) {
                try_merge_riprel(dc, instr, memref);
            }
#endif
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
 *
 * Could also match ld ... no st ... ld.
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
#ifdef VERBOSE
            dr_log(dc, LOG_CLEANCALL, 3,
                   "drcalls: RLE: blocked by non-meta instr: ");
            instr_disassemble(dc, instr, dr_get_logfile(dc));
            dr_log(dc, LOG_CLEANCALL, 3, "\n");
#endif
            store = NULL;
            continue;
        }

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
            continue;
        }
        if (store == NULL)
            continue;

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
#ifdef VERBOSE
            dr_log(dc, LOG_CLEANCALL, 3,
                   "drcalls: RLE: blocked by instr: ");
            instr_disassemble(dc, instr, dr_get_logfile(dc));
            dr_log(dc, LOG_CLEANCALL, 3, "\n");
#endif
            store = NULL;
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
            if (store != NULL && opnd_same(mem_ref, instr_get_dst(instr, 0))) {
                /* Can delete store. */
                dr_log(dc, LOG_CLEANCALL, 3,
                       "drcalls: DSE: deleted dead store: ");
                instr_disassemble(dc, store, dr_get_logfile(dc));
                dr_log(dc, LOG_CLEANCALL, 3, "\n");

                remove_and_destroy(dc_alloc, ilist, store);
            }

            /* Whether we deleted the last store or not, this is our new store
             * to try to delete. */
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
