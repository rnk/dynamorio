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

/* Lean call support.  Maintains map of lean call callees to register save
 * sequences. */

#include "dr_api.h"
#include "hashtable.h"

#include "code_cache.h"
#include "core_compat.h"

/* Hashtable for lean callees.  Maps callee pointer to lean clean call trace. */
static hashtable_t *lean_callee_table;
#define LEAN_CALLEE_TABLE_BITS 6

void
lean_call_init(void)
{
    lean_callee_table = dr_global_alloc(sizeof(*lean_callee_table));
    hashtable_init_ex(lean_callee_table,
                      LEAN_CALLEE_TABLE_BITS,
                      HASH_INTPTR,
                      /* str_dup */ false,
                      /* sync */ false,
                      /* free */ NULL,
                      /* hash key */ NULL,
                      /* cmp key */ NULL
                      );
}

void
lean_call_exit(void)
{
    hashtable_delete(lean_callee_table);
    dr_global_free(lean_callee_table, sizeof(*lean_callee_table));
}

/* Emit the lean call code into the code cache and return the entry point.
 */
static byte *
emit_lean_call(void *dc, void *callee, bool fpstate, uint num_args)
{
    byte *entry;
    instrlist_t *ilist;
    opnd_t args[2];

    dr_log(dc, LOG_CACHE, 3,
           "drcalls: emitting new lean clean call\n");

    /* Generate the clean call ilist. */
    ilist = instrlist_create(dc);
    /* TODO(rnk): Too clever, just use a for loop. */
    switch (num_args) {
    default:
        DR_ASSERT_MSG(false, "Cannot do lean call with >= 2 args");
        return NULL;
    case 2:
        args[1] = dr_reg_spill_slot_opnd(dc, SPILL_SLOT_3);
        /* FALLTHROUGH */
    case 1:
        args[0] = dr_reg_spill_slot_opnd(dc, SPILL_SLOT_2);
        /* FALLTHROUGH */
    case 0:
        break;
    }
    dr_insert_clean_call_vargs(dc, ilist, NULL, callee, fpstate, num_args,
                               args);

    /* Clean call return. */
    APP(ilist, INSTR_CREATE_jmp_ind
        (dc, dr_reg_spill_slot_opnd(dc, SPILL_SLOT_1)));

    entry = code_cache_emit(dc, ilist);

    instrlist_clear_and_destroy(dc, ilist);

    return entry;
}

static reg_id_t
pick_scratch_reg(uint num_args, opnd_t *args)
{
    reg_id_t scratch_reg_num;
    bool scratch_reg_conflicts = true;
    /* Find a reg that does not conflict with any registers used by argument
     * operands. */
    for (scratch_reg_num = DR_REG_XAX; scratch_reg_conflicts; scratch_reg_num++) {
        uint i;
        scratch_reg_conflicts = false;
        for (i = 0; i < num_args; i++) {
            if (opnd_uses_reg(args[i], scratch_reg_num)) {
                scratch_reg_conflicts = true;
                break;
            }
        }
    }
    return scratch_reg_num;
}

static void
materialize_args(void *dc, instrlist_t *ilist, instr_t *where,
                 uint num_args, opnd_t *args)
{
    uint i;
    reg_id_t scratch_reg_num = DR_REG_NULL;  /* Not null if we needed one. */
    opnd_t scratch_reg = {0,};  /* Silence uninitialized warnings. */
    instr_t *before_args = instr_get_prev(where);

    for (i = 0; i < num_args; i++) {
        opnd_t arg = args[i];
        opnd_t arg_spill = dr_reg_spill_slot_opnd(dc, SPILL_SLOT_2 + i);

        if ((opnd_is_immed_int(arg) && opnd_get_size(arg) <= OPSZ_4) ||
            opnd_is_reg(arg)) {
            /* If the argument is a 32-bit immediate or register, can
             * materialize into argument spill slot without a scratch reg. */
            PRE(ilist, where,
                INSTR_CREATE_mov_st(dc, arg_spill, arg));
        } else if (opnd_is_far_base_disp(arg) &&
                   opnd_get_segment(arg) == opnd_get_segment(arg_spill) &&
                   opnd_get_disp(arg) == opnd_get_disp(arg_spill)) {
            /* The argument already is the appropriate spill slot, so we don't
             * touch it. */
        } else {
            /* Otherwise, we'll need to pick a saved and restored scratch reg
             * if we haven't yet. */
            if (scratch_reg_num == DR_REG_NULL) {
                scratch_reg_num = pick_scratch_reg(num_args, args);
                scratch_reg = opnd_create_reg(scratch_reg_num);
            }

            /* Materialize arg into scratch reg. */
            if (opnd_is_immed_int(arg)) {
                PRE(ilist, where,
                    INSTR_CREATE_mov_imm(dc, scratch_reg, arg));
            } else if (opnd_is_memory_reference(arg)) {
                PRE(ilist, where,
                    INSTR_CREATE_mov_ld(dc, scratch_reg, arg));
            } else {
                DR_ASSERT_MSG(false, "Unsupported operand type!");
            }

            /* Store scratch reg to arg spill slot. */
            PRE(ilist, where,
                INSTR_CREATE_mov_st(dc, arg_spill, scratch_reg));
        }
    }

    if (scratch_reg_num != DR_REG_NULL) {
        /* Save and restore our scratch reg only if we ended up picking one. */
        instr_t *first = (before_args != NULL ?
                          instr_get_next(before_args) :
                          instrlist_first(ilist));
        dr_save_reg(dc, ilist, first, scratch_reg_num, SPILL_SLOT_1);
        dr_restore_reg(dc, ilist, where, scratch_reg_num, SPILL_SLOT_1);
    }
}

void
lean_call_insert(void *dc, instrlist_t *ilist, instr_t *where, void *callee,
                 bool fpstate, uint num_args, opnd_t *args)
{
    instr_t *return_label;
    byte *lean_entry;

    /* If we haven't seen this callee, emit the lean call entry/exit sequence to
     * the code cache. */
    hashtable_lock(lean_callee_table);
    lean_entry = hashtable_lookup(lean_callee_table, callee);
    if (lean_entry == NULL) {
        bool success;
        dr_log(dc, LOG_CACHE, 3, "drcalls: emitting lean call\n");
        lean_entry = emit_lean_call(dc, callee, fpstate, num_args);
        success = hashtable_add(lean_callee_table, callee, lean_entry);
        DR_ASSERT_MSG(success, "Unable to insert into lean callee table");
    }
    hashtable_unlock(lean_callee_table);

    /* Store the arguments in spill slots.  We materialize the opnd_t values
     * into XAX and then save XAX to the appropriate spill slot. */
    materialize_args(dc, ilist, where, num_args, args);

    /* Store the return label in a spill slot, and jump to the lean call
     * sequence. */
    return_label = INSTR_CREATE_label(dc);
    PRE(ilist, where,
        INSTR_CREATE_mov_imm(dc, dr_reg_spill_slot_opnd(dc, SPILL_SLOT_1),
                             opnd_create_instr(return_label)));
    PRE(ilist, where,
        INSTR_CREATE_jmp(dc, opnd_create_pc(lean_entry)));
    PRE(ilist, where, return_label);
}

