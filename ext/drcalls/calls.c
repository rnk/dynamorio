/* *******************************************************************************
 * Copyright (c) 2010 Massachusetts Institute of Technology  All rights reserved.
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
 * ARE DISCLAIMED. IN NO EVENT SHALL VMWARE, INC. OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#include "dr_api.h"
#include "dr_calls.h"
#include "hashtable.h"
#include <stdarg.h> /* for varargs */

/* Standard alignment hack. */
#define ALIGN_FORWARD(x, alignment) \
    ((((ptr_uint_t)x) + ((alignment)-1)) & (~((alignment)-1)))

/* Make code more readable by shortening long lines.  We mark all as meta to
 * avoid client interface asserts.
 */
#define POST instrlist_meta_postinsert
#define PRE  instrlist_meta_preinsert
#define APP  instrlist_meta_append

#define CODE_CACHE_BLOCK_SIZE PAGE_SIZE

/* The memory protections we use for the code cache. */
#define CODE_RWX (DR_MEMPROT_READ|DR_MEMPROT_WRITE|DR_MEMPROT_EXEC)
#define CODE_RX  (DR_MEMPROT_READ|DR_MEMPROT_EXEC)

/* Maps callee addresses to shared clean call entry points. */
#define ENTRY_TABLE_BITS 6

/* Linked list node present at the start of every block of code cache memory.
 */
typedef struct _code_cache_block_t code_cache_block_t;
struct _code_cache_block_t {
    code_cache_block_t *next;
};

/* Code cache data. */
/* TODO(rnk): For now we assume that there will be roughly a constant number of
 * callees, so the clean call context switch sequences live forever.  It may be
 * worth revisiting this later if the role of this code cache expands. */
typedef struct _code_cache_t {
    code_cache_block_t *root;
    code_cache_block_t *cur_block;
    byte *cur_pc;
    hashtable_t entry_point_table;
    void *lock;
} code_cache_t;

static code_cache_t *code_cache;

/* Returns an upper bound on the encoded length of the instructions in bytes.
 */
static uint
instrlist_length(void *drcontext, instrlist_t *ilist)
{
    instr_t *inst;
    uint len = 0;
    for (inst = instrlist_first(ilist); inst; inst = instr_get_next(inst)) {
        len += instr_length(drcontext, inst);
    }
    return len;
}

static byte *
align_to_cacheline(byte *pc)
{
    return (byte *)ALIGN_FORWARD(pc, proc_get_cache_line_size());
}

/* Adjust the permissions on a code cache block to prot. */
static void
code_cache_memprotect(code_cache_block_t *block, uint prot)
{
    bool ok;
    ok = dr_memory_protect(block, CODE_CACHE_BLOCK_SIZE, prot);
    DR_ASSERT_MSG(ok, "Changing code cache protection bits failed");
}

/* Add a block to the linked list of blocks in our code cache and update the
 * current pc and block. */
static void
code_cache_grow(void *drcontext, code_cache_t *code_cache)
{
    code_cache_block_t *prev_block;
    code_cache_block_t *new_block;

    dr_log(drcontext, LOG_CACHE, 3,
           "drcalls: growing shared clean call code cache\n");

    new_block = (code_cache_block_t *)dr_nonheap_alloc(
        CODE_CACHE_BLOCK_SIZE, CODE_RWX);
    new_block->next = NULL;

    prev_block = code_cache->cur_block;
    code_cache->cur_block = new_block;

    /* If this was the first block we allocated, set the root to it.
     * Otherwise, link it with the previous block. */
    if (prev_block == NULL) {
        code_cache->root = new_block;
    } else {
        code_cache_memprotect(prev_block, CODE_RWX);
        prev_block->next = new_block;
        code_cache_memprotect(prev_block, CODE_RX);
    }
    code_cache->cur_pc = (byte *)(new_block + 1);
}

/* Emit the shared clean call code into the code cache and return the entry
 * point.  Grows the code cache as necessary.  Assumes that the code cache lock
 * is held. */
static byte *
emit_shared_call(void *drcontext, void *callee, uint num_args)
{
    byte *entry;
    instrlist_t *ilist;
    opnd_t args[2];

    dr_log(drcontext, LOG_CACHE, 3,
           "drcalls: emitting new shared clean call\n");

    /* Generate the clean call ilist. */
    ilist = instrlist_create(drcontext);
    switch (num_args) {
    default:
        DR_ASSERT_MSG(false, "Cannot do shared clean call with >= 2 args");
        return NULL;
    case 2:
        args[1] = dr_reg_spill_slot_opnd(drcontext, SPILL_SLOT_3);
        /* FALLTHROUGH */
    case 1:
        args[0] = dr_reg_spill_slot_opnd(drcontext, SPILL_SLOT_2);
        /* FALLTHROUGH */
    case 0:
        break;
    }
    dr_insert_clean_call_vargs(drcontext, ilist, NULL, callee, true, num_args,
                               &args[0]);

    /* Clean call return. */
    APP(ilist, INSTR_CREATE_jmp_ind(drcontext,
                                    dr_reg_spill_slot_opnd(drcontext,
                                                           SPILL_SLOT_1)));

    /* Check that there's space to encode the ilist.  Clean calls on x86_64 use
     * about 469 bytes, so we can usually pack about 8 calls per page. */
    entry = align_to_cacheline(code_cache->cur_pc);
    if (code_cache->cur_block == NULL ||
        (entry + instrlist_length(drcontext, ilist) >
         (byte*)code_cache->cur_block + CODE_CACHE_BLOCK_SIZE)) {
        code_cache_grow(drcontext, code_cache);
        entry = align_to_cacheline(code_cache->cur_pc);
        DR_ASSERT_MSG(entry + instrlist_length(drcontext, ilist) <=
                      (byte*)code_cache->cur_block + CODE_CACHE_BLOCK_SIZE,
                      "Clean call did not fit in single code cache block!");
    }

    /* Unprotect the page, encode the instructions, and reprotect it. */
    code_cache_memprotect(code_cache->cur_block, CODE_RWX);
    code_cache->cur_pc = instrlist_encode(drcontext, ilist, entry, false);
    code_cache_memprotect(code_cache->cur_block, CODE_RX);

    instrlist_clear_and_destroy(drcontext, ilist);

    return entry;
}

void
drcalls_init(void)
{
    code_cache = dr_global_alloc(sizeof(code_cache_t));
    code_cache->root = NULL;
    code_cache->cur_block = NULL;
    code_cache->cur_pc = NULL;
    hashtable_init(&code_cache->entry_point_table, ENTRY_TABLE_BITS,
                   HASH_INTPTR, false);
    code_cache->lock = dr_mutex_create();
}

void
drcalls_exit(void)
{
    code_cache_block_t *block;
    code_cache_block_t *block_next;

    /* Hashtable points into code cache, which we free below, so we don't need
     * to free the elements. */
    hashtable_delete(&code_cache->entry_point_table);

    /* Free the linked list of code cache blocks. */
    for (block = code_cache->root; block != NULL; block = block_next) {
        block_next = block->next;
        dr_nonheap_free(block, CODE_CACHE_BLOCK_SIZE);
    }

    dr_mutex_destroy(code_cache->lock);
    dr_global_free(code_cache, sizeof(code_cache_t));
    code_cache = NULL;
}

static void
convert_va_list_to_opnd(opnd_t *args, uint num_args, va_list ap)
{
    uint i;
    /* There's no way to check num_args vs actual args passed in, or publicly
     * check opnd_t for validity. */
    for (i = 0; i < num_args; i++) {
        args[i] = va_arg(ap, opnd_t);
    }
}

static void
materialize_args(void *drcontext, instrlist_t *ilist, instr_t *where,
                 uint num_args, opnd_t *args)
{
    int i;
    opnd_t xax_opnd;

    dr_save_reg(drcontext, ilist, where, DR_REG_XAX, SPILL_SLOT_1);
    xax_opnd = opnd_create_reg(DR_REG_XAX);

    for (i = 0; i < num_args; i++) {
        opnd_t arg = args[i];
        opnd_t arg_spill = dr_reg_spill_slot_opnd(drcontext, SPILL_SLOT_2 + i);
        /* Materialize the arg operand in XAX. */
        if (opnd_is_immed_int(arg)) {
            PRE(ilist, where,
                INSTR_CREATE_mov_imm(drcontext, xax_opnd, arg));
        } else if (opnd_is_memory_reference(arg) || opnd_is_reg(arg)) {
            PRE(ilist, where,
                INSTR_CREATE_mov_ld(drcontext, xax_opnd, arg));
        } else {
            DR_ASSERT_MSG(false, "Unsupported operand type!");
        }
        /* Store XAX to arg spill slot. */
        PRE(ilist, where,
            INSTR_CREATE_mov_st(drcontext, arg_spill, xax_opnd));
    }

    dr_restore_reg(drcontext, ilist, where, DR_REG_XAX, SPILL_SLOT_1);
}

void
drcalls_shared_call(void *drcontext, instrlist_t *ilist, instr_t *where,
                    void *callee, uint num_args, ...)
{
    va_list ap;
    instr_t *return_label;
    byte *shared_entry;
    opnd_t stack_args[2];

    /* If there are more args than TLS spill slots, give up and insert a normal
     * clean call. */
    if (num_args > 2) {
        opnd_t *args;
        size_t arg_alloc_size = sizeof(opnd_t) * num_args;
        args = dr_thread_alloc(drcontext, arg_alloc_size);
        va_start(ap, num_args);
        convert_va_list_to_opnd(args, num_args, ap);
        va_end(ap);

        dr_log(drcontext, LOG_ALL, 3,
               "drcalls: unable to share clean call save/restore code, "
               "performance may suffer\n");
        dr_insert_clean_call_vargs(drcontext, ilist, where, callee, true,
                                   num_args, args);
        dr_thread_free(drcontext, args, arg_alloc_size);
        return;
    }

    /* If we haven't seen this callee, emit the shared clean call entry/exit
     * sequence to the code cache. */
    shared_entry = hashtable_lookup(&code_cache->entry_point_table, callee);
    if (shared_entry == NULL) {
        dr_mutex_lock(code_cache->lock);
        /* Now that we have the lock, make sure no one added the callee while we
         * were waiting. */
        shared_entry = hashtable_lookup(&code_cache->entry_point_table, callee);
        if (shared_entry == NULL) {
            bool success;
            shared_entry = emit_shared_call(drcontext, callee, num_args);
            success = hashtable_add(&code_cache->entry_point_table, callee,
                                    shared_entry);
            DR_ASSERT_MSG(success,
                          "Unable to insert into code cache hashtable");
        }
        dr_mutex_unlock(code_cache->lock);
    }

    /* Store the arguments in spill slots.  We materialize the opnd_t values
     * into XAX and then save XAX to the appropriate spill slot. */
    va_start(ap, num_args);
    convert_va_list_to_opnd(&stack_args[0], num_args, ap);
    va_end(ap);
    materialize_args(drcontext, ilist, where, num_args, &stack_args[0]);

    /* Store the return label in a spill slot, and jump to the shared clean
     * call sequence. */
    return_label = INSTR_CREATE_label(drcontext);
    PRE(ilist, where,
        INSTR_CREATE_mov_imm(drcontext,
                             dr_reg_spill_slot_opnd(drcontext, SPILL_SLOT_1),
                             opnd_create_instr(return_label)));
    PRE(ilist, where,
        INSTR_CREATE_jmp(drcontext, opnd_create_pc(shared_entry)));
    PRE(ilist, where, return_label);
}
