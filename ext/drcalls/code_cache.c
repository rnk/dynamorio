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

/* drcalls code cache.  All cached code lives for the lifetime of the process,
 * so only small amounts of code should live here. */

#include "dr_api.h"

/* internal includes */
#include "core_compat.h"
#include "code_cache.h"

#define CACHE_BLOCK_SIZE PAGE_SIZE

/* The memory protections we use for the code cache. */
#define CODE_RWX (DR_MEMPROT_READ|DR_MEMPROT_WRITE|DR_MEMPROT_EXEC)
#define CODE_RX  (DR_MEMPROT_READ|DR_MEMPROT_EXEC)

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
    void *lock;
} code_cache_t;

static code_cache_t *code_cache;

void
code_cache_init(void)
{
    code_cache = dr_global_alloc(sizeof(code_cache_t));
    code_cache->root = NULL;
    code_cache->cur_block = NULL;
    code_cache->cur_pc = NULL;
    code_cache->lock = dr_mutex_create();
}

void
code_cache_destroy(void)
{
    code_cache_block_t *block;
    code_cache_block_t *block_next;

    /* Free the linked list of code cache blocks. */
    for (block = code_cache->root; block != NULL; block = block_next) {
        block_next = block->next;
        dr_nonheap_free(block, CACHE_BLOCK_SIZE);
    }

    dr_mutex_destroy(code_cache->lock);
    dr_global_free(code_cache, sizeof(code_cache_t));
    code_cache = NULL;
}

/* Adjust the permissions on a code cache block to prot. */
static void
code_cache_memprotect(code_cache_block_t *block, uint prot)
{
    bool ok;
    ok = dr_memory_protect(block, CACHE_BLOCK_SIZE, prot);
    DR_ASSERT_MSG(ok, "Changing code cache protection bits failed");
}

/* Add a block to the linked list of blocks in our code cache and update the
 * current pc and block. */
static void
code_cache_grow(void *dc, code_cache_t *code_cache)
{
    code_cache_block_t *prev_block;
    code_cache_block_t *new_block;

    /* TODO(rnk): Assert lock held. */

    dr_log(dc, LOG_CACHE, 3,
           "drcalls: growing shared clean call code cache\n");

    new_block = (code_cache_block_t *)dr_nonheap_alloc(
        CACHE_BLOCK_SIZE, CODE_RWX);
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

static byte *
align_to_cacheline(byte *pc)
{
    return (byte *)ALIGN_FORWARD(pc, proc_get_cache_line_size());
}

app_pc
code_cache_emit(void *dc, instrlist_t *ilist)
{
    app_pc entry;
    ptr_uint_t offset;
    instr_t *instr;

    dr_mutex_lock(code_cache->lock);

    /* Compute size of each instr and set each note with the offset.  This
     * avoids a second pass in encoding. */
    offset = 0;
    for (instr = instrlist_first(ilist); instr != NULL;
         instr = instr_get_next(instr)) {
        instr_set_note(instr, (void *)offset);
        offset += instr_length(dc, instr);
    }

    /* Check that there's space to encode the ilist.  Clean calls on x86_64 use
     * about 469 bytes, so we can usually pack about 8 calls per page. */
    entry = align_to_cacheline(code_cache->cur_pc);
    if (code_cache->cur_block == NULL ||
        (entry + offset > (byte*)code_cache->cur_block + CACHE_BLOCK_SIZE)) {
        code_cache_grow(dc, code_cache);
        entry = align_to_cacheline(code_cache->cur_pc);
        /* TODO(rnk): Make this not assert.  Should alloc enough space. */
        DR_ASSERT_MSG((entry + offset <=
                       (byte*)code_cache->cur_block + CACHE_BLOCK_SIZE),
                      "Clean call did not fit in single code cache block!");
    }

    /* Unprotect the page, encode the instructions, and reprotect it. */
    code_cache_memprotect(code_cache->cur_block, CODE_RWX);
    code_cache->cur_pc = instrlist_encode(dc, ilist, entry, false);
    code_cache_memprotect(code_cache->cur_block, CODE_RX);

    dr_mutex_unlock(code_cache->lock);

    dr_log(dc, LOG_CACHE, 3, "drcalls: cached ilist:\n");
    DOLOG(3, LOG_CLEANCALL, {
        instrlist_disassemble(dc, entry, ilist, dr_get_logfile(dc));
    });
    return entry;
}
