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

/* Unit tests for optimize.c. */

#include "dr_api.h"
#include "dr_calls.h"

#include "callee.h"
#include "instr_builder.h"
#include "optimize.h"

#include <string.h>

#ifdef WINDOWS
# define EXPORT __declspec(dllexport)
# define NOINLINE __declspec(noinline)
#else
# define EXPORT __attribute__((visibility("default")))
# define NOINLINE __attribute__((noinline))
#endif

/******************************************************************************/
/* Sample application bb */

static app_pc app_bb_tag = (app_pc)0x07f0000;
static instrlist_t *app_bb;

static void
codegen_app_bb(void *dc)
{
    instr_builder_t ib;
    INSTR_BUILDER_INIT(ib, dc, instrlist_create(dc), NULL, /*meta=*/false);

    BUILD(ib, mov_ld, REG(XAX),                         MEM_IDX(XSP, XBX, 4, 0x10, PTR));
    BUILD(ib, mov_st, MEM_IDX(XDI, XCX, 4, 0x10, PTR),  REG(XAX));
    BUILD(ib, jmp,    opnd_create_pc(app_bb_tag));

    app_bb = ib.ilist;
}

/******************************************************************************/
/* Listing utilities */

static void
my_disas(void *dc, instrlist_t *ilist, file_t outfile)
{
    instr_t *instr;
    for (instr = instrlist_first(ilist); instr; instr = instr_get_next(instr)) {
        instr_disassemble(dc, instr, outfile);
        dr_fprintf(outfile, "\n");
    }
}

#define PATH_MAX 1024

void
make_listing(void *dc, const char *name, instrlist_t *ilist)
{
    char fname[PATH_MAX];
    dr_snprintf(fname, PATH_MAX, "./asm_listings/%s.asm", name);
    file_t f = dr_open_file(fname, DR_FILE_WRITE_OVERWRITE);
    ASSERT(f != INVALID_FILE);
    my_disas(dc, ilist, f);
    dr_close_file(f);
}

/******************************************************************************/
/* Inscount code. */

static uint64 global_count = 0;

//static void
//__attribute__((optimize(2)))
//__attribute__((optimize("-fomit-frame-pointer")))
//inc_count(void)
//{
    //global_count++;
//}

static void (*inc_count)(void);

static byte inc_count_buf[128];

static dr_emit_flags_t
inscount_event_bb(void *drcontext, void *tag, instrlist_t *bb,
                  bool for_trace, bool translating)
{
    instr_t *instr;
    for (instr = instrlist_first(bb); instr != NULL;
         instr = instr_get_next(instr)) {
        drcalls_insert_call(drcontext, bb, instr, (void *)inc_count,
                            false /* save fpstate */, 0);
    }

    drcalls_done(drcontext, bb);

    return DR_EMIT_DEFAULT;
}

static void
codegen_inc_count(void *dc)
{
    instr_builder_t ib;
    INSTR_BUILDER_INIT(ib, dc, instrlist_create(dc), NULL, /*meta=*/false);

    BUILD(ib, push,   REG(XBP));
    BUILD(ib, mov_ld, REG(XBP),                     REG(XSP));
    BUILD(ib, mov_ld, REG(XAX),                     MEM_REL(&global_count, 8));
    BUILD(ib, add,    REG(XAX),                     IMM(1, 4));
    BUILD(ib, mov_st, MEM_REL(&global_count, 8),    REG(XAX));
    INSERT(ib, INSTR_CREATE_leave(dc));
    INSERT(ib, INSTR_CREATE_ret(dc));

    instrlist_encode(dc, ib.ilist, &inc_count_buf[0], false);
    inc_count = (void (*)(void)) &inc_count_buf[0];
    instrlist_clear_and_destroy(dc, ib.ilist);
}

void
inscount_listings(void *dc)
{
    codegen_app_bb(dc);
    codegen_inc_count(dc);

    instrlist_t *bb;

    dr_printf("\nCLEAN CALL LISTING:\n");
    drcalls_init();
    bb = instrlist_clone(dc, app_bb);
    drcalls_set_optimization(0);
    inscount_event_bb(dc, app_bb_tag, bb, false, false);
    drcalls_done(dc, bb);
    my_disas(dc, bb, STDOUT);
    instrlist_clear_and_destroy(dc, bb);
    drcalls_exit();

    dr_printf("\ninc_count disas:\n");
    bb = decode_as_bb(dc, (byte*)inc_count);
    my_disas(dc, bb, STDOUT);
    instrlist_clear_and_destroy(dc, bb);

    dr_printf("\nopt2:\n");
    drcalls_init();
    bb = instrlist_clone(dc, app_bb);
    drcalls_set_optimization(2);
    inscount_event_bb(dc, app_bb_tag, bb, false, false);
    drcalls_done(dc, bb);
    my_disas(dc, bb, STDOUT);
    instrlist_clear_and_destroy(dc, bb);
    drcalls_exit();

    dr_printf("\nopt3:\n");
    drcalls_init();
    bb = instrlist_clone(dc, app_bb);
    drcalls_set_optimization(3);
    inscount_event_bb(dc, app_bb_tag, bb, false, false);
    drcalls_done(dc, bb);
    my_disas(dc, bb, STDOUT);
    instrlist_clear_and_destroy(dc, bb);
    drcalls_exit();

    dr_printf("\nopt3_tls:\n");
    drcalls_init();
    bb = instrlist_clone(dc, app_bb);
    drcalls_set_optimization(3);
    drcalls_set_use_tls_inline(true);
    inscount_event_bb(dc, app_bb_tag, bb, false, false);
    drcalls_done(dc, bb);
    my_disas(dc, bb, STDOUT);
    instrlist_clear_and_destroy(dc, bb);
    drcalls_exit();

    dr_printf("\nopt3_tls_rle:\n");
    drcalls_init();
    bb = instrlist_clone(dc, app_bb);
    drcalls_set_optimization(3);
    drcalls_set_use_tls_inline(true);
    inscount_event_bb(dc, app_bb_tag, bb, false, false);
    drcalls_done(dc, bb);
    redundant_load_elim(dc, dc, bb);
    my_disas(dc, bb, STDOUT);
    instrlist_clear_and_destroy(dc, bb);
    drcalls_exit();

    dr_printf("\nopt3_rle_dse:\n");
    drcalls_init();
    bb = instrlist_clone(dc, app_bb);
    drcalls_set_optimization(3);
    drcalls_set_use_tls_inline(true);
    inscount_event_bb(dc, app_bb_tag, bb, false, false);
    drcalls_done(dc, bb);
    redundant_load_elim(dc, dc, bb);
    dead_store_elim(dc, dc, bb);
    my_disas(dc, bb, STDOUT);
    instrlist_clear_and_destroy(dc, bb);
    drcalls_exit();

    dr_printf("\nopt4:\n");
    drcalls_init();
    bb = instrlist_clone(dc, app_bb);
    drcalls_set_optimization(4);
    drcalls_set_use_tls_inline(true);
    inscount_event_bb(dc, app_bb_tag, bb, false, false);
    drcalls_done(dc, bb);
    my_disas(dc, bb, STDOUT);
    instrlist_clear_and_destroy(dc, bb);
    drcalls_exit();

    dr_printf("\nopt5:\n");
    drcalls_init();
    bb = instrlist_clone(dc, app_bb);
    drcalls_set_optimization(5);
    inscount_event_bb(dc, app_bb_tag, bb, false, false);
    drcalls_done(dc, bb);
    my_disas(dc, bb, STDOUT);
    instrlist_clear_and_destroy(dc, bb);
    drcalls_exit();

    instrlist_clear_and_destroy(dc, app_bb);
}

/* Alignment instrumentation. */
void
//__attribute__((visibility("default")))
//__attribute__((optimize(2)))
//__attribute__((optimize("-fomit-frame-pointer")))
check_access(ptr_uint_t ea, app_pc pc, uint size, bool write)
{
    /* Check alignment.  Assumes size is a power of two. */
    if ((ea & (size - 1)) == 0) {
        return;
    }

    dr_fprintf(STDOUT,
               "Unaligned %s access to ea "PFX" at pc "PFX" of size %d\n",
               (write ? "write" : "read"), ea, pc, size);
}

/* Memtrace instrumentation. */
typedef struct _memop_t {
    ptr_uint_t ea;
    ptr_uint_t pc;
    uint size;
    uint write;
} memop_t;

#define BUFFER_SIZE 1024
static uint pos;
static memop_t buffer[BUFFER_SIZE];

NOINLINE static void
flush_buffer(void)
{
    int i;

    for (i = 0; i < pos; i++) {
        check_access(buffer[i].ea,
                     (app_pc)buffer[i].pc,
                     buffer[i].size,
                     buffer[i].write);
    }
    pos = 0;
}

void
//__attribute__((visibility("default")))
//__attribute__((optimize(2)))
//__attribute__((optimize("-fomit-frame-pointer")))
buffer_memop(ptr_uint_t ea, ptr_uint_t pc, uint size, bool write)
{
    buffer[pos].ea = ea;
    buffer[pos].pc = pc;
    buffer[pos].size = size;
    buffer[pos].write = write;
    pos++;
    if (pos >= BUFFER_SIZE) {
        flush_buffer();
    }
}

static void
instrument_mem(void *dc, instrlist_t *ilist, instr_t *where, int pos,
               bool write)
{
    opnd_t memop = (write ? instr_get_dst(where, pos)
                          : instr_get_src(where, pos));
    uint opsize = opnd_size_in_bytes(opnd_get_size(memop));
    opnd_set_size(&memop, OPSZ_lea);
    drcalls_insert_call(dc, ilist, where, (void*)check_access, false, 4,
                        memop, OPND_CREATE_INTPTR(instr_get_app_pc(where)),
                        OPND_CREATE_INT32(opsize), OPND_CREATE_INT32(write));
}

static dr_emit_flags_t
alignment_event_bb(void *dc, void *entry_pc, instrlist_t *bb, bool for_trace,
                   bool translating)
{
    instr_t *instr;
    int i;

    for (instr = instrlist_first(bb); instr != NULL;
         instr = instr_get_next(instr)) {
        /* Some nop instructions have memory operands as a way of varying the
         * operation size.  We don't want those. */
        if (instr_is_nop(instr) || instr_get_opcode(instr) == OP_nop_modrm)
            continue;
        if (instr_reads_memory(instr))
            for (i = 0; i < instr_num_srcs(instr); i++)
                if (opnd_is_memory_reference(instr_get_src(instr, i)))
                    instrument_mem(dc, bb, instr, i, false);
        if (instr_writes_memory(instr))
            for (i = 0; i < instr_num_dsts(instr); i++)
                if (opnd_is_memory_reference(instr_get_dst(instr, i)))
                    instrument_mem(dc, bb, instr, i, true);
    }

    drcalls_done(dc, bb);

    return DR_EMIT_DEFAULT;
}

void
alignment_listings(void *dc)
{
    instrlist_t *bb;
    callee_info_t *ci;

    codegen_app_bb(dc);

    dr_printf("\ncheck_access disas:\n");
    ci = callee_info_create((app_pc)check_access, 4);
    decode_callee_ilist(dc, ci);
    //my_disas(dc, ci->ilist, STDOUT);
    instrlist_disassemble(dc, (app_pc)check_access, ci->ilist, STDOUT);
    callee_info_free(ci);

    dr_printf("\nbuffer_memop disas:\n");
    ci = callee_info_create((app_pc)buffer_memop, 4);
    decode_callee_ilist(dc, ci);
    //my_disas(dc, ci->ilist, STDOUT);
    instrlist_disassemble(dc, (app_pc)buffer_memop, ci->ilist, STDOUT);
    callee_info_free(ci);

    dr_printf("\nCLEAN CALL LISTING:\n");
    drcalls_init();
    bb = instrlist_clone(dc, app_bb);
    drcalls_set_optimization(0);
    alignment_event_bb(dc, app_bb_tag, bb, false, false);
    drcalls_done(dc, bb);
    my_disas(dc, bb, STDOUT);
    instrlist_clear_and_destroy(dc, bb);
    drcalls_exit();

    instrlist_clear_and_destroy(dc, app_bb);
}

int
main(void)
{
    dr_app_setup();

    void *dc = dr_get_current_drcontext();

    //inscount_listings(dc);
    alignment_listings(dc);

    dr_app_cleanup();
    return 0;
}
