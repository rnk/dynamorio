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

#undef ASSERT
#include "unittest.h"

#include <string.h>

/******************************************************************************/
/* Test cases */

TEST(dce_basic) {
    ilist_cmp_test_t t;
    ilist_cmp_test_init(&t);

    t.ib.ilist = t.before;
    BUILD(t.ib, mov_imm, REG(EAX),        IMM(0x5555, 4)); /* live */
    BUILD(t.ib, mov_imm, REG(EBX),        IMM(0x5555, 4)); /* dead */
    BUILD(t.ib, mov_st,  MEM(XSP, 0, 4),  REG(EAX));

    t.ib.ilist = t.after;
    BUILD(t.ib, mov_imm, REG(EAX),        IMM(0x5555, 4));
    BUILD(t.ib, mov_st,  MEM(XSP, 0, 4),  REG(EAX));

    t.ci.ilist = t.before;
    dce_and_copy_prop(t.dc, &t.ci);

    ASSERT(ilists_same(t.dc, t.before, t.after), "DCE failed");

    ilist_cmp_test_fini(&t);
}

TEST(copy_prop_basic) {
    ilist_cmp_test_t t;
    ilist_cmp_test_init(&t);

    t.ib.ilist = t.before;
    BUILD(t.ib, mov_ld, REG(XAX),           MEM(XSP, 0, PTR));  /* load */
    BUILD(t.ib, mov_ld, REG(XBX),           REG(XAX));          /* copy */
    BUILD(t.ib, mov_st, MEM(XSP, 0, PTR),   REG(XBX));          /* use */
    BUILD(t.ib, mov_st, MEM_IDX(XSP, XBX, 1, 0, PTR), REG(XBX)); /* two use */
    BUILD(t.ib, mov_ld, REG(XBX),           MEM(XSP, 0x08, PTR));  /* clobber */
    BUILD(t.ib, mov_st, MEM(XSP, 0, PTR),   REG(XBX));          /* use */

    t.ib.ilist = t.after;
    BUILD(t.ib, mov_ld, REG(XAX),           MEM(XSP, 0, PTR));  /* load */
    BUILD(t.ib, mov_st, MEM(XSP, 0, PTR),   REG(XAX));          /* store */
    BUILD(t.ib, mov_st, MEM_IDX(XSP, XAX, 1, 0, PTR), REG(XAX)); /* store */
    BUILD(t.ib, mov_ld, REG(XBX),           MEM(XSP, 0x08, PTR));  /* clobber */
    BUILD(t.ib, mov_st, MEM(XSP, 0, PTR),   REG(XBX));          /* use */

    t.ci.ilist = t.before;
    dce_and_copy_prop(t.dc, &t.ci);

    ASSERT(ilists_same(t.dc, t.before, t.after), "copy prop failed");

    ilist_cmp_test_fini(&t);
}

TEST(test_reuse_dead_reg) {
    ilist_cmp_test_t t;
    ilist_cmp_test_init(&t);

    t.ib.ilist = t.before;
    BUILD(t.ib, mov_ld, REG(XAX),           MEM(XSP, 0, PTR));  /* load */
    BUILD(t.ib, mov_st, MEM(XSP, 0, PTR),   REG(XAX));          /* store */
    /* xax is dead now, can replace xbx w/ xax */
    BUILD(t.ib, mov_ld, REG(XBX),           MEM(XSP, 0, PTR));  /* load */
    BUILD(t.ib, mov_st, MEM(XSP, 0, PTR),   REG(XBX));          /* store */

    t.ib.ilist = t.after;
    BUILD(t.ib, mov_ld, REG(XAX),           MEM(XSP, 0, PTR));  /* load */
    BUILD(t.ib, mov_st, MEM(XSP, 0, PTR),   REG(XAX));          /* store */
    BUILD(t.ib, mov_ld, REG(XAX),           MEM(XSP, 0, PTR));  /* load */
    BUILD(t.ib, mov_st, MEM(XSP, 0, PTR),   REG(XAX));          /* store */

    t.ci.ilist = t.before;
    memset(t.ci.reg_used, 0, sizeof(t.ci.reg_used));
    /* Used XAX and XBX. */
    t.ci.reg_used[DR_REG_XAX - DR_REG_XAX] = true;
    t.ci.reg_used[DR_REG_XBX - DR_REG_XAX] = true;
    reuse_registers(t.dc, &t.ci);

    ASSERT(ilists_same(t.dc, t.before, t.after), "copy prop failed");

    ilist_cmp_test_fini(&t);
}

TEST(fold_mov_imm_basic) {
    ilist_cmp_test_t t;
    ilist_cmp_test_init(&t);

    t.ib.ilist = t.before;
    BUILD(t.ib, mov_imm, REG(EAX), IMM(0x44, 4)); /* immediate */
    /* folding into memops */
    BUILD(t.ib, mov_st, MEM_IDX(XAX,  XBP, 1, 0, PTR), REG(XBX)); /* base */
    BUILD(t.ib, mov_st, MEM_IDX(XSP,  XAX, 1, 0, PTR), REG(XBX)); /* index */
    BUILD(t.ib, mov_st, MEM_IDX(XSP,  XAX, 2, 0, PTR), REG(XBX)); /* scale 2 */
    BUILD(t.ib, mov_st, MEM_IDX(NULL, XAX, 2, 0, PTR), REG(XBX)); /* no base */
    /* other */
    BUILD(t.ib, add,    REG(EBX), REG(EAX));
    BUILD(t.ib, cmp,    REG(EBX), REG(EAX));
    BUILD(t.ib, test,   REG(EBX), REG(EAX));

    t.ib.ilist = t.after;
    BUILD(t.ib, mov_st, MEM(XBP,  0x44, PTR), REG(XBX)); /* base */
    BUILD(t.ib, mov_st, MEM(XSP,  0x44, PTR), REG(XBX)); /* index */
    BUILD(t.ib, mov_st, MEM(XSP,  0x88, PTR), REG(XBX)); /* scale 2 */
    BUILD(t.ib, mov_st, MEM(NULL, 0x88, PTR), REG(XBX)); /* no base */
    /* other */
    BUILD(t.ib, add,    REG(EBX), IMM(0x44, 4));
    BUILD(t.ib, cmp,    REG(EBX), IMM(0x44, 4));
    BUILD(t.ib, test,   REG(EBX), IMM(0x44, 4));

    try_fold_immeds(t.dc, GLOBAL_DCONTEXT, t.before);

    ASSERT(ilists_same(t.dc, t.before, t.after), "fold immediates failed");

    ilist_cmp_test_fini(&t);
}

TEST(fold_mov_imm_64bit) {
#ifdef X64
    ilist_cmp_test_t t;
    ilist_cmp_test_init(&t);

    t.ib.ilist = t.before;
    BUILD(t.ib, mov_imm, REG(RAX), IMM(0x44, 8)); /* immediate */
    BUILD(t.ib, mov_st, MEM_IDX(XSP, XAX, 1, 0, PTR), REG(XBX)); /* index */

    t.ib.ilist = t.after;
    BUILD(t.ib, mov_st, MEM(XSP, 0x44, PTR), REG(XBX)); /* index */

    try_fold_immeds(t.dc, GLOBAL_DCONTEXT, t.before);

    ASSERT(ilists_same(t.dc, t.before, t.after), "fold immediates failed");

    ilist_cmp_test_fini(&t);
#endif
}

TEST(test_commute_op_test) {
    ilist_cmp_test_t t;
    ilist_cmp_test_init(&t);

    t.ib.ilist = t.before;
    BUILD(t.ib, mov_imm, REG(EAX), IMM(0x44, 4)); /* immediate */
    BUILD(t.ib, test,    REG(EAX), REG(EBX));

    t.ib.ilist = t.after;
    BUILD(t.ib, test,    REG(EBX), IMM(0x44, 4));

    try_fold_immeds(t.dc, GLOBAL_DCONTEXT, t.before);

    ASSERT(ilists_same(t.dc, t.before, t.after), "fold immediates failed");

    ilist_cmp_test_fini(&t);
}

TEST(test_jmp_next) {
    instr_t *label;
    ilist_cmp_test_t t;
    ilist_cmp_test_init(&t);

    t.ib.ilist = t.before;
    label = INSTR_CREATE_label(t.ib.dcontext);
    BUILD(t.ib, jmp, opnd_create_instr(label)); /* immediate */
    INSERT(t.ib, label);

    /* t.after is empty */

    remove_jmp_next_instr(t.dc, GLOBAL_DCONTEXT, t.before);

    ASSERT(ilists_same(t.dc, t.before, t.after), "ilists differ");

    ilist_cmp_test_fini(&t);
}

TEST(test_rle) {
    ilist_cmp_test_t t;
    ilist_cmp_test_init(&t);

    t.ib.ilist = t.before;
    BUILD(t.ib, mov_st, MEM(XSP, 0x00, PTR), REG(XAX));
    BUILD(t.ib, mov_ld, REG(XAX), MEM(XSP, 0x00, PTR));  /* redundant */
    BUILD(t.ib, mov_ld, REG(XBX), MEM(XSP, 0x00, PTR));  /* needs copy */
    BUILD(t.ib, mov_st, MEM(XBP, 0x00, PTR), REG(XCX));  /* can alias */
    BUILD(t.ib, mov_ld, REG(XAX), MEM(XSP, 0x00, PTR));  /* not redundant */
    BUILD(t.ib, mov_ld, REG(XCX), MEM(XBP, 0x00, PTR));  /* redundant */
    BUILD(t.ib, add,    REG(XBP), IMM(0x10, 4));         /* modifies base */
    BUILD(t.ib, mov_ld, REG(XCX), MEM(XBP, 0x00, PTR));  /* not redundant */


    t.ib.ilist = t.after;
    BUILD(t.ib, mov_st, MEM(XSP, 0x00, PTR), REG(XAX));
    BUILD(t.ib, mov_ld, REG(XBX),            REG(XAX));  /* needs copy */
    BUILD(t.ib, mov_st, MEM(XBP, 0x00, PTR), REG(XCX));  /* can alias */
    BUILD(t.ib, mov_ld, REG(XAX), MEM(XSP, 0x00, PTR));  /* not redundant */
    BUILD(t.ib, add,    REG(XBP), IMM(0x10, 4));         /* modifies base */
    BUILD(t.ib, mov_ld, REG(XCX), MEM(XBP, 0x00, PTR));  /* not redundant */

    redundant_load_elim(t.dc, GLOBAL_DCONTEXT, t.before);

    ASSERT(ilists_same(t.dc, t.before, t.after), "ilists differ");

    ilist_cmp_test_fini(&t);
}

TEST(test_dse) {
    ilist_cmp_test_t t;
    ilist_cmp_test_init(&t);

    t.ib.ilist = t.before;
    BUILD(t.ib, mov_st, MEM(XSP, 0x00, PTR), REG(XAX));  /* dead */
    BUILD(t.ib, mov_st, MEM(XSP, 0x00, PTR), REG(XBX));  /* dead */
    BUILD(t.ib, mov_st, MEM(XSP, 0x00, PTR), REG(XCX));
    BUILD(t.ib, sub,    REG(XSP), IMM(0x18, 4)); /* modifies base */
    BUILD(t.ib, mov_st, MEM(XSP, 0x00, PTR), REG(XAX));
    BUILD(t.ib, mov_st, MEM(XSP, 0x01, PTR), REG(XAX));  /* write aliases */
    BUILD(t.ib, mov_st, MEM(XSP, 0x00, PTR), REG(XAX));
    BUILD(t.ib, mov_ld, REG(XBX), MEM(XBP, 0x10, PTR));  /* load could alias */
    BUILD(t.ib, mov_st, MEM(XSP, 0x00, PTR), REG(XAX));
#ifdef X64
    BUILD(t.ib, mov_st, MEM_REL((void*)&test_dse, PTR), REG(XAX));
    BUILD(t.ib, mov_st, MEM_REL((void*)&test_dse, PTR), REG(XAX));
#endif

    t.ib.ilist = t.after;
    BUILD(t.ib, mov_st, MEM(XSP, 0x00, PTR), REG(XCX));
    BUILD(t.ib, sub,    REG(XSP), IMM(0x18, 4)); /* modifies base */
    BUILD(t.ib, mov_st, MEM(XSP, 0x00, PTR), REG(XAX));
    BUILD(t.ib, mov_st, MEM(XSP, 0x01, PTR), REG(XAX));  /* write aliases */
    BUILD(t.ib, mov_st, MEM(XSP, 0x00, PTR), REG(XAX));
    BUILD(t.ib, mov_ld, REG(XBX), MEM(XBP, 0x10, PTR));  /* load could alias */
    BUILD(t.ib, mov_st, MEM(XSP, 0x00, PTR), REG(XAX));
#ifdef X64
    BUILD(t.ib, mov_st, MEM_REL((void*)&test_dse, PTR), REG(XAX));
#endif

    dead_store_elim(t.dc, GLOBAL_DCONTEXT, t.before);

    ASSERT(ilists_same(t.dc, t.before, t.after), "fold immediates failed");

    ilist_cmp_test_fini(&t);
}

TEST(test_fold_lea) {
    ptr_int_t ptr = (ptr_int_t)&test_fold_lea;
    ilist_cmp_test_t t;
    ilist_cmp_test_init(&t);

    t.ib.ilist = t.before;
    /* Simple base disp -> base disp fold. */
    BUILD(t.ib, lea,    REG(XAX), MEM(XSP, 0x10, lea));
    BUILD(t.ib, mov_ld, REG(XBX), MEM(XAX, 0x10, PTR));
    /* Don't fold if regs used in lea are clobbered. */
    BUILD(t.ib, lea,    REG(XAX), MEM(XSI, 0x10, lea));
    BUILD(t.ib, xor,    REG(XSI), REG(XSI));
    BUILD(t.ib, mov_ld, REG(XBX), MEM(XAX, 0x10, PTR));
    /* Index in lea, not target. */
    BUILD(t.ib, lea,    REG(XAX), MEM_IDX(XBP, XSI, 0x04, 0x10, lea));
    BUILD(t.ib, mov_ld, REG(XBX), MEM(XAX, 0x10, PTR));
    /* Index in target, not lea. */
    BUILD(t.ib, lea,    REG(XAX), MEM(XBP, 0x10, lea));
    BUILD(t.ib, mov_ld, REG(XBX), MEM_IDX(XAX, XSI, 0x04, 0x10, PTR));
    /* Shift lea into base with index and scale 1. */
    BUILD(t.ib, lea,    REG(XAX), MEM_IDX(NULL, XSI, 0x08, 0x10, lea));
    BUILD(t.ib, mov_ld, REG(XBX), MEM_IDX(XAX,  XDI, 0x01, 0x10, PTR));
    /* Index, no scale lea into full BISD. */
    BUILD(t.ib, lea,    REG(XAX), MEM_IDX(NULL, XSI, 0x01, 0x10, lea));
    BUILD(t.ib, mov_ld, REG(XBX), MEM_IDX(XAX,  XDI, 0x08, 0x10, PTR));
    /* Merge riprel. */
    BUILD(t.ib, lea,    REG(XAX), MEM_REL(ptr + 0x00, lea));
    BUILD(t.ib, lea,    REG(XBX), MEM_REL(ptr + 0x10, lea));
    BUILD(t.ib, lea,    REG(XCX), MEM_REL(ptr - 0x10, lea));
    BUILD(t.ib, mov_ld, REG(XDX), MEM(XAX, 0, PTR));
    BUILD(t.ib, mov_ld, REG(XDI), MEM(XBX, 0, PTR));
    BUILD(t.ib, mov_ld, REG(XSI), MEM(XCX, 0, PTR));
    BUILD(t.ib, mov_ld, REG(XDX), MEM_REL(ptr + 0x20, PTR));
    BUILD(t.ib, mov_st, MEM_REL(ptr + 0x20, PTR), REG(XDX));

    t.ib.ilist = t.after;
    /* Simple base disp -> base disp fold. */
    BUILD(t.ib, mov_ld, REG(XBX), MEM(XSP, 0x20, PTR));
    /* Don't fold if regs used in lea are clobbered. */
    BUILD(t.ib, lea,    REG(XAX), MEM(XSI, 0x10, lea));
    BUILD(t.ib, xor,    REG(XSI), REG(XSI));
    BUILD(t.ib, mov_ld, REG(XBX), MEM(XAX, 0x10, PTR));
    /* Index in lea, not target. */
    BUILD(t.ib, mov_ld, REG(XBX), MEM_IDX(XBP, XSI, 0x04, 0x20, PTR));
    /* Index in target, not lea. */
    BUILD(t.ib, mov_ld, REG(XBX), MEM_IDX(XBP, XSI, 0x04, 0x20, PTR));
    /* Shift lea into base with index and scale 1. */
    BUILD(t.ib, mov_ld, REG(XBX), MEM_IDX(XDI, XSI, 0x08, 0x20, PTR));
    /* Index, no scale lea into full BISD. */
    BUILD(t.ib, mov_ld, REG(XBX), MEM_IDX(XSI, XDI, 0x08, 0x20, PTR));
    /* Merge riprel. */
    BUILD(t.ib, lea,    REG(XAX), MEM_REL(ptr, lea));
    BUILD(t.ib, mov_ld, REG(XDX), MEM(XAX,  0x00, PTR));
    BUILD(t.ib, mov_ld, REG(XDI), MEM(XAX,  0x10, PTR));
    BUILD(t.ib, mov_ld, REG(XSI), MEM(XAX, -0x10, PTR));
    BUILD(t.ib, mov_ld, REG(XDX), MEM(XAX,  0x20, PTR));
    BUILD(t.ib, mov_st, MEM(XAX, 0x20, PTR), REG(XDX));

    fold_leas(t.dc, GLOBAL_DCONTEXT, t.before);

    ASSERT(ilists_same(t.dc, t.before, t.after), "fold leas failed");

    ilist_cmp_test_fini(&t);
}

TEST(test_fold_consecutive_lea) {
    ilist_cmp_test_t t;
    ilist_cmp_test_init(&t);

    t.ib.ilist = t.before;
    /* 4 consecutive leas. */
    BUILD(t.ib, lea,    REG(XAX), MEM(XAX, 0x01, lea));
    BUILD(t.ib, lea,    REG(XAX), MEM(XAX, 0x01, lea));
    BUILD(t.ib, lea,    REG(XAX), MEM(XAX, 0x01, lea));
    BUILD(t.ib, lea,    REG(XAX), MEM(XAX, 0x01, lea));
    BUILD(t.ib, mov_st, MEM(XSP, 0x00, PTR), REG(XAX));

    t.ib.ilist = t.after;
    BUILD(t.ib, lea,    REG(XAX), MEM(XAX, 0x04, lea));
    BUILD(t.ib, mov_st, MEM(XSP, 0x00, PTR), REG(XAX));

    fold_leas(t.dc, GLOBAL_DCONTEXT, t.before);

    ASSERT(ilists_same(t.dc, t.before, t.after), "fold leas failed");

    ilist_cmp_test_fini(&t);
}

TEST(test_fold_imm_across_partial) {
    ilist_cmp_test_t t;
    ilist_cmp_test_init(&t);

    t.ib.ilist = t.before;
    BUILD(t.ib, mov_imm, REG(EAX), IMM(0x44, 4)); /* immediate */
    BUILD(t.ib, test,    REG(RCX), REG(RCX));     /* partial check */
    INSERT(t.ib, INSTR_CREATE_jcc(t.ib.dcontext, OP_jz, opnd_create_pc(NULL)));
    INSERT(t.ib, INSTR_CREATE_label(t.ib.dcontext));
    BUILD(t.ib, call, opnd_create_pc(NULL));
    instr_set_ok_to_mangle(instrlist_last(t.ib.ilist), true);
    INSERT(t.ib, INSTR_CREATE_label(t.ib.dcontext));
    BUILD(t.ib, jmp_short, opnd_create_pc(NULL));
    BUILD(t.ib, mov_st,  MEM(XBP, 0, 4), REG(EAX)); /* use */

    t.ib.ilist = t.after;
    BUILD(t.ib, test,    REG(RCX), REG(RCX));     /* partial check */
    INSERT(t.ib, INSTR_CREATE_jcc(t.ib.dcontext, OP_jz, opnd_create_pc(NULL)));
    INSERT(t.ib, INSTR_CREATE_label(t.ib.dcontext));
    BUILD(t.ib, call, opnd_create_pc(NULL));
    instr_set_ok_to_mangle(instrlist_last(t.ib.ilist), true);
    INSERT(t.ib, INSTR_CREATE_label(t.ib.dcontext));
    BUILD(t.ib, jmp_short, opnd_create_pc(NULL));
    BUILD(t.ib, mov_st,  MEM(XBP, 0, 4), IMM(0x44, 4)); /* use */

    try_fold_immeds(t.dc, GLOBAL_DCONTEXT, t.before);

    ASSERT(ilists_same(t.dc, t.before, t.after), "fold immed failed");

    ilist_cmp_test_fini(&t);
}

#define TEST_FUNCS() \
    TEST_FN(dce_basic) \
    TEST_FN(copy_prop_basic) \
    TEST_FN(test_reuse_dead_reg) \
    TEST_FN(fold_mov_imm_basic) \
    TEST_FN(fold_mov_imm_64bit) \
    TEST_FN(test_commute_op_test) \
    TEST_FN(test_jmp_next) \
    TEST_FN(test_rle) \
    TEST_FN(test_dse) \
    TEST_FN(test_fold_lea) \
    TEST_FN(test_fold_consecutive_lea) \
    TEST_FN(test_fold_imm_across_partial) \

#if 0
/* Non macro framework idea: */
static test_fn_t test_funcs[] = {
    dce_basic,
    copy_prop_basic,
    fold_mov_imm_basic,
    fold_mov_imm_64bit,
    cant_encode_test_immed,
    test_jmp_next,
    test_rle,
    test_dse,
    NULL
};

static int
fake_main(void)
{
    int i;
    test_results_t results;
    test_results_init(&results);
    for (i = 0; test_funcs[i] != NULL; i++) {
        run_test_func(&results, test_funcs[i]);
    }
    return (&results);
}
#endif

static int
fake_main(void)
{
    RUN_ALL_TESTS_START();
#define TEST_FN RUN_TEST
    TEST_FUNCS();
#undef TEST_FN
    RUN_ALL_TESTS_END();
}

int
main(void)
{
    int r;
    dr_app_setup();
    drcalls_init();

    r = fake_main();

    drcalls_exit();
    dr_app_cleanup();
    return r;
}
