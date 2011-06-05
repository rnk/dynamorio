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

typedef struct ilist_cmp_test_t_ {
    void *dc;
    instrlist_t *before;
    instrlist_t *after;
    instr_builder_t ib;
    callee_info_t ci;
} ilist_cmp_test_t;

static void
ilist_cmp_test_init(ilist_cmp_test_t *test)
{
    test->dc = dr_get_current_drcontext();
    test->before = instrlist_create(GLOBAL_DCONTEXT);
    test->after = instrlist_create(GLOBAL_DCONTEXT);
    INSTR_BUILDER_INIT(test->ib, GLOBAL_DCONTEXT, NULL, NULL, false);
    callee_info_init(&test->ci);
}

static void
ilist_cmp_test_fini(ilist_cmp_test_t *test)
{
    instrlist_clear_and_destroy(GLOBAL_DCONTEXT, test->before);
    instrlist_clear_and_destroy(GLOBAL_DCONTEXT, test->after);
}

static bool
ilists_same(void *dc, instrlist_t *ilist_a, instrlist_t *ilist_b)
{
    instr_t *instr_a, *instr_b;
    bool same = true;
    for (instr_a = instrlist_first(ilist_a),
         instr_b = instrlist_first(ilist_b);
         instr_a != NULL && instr_b != NULL;
         instr_a = instr_get_next(instr_a),
         instr_b = instr_get_next(instr_b)) {
        if (!instr_same(instr_a, instr_b)) {
            same = false;
            break;
        }
    }
    same &= (instr_a == NULL && instr_b == NULL);
    if (!same) {
        dr_fprintf(STDERR, "ilists differ!\n");
        dr_fprintf(STDERR, "ilist A:\n");
        instrlist_disassemble(dc, NULL, ilist_a, STDERR);
        dr_fprintf(STDERR, "\nilist B:\n");
        instrlist_disassemble(dc, NULL, ilist_b, STDERR);
        dr_fprintf(STDERR, "\n");
    }
    return same;
}

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
    BUILD(t.ib, mov_st, MEM(XSP, 0, PTR),   REG(XBX));          /* store */
    BUILD(t.ib, mov_st, MEM_IDX(XSP, XBX, 1, 0, PTR), REG(XBX)); /* store */

    t.ib.ilist = t.after;
    BUILD(t.ib, mov_ld, REG(XAX),           MEM(XSP, 0, PTR));  /* load */
    BUILD(t.ib, mov_st, MEM(XSP, 0, PTR),   REG(XAX));          /* store */
    BUILD(t.ib, mov_st, MEM_IDX(XSP, XAX, 1, 0, PTR), REG(XAX)); /* store */

    t.ci.ilist = t.before;
    dce_and_copy_prop(t.dc, &t.ci);

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

TEST(cant_encode_test_immed) {
    ilist_cmp_test_t t;
    ilist_cmp_test_init(&t);

    t.ib.ilist = t.before;
    BUILD(t.ib, mov_imm, REG(EAX), IMM(0x44, 4)); /* immediate */
    BUILD(t.ib, test,    REG(EAX), REG(EBX));

    /* Should be no change, since immediate cannot be encoded to "left" operand
     * of test. */
    /* XXX: If we teach the optimizer how to flip operands, this might change. */
    instrlist_destroy(GLOBAL_DCONTEXT, t.after);
    t.after = instrlist_clone(GLOBAL_DCONTEXT, t.before);

    try_fold_immeds(t.dc, GLOBAL_DCONTEXT, t.before);

    ASSERT(ilists_same(t.dc, t.before, t.after), "fold immediates failed");

    ilist_cmp_test_fini(&t);
}

#define TEST_FUNCS() \
    TEST_FN(dce_basic) \
    TEST_FN(copy_prop_basic) \
    TEST_FN(fold_mov_imm_basic) \
    TEST_FN(fold_mov_imm_64bit) \
    TEST_FN(cant_encode_test_immed)

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
