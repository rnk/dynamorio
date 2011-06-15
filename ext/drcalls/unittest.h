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

/* Unit test "framework" I made up for drcalls.
 * Tries to be minimalist.  If we want more, we can add a real framework.
 */

/* Tests are declared static so -Wunused will complain if they are not called.
 */
#define TEST(test_fn) \
    static void test_fn(void)

/* Global test pass/fail var. */
static bool test_passed_;

/* TODO: document
 * IDEA: msg can include format string stuff, use varargs to pass params
 * through.
 */
#define ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            test_passed_ = false; \
            dr_fprintf(STDERR, "test %s failed: "msg"\n", __func__); \
            return; \
        } \
    } while (0)

typedef void (*test_fn_t)(void);

#define RUN_ALL_TESTS_START() \
    { /* open new scope to declare var */ \
        bool all_passed = true; \
        int num_failed = 0; \
        int num_passed = 0;
#define RUN_TEST(test_fn) { \
        bool passed; \
        test_passed_ = true; \
        test_fn(); \
        passed = test_passed_; \
        all_passed &= passed; \
        num_failed += (passed ? 0 : 1); \
        num_passed += (passed ? 1 : 0); \
    }
#define RUN_ALL_TESTS_END() \
        if (all_passed) { \
            dr_fprintf(STDOUT, "All tests PASSED!\n"); \
            return 0; \
        } else { \
            dr_fprintf(STDERR, "%d tests PASSED\n", num_passed); \
            dr_fprintf(STDERR, "%d tests FAILED!\n", num_failed); \
            /* return non-zero to indicate failure. */ \
            return -1; \
        } \
    }

/* Ilist comparison test support. */
/* TODO(rnk): Obviously code shouldn't go in headers, but moving it out into a
 * separate file requires mucking with CMake... */

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
    INSTR_BUILDER_INIT(test->ib, GLOBAL_DCONTEXT, NULL, NULL, /*meta=*/true);
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

