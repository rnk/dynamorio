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
