/* *******************************************************************************
 * Copyright (c) 2010-2011 Massachusetts Institute of Technology  All rights reserved.
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

#include "dr_api.h"
#include "dr_calls.h"
#include "hashtable.h"

#include <stdarg.h> /* for varargs */

/* internal includes */
#include "code_cache.h"
#include "core_compat.h"
#include "lean_call.h"
#include "inline.h"

static bool drcalls_initialized = false;

void
drcalls_init(void)
{
    code_cache_init();
    lean_call_init();
    inline_init();

    drcalls_initialized = true;
}

static void
check_init(void)
{
    DR_ASSERT_MSG(drcalls_initialized,
                  "Used drcalls before calling drcalls_init\n");
}

void
drcalls_exit(void)
{
    check_init();

    inline_exit();
    lean_call_exit();
    code_cache_destroy();

    drcalls_initialized = false;
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

void
drcalls_insert_call(void *dc, instrlist_t *ilist, instr_t *where, void *callee,
                    bool fpstate, uint num_args, ...)
{
    va_list ap;
    opnd_t *args = NULL;
    clean_call_info_t cci; /* information for clean call insertion. */

    check_init();

    /* Read arguments. */
    if (num_args > 0)
        args = dr_thread_alloc(dc, sizeof(opnd_t) * num_args);
    va_start(ap, num_args);
    convert_va_list_to_opnd(args, num_args, ap);
    va_end(ap);

    if (analyze_clean_call(dc, &cci, where, callee, fpstate, num_args, args)) {
        /* See if we can inline. */
        insert_inline_clean_call(dc, &cci, ilist, where, args);
        dr_log(dc, LOG_CLEANCALL, 2,
               "drcalls: inlined callee "PFX"\n", callee);
    } else {
        /* Otherwise, just use a clean call. */
        dr_insert_clean_call_vargs(dc, ilist, where, callee, fpstate, num_args,
                                   args);
    }

    if (num_args > 0)
        dr_thread_free(dc, args, sizeof(opnd_t) * num_args);
}

/* TODO(rnk): Should take save_fpstate. */
void
drcalls_lean_call(void *dc, instrlist_t *ilist, instr_t *where, void *callee,
                  uint num_args, ...)
{
    va_list ap;
    opnd_t *args = NULL;
    bool fpstate = false;

    check_init();

    if (num_args > 0) {
        args = dr_thread_alloc(dc, sizeof(opnd_t) * num_args);
        va_start(ap, num_args);
        convert_va_list_to_opnd(args, num_args, ap);
        va_end(ap);
    }

    /* If there are more args than TLS spill slots, give up and insert a normal
     * clean call. */
    if (num_args > 2) {
        dr_log(dc, LOG_ALL, 1, "drcalls: too many arguments for lean call, "
               "performance may suffer.\n");
        dr_insert_clean_call_vargs(dc, ilist, where, callee, fpstate, num_args,
                                   args);
    } else {
        lean_call_insert(dc, ilist, where, callee, fpstate, num_args, args);
    }

    if (num_args > 0) {
        dr_thread_free(dc, args, sizeof(opnd_t) * num_args);
    }
}

void
drcalls_set_optimization(uint opt_level)
{
    opt_cleancall = opt_level;
}

void
drcalls_done(void *dc, instrlist_t *bb)
{
    expand_and_optimize_bb(dc, bb);
}
