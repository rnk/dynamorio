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
 * * Neither the name of VMware, Inc. nor the names of its contributors may be
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

static int global_bb_count;

static void event_exit(void);
static dr_emit_flags_t event_basic_block(void *drcontext, void *tag,
                                         instrlist_t *bb, bool for_trace,
                                         bool translating);

DR_EXPORT void
dr_init(client_id_t id)
{
    drcalls_init();
    dr_fprintf(STDERR, "Client 'bbcount' initializing\n");
    dr_register_exit_event(event_exit);
    dr_register_bb_event(event_basic_block);
}

static void
event_exit(void)
{
    int loop_iterations = 10000;
    /* Should be 3 bbs per loop iteration: top to call, leaf body, loop return.
     * We have to allow some slop for other bbs outside the loop. */
    int expected_bbcount = 3 * loop_iterations;
    int lower_bound = expected_bbcount - (expected_bbcount / 10);
    int upper_bound = expected_bbcount + (expected_bbcount / 10);
    if (lower_bound < global_bb_count && global_bb_count < upper_bound) {
        dr_fprintf(STDERR, "PASSED\n");
    } else {
        dr_fprintf(STDERR, "FAILED\n");
        dr_fprintf(STDERR, "lower_bound: %d, upper_bound: %d, bbcount: %d\n",
                  lower_bound, upper_bound, global_bb_count);
    }
    drcalls_exit();
}

static void
do_counting(void)
{
    /* This update is racy, but it's acceptable for single threaded
     * applications and we're trying to keep this sample code simple. */
    global_bb_count++;
}

static dr_emit_flags_t
event_basic_block(void *drcontext, void *tag, instrlist_t *bb,
                  bool for_trace, bool translating)
{
    drcalls_shared_call(drcontext, bb, instrlist_first(bb), do_counting, 0);
    return DR_EMIT_DEFAULT;
}
