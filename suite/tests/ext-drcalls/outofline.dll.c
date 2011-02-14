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

/* This is the simplest possible basic block client that uses shared,
 * out-of-line clean call sequences.
 */

#include "dr_api.h"
#include "dr_calls.h"

static void *last_ret_addr;
static bool failed;

static void event_exit(void);
static dr_emit_flags_t event_basic_block(void *drcontext, void *tag,
                                         instrlist_t *bb, bool for_trace,
                                         bool translating);

DR_EXPORT void
dr_init(client_id_t id)
{
    drcalls_init();
    dr_register_exit_event(event_exit);
    dr_register_bb_event(event_basic_block);

    last_ret_addr = NULL;
    failed = false;
}

static void
event_exit(void)
{
    dr_fprintf(STDERR, failed ? "FAILED\n" : "PASSED\n");
    drcalls_exit();
}

/* Get the return address of the clean call instrumentation function.  That
 * should point into the clean call context-switching code. */
static void
check_caller(void)
{
    void *my_ret_addr = __builtin_return_address(0);
    if (last_ret_addr && last_ret_addr != my_ret_addr) {
        dr_fprintf(STDERR, "Clean call return addrs does not match!\n"
                   "Last return addr: %p, My return addr: %p\n",
                   last_ret_addr, my_ret_addr);
        failed = true;
    }
    last_ret_addr = my_ret_addr;
}

int fptr_counter = 0;

static dr_emit_flags_t
event_basic_block(void *drcontext, void *tag, instrlist_t *bb,
                  bool for_trace, bool translating)
{
    /* If we used a regular clean call here, the return addrs don't match. */
    drcalls_shared_call(drcontext, bb, instrlist_first(bb), check_caller, 0);
    return DR_EMIT_DEFAULT;
}
