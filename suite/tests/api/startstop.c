/* **********************************************************
 * Copyright (c) 2011 Google, Inc.  All rights reserved.
 * Copyright (c) 2003-2008 VMware, Inc.  All rights reserved.
 * **********************************************************/

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

#include <assert.h>
#include <stdio.h>
#include <math.h>
#ifdef USE_DYNAMO
#include "configure.h"
#include "dr_api.h"
#endif
#include "tools.h"
#include "threads.h"
#ifdef WINDOWS
# include <windows.h>
#endif

#define ITERS 150000

/* We have event bb look for this to make sure we're instrumenting the sideline
 * thread.  
 */
NOINLINE void sideline_func(void) { }

static bool took_over_sideline = false;

static dr_emit_flags_t
event_bb(void *drcontext, void *tag, instrlist_t *bb, bool for_trace,
         bool translating)
{
    if (instr_get_app_pc(instrlist_first(bb)) == (app_pc)&sideline_func)
        took_over_sideline = true;
    return DR_EMIT_DEFAULT;
}

/* This is a thread that spins and calls sideline_func.  It will call
 * sideline_func at least once before returning and joining the parent.
 */
static volatile bool should_spin = true;

int
sideline_spinner(void *unused_arg)
{
    do {
        sideline_func();
#ifdef WINDOWS
        Sleep(0);
#else
        sleep(0);
#endif
    } while (should_spin);
    return 0;
}

void foo(void)
{
}

int main(void)
{
    double res = 0.;
    int i,j;
    void *stack = NULL;
    thread_t thread;
    ptr_uint_t tid;

#ifdef USE_DYNAMO
    dr_app_setup();
#endif

    dr_register_bb_event(event_bb);

    /* Create a spinning sideline thread. */
    thread = thread_create(sideline_spinner, NULL, &stack);
#ifdef USE_DYNAMO
# ifdef WINDOWS
    tid = GetThreadId(thread);
# else /* LINUX */
    tid = thread;
# endif /* LINUX */
#endif

    for (j=0; j<10; j++) {
#ifdef USE_DYNAMO
        dr_app_start();
#endif
        for (i=0; i<ITERS; i++) {
            if (i % 2 == 0) {
                res += cos(1./(double)(i+1));
            } else {
                res += sin(1./(double)(i+1));
	    }
	}
	foo();
#ifdef USE_DYNAMO
	dr_app_stop();
#endif
    }
    /* PR : we get different floating point results on different platforms,
     * so we no longer print out res */
    print("all done: %d iters\n", i);

    if (!took_over_sideline)
        print("failed to take over sideline thread!\n");

    should_spin = false;  /* Break it's loop. */
    delete_thread(thread, stack);

#ifdef USE_DYNAMO
    dr_app_cleanup();
#endif

    return 0;
}
