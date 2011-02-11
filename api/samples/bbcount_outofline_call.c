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

/* This is the simplest possible basic block client that uses shared,
 * out-of-line clean call sequences.
 */

#include "dr_api.h"
#include "dr_calls.h"

#ifdef WINDOWS
# define DISPLAY_STRING(msg) dr_messagebox(msg)
#else
# define DISPLAY_STRING(msg) dr_printf("%s\n", msg);
#endif

#define NULL_TERMINATE(buf) buf[(sizeof(buf)/sizeof(buf[0])) - 1] = '\0'

static uint64 global_bb_count;

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
    dr_log(NULL, LOG_ALL, 1, "Client 'bbcount_clean_call' initializing\n");
}

static void
event_exit(void)
{
    char msg[512];
    int len;
    len = dr_snprintf(msg, sizeof(msg)/sizeof(msg[0]),
                      "Instrumentation results:\n"
                      "%10"UINT64_FORMAT_CODE" basic block executions\n",
                      global_bb_count);
    DR_ASSERT(len > 0);
    NULL_TERMINATE(msg);
    DISPLAY_STRING(msg);
    drcalls_exit();
}

static void
do_counting(void)
{
    /* This update is racy, but it's acceptable for single threaded
     * applications and we're trying to keep this sample code simple. */
    global_bb_count++;
}

static void do_counting00(void) { global_bb_count++; }
static void do_counting01(void) { global_bb_count++; }
static void do_counting02(void) { global_bb_count++; }
static void do_counting03(void) { global_bb_count++; }
static void do_counting04(void) { global_bb_count++; }
static void do_counting05(void) { global_bb_count++; }
static void do_counting06(void) { global_bb_count++; }
static void do_counting07(void) { global_bb_count++; }
static void do_counting08(void) { global_bb_count++; }
static void do_counting09(void) { global_bb_count++; }
static void do_counting10(void) { global_bb_count++; }
static void do_counting11(void) { global_bb_count++; }
static void do_counting12(void) { global_bb_count++; }
static void do_counting13(void) { global_bb_count++; }
static void do_counting14(void) { global_bb_count++; }
static void do_counting15(void) { global_bb_count++; }
static void do_counting16(void) { global_bb_count++; }
static void do_counting17(void) { global_bb_count++; }
static void do_counting18(void) { global_bb_count++; }
static void do_counting19(void) { global_bb_count++; }
static void do_counting20(void) { global_bb_count++; }
static void do_counting21(void) { global_bb_count++; }
static void do_counting22(void) { global_bb_count++; }
static void do_counting23(void) { global_bb_count++; }
static void do_counting24(void) { global_bb_count++; }
static void do_counting25(void) { global_bb_count++; }
static void do_counting26(void) { global_bb_count++; }
static void do_counting27(void) { global_bb_count++; }
static void do_counting28(void) { global_bb_count++; }
static void do_counting29(void) { global_bb_count++; }
static void do_counting30(void) { global_bb_count++; }
static void do_counting31(void) { global_bb_count++; }
static void do_counting32(void) { global_bb_count++; }
static void do_counting33(void) { global_bb_count++; }
static void do_counting34(void) { global_bb_count++; }
static void do_counting35(void) { global_bb_count++; }
static void do_counting36(void) { global_bb_count++; }
static void do_counting37(void) { global_bb_count++; }
static void do_counting38(void) { global_bb_count++; }
static void do_counting39(void) { global_bb_count++; }

typedef void (*count_func)(void);

count_func fptr_array[] = {
    do_counting00, do_counting01, do_counting02, do_counting03, do_counting04,
    do_counting05, do_counting06, do_counting07, do_counting08, do_counting09,
    do_counting10, do_counting11, do_counting12, do_counting13, do_counting14,
    do_counting15, do_counting16, do_counting17, do_counting18, do_counting19,
    do_counting20, do_counting21, do_counting22, do_counting23, do_counting24,
    do_counting25, do_counting26, do_counting27, do_counting28, do_counting29,
    do_counting30, do_counting31, do_counting32, do_counting33, do_counting34,
    do_counting35, do_counting36, do_counting37, do_counting38, do_counting39,
    do_counting,
};

int fptr_counter = 0;

static dr_emit_flags_t
event_basic_block(void *drcontext, void *tag, instrlist_t *bb,
                  bool for_trace, bool translating)
{
    count_func f = fptr_array[fptr_counter];
    fptr_counter = (fptr_counter + 1) % 40;
    drcalls_shared_call(drcontext, bb, instrlist_first(bb), f, 0);
    return DR_EMIT_DEFAULT;
}
