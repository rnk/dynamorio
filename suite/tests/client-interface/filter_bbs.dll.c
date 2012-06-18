/* **********************************************************
 * Copyright (c) 2012 Google, Inc.  All rights reserved.
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
 * * Neither the name of Google, Inc. nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL GOOGLE, INC. OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#include "dr_api.h"

static app_pc interesting_pc;
static int first_bb_was_interesting;  /* Tri-state, -1 is false. */
static int first_bb_event_count;
static int second_bb_event_count;

static bool
filter_bb_event(void *drcontext, app_pc start_pc)
{
    return start_pc == interesting_pc;
}

static
dr_emit_flags_t
second_bb_event(void *drcontext, void *tag, instrlist_t *bb, bool for_trace,
               bool translating)
{
    second_bb_event_count++;
    return DR_EMIT_DEFAULT;
}

static
dr_emit_flags_t
bb_event(void *drcontext, void *tag, instrlist_t *bb, bool for_trace,
         bool translating)
{
    if (first_bb_was_interesting == 0) {
        first_bb_was_interesting =
            (dr_fragment_app_pc(tag) == interesting_pc) ? 1 : -1;
        /* Now that we know the filter worked, register another bb event to
         * simulate two clients being present.  This should bypass the filter
         * and both events should get called.
         */
        dr_register_bb_event(second_bb_event);
    } else {
        first_bb_event_count++;
    }
    return DR_EMIT_DEFAULT;
}

static void
exit_event(void)
{
    dr_fprintf(STDERR, "first bb was interesting: %d\n", first_bb_was_interesting);
    dr_fprintf(STDERR, "first bb hook was called: %d\n", first_bb_event_count > 20);
    dr_fprintf(STDERR, "second bb hook was called: %d\n", second_bb_event_count > 20);
}

DR_EXPORT
void dr_init(client_id_t id)
{
    module_data_t *exe_data =
        dr_lookup_module_by_name(dr_get_application_name());
    interesting_pc = (app_pc) dr_get_proc_address(exe_data->handle,
                                                  "interesting_pc");
    dr_free_module_data(exe_data);

    dr_register_filter_bb_event(filter_bb_event);
    dr_register_bb_event(bb_event);
    dr_register_exit_event(exit_event);
}
