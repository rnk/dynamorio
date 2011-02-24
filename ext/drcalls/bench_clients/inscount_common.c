/* ******************************************************************************
 * Copyright (c) 2011 Massachusetts Institute of Technology  All rights reserved.
 * ******************************************************************************/

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

/* Common code for the various inscount benchmark clients.  They all instrument
 * basic blocks, count instructions from ilists, and print out the number of
 * instructions on exit.
 */

#include "dr_api.h"

#include "dr_calls.h"

/* Each inscount client variant defines a unique client_name and
 * event_basic_block function for instrumentation. */
extern const char *client_name;
dr_emit_flags_t event_basic_block(void *drcontext, void *tag, instrlist_t *bb,
                                  bool for_trace, bool translating);

/* We only have a global count. */
uint64 global_count;

/* A simple clean call that will be automatically inlined because it has only
 * one argument and contains no calls to other functions.
 */
static void event_exit(void);

DR_EXPORT void
dr_init(client_id_t id)
{
    drcalls_init();
    /* register events */
    dr_register_exit_event(event_exit);
    dr_register_bb_event(event_basic_block);

    dr_printf("Client %s running.\n", client_name);
}

static void
event_exit(void)
{
    dr_printf("Instrumentation results: %llu instructions executed\n", global_count);
    drcalls_exit();
}

uint
count_instrs(instrlist_t *bb)
{
    instr_t *instr;
    uint num_instrs = 0;
    for (instr = instrlist_first(bb); instr != NULL;
         instr = instr_get_next(instr)) {
        num_instrs++;
    }
    return num_instrs;
}

/* inscount callback used for clean calls. */
void
inscount(uint num_instrs)
{
    global_count += num_instrs;
}
