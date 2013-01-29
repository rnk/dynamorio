/* **********************************************************
 * Copyright (c) 2013 Google, Inc.  All rights reserved.
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

/* Intercepts module transitions for native execution for ELF modules.
 */

#include "../globals.h"
#include "native_exec.h"
#include "module.h"
#include "instr.h"
#include "decode.h"

/* According to the SysV amd64 psABI docs, there are three reserved entries
 * in the PLTGOT:
 * 1. offset to .dynamic section
 * 2. available for loader data, used for link map
 * 3. pointer to resolution stub, used for _dl_runtime_resolve
 *
 * We want to replace 3, _dl_runtime_resolve, with a stub in x86.asm.  See
 * Figure 5.2 for more details.
 *
 * FIXME: Find documentation that this is the same on ia32 and test it.
 */
enum { DL_RUNTIME_RESOLVE_IDX = 2 };

/* The loader's _dl_fixup. */
void *(*app_dl_fixup)(void *link_map, uint dynamic_index);

static bool
module_contains_pc(module_area_t *ma, app_pc pc)
{
    return pc >= ma->start && pc <= ma->end;
}

static app_pc
module_get_pltgot(module_area_t *ma)
{
    os_privmod_data_t opd;
    app_pc pltgot;
    memset(&opd, 0, sizeof(opd));
    module_get_os_privmod_data(ma->start, ma->end - ma->start, &opd);
    pltgot = (app_pc) opd.pltgot;
    if (module_contains_pc(ma, pltgot)) {
        return pltgot;
    }
    /* FIXME: module_get_os_privmod_data() assumes the module hasn't already
     * been relocated, which may be the case for loaded modules.  Try to recover
     * by subtracting out the module base, which is typically equal to
     * load_delta.
     */
    pltgot = (app_pc) (pltgot - ma->start);
    if (module_contains_pc(ma, pltgot)) {
        return pltgot;
    }
    return NULL;
}

void
find_dl_fixup(dcontext_t *dcontext, app_pc resolver)
{
    instr_t instr;
    int max_decodes = 30;
    int i = 0;
    app_pc pc = resolver;

    instr_init(dcontext, &instr);
    while (pc != NULL && i < max_decodes) {
        pc = decode(dcontext, pc, &instr);
        if (instr_get_opcode(&instr) == OP_call) {
            opnd_t tgt = instr_get_target(&instr);
            app_dl_fixup = (void *(*)(void *, uint)) opnd_get_pc(tgt);
            break;
        } else if (instr_is_cti(&instr)) {
            break;
        }
        instr_reset(dcontext, &instr);
    }
    instr_free(dcontext, &instr);
}

/* Find the _dl_runtime_resolve pointer and overwrite it with our own.
 */
void
hook_module_for_native_exec(module_area_t *ma)
{
    dcontext_t *dcontext = get_thread_private_dcontext();
    app_pc *pltgot = (app_pc *) module_get_pltgot(ma);
    app_pc resolver;
    ASSERT_CURIOSITY(pltgot != NULL && "unable to locate DT_PLTGOT");
    if (pltgot == NULL)
        return;

    resolver = pltgot[DL_RUNTIME_RESOLVE_IDX];
    if (resolver == NULL) {
        ASSERT_CURIOSITY(false && "loader will overwrite our stub");
        /* NYI */
    }
    if (resolver != NULL && app_dl_fixup == NULL) {
        /* _dl_fixup is not exported, so we have to go find it. */
        find_dl_fixup(dcontext, resolver);
        ASSERT_CURIOSITY(app_dl_fixup != NULL && "failed to find _dl_fixup");
    }

    LOG(THREAD, LOG_LOADER, 3,
        "%s: replacing _dl_runtime_resolve "PFX" with "PFX"\n",
        __FUNCTION__, resolver, _dynamorio_runtime_resolve);
    pltgot[DL_RUNTIME_RESOLVE_IDX] = (app_pc) _dynamorio_runtime_resolve;
}

/* Our replacement for _dl_fixup.
 */
void *
dynamorio_dl_fixup(void *link_map, uint dynamic_index)
{
    return app_dl_fixup(link_map, dynamic_index);
}
