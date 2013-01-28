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

/* Support for natively executing some of the app's code.  At Determina, native
 * exec was used primarily to avoid security violation false positives in JITs.
 * For instrumentation clients, it can offer improved performance when dealing
 * with libraries that don't need to be instrumented.  However, we cannot
 * guarantee that we won't lose control or violate transparency.
 */

#include "native_exec.h"
#include "../globals.h"
#include "../vmareas.h"
#include "../module_shared.h"
#include "../instrlist.h"
#include "arch_exports.h"
#include "instr.h"
#include "decode_fast.h"

/* XXX: Centralize more of the native_exec implementation details here instead
 * of scattering it across interp.c, vmareas.c, etc.
 */

/* list of native_exec module regions 
 */
vm_area_vector_t *native_exec_areas;

void
native_exec_init(void)
{
    if (!DYNAMO_OPTION(native_exec) || DYNAMO_OPTION(thin_client))
        return;
    VMVECTOR_ALLOC_VECTOR(native_exec_areas, GLOBAL_DCONTEXT, VECTOR_SHARED,
                          native_exec_areas);
}

void
native_exec_exit(void)
{
    if (native_exec_areas == NULL)
        return;
    vmvector_delete_vector(GLOBAL_DCONTEXT, native_exec_areas);
    native_exec_areas = NULL;
}

static bool
on_native_exec_list(const char *modname)
{
    bool onlist = false;

    ASSERT(!DYNAMO_OPTION(thin_client));
    if (!DYNAMO_OPTION(native_exec))
        return false;

    if (!IS_STRING_OPTION_EMPTY(native_exec_default_list)) {
        string_option_read_lock();
        LOG(THREAD_GET, LOG_INTERP|LOG_VMAREAS, 4,
            "on_native_exec_list: module %s vs default list %s\n",
            modname==NULL?"null":modname,
            DYNAMO_OPTION(native_exec_default_list));
        onlist = check_filter(DYNAMO_OPTION(native_exec_default_list), modname);
        string_option_read_unlock();
    }
    if (!onlist &&
        !IS_STRING_OPTION_EMPTY(native_exec_list)) {
        string_option_read_lock();
        LOG(THREAD_GET, LOG_INTERP|LOG_VMAREAS, 4,
            "on_native_exec_list: module %s vs append list %s\n",
            modname==NULL?"null":modname, DYNAMO_OPTION(native_exec_list));
        onlist = check_filter(DYNAMO_OPTION(native_exec_list), modname);
        string_option_read_unlock();
    }
    return onlist;
}

static void
check_and_mark_native_exec(module_area_t *ma, bool add)
{
    bool is_native = false;
    const char *name = GET_MODULE_NAME(&ma->names);
    ASSERT(os_get_module_info_locked());
    if (DYNAMO_OPTION(native_exec) && name != NULL &&
        on_native_exec_list(name)) {
        LOG(GLOBAL, LOG_INTERP|LOG_VMAREAS, 1,
            "module %s is on native_exec list\n", name);
        is_native = true;
    }

    if (add && is_native) {
        RSTATS_INC(num_native_module_loads);
        vmvector_add(native_exec_areas, ma->start, ma->end, NULL);
    } else if (!add) {
        /* If we're removing and it's native, it should be on there already.  If
         * it's not native, then it shouldn't be present, but we'll remove
         * whatever is there.
         */
        DEBUG_DECLARE(bool present =)
            vmvector_remove(native_exec_areas, ma->start, ma->end);
        ASSERT_CURIOSITY((is_native && present) || (!is_native && !present));
    }
}

void
native_exec_module_load(module_area_t *ma)
{
    check_and_mark_native_exec(ma, true/*add*/);
}

void
native_exec_module_unload(module_area_t *ma)
{
    check_and_mark_native_exec(ma, false/*!add*/);
}

bool
at_native_exec_gateway(dcontext_t *dcontext, app_pc start
                       _IF_DEBUG(bool xfer_target))
{
    /* ASSUMPTION: transfer to another module will always be by indirect call
     * or non-inlined direct call from a fragment that will not be flushed.
     * For now we will only go native if last_exit was
     * a call, a true call*, or a PLT-style call,jmp* (and we detect the latter only
     * if the call is inlined, so if the jmp* table is in a DGC-marked region
     * or if -no_inline_calls we will miss these: FIXME).
     * FIXME: what if have PLT-style but no GOT indirection: call,jmp ?!?
     *
     * We try to identify funky call* constructions (like
     * call*,...,jmp* in case 4269) by examining TOS to see whether it's a
     * retaddr -- we do this if last_exit is a jmp* or is unknown (for the
     * target_delete ibl path).
     *
     * FIXME: we will fail to identify a delay-loaded indirect xfer!
     * Need to know dynamic link patchup code to look for.
     *
     * FIXME: we will fail to take over w/ non-call entrances to a dll, like
     * NtContinue or direct jmp from DGC.
     * we could try to take the top-of-stack value and see if it's a retaddr by
     * decoding the prev instr to see if it's a call.  decode backwards may have
     * issues, and if really want everything will have to do this on every bb,
     * not just if lastexit is ind xfer.
     *
     * We count up easy-to-identify cases we've missed in the DOSTATS below.
     */ 
    bool native_exec_bb = false;
    if (DYNAMO_OPTION(native_exec) &&
        !vmvector_empty(native_exec_areas)) {
        /* do we KNOW that we came from an indirect call? */
        if (TEST(LINK_CALL/*includes IND_JMP_PLT*/, dcontext->last_exit->flags) &&
            /* only check direct calls if native_exec_dircalls is on */
            (DYNAMO_OPTION(native_exec_dircalls) ||
             LINKSTUB_INDIRECT(dcontext->last_exit->flags))) {
            STATS_INC(num_native_entrance_checks);
            /* we do the overlap check last since it's more costly */
            if (vmvector_overlap(native_exec_areas, start, start+1)) {
                native_exec_bb = true;
                DOSTATS({
                    if (EXIT_IS_CALL(dcontext->last_exit->flags)) {
                        if (LINKSTUB_INDIRECT(dcontext->last_exit->flags))
                            STATS_INC(num_native_module_entrances_indcall);
                        else
                            STATS_INC(num_native_module_entrances_call);
                    } else
                        STATS_INC(num_native_module_entrances_plt);
                    
                });
            }
        } 
        /* can we GUESS that we came from an indirect call? */
        else if (DYNAMO_OPTION(native_exec_guess_calls) &&
                 (/* FIXME: require jmp* be in separate module? */
                  (LINKSTUB_INDIRECT(dcontext->last_exit->flags) &&
                   EXIT_IS_JMP(dcontext->last_exit->flags)) ||
                  LINKSTUB_FAKE(dcontext->last_exit))) {
            /* if unknown last exit, or last exit was jmp*, examine TOS and guess
             * whether it's a retaddr
             */
            app_pc *tos = (app_pc *) get_mcontext(dcontext)->xsp;
            STATS_INC(num_native_entrance_TOS_checks);
            /* vector check cheaper than is_readable syscall, etc. so do it before them,
             * but after last_exit checks above since overlap is more costly
             */
            if (vmvector_overlap(native_exec_areas, start, start+1) &&
                is_readable_without_exception((app_pc)tos, sizeof(app_pc))) {
                enum { MAX_CALL_CONSIDER = 6 /* ignore prefixes */ };
                app_pc retaddr = *tos;
                LOG(THREAD, LOG_INTERP|LOG_VMAREAS, 2,
                    "at native_exec target: checking TOS "PFX" => "PFX" for retaddr\n",
                    tos, retaddr);
#ifdef RETURN_AFTER_CALL
                if (DYNAMO_OPTION(ret_after_call)) {
                    native_exec_bb = is_observed_call_site(dcontext, retaddr);
                    LOG(THREAD, LOG_INTERP|LOG_VMAREAS, 2,
                        "native_exec: *TOS is %sa call site in ret-after-call table\n",
                        native_exec_bb ? "" : "NOT ");
                } else {
#endif
                    /* try to decode backward -- make sure readable for decoding */
                    if (is_readable_without_exception(retaddr - MAX_CALL_CONSIDER,
                                                      MAX_CALL_CONSIDER +
                                                      MAX_INSTR_LENGTH)) {
                        /* ind calls have variable length and form so we decode
                         * each byte rather than searching for ff and guessing length
                         */
                        app_pc pc, next_pc;
                        instr_t instr;
                        instr_init(dcontext, &instr);
                        for (pc = retaddr - MAX_CALL_CONSIDER; pc < retaddr; pc++) {
                            LOG(THREAD, LOG_INTERP|LOG_VMAREAS, 3,
                                "native_exec: decoding @"PFX" looking for call\n", pc);
                            instr_reset(dcontext, &instr);
                            next_pc = decode_cti(dcontext, pc, &instr);
                            STATS_INC(num_native_entrance_TOS_decodes);
                            if (next_pc == retaddr && instr_is_call(&instr)) {
                                native_exec_bb = true;
                                LOG(THREAD, LOG_INTERP|LOG_VMAREAS, 2,
                                    "native_exec: found call @ pre-*TOS "PFX"\n", pc);
                                break;
                            }
                        }
                        instr_free(dcontext, &instr);
                    }
#ifdef RETURN_AFTER_CALL
                }
#endif
                DOSTATS({
                    if (native_exec_bb) {
                        if (LINKSTUB_FAKE(dcontext->last_exit))
                            STATS_INC(num_native_module_entrances_TOS_unknown);
                        else
                            STATS_INC(num_native_module_entrances_TOS_jmp);
                    }
                });
            }
        }
        DOSTATS({
            /* did we reach a native dll w/o going through an ind call caught above? */
            if (!xfer_target /* else we'll re-check at the target itself */ &&
                !native_exec_bb && vmvector_overlap(native_exec_areas, start, start+1)) {
                LOG(THREAD, LOG_INTERP|LOG_VMAREAS, 2,
                    "WARNING: pc "PFX" is on native list but reached bypassing gateway!\n",
                    start);
                STATS_INC(num_native_entrance_miss);
                /* do-once since once get into dll past gateway may xfer
                 * through a bunch of lastexit-null or indjmp to same dll
                 */
                ASSERT_CURIOSITY_ONCE(false && "inside native_exec dll");
            }
        });
    }

    return native_exec_bb;
}
