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

/* Test passing various kinds of arguments to shared, out-of-line clean calls.
 */

/* Argument kinds:
 * - immediate
 * - memory ref
 * - register
 * - base disp, ie register w/ arithmetic
 * - various sizes thereof
 */

#include "dr_api.h"
#include "dr_calls.h"

static app_pc foo_pc;
static app_pc bar_pc;
static app_pc baz_pc;
static app_pc qux_pc;

static void event_exit(void);
static dr_emit_flags_t event_basic_block(void *drcontext, void *tag,
                                         instrlist_t *bb, bool for_trace,
                                         bool translating);
static void lookup_pcs(void);

DR_EXPORT void
dr_init(client_id_t id)
{
    drcalls_init();
    dr_register_exit_event(event_exit);
    dr_register_bb_event(event_basic_block);

    /* Lookup pcs. */
    lookup_pcs();
}

static void
lookup_pcs(void)
{
    dr_module_iterator_t *iter;

    iter = dr_module_iterator_start();
    while (dr_module_iterator_hasnext(iter)) {
        module_data_t *data = dr_module_iterator_next(iter);
        module_handle_t handle = data->handle;
        app_pc foo = (app_pc)dr_get_proc_address(handle, "foo");
        app_pc bar = (app_pc)dr_get_proc_address(handle, "bar");
        app_pc baz = (app_pc)dr_get_proc_address(handle, "baz");
        app_pc qux = (app_pc)dr_get_proc_address(handle, "qux");
        if (foo != NULL) { foo_pc = foo; }
        if (bar != NULL) { bar_pc = bar; }
        if (baz != NULL) { baz_pc = baz; }
        if (qux != NULL) { qux_pc = qux; }
        dr_free_module_data(data);
    }
    dr_module_iterator_stop(iter);

    DR_ASSERT_MSG(foo_pc != NULL &&
                  bar_pc != NULL &&
                  baz_pc != NULL &&
                  qux_pc != NULL,
                  "Didn't find one of our functions!");
}

static void
event_exit(void)
{
    dr_fprintf(STDERR, "PASSED\n");
    drcalls_exit();
}

static void
foo_call(app_pc entry_pc)
{
    dr_fprintf(STDERR, "foo_call\n");
    //DR_ASSERT_MSG(entry_pc == foo_pc, "Bad immediate arg");
}

static void
bar_call(app_pc entry_pc, int a)
{
    dr_fprintf(STDERR, "bar_call\n");
    //DR_ASSERT_MSG(entry_pc == bar_pc, "Bad immediate arg");
    //DR_ASSERT_MSG(a == 0x11223344, "Bad reg or mem arg");
}

static void
baz_call(int a, int b)
{
    dr_fprintf(STDERR, "baz_call\n");
    //DR_ASSERT_MSG(a == 0x11223344, "Bad reg or mem arg");
    //DR_ASSERT_MSG(b == 0x55667788, "Bad reg or mem arg");
}

static void
qux_call(int g, int h)
{
    dr_fprintf(STDERR, "qux_call\n");
    //DR_ASSERT_MSG(g == 0x11223344, "Bad reg or mem arg");
    //DR_ASSERT_MSG(h == 0x55667788, "Bad reg or mem arg");
}

#ifdef X64
#ifdef WINDOWS
enum { N_ARG_REGS = 4 };
static const int arg_regs[N_ARG_REGS] = {
    DR_REG_RCX,
    DR_REG_RDX,
    DR_REG_R8,
    DR_REG_R9,
};
#else /* WINDOWS */
enum { N_ARG_REGS = 6 };
static const int arg_regs[N_ARG_REGS] = {
    DR_REG_RDI,
    DR_REG_RSI,
    DR_REG_RDX,
    DR_REG_RCX,
    DR_REG_R8,
    DR_REG_R9,
};
#endif /* WINDOWS */
#endif /* X64 */

/* Create an opnd_t that refers to the n'th argument of a function with the
 * standard system calling convention. */
static opnd_t
opnd_create_arg(uint arg_index)
{
#ifdef X64
    if (arg_index < N_ARG_REGS) {
        return opnd_create_reg(arg_regs[arg_index]);
    } else {
        arg_index -= N_ARG_REGS;
    }
#endif
    /* This simplistically assumes that all arguments pointer-sized. */
    return OPND_CREATE_MEMPTR(DR_REG_XSP, (arg_index + 1) * sizeof(void*));
}

static dr_emit_flags_t
event_basic_block(void *drcontext, void *tag, instrlist_t *bb,
                  bool for_trace, bool translating)
{
    instr_t *entry = instrlist_first(bb);
    app_pc entry_pc = instr_get_app_pc(entry);
    if (entry_pc == foo_pc) {
        drcalls_shared_call(drcontext, bb, entry, foo_call, 1,
                            OPND_CREATE_INTPTR(entry_pc));
    }
    if (entry_pc == bar_pc) {
        drcalls_shared_call(drcontext, bb, entry, bar_call, 2,
                            OPND_CREATE_INTPTR(entry_pc),
                            opnd_create_arg(0));
    }
    if (entry_pc == baz_pc) {
        drcalls_shared_call(drcontext, bb, entry, baz_call, 2,
                            opnd_create_arg(0),
                            opnd_create_arg(1));
    }
    if (entry_pc == qux_pc) {
        /* We have an 8-arg function which forces the x86_64 sys V calling
         * convention to start pushing args to the stack.  This tests
         * materializing memref operands. */
        drcalls_shared_call(drcontext, bb, entry, qux_call, 2,
                            opnd_create_arg(6),
                            opnd_create_arg(7));
    }
    return DR_EMIT_DEFAULT;
}
