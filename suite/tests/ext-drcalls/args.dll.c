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

#define PRE  instrlist_meta_preinsert

static app_pc foo_pc;
static app_pc bar_pc;
static app_pc baz_pc;
static app_pc qux_pc;
static app_pc reg_pc;
static app_pc tls_pc;

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
        app_pc reg = (app_pc)dr_get_proc_address(handle, "reg");
        app_pc tls = (app_pc)dr_get_proc_address(handle, "tls");
        if (foo != NULL) { foo_pc = foo; }
        if (bar != NULL) { bar_pc = bar; }
        if (baz != NULL) { baz_pc = baz; }
        if (qux != NULL) { qux_pc = qux; }
        if (reg != NULL) { reg_pc = reg; }
        if (tls != NULL) { tls_pc = tls; }
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
    DR_ASSERT_MSG(entry_pc == foo_pc, "Bad immediate arg");
}

static void
bar_call(app_pc entry_pc, int a)
{
    dr_fprintf(STDERR, "bar_call\n");
    DR_ASSERT_MSG(entry_pc == bar_pc, "Bad immediate arg");
    DR_ASSERT_MSG(a == 0x11223344, "Bad reg or mem arg");
}

static void
baz_call(int a, int b)
{
    dr_fprintf(STDERR, "baz_call\n");
    DR_ASSERT_MSG(a == 0x11223344, "Bad reg or mem arg");
    DR_ASSERT_MSG(b == 0x55667788, "Bad reg or mem arg");
}

static void
qux_call(int g, int h)
{
    dr_fprintf(STDERR, "qux_call\n");
    DR_ASSERT_MSG(g == 0x11223344, "Bad reg or mem arg");
    DR_ASSERT_MSG(h == 0x55667788, "Bad reg or mem arg");
}

static void
reg_call(int xcx, int xax)
{
    dr_fprintf(STDERR, "reg_call\n");
    DR_ASSERT_MSG(xcx == 0xdeadbeef, "XCX was not handled");
    DR_ASSERT_MSG(xax == 0xcafebabe, "XAX was not handled");
}

static void
tls_call(int slot1, int slot2)
{
    dr_fprintf(STDERR, "tls_call\n");
    DR_ASSERT_MSG(slot1 == 0x12345678, "Bad spill slot arg");
    DR_ASSERT_MSG(slot2 == 0x76543210, "Bad spill slot arg");
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
    if (entry_pc == reg_pc) {
        /* Try setting RAX and RCX and calling.  Pretty much evey x86 calling
         * convention considers these registers clobbered by function calls,
         * and we know the app just called qux, so writing these should be
         * safe.  This test used to fail when we always grabbed RAX as scratch.
         */
        opnd_t reg_xcx = opnd_create_reg(DR_REG_XCX);
        opnd_t reg_xax = opnd_create_reg(DR_REG_XAX);
        PRE(bb, entry, INSTR_CREATE_mov_imm(drcontext, reg_xcx,
                                            OPND_CREATE_INT32((int)0xdeadbeef)));
        PRE(bb, entry, INSTR_CREATE_mov_imm(drcontext, reg_xax,
                                            OPND_CREATE_INT32((int)0xcafebabe)));
        drcalls_shared_call(drcontext, bb, entry, reg_call, 2,
                            reg_xcx, reg_xax);
    }
    if (entry_pc == tls_pc) {
        /* If we're careful to pass spill slots to the clean call in the way
         * that the shared call passes them, then we can do so successfully. */
        opnd_t spill_2 = dr_reg_spill_slot_opnd(drcontext, SPILL_SLOT_2);
        opnd_t spill_3 = dr_reg_spill_slot_opnd(drcontext, SPILL_SLOT_3);
        PRE(bb, entry, INSTR_CREATE_mov_st(drcontext, spill_2,
                                           OPND_CREATE_INT32(0x12345678)));
        PRE(bb, entry, INSTR_CREATE_mov_st(drcontext, spill_3,
                                           OPND_CREATE_INT32(0x76543210)));
        drcalls_shared_call(drcontext, bb, entry, tls_call, 2,
                            spill_2, spill_3);
    }
    return DR_EMIT_DEFAULT;
}
