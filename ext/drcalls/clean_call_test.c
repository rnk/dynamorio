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

/* Unit tests for optimize.c. */

#include "dr_api.h"
#include "dr_calls.h"

#include "callee.h"
#include "clean_call.h"
#include "instr_builder.h"
#include "mcontext.h"

#undef ASSERT
#include "unittest.h"

#include <stddef.h>
#include <string.h>

TEST(test_arg_mat) {
    bool regs_clobbered[NUM_GP_REGS];
    ilist_cmp_test_t t;
    uint framesize = sizeof(dr_mcontext_t);

    ilist_cmp_test_init(&t);
    memset(regs_clobbered, 0, sizeof(regs_clobbered));
    regs_clobbered[DR_REG_XAX - DR_REG_XAX] = true;
    regs_clobbered[DR_REG_XBX - DR_REG_XAX] = true;
    regs_clobbered[DR_REG_XSP - DR_REG_XAX] = true;

    t.ib.ilist = t.before;
    materialize_arg_into_reg(t.ib.dcontext, t.before, NULL, regs_clobbered,
                             get_mc_reg_slot, &framesize, REG(XAX),
                             DR_REG_XDI);
    materialize_arg_into_reg(t.ib.dcontext, t.before, NULL, regs_clobbered,
                             get_mc_reg_slot, &framesize, REG(XBX),
                             DR_REG_XDI);
    materialize_arg_into_reg(t.ib.dcontext, t.before, NULL, regs_clobbered,
                             get_mc_reg_slot, &framesize, REG(XCX),
                             DR_REG_XDI);
    materialize_arg_into_reg(t.ib.dcontext, t.before, NULL, regs_clobbered,
                             get_mc_reg_slot, &framesize,
                             MEM(XBX, 0x44, PTR), DR_REG_XDI);
    materialize_arg_into_reg(t.ib.dcontext, t.before, NULL, regs_clobbered,
                             get_mc_reg_slot, &framesize,
                             MEM(XBX, 0x44, lea), DR_REG_XDI);
    materialize_arg_into_reg(t.ib.dcontext, t.before, NULL, regs_clobbered,
                             get_mc_reg_slot, &framesize,
                             MEM(XSP, 0x44, lea), DR_REG_XDI);
    materialize_arg_into_reg(t.ib.dcontext, t.before, NULL, regs_clobbered,
                             get_mc_reg_slot, &framesize,
                             MEM_IDX(XAX, XBX, 1, 0x44, lea), DR_REG_XDI);
    materialize_arg_into_reg(t.ib.dcontext, t.before, NULL, regs_clobbered,
                             get_mc_reg_slot, &framesize,
                             MEM_IDX(XSP, XBX, 1, 0x44, lea), DR_REG_XDI);
    materialize_arg_into_reg(t.ib.dcontext, t.before, NULL, regs_clobbered,
                             get_mc_reg_slot, &framesize,
                             MEM_IDX(XAX, XSP, 1, 0x44, lea), DR_REG_XDI);

    t.ib.ilist = t.after;
    BUILD(t.ib, mov_ld, REG(XDI), MEM(XSP, offsetof(dr_mcontext_t, xax), PTR));
    BUILD(t.ib, mov_ld, REG(XDI), MEM(XSP, offsetof(dr_mcontext_t, xbx), PTR));
    BUILD(t.ib, mov_ld, REG(XDI), REG(XCX));
    /* memref */
    BUILD(t.ib, mov_ld, REG(XDI), MEM(XSP, offsetof(dr_mcontext_t, xbx), PTR));
    BUILD(t.ib, mov_ld, REG(XDI), MEM(XDI, 0x44, PTR));
    /* lea memref */
    BUILD(t.ib, mov_ld, REG(XDI), MEM(XSP, offsetof(dr_mcontext_t, xbx), PTR));
    BUILD(t.ib, lea,    REG(XDI), MEM(XDI, 0x44, lea));
    /* stack lea */
    BUILD(t.ib, mov_ld, REG(XDI), opnd_get_tls_xax(t.dc));
    BUILD(t.ib, lea,    REG(XDI), MEM(XDI, 0x44, lea));
    /* index lea */
    BUILD(t.ib, mov_ld, REG(XDI), MEM(XSP, offsetof(dr_mcontext_t, xax), PTR));
    BUILD(t.ib, mov_ld, REG(XAX), MEM(XSP, offsetof(dr_mcontext_t, xbx), PTR));
    BUILD(t.ib, lea,    REG(XDI), MEM_IDX(XDI, XAX, 1, 0x44, lea));
    /* stack base index */
    BUILD(t.ib, mov_ld, REG(XDI), opnd_get_tls_xax(t.dc));
    BUILD(t.ib, mov_ld, REG(XAX), MEM(XSP, offsetof(dr_mcontext_t, xbx), PTR));
    BUILD(t.ib, lea,    REG(XDI), MEM_IDX(XDI, XAX, 1, 0x44, lea));
    /* base stack index */
    BUILD(t.ib, mov_ld, REG(XDI), MEM(XSP, offsetof(dr_mcontext_t, xax), PTR));
    BUILD(t.ib, mov_ld, REG(XAX), opnd_get_tls_xax(t.dc));
    BUILD(t.ib, lea,    REG(XDI), MEM_IDX(XDI, XAX, 1, 0x44, lea));

    ASSERT(ilists_same(t.dc, t.before, t.after), "arg materialize failed");

    ilist_cmp_test_fini(&t);
}

#define TEST_FUNCS() \
    TEST_FN(test_arg_mat) \

static int
fake_main(void)
{
    RUN_ALL_TESTS_START();
#define TEST_FN RUN_TEST
    TEST_FUNCS();
#undef TEST_FN
    RUN_ALL_TESTS_END();
}

int
main(void)
{
    int r;
    dr_app_setup();
    drcalls_init();

    r = fake_main();

    drcalls_exit();
    dr_app_cleanup();
    return r;
}
