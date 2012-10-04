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

#ifndef ASM_CODE_ONLY

#include "tools.h"

typedef unsigned char byte;

static byte global_buf[8];

void sandbox_ind_call(int i, byte buf[8]);

void
print_int(int a)
{
    int i;
    for (i = 0; i < sizeof(global_buf); i++) {
        if (a != global_buf[i])  /* Can't do more than 256 iters. */
            print("global_buf not set right");
    }
    print("%d\n", a);
}

int
main(void)
{
    int i;

    INIT();

    /* Make sandbox_ind_call code writable */
    protect_mem(sandbox_ind_call, 1024, ALLOW_READ|ALLOW_WRITE|ALLOW_EXEC);

    /* Trigger sandboxing by repeatedly modifying and executing this code.  The
     * default sandbox2ro_threshold is 10.
     */
    for (i = 0; i < 50; i++) {
        sandbox_ind_call(i, global_buf);
    }

    return 0;
}

#else /* ASM_CODE_ONLY */

#include "asm_defines.asm"

START_FILE

DECL_EXTERN(print_int)


    /* The following code needs to cross a page boundary. */
.align 4096
.fill 4080, 1, 0x90  /* nop fill */

#define FUNCNAME sandbox_ind_call
        DECLARE_FUNC(FUNCNAME)
GLOBAL_LABEL(FUNCNAME:)
        mov    REG_XAX, ARG1
        mov    REG_XCX, ARG2
        push   REG_XBP
        push   REG_XDX
        push   REG_XDI  /* for 16-alignment on x64 */
        xchg   REG_XDI, REG_XDI  /* NOCHECKIN */

        .align 4096, 0x90  /* nop fill */

        /* Do enough writes to cause the sandboxing code to split the block. */
        mov    [REG_XCX + 0], al
        mov    [REG_XCX + 1], al
        mov    [REG_XCX + 2], al
        mov    [REG_XCX + 3], al

        lea    REG_XDX, SYMREF(immediate_addr_plus_four - 4)
        mov    DWORD [REG_XDX], eax        /* selfmod write */

        /* More writes to split the block. */
        mov    [REG_XCX + 4], al
        mov    [REG_XCX + 5], al
        mov    [REG_XCX + 6], al
        mov    [REG_XCX + 7], al

        mov    REG_XDX, HEX(0)             /* mov_imm to modify */
ADDRTAKEN_LABEL(immediate_addr_plus_four:)
        lea    REG_XAX, SYMREF(print_int)
        CALLC1(REG_XAX, REG_XDX)

        /* restore */
        pop    REG_XDI
        pop    REG_XDX
        pop    REG_XBP
        ret
        END_FUNC(FUNCNAME)

END_FILE

#endif /* ASM_CODE_ONLY */
