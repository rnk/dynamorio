/* **********************************************************
 * Copyright (c) 2011 MIT  All rights reserved.
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

#ifndef ASM_CODE_ONLY

#include "tools.h"

#define NUM_ITERATIONS 10

#ifdef WINDOWS
# define EXPORT __declspec(dllexport)
#else
# define EXPORT __attribute__((visibility("default")))
#endif

EXPORT void start_monitor(void) {}
EXPORT void stop_monitor(void) {}

void unaligned_stack_accesses(void);

EXPORT void
foo(char *a, char *b, char *c, char *d)
{
    int i;
    int *ai = (int*)a;
    int *bi = (int*)b;
    int *ci = (int*)c;
    int *di = (int*)d;

    start_monitor();
    for (i = 0; i < NUM_ITERATIONS; i++) {
        *ai = 0x11111111;
        *bi = 0x22222222;
        *ci = 0x33333333;
        *di = 0x44444444;
    }
    unaligned_stack_accesses();
    stop_monitor();
}

int
main(void)
{
    char s[8];  /* Should have at least 4-byte alignment. */
    foo(&s[0], &s[1], &s[2], &s[3]);
    assert(s[0] == 0x11 && s[1] == 0x22 && s[2] == 0x33 && s[3] == 0x44);
}

#else /* ASM_CODE_ONLY */

#include "asm_defines.asm"

START_FILE

DECLARE_FUNC(unaligned_stack_accesses)
GLOBAL_LABEL(unaligned_stack_accesses:)
    enter 16, 0
    sub REG_XSP, 1
    mov REG_XAX, 0
    mov BYTE  [REG_XSP], al   /* 1 byte aligned */
    mov DWORD [REG_XSP], eax  /* unaligned */
    sub REG_XSP, 1
    mov WORD  [REG_XSP], ax   /* 2 byte aligned */
    mov DWORD [REG_XSP], eax  /* unaligned */
    sub REG_XSP, 2
    mov DWORD [REG_XSP], eax  /* 4 byte aligned */
    sub REG_XSP, 1
    push 0  /* catch unaligned pushes as well */
    leave
    ret

END_FILE

#endif
