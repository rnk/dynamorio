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

/* List of instrumented functions. */
#define FUNCTIONS() \
        FUNCTION(empty) \
        FUNCTION(inscount) \
        FUNCTION(callpic_pop) \
        FUNCTION(callpic_mov) \
        FUNCTION(nonleaf) \
        FUNCTION(cond_br) \
        FUNCTION(tls_clobber) \
        FUNCTION(aflags_clobber) \
        LAST_FUNCTION()

#ifndef ASM_CODE_ONLY

#include "tools.h"

/* Create an enum for each function. */
#define FUNCTION(fn_name) FN_##fn_name,
#define LAST_FUNCTION() LAST_FUNC_ENUM
enum {
    FUNCTIONS()
};
#undef FUNCTION
#undef LAST_FUNCTION

/* A separate define so ctags can find it. */
#define N_FUNCS LAST_FUNC_ENUM

/* Declarations for every function. */
#define FUNCTION(FUNCNAME) void FUNCNAME(void);
#define LAST_FUNCTION()
FUNCTIONS()
#undef FUNCTION
#undef LAST_FUNCTION

/* Table of function names. */
#define FUNCTION(fn_name) #fn_name,
#define LAST_FUNCTION() NULL
static const char *func_names[] = {
    FUNCTIONS()
};
#undef FUNCTION
#undef LAST_FUNCTION

typedef void (*void_func_t)(void);

/* Table of function pointers. */
#define FUNCTION(FUNCNAME) FUNCNAME,
#define LAST_FUNCTION() NULL
static const void_func_t func_ptrs[] = {
    FUNCTIONS()
};
#undef FUNCTION
#undef LAST_FUNCTION

static const char *reg_names[] = {
    "xdi", "xsi", "xbp", "xsp", "xbx", "xdx", "xcx", "xax",
#ifdef X64
    "r8 ", "r9 ", "r10", "r11", "r12", "r13", "r14", "r15",
#endif
    NULL
};

typedef struct _regs_t {
    ptr_int_t xdi;
    ptr_int_t xsi;
    ptr_int_t xbp;
    ptr_int_t xsp;
    ptr_int_t xbx;
    ptr_int_t xdx;
    ptr_int_t xcx;
    ptr_int_t xax;
#ifdef X64
    ptr_int_t r8 ;
    ptr_int_t r9 ;
    ptr_int_t r10;
    ptr_int_t r11;
    ptr_int_t r12;
    ptr_int_t r13;
    ptr_int_t r14;
    ptr_int_t r15;
#endif
} regs_t;

/* Calls 'func' and checks that all register values are preserved.
 */
void save_regs_around_call(void (*func)(void), regs_t *before,
                           regs_t *after);

int
main(void)
{
    int i;
    for (i = 0; i < N_FUNCS; i++) {
        regs_t before, after;
        print("Calling func %s...\n", func_names[i]);
        save_regs_around_call(func_ptrs[i], &before, &after);

        /* xsp should be off by exactly the size of one regs_t struct. */
        after.xsp += sizeof(regs_t);
        if (memcmp(&before, &after, sizeof(regs_t))) {
            ptr_int_t *before_arr = (ptr_int_t*)&before;
            ptr_int_t *after_arr = (ptr_int_t*)&after;
            int j, num_regs = IF_X64_ELSE(16, 8);

            print("Registers clobbered by supposedly clean call!\n");
            for (j = 0; j < num_regs; j++) {
                const char *diff_str = (before_arr[j] == after_arr[j] ?
                                        "" : " <- DIFFERS");
                print("%s before: "PFX" after: "PFX"%s\n",
                      reg_names[j], before_arr[j], after_arr[j], diff_str);
            }
        }
        print("Called func %s.\n", func_names[i]);
    }
}

#else /* ASM_CODE_ONLY */

#include "asm_defines.asm"

START_FILE

DECL_EXTERN(memcpy)

/* All instrumentation functions are a single 'ret'.  This ensures that they
 * don't modify any registers. */
#define FUNCTION(FUNCNAME) \
    DECLARE_EXPORTED_FUNC(FUNCNAME) @N@\
    GLOBAL_LABEL(FUNCNAME:) @N@\
        ret @N@\
    END_FUNC(FUNCNAME) @N@@N@
#define LAST_FUNCTION()
FUNCTIONS()
#undef FUNCTION
#undef LAST_FUNCTION

/* PUSHALL pushes registers in regs_t order, which is the same as
 * dr_mcontext_t order.  Ideally we could share this with core/x86/x86.asm, but
 * including that and dr_api.h from here is tricky.
 */
#ifdef X64
# define PUSHALL \
        push     r15 @N@\
        push     r14 @N@\
        push     r13 @N@\
        push     r12 @N@\
        push     r11 @N@\
        push     r10 @N@\
        push     r9  @N@\
        push     r8  @N@\
        push     rax @N@\
        push     rcx @N@\
        push     rdx @N@\
        push     rbx @N@\
        /* not the pusha pre-push rsp value but see above */ @N@\
        push     rsp @N@\
        push     rbp @N@\
        push     rsi @N@\
        push     rdi
# define REGS_T_SIZE (16 * ARG_SZ)
#else
# define PUSHALL \
        pusha
# define REGS_T_SIZE (8 * ARG_SZ)
#endif

DECLARE_FUNC(save_regs_around_call)
GLOBAL_LABEL(save_regs_around_call:)
    /* Copy args into caller-saved regs before adjusting esp, as the ARG macros
     * don't expect XBP to be on the stack. */
    mov REG_XAX, ARG1  /* func */
    mov REG_XCX, ARG2  /* before */
    mov REG_XDX, ARG3  /* after */

    /* 2 locals: before, after */
    enter (2 * ARG_SZ), 0

    /* Save before/after to stack across calls. */
    mov [REG_XBP - ARG_SZ],     REG_XCX  /* before */
    mov [REG_XBP - 2 * ARG_SZ], REG_XDX  /* after */

    /* Save all our registers, do the call, and do it again. */
    PUSHALL
    call REG_XAX
    PUSHALL

    /* copy out before regs */
    lea REG_XAX, [REG_XSP + REGS_T_SIZE]
    CALLC3(memcpy, [REG_XBP - ARG_SZ], REG_XAX, REGS_T_SIZE)
    /* copy out after regs */
    lea REG_XAX, [REG_XSP]
    CALLC3(memcpy, [REG_XBP - 2 * ARG_SZ], REG_XAX, REGS_T_SIZE)

    leave
    ret
END_FUNC(save_regs_around_call)

END_FILE

#endif
