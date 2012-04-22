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

/* Implements DR injection using ptrace API.
 */

#include "../globals.h"
#include "../utils.h"
#include "../os_shared.h"
#include "syscall.h"
#include "instrument.h"
#include "instr.h"
#include "instr_create.h"
#include "decode.h"
#include "disassemble.h"

#include <string.h>
#include <sys/ptrace.h>
#include <sys/wait.h>
#include <unistd.h>
#include <errno.h>
#include <sys/user.h>

static bool verbose = false;

/* Opaque type to users, holds our state */
typedef struct _dr_inject_info_t {
    pid_t pid;
    char image_name[MAXIMUM_PATH];
} dr_inject_info_t;

typedef struct _enum_name_pair_t {
    const int enum_val;
    const char * const enum_name;
} enum_name_pair_t;

/* Ptrace request enum name mapping, from sys/ptrace.h. */
static const enum_name_pair_t pt_req_map[] = {
    {PTRACE_TRACEME,        "PTRACE_TRACEME"},
    {PTRACE_PEEKTEXT,       "PTRACE_PEEKTEXT"},
    {PTRACE_PEEKDATA,       "PTRACE_PEEKDATA"},
    {PTRACE_PEEKUSER,       "PTRACE_PEEKUSER"},
    {PTRACE_POKETEXT,       "PTRACE_POKETEXT"},
    {PTRACE_POKEDATA,       "PTRACE_POKEDATA"},
    {PTRACE_POKEUSER,       "PTRACE_POKEUSER"},
    {PTRACE_CONT,           "PTRACE_CONT"},
    {PTRACE_KILL,           "PTRACE_KILL"},
    {PTRACE_SINGLESTEP,     "PTRACE_SINGLESTEP"},
    {PTRACE_GETREGS,        "PTRACE_GETREGS"},
    {PTRACE_SETREGS,        "PTRACE_SETREGS"},
    {PTRACE_GETFPREGS,      "PTRACE_GETFPREGS"},
    {PTRACE_SETFPREGS,      "PTRACE_SETFPREGS"},
    {PTRACE_ATTACH,         "PTRACE_ATTACH"},
    {PTRACE_DETACH,         "PTRACE_DETACH"},
    {PTRACE_GETFPXREGS,     "PTRACE_GETFPXREGS"},
    {PTRACE_SETFPXREGS,     "PTRACE_SETFPXREGS"},
    {PTRACE_SYSCALL,        "PTRACE_SYSCALL"},
    {PTRACE_SETOPTIONS,     "PTRACE_SETOPTIONS"},
    {PTRACE_GETEVENTMSG,    "PTRACE_GETEVENTMSG"},
    {PTRACE_GETSIGINFO,     "PTRACE_GETSIGINFO"},
    {PTRACE_SETSIGINFO,     "PTRACE_SETSIGINFO"},
    {0}
};

/* Ptrace syscall wrapper, for logging and avoiding libc deps. */
static long
os_ptrace(enum __ptrace_request request, pid_t pid, void *addr, void *data)
{
    long r = dynamorio_syscall(SYS_ptrace, 4, request, pid, addr, data);
    //long r = ptrace(request, pid, addr, data);
    /* TODO: Proper logging. */
    const enum_name_pair_t *pair = NULL;
    int i;
    for (i = 0; pt_req_map[i].enum_name != NULL; i++) {
         if (pt_req_map[i].enum_val == request) {
             pair = &pt_req_map[i];
             break;
         }
    }
    ASSERT(pair != NULL);
    if (verbose) {
        dr_printf("ptrace(%s, %d, %p, %p) -> %d\n",
                  pair->enum_name, pid, addr, data, r);
    }
    return r;
}
#define ptrace use_os_ptrace

/* Returns 0 on success.
 * On failure, returns a Windows API error code.
 */
DYNAMORIO_EXPORT
int
dr_inject_process_create(const char *app_name, const char *app_cmdline,
                         void **data OUT)
{
    dr_inject_info_t *inject_data;
    pid_t pid;
    int status;
    bool success;

    pid = fork();
    if (pid == 0) {
        /* Child. */
        /* TODO: Translate str to argv, or force caller to. */
        char *cmd_line[3];
        cmd_line[0] = (char*)app_name;
        cmd_line[1] = (char*)app_cmdline;
        cmd_line[2] = NULL;
        os_ptrace(PTRACE_TRACEME, 0, NULL, NULL);
        status = execv(app_name, cmd_line);
        dr_printf("returned from execv: %ld\n", status);
        dr_abort();
    }

    /* Parent. */
    inject_data = dr_global_alloc(sizeof(*inject_data));
    inject_data->pid = pid;
    strncpy(inject_data->image_name, app_name,
            BUFFER_SIZE_ELEMENTS(inject_data->image_name));
    *data = inject_data;

    /* We don't need PTRACE_ATTACH because the child will do PTRACE_TRACEME. */
    waitpid(inject_data->pid, &status, 0);
    /* We succeed if the child is stopped, ie it didn't exit or crash.  We
     * return to the user and let them inject DR into the child before
     * continuing.
     */
    success = WIFSTOPPED(status);

    return success;
}

#define APP  instrlist_meta_append

static void
gen_print(void *dc, instrlist_t *ilist, const char *msg)
{
    opnd_t xax = opnd_create_reg(DR_REG_XAX);
    opnd_t rdi = opnd_create_reg(DR_REG_RDI);
    opnd_t rsi = opnd_create_reg(DR_REG_RSI);
    opnd_t rdx = opnd_create_reg(DR_REG_RDX);
    instr_t *next_label = INSTR_CREATE_label(dc);
    instr_t *after_msg = INSTR_CREATE_label(dc);
    instr_t *msg_instr = instr_build_bits(dc, OP_UNDECODED, strlen(msg));
    instr_set_raw_bytes(msg_instr, (byte*)msg, strlen(msg));
    instr_set_raw_bits_valid(msg_instr, true);
    APP(ilist, INSTR_CREATE_call(dc, opnd_create_instr(next_label)));
    APP(ilist, next_label);
    APP(ilist, INSTR_CREATE_jmp_short(dc, opnd_create_instr(after_msg)));
    APP(ilist, msg_instr);
    APP(ilist, after_msg);
    APP(ilist, INSTR_CREATE_pop(dc, rsi));
    APP(ilist, INSTR_CREATE_add(dc, rsi, OPND_CREATE_INT32(2)));

    APP(ilist, INSTR_CREATE_mov_imm(dc, xax, OPND_CREATE_INTPTR(SYS_write)));
    APP(ilist, INSTR_CREATE_mov_imm(dc, rdi, OPND_CREATE_INTPTR(2)));
    /* rsi is set */
    APP(ilist, INSTR_CREATE_mov_imm(dc, rdx, OPND_CREATE_INTPTR(strlen(msg))));
    APP(ilist, INSTR_CREATE_syscall(dc));
}

#define MAX_SHELL_CODE 4096

static app_pc
generate_shellcode(app_pc shellcode, size_t *code_size)
{
    void *dc = GLOBAL_DCONTEXT;
    instrlist_t *ilist = instrlist_create(dc);
    //opnd_t xax = opnd_create_reg(DR_REG_XAX);
    //opnd_t rdi = opnd_create_reg(DR_REG_RDI);
    //opnd_t rsi = opnd_create_reg(DR_REG_RSI);
    //opnd_t rdx = opnd_create_reg(DR_REG_RDX);
    //opnd_t rcx = opnd_create_reg(DR_REG_RCX);
    //opnd_t r8  = opnd_create_reg(DR_REG_R8);
    //opnd_t r9  = opnd_create_reg(DR_REG_R9);
    //opnd_t r10 = opnd_create_reg(DR_REG_R10);

    /* Shift down the stack. */
    gen_print(dc, ilist, "starting syscalls\n");
    //[> open(library_path+"lib64/debug/libdynamorio.so", 0); <]
    //APP(ilist, INSTR_CREATE_mov_imm(dc, xax, OPND_CREATE_INTPTR(SYS_open)));
    //[> mmap(0x71000000, dr_sz, RWX, 0[>flags<], dr_fd, 0); <]
    //APP(ilist, INSTR_CREATE_mov_imm(dc, xax, OPND_CREATE_INTPTR(SYS_mmap)));
    //APP(ilist, INSTR_CREATE_mov_imm(dc, rdi, OPND_CREATE_INTPTR(0x71000000)));
    //APP(ilist, INSTR_CREATE_mov_imm(dc, rsi, OPND_CREATE_INTPTR(PROT_READ|PROT_EXEC|PROT_WRITE)));
    //APP(ilist, INSTR_CREATE_mov_imm(dc, rdx, OPND_CREATE_INTPTR(0x71000000)));
    //shell_syscall(dc, ilist, SYS_mmap, 0x71000000, PROT_READ|PROT_EXEC|PROT_WRITE, 0,
                  //dynamorio_fd, 0);
    APP(ilist, INSTR_CREATE_int3(dc));

    instrlist_encode_to_copy(dc, ilist, shellcode, shellcode, NULL/*max?*/,
                             true/*jmp*/);
    *code_size = MAX_SHELL_CODE;
    instrlist_clear_and_destroy(dc, ilist);
}

DYNAMORIO_EXPORT
bool
dr_inject_process_inject(void *data, bool force_injection,
                         const char *library_path)
{
    dr_inject_info_t *info = (dr_inject_info_t *) data;
    int status;
    app_pc shellcode;
    size_t code_size;
    struct user_regs_struct regs;

    /* Walk the process forwards until we hit the exec syscall.  */
    os_ptrace(PTRACE_SETOPTIONS, info->pid, NULL, (void*)PTRACE_O_TRACEEXEC);
    os_ptrace(PTRACE_SYSCALL, info->pid, NULL, NULL);
    waitpid(info->pid, &status, 0);

    if (!WIFSTOPPED(status))
        return false;

    /* Single-step across the exec. */
    os_ptrace(PTRACE_SYSCALL, info->pid, NULL, NULL);
    waitpid(info->pid, &status, 0);

    /* TODO: inject! */
    shellcode = dr_nonheap_alloc(MAX_SHELL_CODE, MEMPROT_WRITE|MEMPROT_EXEC|MEMPROT_READ);
    generate_shellcode(shellcode, &code_size);
    /* call it? */
    //((void(*)(void))code)();

    os_ptrace(PTRACE_GETREGS, info->pid, NULL, &regs);
    uint i;
    long orig_data[MAX_SHELL_CODE];
    memset(orig_data, 0, sizeof(orig_data));
    for (i = 0; i < code_size / sizeof(long); i++) {
        os_ptrace(PTRACE_PEEKDATA, info->pid, (void*)(regs.rip + i * 8), &orig_data[i]);
        os_ptrace(PTRACE_POKEDATA, info->pid, (void*)(regs.rip + i * 8), (void*)((long*)shellcode)[i]);
    }

    /* Continue until breakpoint. */
    os_ptrace(PTRACE_CONT, info->pid, NULL, NULL);
    waitpid(info->pid, &status, 0);

    /* Put back original stuff. */
    for (i = 0; i < code_size / sizeof(long); i++) {
        os_ptrace(PTRACE_POKEDATA, info->pid, (void*)(regs.rip + i * 8), (void*)orig_data[i]);
    }
    os_ptrace(PTRACE_SETREGS, info->pid, NULL, &regs);

    return WIFSTOPPED(status);
}

DYNAMORIO_EXPORT
bool
dr_inject_process_run(void *data)
{
    dr_inject_info_t *info = (dr_inject_info_t *) data;
    os_ptrace(PTRACE_DETACH, info->pid, NULL, NULL);
    return true;
}

DYNAMORIO_EXPORT
int
dr_inject_process_wait(void *data)
{
    dr_inject_info_t *info = (dr_inject_info_t *) data;
    int r;
    waitpid(info->pid, &r, 0);
    return r;
}

DYNAMORIO_EXPORT
int
dr_inject_process_exit(void *data, bool terminate)
{
    dr_inject_info_t *info = (dr_inject_info_t *) data;
    dr_global_free(info, sizeof(*info));
    /* If the process already terminated, there's no way to re-acquire its exit
     * code after it's been waited on, so we just return -1 and hope the caller
     * doesn't care.
     */
    if (terminate) {
        dr_fprintf(STDERR, "injected child termination unsupported!\n");
    }
    return -1;
}

DYNAMORIO_EXPORT
char *
dr_inject_get_image_name(void *data)
{
    dr_inject_info_t *info = (dr_inject_info_t *) data;
    if (data == NULL)
        return NULL;
    return info->image_name;
}

DYNAMORIO_EXPORT
process_id_t
dr_inject_get_process_id(void *data)
{
    dr_inject_info_t *info = (dr_inject_info_t *) data;
    if (data == NULL)
        return 0;
    return info->pid;
}
