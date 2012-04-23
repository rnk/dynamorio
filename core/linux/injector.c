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
#include "include/syscall.h"            /* our own local copy */
#include "instrument.h"
#include "instr.h"
#include "instr_create.h"
#include "decode.h"
#include "disassemble.h"
#include "os_private.h"

#include <string.h>
#include <sys/ptrace.h>
#include <sys/wait.h>
#include <unistd.h>
#include <errno.h>
#include <sys/user.h>
#include <sys/mman.h>
#include <fcntl.h>

/* FIXME: Avoid duplicating preferred base here. */
#define PREFERRED_BASE ((void*)IF_X64_ELSE(0x71000000, 0x15000000))

/* Shadow DR's internal_error so assertions work in standalone mode.  DR tries
 * to use safe_read to take a stack trace, but none of its signal handlers are
 * installed, so it will segfault before it prints our error.
 */
#define internal_error injector_error
static void
injector_error(char *file, int line, char *expr)
{
    dr_fprintf(STDERR, "ASSERT failed: %s:%d (%s)\n", file, line, expr);
    dr_flush_file(STDERR);
    dr_abort();
}

static bool op_exec_gdb = false;
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

/* Ptrace syscall wrapper, for logging.
 * XXX: We could call libc's ptrace instead of using dynamorio_syscall.
 * Initially I used the raw syscall to avoid adding a libc import, but calling
 * libc from the injector process should always work.
 */
static long
os_ptrace(enum __ptrace_request request, pid_t pid, void *addr, void *data)
{
    long r = dynamorio_syscall(SYS_ptrace, 4, request, pid, addr, data);
    if (r < 0 || verbose) {
        const enum_name_pair_t *pair = NULL;
        int i;
        for (i = 0; pt_req_map[i].enum_name != NULL; i++) {
             if (pt_req_map[i].enum_val == request) {
                 pair = &pt_req_map[i];
                 break;
             }
        }
        ASSERT(pair != NULL);
        dr_fprintf(STDERR, "ptrace failed: ptrace(%s, %d, %p, %p) -> %d %s\n",
                   pair->enum_name, pid, addr, data, r, strerror(-r));
    }
    return r;
}
#define ptrace use_os_ptrace

/* Copies memory from traced process into parent.
 */
static bool
ptrace_read_memory(pid_t pid, void *dst, void *src, size_t len)
{
    uint i;
    reg_t *dst_reg = dst;
    reg_t *src_reg = src;
    ASSERT(len % sizeof(reg_t) == 0);  /* FIXME handle */
    for (i = 0; i < len / sizeof(reg_t); i++) {
        long r = os_ptrace(PTRACE_PEEKDATA, pid, &src_reg[i], &dst_reg[i]);
        if (r < 0)
            return false;
    }
    return true;
}

/* Copies memory from parent into traced process.
 * FIXME: Should return success.
 */
static bool
ptrace_write_memory(pid_t pid, void *dst, void *src, size_t len)
{
    uint i;
    reg_t *dst_reg = dst;
    reg_t *src_reg = src;
    ASSERT(len % sizeof(reg_t) == 0);  /* FIXME handle */
    for (i = 0; i < len / sizeof(reg_t); i++) {
        long r = os_ptrace(PTRACE_POKEDATA, pid, &dst_reg[i], (void*)src_reg[i]);
        if (r < 0)
            return false;
    }
    return true;
}

/* Returns 0 on success.  On failure, returns the status code retreived from
 * waitpid.
 */
DYNAMORIO_EXPORT
int
dr_inject_process_create(const char *app_name, const char *app_cmdline,
                         void **data OUT)
{
    dr_inject_info_t *inject_data;
    pid_t pid;
    int status;

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

    /* We don't need PTRACE_ATTACH because the child will do PTRACE_TRACEME.
     * The child will stop at the next syscall, which for us is execve.
     */
    waitpid(inject_data->pid, &status, 0);

    /* To match Windows behavior, return 0 if the child stopped, and the whole
     * status otherwise.
     */
    return WIFSTOPPED(status) ? 0 : status;
}

#define APP  instrlist_meta_append

/* Push a pointer to a string to the stack.  We create a fake instruction with
 * raw bytes equal to the string we want to put in the injectee.  The call will
 * pass these invalid instruction bytes, and the return address on the stack
 * will point to the string.
 */
static void
gen_push_string(void *dc, instrlist_t *ilist, const char *msg)
{
    instr_t *after_msg = INSTR_CREATE_label(dc);
    instr_t *msg_instr = instr_build_bits(dc, OP_UNDECODED, strlen(msg) + 1);
    APP(ilist, INSTR_CREATE_call(dc, opnd_create_instr(after_msg)));
    instr_set_raw_bytes(msg_instr, (byte*)msg, strlen(msg) + 1);
    instr_set_raw_bits_valid(msg_instr, true);
    APP(ilist, msg_instr);
    APP(ilist, after_msg);
}

static const reg_id_t syscall_parms[] = {
#ifdef X64
    DR_REG_RDI,
    DR_REG_RSI,
    DR_REG_RDX,
    DR_REG_R10,  /* RCX goes here in normal x64 CC. */
    DR_REG_R8,
    DR_REG_R9,
#else
    DR_REG_EBX,
    DR_REG_ECX,
    DR_REG_EDX,
    DR_REG_ESI,
    DR_REG_EDI,
    DR_REG_EBP,
#endif
    DR_REG_NULL
};

static void
gen_syscall(void *dc, instrlist_t *ilist, int sysnum, uint num_opnds,
            opnd_t *args)
{
    uint i;

    APP(ilist, INSTR_CREATE_mov_imm
        (dc, opnd_create_reg(DR_REG_XAX), OPND_CREATE_INTPTR(sysnum)));
    for (i = 0; i < num_opnds; i++) {
        if (opnd_is_immed_int(args[i]) || opnd_is_instr(args[i])) {
            APP(ilist, INSTR_CREATE_mov_imm
                (dc, opnd_create_reg(syscall_parms[i]), args[i]));
        } else if (opnd_is_base_disp(args[i])) {
            APP(ilist, INSTR_CREATE_mov_ld
                (dc, opnd_create_reg(syscall_parms[i]), args[i]));
        }
    }
    APP(ilist, INSTR_CREATE_syscall(dc));
}

/* Useful for debugging gen_syscall and gen_push_string. */
#if 0
static void
gen_print(void *dc, instrlist_t *ilist, const char *msg)
{
    opnd_t args[3];
    args[0] = OPND_CREATE_INTPTR(2);
    args[1] = OPND_CREATE_MEMPTR(DR_REG_XSP, 0);  /* msg is on TOS. */
    args[2] = OPND_CREATE_INTPTR(strlen(msg));
    gen_push_string(dc, ilist, msg);
    gen_syscall(dc, ilist, SYS_write, BUFFER_SIZE_ELEMENTS(args), args);
}
#endif

#define MAX_SHELL_CODE 4096

/* Injects the code in ilist into the injectee and runs it, returning the value
 * left in xax at the end of ilist execution.  Frees ilist.
 */
static ptr_int_t
injectee_run_get_xax(dr_inject_info_t *info, void *dc, instrlist_t *ilist)
{
    struct user_regs_struct regs;
    byte shellcode[MAX_SHELL_CODE];
    reg_t orig_code[MAX_SHELL_CODE / sizeof(reg_t)];
    app_pc end_pc;
    size_t code_size;
    ptr_int_t xax;
    int status;

    /* Append an int3 so we can catch the break. */
    APP(ilist, INSTR_CREATE_int3(dc));
    if (verbose) {
        dr_fprintf(STDERR, "injecting code:\n");
        instrlist_disassemble(dc, (void*)regs.rip, ilist, STDERR);
    }

    /* Get register state before executing the shellcode. */
    os_ptrace(PTRACE_GETREGS, info->pid, NULL, &regs);

    /* Encode ilist into shellcode. */
    end_pc = instrlist_encode_to_copy(dc, ilist, shellcode, (app_pc)regs.rip,
                                      &shellcode[MAX_SHELL_CODE], true/*jmp*/);
    code_size = end_pc - &shellcode[0];
    code_size = ALIGN_FORWARD(code_size, sizeof(reg_t));
    instrlist_clear_and_destroy(dc, ilist);

    /* Copy shell code into injectee at the current PC.
     * XXX: We could inspect /proc/pid/maps to find some executable memory, but
     * this is easier.
     */
    ptrace_read_memory(info->pid, orig_code, (void*)regs.rip, code_size);
    ptrace_write_memory(info->pid, (void*)regs.rip, shellcode, code_size);

    /* Continue until breakpoint. */
    os_ptrace(PTRACE_CONT, info->pid, NULL, NULL);
    waitpid(info->pid, &status, 0);
    if (!WIFSTOPPED(status) || WSTOPSIG(status) != SIGTRAP) {
        ptr_int_t pc;
        os_ptrace(PTRACE_PEEKUSER, info->pid,
                  (void*)offsetof(struct user_regs_struct, rip), &pc);
        dr_fprintf(STDERR, "Unexpected trace event, expected SIGTRAP, got:\n"
                   "status: 0x%x, stopped by signal: %d, at pc: %p\n",
                   status, WIFSTOPPED(status), WSTOPSIG(status), pc);
        return -EUNATCH;  /* Some unique errno value. */
    }

    /* Get xax. */
    xax = -EUNATCH;
    os_ptrace(PTRACE_PEEKUSER, info->pid,
              (void*)offsetof(struct user_regs_struct, rax), &xax);

    /* Put back original code and registers. */
    ptrace_write_memory(info->pid, (void*)regs.rip, orig_code, code_size);
    os_ptrace(PTRACE_SETREGS, info->pid, NULL, &regs);

    return xax;
}

/* Call sys_open in the child. */
static ptr_int_t
injectee_open(dr_inject_info_t *info, const char *path, int flags, mode_t mode)
{
    void *dc = GLOBAL_DCONTEXT;
    instrlist_t *ilist = instrlist_create(dc);
    opnd_t args[3];
    int num_args;

    gen_push_string(dc, ilist, path);
    num_args = 0;
    args[num_args++] = OPND_CREATE_MEMPTR(DR_REG_XSP, 0);
    args[num_args++] = OPND_CREATE_INTPTR(flags);
    args[num_args++] = OPND_CREATE_INTPTR(mode);
    gen_syscall(dc, ilist, SYS_open, num_args, args);

    return injectee_run_get_xax(info, dc, ilist);
}

static void *
injectee_mmap(dr_inject_info_t *info, void *addr, size_t sz, int prot,
              int flags, int fd, off_t offset)
{
    void *dc = GLOBAL_DCONTEXT;
    instrlist_t *ilist = instrlist_create(dc);
    opnd_t args[6];
    int num_args;

    num_args = 0;
    args[num_args++] = OPND_CREATE_INTPTR(addr);
    args[num_args++] = OPND_CREATE_INTPTR(sz);
    args[num_args++] = OPND_CREATE_INTPTR(prot);
    args[num_args++] = OPND_CREATE_INTPTR(flags); /* flags */
    args[num_args++] = OPND_CREATE_INTPTR(fd); /* fd from open */
    args[num_args++] = OPND_CREATE_INTPTR(offset); /* offset */
    gen_syscall(dc, ilist, SYS_mmap, num_args, args);

    return (void*)injectee_run_get_xax(info, dc, ilist);
}

#if 0
typedef ptr_int_t (*non_void_fn_t)();

/* Call a DR function in the injectee.
 */
static ptr_int_t
injectee_call_dr_fn(dr_inject_info_t *info, non_void_fn_t dr_fn)
{
    void *dc = GLOBAL_DCONTEXT;
    instrlist_t *ilist = instrlist_create(dc);
    /* FIXME: Assumes that DR has the same load base in this process as well as
     * injectee.  Should subtract current base and add injectee base.
     */
    APP(ilist, INSTR_CREATE_mov_imm
        (dc, opnd_create_reg(DR_REG_XAX), OPND_CREATE_INTPTR(dr_fn)));
    APP(ilist, INSTR_CREATE_call_ind(dc, opnd_create_reg(DR_REG_XAX)));
    return injectee_run_get_xax(info, dc, ilist);
}
#endif

#if 0
static ssize_t
get_file_size(const char *path)
{
    file_t f;
    uint64_t size;
    bool success;

    f = dr_open_file(path, OS_OPEN_READ);
    if (f == INVALID_FILE)
        return -1;
    success = dr_file_size(f, &size);
    if (!success)
        return -1;
    dr_close_file(f);
    return size;
}
#endif

/* Struct containing info passed from the injector to _dr_start in the injectee.
 */
typedef struct _inject_cxt_t {
    /* FIXME: We can easily translate to priv_mcontext_t in the injector, making
     * our code more portable.
     */
    struct user_regs_struct regs;

    /* We know these values at injection time, so we pass them in and initialize
     * them rather than attempting to parse /proc/self/maps, which requires
     * sscanf from libc.
     */
    app_pc dynamorio_dll_start;
    app_pc dynamorio_dll_end;
    char dynamorio_library_path[MAXIMUM_PATH];
} inject_cxt_t;

/* NOCHECKIN: Put in os_private.h? */
extern char **stack_env_vars;
void os_loader_finish_injection(void);

ptr_int_t
_dr_start(inject_cxt_t *cxt)
{
    /* Wait for debugger.  Ideally we'd use PTRACE_TRACEME, but that confuses
     * gdb since it uses PTRACE_ATTACH, which will fail if we're already being
     * traced.
     */
    int i;
    for (i = 0; i < 100000000; i++) {
    }

    dr_printf("hello, dr_printf, %d %x\n", 0xdead, 0xdead);

    void **orig_sp = (void**)cxt->regs.rsp;
    int argc = (long)orig_sp[0];
    char **argv = (char**)&orig_sp[1];
    char **envp = (char**)&orig_sp[2];
    for (i = 0; i < argc; i++) {
        dr_printf("argv[%d]: %s\n", i, argv[i]);
    }

    dr_early_injected = true;
    stack_env_vars = envp;
    os_inject_init(cxt->dynamorio_dll_start, cxt->dynamorio_dll_end,
                   cxt->dynamorio_library_path);

    acquire_recursive_lock(&privload_lock);
    dynamorio_app_init();
    release_recursive_lock(&privload_lock);

    dr_printf("called os_loader_finish_injection\n");

    dr_printf("attempting to use libc\n");
    int hex, dec;
    asm ("int3");
    int r = sscanf("0xdead 1234", "0x%x %d", &hex, &dec);
    ASSERT(hex == 0xdead && dec == 1234);

    int init = dynamorio_app_init();
    dr_fprintf(STDERR, "init: %d\n", init);

    /* Should send control back to the injector, unless we use gdb to continue
     * past it.
     */
    asm ("int3");
    /* Better to exit than crash. */
    //dynamorio_syscall(SYS_exit, 0);
    /* Not reachable. */
    return 0xdead;
}

static void *
translate_dr_dll(void *injected_base, void *dr_addr)
{
    app_pc dr_base = get_dynamorio_dll_start();
    return ((app_pc)dr_addr - dr_base) + injected_base;
}

static int injected_dr_fd;
static dr_inject_info_t *injected_info;

static byte *
injectee_map_file(file_t f, size_t *size INOUT, uint64 offs, app_pc addr,
                  uint prot, bool copy_on_write, bool image, bool fixed)
{
    int fd;
    int flags = 0;
    ptr_int_t r;
    if (copy_on_write)
        flags |= MAP_PRIVATE;
    if (fixed)
        flags |= MAP_FIXED;
    /* Assumes that f is for libdynamorio.so. */
    if (f == -1) {
        fd = -1;
        flags |= MAP_ANONYMOUS;
    } else {
        fd = injected_dr_fd;
    }
    /* image is a nop on Linux. */
    r = (ptr_int_t)injectee_mmap(injected_info, addr, *size,
                                 memprot_to_osprot(prot), flags, fd, offs);
    if (r < 0) {
        dr_printf("injectee_mmap(%p, %p) -> %p\n", addr, *size, r);
        return NULL;
    }
    return (byte*)r;
}

/* Do an munmap syscall in the injectee. */
static bool
injectee_unmap(byte *addr, size_t size)
{
    void *dc = GLOBAL_DCONTEXT;
    instrlist_t *ilist = instrlist_create(dc);
    opnd_t args[2];
    ptr_int_t r;
    int num_args = 0;
    args[num_args++] = OPND_CREATE_INTPTR(addr);
    args[num_args++] = OPND_CREATE_INTPTR(size);
    gen_syscall(dc, ilist, SYS_munmap, num_args, args);
    r = injectee_run_get_xax(injected_info, dc, ilist) == 0;
    if (r < 0) {
        dr_printf("injectee_munmap(%p, %p) -> %p\n", addr, size, r);
        return false;
    }
    return true;
}

/* Do an mprotect syscall in the injectee. */
static bool
injectee_prot(byte *addr, size_t size, uint prot/*MEMPROT_*/)
{
    void *dc = GLOBAL_DCONTEXT;
    instrlist_t *ilist = instrlist_create(dc);
    opnd_t args[3];
    ptr_int_t r;
    int num_args = 0;
    args[num_args++] = OPND_CREATE_INTPTR(addr);
    args[num_args++] = OPND_CREATE_INTPTR(size);
    args[num_args++] = OPND_CREATE_INTPTR(memprot_to_osprot(prot));
    gen_syscall(dc, ilist, SYS_mprotect, num_args, args);
    r = injectee_run_get_xax(injected_info, dc, ilist) == 0;
    if (r < 0) {
        dr_printf("injectee_prot(%p, %p, %x) -> %d\n", addr, size, prot, r);
        return false;
    }
    return true;
}

DYNAMORIO_EXPORT
bool
dr_inject_process_inject(void *data, bool force_injection,
                         const char *library_path)
{
    dr_inject_info_t *info = (dr_inject_info_t *) data;

    /* Open libdynamorio.so as readonly in the child. */
    int dr_fd = injectee_open(info, library_path, O_RDONLY, 0);
    if (dr_fd < 0) {
        dr_fprintf(STDERR, "unable to open libdynamorio.so in injectee. "
                   "errno %d msg %s\n", -dr_fd, strerror(-dr_fd));
        return false;
    }

    /* Call our private loader, but perform the mmaps in the child process
     * instead of the parent.
     */
    size_t loaded_size;
    app_pc injected_base;
    /* XXX: Have to use globals to communicate to injectee_map_file. =/ */
    injected_dr_fd = dr_fd;
    injected_info = info;
    acquire_recursive_lock(&privload_lock);
    injected_base = linux_map_and_relocate(library_path, &loaded_size,
                                           injectee_map_file, injectee_unmap,
                                           injectee_prot);
    release_recursive_lock(&privload_lock);

    struct user_regs_struct regs;
    os_ptrace(PTRACE_GETREGS, info->pid, NULL, &regs);

    if (verbose) {
        dr_printf("regs.r15: 0x%lx\n", regs.r15);
        dr_printf("regs.r14: 0x%lx\n", regs.r14);
        dr_printf("regs.r13: 0x%lx\n", regs.r13);
        dr_printf("regs.r12: 0x%lx\n", regs.r12);
        dr_printf("regs.rbp: 0x%lx\n", regs.rbp);
        dr_printf("regs.rbx: 0x%lx\n", regs.rbx);
        dr_printf("regs.r11: 0x%lx\n", regs.r11);
        dr_printf("regs.r10: 0x%lx\n", regs.r10);
        dr_printf("regs.r9 : 0x%lx\n", regs.r9);
        dr_printf("regs.r8 : 0x%lx\n", regs.r8);
        dr_printf("regs.rax: 0x%lx\n", regs.rax);
        dr_printf("regs.rcx: 0x%lx\n", regs.rcx);
        dr_printf("regs.rdx: 0x%lx\n", regs.rdx);
        dr_printf("regs.rsi: 0x%lx\n", regs.rsi);
        dr_printf("regs.rdi: 0x%lx\n", regs.rdi);
        dr_printf("regs.rsp: 0x%lx\n", regs.rsp);
    }

    /* Create an injection context and "push" it onto the stack of the injectee.
     * If you need to pass more info to the injected child process, this is a
     * good place to put it.
     */
    inject_cxt_t cxt = {0};
    memcpy(&cxt.regs, &regs, sizeof(regs));
    cxt.dynamorio_dll_start = injected_base;
    cxt.dynamorio_dll_end = injected_base + loaded_size;
    strncpy(cxt.dynamorio_library_path, library_path, MAXIMUM_PATH);
    regs.rsp -= sizeof(cxt); /* Allocate space for cxt. */
    regs.rsp = ALIGN_BACKWARD(regs.rsp, 16);  /* Align stack. */
    regs.rdi = regs.rsp;  /* Pass the address of cxt as parameter 1. */
    ptrace_write_memory(info->pid, (void*)regs.rsp, &cxt, sizeof(cxt));
    void *injected_dr_start = translate_dr_dll(injected_base, &_dr_start);
    regs.rip = (reg_t)injected_dr_start;
    os_ptrace(PTRACE_SETREGS, info->pid, NULL, &regs);

    op_exec_gdb = true;
    if (op_exec_gdb) {
        /* Initializing in the child will be tough, and we can't attach gdb to a
         * process under ptrace, so for now we re-exec ourselves as gdb and the
         * child does another PTRACE_TRACEME.
         */
        char pid_str[10];
        char *argv[10];
        int num_args = 0;
        os_ptrace(PTRACE_DETACH, info->pid, NULL, NULL);
        dr_snprintf(pid_str, sizeof(pid_str), "%d", info->pid);
        argv[num_args++] = "/usr/bin/gdb";
        argv[num_args++] = "--quiet";
        argv[num_args++] = "--pid";
        argv[num_args++] = pid_str;
        argv[num_args++] = "-ex";
        argv[num_args++] = "python execfile(\"drinject_add_syms\")";
        argv[num_args++] = NULL;
        execv("/usr/bin/gdb", argv);
        ASSERT(false && "failed to exec gdb?");
    }

    os_ptrace(PTRACE_CONT, info->pid, NULL, NULL);
    int status;
    waitpid(info->pid, &status, 0);
    if (!WIFSTOPPED(status) || WSTOPSIG(status) != SIGTRAP) {
        ptr_int_t pc;
        os_ptrace(PTRACE_PEEKUSER, info->pid,
                  (void*)offsetof(struct user_regs_struct, rip), &pc);
        dr_fprintf(STDERR, "Unexpected trace event, expected SIGTRAP, got:\n"
                   "status: 0x%x, stopped: %d, by signal: %d, at pc: %p\n",
                   status, WIFSTOPPED(status), WSTOPSIG(status), pc);
        return false;
    }

    //int init = injectee_call_dr_fn(info, injected_dr_start);
    //dr_printf("_dr_start: 0x%x\n", init);

    /* Reset back to initial state (for now, eventually adjust state to start in
     * DR).
     */
    os_ptrace(PTRACE_SETREGS, info->pid, NULL, &cxt.regs);

    return true;
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
