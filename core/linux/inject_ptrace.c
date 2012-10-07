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
    //os_loader_finish_injection();
    release_recursive_lock(&privload_lock);

    dr_printf("called os_loader_finish_injection\n");

    dr_printf("attempting to use libc\n");
    int hex, dec;
    asm ("int3");

    int my_sscanf(const char *str, char const *fmt0, ...);
    int r = my_sscanf("0xdead 1234", "0x%x %d", &hex, &dec);
    ASSERT(r == 2);
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
    injected_base = map_elf_phdrs(library_path, true/*fixed*/, &loaded_size,
                                  &load_delta, NULL/*text addr*/,
                                  injectee_map_file, injectee_unmap,
                                  injectee_prot);

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
    inject_cxt_t cxt;
    memset(&cxt, 0, sizeof(cxt));
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
