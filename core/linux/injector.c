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

/* Simple reimplementation of the dr_inject API for Linux.
 *
 * To match the Windows API, we fork a child and suspend it before the call to
 * exec.
 */

#include "configure.h"
#include "globals_shared.h"
#include "../config.h"  /* for get_config_val_other_app */
#include "../globals.h"
#include "include/syscall.h"  /* for SYS_ptrace */
#include "instrument.h"
#include "instr.h"
#include "instr_create.h"
#include "decode.h"
#include "disassemble.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/ptrace.h>
#include <sys/user.h>
#include <sys/wait.h>
#include <unistd.h>

typedef enum _inject_method_t {
    INJECT_EXEC_DR,     /* Works with self or child. */
    INJECT_LD_PRELOAD,  /* Works with self or child.  FIXME i#840: NYI */
    INJECT_PTRACE       /* Doesn't work with exec_self. */
} inject_method_t;

/* Opaque type to users, holds our state */
typedef struct _dr_inject_info_t {
    process_id_t pid;
    const char *exe;            /* points to user data */
    const char *image_name;     /* basename of exe */
    const char **argv;          /* points to user data */
    int pipe_fd;

    bool exec_self;             /* this process will exec the app */
    inject_method_t method;
} dr_inject_info_t;

bool
inject_ptrace(dr_inject_info_t *info, const char *library_path);

static long
os_ptrace(enum __ptrace_request request, pid_t pid, void *addr, void *data);

/* Shadow DR's internal_error so assertions work in standalone mode.  DR tries
 * to use safe_read to take a stack trace, but none of its signal handlers are
 * installed, so it will segfault before it prints our error.
 */
void
internal_error(char *file, int line, char *expr)
{
    fprintf(stderr, "ASSERT failed: %s:%d (%s)\n", file, line, expr);
    fflush(stderr);
    abort();
}

void
report_dynamorio_problem(dcontext_t *dcontext, uint dumpcore_flag,
                         app_pc exception_addr, app_pc report_ebp,
                         const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "DynamoRIO problem: ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
    fflush(stderr);
    abort();
}

static process_id_t
fork_suspended_child(const char *exe, const char **argv, int fds[2])
{
    process_id_t pid = fork();
    if (pid == 0) {
        /* child, suspend before exec */
        char pipe_cmd[MAXIMUM_PATH];
        ssize_t nread;
        size_t sofar = 0;
        char *real_exe;
        close(fds[1]);  /* Close writer in child, keep reader. */
        do {
            nread = read(fds[0], pipe_cmd + sofar,
                         BUFFER_SIZE_BYTES(pipe_cmd) - sofar);
            sofar += nread;
        } while (nread > 0 && sofar < BUFFER_SIZE_BYTES(pipe_cmd)-1);
        pipe_cmd[sofar] = '\0';
        close(fds[0]);  /* Close reader before exec. */
        if (pipe_cmd[0] == '\0') {
            /* If nothing was written to the pipe, let it run natively. */
            real_exe = (char *) exe;
        } else if (strcmp("ptrace", pipe_cmd) == 0) {
            real_exe = (char *) exe;
            os_ptrace(PTRACE_TRACEME, 0, NULL, NULL);
        } else if (strncmp("exec_dr ", pipe_cmd, 8) == 0) {
            setenv(DYNAMORIO_VAR_EXE_PATH, exe, true/*overwrite*/);
            real_exe = pipe_cmd;
        }
        execv(real_exe, (char **) argv);
        /* If execv returns, there was an error. */
        exit(-1);
    }
    return pid;
}

static void
write_pipe_cmd(int pipe_fd, const char *cmd)
{
    ssize_t towrite = strlen(cmd);
    ssize_t written = 0;
    while (towrite > 0) {
        ssize_t nwrote = write(pipe_fd, cmd + written, towrite);
        if (nwrote <= 0)
            break;
        towrite -= nwrote;
        written += nwrote;
    }
}

static bool
inject_exec_dr(dr_inject_info_t *info, const char *library_path)
{
    if (info->exec_self) {
        /* exec DR with the original command line and set an environment
         * variable pointing to the real exe.
         */
        /* XXX: setenv will modify the environment on failure. */
        setenv(DYNAMORIO_VAR_EXE_PATH, info->exe, true/*overwrite*/);
        execv(library_path, (char **) info->argv);
        return false;  /* if execv returns, there was an error */
    } else {
        /* Write the path to DR to the pipe. */
        char cmd[MAXIMUM_PATH];
        snprintf(cmd, BUFFER_SIZE_ELEMENTS(cmd), "exec_dr %s", library_path);
        write_pipe_cmd(info->pipe_fd, cmd);
    }
    return true;
}

static void
set_exe_and_argv(dr_inject_info_t *info, const char *exe, const char **argv)
{
    info->exe = exe;
    info->argv = argv;
    info->image_name = strrchr(exe, '/');
    if (info->image_name == NULL)
        info->image_name = exe;
}

/* Returns 0 on success.
 */
DR_EXPORT
int
dr_inject_process_create(const char *exe, const char **argv, void **data OUT)
{
    int r;
    int fds[2];
    dr_inject_info_t *info = malloc(sizeof(*info));
    set_exe_and_argv(info, exe, argv);

    /* Create a pipe to a forked child and have it block on the pipe. */
    r = pipe(fds);
    if (r != 0)
        goto error;
    info->pid = fork_suspended_child(exe, argv, fds);
    close(fds[0]);  /* Close reader, keep writer. */
    info->pipe_fd = fds[1];
    info->exec_self = false;
    info->method = INJECT_EXEC_DR;

    if (info->pid == -1)
        goto error;
    *data = info;
    return 0;

error:
    free(info);
    return errno;
}

DR_EXPORT
int
dr_inject_prepare_to_exec(const char *exe, const char **argv, void **data OUT)
{
    dr_inject_info_t *info = malloc(sizeof(*info));
    set_exe_and_argv(info, exe, argv);
    info->pid = getpid();
    info->pipe_fd = 0;  /* No pipe. */
    info->exec_self = true;
    info->method = INJECT_EXEC_DR;
    *data = info;
    return 0;
}

DR_EXPORT
bool
dr_inject_use_ptrace(void *data)
{
    dr_inject_info_t *info = (dr_inject_info_t *) data;
    if (data == NULL)
        return false;
    if (info->exec_self)
        return false;
    info->method = INJECT_PTRACE;
    return true;
}

DR_EXPORT
process_id_t
dr_inject_get_process_id(void *data)
{
    dr_inject_info_t *info = (dr_inject_info_t *) data;
    return info->pid;
}

DR_EXPORT
char *
dr_inject_get_image_name(void *data)
{
    dr_inject_info_t *info = (dr_inject_info_t *) data;
    return (char *) info->image_name;
}

DR_EXPORT
bool
dr_inject_process_inject(void *data, bool force_injection,
                         const char *library_path)
{
    dr_inject_info_t *info = (dr_inject_info_t *) data;
    char dr_path_buf[MAXIMUM_PATH];

    /* Read the autoinject var from the config file if the caller didn't
     * override it.
     */
    if (library_path == NULL) {
        if (!get_config_val_other_app(info->image_name, info->pid,
                                      DYNAMORIO_VAR_AUTOINJECT, dr_path_buf,
                                      BUFFER_SIZE_ELEMENTS(dr_path_buf), NULL,
                                      NULL, NULL)) {
            return false;
        }
        library_path = dr_path_buf;
    }

    switch (info->method) {
    case INJECT_EXEC_DR:
        return inject_exec_dr(info, library_path);
    case INJECT_LD_PRELOAD:
        return false;  /* NYI */
    case INJECT_PTRACE:
        return inject_ptrace(info, library_path);
    }

    return false;
}

DR_EXPORT
bool
dr_inject_process_run(void *data)
{
    dr_inject_info_t *info = (dr_inject_info_t *) data;
    if (info->exec_self) {
        /* Let the app run natively if we haven't already injected. */
        execv(info->image_name, (char **) info->argv);
        return false;  /* if execv returns, there was an error */
    } else {
        if (info->method == INJECT_PTRACE) {
            os_ptrace(PTRACE_DETACH, info->pid, NULL, NULL);
        }
        /* Close the pipe. */
        close(info->pipe_fd);
        info->pipe_fd = 0;
    }
    return true;
}

DR_EXPORT
int
dr_inject_process_exit(void *data, bool terminate)
{
    dr_inject_info_t *info = (dr_inject_info_t *) data;
    int status;
    if (terminate) {
        kill(info->pid, SIGKILL);
    }
    if (info->pipe_fd != 0)
        close(info->pipe_fd);
    waitpid(info->pid, &status, 0);
    free(info);
    return status;
}

/*******************************************************************************
 * PTRACE INJECTION CODE
 */

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

static bool op_exec_gdb = false;
static bool verbose = false;

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
        fprintf(stderr, "ptrace failed: ptrace(%s, %d, %p, %p) -> %ld %s\n",
                   pair->enum_name, (int)pid, addr, data, r, strerror(-r));
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
        fprintf(stderr, "injecting code:\n");
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
        fprintf(stderr, "Unexpected trace event, expected SIGTRAP, got:\n"
                   "status: 0x%x, stopped by signal: %d, at pc: %p\n",
                   status, WSTOPSIG(status), (void*)pc);
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

static void *
translate_dr_dll(void *injected_base, void *dr_addr)
{
    app_pc dr_base = get_dynamorio_dll_start();
    return ((app_pc)dr_addr - dr_base) + injected_base;
}

static int injected_dr_fd;
static dr_inject_info_t *injected_info;

/* translate platform independent protection bits to native flags */
static inline uint
memprot_to_osprot(uint prot)
{
    uint mmap_prot = 0;
    if (TEST(MEMPROT_EXEC, prot))
        mmap_prot |= PROT_EXEC;
    if (TEST(MEMPROT_READ, prot))
        mmap_prot |= PROT_READ;
    if (TEST(MEMPROT_WRITE, prot))
        mmap_prot |= PROT_WRITE;
    return mmap_prot;
}

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
    if (r < 0 && r >= -4096) {
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


/* Returns the minimum p_vaddr field, aligned to page boundaries, in
 * the loadable segments in the prog_header array, or POINTER_MAX if
 * there are no loadable segments.
 *
 * NOCHECKIN: Copied from module.c
 */
app_pc
module_vaddr_from_prog_header(app_pc prog_header, uint num_segments,
                              OUT app_pc *out_end)
{
    uint i;
    app_pc min_vaddr = (app_pc) POINTER_MAX;
    app_pc mod_end = (app_pc) PTR_UINT_0;
    for (i = 0; i < num_segments; i++) {
        /* Without the ELF header we use sizeof instead of elf_hdr->e_phentsize
         * which must be a reliable assumption as dl_iterate_phdr() doesn't
         * bother to deliver the entry size.
         */
        ELF_PROGRAM_HEADER_TYPE *prog_hdr = (ELF_PROGRAM_HEADER_TYPE *)
            (prog_header + i * sizeof(ELF_PROGRAM_HEADER_TYPE));
        if (prog_hdr->p_type == PT_LOAD) {
            /* ELF requires p_vaddr to already be aligned to p_align */
            min_vaddr =
                MIN(min_vaddr, (app_pc) ALIGN_BACKWARD(prog_hdr->p_vaddr, PAGE_SIZE));
            mod_end =
                MAX(mod_end, (app_pc)
                    ALIGN_FORWARD(prog_hdr->p_vaddr + prog_hdr->p_memsz, PAGE_SIZE));
        }
    }
    if (out_end != NULL)
        *out_end = mod_end;
    return min_vaddr;
}

typedef byte *(*map_fn_t)(file_t f, size_t *size INOUT, uint64 offs,
                          app_pc addr, uint prot/*MEMPROT_*/, bool cow,
                          bool image, bool fixed);
typedef bool (*unmap_fn_t)(byte *map, size_t size);
typedef bool (*prot_fn_t)(byte *map, size_t size, uint prot/*MEMPROT_*/);


/* Map in the PT_LOAD segments in an ELF's program headers.
 *
 * NOCHECKIN: Copied from loader.c
 *
 * XXX: Instead of mapping the file twice and disturbing the address space, we
 * should use os_read to read the ELF header and program headers.  We can't do
 * this currently because we need to compute text_addr for gdb.
 */
static app_pc
map_elf_phdrs(const char *filename, bool fixed, size_t *size OUT,
              ptr_int_t *load_delta OUT, app_pc *text_addr_p OUT,
              map_fn_t map_func, unmap_fn_t unmap_func, prot_fn_t prot_func)
{
    file_t fd;
    app_pc file_map, lib_base, lib_end, last_end;
    ELF_HEADER_TYPE *elf_hdr;
    app_pc  map_base, map_end;
    reg_t   pg_offs;
    size_t map_size;
    uint64 file_size_64;
    size_t file_size;
    uint   seg_prot, i;
    ptr_int_t delta;

    /* Zero optional out args. */
    if (size != NULL)
        *size = 0;
    if (load_delta != NULL)
        *load_delta = 0;

    /* open file for mmap later */
    fd = os_open(filename, OS_OPEN_READ);
    if (fd == INVALID_FILE) {
        LOG(GLOBAL, LOG_LOADER, 1, "%s: failed to open %s\n", __FUNCTION__, filename);
        return NULL;
    }

    /* get file size */
    if (!os_get_file_size_by_handle(fd, &file_size_64)) {
        os_close(fd);
        LOG(GLOBAL, LOG_LOADER, 1, "%s: failed to get library %s file size\n",
            __FUNCTION__, filename);
        return NULL;
    }
    file_size = file_size_64;  /* Truncate, we have to pass it as size_t *. */

    /* map the library file into memory for parsing */
    file_map = os_map_file(fd, &file_size, 0/*offs*/, NULL/*base*/,
                           MEMPROT_READ /* for parsing only */,
                           true  /*writes should not change file*/,
                           false /*image*/,
                           false /*!fixed*/);
    if (file_map == NULL) {
        os_close(fd);
        LOG(GLOBAL, LOG_LOADER, 1, "%s: failed to map %s\n", __FUNCTION__, filename);
        return NULL;
    }

#if 0  /* FIXME: is_elf_so_header undefined, trust it for now */
    /* verify if it is elf so header */
    if (!is_elf_so_header(file_map, file_size)) {
        (*unmap_func)(file_map, file_size);
        os_close(fd);
        LOG(GLOBAL, LOG_LOADER, 1, "%s: %s is not a elf so header\n", 
            __FUNCTION__, filename);
        return NULL;
    }
#endif

    /* more sanity check */
    elf_hdr = (ELF_HEADER_TYPE *) file_map;
    ASSERT_CURIOSITY(elf_hdr->e_phoff != 0);
    ASSERT_CURIOSITY(elf_hdr->e_phentsize == 
                     sizeof(ELF_PROGRAM_HEADER_TYPE));

    /* get the library size and preferred base */
    map_base = 
        module_vaddr_from_prog_header(file_map + elf_hdr->e_phoff,
                                      elf_hdr->e_phnum,
                                      &map_end);
    map_size = map_end - map_base;
    if (size != NULL)
        *size = map_size;

    /* reserve the memory from os for library */
    lib_base = (*map_func)(-1, &map_size, 0, map_base,
                           MEMPROT_WRITE | MEMPROT_READ /* prot */,
                           true  /* copy-on-write */,
                           true  /* image, make it reachable */,
                           fixed);
    ASSERT(lib_base != NULL);
    lib_end = lib_base + map_size;

    if (map_base != NULL && map_base != lib_base) {
        /* the mapped memory is not at preferred address, 
         * should be ok if it is still reachable for X64,
         * which will be checked later. 
         */
        LOG(GLOBAL, LOG_LOADER, 1, "%s: module not loaded at preferred address\n",
            __FUNCTION__);
    }
    delta = lib_base - map_base;
    if (load_delta != NULL)
        *load_delta = delta;

    /* walk over the program header to load the individual segments */
    last_end = lib_base;
    for (i = 0; i < elf_hdr->e_phnum; i++) {
        app_pc seg_base, seg_end, map, file_end;
        size_t seg_size;
        ELF_PROGRAM_HEADER_TYPE *prog_hdr = (ELF_PROGRAM_HEADER_TYPE *)
            (file_map + elf_hdr->e_phoff + i * elf_hdr->e_phentsize);
        if (prog_hdr->p_type == PT_LOAD) {
            seg_base = (app_pc)ALIGN_BACKWARD(prog_hdr->p_vaddr, PAGE_SIZE)
                       + delta;
            seg_end  = (app_pc)ALIGN_FORWARD(prog_hdr->p_vaddr +
                                             prog_hdr->p_filesz,
                                             PAGE_SIZE) 
                       + delta;
            seg_size = seg_end - seg_base;
            if (seg_base != last_end) {
                /* XXX: a hole, I reserve this space instead of unmap it */
                size_t hole_size = seg_base - last_end;
                (*prot_func)(last_end, hole_size, MEMPROT_NONE);
            }
            seg_prot = module_segment_prot_to_osprot(prog_hdr);
            pg_offs  = ALIGN_BACKWARD(prog_hdr->p_offset, PAGE_SIZE);
            /* FIXME: 
             * This function can be called after dynamorio_heap_initialized,
             * and we will use map_file instead of os_map_file.
             * However, map_file does not allow mmap with overlapped memory, 
             * so we have to unmap the old memory first.
             * This might be a problem, e.g. 
             * one thread unmaps the memory and before mapping the actual file,
             * another thread requests memory via mmap takes the memory here,
             * a racy condition.
             */
            (*unmap_func)(seg_base, seg_size);
            map = (*map_func)(fd, &seg_size, pg_offs,
                              seg_base /* base */,
                              seg_prot | MEMPROT_WRITE /* prot */,
                              true /* writes should not change file */,
                              true /* image */,
                              true /* fixed */);
            ASSERT(map != NULL);
            /* fill zeros at extend size */
            file_end = (app_pc)prog_hdr->p_vaddr + prog_hdr->p_filesz;
            if (seg_end > file_end + delta)
                memset(file_end + delta, 0, seg_end - (file_end + delta));
            seg_end  = (app_pc)ALIGN_FORWARD(prog_hdr->p_vaddr + 
                                             prog_hdr->p_memsz,
                                             PAGE_SIZE) + delta;
            seg_size = seg_end - seg_base;
            (*prot_func)(seg_base, seg_size, seg_prot);
            last_end = seg_end;
        } 
    }
    ASSERT(last_end == lib_end);

    /* unmap the file_map */
    os_unmap_file(file_map, file_size);
    os_close(fd); /* no longer needed */

    return lib_base;
}

bool
inject_ptrace(dr_inject_info_t *info, const char *library_path)
{
    /* Cause the child to call PTRACE_TRACEME. */
    /* XXX: To implement attach, we should switch to PTRACE_ATTACH. */
    write_pipe_cmd(info->pipe_fd, "ptrace");

    /* Open libdynamorio.so as readonly in the child. */
    int dr_fd = injectee_open(info, library_path, O_RDONLY, 0);
    if (dr_fd < 0) {
        fprintf(stderr, "unable to open libdynamorio.so in injectee. "
                   "errno %d msg %s\n", -dr_fd, strerror(-dr_fd));
        return false;
    }

    /* Call our private loader, but perform the mmaps in the child process
     * instead of the parent.
     */
    size_t loaded_size;
    app_pc injected_base;
    ptr_int_t load_delta;
    elf_loader_t loader;
    /* XXX: Have to use globals to communicate to injectee_map_file. =/ */
    injected_dr_fd = dr_fd;
    injected_info = info;
    if (!elf_loader_read_headers(&loader, library_path)) {
        return false;
    }
    injected_base = elf_loader_map_phdrs(&loader, true/*fixed*/,
                                         injectee_map_file, injectee_unmap,
                                         injectee_prot);
    if (injected_base == NULL) {
        return false;
    }
    /* Looking up exports through ptrace is hard, so we use the same entry
     * point as early injection with different arguments.
     */
    void *injected_dr_start = loader.ehdr->e_entry + loader.load_delta;
    elf_loader_destroy(&loader);

    struct user_regs_struct regs;
    os_ptrace(PTRACE_GETREGS, info->pid, NULL, &regs);

    if (verbose) {
        printf("regs.r15: 0x%lx\n", regs.r15);
        printf("regs.r14: 0x%lx\n", regs.r14);
        printf("regs.r13: 0x%lx\n", regs.r13);
        printf("regs.r12: 0x%lx\n", regs.r12);
        printf("regs.rbp: 0x%lx\n", regs.rbp);
        printf("regs.rbx: 0x%lx\n", regs.rbx);
        printf("regs.r11: 0x%lx\n", regs.r11);
        printf("regs.r10: 0x%lx\n", regs.r10);
        printf("regs.r9 : 0x%lx\n", regs.r9);
        printf("regs.r8 : 0x%lx\n", regs.r8);
        printf("regs.rax: 0x%lx\n", regs.rax);
        printf("regs.rcx: 0x%lx\n", regs.rcx);
        printf("regs.rdx: 0x%lx\n", regs.rdx);
        printf("regs.rsi: 0x%lx\n", regs.rsi);
        printf("regs.rdi: 0x%lx\n", regs.rdi);
        printf("regs.rsp: 0x%lx\n", regs.rsp);
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
    regs.rip = (reg_t)injected_dr_start;
    os_ptrace(PTRACE_SETREGS, info->pid, NULL, &regs);

    //op_exec_gdb = true;
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
        fprintf(stderr, "Unexpected trace event, expected SIGTRAP, got:\n"
                   "status: 0x%x, stopped: %d, by signal: %d, at pc: %p\n",
                   status, WIFSTOPPED(status), WSTOPSIG(status), pc);
        return false;
    }

    //int init = injectee_call_dr_fn(info, injected_dr_start);
    //printf("_dr_start: 0x%x\n", init);

    /* Reset back to initial state (for now, eventually adjust state to start in
     * DR).
     */
    os_ptrace(PTRACE_SETREGS, info->pid, NULL, &cxt.regs);

    return true;
}
