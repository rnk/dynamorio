/* **********************************************************
 * Copyright (c) 2013 Google, Inc.  All rights reserved.
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

/* Helper for testing DynamoRIO's ability to attach to running processes on
 * Linux.
 *
 * The issue is that modern distros restrict the user's ability to arbitrarily
 * ptrace running processes.  The user has to elevate their permissions or set
 * /proc/sys/kernel/yama/ptrace_scope to 0 in order to target any process.
 * Short of doing that, they can only trace direct children of the tracing
 * process.  This test harness arranges for that by launching the child app,
 * and then calling execv() on the injector.
 */

#include "tools.h"

#include <limits.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

enum { MAX_ARGS = 40 };

/* PID_MAX_LIMIT is 4 million, which is 7 digits, plus a null. */
enum { PID_STR_MAX = 8 };

static const char *argv0;

static void
__attribute__((noreturn))
usage(void)
{
    print("usage: %s <ops> -- drrun ... -- app ...\n", argv0);
    exit(1);
}

static int
find_next_separator(int argc, char **argv, int i)
{
    for (; i < argc; i++) {
        if (strcmp(argv[i], "--") == 0)
            break;
    }
    if (i >= argc) {
        usage();
    }
    return i;
}

static void
launch_app_and_exec_drrun(char **drrun_cmd, char **app_cmd, int writer)
{
    /* XXX: To be totally safe according to POSIX, we should probably re-exec
     * ourselves if we ever need to use non-signal safe functions like malloc.
     */
    int i;
    pid_t app_pid = fork();
    if (app_pid < 0) {
        perror(argv0);
        exit(1);
    } else if (app_pid == 0) {
        execv(app_cmd[0], app_cmd);
        perror("app execv failed");
    } else {
        const char *my_drrun_cmd[MAX_ARGS];
        char pid_str[PID_STR_MAX];
        int i;
        ssize_t r;
        struct timespec pause_time;

        /* Wait for the app get started.
         * XXX: Determinize this with a pipe or signal or something.
         */
        pause_time.tv_sec = 0;
        pause_time.tv_nsec = 100 * 1000 * 1000;  /* 100ms */
        nanosleep(&pause_time, &pause_time);

        /* Tell the master process the pid of the app so it can signal it. */
        snprintf(pid_str, BUFFER_SIZE_ELEMENTS(pid_str), "%d", app_pid);
        NULL_TERMINATE_BUFFER(pid_str);
        r = write(writer, pid_str, strlen(pid_str)+1);

        /* execv() drrun -attach pid ... */
        my_drrun_cmd[0] = drrun_cmd[0];
        my_drrun_cmd[1] = "-attach";
        my_drrun_cmd[2] = pid_str;
        for (i = 3; i < MAX_ARGS && drrun_cmd[i - 2] != NULL; i++) {
            my_drrun_cmd[i] = drrun_cmd[i - 2];
        }
        if (i < MAX_ARGS)
            my_drrun_cmd[i] = NULL;
        execv(my_drrun_cmd[0], (char **)my_drrun_cmd);
        perror("drrun execv failed");
        _exit(1);
    }
    /* _exit() to avoid double-flushing any buffers or at exit handlers. */
    _exit(0);
}

int
main(int argc, char **argv)
{
    int i;
    char **app_cmd;
    char **drrun_cmd;
    int pipe_fds[2];
    int reader;
    int writer;
    ssize_t r;

    /* argv is a drrun command.  The app command starts with --. */
    argv0 = argv[0];
    if (argc < 3) {
        usage();
    }
    i = 1;
    i = find_next_separator(argc, argv, i);
    drrun_cmd = &argv[++i];
    if (strstr(drrun_cmd[0], "drrun") == NULL) {
        usage();
    }
    i = find_next_separator(argc, argv, i);
    app_cmd = &argv[++i];

    /* We can't wait on grandchildren (the app's process) with any of the wait
     * syscalls, so we create a pipe that gets inherited by all our children,
     * and block on a read from that.  Note that we *don't* want O_CLOEXEC on
     * these fds.
     */
    r = pipe(pipe_fds);
    if (r < 0) {
        perror("pipe failed");
        return 1;
    }
    reader = pipe_fds[0];
    writer = pipe_fds[1];

    /* OK, make the injector process. */
    pid_t injector_pid = fork();
    if (injector_pid < 0) {
        perror(argv0);
        return 1;
    } else if (injector_pid == 0) {
        /* child */
        close(reader);
        launch_app_and_exec_drrun(drrun_cmd, app_cmd, writer);
    } else {
        /* As the parent, wait on the child and then on the pipe. */
        char buf[10];
        pid_t app_pid;
        struct timespec pause_time;

        close(writer);
        waitpid(injector_pid, 0, 0);

        /* Injection is done.  Get the pid of the app so we can signal it. */
        r = read(reader, buf, sizeof(buf));
        if (r < 0) {
            perror("pipe read error");
            return 1;
        }
        if (r == 0 || buf[r-1] != '\0') {
            print("expected string on pipe\n");
            return 1;
        }
        app_pid = strtoul(buf, NULL, 10);
        if (app_pid == ULONG_MAX) {
            print("expected pid on pipe\n");
            return 1;
        }

        /* Sleep, then kill it. */
        pause_time.tv_sec = 0;
        pause_time.tv_nsec = 500 * 1000 * 1000;  /* 500ms */
        nanosleep(&pause_time, &pause_time);
        r = kill(app_pid, SIGTERM);
        if (r < 0) {
            perror("kill failed");
            return 1;
        }

        /* Wait for it to terminate by reading EOF from the pipe. */
        r = read(reader, buf, sizeof(buf));
        if (r < 0) {
            perror("pipe read error");
            return 1;
        } else if (r > 0) {
            print("unexpected write to the pipe: %*.s\n",
                  r, buf);
            return 1;
        } else {
            print("pipe closed, app exited\n");
        }
    }

    return 0;
}
