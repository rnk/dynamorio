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
#include "globals_shared.h"  /* for the types */

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#define EXPORT __attribute__((visibility("default")))

/* Opaque type to users, holds our state */
typedef struct _dr_inject_info_t {
    process_id_t pid;
    char image_name[MAXIMUM_PATH];
    int pipe_fd;
} dr_inject_info_t;


static process_id_t
fork_suspended_child(const char *exe, const char **argv, int *fds)
{
    process_id_t pid = fork();
    if (pid == 0) {
        /* child, suspend before exec */
        char libdr_path[MAXIMUM_PATH];
        ssize_t nread;
        size_t sofar = 0;
        close(fds[1]);  /* Close writer in child, keep reader. */
        do {
            nread = read(fds[0], libdr_path + sofar, sizeof(libdr_path) - sofar);
            sofar += nread;
        } while (nread > 0 && sofar < sizeof(libdr_path)-1);
        libdr_path[sofar] = '\0';
        close(fds[0]);  /* Close reader before exec. */
        argv[0] = exe;  /* Make argv[0] absolute for DR's benefit. */
        execv(libdr_path, (char **) argv);
        exit(-1);
    }
    return pid;
}

/* Returns 0 on success.
 */
EXPORT
int
dr_inject_process_create(const char *exe, const char **argv, void **data OUT)
{
    int r;
    int fds[2];
    dr_inject_info_t *info = malloc(sizeof(*info));
    const char *basename = strrchr(exe, '/');
    if (basename == NULL) {
        return -1;  /* exe should be absolute. */
    }
    basename++;
    strncpy(info->image_name, basename, sizeof(info->image_name));

    /* Create a pipe to a forked child and have it block on the pipe. */
    r = pipe(fds);
    if (r != 0)
        goto error;
    info->pid = fork_suspended_child(exe, argv, fds);
    close(fds[0]);  /* Close reader, keep writer. */
    info->pipe_fd = fds[1];

    if (info->pid == -1)
        goto error;
    *data = info;
    return 0;

error:
    free(info);
    return errno;
}

EXPORT
process_id_t
dr_inject_get_process_id(void *data)
{
    dr_inject_info_t *info = (dr_inject_info_t *) data;
    return info->pid;
}

EXPORT
char *
dr_inject_get_image_name(void *data)
{
    dr_inject_info_t *info = (dr_inject_info_t *) data;
    return info->image_name;
}

/* XXX: library_path is optional on Windows, but on Linux it's excessively
 * difficult to get access to the config file and read DYNAMORIO_AUTOINJECT.
 * Therefore, we make it non-optional and push the work on the the caller,
 * since chances are they know where libdynamorio.so is.
 */
EXPORT
bool
dr_inject_process_inject(void *data, bool force_injection,
                         const char *library_path)
{
    dr_inject_info_t *info = (dr_inject_info_t *) data;
    write(info->pipe_fd, library_path, strlen(library_path));
    close(info->pipe_fd);
    info->pipe_fd = 0;
    return true;
}

EXPORT
bool
dr_inject_process_run(void *data)
{
    dr_inject_info_t *info = (dr_inject_info_t *) data;
    kill(info->pid, SIGCONT);
    return true;
}

EXPORT
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
