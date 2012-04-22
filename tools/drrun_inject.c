//#define _GNU_SOURCE  [> for waitid <]

#include "configure.h"
#include "globals_shared.h"

#include "dr_inject.h"
#include "dr_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>

void *dr_standalone_init(void);

int
main(int argc, char **argv)
{
    void *inject_data;
    int r;
    bool success;
    process_id_t pid;
    void *dc;
    //siginfo_t siginfo;

    /* XXX: Because we're using DR standalone instead of a split drinjectlib. */
    dc = dr_standalone_init();

    r = dr_inject_process_create("hello", "arg1 arg2 arg3", &inject_data);
    printf("dr_inject_process_create: %d\n", r);

    pid = dr_inject_get_process_id(inject_data);
    printf("pid: %d\n", pid);

    success = dr_inject_process_inject(
            inject_data, true/*force?*/,
            "/usr/local/google/home/rnk/dynamorio_build/install");
    printf("dr_inject_process_inject: %d\n", success);

    dr_inject_process_run(inject_data);

    r = dr_inject_process_wait(inject_data);
    printf("status: %x, exited: %d, signaled: %d, stopped: %d\n",
           r, WIFEXITED(r), WIFSIGNALED(r), WIFSTOPPED(r));

    dr_inject_process_exit(inject_data, false/*terminate*/);

    return r;
}
