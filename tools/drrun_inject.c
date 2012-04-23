//#define _GNU_SOURCE  [> for waitid <]

#include "configure.h"
#include "globals_shared.h"

#include "dr_inject.h"
#include "dr_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

/* FIXME: Figure out how to include dr_api.h properly.  We currently get the
 * core/lib version of dr_api.h, and core/x86/instrument.h pulls in too much DR
 * internal stuff.
 */
void *
dr_standalone_init(void);
bool
dr_file_exists(const char *fname);

int
main(int argc, char **argv)
{
    void *inject_data;
    int r;
    bool success;
    process_id_t pid;
    void *dc;
    char dr_lib_path[PATH_MAX];
    char *cur;
    char *end = &dr_lib_path[PATH_MAX];
    /* FIXME: Remove default and parse args. */
    const char *exe_path = "hello";

    if (argc >= 2) {
        exe_path = argv[1];
    }

    /* Figure out where argv[0] is and guess where libdynamorio.so is. */
    if (argv[0][0] != '/') {
        getcwd(dr_lib_path, sizeof(dr_lib_path));
        cur = strchr(dr_lib_path, '\0');
        *cur++ = '/';
    } else {
        cur = dr_lib_path;
    }
    strncpy(cur, argv[0], end - cur);
    cur = strrchr(dr_lib_path, '/');
    strncpy(cur, "/../lib64/debug/libdynamorio.so", end - cur);
    end[-1] = '\0';
    if (!dr_file_exists(dr_lib_path)) {
        printf("Couldn't find libdynamorio.so at %s\n", dr_lib_path);
        return 1;
    }

    /* XXX: Because we're using DR standalone instead of a split drinjectlib. */
    dc = dr_standalone_init();

    r = dr_inject_process_create(exe_path, "arg1 arg2 arg3", &inject_data);
    if (r != 0) {
        fprintf(stderr, "dr_inject_process_create failed: %d\n", r);
        return r;
    }

    pid = dr_inject_get_process_id(inject_data);
    printf("created child pid: %d\n", pid);

    success = dr_inject_process_inject(inject_data, true/*force*/, dr_lib_path);
    if (!success) {
        fprintf(stderr, "dr_inject_process_inject failed\n");
        return 1;
    }

    success = dr_inject_process_run(inject_data);
    if (!success) {
        fprintf(stderr, "dr_inject_process_run failed\n");
        return 1;
    }

    r = dr_inject_process_wait(inject_data);
    printf("status: %x, exited: %d, signaled: %d, stopped: %d\n",
           r, WIFEXITED(r), WIFSIGNALED(r), WIFSTOPPED(r));

    dr_inject_process_exit(inject_data, false/*terminate*/);

    return r;
}
