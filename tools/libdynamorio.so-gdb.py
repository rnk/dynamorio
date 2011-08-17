#!/usr/bin/env python

import gdb
import os
import traceback

print 'Loading gdb scripts for debugging DynamoRIO...'

def compute_lib_dir():
    # If we attached to a process with libdynamorio in it, use that libdir.
    for objfile in gdb.objfiles():
        if "libdynamorio.so" in objfile.filename:
            return os.path.dirname(os.path.abspath(objfile.filename))
    # If we didn't attach, pick the libdir from the last configuration.
    libdir = "@DR_LIBRARY_OUTPUT_DIRECTORY@"
    return libdir


class DROption(gdb.Parameter):

    def __init__(self, dr_option, param_class):
        super(DROption, self).__init__("dr-" + dr_option, gdb.COMMAND_OBSCURE,
                                       param_class)
        self.dr_option = dr_option

    set_doc = ("DynamoRIO option of the same name.")
    show_doc = set_doc
    def get_set_string(self):
        return str(self.value)
    def get_show_string(self, svalue):
        return svalue


# The client and client-args options are special.
class DRClient(DROption):
    def __init__(self):
        super(DRClient, self).__init__("client", gdb.PARAM_OPTIONAL_FILENAME)
    set_doc = ("Path to DynamoRIO client to run when invoking DR.  "
               "Leave blank to run without a client.")
    show_doc = set_doc
class DRClientArgs(DROption):
    def __init__(self):
        super(DRClientArgs, self).__init__("client-args", gdb.PARAM_OPTIONAL_FILENAME)
    set_doc = ("DynamoRIO client arguments.")
    show_doc = set_doc


# Values contained in here.
dr_client = DRClient()
dr_client_args = DRClientArgs()
dr_options = [
        DROption('msgbox_mask', gdb.PARAM_INTEGER),
        DROption('loglevel', gdb.PARAM_INTEGER),
        DROption('logmask', gdb.PARAM_INTEGER),
        ]


class RunDR(gdb.Command):

    def __init__(self):
        super(RunDR, self).__init__("rundr", gdb.COMMAND_OBSCURE)

    def invoke(self, arg, from_tty):
        # Build LD_LIBRARY_PATH.
        lib_dir = compute_lib_dir()
        parts = lib_dir.split(os.sep)
        ext_lib_dir = os.sep.join(parts[:-2] + ['ext'] + parts[-2:])
        ld_paths = [lib_dir, ext_lib_dir]
        env_ld_path = os.environ.get("LD_LIBRARY_PATH")
        if env_ld_path:
            ld_paths.append(env_ld_path)
        ld_path = ':'.join(ld_paths)

        # Build options string.
        dr_opts = os.environ.get("DYNAMORIO_OPTIONS", "")
        param_opts = ' '.join('-%s %s' % (p.dr_option, p.value)
                              for p in dr_options)
        if dr_opts and param_opts:
            dr_opts += ' '
        dr_opts += param_opts
        if dr_client.value:
            client_opts = ('-code_api -client_lib %s;0;%s' %
                           (dr_client.value, dr_client_args.value))
            dr_opts = client_opts + " " + dr_opts

        gdb.execute("set env LD_LIBRARY_PATH " + ld_path)
        gdb.execute("set env LD_PRELOAD libdrpreload.so libdynamorio.so")
        gdb.execute("set env DYNAMORIO_OPTIONS " + dr_opts)
        gdb.execute("run " + arg)

RunDR()


class PrivmodFinalizeBP(gdb.Breakpoint):

    # Enable to debug this breakpoint.
    DEBUG = True
    DYNAMORIO_BP = True

    def __init__(self):
        super(PrivmodFinalizeBP, self).__init__("privload_load_finalize",
                                                internal=self.DEBUG)

    def stop(self):
        frame = gdb.newest_frame()
        print "pc from pygdb:", frame.pc()
        return self.DEBUG  # Controls whether the user stops here or not.


class InitBP(gdb.Breakpoint):

    DEBUG = False
    DYNAMORIO_BP = True

    def __init__(self):
        super(InitBP, self).__init__("_init", internal=not self.DEBUG)

    def stop(self):
        frame = gdb.newest_frame()
        solib_name = gdb.solib_name(frame.pc())
        if "drpreload" in solib_name:
            PrivmodFinalizeBP()
            self.delete()
        return self.DEBUG


# Delete all breakpoints set from previous runs and initializations and replace
# them with new ones.
def init_bp():
    bps = gdb.breakpoints()
    if not bps:
        return
    for bp in bps:
        if getattr(bp, 'DYNAMORIO_BP', False):
            bp.delete()
init_bp()
the_init_bp = InitBP()
