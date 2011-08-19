#!/usr/bin/env python

import gdb
import os

print 'Loading gdb scripts for debugging DynamoRIO...'

# TODO: Get this from build dir or introspect.
DR_BUILD = "/scratch/rnk/dynamorio/build_git-clone"
BUILD_MODE = "debug"


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
        ld_path = (DR_BUILD +     "/lib64/" + BUILD_MODE + ":" +
                   DR_BUILD + "/ext/lib64/" + BUILD_MODE)
        orig_ld_path = os.environ.get("LD_LIBRARY_PATH")
        if orig_ld_path:
            ld_path += ":" + orig_ld_path

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
