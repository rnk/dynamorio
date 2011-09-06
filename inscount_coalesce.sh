#!/bin/bash
source shrc
export SPEC_CPU2006_NO_RUNDIR_DEL=1
DYNAMORIO_BUILD="/scratch/rnk/dynamorio/build_git-clone"
my_runspec() {
    #bm="int ^400"
    bm="bzip2"
    cmd="runspec --size=test --noreportable --iterations 1 $@ $bm"
    echo $cmd
    $cmd
}
dr_runspec() {
    DYNAMORIO_OPTIONS="$2 -no_private_loader" my_runspec \
        --define DYNAMORIO_BUILD="$DYNAMORIO_BUILD" \
        --define DR_EXT="$1" \
        --define DR_CLIENT="ext/lib64/release/libinscount_cleancall.so" \
        --define DR_CLIENT_OPTS="$3" \
        --config=linux64-amd64-gcc-dr.cfg
}
# native
my_runspec --config=linux64-amd64-gcc.cfg
# clean
#dr_runspec opt0 '-max_bb_instrs 50' '0'
# inline
dr_runspec opt2 '-max_bb_instrs 50' '2'
# coalesced
dr_runspec opt3 '-max_bb_instrs 50' '3'
