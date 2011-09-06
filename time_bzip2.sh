#!/bin/bash
set -e
run_dir="/scratch/rnk/dynamorio/spec06/benchspec/CPU2006/401.bzip2/run"
run_dir="${run_dir}/run_base_test_amd64-m64-gcc-dr-opt0.0001"
bzip2_cmd="${run_dir}/bzip2_base.amd64-m64-gcc-dr-opt0 ${run_dir}/dryer.jpg 2"
dr_build="/scratch/rnk/dynamorio/build_git-clone"
#drrun_ops="${dr_build}/bin64/drrun -ops '-no_private_loader'"
drrun="${dr_build}/bin64/drrun"
drcalls_dir="${dr_build}/ext/lib64/release"
#output="/dev/stdout"
output="/dev/null"
align_ops="print_count"
#align_ops="print_count print_accesses print_symbols"
client_alignment="${drcalls_dir}/libdrcalls_bench.alignment.so"

(cd ${dr_build} && make > /dev/null)
export DYNAMORIO_OPTIONS="-no_private_loader -max_bb_instrs 50"

## Inscount
#echo native
#time ${bzip2_cmd} > ${output}
#echo noclient
#time ${drrun} ${bzip2_cmd} > ${output}
#echo inscount_manual
#time ${drrun} -client ${drcalls_dir}/libdrcalls_bench.inscount_manual.so 0 "2" ${bzip2_cmd} > ${output}
#echo inscount_opt3
#time ${drrun} -client ${drcalls_dir}/libdrcalls_bench.inscount_cleancall.so 0 "3" ${bzip2_cmd} > ${output}
#echo inscount_opt3_tls
#time ${drrun} -client ${drcalls_dir}/libdrcalls_bench.inscount_cleancall.so 0 "3 use_tls" ${bzip2_cmd} > ${output}
#echo inscount_opt0
#time ${drrun} -client ${drcalls_dir}/libdrcalls_bench.inscount_cleancall.so 0 "0" ${bzip2_cmd} > ${output}
#echo inscount_opt0_bb
#time ${drrun} -client ${drcalls_dir}/libdrcalls_bench.inscount_bbcall.so 0 "0" ${bzip2_cmd} > ${output}

# New times
function inscount_time() {
    time ${drrun} \
      -client ${drcalls_dir}/libdrcalls_bench.inscount_cleancall.so 0 "$1" \
      ${bzip2_cmd} > ${output}
}

echo native
time ${bzip2_cmd} > ${output}
echo opt0
inscount_time 0
echo "opt2 (inline, noopt)"
inscount_time 2
echo "opt3 (coalesce)"
inscount_time 3
echo "opt3 (tls)"
inscount_time "3 use_tls"
echo "opt5 (rle+dse)"
inscount_time "5 use_tls"
echo "opt6 (avoid aflags)"
inscount_time "6 use_tls"
echo "opt7 (fold_lea)"
inscount_time "7 use_tls"

## Alignment
#echo alignment_opt3
#time ${drrun} -client ${client_alignment} 0 "${align_ops} opt_calls 3" ${bzip2_cmd} > ${output}
#echo mem_buffer_opt3
#time ${drrun} -client ${client_alignment} 0 "'${align_ops} use_buffer opt_calls 3'" ${bzip2_cmd} > ${output}
#echo alignment_opt0
#time ${drrun} -client ${client_alignment} 0 "'${align_ops} opt_calls 0'" ${bzip2_cmd} > ${output}
