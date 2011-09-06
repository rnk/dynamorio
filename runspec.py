#!/afs/csail.mit.edu/proj/courses/6.172/bin/python
#!/usr/bin/env python2.7

import os
import subprocess
import sys
import re


# Configuration points when running spec:
# DYNAMORIO_BUILD
# DYNAMORIO_CLIENT
# DR_EXT
# DR_CLIENT_OPTS
# DYNAMORIO_OPTIONS

SPEC_DIR = "/scratch/rnk/dynamorio/spec06"
DR_BUILD = "/scratch/rnk/dynamorio/build_git-clone"
DYNAMORIO_OPTIONS = ["-no_private_loader", "-max_bb_instrs", "50"]


def normalize_times(base_times, times):
    """Normalize times to base_times.

    Both are lists of (bm_name, time) tuples.
    """
    new_times = []
    both_times = zip(times, base_times)
    for (i, ((bm, time), (base_bm, base_time))) in enumerate(both_times):
        assert bm == base_bm
        norm_time = time / base_time
        new_times.append((bm, norm_time))
    return new_times


def normalize_all(base_times, configs_times):
    return [(config_name, normalize_times(base_times, bm_times))
            for (config_name, bm_times) in configs_times]


def avg(xs):
    xs = list(xs)
    return sum(xs) / float(len(xs))


def avg_bm_times(times):
    return avg(time for (bm_name, time) in times)


def spawn_separate(cmd, env):
    flags = 0
    #flags |= subprocess.CREATE_NEW_PROCESS_GROUP
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, env=env, cwd=SPEC_DIR,
                            creationflags=flags)


def parse_spec_csv(csv_path):
    """Given a path to a SPEC csv results file, return a list of bm_name/time
    tuples."""
    times = []
    # These are not standard csv files so we do our own parsing.
    with open(csv_path) as csv_in:
        for line in csv_in:
            # Ignore non-benchmark lines.
            if not re.match(r'^\d{3}', line):
                continue
            # Stop after first blank.
            if not line.strip():
                break
            parts = line.split(',')
            bm_name = parts[0]
            # Drop the SPEC number from the bm name.
            bm_name = bm_name[4:]
            time = parts[2]
            try:
                time = float(time)
            except:
                time = None
            times.append((bm_name, time))
    times.sort()
    return times


class Benchmark(object):

    def __init__(self, name, config, bms):
        self.name = name
        self.config = config
        self.bms = bms
        self.dr_build = DR_BUILD
        self.dr_opts = DYNAMORIO_OPTIONS
        self.dr_client = None
        self.dr_client_opts = None
        self.times = None

    def _add_def(self, cmd, cfg_var, val):
        if val is not None:
            cmd.extend(['--define', '%s=%s' % (cfg_var, val)])

    def build_command(self):
        cmd = ['./runspec.sh', '--size=test', '--noreportable', '--iterations',
               '1']
        self._add_def(cmd, 'DYNAMORIO_BUILD', self.dr_build)
        self._add_def(cmd, 'DR_CLIENT',       self.dr_client)
        self._add_def(cmd, 'DR_CLIENT_OPTS',  self.dr_client_opts)
        self._add_def(cmd, 'DR_EXT',          self.name)
        cmd.extend(['--config', self.config])
        cmd.extend(self.bms)
        env = dict(os.environ)
        if self.dr_opts:
            env['DYNAMORIO_OPTIONS'] = ' '.join(self.dr_opts)
        return (cmd, env)

    def runspec(self):
        # size? reportable? iterations?
        (cmd, env) = self.build_command()
        print 'Running bm %s' % self.name
        print ' '.join(cmd)
        try:
            proc = spawn_separate(cmd, env)
            (output, _) = proc.communicate()
        except KeyboardInterrupt:
            print
            print ('Killed benchmark %s running command:\n%s.' %
                   (self.name, ' '.join(cmd)))
            proc.kill()
            raise
        with open('/scratch/rnk/dynamorio/spec06/speclog.log', 'a') as speclog:
            speclog.write(output)

        # Scrape output for csv file.
        match = re.search(r'format: CSV -> *([^ ].*\.csv)', output)
        assert match, "Unable to find SPEC csv file."
        csv_file = match.group(1)
        print 'CSV results file:', csv_file
        self.csv_file = csv_file
        return csv_file

    def parse_csv_file(self):
        times = parse_spec_csv(self.csv_file)
        self.times = times
        return times

    def clone_with(self, name, **kwargs):
        clone = Benchmark(name, self.config, self.bms)
        clone.__dict__.update(self.__dict__)
        clone.__dict__.update(kwargs)
        clone.name = name
        return clone


# Configs:
# native
# noclient
# inscount_opt3
# inscount_opt3_tls
# inscount_opt0
# inscount_opt0_bb
# alignment_opt3
# alignment_opt0
# memtrace_opt3
# memtrace_opt0

CLIENT_BASE = os.path.join(DR_BUILD, 'ext/lib64/release', 'libdrcalls_bench.')

BENCHMARKS = ["int", "^perl", '^xalancbmk', '^libquantum', '^omnetpp', "^gcc"]
#BENCHMARKS = ["bzip2"]

native   = Benchmark('native',   'linux64-amd64-gcc', bms=BENCHMARKS)
noclient = Benchmark('noclient', 'dr-noclient', bms=BENCHMARKS)

# This is verbose, I fail at Python right now.  Cleanup later.

inscount_base = noclient.clone_with('inscount_base',
                                    config='dr-client',
                                    dr_client=(CLIENT_BASE +
                                               "inscount_cleancall.so"))
inscount_opt0     = inscount_base.clone_with('inscount_opt0', dr_client_opts="0")
inscount_opt3     = inscount_base.clone_with('inscount_opt3', dr_client_opts="3")
inscount_opt3_tls = inscount_base.clone_with('inscount_opt3_tls',
                                             dr_client_opts="3 use_tls")
inscount_opt0_bb  = inscount_opt0.clone_with('inscount_opt0_bb',
                                             dr_client=(CLIENT_BASE +
                                                        "inscount_bbcall.so"))
inscount_manual = inscount_base.clone_with('inscount_manual',
                                           dr_client=(CLIENT_BASE +
                                                      "inscount_manual.so"))

alignment_opt3 = noclient.clone_with('alignment_opt3',
                                     config='dr-client',
                                     dr_client=(CLIENT_BASE + "alignment.so"),
                                     dr_client_opts="print_count opt_calls 3")
alignment_opt0 = alignment_opt3.clone_with('alignment_opt0',
                                       dr_client_opts="print_count opt_calls 0")

memtrace_opt3 = alignment_opt3.clone_with('memtrace_opt3',
                              dr_client_opts="print_count use_buffer opt_calls 3")
memtrace_opt0 = alignment_opt3.clone_with('memtrace_opt0',
                              dr_client_opts="print_count use_buffer opt_calls 0")

bms_to_run = [
        native,
        noclient,
        inscount_opt0,
        inscount_opt3,
        inscount_opt3_tls,
        inscount_opt0_bb,
        inscount_manual,
        alignment_opt3,
        alignment_opt0,
        memtrace_opt3,
        memtrace_opt0,
        ]

results_filename = '/scratch/rnk/dynamorio/spec06/spec_results.txt'
#results_filename = 'spec_results.txt'

def pretty_dict_repr(d):
    lines = ['    %r: %r,\n' % (k, v) for (k, v) in d.iteritems()]
    lines.sort()
    return '{\n%s}\n' % ''.join(lines)


def main(argv):
    csv_dict = {}
    if os.path.exists(results_filename):
        try:
            csv_dict.update(eval(open(results_filename).read()))
        except:
            print "Parsing the results file failed, aborting."
            sys.exit(1)
    if not os.path.exists(results_filename) or '--rerun' in argv:
        for bm in bms_to_run:
            bm.runspec()
        csv_dict[bm.name] = bm.csv_file
        # Derp, this is not atomic, silly.
        with open(results_filename, 'w') as outfile:
            outfile.write(pretty_dict_repr(csv_dict))

    #for bm in bms_to_run:
        #print bm.name
        #print ' '.join(bm.build_command()[0])

    bms_dict = dict((bm.name, bm) for bm in bms_to_run)
    for (bm_name, bm) in bms_dict.items():
        if bm_name not in csv_dict:
            print "Missing CSV file for %s, do you need to --rerun?" % bm_name
        bm.csv_file = csv_dict[bm_name]
        bm.parse_csv_file()

    #for bm in bms_to_run:
        #print bm.name, bm.times

    # Silence benchmarks that are too short when run natively to be relevant.
    # We'll fix this with another run on larger inputs later.
    bad_bms = ['xalancbmk', 'libquantum', 'omnetpp', 'gcc']
    for bm in bms_dict.values():
        bm.times = [(bm_name, time) for (bm_name, time) in bm.times
                    if bm_name not in bad_bms]

    # Graph printing, 3 graphs, 3 tools, inscount, alignment, memtrace.
    # bms sorted from expected worst to best perf.
    # Should I have a graph with noclient?
    def bms_to_tuples(bms):
        return [(bm.name, bm.times) for bm in bms]
    inscount_times = bms_to_tuples([
            bms_dict['inscount_opt0'],
            bms_dict['inscount_opt0_bb'],
            bms_dict['inscount_opt3'],
            bms_dict['inscount_opt3_tls'],
            bms_dict['inscount_manual'],
            ])
    alignment_times = bms_to_tuples([
            bms_dict['alignment_opt0'],
            bms_dict['alignment_opt3'],
            ])
    memtrace_times = bms_to_tuples([
            bms_dict['memtrace_opt0'],
            bms_dict['memtrace_opt3'],
            ])

    try:
        os.mkdir("perf_charts")
    except:
        pass
    if not os.path.isdir("perf_charts"):
        print "Unable to make perf_charts dir!"
        sys.exit(1)

    # Instruction count
    inscount_norm = normalize_all(native.times, inscount_times)
    inscount_src = build_cluster_graph(inscount_norm)
    generate_graph('inscount_all', inscount_src)
    inscount_opts = inscount_norm[2:]  # opt0, opt0_bb
    inscount_src = build_cluster_graph(inscount_opts, colors=COLORS[2:])
    generate_graph('inscount_no0', inscount_src)
    opt3_tls_slow = avg_bm_times(inscount_opts[1][1])
    manual_slow   = avg_bm_times(inscount_opts[2][1])
    print "inscount_opt3_tls avg slowdown from native:", opt3_tls_slow
    print "inscount_manual   avg slowdown from native:", manual_slow

    # Speedup.
    inscount_noopt = bms_to_tuples([inscount_opt0, inscount_opt0_bb])
    # Use opt3_tls as base to get speedup.
    inscount_speed = normalize_all(inscount_opt3_tls.times, inscount_noopt)
    inscount_src = build_cluster_graph(inscount_speed, colors=COLORS[0:2],
                                       ylabel="Times Speedup with opt3 and TLS")
    generate_graph('inscount_speedup', inscount_src)
    opt0_speedup    = avg_bm_times(inscount_speed[0][1])
    opt0_bb_speedup = avg_bm_times(inscount_speed[1][1])
    print "inscount_opt3_tls avg speedup over inscount_opt0   :", opt0_speedup
    print "inscount_opt3_tls avg speedup over inscount_opt0_bb:", opt0_bb_speedup

    # Alignment
    alignment_norm = normalize_all(native.times, alignment_times)
    alignment_src = build_cluster_graph(alignment_norm, colors=COLORS[3:])
    generate_graph('alignment_slowdown', alignment_src)
    opt0_slow = avg_bm_times(alignment_norm[0][1])
    opt3_slow = avg_bm_times(alignment_norm[1][1])
    print "alignment_opt0 avg slowdown from native:", opt0_slow
    print "alignment_opt3 avg slowdown from native:", opt3_slow

    alignment_noopt = bms_to_tuples([alignment_opt0])
    alignment_speed = normalize_all(alignment_opt3.times, alignment_noopt)
    alignment_src = build_cluster_graph(alignment_speed, colors=COLORS[3:4])
    generate_graph('alignment_speedup', alignment_src)
    speedup = avg_bm_times(alignment_speed[0][1])
    print "alignment_opt3 avg speedup over alignment_opt0:", speedup

    # Alignment
    memtrace_norm = normalize_all(native.times, memtrace_times)
    memtrace_src = build_cluster_graph(memtrace_norm, colors=COLORS[3:])
    generate_graph('memtrace_slowdown', memtrace_src)
    opt0_slow = avg_bm_times(memtrace_norm[0][1])
    opt3_slow = avg_bm_times(memtrace_norm[1][1])
    print "memtrace_opt0 avg slowdown from native:", opt0_slow
    print "memtrace_opt3 avg slowdown from native:", opt3_slow

    memtrace_noopt = bms_to_tuples([memtrace_opt0])
    memtrace_speed = normalize_all(memtrace_opt3.times, memtrace_noopt)
    memtrace_src = build_cluster_graph(memtrace_speed, colors=COLORS[3:4])
    generate_graph('memtrace_speedup', memtrace_src)
    speedup = avg_bm_times(memtrace_speed[0][1])
    print "memtrace_opt3 avg speedup over memtrace_opt0:", speedup


CLUSTER_TEMPLATE = """\
# clustered graph example from Derek Bruening's CGO 2005 talk
=cluster;%(legend)s
# green instead of gray since not planning on printing this
colors=%(colors)s
=table
yformat=%%gx
max=%(y_max_value)s
min=1
=norotate
ylabel=%(ylabel)s
xlabel=SPEC 2006 Benchmark
# stretch it out in x direction
extraops=set size 1.2,1

%(graph_times)s
"""

COLORS = ['black', 'yellow', 'red', 'med_blue', 'light_green']

def build_cluster_graph(config_times, colors=COLORS,
                        ylabel="Times Slowdown from Native"):
    # Derp, naming is terrible.
    config_names = [config for (config, times) in config_times]
    (_, first_config_times) = config_times[0]
    transposed_times = [[bm_name] for (bm_name, _) in first_config_times]
    for (config_name, bm_times) in config_times:
        for (i, (bm_name, time)) in enumerate(bm_times):
            assert transposed_times[i][0] == bm_name
            transposed_times[i].append(str(time))

    # Variables for bar graph file template.
    colors = ','.join(colors)
    y_max_value = max(time for (_, bm_times) in config_times
                      for (_, time) in bm_times)
    legend = ';'.join(config_names)
    graph_times = '\n'.join(' '.join(elts) for elts in transposed_times)

    return CLUSTER_TEMPLATE % locals()


def generate_graph(graph_name, graph_source):
    graph_file = 'perf_charts/' + graph_name + '.perf'
    pdf_file   = 'perf_charts/' + graph_name + '.pdf'
    with open(graph_file, 'w') as output:
        output.write(graph_source)
    with open(pdf_file, 'w') as pdf_fd:
        proc = subprocess.Popen(['./bargraphgen/bargraph.pl', '-pdf',
                                 graph_file], stdout=pdf_fd)
        (_, err) = proc.communicate()
        if proc.returncode or err:
            print 'bargraph.pl error:'
            print err


if __name__ == '__main__':
    main(sys.argv)
