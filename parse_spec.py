#!/usr/bin/env python

from __future__ import with_statement

import re
import sys


GRAPH_TEMPLATE = """\
type=clustered
size=600x400
title=Inscount Performance Comparison
x_label=Benchmark
y_label=Normalized Execution
legend=%(legend)s
y_max_value=%(y_max_value)s
y_tick_number=14
y_number_format=%%3.1f

%(graph_times)s
"""

CONFIG_ORDER = ["noclient", "dstack", "tls"]


def normalize_results(results, base_times):
    for times in results.values():
        both_times = zip(times, base_times)
        for (i, ((bm, time), (base_bm, base_time))) in enumerate(both_times):
            assert bm == base_bm
            norm_time = time / base_time
            times[i] = (bm, norm_time)


def avg(xs):
    xs = list(xs)
    return sum(xs) / float(len(xs))


def main(argv):
    argv = iter(argv)
    pairs = []
    for arg in argv:
        name = arg
        csv_path = argv.next()
        pairs.append((name, csv_path))

    results = {}
    for (name, csv_path) in pairs:
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
        results[name] = times

    # Normalize to native.
    #native_times = results['native']
    #del results['native']
    #normalize_results(results, native_times)
    dstack_times = results['dstack']
    del results['dstack']
    normalize_results(results, dstack_times)

    for config in ('tls', 'dstack_sahf'):
        print config
        for (bm, time) in results[config]:
            print bm, time

        val = avg(t for (_, t) in results[config])
        print 'average:', val

        percent = (1 - val) * 100
        print '%s is %2.1f%% faster than dstack' % (config, percent)
        print


DSTACK_TLS_RESULTS = [
        'native',      'result/CINT2006.082.test.csv',
        'noclient',    'result/CINT2006.083.test.csv',
        'dstack',      'result/CINT2006.084.test.csv',
        'dstack_sahf', 'result/CINT2006.088.test.csv',
        'tls',         'result/CINT2006.085.test.csv',
        ]


if __name__ == '__main__':
    #main(sys.argv[1:])
    main(DSTACK_TLS_RESULTS)
