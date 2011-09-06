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

CONFIG_ORDER = ["pin", "DRinline", "DRmanual"]


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
    native_times = results['native']
    del results['native']
    for times in results.values():
        both_times = zip(times, native_times)
        for (i, ((bm, time), (nat_bm, nat_time))) in enumerate(both_times):
            assert bm == nat_bm
            norm_time = time / nat_time
            times[i] = (bm, norm_time)

    transposed_times = [[bm_name] for (bm_name, _) in native_times]
    for (_, times) in sorted(results.items(),
                             key=lambda (k, v): CONFIG_ORDER.index(k)):
        for (i, (bm_name, time)) in enumerate(times):
            assert transposed_times[i][0] == bm_name
            transposed_times[i].append(str(time))

    # Variables for bar graph file template.
    y_max_value = max(time for times in results.values()
                      for (_, time) in times)
    legend = ' '.join('"%s"' % config for config in
                      sorted(results.keys(), key=CONFIG_ORDER.index))
    graph_times = '\n'.join(' '.join(elts) for elts in transposed_times)

    print GRAPH_TEMPLATE % locals(),


PIN_DR_RESULTS = [
        'native', 'result/CINT2006.012.test.csv',
        'pin', 'result/CINT2006.013.test.csv',
        'DRinline', 'result/CINT2006.014.test.csv',
        'DRmanual', 'result/CINT2006.015.test.csv',
        ]


if __name__ == '__main__':
    #main(sys.argv[1:])
    main(PIN_DR_RESULTS)
