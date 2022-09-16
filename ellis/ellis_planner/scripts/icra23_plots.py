import argparse
import yaml
import os
import scipy.stats as sps
import numpy as np

# METHODS = ["ours", "replanning", "cmax"]
METHODS = ["ours", "cmax"]
# METHODS = ["ours"]


def parse_data(output_dir):
    print("Parsing data from {}...".format(output_dir))

    d = {}

    for filename in os.listdir(output_dir):
        f = os.path.join(output_dir, filename)
        if os.path.isfile(f):
            name, ext = os.path.splitext(filename)
            if ext == '.yaml':
                print("Processing {}...".format(f))
                exp, i, method = name.split('_')
                print("  i: {}, method: {}".format(i, method))
                if i not in d:
                    d[i] = {}

                d[i][method] = yaml.load(open(f, 'r'))

    print("Loaded data from {} experiments".format(len(d)))
    method_to_exec_times = {}
    method_to_dist = {}
    method_to_planning_times = {}
    for method in METHODS:
        method_to_exec_times[method] = []
        method_to_dist[method] = []
        method_to_planning_times[method] = []

    exec_time_diffs = []
    dist_diffs = []

    # tmp = []
    # for i, v in d.iteritems():
    #     tmp.append((v['ours']['execution_time_sec'], int(i)))
    #     # print("  {}: {}".format(i, v['ours']['execution_time_sec']))
    # tmp.sort()
    # for t in tmp:
    #     print(t)

    num_failed = 0
    for i, v in d.iteritems():
        all_reached_goal = True
        for method in METHODS:
            if method not in v or not v[method]['reached_goal']:
                all_reached_goal = False
                break

        if not all_reached_goal:
            num_failed += 1
            continue

        # exec_time_diffs.append(
        #     v['ours']['execution_time_sec'] - v['cmax']['execution_time_sec'])
        # dist_diffs.append(
        #     v['ours']['total_distance'] - v['cmax']['total_distance'])

        for method in METHODS:
            if method in v:
                method_to_exec_times[method].append(
                    v[method]['execution_time_sec'])
                method_to_dist[method].append(
                    v[method]['total_distance'])

                for p in v[method]['profiling']:
                    method_to_planning_times[method].append(
                        p['search']['planning_time_sec'])

    # print(method_to_exec_times)
    print(method_to_planning_times)
    print("num failed: {}".format(num_failed))
    print(len(method_to_exec_times[METHODS[0]]))
    for method in METHODS:
        if len(method_to_exec_times[method]) > 0:
            print("{} mean time (sec): {} ({})".format(
                method, np.mean(method_to_exec_times[method]), sps.sem(method_to_exec_times[method])))
            print("{} mean dist (m): {} ({})".format(method, np.mean(method_to_dist[method]), sps.sem(method_to_dist[method])))

    # print("exec time diff: {} ({})".format(np.mean(exec_time_diffs), sps.sem(exec_time_diffs)))
    # print("dist diff: {} ({})".format(np.mean(dist_diffs), sps.sem(dist_diffs)))



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output_dir')
    args = parser.parse_args()

    parse_data(args.output_dir)
