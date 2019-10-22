import matplotlib.pyplot as plt
import numpy as np
import re
import argparse

metric = ""
expected_ns = 0
jitter_ns = 0
def create_parser():
    parser = argparse.ArgumentParser(description='Realtime Logparser Configuration')
    parser.add_argument('-f', '--filename', type=str, dest='filename', default='logfile.txt', help='Log file name')
    parser.add_argument('-d', '--desired', type=int, dest='desired', default=100000000, help='Desired Looptime')
    parser.add_argument('-a', '--acceptable', type=int, dest='acceptable', default=110000000, help='Acceptable Looptime')

    return parser

def read(logfile):
    X = []
    Y = []
    global metric
    global expected_ns
    global jitter_ns
    with open(logfile) as f:
        line = f.readline()
        while line:
            # process line
            if line.startswith('RTMonitor Performance '):
                metric_sre = re.search('Metric: (\w+)', line, re.IGNORECASE)
                metric = metric_sre.group(1)
            elif line.startswith('Expected Metric'):
                expected_ns_sre = re.search('value: (\d+)', line, re.IGNORECASE)
                expected_ns = int(expected_ns_sre.group(1))
            elif line.startswith('Acceptable Jitter'):
                jitter_ns_sre = re.search('value: (\d+)', line, re.IGNORECASE)
                jitter_ns = int(jitter_ns_sre.group(1))
            elif line.startswith('Iteration:'):
                m = re.search('Iteration: (\d+)', line, re.IGNORECASE)
                #print(m.group(1))
                X.append(float(m.group(1)))
                n = re.search('(\d+) nsecs', line, re.IGNORECASE)
                #print(n.group(1))
                Y.append(float(n.group(1)))
            line = f.readline()
    return X,Y

def plot_bar(args, X, Y):
    global metric
    global expected_ns
    global jitter_ns
    plt.bar(X, Y, align='edge', width=0.3)
    # plt.bar(X, Y)
    plt.title('RTMonitor Metric: '+ metric, fontsize=20, fontweight='bold')
    plt.xlabel('Iteration Count', fontsize=15)
    plt.ylabel('Duration (nsec)', fontsize=15)
    if expected_ns > 0:
        plt.hlines(expected_ns, 0, len(X), colors="g", linestyle="dashed")
        plt.text(len(X), expected_ns, ' Desired', ha='left', va='center', color="green")
        plt.hlines(expected_ns+jitter_ns, 0, len(X), colors="r", linestyle="dashed")
        plt.text(len(X), expected_ns+jitter_ns, ' Acceptable', ha='left', va='center', color="red")
        plt.hlines(expected_ns-jitter_ns, 0, len(X), colors="r", linestyle="dashed")
        plt.text(len(X), expected_ns-jitter_ns, ' Acceptable', ha='left', va='center', color="red")
    plt.show()
    return

def main():
    args = create_parser().parse_args()
    # print(args.filename)
    # print(args.desired)
    # print(args.acceptable)
    X,Y = read(args.filename)
    plot_bar(args, X, Y)

if __name__ == "__main__":
    main()
