import matplotlib.pyplot as plt
import numpy as np
import re
import argparse

def create_parser():
    parser = argparse.ArgumentParser(description='Realtime Logparser Configuration')
    parser.add_argument('-f', '--filename', type=str, dest='filename', default='logfile.txt', help='Log file name')
    parser.add_argument('-d', '--desired', type=int, dest='desired', default=100000000, help='Desired Looptime')
    parser.add_argument('-a', '--acceptable', type=int, dest='acceptable', default=110000000, help='Acceptable Looptime')

    return parser

def read(logfile):
    X = []
    Y = []
    with open(logfile) as f:
        line = f.readline()
        while line:
            # process line
            if line.startswith('Iteration:'):
                m = re.search('Iteration: (\d+)', line, re.IGNORECASE)
                #print(m.group(1))
                X.append(float(m.group(1)))
                n = re.search('(\d+) nsecs', line, re.IGNORECASE)
                #print(n.group(1))
                Y.append(float(n.group(1)))
            line = f.readline()
    return X,Y

def plot_bar(args, X, Y):
    fig, ax = plt.subplots()
    ymin = np.min(Y)
    print("Min:", ymin)
    ymax = np.max(Y)
    print("Max:", ymax)
    ymean = np.mean(Y)
    print("Mean:", ymean)
    ymedian = np.median(Y)
    print("Median:", ymedian)
    ydiff = np.diff(Y)
    jitter = np.mean(np.absolute(ydiff))
    print("Jitter:", jitter)
    textstr = '\n'.join((
    r'$Min=%.0f$' % (ymin, ),
    r'$Max=%.0f$' % (ymax, ),
    r'$Median=%.0f$' % (ymedian, ),
    r'$Jitter=%.0f$' % (jitter, )))
    props = props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    ax.text(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=14,
        verticalalignment='top', bbox=props)
    #plt.bar(X, Y, align='edge', width=0.3)
    #plt.bar(X, Y)
    #plt.plot(X,Y)
    plt.scatter(X,Y)
    #plt.plot(10,1500000,'r*')
    plt.title('ROS2.0 Communication Latency', fontsize=20, fontweight='bold')
    plt.xlabel('Iteration', fontsize=15)
    plt.ylabel('Round Trip Time (nsec)', fontsize=15)
    #plt.hlines(args.desired, 0, len(X), colors="g", linestyle="dashed")
    #plt.text(len(X), args.desired, ' Desired', ha='left', va='center', color="green")
    #plt.hlines(args.acceptable, 0, len(X), colors="r", linestyle="dashed")
    #plt.text(len(X), args.acceptable, ' Acceptable', ha='left', va='center', color="red")
    #plt.hlines(ymin, 0, len(X), colors="g", linestyle="dashed")
    #plt.text(len(X), args.desired, ' Minimum', ha='left', va='center', color="green")
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
