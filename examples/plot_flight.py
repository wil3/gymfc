import argparse
import numpy as np
import matplotlib.pyplot as plt

from gymfc.tools.plot import *

if __name__ == '__main__':
    parser = argparse.ArgumentParser("Plot recorded flight data.")
    parser.add_argument("log_file", help="Log file.")
    parser.add_argument("--title", help="Title for the plot.",
                        default="Aircraft Response")
    args = parser.parse_args()

    fdata = np.loadtxt(args.log_file, delimiter=",")

    # Plot the response
    f, ax = plt.subplots(5, sharex=True, sharey=False)
    plt.suptitle(args.title)
    plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
    t = fdata[:, 0]
    pqr = fdata[:, 11:14]
    pqr_sp = fdata[:, 14:17]
    plot_rates(ax[:3], t, pqr_sp, pqr)

    us = fdata[:, 17:21]
    plot_u(ax[3], t, us)

    rpms = fdata[:, 21:25]
    plot_motor_rpms(ax[4], t, rpms)

    ax[-1].set_xlabel("Time (s)")
    plt.show()

