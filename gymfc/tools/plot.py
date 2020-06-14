"""Helper functions for plotting""" 

import matplotlib.pyplot as plt
import math
import logging
import numpy as np

def plot_rates(ax, t, pqr_desired, pqr_actual):
    """Plot angular rates where each axis is placed in its own Matplotlib
    subplot.

    Args:
        ax: Matplotlib axis the plot will be applied to
        t: Numpy array of times
        pqr_desired: 2D numpy array of desired angular rates for each roll, 
            pitch and yaw axis.
        pqr_actual: 2D numpy array of the actual angular rates for each roll, 
            pitch and yaw axis.
    """


    unit_rotation = " (deg/s)"
    axis_label = ["p", "q", "r"]
    for i in range(len(ax)):
        ax[i].plot(t, pqr_desired[:,i], '--', c='k', label="Desired")
        ax[i].plot(t, pqr_actual[:,i], '-', c='b', label="Actual")
        #ax[i].axhline(0, color="#AAAAAA")
        ax[i].legend(loc="right")
        ax[i].grid(True)
        ax[i].set_ylabel(axis_label[i] + unit_rotation)


def plot_motor_rpms(ax, t, rpms, alpha=1.0):
    """ Plot the motor rpms values

    Args:

        ax: Matplotlib axis the plot will be applied to
        t: Numpy array of motor time 
        rpms: 2D Numpy array where the length is equal to len(t) and the width
        corresponds to the number of actuators. 
    """
    ax.set_ylabel("RPM")

    lines = ["-", "-.", ":", "--"]
    for i in range(4):
        ax.plot(t, rpms[:,i], label="M{}".format(i+1), linestyle=lines[i], 
                alpha=alpha)

    ax.legend(loc='upper right', ncol=4)
    ax.grid(True)


def plot_u(ax, t, u, alpha=1.0):
    """ Plot motor control signals """
    ax.set_ylabel("u (\%)")
    lines = ["-", "-.", ":", "--"]
    for i in range(4):
        ax.plot(t, u[:,i], label="M{}".format(i+1), linestyle=lines[i], 
                alpha=alpha)

    ax.legend(loc='upper right', ncol=4)
    ax.grid(True)


